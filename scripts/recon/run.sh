#!/usr/bin/env bash
# Real-to-sim background: phone video or photos -> COLMAP -> 3DGRUT (3DGUT, MCMC) -> NuRec USDZ for Isaac Sim 5.1.
#
#   scripts/recon/run.sh <video file | image folder> <scene name> [stage...]
#   scripts/recon/run.sh ~/capture/room.mp4 workspace            # all stages
#   scripts/recon/run.sh ~/capture/room.mp4 workspace train export  # rerun only some stages
#
# Stages: frames colmap train export (default: all, in that order).
# Outputs (all git-ignored):
#   data/recon/<scene>/images/          frames resized to MAX_DIM (COLMAP and 3DGRUT both read these)
#   data/recon/<scene>/sparse/0/        COLMAP model (largest reconstructed component)
#   outputs/3dgrut/<scene>/             3DGRUT run dir (checkpoints, logs)
#   assets/scene/<scene>.usdz           NuRec USDZ (Isaac Sim 5.0-6.0 loads this; ParticleField needs 6.0)
#
# Envs: conda "colmap" (setup/05_colmap_env.sh) and conda "3dgrut" (setup/06_3dgrut_env.sh).
# Tunables (env vars):
#   FPS=2            frames per second sampled from a video
#   MAX_DIM=1600     longest image side in pixels (VRAM and time scale with this)
#   MATCHER=auto     sequential for video, exhaustive for photos; or set explicitly
#   CAMERA_MODEL=OPENCV   single shared camera; 3DGUT trains on distorted images directly
#   ITERS=30000      3DGRUT iterations
#   MAX_GAUSSIANS=1000000  MCMC cap; 1M at 1600 px fits in 16 GB
#   CONFIG=apps/colmap_3dgut_mcmc.yaml
#   EXTRA_TRAIN_ARGS="..."  extra hydra overrides passed to train.py
set -eo pipefail

if [ $# -lt 2 ]; then
  sed -n '2,25p' "$0"
  exit 1
fi

# ~/.bashrc sources ROS 2 Jazzy (Python 3.12); keep it out of these Python envs.
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
      ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
INPUT="$(realpath "$1")"
SCENE="$2"
shift 2
STAGES="${*:-frames colmap train export}"

FPS="${FPS:-2}"
MAX_DIM="${MAX_DIM:-1600}"
MATCHER="${MATCHER:-auto}"
CAMERA_MODEL="${CAMERA_MODEL:-OPENCV}"
ITERS="${ITERS:-30000}"
MAX_GAUSSIANS="${MAX_GAUSSIANS:-1000000}"
CONFIG="${CONFIG:-apps/colmap_3dgut_mcmc.yaml}"

SCENE_DIR="$ROOT/data/recon/$SCENE"
RUN_DIR="$ROOT/outputs/3dgrut/$SCENE"
USDZ="$ROOT/assets/scene/$SCENE.usdz"
THREEDGRUT="$ROOT/third_party/3dgrut"
mkdir -p "$SCENE_DIR" "$RUN_DIR" "$(dirname "$USDZ")"

source "$HOME/miniforge3/etc/profile.d/conda.sh"
has_stage() { [[ " $STAGES " == *" $1 "* ]]; }
log() { echo "[recon $(date +%H:%M:%S)] $*"; }

if [ -f "$INPUT" ]; then IS_VIDEO=1; else IS_VIDEO=0; fi
if [ "$MATCHER" = auto ]; then
  if [ "$IS_VIDEO" = 1 ]; then MATCHER=sequential; else MATCHER=exhaustive; fi
fi

# ---- 1. frames: video -> JPEG frames, or copy/resize photos (longest side <= MAX_DIM) ----
if has_stage frames; then
  conda activate colmap
  rm -rf "$SCENE_DIR/images" && mkdir -p "$SCENE_DIR/images"
  SCALE="scale='if(gt(iw,ih),min(iw,$MAX_DIM),-2)':'if(gt(iw,ih),-2,min(ih,$MAX_DIM))'"
  if [ "$IS_VIDEO" = 1 ]; then
    log "extracting frames from $INPUT at ${FPS} fps"
    ffmpeg -hide_banner -loglevel error -i "$INPUT" -vf "fps=$FPS,$SCALE" -qscale:v 2 \
      "$SCENE_DIR/images/frame_%05d.jpg"
  else
    log "resizing photos from $INPUT"
    find -L "$INPUT" -maxdepth 1 -type f -iregex '.*\.\(jpg\|jpeg\|png\)' | sort | while read -r f; do
      ffmpeg -hide_banner -loglevel error -nostdin -i "$f" -vf "$SCALE" -qscale:v 2 \
        "$SCENE_DIR/images/$(basename "${f%.*}").jpg"
    done
  fi
  log "$(ls "$SCENE_DIR/images" | wc -l) images in $SCENE_DIR/images"
  conda deactivate
fi

# ---- 2. colmap: features -> matching -> incremental mapping; keep the largest model as sparse/0 ----
if has_stage colmap; then
  conda activate colmap
  DB="$SCENE_DIR/database.db"
  rm -rf "$DB" "$SCENE_DIR/sparse" "$SCENE_DIR/sparse_all" && mkdir -p "$SCENE_DIR/sparse_all"
  log "feature_extractor ($CAMERA_MODEL, single camera, GPU)"
  colmap feature_extractor --database_path "$DB" --image_path "$SCENE_DIR/images" \
    --ImageReader.camera_model "$CAMERA_MODEL" --ImageReader.single_camera 1 \
    --FeatureExtraction.use_gpu 1
  log "${MATCHER}_matcher"
  if [ "$MATCHER" = sequential ]; then
    colmap sequential_matcher --database_path "$DB" --FeatureMatching.use_gpu 1 \
      --SequentialMatching.overlap 15
  else
    colmap exhaustive_matcher --database_path "$DB" --FeatureMatching.use_gpu 1
  fi
  log "mapper"
  colmap mapper --database_path "$DB" --image_path "$SCENE_DIR/images" --output_path "$SCENE_DIR/sparse_all"
  # Pick the component with the most registered images.
  BEST=""; BEST_N=0
  for m in "$SCENE_DIR"/sparse_all/*/; do
    n=$(colmap model_analyzer --path "$m" 2>&1 | sed -n 's/.*Registered images: *\([0-9]*\).*/\1/p' | head -1)
    log "model $m: ${n:-0} registered images"
    if [ "${n:-0}" -gt "$BEST_N" ]; then BEST="$m"; BEST_N="$n"; fi
  done
  [ -n "$BEST" ] || { echo "ERROR: COLMAP reconstructed no model" >&2; exit 1; }
  mkdir -p "$SCENE_DIR/sparse" && cp -r "$BEST" "$SCENE_DIR/sparse/0"
  log "using $BEST ($BEST_N / $(ls "$SCENE_DIR/images" | wc -l) images) as sparse/0"
  conda deactivate
fi

# ---- 3. train: 3DGUT with the MCMC strategy (bounded Gaussian count keeps VRAM predictable) ----
if has_stage train; then
  set +u
  conda activate 3dgrut
  cd "$THREEDGRUT"
  log "training $CONFIG for $ITERS iterations"
  # shellcheck disable=SC2086
  python train.py --config-name "$CONFIG" \
    path="$SCENE_DIR" out_dir="$RUN_DIR" experiment_name="$SCENE" \
    n_iterations="$ITERS" strategy.add.max_n_gaussians="$MAX_GAUSSIANS" \
    dataset.downsample_factor=1 num_workers=4 \
    export_usd.enabled=true export_usd.format=nurec export_usd.path="$RUN_DIR/${SCENE}_nurec.usdz" \
    $EXTRA_TRAIN_ARGS
  conda deactivate
fi

# ---- 4. export: copy the NuRec USDZ into assets/scene/ (re-export from the checkpoint if needed) ----
if has_stage export; then
  if [ ! -f "$RUN_DIR/${SCENE}_nurec.usdz" ]; then
    CKPT="$(ls -t "$RUN_DIR"/*/*/ckpt_last.pt 2>/dev/null | head -1)"
    [ -n "$CKPT" ] || { echo "ERROR: no checkpoint under $RUN_DIR" >&2; exit 1; }
    set +u
    conda activate 3dgrut
    cd "$THREEDGRUT"
    log "exporting $CKPT"
    python -m threedgrut.export.scripts.export_usd --checkpoint "$CKPT" \
      --output "$RUN_DIR/${SCENE}_nurec.usdz" --dataset "$SCENE_DIR" --format nurec
    conda deactivate
  fi
  cp "$RUN_DIR/${SCENE}_nurec.usdz" "$USDZ"
  log "wrote $USDZ ($(du -h "$USDZ" | cut -f1))"
fi
