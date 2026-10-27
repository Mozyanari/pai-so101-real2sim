#!/usr/bin/env bash
# Train a LeRobot policy on a dataset written by so101_pai.dataset.bag2lerobot.
#
#   scripts/train/train.sh act local/pai_sim data/datasets/pai_sim         # ACT, local dataset (RTX 5060 Ti)
#   STREAMING=1 scripts/train/train.sh act <hf_user>/pai_sim               # ACT, streamed from the HF Hub
#   scripts/train/train.sh smolvla <hf_user>/pai_sim                       # SmolVLA fine-tune (B300)
#
# Env overrides: STEPS, BATCH, OUT, PYTHON (default .venv/bin/python; on the B300 the venv from setup/07),
# EXTRA (more lerobot-train flags, e.g. EXTRA="--wandb.enable=true").
# Checkpoints land in $OUT/checkpoints/<step>/pretrained_model; policy_node loads that directory.
set -eo pipefail

POLICY=${1:?policy: act | smolvla}
REPO_ID=${2:?dataset repo id, e.g. local/pai_sim or <hf_user>/pai_sim}
DATA_ROOT=${3:-}
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON=${PYTHON:-$ROOT/.venv/bin/python}

case "$POLICY" in
  act)
    POLICY_ARGS=(--policy.type=act)
    STEPS=${STEPS:-50000}; BATCH=${BATCH:-8} ;;
  smolvla)
    # Fine-tune the pretrained SmolVLA base (trained on community SO-100/101 data, same motor units).
    # Its config names the cameras camera1..3, so map ours onto them.
    POLICY_ARGS=(--policy.path=lerobot/smolvla_base
      --rename_map='{"observation.images.front": "observation.images.camera1", "observation.images.wrist": "observation.images.camera2"}')
    STEPS=${STEPS:-20000}; BATCH=${BATCH:-64} ;;
  *) echo "unknown policy: $POLICY" >&2; exit 1 ;;
esac

OUT=${OUT:-$ROOT/outputs/train/${POLICY}_$(basename "$REPO_ID")_$(date +%m%d_%H%M)}
DATASET_ARGS=(--dataset.repo_id="$REPO_ID")
[ -n "$DATA_ROOT" ] && DATASET_ARGS+=(--dataset.root="$DATA_ROOT")
[ "${STREAMING:-0}" = "1" ] && DATASET_ARGS+=(--dataset.streaming=true)

# The ROS env from ~/.bashrc is irrelevant to training; keep its Python 3.12 packages off the path.
unset PYTHONPATH

set -x
exec "$PYTHON" -m lerobot.scripts.lerobot_train \
  "${POLICY_ARGS[@]}" "${DATASET_ARGS[@]}" \
  --policy.device=cuda --policy.push_to_hub=false \
  --output_dir="$OUT" --job_name="$(basename "$OUT")" \
  --steps="$STEPS" --batch_size="$BATCH" \
  --save_freq="${SAVE_FREQ:-5000}" --log_freq=100 \
  --wandb.enable=false \
  $EXTRA
