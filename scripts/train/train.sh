#!/usr/bin/env bash
# Train a LeRobot policy on a dataset written by so101_pai.dataset.bag2lerobot.
#
#   scripts/train/train.sh act local/pai_sim_v5 data/datasets/pai_sim_v5         # ACT from scratch (the reported runs)
#   INIT=<checkpoint> scripts/train/train.sh act local/pai_real_v1 data/datasets/pai_real_v1   # fine-tune it
#   scripts/train/train.sh smolvla local/pai_sim_v5 data/datasets/pai_sim_v5     # SmolVLA from smolvla_base
#
# Env overrides: STEPS, BATCH, OUT, SAVE_FREQ, INIT (smolvla: fine-tune this checkpoint instead of smolvla_base;
# act: fine-tune it instead of starting from scratch), PYTHON (default .venv/bin/python),
# EXTRA (more lerobot-train flags; the reported runs used
# EXTRA="--dataset.image_transforms.enable=true --policy.use_amp=true"). docs/REPRODUCE.md has the exact commands.
# Checkpoints land in $OUT/checkpoints/<step>/pretrained_model; policy_node loads that directory.
set -eo pipefail

POLICY=${1:?policy: act | smolvla}
REPO_ID=${2:?dataset repo id, e.g. local/pai_sim_v5}
DATA_ROOT=${3:-}
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON=${PYTHON:-$ROOT/.venv/bin/python}

case "$POLICY" in
  smolvla)
    # Fine-tune the pretrained SmolVLA base (trained on community SO-100/101 data, same motor units).
    # Its config names the cameras camera1..3, so map ours onto them.
    POLICY_ARGS=(--policy.path="${INIT:-lerobot/smolvla_base}"
      --rename_map='{"observation.images.front": "observation.images.camera1", "observation.images.wrist": "observation.images.camera2"}')
    STEPS=${STEPS:-20000}; BATCH=${BATCH:-16} ;;
  act)
    # From scratch, or (INIT=<checkpoint>) fine-tune e.g. the sim-trained ACT on real data; either way the
    # normalization stats come from this dataset (lerobot_train overrides the checkpoint's).
    if [ -n "${INIT:-}" ]; then POLICY_ARGS=(--policy.path="$INIT"); else POLICY_ARGS=(--policy.type=act); fi
    STEPS=${STEPS:-50000}; BATCH=${BATCH:-8} ;;
  *) echo "unknown policy: $POLICY" >&2; exit 1 ;;
esac

OUT=${OUT:-$ROOT/outputs/train/${POLICY}_$(basename "$REPO_ID")_$(date +%m%d_%H%M)}
DATASET_ARGS=(--dataset.repo_id="$REPO_ID")
[ -n "$DATA_ROOT" ] && DATASET_ARGS+=(--dataset.root="$DATA_ROOT")

# The ROS env from ~/.bashrc is irrelevant to training; keep its Python 3.12 packages off the path.
unset PYTHONPATH

set -x
# shellcheck disable=SC2086  # EXTRA is intentionally word-split into flags
exec "$PYTHON" -m lerobot.scripts.lerobot_train \
  "${POLICY_ARGS[@]}" "${DATASET_ARGS[@]}" \
  --policy.device=cuda --policy.push_to_hub=false \
  --output_dir="$OUT" --job_name="$(basename "$OUT")" \
  --steps="$STEPS" --batch_size="$BATCH" \
  --save_freq="${SAVE_FREQ:-5000}" --log_freq=100 \
  --wandb.enable=false \
  $EXTRA
