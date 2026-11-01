#!/usr/bin/env bash
# Train a LeRobot policy on a dataset written by so101_pai.dataset.bag2lerobot.
#
#   scripts/train/train.sh smolvla local/pai_sim data/datasets/pai_sim     # SmolVLA, local (RTX 5060 Ti) - main
#   STREAMING=1 scripts/train/train.sh smolvla <hf_user>/pai_sim           # same, streamed from the HF Hub
#   PYTHON=.venv-train/bin/python STREAMING=1 scripts/train/train.sh groot <hf_user>/pai_sim   # GR00T N1.7 (B300)
#   scripts/train/train.sh act local/pai_sim data/datasets/pai_sim         # ACT baseline (small, from scratch)
#
# Env overrides: STEPS, BATCH, OUT, SAVE_FREQ, INIT (smolvla: fine-tune this checkpoint instead of smolvla_base; act: fine-tune it instead of starting from scratch), PYTHON (default .venv/bin/python; on the B300 the venv from setup/07),
# EXTRA (more lerobot-train flags, e.g. EXTRA="--wandb.enable=true").
# Checkpoints land in $OUT/checkpoints/<step>/pretrained_model; policy_node loads that directory.
set -eo pipefail

POLICY=${1:?policy: smolvla | groot | act}
REPO_ID=${2:?dataset repo id, e.g. local/pai_sim or <hf_user>/pai_sim}
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
  groot)
    # GR00T N1.7 3B, following LeRobot's SO-101 recipe (docs/source/groot.mdx): new_embodiment tag, 16-step chunks,
    # bf16. The VLM backbone stays frozen (LeRobot default); projector + action head are trained.
    # Absolute actions (use_relative_actions=false) keep inference on the same simple path as the other policies.
    POLICY_ARGS=(--policy.type=groot --policy.base_model_path=nvidia/GR00T-N1.7-3B
      --policy.embodiment_tag=new_embodiment --policy.chunk_size=16 --policy.n_action_steps=16
      --policy.use_relative_actions=false --policy.use_bf16=true --dataset.image_transforms.enable=true)
    STEPS=${STEPS:-20000}; BATCH=${BATCH:-64} ;;
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
[ "${STREAMING:-0}" = "1" ] && DATASET_ARGS+=(--dataset.streaming=true)

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
