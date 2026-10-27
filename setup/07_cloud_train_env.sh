#!/usr/bin/env bash
# Training-only environment for the B300 cloud machine (no ROS, no Isaac Sim).
# Run it on the cloud machine from a clone of this repo:
#
#   git clone <this repo> pai && cd pai && setup/07_cloud_train_env.sh
#   hf auth login                                   # token with read access to the private dataset
#   PYTHON=.venv-train/bin/python STREAMING=1 scripts/train/train.sh groot <hf_user>/pai_sim
#
# B300 is Blackwell Ultra: it needs a CUDA 12.9+ PyTorch build. TORCH_INDEX picks the wheel index
# (default cu130). The script stops if torch cannot run a kernel on the GPU.
# LeRobot is pinned to the same commit used locally, so datasets and checkpoints are compatible.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$ROOT/.venv-train"
LEROBOT_DIR=${LEROBOT_DIR:-$HOME/lerobot}
LEROBOT_COMMIT=e40b58a8dfa9e7b86918c374791599d070518d11  # v0.6.0-3 ("v0.6.1" in PLAN.md), as on the local machine
TORCH_INDEX=${TORCH_INDEX:-https://download.pytorch.org/whl/cu130}

command -v uv >/dev/null || { echo "install uv first: https://docs.astral.sh/uv/" >&2; exit 1; }

if [ ! -d "$LEROBOT_DIR/.git" ]; then
  git clone https://github.com/huggingface/lerobot.git "$LEROBOT_DIR"
fi
git -C "$LEROBOT_DIR" fetch --quiet origin
git -C "$LEROBOT_DIR" checkout --quiet "$LEROBOT_COMMIT"

[ -d "$VENV" ] || uv venv --python 3.12 "$VENV"
# torch first from the CUDA index, then LeRobot (its range torch>=2.7,<2.12 accepts the installed build).
uv pip install --python "$VENV/bin/python" --index-url "$TORCH_INDEX" "torch<2.12" "torchvision<0.27"
uv pip install --python "$VENV/bin/python" -e "$LEROBOT_DIR[dataset,training,groot]"

"$VENV/bin/python" - <<'EOF'
import torch
assert torch.cuda.is_available(), "CUDA not available"
name = torch.cuda.get_device_name(0)
x = torch.randn(1024, 1024, device="cuda")
print(f"torch {torch.__version__} on {name} (sm_{''.join(map(str, torch.cuda.get_device_capability(0)))}): ok {float((x @ x).sum()):.1f}")
EOF
echo "train with: PYTHON=$VENV/bin/python scripts/train/train.sh groot <hf_user>/pai_sim"
