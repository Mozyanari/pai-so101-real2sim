#!/usr/bin/env bash
# Clone 3DGRUT (3DGRT/3DGUT Gaussian reconstruction) into third_party/3dgrut and build its own env.
# Follows the official "Sub-option A2 — conda-managed CUDA" path in 3DGRUT's README:
#   scripts/create_conda.sh  -> conda env "3dgrut" (Python 3.11, CUDA toolkit 12.8.1 from nvidia's conda label,
#                               cmake, ninja; CC/CXX/TORCH_* persisted as env vars)
#   install_env_uv.sh        -> uv installs torch 2.8.0+cu128, 3DGRUT, tiny-cuda-nn, Kaolin, ppisp, fused-ssim,
#                               slangc into that same conda env (UV_PROJECT_ENVIRONMENT=$CONDA_PREFIX)
# Why conda and not a plain uv .venv: building the CUDA extensions needs nvcc 12.8, and this machine has no
# system CUDA toolkit (sub-option A1 would need apt/sudo). Sub-option A3 (.venv-local CUDA) installs the CUDA
# runfile with --silent, which accepts the CUDA EULA on the user's behalf; the conda toolkit is the same
# route setup/01_leisaac_env.sh already uses. Nothing is installed system-wide; see docs/DESIGN.md §3.
set -euo pipefail

unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
      ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_DIR="$ROOT/third_party/3dgrut"
ENV_NAME=3dgrut
# Commit verified on 2026-10-27 (main, 2026-09-22). Override with THREEDGRUT_REF=<sha|tag>.
THREEDGRUT_REF="${THREEDGRUT_REF:-e6f4552b408af020a96de09ab5757f9b57430aaa}"
# CUDA 12.8 -> torch 2.8.0+cu128, which supports Blackwell (sm_120).
export CUDA_VERSION=12.8
# Build the CUDA extensions only for the RTX 5060 Ti (sm_120); the upstream default builds 6 archs and is slow.
export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-12.0}"

if [ ! -d "$REPO_DIR/.git" ]; then
  git clone --recursive https://github.com/nv-tlabs/3dgrut.git "$REPO_DIR"
fi
cd "$REPO_DIR"
git checkout -q "$THREEDGRUT_REF"
git submodule update --init --recursive

# The upstream scripts call the `conda` executable from child shells.
export PATH="$HOME/miniforge3/condabin:$PATH"
# conda's activate.d hooks (cuda-nvcc, gcc) reference unset variables, so nounset stays off from here.
set +u
source "$HOME/miniforge3/etc/profile.d/conda.sh"
if ! conda env list | grep -qE "^${ENV_NAME}\s"; then
  bash scripts/create_conda.sh "$ENV_NAME"
fi
conda activate "$ENV_NAME"

# install_env_uv.sh rebuilds tiny-cuda-nn and the other extensions every time (~10+ min); skip when they
# already import. FORCE=1 reinstalls.
if [ "${FORCE:-0}" != "1" ] && python - <<'PY' 2>/dev/null
import torch, threedgrut, tinycudann, kaolin, ppisp  # noqa: F401
from fused_ssim import fused_ssim  # noqa: F401
assert torch.cuda.is_available()
PY
then
  echo "3DGRUT env already installed (FORCE=1 to reinstall)"
else
  bash install_env_uv.sh "$ENV_NAME"
fi

python -c "import torch; print('torch', torch.__version__, 'cuda', torch.version.cuda, torch.cuda.get_device_name(0), torch.cuda.get_arch_list())"
nvcc --version | tail -n 2
echo "DONE: conda env '$ENV_NAME' is ready (3DGRUT at $REPO_DIR)"
