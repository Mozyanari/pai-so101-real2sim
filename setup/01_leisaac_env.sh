#!/usr/bin/env bash
# Create the conda env "leisaac": Isaac Sim 5.1 + Isaac Lab (LeIsaac submodule) + LeIsaac.
# Follows https://lightwheelai.github.io/leisaac/docs/getting_started/installation (install from source).
# Everything goes into the conda env; nothing is installed system-wide.
set -euo pipefail

# ~/.bashrc sources ROS 2 Jazzy (Python 3.12); keep it out of this Python 3.11 env.
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
      ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY="$ROOT/third_party"
ENV_NAME=leisaac

# conda's activate.d hooks (e.g. cuda-nvcc) reference unset variables, so nounset stays off from here.
set +u
source "$HOME/miniforge3/etc/profile.d/conda.sh"
if ! conda env list | grep -qE "^${ENV_NAME}\s"; then
  conda create -y -n "$ENV_NAME" python=3.11
fi
conda activate "$ENV_NAME"

conda install -y -c "nvidia/label/cuda-12.8.1" cuda-toolkit
pip install -U torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url https://download.pytorch.org/whl/cu128
pip install --upgrade pip
# setuptools>=81 dropped pkg_resources, which flatdict 4.0.1 (an isaaclab dependency) needs to build.
pip install "setuptools<81"
pip install --no-build-isolation flatdict==4.0.1
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com

mkdir -p "$THIRD_PARTY"
if [ ! -d "$THIRD_PARTY/leisaac" ]; then
  git clone --recursive https://github.com/LightwheelAI/leisaac.git "$THIRD_PARTY/leisaac"
fi

# cmake and build-essential are already installed system-wide, so the doc's apt step is skipped.
# The user accepted the Isaac Sim EULA on 2026-10-27; isaaclab.sh imports isaacsim at the end.
export OMNI_KIT_ACCEPT_EULA=YES
cd "$THIRD_PARTY/leisaac/dependencies/IsaacLab"
./isaaclab.sh --install
# Latest tensordict / stable-baselines3 require torch>=2.8; keep versions that work with torch 2.7.
pip install "tensordict<0.11" "stable-baselines3<2.8" torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 \
  --extra-index-url https://download.pytorch.org/whl/cu128

cd "$THIRD_PARTY/leisaac"
pip install -e source/leisaac

# Isaac Sim 5.1 binaries are built against numpy 1.26; later installs may pull numpy 2.x and crash Kit at startup.
pip install "numpy==1.26.0" "pandas<2.3" "click==8.1.7" "idna==3.10" "psutil==5.9.8" "typing_extensions==4.12.2" "packaging==23.0"

echo "DONE: conda env '$ENV_NAME' is ready"
