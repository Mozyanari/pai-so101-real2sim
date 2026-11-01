#!/usr/bin/env bash
# Create the conda env "colmap": COLMAP (CUDA build) + ffmpeg, both from conda-forge.
# Used by scripts/recon/run.sh for frame extraction and structure-from-motion.
# Nothing is installed system-wide (no apt, no sudo); see docs/DESIGN.md §3.
set -euo pipefail

# ~/.bashrc sources ROS 2 Jazzy; keep its Python/CMake paths out of this env.
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
      ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE

ENV_NAME=colmap
# conda-forge ships CUDA 12.9 and 13.4 builds of COLMAP. Driver 580 supports CUDA <= 13.0, so pin the
# cuda_129 variant (it bundles its own cudart; no system CUDA toolkit is needed). Set COLMAP_BUILD=cpu*
# to fall back to the CPU build.
COLMAP_VERSION="${COLMAP_VERSION:-4.2.1}"
COLMAP_BUILD="${COLMAP_BUILD:-cuda_129*}"

# conda's activate.d hooks reference unset variables, so nounset stays off from here.
set +u
source "$HOME/miniforge3/etc/profile.d/conda.sh"
if ! conda env list | grep -qE "^${ENV_NAME}\s"; then
  conda create -y -n "$ENV_NAME" -c conda-forge --override-channels \
    "colmap=${COLMAP_VERSION}=${COLMAP_BUILD}" ffmpeg
else
  conda install -y -n "$ENV_NAME" -c conda-forge --override-channels \
    "colmap=${COLMAP_VERSION}=${COLMAP_BUILD}" ffmpeg
fi
conda activate "$ENV_NAME"

colmap -h | head -n 3
ffmpeg -hide_banner -version | head -n 1
echo "DONE: conda env '$ENV_NAME' is ready"
