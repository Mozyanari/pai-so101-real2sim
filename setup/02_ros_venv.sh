#!/usr/bin/env bash
# Create the repo-local uv venv for the ROS 2 nodes (leader/follower/policy) and bag2lerobot.
# Python 3.12 matches ROS 2 Jazzy, and --system-site-packages exposes rclpy/rosbag2_py from /opt/ros/jazzy
# (read-only). Everything else, including LeRobot v0.6.1 from ~/lerobot, is installed into .venv.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LEROBOT_DIR="${LEROBOT_DIR:-$HOME/lerobot}"

cd "$ROOT"
if [ ! -d .venv ]; then
  uv venv --python /usr/bin/python3.12 --system-site-packages .venv
fi

# Blackwell (RTX 50) needs CUDA 12.8 wheels.
uv pip install --python .venv/bin/python "torch>=2.7,<2.12" "torchvision>=0.22,<0.27" \
  --index-url https://download.pytorch.org/whl/cu128
uv pip install --python .venv/bin/python -e "${LEROBOT_DIR}[feetech,dataset,training,smolvla]"
# System dist-packages (apt) are built for numpy 1.x; shadow the ones LeRobot pulls in with venv copies.
uv pip install --python .venv/bin/python scipy matplotlib
uv pip install --python .venv/bin/python "rerun-sdk>=0.24.0,<0.34.0"  # lerobot-dataset-viz (the viz extra pulls more than needed)

echo "DONE: .venv is ready (source /opt/ros/jazzy/setup.bash before use)"
