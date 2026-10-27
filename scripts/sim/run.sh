#!/usr/bin/env bash
# Run a command inside the conda env "leisaac" with ROS 2 Jazzy (sourced by ~/.bashrc) stripped out.
#   scripts/sim/run.sh python third_party/leisaac/scripts/environments/teleoperation/teleop_se3_agent.py ...
# The user accepted the Isaac Sim EULA on 2026-10-27.
set -eo pipefail

unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH \
      ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE
export OMNI_KIT_ACCEPT_EULA=YES

source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate leisaac

# LEISAAC_ROS=1: use Isaac Sim's bundled ROS 2 Jazzy (Python 3.11 build) so sim scripts can import rclpy.
# It talks to the system Jazzy (Python 3.12) nodes over DDS as usual.
if [ "${LEISAAC_ROS:-0}" = "1" ]; then
  ROS2_BRIDGE="$CONDA_PREFIX/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/jazzy"
  export ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export PYTHONPATH="$ROS2_BRIDGE/rclpy"
  export LD_LIBRARY_PATH="$ROS2_BRIDGE/lib"
fi
exec "$@"
