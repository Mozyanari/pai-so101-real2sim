#!/usr/bin/env bash
# Teleop the real SO-101 (leader -> follower) and record episodes to a rosbag, under the same topic contract
# as the sim. Used for the ~10 real episodes of evaluation condition C.
#
#   ENABLE_FOLLOWER=1 scripts/teleop/real.sh [session]
#
# The follower only moves with ENABLE_FOLLOWER=1; without it the follower node just publishes joint states.
# Keys: s=start  y=success  n=fail  q=quit (reset the block by hand between episodes).
# Env: LEADER_PORT=/dev/ttyACM0 FOLLOWER_PORT=/dev/ttyACM1 LEADER_ID=pai_leader FOLLOWER_ID=pai_follower
#      FRONT_CAM=/dev/video0 WRIST_CAM=/dev/video2 MAX_STEP=10 (follower step limit, motor units)
# Output: data/bags/real/<session>/
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"

SESSION=${1:-$(date +%m%d_%H%M%S)}
BAG="$ROOT/data/bags/real/$SESSION"
LOG_DIR="$ROOT/logs/teleop_real/$SESSION"
[ -e "$BAG" ] && { echo "$BAG already exists; pick another session name" >&2; exit 1; }
ENABLE=false
[ "${ENABLE_FOLLOWER:-0}" = "1" ] && ENABLE=true
trap stop_all EXIT
ros_env

node follower so101_pai.robot.follower_node -p port:="${FOLLOWER_PORT:-/dev/ttyACM1}" \
  -p id:="${FOLLOWER_ID:-pai_follower}" -p enable:=$ENABLE -p max_relative_target:="${MAX_STEP:-10.0}"
node cam_front so101_pai.robot.camera_node -p camera:=front -p device:="${FRONT_CAM:-/dev/video0}"
node cam_wrist so101_pai.robot.camera_node -p camera:=wrist -p device:="${WRIST_CAM:-/dev/video2}"
node leader so101_pai.robot.leader_node -p port:="${LEADER_PORT:-/dev/ttyACM0}" -p id:="${LEADER_ID:-pai_leader}"
wait_for_topic /joint_states 30
start_bag "$BAG"

echo "follower enable=$ENABLE"
"$PY" -m so101_pai.teleop.operator_node --ros-args -p mode:=real
echo "bag: $BAG"
