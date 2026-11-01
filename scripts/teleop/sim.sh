#!/usr/bin/env bash
# Teleop in the sim with the SO-101 leader arm and record every episode to a rosbag.
#
#   scripts/teleop/sim.sh [session]            # leader arm on /dev/ttyACM0 (needs calibration id pai_leader)
#   FAKE_LEADER=1 scripts/teleop/sim.sh test    # scripted arm motion instead of the leader (no hardware)
#
# Keys in this terminal: s=start  y=success  n=fail  r=reset  q=quit. The sim also ends an episode by itself
# when the block is on the plate and released (success) or after 45 s (fail). The scene resets right after an
# episode ends, so s can be pressed again at once.
# The operator watches the front and wrist cameras in two rqt_image_view windows while Isaac Sim runs headless:
# its own GUI viewport cut recording from 30 Hz to ~20 Hz on the RTX 5060 Ti. GUI=1 shows the Isaac Sim window instead.
# Output: data/bags/sim/<session>/ (convert with scripts/dataset/build.sh). Env: GUI=1, LEADER_PORT, LEADER_ID.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"

SESSION=${1:-$(date +%m%d_%H%M%S)}
BAG="$ROOT/data/bags/sim/$SESSION"
LOG_DIR="$ROOT/logs/teleop_sim/$SESSION"
[ -e "$BAG" ] && { echo "$BAG already exists; pick another session name" >&2; exit 1; }
trap stop_all EXIT

if [ "${GUI:-0}" = "1" ]; then
  start_sim_bridge "$LOG_DIR/bridge.log"
else
  HEADLESS=1 start_sim_bridge "$LOG_DIR/bridge.log"
fi
ros_env
wait_for_topic /joint_states
if [ "${GUI:-0}" != "1" ]; then
  for cam in front wrist; do
    start_bg "$LOG_DIR/view_$cam.log" ros2 run rqt_image_view rqt_image_view "/cam_$cam/image_raw"
  done
fi

if [ "${FAKE_LEADER:-0}" = "1" ]; then
  node leader so101_pai.robot.fake_leader_node
else
  node leader so101_pai.robot.leader_node -p port:="${LEADER_PORT:-/dev/ttyACM0}" -p id:="${LEADER_ID:-pai_leader}"
fi
start_bag "$BAG"

"$PY" -m so101_pai.teleop.operator_node --ros-args -p mode:=sim
echo "bag: $BAG"
