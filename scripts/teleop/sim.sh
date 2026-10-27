#!/usr/bin/env bash
# Teleop in the sim with the SO-101 leader arm and record every episode to a rosbag.
#
#   scripts/teleop/sim.sh [session]            # leader arm on /dev/ttyACM0 (needs calibration id pai_leader)
#   FAKE_LEADER=1 scripts/teleop/sim.sh test    # scripted arm motion instead of the leader (no hardware)
#
# Keys in this terminal: s=start  y=success  n=fail  r=reset  q=quit. The sim also ends an episode by itself
# when the block is on the plate and released (success) or after 30 s (fail). Wait ~6 s after an episode ends
# for the scene to reset before pressing s again.
# Output: data/bags/sim/<session>/ (convert with scripts/dataset/build.sh). Env: HEADLESS=1, LEADER_PORT, LEADER_ID.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"

SESSION=${1:-$(date +%m%d_%H%M%S)}
BAG="$ROOT/data/bags/sim/$SESSION"
LOG_DIR="$ROOT/logs/teleop_sim/$SESSION"
[ -e "$BAG" ] && { echo "$BAG already exists; pick another session name" >&2; exit 1; }
trap stop_all EXIT

start_sim_bridge "$LOG_DIR/bridge.log"
ros_env
wait_for_topic /joint_states

if [ "${FAKE_LEADER:-0}" = "1" ]; then
  node leader so101_pai.robot.fake_leader_node
else
  node leader so101_pai.robot.leader_node -p port:="${LEADER_PORT:-/dev/ttyACM0}" -p id:="${LEADER_ID:-pai_leader}"
fi
start_bag "$BAG"

"$PY" -m so101_pai.teleop.operator_node --ros-args -p mode:=sim
echo "bag: $BAG"
