#!/usr/bin/env bash
# Move the real follower to the rest pose the sim episodes start from (joint interpolation, ~30 deg/s), then exit.
# Use it when nothing else drives the follower; during scripts/eval/real.sh press r instead.
#
#   scripts/real/rest.sh
#
# Env: FOLLOWER_PORT=/dev/ttyACM1 FOLLOWER_ID=pai_follower SPEED=30 (deg/s) MAX_STEP=10 (follower step limit)
# On exit the follower node disconnects, which turns the torque off; the folded rest pose stays put without it.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"

LOG_DIR="$ROOT/logs/real_rest"
trap stop_all EXIT
ros_env

node follower so101_pai.robot.follower_node -p port:="${FOLLOWER_PORT:-/dev/ttyACM1}" \
  -p id:="${FOLLOWER_ID:-pai_follower}" -p enable:=true -p max_relative_target:="${MAX_STEP:-10.0}"
wait_for_topic /joint_states 30 || { echo "follower did not start; see $LOG_DIR/follower.log" >&2; exit 1; }
"$PY" -m so101_pai.robot.rest --ros-args -p speed_deg_s:="${SPEED:-30.0}"
sleep 1  # let the follower reach the last target before its node disconnects
