#!/usr/bin/env bash
# Evaluation conditions B/C: run a trained policy on the real SO-101; a person starts and judges each trial.
#
#   ENABLE_FOLLOWER=1 scripts/eval/real.sh <policy checkpoint> [trials] [name]
#
# Keys: s=start a trial  y=success  n=fail  q=quit. Place the block at the next of the 5 marked positions
# before pressing s. Success = block on the plate and released within 30 s (judge by eye).
# Safety: the follower moves only with ENABLE_FOLLOWER=1, and each command step is clamped to MAX_STEP.
# Env: FOLLOWER_PORT FOLLOWER_ID FRONT_CAM WRIST_CAM MAX_STEP (see scripts/teleop/real.sh), RECORD=1.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"

POLICY=${1:?policy checkpoint dir or Hub id}
TRIALS=${2:-20}
NAME=${3:-B_real_$(date +%m%d_%H%M)}
LOG_DIR="$ROOT/logs/eval/$NAME"
ENABLE=false
[ "${ENABLE_FOLLOWER:-0}" = "1" ] && ENABLE=true
trap stop_all EXIT
ros_env

node follower so101_pai.robot.follower_node -p port:="${FOLLOWER_PORT:-/dev/ttyACM1}" \
  -p id:="${FOLLOWER_ID:-pai_follower}" -p enable:=$ENABLE -p max_relative_target:="${MAX_STEP:-10.0}"
node cam_front so101_pai.robot.camera_node -p camera:=front -p device:="${FRONT_CAM:-/dev/video0}"
node cam_wrist so101_pai.robot.camera_node -p camera:=wrist -p device:="${WRIST_CAM:-/dev/video2}"
node policy so101_pai.policy.policy_node -p policy_path:="$POLICY"
node eval so101_pai.policy.eval_node -p mode:=real -p trials:="$TRIALS" \
  -p out:="$ROOT/outputs/eval/$NAME.json" -p condition:="$NAME"
[ "${RECORD:-0}" = "1" ] && start_bag "$ROOT/data/bags/eval/$NAME"

echo "follower enable=$ENABLE"
"$PY" -m so101_pai.teleop.operator_node --ros-args -p mode:=real
