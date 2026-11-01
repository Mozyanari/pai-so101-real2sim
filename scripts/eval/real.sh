#!/usr/bin/env bash
# Evaluation conditions B/C: run a trained policy on the real SO-101; a person starts and judges each trial.
#
#   ENABLE_FOLLOWER=1 scripts/eval/real.sh <policy checkpoint> [trials] [name]
#
# Keys: s=start a trial  y=success  n=fail  r=move the arm to the rest pose (between trials)  q=quit.
# Before pressing s, press r and place the block at the next of the 5 marked positions. Success = block on the plate and released within 45 s (judge by eye).
# Safety: the follower moves only with ENABLE_FOLLOWER=1, and each command step is clamped to MAX_STEP.
# Env: FOLLOWER_PORT FOLLOWER_ID FRONT_CAM WRIST_CAM MAX_STEP (see scripts/teleop/real.sh), RECORD=1,
# ACTION_STEPS and ASYNC (see scripts/eval/sim.sh), REST_POSE (the pose r moves to: "rest" = the sim demos' folded
# pose, the default; "start" = wrist lowered and gripper a little open, so101_pai.robot.rest.START_POSE_DEG; or 6
# comma-separated degrees; "lifted" = arm lifted off the table, so101_pai.robot.rest.LIFTED_POSE_DEG).
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"

POLICY=${1:?policy checkpoint dir or Hub id}
TRIALS=${2:-20}
NAME=${3:-B_real_$(date +%m%d_%H%M)}
LOG_DIR="$ROOT/logs/eval/$NAME"
ENABLE=false
[ "${ENABLE_FOLLOWER:-0}" = "1" ] && ENABLE=true
# A Hub id is "user/name"; anything with more slashes is a local checkpoint and must exist (checked before the arm
# is connected; otherwise the policy node would try it as a Hub id and fail later).
if [[ "$POLICY" == */*/* || "$POLICY" == /* || "$POLICY" == .* ]] && [ ! -d "$POLICY" ]; then
  echo "no checkpoint at $POLICY" >&2
  CKPTS="${POLICY%/checkpoints/*}/checkpoints"
  [ -d "$CKPTS" ] && echo "available in $CKPTS: $(ls "$CKPTS" | tr '\n' ' ')" >&2
  exit 1
fi
trap stop_all EXIT
ros_env

node follower so101_pai.robot.follower_node -p port:="${FOLLOWER_PORT:-/dev/ttyACM1}" \
  -p id:="${FOLLOWER_ID:-pai_follower}" -p enable:=$ENABLE -p max_relative_target:="${MAX_STEP:-10.0}"
# A follower that failed to connect would leave the policy commanding nobody; stop here instead.
wait_for_topic /joint_states 30 || { echo "follower did not start; see $LOG_DIR/follower.log" >&2; exit 1; }
node cam_front so101_pai.robot.camera_node -p camera:=front -p device:="${FRONT_CAM:-/dev/video0}"
node cam_wrist so101_pai.robot.camera_node -p camera:=wrist -p device:="${WRIST_CAM:-/dev/video2}"
node policy so101_pai.policy.policy_node -p policy_path:="$POLICY" -p n_action_steps:="${ACTION_STEPS:-0}" \
  -p asynchronous:="$([ "${ASYNC:-1}" = 1 ] && echo true || echo false)"
node eval so101_pai.policy.eval_node -p mode:=real -p trials:="$TRIALS" \
  -p out:="$ROOT/outputs/eval/$NAME.json" -p condition:="$NAME"
[ "${RECORD:-0}" = "1" ] && start_bag "$ROOT/data/bags/eval/$NAME"

# Show what the nodes actually took (their own startup lines), not just what was asked for.
for _ in $(seq 60); do
  grep -q "waiting for an episode start" "$LOG_DIR/policy.log" 2>/dev/null && break
  # A policy that failed to load (e.g. a wrong checkpoint path) would leave the eval waiting with nothing to run.
  grep -q "Traceback" "$LOG_DIR/policy.log" 2>/dev/null &&
    { tail -3 "$LOG_DIR/policy.log" >&2; echo "the policy failed to load; see $LOG_DIR/policy.log" >&2; exit 1; }
  sleep 1
done
echo "=== $NAME: trials=$TRIALS record=${RECORD:-0} (logs: $LOG_DIR)"
grep -h "follower on\|-> /cam\|policy .* on " "$LOG_DIR"/{follower,cam_front,cam_wrist,policy}.log 2>/dev/null | sed 's/^\[INFO\] \[[0-9.]*\] //'
grep -q "waiting for an episode start" "$LOG_DIR/policy.log" || echo "WARNING: the policy has not loaded yet; see $LOG_DIR/policy.log" >&2
case "${REST_POSE:-rest}" in
  rest) REST_DEG=$("$PY" -c "from so101_pai.robot.rest import REST_POSE_DEG as p; print(','.join(map(str, p)))") ;;
  start) REST_DEG=$("$PY" -c "from so101_pai.robot.rest import START_POSE_DEG as p; print(','.join(map(str, p)))") ;;
  lifted) REST_DEG=$("$PY" -c "from so101_pai.robot.rest import LIFTED_POSE_DEG as p; print(','.join(map(str, p)))") ;;
  *) REST_DEG=$REST_POSE ;;
esac
echo "r moves to: ${REST_POSE:-rest} = [$REST_DEG] deg"
"$PY" -m so101_pai.teleop.operator_node --ros-args -p mode:=real -p rest:=true -p rest_pose_deg:="[$REST_DEG]"
