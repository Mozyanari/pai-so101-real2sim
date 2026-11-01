#!/usr/bin/env bash
# Evaluation condition A: run a trained policy in the sim for N trials and write a JSON summary.
#
#   scripts/eval/sim.sh outputs/train/<run>/checkpoints/last/pretrained_model [trials] [name]
#
# The eval node drives the bridge (reset -> settle -> start -> wait for success/timeout) by itself.
# Env: ACTION_STEPS (actions run per predicted chunk; 0 = the checkpoint's setting), ASYNC (1 = infer the next
# chunk while the current one plays, the default; 0 = LeRobot's select_action, which pauses for every inference).
# Output: outputs/eval/<name>.json and, with RECORD=1, a rosbag of the rollouts in data/bags/eval/<name>/.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"

POLICY=${1:?policy checkpoint dir or Hub id}
TRIALS=${2:-20}
NAME=${3:-A_sim_$(date +%m%d_%H%M)}
LOG_DIR="$ROOT/logs/eval/$NAME"
trap stop_all EXIT

HEADLESS=${HEADLESS:-1} start_sim_bridge "$LOG_DIR/bridge.log"
ros_env
wait_for_topic /joint_states
node policy so101_pai.policy.policy_node -p policy_path:="$POLICY" -p n_action_steps:="${ACTION_STEPS:-0}" \
  -p asynchronous:="$([ "${ASYNC:-1}" = 1 ] && echo true || echo false)"
[ "${RECORD:-0}" = "1" ] && start_bag "$ROOT/data/bags/eval/$NAME"

"$PY" -m so101_pai.policy.eval_node --ros-args -p mode:=sim -p trials:="$TRIALS" \
  -p out:="$ROOT/outputs/eval/$NAME.json" -p condition:="$NAME"
