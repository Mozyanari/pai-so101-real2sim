#!/usr/bin/env bash
# Record sim demos with the scripted pick-and-place expert instead of a human on the leader arm.
#
#   scripts/teleop/expert.sh [session] [episodes]     # default: expert_<date>, 50 successful episodes
#
# Isaac Sim runs headless; the expert (sim/ros_bridge.py --expert) starts each episode once the recorder is
# subscribed and publishes its joint commands on /leader/joint_states, so the bag has the same topics as teleop.
# Output: data/bags/sim/<session>/ (convert with scripts/dataset/build.sh). Env: SEED (expert jitter), NOISE_DEG
# (std of the smooth noise added to the executed arm commands but not to the recorded ones; 0 disables it).
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"

SESSION=${1:-expert_$(date +%m%d_%H%M%S)}
EPISODES=${2:-50}
BAG="$ROOT/data/bags/sim/$SESSION"
LOG_DIR="$ROOT/logs/teleop_expert/$SESSION"
[ -e "$BAG" ] && { echo "$BAG already exists; pick another session name" >&2; exit 1; }
trap stop_all EXIT

HEADLESS=1 start_sim_bridge "$LOG_DIR/bridge.log" --expert "$EPISODES" --seed "${SEED:-0}" \
  --noise-deg "${NOISE_DEG:-2}"
ros_env
wait_for_topic /joint_states
start_bag "$BAG"

# ~10 s per episode plus resets; allow 2x the attempts the bridge may use (it quits after 2N).
limit=$((60 + 2 * EPISODES * 20))
echo "recording up to $EPISODES successful episodes (timeout ${limit}s) ..."
timeout "$limit" bash -c "until grep -q 'expert done' '$LOG_DIR/bridge.log'; do sleep 5; done" \
  || echo "expert did not finish within ${limit}s; keeping what was recorded" >&2
grep -o "expert done.*" "$LOG_DIR/bridge.log" || true
stop_all
echo "bag: $BAG"
