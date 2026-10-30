#!/usr/bin/env bash
# Shared helpers for the launch scripts. Source it:  source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"
#
# Process rules learned the hard way (docs/WORKLOG.md 2026-10-27):
#  - Background jobs of a non-interactive shell ignore SIGINT, so `ros2 bag record` is stopped with SIGTERM
#    (it still closes the bag and writes metadata.yaml).
#  - Isaac Sim ignores SIGTERM, so every process gets TERM, a grace period, then KILL.
#  - Never `pkill -f <pattern>`: the pattern also matches the shell running it. Track PIDs instead.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PY="$ROOT/.venv/bin/python"
BAG_TOPICS=(/joint_states /leader/joint_states /cam_front/image_raw /cam_wrist/image_raw /episode/status)
PIDS=()

# Load ROS 2 Jazzy for the system-Python nodes (setup.bash reads unset variables, hence no `set -u`).
ros_env() { set +u; source /opt/ros/jazzy/setup.bash; }

# start_bg <log file> <command...>: run in the background and remember the PID for stop_all.
start_bg() {
  local log=$1; shift
  mkdir -p "$(dirname "$log")"
  "$@" > "$log" 2>&1 &
  PIDS+=($!)
  echo "started $(basename "$log" .log) (pid $!), log: $log"
}

# stop_all (safe under set -e): SIGTERM everything started with start_bg (newest first), wait, then SIGKILL what is left.
stop_all() {
  local pid i
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do
    pid=${PIDS[i]}
    pkill -TERM -P "$pid" 2>/dev/null || true; kill -TERM "$pid" 2>/dev/null || true
  done
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    local alive=0
    for pid in "${PIDS[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1 || true; done
    [ $alive = 0 ] && break
    sleep 0.5
  done
  for pid in "${PIDS[@]}"; do pkill -KILL -P "$pid" 2>/dev/null || true; kill -KILL "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
  PIDS=()
}

# start_sim_bridge <log file> [extra args]: the Isaac Sim robot (headless when HEADLESS=1).
# BACKGROUND picks the 3DGS background USDZ (default: the aligned workspace with the real follower removed, when it
# exists); BACKGROUND=none runs on the plain sim table.
start_sim_bridge() {
  local log=$1; shift
  local view=()
  [ "${HEADLESS:-0}" = "1" ] && view=(--headless)
  local background=${BACKGROUND:-$ROOT/assets/scene/workspace_crop.usdz}
  if [ "$background" != "none" ] && [ -f "$background" ]; then
    export PAI_BACKGROUND_USD="$background"
    echo "background: $background"
  else
    unset PAI_BACKGROUND_USD
  fi
  LEISAAC_ROS=1 start_bg "$log" "$ROOT/scripts/sim/run.sh" python "$ROOT/sim/ros_bridge.py" \
    "${view[@]}" --enable_cameras --rendering_mode performance "$@"
}

# wait_for_topic <topic> [timeout s]: block until one message arrives (the sim takes ~1-2 min to start).
wait_for_topic() {
  local topic=$1 limit=${2:-600}
  echo "waiting for $topic ..."
  timeout "$limit" bash -c "until ros2 topic echo --once $topic >/dev/null 2>&1; do sleep 2; done" \
    || { echo "no message on $topic after ${limit}s" >&2; return 1; }
}

# start_bag <output dir>: record the contract topics (mcap + zstd; ~13 MB/s with two 640x480 cameras).
start_bag() {
  local out=$1
  mkdir -p "$(dirname "$out")"
  start_bg "$out.log" ros2 bag record -s mcap --storage-preset-profile zstd_fast -o "$out" "${BAG_TOPICS[@]}"
}

# node <log name> <module> [ros args...]: run a so101_pai ROS node from the project venv.
node() {
  local name=$1 module=$2; shift 2
  start_bg "$LOG_DIR/$name.log" "$PY" -m "$module" --ros-args "$@"
}
