#!/usr/bin/env bash
# End-to-end check without hardware: sim bridge + fake leader -> rosbag2 -> bag2lerobot -> LeRobot dataset.
#   scripts/sim/e2e_pipeline.sh            (writes data/e2e/{bag,dataset}, logs in logs/e2e/)
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../lib/common.sh"

OUT="$ROOT/data/e2e"
LOG_DIR="$ROOT/logs/e2e"
rm -rf "$OUT" && mkdir -p "$OUT"
trap stop_all EXIT

HEADLESS=1 start_sim_bridge "$LOG_DIR/bridge.log"
ros_env
wait_for_topic /joint_states
node leader so101_pai.robot.fake_leader_node
start_bag "$OUT/bag"
sleep 3

cmd() { ros2 topic pub --once /episode/command std_msgs/msg/String "{data: $1}" >/dev/null; }
cmd start; sleep 4; cmd fail; sleep 8
cmd start; sleep 4; cmd success; sleep 2
stop_all

ros2 bag info "$OUT/bag" | grep -E "Duration|Topic:"
"$PY" -m so101_pai.dataset.bag2lerobot "$OUT/bag" --repo-id local/e2e --root "$OUT/dataset" --include-failed
"$PY" - "$OUT/dataset" <<'EOF'
import sys
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset("local/e2e", root=sys.argv[1])
print("episodes", ds.meta.total_episodes, "frames", ds.meta.total_frames, "fps", ds.meta.fps)
first = ds[0]
print("task:", first["task"])
print("state[0]:", np.round(first["observation.state"].numpy(), 1))
print("action[0]:", np.round(first["action"].numpy(), 1))
print("front image:", tuple(first["observation.images.front"].shape))
EOF
