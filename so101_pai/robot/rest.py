"""Move the real follower back to the pose the sim demo episodes start from.

rest_trajectory() is shared by the eval operator (key r between trials) and this module's one-shot node, which
scripts/real/rest.sh runs next to the follower node:
    python -m so101_pai.robot.rest --ros-args -p speed_deg_s:=30.0
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from so101_pai.common import contract
from so101_pai.common.ros_msgs import joint_state_msg, joint_state_to_array

# Where the sim demos start (JOINT_NAMES order): the task's reset pose (REST_POSE_DEG in place_block_env_cfg.py)
# after the arm settles under gravity. Median first frame of the real1031c_b28/b30/b32 bags; elbow and gripper sit
# at their joint limits.
REST_POSE_DEG = np.array([0.0, -98.2, 90.0, 52.3, 0.0, -10.0])
SPEED_DEG_S = 30.0


# A start pose for real eval (scripts/eval/real.sh REST_POSE=start): wrist lowered and gripper a little open, like
# the real demos that did not start folded (ep4-9 of pai_real_v1). From the folded rest pose, ACT trained on those
# demos predicts waiting there for up to the whole chunk, since the demos sat still for 1-4 s after the start (11/1).
START_POSE_DEG = np.array([0.0, -98.2, 89.0, 40.0, 0.0, 12.0])
# Arm lifted off the table (REST_POSE=lifted): the median pose of the 10 real demos when the shoulder first lifts past
# -75 deg on the way to the block, so the policy sees a state from early in its own approach.
LIFTED_POSE_DEG = np.array([0.0, -75.0, 71.0, 47.0, -10.0, 12.0])


def rest_trajectory(
    start_rad: np.ndarray, fps: float, speed_deg_s: float = SPEED_DEG_S, target_deg: np.ndarray = REST_POSE_DEG
) -> list[np.ndarray]:
    """Joint targets (rad) from `start_rad` to `target_deg`, eased in and out, at about `speed_deg_s` on average."""
    start = np.asarray(start_rad, dtype=np.float64)
    target = np.radians(np.asarray(target_deg, dtype=np.float64))
    distance_deg = float(np.degrees(np.abs(target - start)).max())
    steps = max(1, math.ceil(distance_deg / speed_deg_s * fps))
    return [start + (target - start) * (1 - math.cos(math.pi * k / steps)) / 2 for k in range(1, steps + 1)]


class RestNode(Node):
    """Wait for one follower state, play the trajectory to rest on the action topic, then exit."""

    def __init__(self, **node_kwargs) -> None:
        # node_kwargs (e.g. parameter_overrides) let tests build the node without the command line.
        super().__init__("so101_rest", **node_kwargs)
        self.speed = self.declare_parameter("speed_deg_s", SPEED_DEG_S).value
        self.plan: list[np.ndarray] | None = None
        self.done = False
        self.action_pub = self.create_publisher(JointState, contract.TOPIC_ACTION, 10)
        self.create_subscription(JointState, contract.TOPIC_STATE, self.on_state, 10)
        self.create_timer(1.0 / contract.FPS, self.tick)

    def on_state(self, msg: JointState) -> None:
        if self.plan is None:
            self.plan = rest_trajectory(joint_state_to_array(msg), contract.FPS, self.speed)
            self.get_logger().info(f"moving to the rest pose in {len(self.plan) / contract.FPS:.1f} s")

    def tick(self) -> None:
        if not self.plan:
            self.done = self.plan is not None
            return
        self.action_pub.publish(joint_state_msg(self.get_clock().now().to_msg(), self.plan.pop(0)))


def main() -> None:
    rclpy.init()
    node = RestNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
