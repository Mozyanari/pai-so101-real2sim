"""Stand-in for the leader arm: publish a scripted joint trajectory on contract.TOPIC_ACTION.

Starts at the rest pose and sweeps shoulder_pan, for pipeline tests without hardware.
    python -m so101_pai.robot.fake_leader_node --ros-args -p amplitude_deg:=30.0 -p period_s:=4.0
"""

from __future__ import annotations

import math

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState

from so101_pai.common import contract
from so101_pai.common.ros_msgs import joint_state_msg
from so101_pai.common.ros_node import run
from so101_pai.robot.rest import REST_POSE_DEG


class FakeLeaderNode(Node):
    def __init__(self, **node_kwargs) -> None:
        # node_kwargs (e.g. parameter_overrides) let tests build the node without the command line.
        super().__init__("fake_leader", **node_kwargs)
        self.amplitude = math.radians(self.declare_parameter("amplitude_deg", 30.0).value)
        self.period = self.declare_parameter("period_s", 4.0).value
        self.publisher = self.create_publisher(JointState, contract.TOPIC_ACTION, 10)
        self.start = self.get_clock().now()
        self.create_timer(1.0 / 60.0, self.tick)

    def tick(self) -> None:
        t = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        joints = np.radians(REST_POSE_DEG)
        joints[0] = self.amplitude * math.sin(2 * math.pi * t / self.period)
        self.publisher.publish(joint_state_msg(self.get_clock().now().to_msg(), joints))


def main() -> None:
    run(FakeLeaderNode)


if __name__ == "__main__":
    main()
