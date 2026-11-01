"""Publish the SO-101 leader arm on contract.TOPIC_ACTION (sensor_msgs/JointState, radians).

ros2-style run (inside .venv with /opt/ros/jazzy sourced):
    python -m so101_pai.robot.leader_node --ros-args -p port:=/dev/ttyACM0 -p id:=my_leader
Used for both sim and real teleop, so recorded actions are identical in both.
"""

from __future__ import annotations

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState

from so101_pai.common import contract
from so101_pai.common.ros_msgs import joint_state_msg, motor_dict_to_array
from so101_pai.common.ros_node import run
from so101_pai.robot.calibration import WRIST_ROLL_OFFSET_DEG, arm_motor_to_radians, connect_calibrated


class LeaderNode(Node):
    def __init__(self, **node_kwargs) -> None:
        # node_kwargs (e.g. parameter_overrides) let tests build the node without the command line.
        super().__init__("so101_leader", **node_kwargs)
        from lerobot.teleoperators.so_leader import SO101LeaderConfig, SOLeader

        port = self.declare_parameter("port", "/dev/ttyACM0").value
        # Calibration id under ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader. The older
        # my_awesome_leader_arm file no longer matches the motors; create pai_leader with lerobot-calibrate.
        arm_id = self.declare_parameter("id", "pai_leader").value
        rate_hz = self.declare_parameter("rate_hz", 60.0).value
        # Same wrist_roll correction as the follower node, so teleop maps pose to pose.
        self.roll_offset = self.declare_parameter("wrist_roll_offset_deg", WRIST_ROLL_OFFSET_DEG).value

        # RANGE_M100_100 matches the motor units assumed by arm_motor_to_radians (LeIsaac convention).
        self.leader = SOLeader(SO101LeaderConfig(port=port, id=arm_id, use_degrees=False))
        connect_calibrated(
            self.leader, f"--teleop.type=so101_leader --teleop.port={port} --teleop.id={arm_id}", self.get_logger()
        )
        self.publisher = self.create_publisher(JointState, contract.TOPIC_ACTION, 10)
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(f"leader on {port} publishing {contract.TOPIC_ACTION}")

    def tick(self) -> None:
        motor = motor_dict_to_array(self.leader.get_action())
        radians = arm_motor_to_radians(motor, self.roll_offset)
        self.publisher.publish(joint_state_msg(self.get_clock().now().to_msg(), np.asarray(radians)))

    def destroy_node(self) -> None:
        self.leader.disconnect()
        super().destroy_node()


def main() -> None:
    run(LeaderNode)


if __name__ == "__main__":
    main()
