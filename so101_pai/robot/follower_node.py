"""Drive the real SO-101 follower from contract.TOPIC_ACTION and publish its state on contract.TOPIC_STATE.

    python -m so101_pai.robot.follower_node --ros-args -p port:=/dev/ttyACM1 -p id:=my_follower -p enable:=true

Safety: the arm only moves when `enable` is true, and each command step is clamped by
`max_relative_target` (LeRobot motor units). With enable=false the node just publishes state.
"""

from __future__ import annotations

from rclpy.node import Node
from sensor_msgs.msg import JointState

from so101_pai.common import contract
from so101_pai.common.ros_msgs import array_to_motor_dict, joint_state_msg, joint_state_to_array, motor_dict_to_array
from so101_pai.common.ros_node import run
from so101_pai.robot.calibration import (
    WRIST_ROLL_OFFSET_DEG,
    arm_motor_to_radians,
    connect_calibrated,
    radians_to_arm_motor,
)


class FollowerNode(Node):
    def __init__(self, **node_kwargs) -> None:
        # node_kwargs (e.g. parameter_overrides) let tests build the node without the command line.
        super().__init__("so101_follower", **node_kwargs)
        from lerobot.robots.so_follower import SO101FollowerConfig, SOFollower

        port = self.declare_parameter("port", "/dev/ttyACM1").value
        # Calibration id under ~/.cache/huggingface/lerobot/calibration/robots/so_follower
        # (create it with lerobot-calibrate).
        arm_id = self.declare_parameter("id", "pai_follower").value
        rate_hz = self.declare_parameter("rate_hz", float(contract.FPS)).value
        max_step = self.declare_parameter("max_relative_target", 10.0).value
        self.enable = self.declare_parameter("enable", False).value
        # Where the sim's wrist_roll zero is on this arm (see calibration.WRIST_ROLL_OFFSET_DEG).
        self.roll_offset = self.declare_parameter("wrist_roll_offset_deg", WRIST_ROLL_OFFSET_DEG).value

        self.robot = SOFollower(
            SO101FollowerConfig(port=port, id=arm_id, use_degrees=False, max_relative_target=max_step)
        )
        connect_calibrated(
            self.robot, f"--robot.type=so101_follower --robot.port={port} --robot.id={arm_id}", self.get_logger()
        )
        self.target = None
        self.publisher = self.create_publisher(JointState, contract.TOPIC_STATE, 10)
        self.create_subscription(JointState, contract.TOPIC_ACTION, self.on_action, 10)
        self.create_timer(1.0 / rate_hz, self.tick)
        mode = "ENABLED (arm will move)" if self.enable else "read-only (set enable:=true to move)"
        self.get_logger().info(f"follower on {port}: {mode}, max_relative_target={max_step}, id={arm_id}")

    def on_action(self, msg: JointState) -> None:
        self.target = radians_to_arm_motor(joint_state_to_array(msg), self.roll_offset)

    def tick(self) -> None:
        if self.enable and self.target is not None:
            self.robot.send_action(array_to_motor_dict(self.target))
        observation = self.robot.get_observation()
        radians = arm_motor_to_radians(motor_dict_to_array(observation), self.roll_offset)
        self.publisher.publish(joint_state_msg(self.get_clock().now().to_msg(), radians))

    def destroy_node(self) -> None:
        self.robot.disconnect()
        super().destroy_node()


def main() -> None:
    run(FollowerNode)


if __name__ == "__main__":
    main()
