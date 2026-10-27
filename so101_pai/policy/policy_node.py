"""Drive the robot (sim or real) with a trained LeRobot policy through the ROS 2 contract.

    python -m so101_pai.policy.policy_node --ros-args -p policy_path:=outputs/act/checkpoints/last/pretrained_model

Subscribes to state, both cameras and /episode/status; while an episode is running it publishes actions on
contract.TOPIC_ACTION, the same topic the leader arm uses, so the sim bridge and follower node need no changes.
Do not run the leader node at the same time.
"""

from __future__ import annotations

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

from so101_pai.common import contract, episode
from so101_pai.common.ros_msgs import image_to_rgb, joint_state_msg, joint_state_to_array
from so101_pai.common.ros_node import run
from so101_pai.policy.runner import ObservationBuffer, load_policy

DEFAULT_TASK = "Put the red block on the plate."


class PolicyNode(Node):
    def __init__(self, **node_kwargs) -> None:
        # node_kwargs (e.g. parameter_overrides) let tests build the node without the command line.
        super().__init__("so101_policy", **node_kwargs)
        path = self.declare_parameter("policy_path", "").value
        if not path:
            raise ValueError("set -p policy_path:=<checkpoint dir or Hub id>")
        device = self.declare_parameter("device", "cuda").value
        task = self.declare_parameter("task", DEFAULT_TASK).value
        rate_hz = self.declare_parameter("rate_hz", float(contract.FPS)).value

        self.runner = load_policy(path, device, task)
        self.buffer = ObservationBuffer()
        self.active = False

        self.action_pub = self.create_publisher(JointState, contract.TOPIC_ACTION, 10)
        self.create_subscription(JointState, contract.TOPIC_STATE, self.on_state, 10)
        self.create_subscription(String, contract.TOPIC_EPISODE, self.on_status, 10)
        for camera, topic in contract.CAMERA_TOPICS.items():
            self.create_subscription(Image, topic, lambda msg, camera=camera: self.on_image(camera, msg), 10)
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(f"policy {path} on {device}: waiting for an episode start")

    def on_state(self, msg: JointState) -> None:
        self.buffer.state = contract.radians_to_motor(joint_state_to_array(msg))

    def on_image(self, camera: str, msg: Image) -> None:
        self.buffer.images[camera] = image_to_rgb(msg)

    def on_status(self, msg: String) -> None:
        event = episode.decode(msg.data)
        if event.event == episode.START:
            self.runner.reset()
        self.active = event.event == episode.START
        self.get_logger().info(f"episode {event.event}: policy {'running' if self.active else 'idle'}")

    def tick(self) -> None:
        if not (self.active and self.buffer.ready()):
            return
        motor = self.runner.act(self.buffer.frame())
        radians = contract.motor_to_radians(np.asarray(motor))
        self.action_pub.publish(joint_state_msg(self.get_clock().now().to_msg(), radians))


def main() -> None:
    run(PolicyNode)


if __name__ == "__main__":
    main()
