"""Publish a USB camera as sensor_msgs/Image (rgb8) at the contract resolution.

python -m so101_pai.robot.camera_node --ros-args -p device:=/dev/video0 -p camera:=front
"""

from __future__ import annotations

import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image

from so101_pai.common import contract
from so101_pai.common.ros_node import run


class CameraNode(Node):
    def __init__(self, **node_kwargs) -> None:
        # node_kwargs (e.g. parameter_overrides) let tests build the node without the command line.
        super().__init__("camera", **node_kwargs)
        device = self.declare_parameter("device", "/dev/video0").value
        camera = self.declare_parameter("camera", "front").value
        if camera not in contract.CAMERA_TOPICS:
            raise ValueError(f"camera must be one of {list(contract.CAMERA_TOPICS)}")

        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        # MJPG: the wrist USB camera returned solid-green frames in its default YUYV mode (checked 10/31).
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, contract.IMAGE_WIDTH)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, contract.IMAGE_HEIGHT)
        self.capture.set(cv2.CAP_PROP_FPS, contract.FPS)
        if not self.capture.isOpened():
            raise RuntimeError(f"cannot open camera {device}")
        self.publisher = self.create_publisher(Image, contract.CAMERA_TOPICS[camera], 10)
        self.create_timer(1.0 / contract.FPS, self.tick)
        self.get_logger().info(f"{device} -> {contract.CAMERA_TOPICS[camera]}")

    def tick(self) -> None:
        ok, bgr = self.capture.read()
        if not ok:
            self.get_logger().warning("frame grab failed", throttle_duration_sec=2.0)
            return
        if bgr.shape[:2] != (contract.IMAGE_HEIGHT, contract.IMAGE_WIDTH):
            bgr = cv2.resize(bgr, (contract.IMAGE_WIDTH, contract.IMAGE_HEIGHT), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding, msg.step = "rgb8", rgb.shape[1] * 3
        msg.data = rgb.tobytes()
        self.publisher.publish(msg)

    def destroy_node(self) -> None:
        self.capture.release()
        super().destroy_node()


def main() -> None:
    run(CameraNode)


if __name__ == "__main__":
    main()
