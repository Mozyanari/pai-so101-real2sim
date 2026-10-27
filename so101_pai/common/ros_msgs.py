"""Conversions between ROS 2 messages and the arrays used by the contract, shared by every node and the converter."""

from __future__ import annotations

import cv2
import numpy as np
from sensor_msgs.msg import JointState

from so101_pai.common import contract


def motor_dict_to_array(values: dict[str, float]) -> np.ndarray:
    """LeRobot action/observation dict ({'shoulder_pan.pos': v, ...}) -> array in JOINT_NAMES order."""
    return np.array([values[name] for name in contract.FEATURE_NAMES], dtype=np.float64)


def array_to_motor_dict(values: np.ndarray) -> dict[str, float]:
    return {name: float(v) for name, v in zip(contract.FEATURE_NAMES, values, strict=True)}


def joint_state_msg(stamp, radians: np.ndarray) -> JointState:
    msg = JointState()
    msg.header.stamp = stamp
    msg.name = list(contract.JOINT_NAMES)
    msg.position = [float(v) for v in radians]
    return msg


def joint_state_to_array(msg) -> np.ndarray:
    """Reorder a sensor_msgs/JointState into contract.JOINT_NAMES order (radians)."""
    by_name = dict(zip(msg.name, msg.position, strict=True))
    missing = [name for name in contract.JOINT_NAMES if name not in by_name]
    if missing:
        raise ValueError(f"JointState is missing joints: {missing}")
    return np.array([by_name[name] for name in contract.JOINT_NAMES], dtype=np.float64)


def image_to_rgb(msg) -> np.ndarray:
    """Decode a sensor_msgs/Image (rgb8/bgr8/rgba8/bgra8) into an HxWx3 RGB uint8 array at contract size."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    image = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)[:, :, :3]
    if msg.encoding.startswith("bgr"):
        image = image[:, :, ::-1]
    if image.shape[:2] != (contract.IMAGE_HEIGHT, contract.IMAGE_WIDTH):
        image = cv2.resize(image, (contract.IMAGE_WIDTH, contract.IMAGE_HEIGHT), interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(image)
