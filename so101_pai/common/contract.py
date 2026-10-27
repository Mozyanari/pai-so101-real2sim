"""ROS 2 topic contract shared by the sim, the real robot, the converter and the policy node.

All joint values on ROS topics are radians in the simulator's joint space (the LeIsaac SO-101 USD).
Datasets store LeRobot's normalized motor units, so policies pretrained on real SO-101 data
(e.g. SmolVLA) see the units they were trained with.
"""

from __future__ import annotations

import numpy as np

FPS = 30
IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640

TOPIC_STATE = "/joint_states"
TOPIC_ACTION = "/leader/joint_states"
TOPIC_EPISODE = "/episode/status"
# Operator commands to the sim bridge (std_msgs/String): "start", "fail", "reset".
TOPIC_EPISODE_COMMAND = "/episode/command"
CAMERA_TOPICS = {
    "front": "/cam_front/image_raw",
    "wrist": "/cam_wrist/image_raw",
}

JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
FEATURE_NAMES = tuple(f"{name}.pos" for name in JOINT_NAMES)

# Joint limits of the SO-101 USD in degrees and the matching LeRobot motor ranges.
# Values and the linear mapping are taken from LeIsaac (Apache-2.0):
# source/leisaac/leisaac/assets/robots/lerobot.py and utils/robot_utils.py
USD_JOINT_LIMITS_DEG = {
    "shoulder_pan": (-110.0, 110.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-100.0, 90.0),
    "wrist_flex": (-95.0, 95.0),
    "wrist_roll": (-160.0, 160.0),
    "gripper": (-10.0, 100.0),
}
MOTOR_LIMITS = {
    "shoulder_pan": (-100.0, 100.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-100.0, 100.0),
    "wrist_flex": (-100.0, 100.0),
    "wrist_roll": (-100.0, 100.0),
    "gripper": (0.0, 100.0),
}

_JOINT_LO = np.array([USD_JOINT_LIMITS_DEG[n][0] for n in JOINT_NAMES])
_JOINT_HI = np.array([USD_JOINT_LIMITS_DEG[n][1] for n in JOINT_NAMES])
_MOTOR_LO = np.array([MOTOR_LIMITS[n][0] for n in JOINT_NAMES])
_MOTOR_HI = np.array([MOTOR_LIMITS[n][1] for n in JOINT_NAMES])


def radians_to_motor(joints_rad: np.ndarray) -> np.ndarray:
    """Map joint angles (rad, ordered as JOINT_NAMES) to LeRobot normalized motor units."""
    degrees = np.degrees(np.asarray(joints_rad, dtype=np.float64))
    ratio = (degrees - _JOINT_LO) / (_JOINT_HI - _JOINT_LO)
    return (ratio * (_MOTOR_HI - _MOTOR_LO) + _MOTOR_LO).astype(np.float32)


def motor_to_radians(motor: np.ndarray) -> np.ndarray:
    """Inverse of radians_to_motor."""
    ratio = (np.asarray(motor, dtype=np.float64) - _MOTOR_LO) / (_MOTOR_HI - _MOTOR_LO)
    return np.radians(ratio * (_JOINT_HI - _JOINT_LO) + _JOINT_LO).astype(np.float32)


def dataset_features(use_videos: bool = True) -> dict:
    """LeRobot feature spec for datasets recorded under this contract."""
    joint_feature = {"dtype": "float32", "shape": (len(JOINT_NAMES),), "names": list(FEATURE_NAMES)}
    image_dtype = "video" if use_videos else "image"
    features = {"observation.state": dict(joint_feature), "action": dict(joint_feature)}
    for camera in CAMERA_TOPICS:
        features[f"observation.images.{camera}"] = {
            "dtype": image_dtype,
            "shape": (IMAGE_HEIGHT, IMAGE_WIDTH, 3),
            "names": ["height", "width", "channels"],
        }
    return features
