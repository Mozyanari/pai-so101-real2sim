"""Connect LeRobot SO-101 arms from ROS nodes, which cannot answer LeRobot's calibration prompt."""

from __future__ import annotations

import time

import numpy as np

from so101_pai.common import contract

CONNECT_ATTEMPTS = 3

# Neither arm's wrist_roll follows the LeIsaac mapping in contract (checked 11/1 against the sim at the
# same joint angles, front and wrist cameras): its calibrated range is the whole turn (0..4095), so one motor unit
# is 1.8 deg, not 1.6 deg; and the pose it was calibrated at is the sim's wrist_roll = +90 deg, so it is shifted by
# WRIST_ROLL_OFFSET_DEG (-90 deg: same view of the servo horn and plate as the real cameras).
WRIST_ROLL = contract.JOINT_NAMES.index("wrist_roll")
WRIST_ROLL_DEG_PER_UNIT = 360.0 / 200.0
WRIST_ROLL_OFFSET_DEG = -90.0


def connect_calibrated(device, calibrate_args: str, logger) -> None:
    """Connect a LeRobot SO leader/follower without the interactive calibration prompt.

    LeRobot's connect() asks on stdin when the motors disagree with the calibration file, and a ROS node has no
    stdin. Instead: fail with the calibration command if no file exists, or write the file to the motors (what
    pressing ENTER in that prompt does; the arm does not move).
    The Feetech bus sometimes answers with a corrupt status packet while connect() configures the motors, so
    retry a few times, closing the port in between.
    """
    for attempt in range(1, CONNECT_ATTEMPTS + 1):
        try:
            device.connect(calibrate=False)
            break
        except ConnectionError as err:
            if attempt == CONNECT_ATTEMPTS:
                raise
            logger.warning(f"connect attempt {attempt} failed ({err}); retrying")
            if device.bus.is_connected:
                device.bus.disconnect(disable_torque=False)
            time.sleep(0.5)
    if not device.calibration:
        raise RuntimeError(f"no calibration file; run: .venv/bin/lerobot-calibrate {calibrate_args}")
    if not device.is_calibrated:
        logger.warning("motor calibration differs from the file; writing the file to the motors")
        device.bus.write_calibration(device.calibration)


def arm_motor_to_radians(motor: np.ndarray, wrist_roll_offset_deg: float = WRIST_ROLL_OFFSET_DEG) -> np.ndarray:
    """Leader or follower motor values (LeRobot units, JOINT_NAMES order) -> sim joint angles (rad).

    Both arms were calibrated the same way (wrist_roll over the whole turn), and teleop needs one mapping for both:
    the follower then copies the leader's physical pose.
    """
    joints = contract.motor_to_radians(motor)
    roll_deg = float(motor[WRIST_ROLL]) * WRIST_ROLL_DEG_PER_UNIT + wrist_roll_offset_deg
    joints[WRIST_ROLL] = np.radians(roll_deg)
    return joints


def radians_to_arm_motor(joints_rad: np.ndarray, wrist_roll_offset_deg: float = WRIST_ROLL_OFFSET_DEG) -> np.ndarray:
    """Inverse of arm_motor_to_radians."""
    motor = contract.radians_to_motor(joints_rad)
    motor[WRIST_ROLL] = (np.degrees(float(joints_rad[WRIST_ROLL])) - wrist_roll_offset_deg) / WRIST_ROLL_DEG_PER_UNIT
    return motor
