"""Connect LeRobot SO-101 arms from ROS nodes, which cannot answer LeRobot's calibration prompt."""

from __future__ import annotations


def connect_calibrated(device, calibrate_args: str, logger) -> None:
    """Connect a LeRobot SO leader/follower without the interactive calibration prompt.

    LeRobot's connect() asks on stdin when the motors disagree with the calibration file, and a ROS node has no
    stdin. Instead: fail with the calibration command if no file exists, or write the file to the motors (what
    pressing ENTER in that prompt does; the arm does not move).
    """
    device.connect(calibrate=False)
    if not device.calibration:
        raise RuntimeError(f"no calibration file; run: .venv/bin/lerobot-calibrate {calibrate_args}")
    if not device.is_calibrated:
        logger.warning("motor calibration differs from the file; writing the file to the motors")
        device.bus.write_calibration(device.calibration)
