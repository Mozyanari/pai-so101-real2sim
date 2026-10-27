import logging

import pytest

from so101_pai.robot.calibration import connect_calibrated


class FakeBus:
    def __init__(self) -> None:
        self.written = None

    def write_calibration(self, calibration) -> None:
        self.written = calibration


class FakeDevice:
    def __init__(self, calibration, is_calibrated: bool) -> None:
        self.calibration = calibration
        self.is_calibrated = is_calibrated
        self.bus = FakeBus()
        self.connect_args = None

    def connect(self, calibrate: bool = True) -> None:
        self.connect_args = calibrate


LOGGER = logging.getLogger("test")


def test_connect_never_prompts_and_writes_file_on_mismatch():
    device = FakeDevice({"shoulder_pan": 1}, is_calibrated=False)
    connect_calibrated(device, "--args", LOGGER)
    assert device.connect_args is False
    assert device.bus.written == {"shoulder_pan": 1}


def test_connect_leaves_matching_motors_alone():
    device = FakeDevice({"shoulder_pan": 1}, is_calibrated=True)
    connect_calibrated(device, "--args", LOGGER)
    assert device.bus.written is None


def test_connect_without_file_explains_how_to_calibrate():
    with pytest.raises(RuntimeError, match="lerobot-calibrate --teleop.id=x"):
        connect_calibrated(FakeDevice({}, is_calibrated=False), "--teleop.id=x", LOGGER)
