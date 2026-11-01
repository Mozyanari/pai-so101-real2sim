import logging

import numpy as np
import pytest

from so101_pai.robot.calibration import connect_calibrated


class FakeBus:
    def __init__(self) -> None:
        self.written = None
        self.is_connected = False
        self.disconnects = 0

    def disconnect(self, disable_torque: bool = True) -> None:
        self.is_connected = False
        self.disconnects += 1

    def write_calibration(self, calibration) -> None:
        self.written = calibration


class FakeDevice:
    def __init__(self, calibration, is_calibrated: bool, failures: int = 0) -> None:
        self.calibration = calibration
        self.is_calibrated = is_calibrated
        self.bus = FakeBus()
        self.connect_args = None
        self.failures = failures

    def connect(self, calibrate: bool = True) -> None:
        self.connect_args = calibrate
        self.bus.is_connected = True
        if self.failures:
            self.failures -= 1
            raise ConnectionError("Incorrect status packet!")


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


def test_connect_retries_transient_bus_errors(monkeypatch):
    monkeypatch.setattr("so101_pai.robot.calibration.time.sleep", lambda _: None)
    device = FakeDevice({"shoulder_pan": 1}, is_calibrated=True, failures=2)
    connect_calibrated(device, "--args", LOGGER)
    assert device.bus.disconnects == 2 and device.bus.is_connected


def test_connect_gives_up_after_repeated_errors(monkeypatch):
    monkeypatch.setattr("so101_pai.robot.calibration.time.sleep", lambda _: None)
    with pytest.raises(ConnectionError):
        connect_calibrated(FakeDevice({"shoulder_pan": 1}, is_calibrated=True, failures=3), "--args", LOGGER)


def test_arm_wrist_roll_is_a_full_turn_shifted_by_the_offset():
    from so101_pai.common import contract
    from so101_pai.robot.calibration import arm_motor_to_radians

    motor = np.array([10.0, -20.0, 30.0, 40.0, 0.0, 50.0])
    rad = arm_motor_to_radians(motor, wrist_roll_offset_deg=-90.0)
    assert np.degrees(rad[4]) == pytest.approx(-90.0)
    assert np.degrees(arm_motor_to_radians(np.array([0, 0, 0, 0, 50.0, 0]), -90.0)[4]) == pytest.approx(0.0)
    np.testing.assert_allclose(np.delete(rad, 4), np.delete(contract.motor_to_radians(motor), 4), rtol=1e-6)


def test_arm_mapping_round_trips():
    from so101_pai.robot.calibration import arm_motor_to_radians, radians_to_arm_motor

    motor = np.array([10.0, -20.0, 30.0, 40.0, -35.0, 50.0])
    back = radians_to_arm_motor(arm_motor_to_radians(motor, -90.0), -90.0)
    np.testing.assert_allclose(back, motor, atol=1e-3)
