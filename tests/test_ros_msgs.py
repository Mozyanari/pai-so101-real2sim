import numpy as np
import pytest
from builtin_interfaces.msg import Time

from so101_pai.common import contract
from so101_pai.common.ros_msgs import array_to_motor_dict, joint_state_msg, joint_state_to_array, motor_dict_to_array


def test_motor_dict_round_trip_and_joint_state_reorder():
    values = np.arange(6, dtype=np.float64)
    assert np.array_equal(motor_dict_to_array(array_to_motor_dict(values)), values)
    msg = joint_state_msg(Time(sec=1), values)
    msg.name = list(reversed(msg.name))
    msg.position = list(reversed(msg.position))
    assert np.array_equal(joint_state_to_array(msg), values)
    assert tuple(reversed(msg.name)) == contract.JOINT_NAMES


def test_joint_state_missing_joint_is_an_error():
    msg = joint_state_msg(Time(sec=1), np.zeros(6))
    msg.name, msg.position = msg.name[:5], msg.position[:5]
    with pytest.raises(ValueError, match="gripper"):
        joint_state_to_array(msg)
