import numpy as np

from so101_pai.common import contract, episode


def test_joint_limits_map_to_motor_limits():
    lo = np.radians([contract.USD_JOINT_LIMITS_DEG[n][0] for n in contract.JOINT_NAMES])
    hi = np.radians([contract.USD_JOINT_LIMITS_DEG[n][1] for n in contract.JOINT_NAMES])
    np.testing.assert_allclose(
        contract.radians_to_motor(lo), [contract.MOTOR_LIMITS[n][0] for n in contract.JOINT_NAMES], atol=1e-4
    )
    np.testing.assert_allclose(
        contract.radians_to_motor(hi), [contract.MOTOR_LIMITS[n][1] for n in contract.JOINT_NAMES], atol=1e-4
    )


def test_round_trip():
    joints = np.radians([10.0, -45.0, 30.0, 5.0, -90.0, 40.0])
    np.testing.assert_allclose(contract.motor_to_radians(contract.radians_to_motor(joints)), joints, atol=1e-5)


def test_episode_messages_round_trip():
    start = episode.EpisodeEvent(episode.START, task="Put the red block on the plate")
    end = episode.EpisodeEvent(episode.END, success=True)
    assert episode.decode(episode.encode(start)) == start
    assert episode.decode(episode.encode(end)) == end


def test_features_cover_both_cameras():
    features = contract.dataset_features()
    assert set(features) == {"observation.state", "action", "observation.images.front", "observation.images.wrist"}
    assert features["action"]["names"][-1] == "gripper.pos"
