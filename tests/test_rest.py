import numpy as np

from so101_pai.robot.rest import REST_POSE_DEG, rest_trajectory


def test_trajectory_ends_at_rest_and_eases_in():
    start = np.radians([30.0, -20.0, 40.0, 70.0, 10.0, 20.0])
    plan = rest_trajectory(start, fps=30, speed_deg_s=30.0)
    np.testing.assert_allclose(plan[-1], np.radians(REST_POSE_DEG))
    assert len(plan) == 79  # the largest move is shoulder_lift, 78.2 deg -> 2.6 s at 30 fps
    steps = np.abs(np.diff(np.degrees(np.array([start, *plan])), axis=0)).max(axis=1)
    assert steps[0] < steps[len(steps) // 2]  # slow start, fastest midway
    assert steps.max() < 30.0 / 30 * np.pi / 2 + 1e-6  # cosine easing peaks at pi/2 times the mean speed


def test_trajectory_at_rest_is_a_single_step():
    plan = rest_trajectory(np.radians(REST_POSE_DEG), fps=30)
    assert len(plan) == 1


def test_trajectory_can_end_at_another_pose():
    from so101_pai.robot.rest import START_POSE_DEG

    plan = rest_trajectory(np.radians(REST_POSE_DEG), fps=30, target_deg=START_POSE_DEG)
    np.testing.assert_allclose(plan[-1], np.radians(START_POSE_DEG))
