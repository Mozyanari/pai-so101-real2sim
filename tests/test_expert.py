import math

import numpy as np
import pytest

from so101_pai.teleop import expert


def test_jaw_heading_picks_the_face_normal_nearest_the_radial_direction():
    assert expert.jaw_heading(0.0, math.radians(80)) == pytest.approx(math.pi / 2)
    assert expert.jaw_heading(math.radians(30), math.radians(100)) == pytest.approx(math.radians(120))
    assert expert.jaw_heading(math.radians(-40), math.radians(90)) == pytest.approx(math.radians(50))


def test_top_down_rotation_is_proper_and_points_down():
    rot = expert.top_down_rotation(0.7)
    np.testing.assert_allclose(rot.T @ rot, np.eye(3), atol=1e-12)
    assert np.linalg.det(rot) == pytest.approx(1.0)
    np.testing.assert_allclose(rot @ expert.TCP_OFFSET, [0, 0, expert.TCP_OFFSET[2]], atol=1e-12)


def test_plan_grasps_the_block_and_releases_over_the_plate():
    block, plate = np.array([-0.08, 0.17, 0.0235]), np.array([0.15, 0.18, 0.0])
    plan = expert.plan_pick_place(block, 0.3, plate, rng=np.random.default_rng(0))
    closes = [i for i, w in enumerate(plan) if w.gripper == expert.GRIPPER_CLOSED]
    grasp = plan[closes[0]]
    # The fixed finger (TCP) stands just outside the block face; the jaw axis points at the block center.
    reach = (expert.BLOCK_HALF + expert.FINGER_CLEARANCE) * grasp.rot[:, 0]
    np.testing.assert_allclose(grasp.pos[:2] + reach[:2], block[:2], atol=0.004 + 1e-12)
    assert grasp.pos[2] == pytest.approx(expert.BLOCK_HALF)
    release = plan[closes[-1] + 1]
    assert release.gripper == expert.GRIPPER_OPEN
    held_block = release.pos + expert.BLOCK_HALF * release.rot[:, 0]
    assert np.abs(held_block[:2] - plate[:2]).max() <= 0.02 + 1e-12
    assert plan[-1].pos[2] > release.pos[2]
    assert 6.0 < expert.plan_duration(plan) < 12.0


def test_plans_vary_with_the_rng_and_repeat_with_the_seed():
    args = (np.array([-0.08, 0.17, 0.0235]), 0.3, np.array([0.15, 0.18, 0.0]))
    a, b = (expert.plan_pick_place(*args, rng=np.random.default_rng(seed)) for seed in (1, 2))
    again = expert.plan_pick_place(*args, rng=np.random.default_rng(1))
    assert expert.plan_duration(a) != expert.plan_duration(b)
    assert not np.allclose(a[0].pos, b[0].pos)
    assert all(np.allclose(x.pos, y.pos) for x, y in zip(a, again, strict=True))


def test_sample_is_continuous_across_waypoints():
    plan = expert.plan_pick_place(np.array([0.0, 0.2, 0.0]), 0.0, np.array([0.15, 0.18, 0.0]))
    start = expert.Waypoint(np.array([0.0, 0.1, 0.25]), np.eye(3), 0.0, 0.0)
    assert expert.sample(plan, start, 0.0).pos == pytest.approx(start.pos)
    boundary = plan[0].duration
    before, after = expert.sample(plan, start, boundary - 1e-6), expert.sample(plan, start, boundary + 1e-6)
    np.testing.assert_allclose(before.pos, after.pos, atol=1e-5)
    np.testing.assert_allclose(before.rot, after.rot, atol=1e-5)
    end = expert.sample(plan, start, 100.0)
    np.testing.assert_allclose(end.pos, plan[-1].pos)


@pytest.mark.parametrize("vec", [[0.1, -0.2, 0.3], [0.0, 0.0, 3.0], [math.pi, 0.0, 0.0]])
def test_rotation_vector_round_trip(vec):
    rot = expert.rotation_from_vector(np.array(vec))
    np.testing.assert_allclose(expert.rotation_from_vector(expert.rotation_vector(rot)), rot, atol=1e-9)


def test_pose_error_and_dls_reach_a_target_on_a_linear_system():
    rng = np.random.default_rng(1)
    jacobian = rng.normal(size=(6, 5))
    target = jacobian @ rng.normal(size=5) * 0.01
    step = expert.damped_least_squares(jacobian, target, damping=1e-4)
    np.testing.assert_allclose(jacobian @ step, target, atol=1e-6)
    np.testing.assert_allclose(expert.pose_error(np.zeros(3), np.eye(3), np.zeros(3), np.eye(3)), np.zeros(6))


def test_tcp_jacobian_adds_the_lever_arm():
    # A pure rotation about z moves a point 1 m along x in +y.
    jacobian = np.zeros((6, 1))
    jacobian[5, 0] = 1.0
    tcp = expert.tcp_jacobian(jacobian, np.array([1.0, 0.0, 0.0]))
    np.testing.assert_allclose(tcp[:3, 0], [0.0, 1.0, 0.0])


def test_smooth_noise_is_zero_mean_with_the_given_std_and_smooth():
    sigma = np.array([0.03, 0.0])
    noise = expert.SmoothNoise(sigma, tau=0.5, dt=1 / 30, rng=np.random.default_rng(0))
    samples = np.array([noise.step() for _ in range(30000)])
    assert np.all(samples[:, 1] == 0.0)
    assert abs(samples[:, 0].mean()) < 0.005
    assert samples[:, 0].std() == pytest.approx(0.03, rel=0.1)
    assert np.diff(samples[:, 0]).std() < 0.4 * 0.03  # steps are much smaller than the excursions
    noise.reset()
    assert np.all(noise.value == 0.0)
