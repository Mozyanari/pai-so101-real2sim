import json

import numpy as np
import pytest

from so101_pai.recon import align

LENGTH, WIDTH, HEIGHT = 2.0, 1.0, 0.8  # reconstruction units
BASE = np.array([-0.7, 0.25])  # (long, short) from the table center: near the -long short edge and the +short long edge
WOOD, WHITE, GRAY = (160, 100, 60), (230, 230, 230), (120, 120, 120)


def _random_rotation(rng):
    q = rng.normal(size=4)
    return align._quat_to_matrix(q / np.linalg.norm(q))


@pytest.fixture
def scene():
    """A table, a floor, a white follower and a dark leader, under a random similarity transform."""
    rng = np.random.default_rng(3)
    parts = []

    def add(xyz, color):
        parts.append((np.asarray(xyz), np.tile(color, (len(xyz), 1))))

    top = rng.uniform([-LENGTH / 2, -WIDTH / 2], [LENGTH / 2, WIDTH / 2], size=(4000, 2))
    add(np.column_stack([top, rng.normal(0, 0.003, 4000)]), WOOD)
    shelf = rng.uniform([1.5, -0.3], [2.0, 0.3], size=(400, 2))  # coplanar wood off the table
    add(np.column_stack([shelf, np.zeros(400)]), WOOD)
    floor = rng.uniform(-3, 3, size=(3000, 2))
    add(np.column_stack([floor, np.full(3000, -HEIGHT)]), GRAY)
    for center, color in ((BASE, WHITE), (np.array([0.75, 0.0]), (40, 40, 40))):
        arm = np.column_stack([center + rng.normal(0, 0.03, (300, 2)), rng.uniform(0.0, 0.5, 300)])
        add(arm, color)
    points = np.vstack([p for p, _ in parts])
    colors = np.vstack([c for _, c in parts]).astype(float)
    cameras = np.column_stack([rng.uniform(-1, 1, (20, 2)), rng.uniform(1.0, 1.5, 20)])

    rot, scale, shift = _random_rotation(rng), 3.7, rng.normal(size=3)
    to_recon = lambda p: shift + scale * p @ rot.T  # noqa: E731
    return to_recon(points), colors, to_recon(cameras), to_recon


def test_fit_table_recovers_size_floor_and_up(scene):
    points, colors, cameras, to_recon = scene
    table = align.fit_table(points, colors, cameras)
    np.testing.assert_allclose(table.size / 3.7, [LENGTH, WIDTH], rtol=0.03)
    assert table.floor_height / 3.7 == pytest.approx(-HEIGHT, abs=0.03)
    up = to_recon(np.array([[0, 0, 1.0]])) - to_recon(np.zeros((1, 3)))
    assert table.normal @ (up[0] / np.linalg.norm(up)) > 0.999


def test_follower_is_the_lighter_arm(scene):
    points, colors, cameras, _ = scene
    table = align.fit_table(points, colors, cameras)
    base = align.find_follower_base(table, points, colors)
    # The table's long axis may point either way; the base is the white arm's footprint.
    assert np.abs(base) / 3.7 == pytest.approx(np.abs(BASE), abs=0.03)


@pytest.mark.parametrize("facing", align.FACINGS)
def test_alignment_puts_base_at_origin_table_at_zero_and_forward_on_y(scene, facing):
    points, colors, cameras, to_recon = scene
    table = align.fit_table(points, colors, cameras)
    base = align.find_follower_base(table, points, colors)
    forward = align.forward_direction(base, facing)
    scale = 0.5 / table.size[0]  # a 2-unit table is 0.5 m
    result = align.alignment(table, base, forward, scale)

    base_world = table.center + base[0] * table.axes[0] + base[1] * table.axes[1]
    np.testing.assert_allclose(align.apply(result, base_world[None])[0], 0.0, atol=1e-9)
    corners = to_recon(np.array([[x, y, 0.0] for x in (-1, 1) for y in (-0.5, 0.5)]))
    np.testing.assert_allclose(align.apply(result, corners)[:, 2], 0.0, atol=0.005)
    ahead = base_world + 0.2 * (forward[0] * table.axes[0] + forward[1] * table.axes[1])
    direction = align.apply(result, ahead[None])[0]
    assert direction[1] > 0 and abs(direction[0]) < 1e-9
    assert result["scale"] == pytest.approx(scale)
    sim_corners = align.apply(result, corners)[:, :2]
    np.testing.assert_allclose(result["table"]["center"], sim_corners.mean(axis=0), atol=1e-3)
    np.testing.assert_allclose(result["table"]["size"], np.ptp(sim_corners, axis=0), rtol=0.06)
    json.dumps(result)


def test_forward_direction_points_into_the_table():
    base = np.array([-0.7, 0.25])
    assert align.forward_direction(base, "along").tolist() == [1.0, 0.0]
    assert align.forward_direction(base, "across-far").tolist() == [0.0, -1.0]
    assert align.forward_direction(base, "across-near").tolist() == [0.0, 1.0]
    with pytest.raises(ValueError):
        align.forward_direction(base, "sideways")


def test_base_from_edges_uses_the_nearest_edges(scene):
    points, colors, cameras, _ = scene
    table = align.fit_table(points, colors, cameras)
    half = table.size / 2
    base = align.base_from_edges(table, np.array([-0.1, 0.2]), from_short=0.3, from_long=0.1)
    np.testing.assert_allclose(base, [-(half[0] - 0.3), half[1] - 0.1])


def test_quaternion_round_trip():
    rot = _random_rotation(np.random.default_rng(0))
    np.testing.assert_allclose(align._quat_to_matrix(align._quat_wxyz(rot)), rot, atol=1e-12)
