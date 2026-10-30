import numpy as np
import pytest

from so101_pai.recon.align import _quat_wxyz, apply
from so101_pai.recon.crop import inside_boxes, parse_box, parse_patch, sim_offset_to_recon


def test_parse_box():
    lo, hi = parse_box("-0.1,-0.2,0.0,0.1,0.2,0.3")
    assert lo.tolist() == [-0.1, -0.2, 0.0] and hi.tolist() == [0.1, 0.2, 0.3]
    with pytest.raises(ValueError):
        parse_box("0,0,0,1,1")
    with pytest.raises(ValueError):
        parse_box("1,0,0,0,1,1")


def test_inside_boxes_is_a_union():
    points = np.array([[0.0, 0.0, 0.1], [0.5, 0.0, 0.1], [0.0, 0.0, -0.1], [2.0, 2.0, 2.0]])
    boxes = [parse_box("-0.1,-0.1,0,0.1,0.1,0.2"), parse_box("0.4,-0.1,0,0.6,0.1,0.2")]
    assert inside_boxes(points, boxes).tolist() == [True, True, False, False]


def test_parse_patch():
    (lo, hi), offset = parse_patch("0,0,0,1,1,1:-0.2,0,0.01")
    assert lo.tolist() == [0, 0, 0] and hi.tolist() == [1, 1, 1] and offset.tolist() == [-0.2, 0, 0.01]
    with pytest.raises(ValueError):
        parse_patch("0,0,0,1,1,1:1,2")


def test_sim_offset_to_recon_round_trips_through_apply():
    yaw = np.radians(30)
    rot = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1.0]])
    align = {"pos": [0.1, 0.2, 0.3], "rot": _quat_wxyz(rot).tolist(), "scale": 0.18}
    p = np.array([[1.0, 2.0, 3.0]])
    offset = np.array([-0.23, 0.05, 0.0])
    moved = apply(align, p + sim_offset_to_recon(align, offset))
    np.testing.assert_allclose(moved - apply(align, p), offset[None], atol=1e-12)
