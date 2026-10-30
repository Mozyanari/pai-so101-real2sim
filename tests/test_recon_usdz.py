import pytest

from so101_pai.recon.usdz import IDENTITY, reset_volume_transform

LAYER = """def Volume "gauss"
{
    matrix4d xformOp:transform = ( (0.7, 0.6, 0, 0.04), (-0.6, 0.7, 0, 0), (0, 0, 1, -0.2), (0, 0, 0, 1) )
    uniform token[] xformOpOrder = ["xformOp:transform"]
}
def Camera "camera_0"
{
    matrix4d xformOp:transform.timeSamples = {
        0: ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (3, 4, 5, 1) ),
    }
}
"""


def test_reset_volume_transform_only_touches_the_volume():
    out = reset_volume_transform(LAYER)
    assert f"matrix4d xformOp:transform = {IDENTITY}\n" in out
    assert "0.7, 0.6" not in out
    assert "(3, 4, 5, 1)" in out  # camera time samples untouched


def test_reset_volume_transform_requires_a_transform():
    with pytest.raises(ValueError):
        reset_volume_transform('def Volume "gauss" {}\n')
