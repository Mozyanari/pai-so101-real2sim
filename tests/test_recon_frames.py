import numpy as np
import pytest
from PIL import Image, ImageFilter

from so101_pai.recon.frames import frame_sharpness, sharpest_per_group


def test_sharpest_per_group_picks_the_max_of_each_run():
    assert sharpest_per_group([1, 5, 2, 9, 3, 4, 7], 3) == [1, 3, 6]
    assert sharpest_per_group([1, 2], 1) == [0, 1]
    with pytest.raises(ValueError):
        sharpest_per_group([1], 0)


def test_blur_lowers_sharpness(tmp_path):
    rng = np.random.default_rng(0)
    image = Image.fromarray(rng.integers(0, 255, (120, 160, 3), dtype=np.uint8))
    image.save(tmp_path / "sharp.jpg")
    image.filter(ImageFilter.GaussianBlur(3)).save(tmp_path / "blurred.jpg")
    assert frame_sharpness(tmp_path / "blurred.jpg") < 0.2 * frame_sharpness(tmp_path / "sharp.jpg")
