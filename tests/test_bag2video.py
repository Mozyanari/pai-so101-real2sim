"""bag2video: frame layouts, and per-episode clips from a synthetic rosbag2 (the one bag2lerobot's test writes)."""

import numpy as np
import pytest

from so101_pai.common import contract

H, W = contract.IMAGE_HEIGHT, contract.IMAGE_WIDTH


def test_inset_puts_the_wrist_view_in_the_bottom_right_corner():
    from so101_pai.dataset.bag2video import compose

    front = np.zeros((H, W, 3), np.uint8)
    wrist = np.full((H, W, 3), 200, np.uint8)
    out = compose(front, wrist, "inset")
    assert out.shape == front.shape
    assert out[H - 20, W - 20, 0] == 200 and out[20, 20, 0] == 0
    assert compose(front, wrist, "side").shape == (H, 2 * W, 3)
    with pytest.raises(ValueError):
        compose(front, wrist, "grid")


def test_convert_writes_one_clip_per_episode_named_by_its_result(tmp_path):
    pytest.importorskip("rosbag2_py")
    import av
    from test_bag2lerobot import write_bag

    from so101_pai.dataset.bag2video import convert

    bag = tmp_path / "bag"
    write_bag(bag)  # episode 0 succeeds and episode 1 fails, 1 s each
    clips = convert(bag, tmp_path / "video")
    assert [p.name for p in clips] == ["bag_ep0_success.mp4", "bag_ep1_fail.mp4"]
    with av.open(str(clips[0])) as container:
        frames = sum(1 for _ in container.decode(video=0))
    assert abs(frames - contract.FPS) <= 1

    whole = convert(bag, tmp_path / "whole", whole=True)
    assert [p.name for p in whole] == ["bag.mp4"]
