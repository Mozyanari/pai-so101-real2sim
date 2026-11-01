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


def test_frames2video_encodes_a_folder_of_pngs(tmp_path):
    import av
    import cv2

    from so101_pai.dataset.frames2video import encode

    for i in range(5):
        cv2.imwrite(str(tmp_path / f"frame_{i:04d}.png"), np.full((64, 96, 3), i * 40, np.uint8))
    out = tmp_path / "clip.mp4"
    assert encode(tmp_path, out) == 5
    with av.open(str(out)) as container:
        assert sum(1 for _ in container.decode(video=0)) == 5


def test_caption_writes_the_text_on_every_kept_frame(tmp_path):
    import av

    from so101_pai.dataset.bag2video import Clip
    from so101_pai.dataset.caption_video import caption

    src = Clip(tmp_path / "in.mp4")
    for _ in range(10):
        src.write(np.zeros((H, W, 3), np.uint8))
    src.close()
    assert caption(tmp_path / "in.mp4", tmp_path / "out.mp4", ["実機｜試行 1：成功", "2倍速"], speed=2) == 5
    with av.open(str(tmp_path / "out.mp4")) as container:
        frame = next(container.decode(video=0)).to_ndarray(format="rgb24")
    assert frame[: H // 8, : W // 4].max() > 200  # white text in the top-left box
    assert frame[H // 2 :, W // 2 :].max() < 30  # the rest is untouched


def test_caption_font_shrinks_to_fit_a_long_line():
    from so101_pai.dataset.caption_video import load_font

    long_line = "B：シミュレーター 225 本で事前学習 → 実機のお手本 10 本で追加学習（20000 ステップ）"
    font = load_font(H, W, [long_line])
    assert font.getlength(long_line) + font.size <= W
    assert load_font(H, W, ["短い"]).size == max(12, H // 26)
