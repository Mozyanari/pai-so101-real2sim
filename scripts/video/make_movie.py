"""Cut the project video (1-3 min, MP4/H.264, at most 2x with the speed shown) from the clips in outputs/video.

    .venv/bin/python scripts/video/make_movie.py [--out outputs/video/final/pai_final.mp4]

Title cards and captions are drawn with Pillow (so101_pai.dataset.caption_video); the clips come from bag2video
(real and sim evaluations) and sim/tools/orbit_video.py. Every shot states whether it is real or sim, the model,
and its speed (等倍 / 2 倍速).
"""

from __future__ import annotations

import argparse
import itertools
from pathlib import Path

import av
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from so101_pai.dataset.bag2video import Clip
from so101_pai.dataset.caption_video import FONT, draw_caption

W, H, FPS = 1280, 720, 30
VIDEO = Path("outputs/video")
BG = (18, 22, 30)
A_COND = "A：実機のお手本 10 本だけで学習"
B_COND = "B：シミュレーター 225 本で事前学習 → 実機のお手本 10 本で追加学習"


def font(size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(FONT, size)


def frames(name: str, speed: int = 1, start: float = 0.0, end: float | None = None):
    """RGB frames of outputs/video/<name>, from `start` to `end` seconds, keeping every `speed`-th one."""
    with av.open(str(VIDEO / name)) as container:
        for i, frame in enumerate(container.decode(video=0)):
            t = i / FPS
            if t < start or i % speed:
                continue
            if end is not None and t >= end:
                break
            yield frame.to_ndarray(format="rgb24")


def fit(rgb: np.ndarray, w: int = W, h: int = H) -> np.ndarray:
    """`rgb` scaled to fit w x h, centred on the background colour."""
    scale = min(w / rgb.shape[1], h / rgb.shape[0])
    size = (int(rgb.shape[1] * scale), int(rgb.shape[0] * scale))
    canvas = Image.new("RGB", (w, h), BG)
    canvas.paste(Image.fromarray(rgb).resize(size, Image.LANCZOS), ((w - size[0]) // 2, (h - size[1]) // 2))
    return np.asarray(canvas)


def card(title: str, lines: list[str] = (), title_size: int = 46, line_size: int = 30) -> np.ndarray:
    """A text slide: `title` and `lines` centred."""
    image = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(image)
    blocks = [(title, font(title_size), (255, 255, 255))] + [(t, font(line_size), (200, 210, 225)) for t in lines]
    heights = [f.size + (24 if k == 0 else 14) for k, (_, f, _) in enumerate(blocks)]
    y = (H - sum(heights)) // 2
    for (text, f, color), h in zip(blocks, heights, strict=True):
        draw.text(((W - draw.textlength(text, font=f)) // 2, y), text, font=f, fill=color)
        y += h
    return np.asarray(image)


def table_card() -> np.ndarray:
    """The real-robot results of A and B (successes out of 3, same layout)."""
    image = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(image)
    draw.text((90, 90), "実機での結果（同じ配置で各 3 回、成功の数）", font=font(42), fill=(255, 255, 255))
    cols = [90, 740, 870, 1000, 1130]
    rows = [
        (["学習ステップ", "1000", "5000", "10000", "20000"], (170, 180, 195)),
        (["A：実機 10 本だけ", "0/3", "0/3", "0/3", "0/3"], (235, 120, 130)),
        (["B：シミュレーターで事前学習 → 実機 10 本", "1/3", "3/3", "2/3", "2/3"], (110, 190, 235)),
    ]
    for r, (cells, color) in enumerate(rows):
        y = 220 + r * 90
        for c, text in enumerate(cells):
            draw.text((cols[c], y), text, font=font(28 if c == 0 else 34), fill=color)
        draw.line((90, y + 62, 1190, y + 62), fill=(60, 68, 82), width=2)
    note = "試行の回数は少ない（各 3 回、積み木はほぼ同じ場所）ので、成功の数は目安"
    draw.text((90, 520), note, font=font(24), fill=(170, 180, 195))
    return np.asarray(image)


def shot(name: str, lines: list[str], speed: int = 1, start: float = 0.0, end: float | None = None):
    """A full-screen clip with its caption."""
    for rgb in frames(name, speed, start, end):
        yield draw_caption(fit(rgb), lines, font(26))


def side_by_side(header: str, left: tuple[str, list[str]], right: tuple[str, list[str]], speed: int = 2):
    """Two clips next to each other under a header; the shorter one holds its last frame."""
    tile_w, tile_h = W // 2, W // 2 * 3 // 4
    head = Image.new("RGB", (W, H - tile_h), BG)
    ImageDraw.Draw(head).text((24, 22), header, font=font(34), fill=(255, 255, 255))
    head = np.asarray(head)
    last = [None, None]
    for pair in itertools.zip_longest(frames(left[0], speed), frames(right[0], speed)):
        tiles = []
        for k, (rgb, (_, lines)) in enumerate(zip(pair, (left, right), strict=True)):
            last[k] = last[k] if rgb is None else rgb
            tiles.append(draw_caption(fit(last[k], tile_w, tile_h), lines, font(22)))
        yield np.vstack([head, np.hstack(tiles)])


def hold(image: np.ndarray, seconds: float):
    yield from itertools.repeat(image, int(seconds * FPS))


def story():
    yield from hold(
        card(
            "実機の机をシミュレーターに再現して学習したロボットアームは、実機で役に立つか",
            ["SO-101 による積み木の Pick & Place", "Real-to-Sim（3DGS + Isaac Sim）→ 学習（ACT）→ 実機"],
            title_size=32,
        ),
        5,
    )
    yield from hold(
        card(
            "1. 実機の机をシミュレーターに再現",
            ["スマホで撮った動画から 3DGS で机・壁・棚を再現し、Isaac Sim の背景にした"],
        ),
        3.5,
    )
    yield from shot(
        "sim_scene_orbit_pick.mp4",
        [
            "シミュレーター（Isaac Sim）｜等倍",
            "机・壁・棚：3DGS で再現｜ロボット・皿・積み木：3D モデル",
            "腕は自動のお手本（IK）。学習データはこの動きで 225 本集めた",
        ],
    )
    yield from hold(
        card("2. シミュレーターのデータだけで学習（ACT）", ["シミュレーターの中では 20 回中 13 回成功"]), 3.5
    )
    yield from shot(
        "A_sim_act_v5_video_ep1_success.mp4", ["シミュレーター｜ACT C｜等倍", "シミュレーター 225 本だけで学習", "成功"]
    )
    yield from hold(
        card(
            "3. そのまま実機で動かす（ゼロショット）",
            ["近づく → 閉じる → 皿へ運ぶ、という動かし方は出るが", "積み木を追わず、決まった位置で閉じてしまい失敗"],
        ),
        4.5,
    )
    yield from shot(
        "C_ver_01_ep0_fail.mp4",
        ["実機｜ACT C｜2 倍速", "C：シミュレーター 225 本だけで学習（実機のデータなし）", "失敗"],
        speed=2,
    )
    yield from hold(
        card(
            "4. 実機のお手本 10 本で追加学習して比べる",
            [A_COND, B_COND, "同じ配置・同じ設定で、学習の途中のモデルを実機で動かした"],
        ),
        5,
    )
    for steps, a, b, a_res, b_res in (
        ("1000", "A1k_video_ep0_fail.mp4", "B1k_video_ep1_success.mp4", "失敗：積み木まで行くが閉じない", "成功"),
        (
            "5000",
            "A5k_video_ep0_fail.mp4",
            "B5k_video_ep0_success.mp4",
            "失敗：積み木を押してしまう",
            "成功（3 回とも）",
        ),
        ("20000", "A20k_video_ep0_fail.mp4", "B20k_video_ep1_success.mp4", "失敗", "成功"),
    ):
        yield from side_by_side(
            f"実機｜学習 {steps} ステップ｜2 倍速",
            (a, ["A：実機 10 本だけ", a_res]),
            (b, ["B：シミュレーターで事前学習 → 実機 10 本", b_res]),
        )
    yield from hold(table_card(), 7)
    yield from hold(
        card(
            "5. 積み木の場所を変えても追いかけてつかむ", ["B（2 万ステップ）。人が積み木を置き直しながら連続で動かした"]
        ),
        3.5,
    )
    yield from shot(
        "B20k_positions_ep0_success.mp4",
        ["実機｜ACT B（2 万ステップ）｜2 倍速", B_COND, "積み木の場所を手で変えながら連続で実行"],
        speed=2,
        end=90,
    )
    yield from hold(
        card(
            "まとめ",
            [
                "シミュレーターだけの学習では、実機でそのままはできなかった（ゼロショット）",
                "それでも「近づく → つかむ → 運ぶ」という動かし方は身についていた",
                "実機 10 本の少しの追加学習で、実機だけで学習するより早く・安定してできるようになった",
                "→ シミュレーターでの事前学習は、実機のデータが少ないときに役立つ",
            ],
            line_size=28,
        ),
        9,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=VIDEO / "final" / "pai_final.mp4")
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    clip = Clip(args.out, crf=20)
    for rgb in story():
        clip.write(rgb)
    clip.close()
    seconds = clip.frames / FPS
    print(f"{args.out}: {seconds:.1f} s ({clip.frames} frames)")
    if not 60 <= seconds <= 180:
        raise SystemExit(f"the assignment needs 1-3 minutes, this is {seconds:.1f} s")


if __name__ == "__main__":
    main()
