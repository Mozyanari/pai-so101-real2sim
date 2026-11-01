"""Write a caption (the run's conditions, in Japanese) on every frame of an MP4, optionally sped up.

    python -m so101_pai.dataset.caption_video IN.mp4 OUT.mp4 --text "1行目" --text "2行目" [--speed 2]

OpenCV's putText has no CJK glyphs, so the text is drawn with Pillow and a Noto Sans CJK font. --speed N keeps every
N-th frame (the assignment allows up to 2x if the video says so; add that to the text).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from so101_pai.dataset.bag2video import Clip

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"


def load_font(height: int, width: int, lines: list[str]) -> ImageFont.FreeTypeFont:
    """A CJK font sized for frames `height` pixels tall, smaller if the longest line would not fit in `width`
    (falls back to Pillow's default without Noto)."""
    size = max(12, height // 26)
    try:
        font = ImageFont.truetype(FONT, size)
    except OSError:
        return ImageFont.load_default(size)
    longest = max(font.getlength(line) for line in lines) + size  # + the box padding
    if longest > width:
        font = ImageFont.truetype(FONT, max(10, int(size * width / longest)))
    return font


def draw_caption(rgb: np.ndarray, lines: list[str], font: ImageFont.FreeTypeFont) -> np.ndarray:
    """`lines` in the top-left corner, white on a translucent dark box."""
    image = Image.fromarray(rgb).convert("RGBA")
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    pad, gap = font.size // 2, font.size // 4
    widths = [draw.textbbox((0, 0), line, font=font)[2] for line in lines]
    box_h = pad * 2 + len(lines) * font.size + (len(lines) - 1) * gap
    draw.rectangle((0, 0, max(widths, default=0) + pad * 2, box_h), fill=(0, 0, 0, 160))
    for k, line in enumerate(lines):
        draw.text((pad, pad + k * (font.size + gap)), line, font=font, fill=(255, 255, 255, 255))
    return np.asarray(Image.alpha_composite(image, overlay).convert("RGB"))


def caption(src: Path, out: Path, lines: list[str], speed: int = 1, crf: int = 20) -> int:
    """Re-encode `src` to `out` with `lines` on every frame, keeping every `speed`-th frame; returns the frame count."""
    import av

    out.parent.mkdir(parents=True, exist_ok=True)
    clip = Clip(out, crf=crf)
    font = None
    with av.open(str(src)) as container:
        for i, frame in enumerate(container.decode(video=0)):
            if i % speed:
                continue
            rgb = frame.to_ndarray(format="rgb24")
            font = font or load_font(rgb.shape[0], rgb.shape[1], lines)
            clip.write(draw_caption(rgb, lines, font))
    clip.close()
    return clip.frames


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("src", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--text", action="append", required=True, help="one caption line (repeat for more)")
    parser.add_argument("--speed", type=int, default=1, help="keep every N-th frame")
    args = parser.parse_args()
    frames = caption(args.src, args.out, args.text, args.speed)
    print(f"{args.out} ({frames} frames)")


if __name__ == "__main__":
    main()
