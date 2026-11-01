"""Encode a folder of PNG frames (e.g. sim/tools/orbit_video.py output) into an H.264 MP4.

python -m so101_pai.dataset.frames2video logs/orbit outputs/video/orbit.mp4 [--crf 18]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from so101_pai.dataset.bag2video import Clip


def encode(frames_dir: Path, out: Path, crf: int = 18) -> int:
    """Write every frame_*.png in `frames_dir` (sorted) to `out` at contract.FPS; returns the frame count."""
    out.parent.mkdir(parents=True, exist_ok=True)
    clip = Clip(out, crf=crf)
    for path in sorted(frames_dir.glob("frame_*.png")):
        clip.write(cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB))
    clip.close()
    return clip.frames


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("frames_dir", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--crf", type=int, default=18, help="H.264 quality (lower = better, 18 is near lossless)")
    args = parser.parse_args()
    print(f"{args.out}: {encode(args.frames_dir, args.out, args.crf)} frames")


if __name__ == "__main__":
    main()
