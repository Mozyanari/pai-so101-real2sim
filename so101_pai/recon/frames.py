"""Keep the sharpest video frame of every `group` consecutive ones, so motion-blurred frames do not reach 3DGRUT.

python -m so101_pai.recon.frames <image dir> <group>      # deletes the other frames in place
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image

ANALYSIS_WIDTH = 800  # sharpness is compared on downscaled grayscale frames


def sharpness(gray: np.ndarray) -> float:
    """Variance of the 4-neighbour Laplacian; blur lowers it."""
    g = gray.astype(np.float32)
    lap = 4 * g[1:-1, 1:-1] - g[:-2, 1:-1] - g[2:, 1:-1] - g[1:-1, :-2] - g[1:-1, 2:]
    return float(lap.var())


def sharpest_per_group(scores: list[float], group: int) -> list[int]:
    """Index of the highest score in each run of `group` consecutive scores (the last run may be shorter)."""
    if group < 1:
        raise ValueError(f"group must be >= 1, got {group}")
    return [start + int(np.argmax(scores[start : start + group])) for start in range(0, len(scores), group)]


def frame_sharpness(path: Path) -> float:
    image = Image.open(path).convert("L")
    height = round(image.height * ANALYSIS_WIDTH / image.width)
    return sharpness(np.asarray(image.resize((ANALYSIS_WIDTH, height))))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("images", type=Path)
    parser.add_argument("group", type=int)
    args = parser.parse_args()
    paths = sorted(args.images.glob("*.jpg"))
    keep = set(sharpest_per_group([frame_sharpness(p) for p in paths], args.group))
    for i, path in enumerate(paths):
        if i not in keep:
            path.unlink()
    print(f"kept {len(keep)} of {len(paths)} frames (sharpest of every {args.group})")


if __name__ == "__main__":
    main()
