"""Make a 3DGRUT NuRec USDZ live in COLMAP's frame.

3DGRUT writes the Gaussian positions in COLMAP coordinates but also authors a "normalizing" transform (camera mean
at the origin, mean down-vector rotated onto an axis) on the NuRec volume, which Isaac Sim applies. Alignment
(`so101_pai.recon.align`) is computed in COLMAP coordinates, so this resets that transform to identity and
repackages the USDZ. Needs `pxr` (conda `3dgrut`):

    conda run -n 3dgrut python -m so101_pai.recon.usdz \
        outputs/3dgrut/<scene>/<scene>_nurec.usdz assets/scene/<scene>.usdz
"""

from __future__ import annotations

import argparse
import re
import tempfile
import zipfile
from pathlib import Path

IDENTITY = "( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) )"
# The volume's own matrix, a single line in gauss.usda; time-sampled camera transforms use other attributes.
VOLUME_TRANSFORM = re.compile(r"matrix4d xformOp:transform = \(.*\)\n")


def reset_volume_transform(layer_text: str) -> str:
    """gauss.usda text with the (first, i.e. the volume's) xformOp:transform set to identity."""
    text, count = VOLUME_TRANSFORM.subn(f"matrix4d xformOp:transform = {IDENTITY}\n", layer_text, count=1)
    if count != 1:
        raise ValueError("no volume xformOp:transform found")
    return text


def repackage(src: str | Path, dst: str | Path) -> None:
    from pxr import Sdf, UsdUtils

    with tempfile.TemporaryDirectory() as tmp:
        with zipfile.ZipFile(src) as archive:
            archive.extractall(tmp)
        layer = Path(tmp) / "gauss.usda"
        layer.write_text(reset_volume_transform(layer.read_text()))
        if not UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(str(Path(tmp) / "default.usda")), str(dst)):
            raise RuntimeError(f"failed to package {dst}")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("src", help="NuRec USDZ written by 3DGRUT")
    parser.add_argument("dst", help="output USDZ (COLMAP frame)")
    args = parser.parse_args(argv)
    repackage(args.src, args.dst)
    print(f"wrote {args.dst}")


if __name__ == "__main__":
    main()
