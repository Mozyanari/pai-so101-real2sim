"""Remove real objects baked into the 3DGS background, e.g. the real follower, which would overlap the sim robot.

Boxes are given in the sim frame (meters; robot base at the origin, table top at z=0), so the alignment JSON from
`so101_pai.recon.align` is needed. Gaussians inside a box get zero opacity. The table surface under a removed object
was never filmed, so `--patch` fills it by copying the Gaussians of a nearby box, shifted (sim frame, meters).
Run in conda `3dgrut` from the repo root (the checkpoint pickles 3DGRUT's config):

    conda run -n 3dgrut python -m so101_pai.recon.crop <ckpt_last.pt> assets/scene/workspace.align.json \
        <out.pt> --box=-0.13,-0.20,0.006,0.08,0.15,0.30 --patch=0.10,-0.09,-0.02,0.31,0.15,0.006:-0.23,0,0
"""

from __future__ import annotations

import argparse
import json

import numpy as np

from so101_pai.recon.align import _quat_to_matrix, apply

HIDDEN_DENSITY = -30.0  # sigmoid(-30) ~ 1e-13: invisible


def parse_box(text: str) -> tuple[np.ndarray, np.ndarray]:
    values = [float(v) for v in text.split(",")]
    if len(values) != 6:
        raise ValueError(f"box needs x0,y0,z0,x1,y1,z1, got {text!r}")
    lo, hi = np.array(values[:3]), np.array(values[3:])
    if np.any(lo >= hi):
        raise ValueError(f"box corners out of order: {text!r}")
    return lo, hi


def parse_patch(text: str) -> tuple[tuple[np.ndarray, np.ndarray], np.ndarray]:
    box, _, shift = text.partition(":")
    offset = np.array([float(v) for v in shift.split(",")])
    if offset.shape != (3,):
        raise ValueError(f"patch needs x0,y0,z0,x1,y1,z1:dx,dy,dz, got {text!r}")
    return parse_box(box), offset


def sim_offset_to_recon(align: dict, offset: np.ndarray) -> np.ndarray:
    """A translation in the sim frame expressed in the reconstruction frame (inverse rotation and scale)."""
    return _quat_to_matrix(np.asarray(align["rot"])).T @ offset / align["scale"]


def inside_boxes(points: np.ndarray, boxes: list[tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
    mask = np.zeros(len(points), dtype=bool)
    for lo, hi in boxes:
        mask |= np.all((points >= lo) & (points <= hi), axis=1)
    return mask


def main(argv: list[str] | None = None) -> None:
    import torch

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("ckpt", help="3DGRUT checkpoint")
    parser.add_argument("align", help="<name>.align.json")
    parser.add_argument("out", help="output checkpoint")
    parser.add_argument("--box", action="append", required=True, help="x0,y0,z0,x1,y1,z1 in the sim frame (m)")
    parser.add_argument("--patch", action="append", default=[], help="x0,y0,z0,x1,y1,z1:dx,dy,dz copy + shift")
    args = parser.parse_args(argv)

    ckpt = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    align = json.loads(open(args.align).read())
    points = apply(align, ckpt["positions"].detach().double().numpy())
    hide = torch.from_numpy(inside_boxes(points, [parse_box(b) for b in args.box]))
    with torch.no_grad():
        ckpt["density"][hide] = HIDDEN_DENSITY
    print(f"hid {int(hide.sum())} of {len(hide)} Gaussians")

    per_gaussian = [k for k, v in ckpt.items() if hasattr(v, "shape") and v.ndim >= 1 and v.shape[0] == len(hide)]
    for patch in args.patch:
        box, offset = parse_patch(patch)
        source = torch.zeros(len(ckpt["positions"]), dtype=torch.bool)  # earlier patches append rows
        source[: len(hide)] = torch.from_numpy(inside_boxes(points, [box]) & ~hide.numpy())
        shift = torch.from_numpy(sim_offset_to_recon(align, offset)).to(ckpt["positions"].dtype)
        with torch.no_grad():
            for key in per_gaussian:
                copied = ckpt[key][source].clone()
                if key == "positions":
                    copied += shift
                merged = torch.cat([ckpt[key].detach(), copied])
                is_param = isinstance(ckpt[key], torch.nn.Parameter)
                ckpt[key] = torch.nn.Parameter(merged, requires_grad=ckpt[key].requires_grad) if is_param else merged
        print(f"patched {int(source.sum())} Gaussians shifted by {offset.tolist()} m")
    torch.save(ckpt, args.out)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
