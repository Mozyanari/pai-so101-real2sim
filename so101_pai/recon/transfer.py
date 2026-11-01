"""Carry a table alignment over to a new capture of the same room, through a joint COLMAP model of both captures.

align.py finds the table in the Gaussians; when that fails for a new capture (a glossy top trains to faint,
large Gaussians), reuse the old capture's alignment instead. A COLMAP model built from both captures' frames
(old frames under `<old_prefix>/`, new ones under `<new_prefix>/`) gives each frame's camera center in one frame;
fitting a similarity to the old model's and the new model's centers of the same frames links the two:

    p_old = T_joint->old (T_joint->new)^-1 p_new,   then   p_sim = old alignment (p_old)

    python -m so101_pai.recon.transfer data/recon/joint/sparse/0 data/recon/workspace/sparse/0 pixel \
        data/recon/workspace2/sparse/0 a7c assets/scene/workspace.align.json assets/scene/workspace2.align.json
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

import numpy as np

from so101_pai.recon.align import _quat_to_matrix, _quat_wxyz


def read_camera_centers_by_name(images_bin: str | Path) -> dict[str, np.ndarray]:
    """{image name: camera center} from a COLMAP sparse model's images.bin."""
    centers = {}
    with open(images_bin, "rb") as f:
        (count,) = struct.unpack("<Q", f.read(8))
        for _ in range(count):
            values = struct.unpack("<I7dI", f.read(64))
            name = b""
            while (char := f.read(1)) != b"\x00":
                name += char
            (n_points,) = struct.unpack("<Q", f.read(8))
            f.seek(24 * n_points, 1)
            q, t = np.array(values[1:5]), np.array(values[5:8])
            centers[name.decode()] = -_quat_to_matrix(q).T @ t
    return centers


def similarity(src: np.ndarray, dst: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """Umeyama: (s, R, t) minimizing |dst - (s R src + t)|^2 over (N, 3) point pairs."""
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    a, b = src - mu_s, dst - mu_d
    u, sigma, vt = np.linalg.svd(b.T @ a / len(src))
    d = np.eye(3)
    d[2, 2] = np.sign(np.linalg.det(u @ vt))
    rot = u @ d @ vt
    scale = float(np.trace(np.diag(sigma) @ d) / (a**2).sum(axis=1).mean())
    return scale, rot, mu_d - scale * rot @ mu_s


def fit_between(joint: dict, model: dict, prefix: str) -> tuple[tuple[float, np.ndarray, np.ndarray], np.ndarray]:
    """Similarity joint -> model from the frames both contain, and its per-frame residuals (model units)."""
    names = sorted(n for n in model if f"{prefix}/{n}" in joint)
    if len(names) < 3:
        raise ValueError(f"only {len(names)} '{prefix}' frames are in both models")
    src = np.array([joint[f"{prefix}/{n}"] for n in names])
    dst = np.array([model[n] for n in names])
    s, r, t = similarity(src, dst)
    return (s, r, t), np.linalg.norm(dst - (s * src @ r.T + t), axis=1)


def compose(align: dict, s: float, r: np.ndarray, t: np.ndarray) -> dict:
    """Alignment of points p_new given the old one and p_old = s r p_new + t."""
    old_rot = _quat_to_matrix(np.asarray(align["rot"]))
    return {
        **align,
        "pos": (np.asarray(align["pos"]) + align["scale"] * old_rot @ t).tolist(),
        "rot": _quat_wxyz(old_rot @ r).tolist(),
        "scale": float(align["scale"] * s),
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("joint", type=Path, help="joint COLMAP model dir")
    parser.add_argument("old", type=Path, help="old capture's COLMAP model dir")
    parser.add_argument("old_prefix", help="folder of the old frames in the joint model")
    parser.add_argument("new", type=Path, help="new capture's COLMAP model dir")
    parser.add_argument("new_prefix", help="folder of the new frames in the joint model")
    parser.add_argument("old_align", type=Path)
    parser.add_argument("out", type=Path)
    args = parser.parse_args(argv)

    joint = read_camera_centers_by_name(args.joint / "images.bin")
    old_align = json.loads(args.old_align.read_text())
    (s_o, r_o, t_o), res_o = fit_between(joint, read_camera_centers_by_name(args.old / "images.bin"), args.old_prefix)
    (s_n, r_n, t_n), res_n = fit_between(joint, read_camera_centers_by_name(args.new / "images.bin"), args.new_prefix)
    m_per_old = old_align["scale"]
    print(f"old frames: {len(res_o)}, residual median {np.median(res_o) * m_per_old * 100:.2f} cm")
    print(f"new frames: {len(res_n)}, residual median {np.median(res_n) * s_o / s_n * m_per_old * 100:.2f} cm")
    # p_old = T_o T_n^-1 p_new, with T_n^-1 p = r_n^T (p - t_n) / s_n
    s = s_o / s_n
    r = r_o @ r_n.T
    t = t_o - s * r @ t_n
    result = compose(old_align, s, r, t)
    args.out.write_text(json.dumps(result, indent=2) + "\n")
    print(f"wrote {args.out}: scale {result['scale']:.4f} m/unit")


if __name__ == "__main__":
    main()
