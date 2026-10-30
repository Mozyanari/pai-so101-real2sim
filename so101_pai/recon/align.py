"""Align the 3DGRUT reconstruction of the real desk with the sim frame.

The reconstruction lives in COLMAP's frame: arbitrary origin, orientation and scale. The sim puts the robot base
at the origin, the table top at z=0 and the robot facing +y. This module finds the table top in the trained
Gaussians (the largest wood-colored plane), measures where the follower stands on it, and writes the similarity
transform that `sim/pai_sim/tasks/place_block` applies to the background prim (`<name>.align.json`).

Scale comes from tape measurements of the real table, so the result is metric. The checkpoint pickles 3DGRUT's
config, so run the CLI in the conda `3dgrut` env from the repo root (the math itself only needs numpy):

    conda run -n 3dgrut python -m so101_pai.recon.align outputs/3dgrut/workspace/workspace/<run>/ckpt_last.pt \
        assets/scene/workspace.align.json --table-length-cm 90 --table-width-cm 47 --facing across-far
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

SH_C0 = 0.28209479177387814
FACINGS = ("along", "across-far", "across-near")


@dataclass(frozen=True)
class TableFit:
    """Table top in the reconstruction frame. `axes` rows are the long and short in-plane directions."""

    normal: np.ndarray  # unit, pointing up (away from the floor)
    offset: float  # height of point p above the table is normal @ p + offset
    center: np.ndarray  # rectangle center on the plane
    axes: np.ndarray  # (2, 3): long axis, short axis; [long, short, normal] is right-handed
    size: np.ndarray  # (2,): long, short extent
    floor_height: float | None  # floor plane height relative to the table top (negative), if found

    def to_table_frame(self, points: np.ndarray) -> np.ndarray:
        """(N, 3) reconstruction points -> (long, short, height) with the origin at the rectangle center."""
        rel = points - self.center
        return np.column_stack([rel @ self.axes[0], rel @ self.axes[1], points @ self.normal + self.offset])


def load_gaussians(ckpt_path: str | Path, min_opacity: float = 0.08, max_size: float = 0.08):
    """Positions and RGB (0-255) of the opaque, small Gaussians in a 3DGRUT checkpoint."""
    import torch

    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)

    def get(key: str) -> np.ndarray:
        return ckpt[key].detach().float().numpy()

    opacity = 1.0 / (1.0 + np.exp(-get("density").ravel()))
    size = np.exp(get("scale")).max(axis=1)
    rgb = np.clip(get("features_albedo") * SH_C0 + 0.5, 0.0, 1.0)
    keep = (opacity > min_opacity) & (size < max_size)
    return get("positions")[keep].astype(np.float64), (rgb[keep] * 255).astype(np.float64)


def read_camera_centers(images_bin: str | Path) -> np.ndarray:
    """(N, 3) camera centers from a COLMAP sparse model's images.bin (world-to-camera q, t -> -R^T t)."""
    import struct

    centers = []
    with open(images_bin, "rb") as f:
        (count,) = struct.unpack("<Q", f.read(8))
        for _ in range(count):
            values = struct.unpack("<I7dI", f.read(64))
            while f.read(1) != b"\x00":  # image name
                pass
            (n_points,) = struct.unpack("<Q", f.read(8))
            f.seek(24 * n_points, 1)
            q, t = np.array(values[1:5]), np.array(values[5:8])
            centers.append(-_quat_to_matrix(q).T @ t)
    return np.array(centers)


def wood_mask(colors: np.ndarray) -> np.ndarray:
    r, g, b = colors.T
    return (r - b > 40) & (r > g) & (g > b) & (r > 80)


def fit_plane(points: np.ndarray, threshold: float, iterations: int = 4000, seed: int = 0):
    """RANSAC plane, refined by SVD on the inliers. Returns (unit normal, offset) with normal @ p + offset = 0."""
    rng = np.random.default_rng(seed)
    best_normal, best_offset, best_count = None, 0.0, -1
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            continue
        normal /= norm
        count = int((np.abs(points @ normal - normal @ a) < threshold).sum())
        if count > best_count:
            best_normal, best_offset, best_count = normal, -normal @ a, count
    inliers = points[np.abs(points @ best_normal + best_offset) < threshold]
    centroid = inliers.mean(axis=0)
    normal = np.linalg.svd(inliers - centroid)[2][2]
    return normal, float(-normal @ centroid)


def _min_area_rectangle(uv: np.ndarray, trim: float = 0.5):
    """Angle, low and high corners of the trimmed min-area rectangle around 2D points (axis 0 = long side)."""
    best = None
    for angle in np.radians(np.arange(0.0, 180.0, 0.25)):
        rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        q = uv @ rot
        lo, hi = np.percentile(q, trim, axis=0), np.percentile(q, 100 - trim, axis=0)
        ext = hi - lo
        if ext[0] < ext[1]:
            continue
        if best is None or ext.prod() < best[0]:
            best = (ext.prod(), rot, lo, hi)
    return best[1], best[2], best[3]


def fit_table(points: np.ndarray, colors: np.ndarray, cameras: np.ndarray, threshold: float = 0.04) -> TableFit:
    """Find the table top: the dominant plane among wood-colored points, facing the cameras that filmed it.

    Other wood in the same plane (a shelf, the floor's edge) is dropped by keeping the largest connected patch.
    """
    wood = points[wood_mask(colors)]
    normal, offset = fit_plane(wood, threshold)
    if np.median(cameras @ normal + offset) < 0:
        normal, offset = -normal, -offset
    floor = _floor_height(points @ normal + offset)

    basis = np.linalg.svd(np.eye(3) - np.outer(normal, normal))[0][:, :2].T  # any orthonormal in-plane basis
    top = wood[np.abs(wood @ normal + offset) < threshold]
    origin = top.mean(axis=0)
    uv = _largest_patch((top - origin) @ basis.T)
    rot, lo, hi = _min_area_rectangle(uv)
    axes = rot.T @ basis
    if np.linalg.det(np.vstack([axes, normal])) < 0:
        axes[1] = -axes[1]
    mid = (lo + hi) / 2
    center = origin + mid[0] * axes[0] + mid[1] * axes[1]
    center = center - (center @ normal + offset) * normal
    return TableFit(normal, offset, center, axes, hi - lo, floor)


def _largest_patch(uv: np.ndarray, cells: int = 120) -> np.ndarray:
    """2D points of the largest 8-connected patch on an occupancy grid of `cells` across the larger extent."""
    from scipy import ndimage

    lo = uv.min(axis=0)
    cell = (uv.max(axis=0) - lo).max() / cells
    ij = ((uv - lo) / cell).astype(int)
    grid = np.zeros(ij.max(axis=0) + 1, dtype=bool)
    grid[ij[:, 0], ij[:, 1]] = True
    grid = ndimage.binary_closing(grid, iterations=2)
    labels, count = ndimage.label(grid, structure=np.ones((3, 3)))
    if count == 0:
        return uv
    sizes = ndimage.sum(grid, labels, index=np.arange(1, count + 1))
    keep = labels[ij[:, 0], ij[:, 1]] == 1 + int(np.argmax(sizes))
    return uv[keep]


def _floor_height(heights: np.ndarray, min_gap: float = 0.5) -> float | None:
    """Height of the strongest horizontal layer well below the table top (the floor), if any."""
    far = heights[heights < -min_gap]
    if len(far) < 100:
        return None
    hist, edges = np.histogram(far, bins=200)
    i = int(np.argmax(hist))
    return float((edges[i] + edges[i + 1]) / 2)


def find_follower_base(table: TableFit, points: np.ndarray, colors: np.ndarray) -> np.ndarray:
    """(long, short) of the follower base: the lighter of the two arm clusters standing near the short edges."""
    local = table.to_table_frame(points)
    half = table.size / 2
    on_table = (np.abs(local[:, 0]) < half[0]) & (np.abs(local[:, 1]) < half[1])
    base_band = on_table & (local[:, 2] > 0.03 * table.size[0]) & (local[:, 2] < 0.08 * table.size[0])
    candidates = []
    for sign in (-1.0, 1.0):
        near_end = base_band & (sign * local[:, 0] > half[0] - 0.35 * table.size[0])
        if near_end.sum() < 20:
            continue
        candidates.append((colors[near_end].mean(), np.median(local[near_end, :2], axis=0)))
    if not candidates:
        raise ValueError("no arm found near either short edge of the table")
    return max(candidates, key=lambda c: c[0])[1]


def base_from_edges(table: TableFit, near_base: np.ndarray, from_short: float, from_long: float) -> np.ndarray:
    """(long, short) of a point measured from the table edges nearest to `near_base` (same units as the table)."""
    half = table.size / 2
    sign = np.sign(near_base)
    sign[sign == 0] = 1.0
    return np.array([sign[0] * (half[0] - from_short), sign[1] * (half[1] - from_long)])


def forward_direction(base: np.ndarray, facing: str) -> np.ndarray:
    """Unit (long, short) direction the robot faces, from where its base sits on the table."""
    if facing not in FACINGS:
        raise ValueError(f"facing must be one of {FACINGS}")
    sign = np.sign(base)
    sign[sign == 0] = 1.0
    if facing == "along":
        return np.array([-sign[0], 0.0])
    if facing == "across-far":
        return np.array([0.0, -sign[1]])
    return np.array([0.0, sign[1]])


def alignment(table: TableFit, base: np.ndarray, forward: np.ndarray, scale: float) -> dict:
    """Similarity transform reconstruction -> sim: base at the origin, table top at z=0, forward along +y.

    Returns {"pos": [x, y, z], "rot": [w, x, y, z], "scale": s} for p_sim = pos + R(rot) @ (scale * p), plus the
    table top in the sim frame ({"center": [x, y], "size": [x, y]} in meters) for the sim's collision box.
    """
    yaw = np.arctan2(forward[0], forward[1])  # rotate forward (long, short) onto +y
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    frame = np.vstack([table.axes, table.normal])  # rows: long, short, up
    rot = rz @ frame
    base_point = table.center + base[0] * table.axes[0] + base[1] * table.axes[1]
    pos = -scale * rot @ base_point
    center = pos + scale * rot @ table.center
    size = np.abs(rz[:2, :2] @ table.size) * scale
    return {
        "pos": pos.tolist(),
        "rot": _quat_wxyz(rot).tolist(),
        "scale": float(scale),
        "table": {"center": center[:2].tolist(), "size": size.tolist()},
    }


def apply(align: dict, points: np.ndarray) -> np.ndarray:
    return np.asarray(align["pos"]) + align["scale"] * points @ _quat_to_matrix(np.asarray(align["rot"])).T


def _quat_wxyz(m: np.ndarray) -> np.ndarray:
    w = np.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("ckpt", help="3DGRUT ckpt_last.pt")
    parser.add_argument("out", help="output <usdz name>.align.json")
    parser.add_argument("--sparse", default="data/recon/workspace/sparse/0", help="COLMAP model (camera centers)")
    parser.add_argument("--table-length-cm", type=float, required=True, help="real table, long side")
    parser.add_argument("--table-width-cm", type=float, help="real table, short side (consistency check)")
    parser.add_argument("--table-height-cm", type=float, help="real table top above the floor (consistency check)")
    parser.add_argument("--base-from-short-cm", type=float, help="follower base center to the nearest short edge")
    parser.add_argument("--base-from-long-cm", type=float, help="follower base center to the nearest long edge")
    parser.add_argument("--facing", choices=FACINGS, required=True, help="direction the follower reaches forward")
    args = parser.parse_args(argv)

    points, colors = load_gaussians(args.ckpt)
    table = fit_table(points, colors, read_camera_centers(Path(args.sparse) / "images.bin"))
    scale = args.table_length_cm / 100.0 / table.size[0]  # meters per reconstruction unit
    print(f"table {table.size[0] * scale * 100:.1f} x {table.size[1] * scale * 100:.1f} cm (scale {scale:.4f} m/unit)")
    if args.table_width_cm:
        print(f"  width: measured {args.table_width_cm:.1f} cm")
    if table.floor_height is not None:
        print(f"  height above floor {-table.floor_height * scale * 100:.1f} cm", end="")
        print(f" (measured {args.table_height_cm:.1f} cm)" if args.table_height_cm else "")

    detected = find_follower_base(table, points, colors)
    edges = (table.size / 2 - np.abs(detected)) * scale * 100
    print(f"follower base detected {edges[0]:.1f} cm from the short edge, {edges[1]:.1f} cm from the long edge")
    base = detected
    if args.base_from_short_cm is not None and args.base_from_long_cm is not None:
        to_units = 1 / 100 / scale
        base = base_from_edges(table, detected, args.base_from_short_cm * to_units, args.base_from_long_cm * to_units)
        print(f"base: measured vs detected differ by {np.linalg.norm(base - detected) * scale * 100:.1f} cm")
    result = alignment(table, base, forward_direction(base, args.facing), scale)
    Path(args.out).write_text(json.dumps(result, indent=2) + "\n")
    print(f"wrote {args.out}: {result}")


if __name__ == "__main__":
    main()
