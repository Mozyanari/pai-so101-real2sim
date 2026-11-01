"""Recolor a 3DGRUT checkpoint per channel (color' = gain * color + offset), to match the real camera's response.

The background was filmed with a different camera (a7C) than the robot's (C920), so the sim renders it brighter and
redder than the real front camera sees the same table. Fit gain/offset from matching regions of a real frame and
a sim render, then:

    conda run -n 3dgrut python -m so101_pai.recon.color <ckpt> <out ckpt> \\
        --gain=1.13,0.91,1.01 --offset=-0.29,-0.07,-0.19
"""

from __future__ import annotations

import argparse

import numpy as np

SH_C0 = 0.28209479177387814


def recolor_dc(features: np.ndarray, gain: np.ndarray, offset: np.ndarray) -> np.ndarray:
    """SH DC coefficients (N, 3) whose base color (SH_C0 * f + 0.5) becomes gain * color + offset."""
    color = SH_C0 * features + 0.5
    return (gain * color + offset - 0.5) / SH_C0


def fit_gain_offset(sim: np.ndarray, real: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Per-channel gain and offset mapping sim colors (N, 3) onto real ones by least squares."""
    gains, offsets = [], []
    for c in range(3):
        a = np.column_stack([sim[:, c], np.ones(len(sim))])
        g, o = np.linalg.lstsq(a, real[:, c], rcond=None)[0]
        gains.append(g)
        offsets.append(o)
    return np.array(gains), np.array(offsets)


def _triple(text: str) -> np.ndarray:
    values = np.array([float(v) for v in text.split(",")])
    if values.shape != (3,):
        raise argparse.ArgumentTypeError(f"expected r,g,b, got {text!r}")
    return values


def main() -> None:
    import torch

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("ckpt")
    parser.add_argument("out")
    parser.add_argument("--gain", type=_triple, required=True, help="r,g,b")
    parser.add_argument("--offset", type=_triple, required=True, help="r,g,b in 0-1 color units")
    args = parser.parse_args()
    ckpt = torch.load(args.ckpt, map_location="cpu", weights_only=False)

    def replace(key: str, value: torch.Tensor) -> None:
        old = ckpt[key]
        value = value.to(old.dtype)
        is_param = isinstance(old, torch.nn.Parameter)
        ckpt[key] = torch.nn.Parameter(value, requires_grad=old.requires_grad) if is_param else value

    albedo = ckpt["features_albedo"].detach()
    replace("features_albedo", torch.from_numpy(recolor_dc(albedo.cpu().numpy(), args.gain, args.offset)))
    # Higher SH bands add view-dependent color on top of the base color; scale them by the gain only. Their layout
    # is (coefficient, rgb) flattened, as 3DGRUT's PLY import writes it.
    specular = ckpt["features_specular"].detach()
    replace("features_specular", specular * torch.from_numpy(np.tile(args.gain, specular.shape[1] // 3)))
    torch.save(ckpt, args.out)
    print(f"wrote {args.out}: gain {args.gain}, offset {args.offset}")


if __name__ == "__main__":
    main()
