"""Place a real camera where the sim camera is: show the live image blended with the sim's view of the same scene.

    PYTHONPATH=$PWD /usr/bin/python3 -m so101_pai.robot.camera_overlay --device /dev/video0 \
        --reference assets/reference/front_sim.png

The .venv's OpenCV is the headless build LeRobot pulls in, so run it with the system Python (Ubuntu's OpenCV has Qt).

The policy only saw the sim front camera, randomized by ±1 cm / ±1 deg, so the real one should match it closely.
Move the camera until the table edge, plate and robot base line up. Keys: [ / ] blend, e = edges only, q = quit.
"""

from __future__ import annotations

import argparse

import cv2
import numpy as np

from so101_pai.common import contract


def blend(live: np.ndarray, reference: np.ndarray, alpha: float, edges: bool) -> np.ndarray:
    """Live BGR frame with the reference on top: alpha-blended, or as red edges over the live image."""
    if edges:
        out = live.copy()
        out[cv2.Canny(cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY), 80, 160) > 0] = (0, 0, 255)
        return out
    return cv2.addWeighted(live, 1.0 - alpha, reference, alpha, 0.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--reference", default="assets/reference/front_sim.png")
    args = parser.parse_args()

    size = (contract.IMAGE_WIDTH, contract.IMAGE_HEIGHT)
    reference = cv2.imread(args.reference)
    if reference is None:
        raise SystemExit(f"cannot read {args.reference}")
    reference = cv2.resize(reference, size)
    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # as camera_node
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    if not capture.isOpened():
        raise SystemExit(f"cannot open camera {args.device}")
    alpha, edges = 0.5, False
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                continue
            # Same resize as camera_node, so what lines up here lines up for the policy.
            frame = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
            cv2.imshow("camera overlay ([ ] blend, e edges, q quit)", blend(frame, reference, alpha, edges))
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key in (ord("["), ord("]")):
                alpha = float(np.clip(alpha + (0.1 if key == ord("]") else -0.1), 0.0, 1.0))
            if key == ord("e"):
                edges = not edges
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
