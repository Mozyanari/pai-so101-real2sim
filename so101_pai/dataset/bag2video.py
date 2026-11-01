"""Cut the camera images of rosbag2 recordings into MP4 clips (H.264), one per episode, for the project video.

Usage (inside .venv with /opt/ros/jazzy sourced):
    python -m so101_pai.dataset.bag2video data/bags/eval/B20k_s30_start --out outputs/video [--layout inset]

Each episode (start -> end event) becomes <out>/<bag>_ep<k>_<success|fail|open>.mp4; --whole writes the whole bag as
one clip instead. Frames are resampled at contract.FPS on the bag's receive clock (zero-order hold, as bag2lerobot),
and the bag is streamed message by message, so a long run does not need to fit in memory. A minute of the two raw
cameras is ~2.2 GB of bag and ~20 MB of MP4.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import cv2
import numpy as np

from so101_pai.common import contract, episode
from so101_pai.common.ros_msgs import image_to_rgb

LOG = logging.getLogger("bag2video")
LAYOUTS = ("inset", "side", "front", "wrist")
FRAME_NS = 1_000_000_000 // contract.FPS


def compose(front: np.ndarray, wrist: np.ndarray, layout: str = "inset") -> np.ndarray:
    """One RGB frame from the two cameras: the front view with the wrist view in its corner, side by side, or one."""
    if layout == "front":
        return front
    if layout == "wrist":
        return wrist
    if layout == "side":
        return np.hstack([front, wrist])
    if layout != "inset":
        raise ValueError(f"layout must be one of {LAYOUTS}, got {layout!r}")
    height, width = front.shape[:2]
    small = cv2.resize(wrist, (width // 3, height // 3), interpolation=cv2.INTER_AREA)
    out = front.copy()
    y, x = height - small.shape[0] - 8, width - small.shape[1] - 8
    out[y - 2 : y + small.shape[0] + 2, x - 2 : x + small.shape[1] + 2] = 255  # white frame around the inset
    out[y : y + small.shape[0], x : x + small.shape[1]] = small
    return out


def put_caption(image: np.ndarray, text: str) -> np.ndarray:
    """`text` in the top-left corner on a dark box."""
    out = image.copy()
    (w, h), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    cv2.rectangle(out, (0, 0), (w + 12, h + base + 10), (0, 0, 0), -1)
    cv2.putText(out, text, (6, h + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return out


class Clip:
    """An H.264 MP4 written frame by frame (yuv420p, so any player and YouTube take it)."""

    def __init__(self, path: Path, crf: int = 23) -> None:
        import av

        self.path = path
        self.container = av.open(str(path), mode="w")
        self.stream = None
        self.crf = crf
        self.frames = 0

    def write(self, rgb: np.ndarray) -> None:
        import av

        if self.stream is None:
            self.stream = self.container.add_stream("libx264", rate=contract.FPS)
            self.stream.height, self.stream.width = rgb.shape[:2]
            self.stream.pix_fmt = "yuv420p"
            self.stream.options = {"crf": str(self.crf)}
        for packet in self.stream.encode(av.VideoFrame.from_ndarray(np.ascontiguousarray(rgb), format="rgb24")):
            self.container.mux(packet)
        self.frames += 1

    def close(self) -> None:
        if self.stream is not None:
            for packet in self.stream.encode():
                self.container.mux(packet)
        self.container.close()


def convert(bag: Path, out_dir: Path, layout: str = "inset", whole: bool = False, caption: bool = False) -> list[Path]:
    """Write the clips of one bag; returns their paths."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    out_dir.mkdir(parents=True, exist_ok=True)
    camera_by_topic = {topic: camera for camera, topic in contract.CAMERA_TOPICS.items()}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag)), rosbag2_py.ConverterOptions("", ""))

    latest: dict[str, object] = {}  # newest raw Image per camera; decoded only when a frame is due
    clip: Clip | None = None
    next_ns = 0
    count = 0
    written: list[Path] = []

    def open_clip(stamp_ns: int) -> Clip:
        nonlocal next_ns
        next_ns = stamp_ns
        name = f"{bag.name}.mp4" if whole else f"{bag.name}_ep{count}.mp4"
        return Clip(out_dir / name)

    def emit_until(stamp_ns: int) -> None:
        nonlocal next_ns
        while clip is not None and next_ns <= stamp_ns:
            if len(latest) == len(contract.CAMERA_TOPICS):
                frame = compose(image_to_rgb(latest["front"]), image_to_rgb(latest["wrist"]), layout)
                if caption:
                    frame = put_caption(frame, f"{bag.name}" + ("" if whole else f"  trial {count + 1}"))
                clip.write(frame)
            next_ns += FRAME_NS

    def close_clip(result: str | None) -> None:
        nonlocal clip
        clip.close()
        path = clip.path
        if clip.frames == 0:
            path.unlink(missing_ok=True)
            LOG.warning("%s: no camera frames, skipped", path.name)
        else:
            if result is not None:
                path = path.rename(path.with_name(f"{path.stem}_{result}.mp4"))
            LOG.info("%s (%.1f s)", path, clip.frames / contract.FPS)
            written.append(path)
        clip = None

    stamp_ns = 0
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if whole and clip is None:
            clip = open_clip(stamp_ns)
        emit_until(stamp_ns)
        if topic in camera_by_topic:
            latest[camera_by_topic[topic]] = deserialize_message(raw, Image)
        elif topic == contract.TOPIC_EPISODE and not whole:
            event = episode.decode(deserialize_message(raw, String).data)
            if event.event == episode.START:
                if clip is not None:  # a start without an end: keep what was recorded
                    close_clip("open")
                    count += 1
                clip = open_clip(stamp_ns)
            elif event.event == episode.END and clip is not None:
                close_clip("success" if event.success else "fail")
                count += 1
    if clip is not None:
        close_clip(None if whole else "open")
    return written


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bags", nargs="+", type=Path, help="rosbag2 directories")
    parser.add_argument("--out", type=Path, default=Path("outputs/video"))
    parser.add_argument("--layout", choices=LAYOUTS, default="inset", help="inset = front, wrist in a corner")
    parser.add_argument("--whole", action="store_true", help="one clip of the whole bag instead of one per episode")
    parser.add_argument("--caption", action="store_true", help="write the bag name and trial number on the frames")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    for bag in args.bags:
        convert(bag, args.out, args.layout, args.whole, args.caption)


if __name__ == "__main__":
    main()
