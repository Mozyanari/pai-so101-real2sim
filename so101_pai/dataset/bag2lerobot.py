"""Convert rosbag2 recordings that follow so101_pai.common.contract into a LeRobot v3.0 dataset.

Usage (inside .venv with /opt/ros/jazzy sourced):
    python -m so101_pai.dataset.bag2lerobot BAG [BAG ...] --repo-id USER/NAME --root OUT_DIR [--include-failed]

Every message is timed by the bag's receive timestamp, so sim and real recordings share one clock.
Frames are resampled at contract.FPS with zero-order hold; frames before every stream has a
message are skipped. Bags are streamed one episode at a time (a 25 min sim session is ~57 GB of images).
"""

from __future__ import annotations

import argparse
import logging
from collections.abc import Iterator
from pathlib import Path

from so101_pai.common import contract, episode
from so101_pai.common.ros_msgs import image_to_rgb, joint_state_to_array
from so101_pai.dataset.timeline import EpisodeSpan, EpisodeSplitter, Timeline, frame_stamps

LOG = logging.getLogger("bag2lerobot")


STREAMS = ("state", "action", *contract.CAMERA_TOPICS)


def read_episodes(path: Path) -> Iterator[tuple[EpisodeSpan, dict[str, Timeline]]]:
    """Each episode of a bag with its streams ("state", "action", camera names). Consume each before the next."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)), rosbag2_py.ConverterOptions("", ""))
    types = {meta.name: get_message(meta.type) for meta in reader.get_all_topics_and_types()}
    stream_by_topic = {contract.TOPIC_STATE: "state", contract.TOPIC_ACTION: "action"}
    stream_by_topic.update({topic: camera for camera, topic in contract.CAMERA_TOPICS.items()})

    splitter = EpisodeSplitter(STREAMS)
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(raw, types[topic])
        if topic == contract.TOPIC_EPISODE:
            span = splitter.event(stamp_ns, episode.decode(msg.data))
            if span is not None:
                yield span, splitter.streams
        elif topic in stream_by_topic:
            name = stream_by_topic[topic]
            splitter.add(name, stamp_ns, msg if name in contract.CAMERA_TOPICS else joint_state_to_array(msg))


def episode_frames(streams: dict[str, Timeline], span: EpisodeSpan) -> list[dict]:
    frames = []
    for stamp_ns in frame_stamps(span.start_ns, span.end_ns, contract.FPS):
        state = streams["state"].sample(stamp_ns)
        action = streams["action"].sample(stamp_ns)
        images = {camera: streams[camera].sample(stamp_ns) for camera in contract.CAMERA_TOPICS}
        if state is None or action is None or any(image is None for image in images.values()):
            continue
        frame = {
            "observation.state": contract.radians_to_motor(state),
            "action": contract.radians_to_motor(action),
            "task": span.task,
        }
        for camera, msg in images.items():
            frame[f"observation.images.{camera}"] = image_to_rgb(msg)
        frames.append(frame)
    return frames


def convert(bags: list[Path], repo_id: str, root: Path, include_failed: bool = False, use_videos: bool = True):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=contract.FPS,
        features=contract.dataset_features(use_videos),
        root=root,
        robot_type="so101_follower",
        use_videos=use_videos,
    )
    saved = skipped = 0
    for bag in bags:
        for span, streams in read_episodes(bag):
            if not span.success and not include_failed:
                skipped += 1
                continue
            frames = episode_frames(streams, span)
            if not frames:
                LOG.warning("%s: episode at %d has no complete frames, skipped", bag, span.start_ns)
                skipped += 1
                continue
            for frame in frames:
                dataset.add_frame(frame)
            dataset.save_episode()
            saved += 1
    dataset.finalize()
    LOG.info("saved %d episodes, skipped %d", saved, skipped)
    return dataset


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bags", nargs="+", type=Path, help="rosbag2 directories")
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--root", type=Path, required=True, help="output directory for the dataset")
    parser.add_argument("--include-failed", action="store_true", help="also keep episodes ended with success=false")
    parser.add_argument("--no-videos", action="store_true", help="store PNG images instead of MP4 videos")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    convert(args.bags, args.repo_id, args.root, args.include_failed, not args.no_videos)


if __name__ == "__main__":
    main()
