"""End-to-end: write a synthetic rosbag2 (mcap) under the contract, convert it, read it back with LeRobot."""

import numpy as np
import pytest

rosbag2_py = pytest.importorskip("rosbag2_py")

from rclpy.serialization import serialize_message  # noqa: E402
from sensor_msgs.msg import Image, JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from so101_pai.common import contract, episode  # noqa: E402
from so101_pai.common.ros_msgs import image_to_rgb  # noqa: E402
from so101_pai.dataset.bag2lerobot import convert  # noqa: E402

SECOND = 1_000_000_000
TOPIC_TYPES = {
    contract.TOPIC_STATE: "sensor_msgs/msg/JointState",
    contract.TOPIC_ACTION: "sensor_msgs/msg/JointState",
    contract.TOPIC_EPISODE: "std_msgs/msg/String",
    **{topic: "sensor_msgs/msg/Image" for topic in contract.CAMERA_TOPICS.values()},
}


def joint_msg(values_rad):
    # Deliberately shuffled order: the converter must reorder by name.
    order = [5, 0, 4, 1, 3, 2]
    msg = JointState()
    msg.name = [contract.JOINT_NAMES[i] for i in order]
    msg.position = [float(values_rad[i]) for i in order]
    return msg


def image_msg(value, encoding="rgb8", height=contract.IMAGE_HEIGHT, width=contract.IMAGE_WIDTH):
    msg = Image()
    msg.height, msg.width, msg.encoding, msg.step = height, width, encoding, width * 3
    pixels = np.zeros((height, width, 3), dtype=np.uint8)
    pixels[..., 0] = value
    msg.data = pixels.tobytes()
    return msg


def write_bag(path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    for topic, type_name in TOPIC_TYPES.items():
        writer.create_topic(rosbag2_py.TopicMetadata(0, topic, type_name, "cdr"))

    def put(topic, msg, t):
        writer.write(topic, serialize_message(msg), t)

    def status(event, t):
        put(contract.TOPIC_EPISODE, String(data=episode.encode(event)), t)

    # Episode 1 (success, 1 s) and episode 2 (failure, 1 s). Sensors stream at 60 Hz throughout.
    status(episode.EpisodeEvent(episode.START, task="Put the red block on the plate"), 0)
    status(episode.EpisodeEvent(episode.END, success=True), SECOND)
    status(episode.EpisodeEvent(episode.START, task="failed try"), 2 * SECOND)
    status(episode.EpisodeEvent(episode.END, success=False), 3 * SECOND)
    for k in range(0, 3 * 60 + 1):
        t = k * SECOND // 60
        angle = np.radians(k % 60)
        put(contract.TOPIC_STATE, joint_msg(np.full(6, angle)), t)
        put(contract.TOPIC_ACTION, joint_msg(np.full(6, angle + 0.01)), t)
        for topic in contract.CAMERA_TOPICS.values():
            put(topic, image_msg(k % 256), t)
    writer.close()


def test_image_bgr_and_resize():
    msg = image_msg(0, encoding="bgr8", height=240, width=320)
    raw = np.zeros((240, 320, 3), dtype=np.uint8)
    raw[..., 2] = 200  # red in BGR
    msg.data = raw.tobytes()
    rgb = image_to_rgb(msg)
    assert rgb.shape == (contract.IMAGE_HEIGHT, contract.IMAGE_WIDTH, 3)
    assert rgb[0, 0, 0] == 200 and rgb[0, 0, 2] == 0


def test_convert_keeps_only_successful_episodes(tmp_path):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    bag = tmp_path / "bag"
    write_bag(bag)
    root = tmp_path / "dataset"
    convert([bag], repo_id="local/test", root=root, use_videos=False)

    dataset = LeRobotDataset("local/test", root=root)
    assert dataset.meta.total_episodes == 1
    assert dataset.meta.total_frames == contract.FPS + 1
    item = dataset[0]
    assert item["task"] == "Put the red block on the plate"
    np.testing.assert_allclose(item["observation.state"].numpy(), contract.radians_to_motor(np.zeros(6)), atol=1e-4)
    assert item["observation.images.front"].shape == (3, contract.IMAGE_HEIGHT, contract.IMAGE_WIDTH)
