import numpy as np
import pytest
import torch

from so101_pai.policy.runner import STATE_KEY, ObservationBuffer, PolicyRunner, image_key


def full_buffer() -> ObservationBuffer:
    buffer = ObservationBuffer()
    buffer.state = np.arange(6)
    buffer.images = {camera: np.full((4, 5, 3), 255, np.uint8) for camera in buffer.cameras}
    return buffer


def test_buffer_waits_for_state_and_every_camera():
    buffer = ObservationBuffer()
    assert not buffer.ready()
    buffer.state = np.zeros(6)
    buffer.images["front"] = np.zeros((4, 5, 3), np.uint8)
    assert not buffer.ready()
    with pytest.raises(RuntimeError):
        buffer.frame()


def test_buffer_frame_uses_dataset_keys():
    frame = full_buffer().frame()
    assert set(frame) == {STATE_KEY, image_key("front"), image_key("wrist")}
    assert frame[STATE_KEY].dtype == np.float32


class FakePolicy:
    def __init__(self) -> None:
        self.resets = 0
        self.seen = None

    def reset(self) -> None:
        self.resets += 1

    def select_action(self, batch):
        self.seen = batch
        return batch[STATE_KEY] + 1


class FakeProcessor:
    def __init__(self) -> None:
        self.resets = 0

    def __call__(self, value):
        return value

    def reset(self) -> None:
        self.resets += 1


def test_runner_batches_images_and_returns_flat_action():
    policy, pre, post = FakePolicy(), FakeProcessor(), FakeProcessor()
    runner = PolicyRunner(policy, pre, post, device="cpu", task="put it")
    action = runner.act(full_buffer().frame())
    assert np.allclose(action, np.arange(6) + 1)
    image = policy.seen[image_key("front")]
    assert image.shape == (1, 3, 4, 5) and image.dtype == torch.float32 and float(image.max()) == 1.0
    assert policy.seen["task"] == "put it"
    runner.reset()
    assert (policy.resets, pre.resets, post.resets) == (1, 1, 1)
