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


class FakeChunkPolicy(FakePolicy):
    def predict_action_chunk(self, batch):
        self.seen = batch
        return torch.stack([batch[STATE_KEY] + i for i in range(3)], dim=1)  # (1, 3, 6)


class AddOne(FakeProcessor):
    def __call__(self, value):
        assert value.shape == (1, 6)  # one action at a time, like LeRobot's policy server
        return value + 1


def test_runner_predict_chunk_postprocesses_every_action():
    runner = PolicyRunner(FakeChunkPolicy(), FakeProcessor(), AddOne(), device="cpu", task="put it")
    chunk = runner.predict_chunk(full_buffer().frame())
    assert chunk.shape == (3, 6)
    np.testing.assert_allclose(chunk, np.arange(6) + np.arange(3)[:, None] + 1)


class _FakePolicy(torch.nn.Module):
    loaded_on = None

    def __init__(self):
        super().__init__()
        self.weight = torch.nn.Parameter(torch.zeros(1))

    @classmethod
    def from_pretrained(cls, path, config):
        cls.loaded_on = config.device
        return cls()


def _patch_lerobot(monkeypatch, config):
    import lerobot.configs.policies as policies_cfg
    import lerobot.policies as policies

    monkeypatch.setattr(policies_cfg.PreTrainedConfig, "from_pretrained", staticmethod(lambda path: config))
    monkeypatch.setattr(policies, "get_policy_class", lambda name: _FakePolicy)
    monkeypatch.setattr(policies, "make_pre_post_processors", lambda **kwargs: (None, None))


def test_load_policy_reads_weights_on_cpu_and_overrides_n_action_steps(monkeypatch):
    from types import SimpleNamespace

    from so101_pai.policy.runner import load_policy

    config = SimpleNamespace(type="act", device="cuda", n_action_steps=100)
    _patch_lerobot(monkeypatch, config)
    runner = load_policy("ckpt", device="cpu", task="t", n_action_steps=20)
    assert _FakePolicy.loaded_on == "cpu"
    assert runner.device == "cpu" and config.device == "cpu"
    assert runner.policy.weight.dtype == torch.float32
    assert config.n_action_steps == 20
