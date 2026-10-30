"""Run a LeRobot policy on observations from the ROS 2 contract (no ROS imports, so it is unit-testable).

Observations and actions use the dataset convention: LeRobot motor units and HxWx3 uint8 RGB images.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from so101_pai.common import contract

STATE_KEY = "observation.state"


def image_key(camera: str) -> str:
    return f"observation.images.{camera}"


@dataclass
class ObservationBuffer:
    """Latest joint state and camera frames; a frame is ready once every input has arrived."""

    cameras: tuple[str, ...] = tuple(contract.CAMERA_TOPICS)
    state: np.ndarray | None = None
    images: dict[str, np.ndarray] = field(default_factory=dict)

    def ready(self) -> bool:
        return self.state is not None and all(camera in self.images for camera in self.cameras)

    def frame(self) -> dict[str, np.ndarray]:
        if not self.ready():
            raise RuntimeError("observation incomplete: waiting for state and all cameras")
        frame = {STATE_KEY: np.asarray(self.state, dtype=np.float32)}
        frame.update({image_key(camera): self.images[camera] for camera in self.cameras})
        return frame


class PolicyRunner:
    """Wraps policy + LeRobot pre/post processors: one observation frame in, one motor-unit action out."""

    def __init__(self, policy, preprocessor: Callable, postprocessor: Callable, device: str, task: str) -> None:
        self.policy = policy
        self.preprocessor = preprocessor
        self.postprocessor = postprocessor
        self.device = device
        self.task = task

    def reset(self) -> None:
        """Clear action chunks and observation history; call at every episode start."""
        self.policy.reset()
        for processor in (self.preprocessor, self.postprocessor):
            if hasattr(processor, "reset"):
                processor.reset()

    def act(self, frame: dict[str, np.ndarray]) -> np.ndarray:
        import torch
        from lerobot.policies.utils import prepare_observation_for_inference

        with torch.inference_mode():
            batch = prepare_observation_for_inference(
                dict(frame), torch.device(self.device), self.task, "so101_follower"
            )
            action = self.postprocessor(self.policy.select_action(self.preprocessor(batch)))
        return action.squeeze(0).float().cpu().numpy()


def load_policy(path: str, device: str, task: str, n_action_steps: int = 0) -> PolicyRunner:
    """Load a checkpoint saved by lerobot-train (e.g. outputs/act/checkpoints/last/pretrained_model) or a Hub id.

    n_action_steps > 0 overrides how many actions of each predicted chunk run before the policy looks again
    (fewer = more closed-loop; must not exceed the chunk size).
    """
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies import get_policy_class, make_pre_post_processors

    config = PreTrainedConfig.from_pretrained(path)
    config.device = device
    if n_action_steps > 0:
        config.n_action_steps = n_action_steps
    policy = get_policy_class(config.type).from_pretrained(path, config=config)
    policy.to(device).eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config,
        pretrained_path=path,
        preprocessor_overrides={"device_processor": {"device": device}},
    )
    return PolicyRunner(policy, preprocessor, postprocessor, device, task)
