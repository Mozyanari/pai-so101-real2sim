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

    @property
    def n_action_steps(self) -> int:
        return self.policy.config.n_action_steps

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

    def predict_chunk(self, frame: dict[str, np.ndarray]) -> np.ndarray:
        """The whole predicted action chunk (T x D, motor units) for one frame; for asynchronous inference."""
        import torch
        from lerobot.policies.utils import prepare_observation_for_inference

        with torch.inference_mode():
            batch = prepare_observation_for_inference(
                dict(frame), torch.device(self.device), self.task, "so101_follower"
            )
            chunk = self.policy.predict_action_chunk(self.preprocessor(batch))
            # The postprocessor (unnormalize) takes one action at a time, as in LeRobot's policy server.
            actions = [self.postprocessor(chunk[:, i]) for i in range(chunk.shape[1])]
        return torch.cat(actions).float().cpu().numpy()


def cast_parameters(module, dtype) -> None:
    """Cast floating-point parameters only. Buffers such as RoPE's inv_freq keep their precision, which
    `module.to(dtype)` would not do (same approach as LeRobot's own fp32 cast in modeling_groot)."""
    for parameter in module.parameters():
        if parameter.is_floating_point():
            parameter.data = parameter.data.to(dtype)


def load_policy(path: str, device: str, task: str, n_action_steps: int = 0) -> PolicyRunner:
    """Load a checkpoint saved by lerobot-train (e.g. outputs/act/checkpoints/last/pretrained_model) or a Hub id.

    n_action_steps > 0 overrides how many actions of each predicted chunk run before the policy looks again
    (fewer = more closed-loop; must not exceed the chunk size).

    Weights are read on the CPU and then moved, so a large fp32 state dict never sits on the GPU next to the model.
    GR00T trained with LeRobot's default recipe keeps fp32 parameters (12.6 GB for its 3.1B); for inference every
    parameter goes to bf16 (6.3 GB, 6.4 GB peak in a chunk), leaving room for Isaac Sim on the 16 GB card. Training
    already computed under bf16 autocast, so only norms and embeddings lose precision.
    """
    import torch
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies import get_policy_class, make_pre_post_processors

    config = PreTrainedConfig.from_pretrained(path)
    config.device = "cpu"
    if n_action_steps > 0:
        config.n_action_steps = n_action_steps
    if config.type == "groot":
        config.model_params_fp32 = False  # otherwise LeRobot casts the model up to fp32 on construction
    policy = get_policy_class(config.type).from_pretrained(path, config=config)
    if config.type == "groot":
        cast_parameters(policy, torch.bfloat16)  # the frozen VLM backbone still loads in fp32
    config.device = device
    policy.to(device).eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=config,
        pretrained_path=path,
        preprocessor_overrides={"device_processor": {"device": device}},
    )
    return PolicyRunner(policy, preprocessor, postprocessor, device, task)
