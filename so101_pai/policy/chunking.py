"""Asynchronous action chunking: run the next chunk's inference while the current chunk is still playing.

With LeRobot's select_action the arm stops for one inference every n_action_steps actions (~0.1 s for SmolVLA on the
RTX 5060 Ti), and replanning more often makes the motion slower. Here the policy node asks for the next chunk
`lead_steps` actions before the queue runs dry; when the chunk arrives, the actions for the ticks that passed during
inference are dropped, so the arm keeps moving. Plain numpy and tick counts, no threads or ROS.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np


@dataclass
class ChunkScheduler:
    """Plays one action per tick from the latest chunk; says when to start the next inference.

    Action i of a chunk predicted from the observation at tick t is meant for tick t + i.
    """

    n_action_steps: int  # replan after this many actions of a chunk (counted from its observation tick)
    lead_steps: int = 5  # start the next inference when this many actions are left
    tick: int = 0
    pending: bool = False  # an inference is running
    queue: deque = field(default_factory=deque)
    last: np.ndarray | None = None

    def __post_init__(self) -> None:
        if not 0 <= self.lead_steps < self.n_action_steps:
            raise ValueError(f"lead_steps must be in [0, n_action_steps), got {self.lead_steps}")

    def reset(self) -> None:
        self.tick, self.pending, self.last = 0, False, None
        self.queue.clear()

    def should_infer(self) -> bool:
        return not self.pending and len(self.queue) <= self.lead_steps

    def started(self) -> int:
        """Mark an inference as running on the current observation; returns its tick for `receive`."""
        self.pending = True
        return self.tick

    def receive(self, chunk: np.ndarray, obs_tick: int) -> None:
        """Replace the queue with the part of `chunk` (T x D) still ahead of us."""
        self.pending = False
        elapsed = self.tick - obs_tick
        end = max(min(self.n_action_steps, len(chunk)), elapsed + 1)  # if inference ran late, keep one action
        self.queue = deque(np.asarray(chunk)[elapsed:end])

    def step(self) -> np.ndarray | None:
        """Action for this tick: the next queued one, else hold the last (None before the first chunk)."""
        if self.queue:
            self.last = self.queue.popleft()
        self.tick += 1
        return self.last
