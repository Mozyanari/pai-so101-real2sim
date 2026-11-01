"""Key bindings for marking teleop episodes, shared by the sim and the real robot.

Sim:  keys become commands on contract.TOPIC_EPISODE_COMMAND; the sim bridge owns /episode/status
      (it also ends an episode by itself when the task's success check fires or time runs out).
Real: there is no bridge, so keys publish /episode/status directly and the operator resets the scene by hand.

    s  start an episode      y  end it as a success      n  end it as a failure
    r  reset the scene (sim) / move the arm to the rest pose (real eval, while not recording)
"""

from __future__ import annotations

from dataclasses import dataclass

from so101_pai.common import episode

SIM = "sim"
REAL = "real"
COMMAND = "command"
STATUS = "status"
REST = "rest"  # real: the node plays rest_trajectory() on the action topic

SIM_KEYS = {"s": "start", "y": "success", "n": "fail", "r": "reset"}
HELP = "s=start  y=success  n=fail  r=reset  q=quit"


@dataclass(frozen=True)
class Outgoing:
    channel: str  # COMMAND, STATUS or REST
    data: str


class Operator:
    """Turns key presses into messages and keeps track of whether an episode is being recorded."""

    def __init__(self, mode: str, task: str) -> None:
        if mode not in (SIM, REAL):
            raise ValueError(f"mode must be {SIM!r} or {REAL!r}, got {mode!r}")
        self.mode = mode
        self.task = task
        self.recording = False
        self.successes = 0
        self.failures = 0

    def on_key(self, key: str) -> list[Outgoing]:
        key = key.lower()
        if self.mode == SIM:
            command = SIM_KEYS.get(key)
            return [Outgoing(COMMAND, command)] if command else []
        if key == "s" and not self.recording:
            return [self._status(episode.EpisodeEvent(episode.START, task=self.task))]
        if key in ("y", "n") and self.recording:
            return [self._status(episode.EpisodeEvent(episode.END, success=key == "y"))]
        if key == "r" and not self.recording:
            return [Outgoing(REST, "")]
        return []

    def on_status(self, event: episode.EpisodeEvent) -> None:
        """Track episodes from /episode/status (in real mode this sees our own messages)."""
        if event.event == episode.START:
            self.recording = True
        elif event.event == episode.END and self.recording:
            self.recording = False
            if event.success:
                self.successes += 1
            else:
                self.failures += 1

    def summary(self) -> str:
        state = "REC" if self.recording else "idle"
        return f"[{state}] success {self.successes} / fail {self.failures}"

    def _status(self, event: episode.EpisodeEvent) -> Outgoing:
        return Outgoing(STATUS, episode.encode(event))
