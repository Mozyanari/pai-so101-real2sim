"""Episode state of a simulated robot: what to do with operator commands and env terminations (no Isaac/ROS).

The sim bridge feeds it /episode/command strings and env done flags, then carries out the returned Decision:
publish the events on /episode/status, reset the scene, and/or restart the env's episode timer.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from so101_pai.common import episode

COMMANDS = ("start", "success", "fail", "reset")


@dataclass(frozen=True)
class Decision:
    events: tuple[episode.EpisodeEvent, ...] = ()
    reset_scene: bool = False
    restart_timer: bool = False


@dataclass
class EpisodeControl:
    task: str
    recording: bool = field(default=False)

    def on_command(self, command: str) -> Decision:
        if command not in COMMANDS:
            raise ValueError(f"unknown episode command {command!r}; expected one of {COMMANDS}")
        if command == "start":
            if self.recording:
                return Decision()
            self.recording = True
            # The env's timeout counts from its last reset; restart it so every episode gets the full time.
            return Decision(events=(episode.EpisodeEvent(episode.START, task=self.task),), restart_timer=True)
        if command == "reset":
            return Decision(reset_scene=not self.recording)
        # success / fail: end the episode if one is running, and always give the operator a fresh scene.
        return Decision(events=self._end(command == "success"), reset_scene=True)

    def on_env_done(self, success: bool) -> Decision:
        """The env terminated (success check) or timed out; it has already reset itself."""
        return Decision(events=self._end(success))

    def _end(self, success: bool) -> tuple[episode.EpisodeEvent, ...]:
        if not self.recording:
            return ()
        self.recording = False
        return (episode.EpisodeEvent(episode.END, success=success),)
