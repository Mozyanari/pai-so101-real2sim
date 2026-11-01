"""Trial bookkeeping for policy evaluation (no ROS imports).

In the sim the session drives the bridge on its own: reset -> wait for the scene to settle -> start -> wait for
the bridge's end event (task success check or episode timeout), with its own timeout as a backstop.
On the real robot a person starts and judges each trial with the operator node; the session only records.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

from so101_pai.common import episode

WAIT_RESET = "wait_reset"
RUNNING = "running"
DONE = "done"


@dataclass(frozen=True)
class Trial:
    index: int
    success: bool
    duration_s: float


@dataclass
class EvalSession:
    trials: int
    settle_s: float = 2.0  # let the reset scene settle before starting
    timeout_s: float = 60.0  # backstop; the sim itself ends an episode after 45 s
    results: list[Trial] = field(default_factory=list)
    phase: str = WAIT_RESET
    _phase_since: float | None = None
    _started_at: float = 0.0

    @property
    def done(self) -> bool:
        return self.phase == DONE

    def next_command(self, now: float) -> str | None:
        """Command to send to the sim bridge at time `now` (seconds), if any."""
        if self.done:
            return None
        if self._phase_since is None:
            self._phase_since = now
            return "reset"
        if self.phase == WAIT_RESET and now - self._phase_since >= self.settle_s:
            self._enter(RUNNING, now)
            self._started_at = now  # refined by the bridge's start event
            return "start"
        if self.phase == RUNNING and now - self._phase_since >= self.timeout_s:
            self._enter(WAIT_RESET, now)  # the "fail" end event is recorded via on_status
            return "fail"
        return None

    def on_status(self, event: episode.EpisodeEvent, now: float) -> None:
        if self.phase == DONE:
            return
        if event.event == episode.START:
            self._started_at = now
            return
        self.results.append(Trial(len(self.results), event.success, round(now - self._started_at, 2)))
        # The bridge resets the scene after every end, so the next trial only needs to wait for it to settle.
        self._enter(DONE if len(self.results) >= self.trials else WAIT_RESET, now)

    def _enter(self, phase: str, now: float) -> None:
        self.phase = phase
        self._phase_since = now


def summarize(trials: list[Trial]) -> dict:
    successes = sum(trial.success for trial in trials)
    return {
        "trials": len(trials),
        "successes": successes,
        "success_rate": successes / len(trials) if trials else 0.0,
        "results": [asdict(trial) for trial in trials],
    }


def write_summary(path: Path, trials: list[Trial], meta: dict) -> dict:
    summary = {**meta, **summarize(trials)}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    return summary
