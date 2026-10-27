"""Pure-Python helpers for turning asynchronous topic streams into fixed-rate frames."""

from __future__ import annotations

import bisect
from dataclasses import dataclass, field
from typing import Any

from so101_pai.common import episode


@dataclass
class Timeline:
    """Messages of one topic, sampled with zero-order hold (latest value at or before t)."""

    stamps: list[int] = field(default_factory=list)
    values: list[Any] = field(default_factory=list)

    def append(self, stamp_ns: int, value: Any) -> None:
        if self.stamps and stamp_ns < self.stamps[-1]:
            index = bisect.bisect_right(self.stamps, stamp_ns)
            self.stamps.insert(index, stamp_ns)
            self.values.insert(index, value)
            return
        self.stamps.append(stamp_ns)
        self.values.append(value)

    def first_stamp(self) -> int | None:
        return self.stamps[0] if self.stamps else None

    def sample(self, stamp_ns: int) -> Any | None:
        index = bisect.bisect_right(self.stamps, stamp_ns) - 1
        return self.values[index] if index >= 0 else None


@dataclass(frozen=True)
class EpisodeSpan:
    start_ns: int
    end_ns: int
    task: str
    success: bool


def episode_spans(events: list[tuple[int, episode.EpisodeEvent]]) -> list[EpisodeSpan]:
    """Pair start/end events. A start without an end, or a second start, drops the open episode."""
    spans: list[EpisodeSpan] = []
    open_start: tuple[int, str] | None = None
    for stamp_ns, event in sorted(events, key=lambda item: item[0]):
        if event.event == episode.START:
            open_start = (stamp_ns, event.task)
        elif event.event == episode.END and open_start is not None:
            spans.append(EpisodeSpan(open_start[0], stamp_ns, open_start[1], event.success))
            open_start = None
    return spans


def frame_stamps(start_ns: int, end_ns: int, fps: int) -> list[int]:
    """Frame times in [start_ns, end_ns] at a fixed rate."""
    step_ns = 1_000_000_000 / fps
    count = int((end_ns - start_ns) // step_ns) + 1
    return [start_ns + round(i * step_ns) for i in range(max(count, 0))]
