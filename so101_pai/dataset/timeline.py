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

    def keep_last(self) -> None:
        """Drop everything but the latest message (all zero-order hold needs from the past)."""
        del self.stamps[:-1], self.values[:-1]

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


class EpisodeSplitter:
    """Pairs episode start/end events while streaming through a bag, holding at most one episode of messages.

    Feed messages in time order: add() for sensor streams, event() for episode events, which returns the span
    when an episode closes. A start without an end, or a second start, drops the open episode. Outside an episode
    each stream keeps only its latest message, so the next frame 0 can still sample it. Read `streams` for a span
    right away: trimming resumes with the next add().
    """

    def __init__(self, names) -> None:
        self.streams: dict[str, Timeline] = {name: Timeline() for name in names}
        self._open: tuple[int, str] | None = None

    def add(self, name: str, stamp_ns: int, value: Any) -> None:
        timeline = self.streams[name]
        timeline.append(stamp_ns, value)
        if self._open is None:
            timeline.keep_last()

    def event(self, stamp_ns: int, event: episode.EpisodeEvent) -> EpisodeSpan | None:
        if event.event == episode.START:
            for timeline in self.streams.values():
                timeline.keep_last()
            self._open = (stamp_ns, event.task)
        elif event.event == episode.END and self._open is not None:
            span = EpisodeSpan(self._open[0], stamp_ns, self._open[1], event.success)
            self._open = None
            return span
        return None


def episode_spans(events: list[tuple[int, episode.EpisodeEvent]]) -> list[EpisodeSpan]:
    """Pair start/end events (EpisodeSplitter's rules) from a complete list."""
    splitter = EpisodeSplitter(())
    spans = (splitter.event(stamp_ns, event) for stamp_ns, event in sorted(events, key=lambda item: item[0]))
    return [span for span in spans if span is not None]


def frame_stamps(start_ns: int, end_ns: int, fps: int) -> list[int]:
    """Frame times in [start_ns, end_ns] at a fixed rate."""
    step_ns = 1_000_000_000 / fps
    count = int((end_ns - start_ns) // step_ns) + 1
    return [start_ns + round(i * step_ns) for i in range(max(count, 0))]
