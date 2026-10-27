"""Episode status messages published on contract.TOPIC_EPISODE as std_msgs/String JSON.

{"event": "start", "task": "Put the red block on the plate"}
{"event": "end", "success": true}
"""

from __future__ import annotations

import json
from dataclasses import dataclass

START = "start"
END = "end"


@dataclass(frozen=True)
class EpisodeEvent:
    event: str
    task: str = ""
    success: bool = False


def encode(event: EpisodeEvent) -> str:
    if event.event == START:
        return json.dumps({"event": START, "task": event.task})
    if event.event == END:
        return json.dumps({"event": END, "success": event.success})
    raise ValueError(f"unknown episode event: {event.event!r}")


def decode(text: str) -> EpisodeEvent:
    data = json.loads(text)
    event = data.get("event")
    if event == START:
        return EpisodeEvent(START, task=str(data.get("task", "")))
    if event == END:
        return EpisodeEvent(END, success=bool(data.get("success", False)))
    raise ValueError(f"unknown episode event: {event!r}")
