"""In-memory event + AI-decision logs with subscriber fan-out.

These are the *observable* record of what the system did and why. Everything the frontend
shows in the live event log and AI decision log comes from here (and is persisted by the
repository).
"""
from __future__ import annotations

import logging
from typing import Any, Callable

from app.core.enums import EventType
from app.simulation.drone_state import sim_iso
from app.utils.logging import EVENT_LOGGER, log_structured

Subscriber = Callable[[dict], None]


def clock_label(sim_time: float) -> str:
    """HH:MM:SS of the simulated clock (starts at 00:00:00)."""
    s = int(sim_time)
    return f"{s // 3600 % 24:02d}:{s // 60 % 60:02d}:{s % 60:02d}"


class EventLog:
    def __init__(self, max_events: int = 5000) -> None:
        self.max_events = max_events
        self._events: list[dict] = []
        self._seq = 0
        self._subs: list[Subscriber] = []

    def subscribe(self, fn: Subscriber) -> None:
        self._subs.append(fn)

    def emit(self, type: EventType | str, message: str, sim_time: float, *,
             drone_id: str | None = None, mission_id: str | None = None,
             severity: str = "INFO", data: dict[str, Any] | None = None) -> dict:
        self._seq += 1
        t = type.value if isinstance(type, EventType) else str(type)
        ev = {
            "id": self._seq, "type": t, "message": message, "severity": severity,
            "drone_id": drone_id, "mission_id": mission_id, "data": data or {},
            "sim_time": round(sim_time, 3), "clock": clock_label(sim_time),
            "timestamp": sim_iso(sim_time),
        }
        self._events.append(ev)
        if len(self._events) > self.max_events:
            del self._events[: len(self._events) - self.max_events]
        level = {"INFO": logging.INFO, "WARNING": logging.WARNING, "CRITICAL": logging.ERROR}.get(
            severity, logging.INFO)
        log_structured(EVENT_LOGGER, level, event=t, message=message, drone_id=drone_id,
                       mission_id=mission_id, sim_time=ev["sim_time"], severity=severity)
        for fn in list(self._subs):
            fn(ev)
        return ev

    def recent(self, limit: int = 100, *, drone_id: str | None = None, mission_id: str | None = None,
               type: str | None = None, since_id: int = 0) -> list[dict]:
        out = [e for e in self._events
               if e["id"] > since_id
               and (drone_id is None or e["drone_id"] == drone_id)
               and (mission_id is None or e["mission_id"] == mission_id)
               and (type is None or e["type"] == type)]
        return out[-limit:]

    def all(self) -> list[dict]:
        return list(self._events)

    def count(self, type: EventType | str) -> int:
        t = type.value if isinstance(type, EventType) else type
        return sum(1 for e in self._events if e["type"] == t)

    def __len__(self) -> int:
        return len(self._events)


class DecisionLog:
    """Explainable AI decisions (selection, reassignment, replanning, rejoin...)."""

    def __init__(self, max_items: int = 1000) -> None:
        self.max_items = max_items
        self._items: list[dict] = []
        self._seq = 0
        self._subs: list[Subscriber] = []

    def subscribe(self, fn: Subscriber) -> None:
        self._subs.append(fn)

    def record(self, kind: str, summary: str, sim_time: float, *, mission_id: str | None = None,
               drone_id: str | None = None, score: float | None = None,
               factors: dict[str, float] | None = None, reasons: list[str] | None = None,
               extra: dict[str, Any] | None = None) -> dict:
        self._seq += 1
        item = {
            "id": self._seq, "kind": kind, "summary": summary, "mission_id": mission_id,
            "drone_id": drone_id, "score": None if score is None else round(score, 4),
            "factors": {k: round(v, 4) for k, v in (factors or {}).items()},
            "reasons": reasons or [], "extra": extra or {},
            "sim_time": round(sim_time, 3), "clock": clock_label(sim_time),
        }
        self._items.append(item)
        if len(self._items) > self.max_items:
            del self._items[: len(self._items) - self.max_items]
        for fn in list(self._subs):
            fn(item)
        return item

    def recent(self, limit: int = 100, *, mission_id: str | None = None) -> list[dict]:
        out = [d for d in self._items if mission_id is None or d["mission_id"] == mission_id]
        return out[-limit:]

    def all(self) -> list[dict]:
        return list(self._items)
