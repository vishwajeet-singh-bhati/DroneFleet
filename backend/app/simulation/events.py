"""Events emitted by the simulation engine. The engine only *reports*; other layers decide."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from app.core.enums import EventType


@dataclass
class SimEvent:
    type: EventType
    sim_time: float
    message: str
    drone_id: str | None = None
    mission_id: str | None = None
    severity: str = "INFO"          # INFO | WARNING | CRITICAL
    data: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "type": self.type.value,
            "sim_time": round(self.sim_time, 3),
            "message": self.message,
            "drone_id": self.drone_id,
            "mission_id": self.mission_id,
            "severity": self.severity,
            "data": self.data,
        }
