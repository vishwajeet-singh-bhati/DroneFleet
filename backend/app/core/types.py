"""Core value types shared between simulation, mission and planning layers."""
from __future__ import annotations

from dataclasses import dataclass, asdict

from app.core.enums import WaypointAction


@dataclass(frozen=True)
class Waypoint:
    """An ordered route point. `id` is stable so the UI can track route changes."""
    id: str
    x: float
    y: float
    altitude: float = 30.0
    action: WaypointAction = WaypointAction.TRANSIT

    @property
    def xy(self) -> tuple[float, float]:
        return (self.x, self.y)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["action"] = self.action.value
        return d
