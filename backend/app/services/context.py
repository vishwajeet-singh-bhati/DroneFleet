"""Shared runtime context passed to every service (a lightweight dependency container)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from app.core.config import Settings
from app.core.enums import EventType
from app.core.event_log import DecisionLog, EventLog
from app.database.repository import Repository
from app.mission.coverage import CoverageMap, InspectionTarget
from app.mission.grid import OccupancyGrid
from app.mission.models import Mission
from app.mission.route_planner import RoutePlanner
from app.simulation.engine import SimulationEngine


@dataclass
class Context:
    settings: Settings
    engine: SimulationEngine
    grid: OccupancyGrid
    planner: RoutePlanner
    events: EventLog
    decisions: DecisionLog
    coverage: CoverageMap
    repo: Repository | None = None
    missions: dict[str, Mission] = field(default_factory=dict)
    # mission_id -> drone_id -> inspection targets planned for that drone (in order)
    mission_targets: dict[str, dict[str, list[InspectionTarget]]] = field(default_factory=dict)
    mission_seq: int = 0
    # filled by the optional maintenance/anomaly extension (empty when ML is disabled)
    maintenance_view: dict[str, dict] = field(default_factory=dict)
    maintenance_risk: dict[str, float] = field(default_factory=dict)
    # drone_id -> vertical cruise layer currently held (released when the drone is idle again)
    altitude_layers: dict[str, int] = field(default_factory=dict)

    @property
    def now(self) -> float:
        return self.engine.time

    def log(self, type: EventType | str, message: str, *, drone_id: str | None = None,
            mission_id: str | None = None, severity: str = "INFO",
            data: dict[str, Any] | None = None) -> dict:
        ev = self.events.emit(type, message, self.now, drone_id=drone_id, mission_id=mission_id,
                              severity=severity, data=data)
        if self.repo:
            self.repo.save_event(ev)
        return ev

    def decide(self, kind: str, summary: str, *, mission_id: str | None = None,
               drone_id: str | None = None, score: float | None = None,
               factors: dict[str, float] | None = None, reasons: list[str] | None = None,
               extra: dict[str, Any] | None = None) -> dict:
        item = self.decisions.record(kind, summary, self.now, mission_id=mission_id, drone_id=drone_id,
                                     score=score, factors=factors, reasons=reasons, extra=extra)
        if self.repo:
            self.repo.save_decision(item)
        self.log(EventType.AI_DECISION, summary, drone_id=drone_id, mission_id=mission_id,
                 data={"decision_id": item["id"], "kind": kind, "score": item["score"]})
        return item

    def persist_mission(self, m: Mission) -> None:
        if self.repo:
            self.repo.upsert_mission(m.to_dict())

    def next_mission_id(self) -> str:
        self.mission_seq += 1
        return f"MISSION-{self.mission_seq:03d}"
