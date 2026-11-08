"""Mission domain object and its explicit state machine."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from app.core.enums import Capability, MissionStatus
from app.core.types import Waypoint


class InvalidTransition(Exception):
    """Raised when a state change is not allowed by the state machine."""


S = MissionStatus
ALLOWED_TRANSITIONS: dict[MissionStatus, set[MissionStatus]] = {
    S.CREATED: {S.PLANNING, S.ABORTED},
    S.PLANNING: {S.READY, S.FAILED, S.ABORTED},
    S.READY: {S.ACTIVE, S.PLANNING, S.ABORTED},
    S.ACTIVE: {S.PAUSED, S.REPLANNING, S.COMPLETED, S.FAILED, S.ABORTED},
    S.PAUSED: {S.ACTIVE, S.REPLANNING, S.ABORTED, S.FAILED},
    S.REPLANNING: {S.ACTIVE, S.PAUSED, S.FAILED, S.ABORTED},
    S.COMPLETED: set(),
    S.FAILED: set(),
    S.ABORTED: set(),
}
TERMINAL = {S.COMPLETED, S.FAILED, S.ABORTED}


@dataclass
class Mission:
    mission_id: str
    name: str
    description: str = ""
    objective: str = "inspection"
    priority: int = 3                       # 1 (lowest) .. 5 (highest)
    target_zone_id: str | None = None
    deadline_s: float | None = None         # seconds after start
    required_capabilities: frozenset[Capability] = frozenset({Capability.CAMERA})
    num_drones: int = 1
    assigned_drones: list[str] = field(default_factory=list)
    routes: dict[str, list[Waypoint]] = field(default_factory=dict)   # drone_id -> ordered route
    status: MissionStatus = MissionStatus.CREATED
    created_at: float = 0.0
    started_at: float | None = None
    completed_at: float | None = None
    include_inspected: bool = False
    route_revision: int = 0
    history: list[dict[str, Any]] = field(default_factory=list)
    target_ids: dict[str, list[str]] = field(default_factory=dict)   # drone_id -> inspection target ids
    failure_reason: str | None = None
    report: dict[str, Any] | None = None
    plan_info: dict[str, Any] = field(default_factory=dict)
    baseline: dict[str, dict[str, float]] = field(default_factory=dict)   # drone_id -> start metrics
    route_history: list[dict[str, Any]] = field(default_factory=list)     # replan records (old/new)
    done_drones: set[str] = field(default_factory=set)                    # drones that finished + landed
    released_drones: list[str] = field(default_factory=list)              # removed (failed / RTH)

    # ---- state machine
    def transition(self, new: MissionStatus, sim_time: float, reason: str = "") -> None:
        if new == self.status:
            raise InvalidTransition(f"{self.mission_id} already {new.value}")
        if new not in ALLOWED_TRANSITIONS[self.status]:
            raise InvalidTransition(f"{self.mission_id}: {self.status.value} -> {new.value} not allowed")
        self.history.append({"from": self.status.value, "to": new.value, "t": round(sim_time, 3),
                             "reason": reason})
        self.status = new
        if new == S.ACTIVE and self.started_at is None:
            self.started_at = sim_time
        if new in TERMINAL:
            self.completed_at = sim_time
            if new == S.FAILED:
                self.failure_reason = reason

    def can_transition(self, new: MissionStatus) -> bool:
        return new in ALLOWED_TRANSITIONS[self.status]

    @property
    def is_terminal(self) -> bool:
        return self.status in TERMINAL

    @property
    def is_running(self) -> bool:
        return self.status in (S.ACTIVE, S.PAUSED, S.REPLANNING)

    def deadline_at(self) -> float | None:
        if self.deadline_s is None or self.started_at is None:
            return None
        return self.started_at + self.deadline_s

    def to_dict(self) -> dict:
        return {
            "mission_id": self.mission_id, "name": self.name, "description": self.description,
            "objective": self.objective, "priority": self.priority,
            "target_zone_id": self.target_zone_id, "deadline_s": self.deadline_s,
            "required_capabilities": sorted(c.value for c in self.required_capabilities),
            "num_drones": self.num_drones, "assigned_drones": list(self.assigned_drones),
            "routes": {k: [w.to_dict() for w in v] for k, v in self.routes.items()},
            "status": self.status.value, "created_at": self.created_at,
            "started_at": self.started_at, "completed_at": self.completed_at,
            "route_revision": self.route_revision, "history": self.history,
            "failure_reason": self.failure_reason, "report": self.report, "plan_info": self.plan_info,
            "released_drones": list(self.released_drones),
            "route_history": self.route_history[-20:],
        }
