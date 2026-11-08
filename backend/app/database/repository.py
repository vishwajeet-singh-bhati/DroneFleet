"""Buffered repository.

Writes are queued (cheap, no I/O) and committed in one transaction by `flush()`. The
simulation loop flushes periodically; reads flush first so they are always consistent.
Upserts (missions, drones) are keyed by primary key.
"""
from __future__ import annotations

import threading
from typing import Any

from sqlalchemy import select
from sqlalchemy.engine import Engine
from sqlalchemy.orm import Session, sessionmaker

from app.database.models import (
    AIDecisionRow, AlertRow, ControlSessionRow, DroneRow, EventRow, MissionAssignmentRow,
    MissionRow, TelemetryRow, WaypointRow,
)
from app.database.session import make_engine, make_session_factory


class Repository:
    def __init__(self, engine: Engine | None = None, url: str = "sqlite:///:memory:") -> None:
        self.engine = engine or make_engine(url)
        self._sf: sessionmaker[Session] = make_session_factory(self.engine)
        self._queue: list[Any] = []
        self._lock = threading.Lock()

    def reset(self) -> None:
        """Drop and recreate all tables (used by simulation reset)."""
        from app.database.models import Base
        with self._lock:
            self._queue = []
        Base.metadata.drop_all(self.engine)
        Base.metadata.create_all(self.engine)

    # --------------------------------------------------------------- writes (buffered)
    def _enqueue(self, *rows: Any) -> None:
        with self._lock:
            self._queue.extend(rows)

    def flush(self) -> int:
        with self._lock:
            rows, self._queue = self._queue, []
        if not rows:
            return 0
        with self._sf() as s:
            for r in rows:
                s.merge(r) if self._is_keyed(r) else s.add(r)
            s.commit()
        return len(rows)

    @staticmethod
    def _is_keyed(row: Any) -> bool:
        return isinstance(row, (MissionRow, DroneRow))

    def save_event(self, ev: dict) -> None:
        self._enqueue(EventRow(seq=ev["id"], type=ev["type"], message=ev["message"],
                               severity=ev["severity"], drone_id=ev["drone_id"],
                               mission_id=ev["mission_id"], sim_time=ev["sim_time"], data=ev["data"]))
        if ev["severity"] in ("WARNING", "CRITICAL"):
            self._enqueue(AlertRow(sim_time=ev["sim_time"], severity=ev["severity"],
                                   drone_id=ev["drone_id"], mission_id=ev["mission_id"],
                                   kind=ev["type"], message=ev["message"]))

    def save_decision(self, d: dict) -> None:
        self._enqueue(AIDecisionRow(seq=d["id"], kind=d["kind"], summary=d["summary"],
                                    mission_id=d["mission_id"], drone_id=d["drone_id"],
                                    score=d["score"], factors=d["factors"], reasons=d["reasons"],
                                    sim_time=d["sim_time"]))

    def save_telemetry(self, drone_dicts: list[dict], sim_time: float) -> None:
        self._enqueue(*[TelemetryRow(
            drone_id=d["drone_id"], sim_time=sim_time, position_x=d["position_x"],
            position_y=d["position_y"], altitude=d["altitude"], velocity=d["velocity"],
            heading=d["heading"], battery_percentage=d["battery_percentage"],
            health_percentage=d["health_percentage"], temperature=d["temperature"],
            gps_quality=d["gps_quality"], signal_strength=d["signal_strength"],
            flight_mode=d["flight_mode"], status=d["status"], mission_id=d["current_mission_id"],
            waypoint_index=d["current_waypoint_index"]) for d in drone_dicts])

    def upsert_drone(self, d: dict) -> None:
        self._enqueue(DroneRow(drone_id=d["drone_id"], capabilities=d["capabilities"],
                               reliability=d["reliability"], home_x=d["home"]["x"],
                               home_y=d["home"]["y"], last_state=d))

    def upsert_mission(self, m: dict) -> None:
        self._enqueue(MissionRow(
            mission_id=m["mission_id"], name=m["name"], description=m["description"],
            objective=m["objective"], priority=m["priority"], target_zone_id=m["target_zone_id"],
            deadline_s=m["deadline_s"], status=m["status"],
            required_capabilities=m["required_capabilities"], assigned_drones=m["assigned_drones"],
            created_at=m["created_at"], started_at=m["started_at"], completed_at=m["completed_at"],
            route_revision=m["route_revision"], history=m["history"],
            failure_reason=m["failure_reason"]))

    def save_waypoints(self, mission_id: str, drone_id: str, revision: int, route: list[dict]) -> None:
        # Mission row must exist before waypoints (FK). Flush ordering is preserved by the queue.
        self._enqueue(*[WaypointRow(mission_id=mission_id, drone_id=drone_id, revision=revision,
                                    seq=i, waypoint_id=w["id"], x=w["x"], y=w["y"],
                                    altitude=w["altitude"], action=w["action"])
                        for i, w in enumerate(route)])

    def save_assignment(self, mission_id: str, drone_id: str, sim_time: float, role: str = "PRIMARY",
                        reason: str = "") -> None:
        self._enqueue(MissionAssignmentRow(mission_id=mission_id, drone_id=drone_id,
                                           assigned_at=sim_time, role=role, reason=reason))

    def release_assignment(self, mission_id: str, drone_id: str, sim_time: float, reason: str) -> None:
        self.flush()
        with self._sf() as s:
            row = s.scalars(select(MissionAssignmentRow).where(
                MissionAssignmentRow.mission_id == mission_id,
                MissionAssignmentRow.drone_id == drone_id,
                MissionAssignmentRow.released_at.is_(None))).first()
            if row:
                row.released_at, row.reason = sim_time, (row.reason + " | " + reason).strip(" |")
                s.commit()

    def save_control_session(self, drone_id: str, operator: str, started_at: float, ended_at: float | None,
                             end_reason: str | None, accepted: int, rejected: int) -> None:
        self._enqueue(ControlSessionRow(drone_id=drone_id, operator=operator, started_at=started_at,
                                        ended_at=ended_at, end_reason=end_reason,
                                        commands_accepted=accepted, commands_rejected=rejected))

    # --------------------------------------------------------------- reads
    def _all(self, model, order=None, limit: int | None = None, **filters):
        self.flush()
        with self._sf() as s:
            q = select(model)
            for k, v in filters.items():
                if v is not None:
                    q = q.where(getattr(model, k) == v)
            if order is not None:
                q = q.order_by(order)
            if limit:
                q = q.limit(limit)
            return list(s.scalars(q))

    def events(self, limit: int = 200, drone_id: str | None = None, mission_id: str | None = None,
               type: str | None = None) -> list[EventRow]:
        rows = self._all(EventRow, order=EventRow.pk.desc(), limit=limit, drone_id=drone_id,
                         mission_id=mission_id, type=type)
        return list(reversed(rows))

    def missions(self) -> list[MissionRow]:
        return self._all(MissionRow, order=MissionRow.created_at)

    def mission(self, mission_id: str) -> MissionRow | None:
        rows = self._all(MissionRow, mission_id=mission_id)
        return rows[0] if rows else None

    def waypoints(self, mission_id: str, revision: int | None = None) -> list[WaypointRow]:
        return self._all(WaypointRow, order=WaypointRow.pk, mission_id=mission_id, revision=revision)

    def telemetry(self, drone_id: str, limit: int = 500) -> list[TelemetryRow]:
        rows = self._all(TelemetryRow, order=TelemetryRow.pk.desc(), limit=limit, drone_id=drone_id)
        return list(reversed(rows))

    def alerts(self, limit: int = 200) -> list[AlertRow]:
        return list(reversed(self._all(AlertRow, order=AlertRow.pk.desc(), limit=limit)))

    def assignments(self, mission_id: str | None = None) -> list[MissionAssignmentRow]:
        return self._all(MissionAssignmentRow, order=MissionAssignmentRow.pk, mission_id=mission_id)

    def control_sessions(self, drone_id: str | None = None) -> list[ControlSessionRow]:
        return self._all(ControlSessionRow, order=ControlSessionRow.pk, drone_id=drone_id)

    def decisions(self, limit: int = 200, mission_id: str | None = None) -> list[AIDecisionRow]:
        rows = self._all(AIDecisionRow, order=AIDecisionRow.pk.desc(), limit=limit, mission_id=mission_id)
        return list(reversed(rows))

    def drones(self) -> list[DroneRow]:
        return self._all(DroneRow, order=DroneRow.drone_id)
