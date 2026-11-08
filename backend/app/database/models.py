"""SQLAlchemy ORM models for persisted entities."""
from __future__ import annotations

from sqlalchemy import JSON, Float, ForeignKey, Integer, String, Text
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column


class Base(DeclarativeBase):
    pass


class DroneRow(Base):
    __tablename__ = "drones"
    drone_id: Mapped[str] = mapped_column(String(16), primary_key=True)
    capabilities: Mapped[list] = mapped_column(JSON, default=list)
    reliability: Mapped[float] = mapped_column(Float, default=0.95)
    home_x: Mapped[float] = mapped_column(Float, default=0.0)
    home_y: Mapped[float] = mapped_column(Float, default=0.0)
    flight_hours: Mapped[float] = mapped_column(Float, default=0.0)
    battery_cycles: Mapped[int] = mapped_column(Integer, default=0)
    last_state: Mapped[dict] = mapped_column(JSON, default=dict)


class MissionRow(Base):
    __tablename__ = "missions"
    mission_id: Mapped[str] = mapped_column(String(32), primary_key=True)
    name: Mapped[str] = mapped_column(String(200))
    description: Mapped[str] = mapped_column(Text, default="")
    objective: Mapped[str] = mapped_column(String(64), default="inspection")
    priority: Mapped[int] = mapped_column(Integer, default=3)
    target_zone_id: Mapped[str | None] = mapped_column(String(64), nullable=True)
    deadline_s: Mapped[float | None] = mapped_column(Float, nullable=True)
    status: Mapped[str] = mapped_column(String(16))
    required_capabilities: Mapped[list] = mapped_column(JSON, default=list)
    assigned_drones: Mapped[list] = mapped_column(JSON, default=list)
    created_at: Mapped[float] = mapped_column(Float, default=0.0)
    started_at: Mapped[float | None] = mapped_column(Float, nullable=True)
    completed_at: Mapped[float | None] = mapped_column(Float, nullable=True)
    route_revision: Mapped[int] = mapped_column(Integer, default=0)
    history: Mapped[list] = mapped_column(JSON, default=list)
    failure_reason: Mapped[str | None] = mapped_column(Text, nullable=True)


class WaypointRow(Base):
    __tablename__ = "waypoints"
    pk: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    mission_id: Mapped[str] = mapped_column(ForeignKey("missions.mission_id"), index=True)
    drone_id: Mapped[str] = mapped_column(String(16))
    revision: Mapped[int] = mapped_column(Integer, default=0)
    seq: Mapped[int] = mapped_column(Integer)
    waypoint_id: Mapped[str] = mapped_column(String(64))
    x: Mapped[float] = mapped_column(Float)
    y: Mapped[float] = mapped_column(Float)
    altitude: Mapped[float] = mapped_column(Float)
    action: Mapped[str] = mapped_column(String(16))


class TelemetryRow(Base):
    __tablename__ = "telemetry"
    pk: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    drone_id: Mapped[str] = mapped_column(String(16), index=True)
    sim_time: Mapped[float] = mapped_column(Float, index=True)
    position_x: Mapped[float] = mapped_column(Float)
    position_y: Mapped[float] = mapped_column(Float)
    altitude: Mapped[float] = mapped_column(Float)
    velocity: Mapped[float] = mapped_column(Float)
    heading: Mapped[float] = mapped_column(Float)
    battery_percentage: Mapped[float] = mapped_column(Float)
    health_percentage: Mapped[float] = mapped_column(Float)
    temperature: Mapped[float] = mapped_column(Float)
    gps_quality: Mapped[float] = mapped_column(Float)
    signal_strength: Mapped[float] = mapped_column(Float)
    flight_mode: Mapped[str] = mapped_column(String(16))
    status: Mapped[str] = mapped_column(String(16))
    mission_id: Mapped[str | None] = mapped_column(String(32), nullable=True)
    waypoint_index: Mapped[int] = mapped_column(Integer, default=0)


class EventRow(Base):
    __tablename__ = "events"
    pk: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    seq: Mapped[int] = mapped_column(Integer, index=True)
    type: Mapped[str] = mapped_column(String(48), index=True)
    message: Mapped[str] = mapped_column(Text)
    severity: Mapped[str] = mapped_column(String(12), default="INFO")
    drone_id: Mapped[str | None] = mapped_column(String(16), nullable=True, index=True)
    mission_id: Mapped[str | None] = mapped_column(String(32), nullable=True, index=True)
    sim_time: Mapped[float] = mapped_column(Float)
    data: Mapped[dict] = mapped_column(JSON, default=dict)


class AlertRow(Base):
    __tablename__ = "alerts"
    pk: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    sim_time: Mapped[float] = mapped_column(Float)
    severity: Mapped[str] = mapped_column(String(12))
    drone_id: Mapped[str | None] = mapped_column(String(16), nullable=True)
    mission_id: Mapped[str | None] = mapped_column(String(32), nullable=True)
    kind: Mapped[str] = mapped_column(String(48))
    message: Mapped[str] = mapped_column(Text)


class MissionAssignmentRow(Base):
    __tablename__ = "mission_assignments"
    pk: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    mission_id: Mapped[str] = mapped_column(ForeignKey("missions.mission_id"), index=True)
    drone_id: Mapped[str] = mapped_column(String(16))
    role: Mapped[str] = mapped_column(String(16), default="PRIMARY")   # PRIMARY | REPLACEMENT
    assigned_at: Mapped[float] = mapped_column(Float)
    released_at: Mapped[float | None] = mapped_column(Float, nullable=True)
    reason: Mapped[str] = mapped_column(Text, default="")


class ControlSessionRow(Base):
    __tablename__ = "control_sessions"
    pk: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    drone_id: Mapped[str] = mapped_column(String(16), index=True)
    operator: Mapped[str] = mapped_column(String(64), default="operator")
    started_at: Mapped[float] = mapped_column(Float)
    ended_at: Mapped[float | None] = mapped_column(Float, nullable=True)
    end_reason: Mapped[str | None] = mapped_column(Text, nullable=True)
    commands_accepted: Mapped[int] = mapped_column(Integer, default=0)
    commands_rejected: Mapped[int] = mapped_column(Integer, default=0)


class AIDecisionRow(Base):
    __tablename__ = "ai_decisions"
    pk: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    seq: Mapped[int] = mapped_column(Integer)
    kind: Mapped[str] = mapped_column(String(48))
    summary: Mapped[str] = mapped_column(Text)
    mission_id: Mapped[str | None] = mapped_column(String(32), nullable=True)
    drone_id: Mapped[str | None] = mapped_column(String(16), nullable=True)
    score: Mapped[float | None] = mapped_column(Float, nullable=True)
    factors: Mapped[dict] = mapped_column(JSON, default=dict)
    reasons: Mapped[list] = mapped_column(JSON, default=list)
    sim_time: Mapped[float] = mapped_column(Float)
