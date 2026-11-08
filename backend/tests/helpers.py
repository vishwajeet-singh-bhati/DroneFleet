"""Shared helpers for integration tests."""
from __future__ import annotations

from app.core.enums import FlightMode
from app.mission.models import Mission
from app.services.mission_service import MissionRequest
from app.services.orchestrator import FleetOrchestrator
from app.utils.geometry import distance


def make_orch(seed: int = 42, **kw) -> FleetOrchestrator:
    return FleetOrchestrator(seed=seed, **kw)


def start_mission(o: FleetOrchestrator, zone="ZONE_A", drones=1, **kw) -> Mission:
    m = o.missions.create(MissionRequest(name=f"Inspect {zone}", zone_id=zone, num_drones=drones, **kw))
    assert m.status.value == "READY", m.failure_reason
    o.missions.start(m.mission_id)
    return m


def fly_manual(o: FleetOrchestrator, drone_id: str, ticks: int, **axes) -> None:
    for _ in range(ticks):
        o.control.manual_input(drone_id, axes)
        o.tick()


def run_until_inspected(o: FleetOrchestrator, drone_id: str, count: int, max_ticks=4000) -> None:
    d = o.engine.get(drone_id)
    o.run_until(lambda x: d.current_waypoint_index >= count and d.flight_mode == FlightMode.AUTONOMOUS,
                max_ticks)


def max_step(o: FleetOrchestrator, drone_id: str, ticks: int) -> float:
    """Largest per-tick displacement observed (teleport detector)."""
    d = o.engine.get(drone_id)
    worst = 0.0
    for _ in range(ticks):
        before = d.position
        o.tick()
        worst = max(worst, distance(before, d.position))
    return worst


def events_of(o: FleetOrchestrator, type_: str, drone_id: str | None = None) -> list[dict]:
    return [e for e in o.ctx.events.all() if e["type"] == type_ and (drone_id is None or e["drone_id"] == drone_id)]
