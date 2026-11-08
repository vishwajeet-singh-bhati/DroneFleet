from __future__ import annotations

from fastapi import APIRouter, Depends

from app.api.deps import get_orch
from app.core.enums import DroneStatus, FlightMode
from app.schemas.api import ManualInputRequest, TakeControlRequest
from app.services.errors import NotFound
from app.services.orchestrator import FleetOrchestrator
from app.simulation.failures import clear_faults

router = APIRouter(prefix="/drones", tags=["drones"])


def _drone(o: FleetOrchestrator, drone_id: str):
    d = o.engine.drones.get(drone_id)
    if d is None:
        raise NotFound(f"drone {drone_id}")
    return d


@router.get("", summary="All drones (authoritative backend state)")
async def list_drones(o: FleetOrchestrator = Depends(get_orch)):
    return [d.to_dict() for d in o.engine.drones_iter()]


@router.get("/{drone_id}")
async def get_drone(drone_id: str, o: FleetOrchestrator = Depends(get_orch)):
    d = _drone(o, drone_id)
    out = d.to_dict()
    out["route"] = [w.to_dict() for w in d.route]
    out["control_session"] = bool(o.control.sessions.get(drone_id))
    return out


@router.get("/{drone_id}/telemetry", summary="Persisted telemetry history")
async def telemetry(drone_id: str, limit: int = 300, o: FleetOrchestrator = Depends(get_orch)):
    _drone(o, drone_id)
    if o.ctx.repo is None:
        return []
    rows = o.ctx.repo.telemetry(drone_id, limit=min(limit, 2000))
    return [{"sim_time": r.sim_time, "x": r.position_x, "y": r.position_y, "altitude": r.altitude,
             "battery": r.battery_percentage, "health": r.health_percentage,
             "temperature": r.temperature, "signal": r.signal_strength, "gps": r.gps_quality,
             "mode": r.flight_mode} for r in rows]


@router.post("/{drone_id}/take-control", summary="Autonomous -> manual (operator takeover)")
async def take_control(drone_id: str, body: TakeControlRequest | None = None,
                       o: FleetOrchestrator = Depends(get_orch)):
    return o.control.takeover(drone_id, (body.operator if body else "operator"))


@router.post("/{drone_id}/manual-input", summary="Send operator axes (validated by the backend)")
async def manual_input(drone_id: str, body: ManualInputRequest, o: FleetOrchestrator = Depends(get_orch)):
    return o.control.manual_input(drone_id, body.model_dump())


@router.post("/{drone_id}/release-control",
             summary="Manual -> autonomous: rejoin the mission route and resume")
async def release_control(drone_id: str, body: TakeControlRequest | None = None,
                          o: FleetOrchestrator = Depends(get_orch)):
    return o.control.release_control(drone_id, (body.operator if body else "operator"))


@router.post("/{drone_id}/pause")
async def pause(drone_id: str, o: FleetOrchestrator = Depends(get_orch)):
    return o.control.pause(drone_id)


@router.post("/{drone_id}/resume", summary="Resume a paused drone")
async def resume(drone_id: str, o: FleetOrchestrator = Depends(get_orch)):
    return o.control.resume(drone_id)


@router.post("/{drone_id}/return-home")
async def return_home(drone_id: str, o: FleetOrchestrator = Depends(get_orch)):
    d = _drone(o, drone_id)
    # an operator-ordered RTH takes the drone off its mission, so the mission is reassigned
    res = o.control.return_home(drone_id, "Operator ordered return home", "operator")
    mission = o.ctx.missions.get(d.current_mission_id) if d.current_mission_id else None
    if mission and mission.is_running and drone_id in mission.assigned_drones:
        o.missions.release_and_reassign(mission, d, "operator ordered return home")
    return res


@router.post("/{drone_id}/emergency")
async def emergency(drone_id: str, o: FleetOrchestrator = Depends(get_orch)):
    d = _drone(o, drone_id)
    res = o.control.emergency(drone_id, "Operator emergency", "operator")
    mission = o.ctx.missions.get(d.current_mission_id) if d.current_mission_id else None
    if mission and mission.is_running and drone_id in mission.assigned_drones:
        o.missions.release_and_reassign(mission, d, "operator emergency")
    return res


@router.post("/{drone_id}/reset", summary="Maintenance reset: clear faults, restore health, recharge")
async def reset_drone(drone_id: str, o: FleetOrchestrator = Depends(get_orch)):
    d = _drone(o, drone_id)
    if d.is_airborne or d.flight_mode != FlightMode.IDLE:
        from app.safety.validator import CommandRejected
        raise CommandRejected("INVALID_STATE", f"{drone_id} must be on the ground to be reset")
    clear_faults(d)
    d.status, d.health_percentage, d.battery_percentage = DroneStatus.READY, 100.0, 100.0
    d.temperature = o.settings.sim.ambient_temp
    d.current_mission_id = None
    o.recovery.clear(drone_id)
    o.ctx.log("INFO", f"{drone_id} maintenance reset", drone_id=drone_id)
    return d.to_dict()
