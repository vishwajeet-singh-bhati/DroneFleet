from __future__ import annotations

from fastapi import APIRouter, Depends

from app.api.deps import get_orch
from app.core.enums import Capability
from app.schemas.api import MissionCreate
from app.services.errors import NotFound
from app.services.mission_service import MissionRequest
from app.services.orchestrator import FleetOrchestrator

router = APIRouter(prefix="/missions", tags=["missions"])


def _view(o: FleetOrchestrator, m) -> dict:
    d = m.to_dict()
    d["progress"] = round(o.fleet.mission_progress(m), 3)
    return d


@router.post("", status_code=201, summary="Create a mission (selects drones and plans routes)")
async def create_mission(body: MissionCreate, o: FleetOrchestrator = Depends(get_orch)):
    req = MissionRequest(
        name=body.name, description=body.description, objective=body.objective, priority=body.priority,
        zone_id=body.zone_id, deadline_s=body.deadline_s,
        required_capabilities=frozenset(body.required_capabilities) or frozenset({Capability.CAMERA}),
        num_drones=body.num_drones, include_inspected=body.include_inspected, auto_start=body.auto_start)
    m = o.missions.create(req)
    return _view(o, m)


@router.get("", summary="List missions")
async def list_missions(o: FleetOrchestrator = Depends(get_orch)):
    return [_view(o, m) for m in o.ctx.missions.values()]


@router.get("/{mission_id}", summary="Mission detail including routes and plan")
async def get_mission(mission_id: str, o: FleetOrchestrator = Depends(get_orch)):
    return _view(o, o.missions.get(mission_id))


@router.get("/{mission_id}/report", summary="Mission report (available once finished)")
async def mission_report(mission_id: str, o: FleetOrchestrator = Depends(get_orch)):
    m = o.missions.get(mission_id)
    return m.report or o.missions.build_report(m)


@router.post("/{mission_id}/start")
async def start_mission(mission_id: str, o: FleetOrchestrator = Depends(get_orch)):
    return _view(o, o.missions.start(mission_id))


@router.post("/{mission_id}/pause")
async def pause_mission(mission_id: str, o: FleetOrchestrator = Depends(get_orch)):
    return _view(o, o.missions.pause(mission_id))


@router.post("/{mission_id}/resume")
async def resume_mission(mission_id: str, o: FleetOrchestrator = Depends(get_orch)):
    return _view(o, o.missions.resume(mission_id))


@router.post("/{mission_id}/abort", summary="Cancel the mission; its drones return home")
async def abort_mission(mission_id: str, o: FleetOrchestrator = Depends(get_orch)):
    return _view(o, o.missions.abort(mission_id))
