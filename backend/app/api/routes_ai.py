from __future__ import annotations

from fastapi import APIRouter, Depends, Request

from app.api.deps import AppState, get_orch, get_state
from app.ai.interpreter import ZoneInfo
from app.core.enums import ZoneKind
from app.schemas.api import InterpretRequest, MissionCreate
from app.services.errors import NotFound
from app.services.mission_service import MissionRequest
from app.services.orchestrator import FleetOrchestrator
from app.core.enums import Capability

router = APIRouter(tags=["ai"])


def _zones(o: FleetOrchestrator) -> list[ZoneInfo]:
    return [ZoneInfo(z.zone_id, z.name or z.zone_id) for z in o.ctx.grid.zones.values()
            if z.kind == ZoneKind.INSPECTION]


@router.post("/missions/interpret", summary="Natural language -> validated mission request (never controls drones)")
async def interpret(body: InterpretRequest, request: Request, s: AppState = Depends(get_state)):
    import asyncio
    interp = request.app.state.interpreter
    o = s.orch
    result = await asyncio.to_thread(interp.interpret, body.text, _zones(o))
    out = result.to_dict()
    out["mission"] = None
    if not result.valid:
        from fastapi.responses import JSONResponse
        return JSONResponse(out, status_code=422)
    # the structured request is re-validated through the same schema as POST /missions
    validated = MissionCreate(**result.structured)
    out["structured"] = validated.model_dump(mode="json")
    if body.create:
        m = o.missions.create(MissionRequest(
            name=validated.name, description=validated.description, objective=validated.objective,
            priority=validated.priority, zone_id=validated.zone_id, deadline_s=validated.deadline_s,
            required_capabilities=frozenset(validated.required_capabilities) or frozenset({Capability.CAMERA}),
            num_drones=validated.num_drones, include_inspected=validated.include_inspected))
        d = m.to_dict()
        d["progress"] = round(o.fleet.mission_progress(m), 3)
        out["mission"] = d
    return out


@router.get("/ai/maintenance", summary="Predictive-maintenance view of the whole fleet (simulated data)")
async def maintenance(o: FleetOrchestrator = Depends(get_orch)):
    return {"disclaimer": "Prototype trained on synthetic data; no real-world accuracy claimed.",
            "drones": list(o.ctx.maintenance_view.values())}


@router.get("/ai/maintenance/{drone_id}")
async def maintenance_one(drone_id: str, o: FleetOrchestrator = Depends(get_orch)):
    v = o.ctx.maintenance_view.get(drone_id)
    if v is None:
        raise NotFound(f"maintenance assessment for {drone_id}")
    return v


@router.get("/ai/model-info", summary="Training/evaluation metrics of the ML models")
async def model_info(request: Request):
    models = getattr(request.app.state, "maintenance_models", None)
    return models.metrics if models else {"enabled": False}


@router.get("/ai/selection/{mission_id}", summary="Why these drones? Ranking with per-factor contributions")
async def selection(mission_id: str, o: FleetOrchestrator = Depends(get_orch)):
    m = o.missions.get(mission_id)
    return {"mission_id": mission_id, "rankings": m.plan_info.get("rankings", []),
            "decisions": [d for d in o.ctx.decisions.all() if d["mission_id"] == mission_id]}
