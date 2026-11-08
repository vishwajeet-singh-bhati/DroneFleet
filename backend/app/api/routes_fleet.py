from __future__ import annotations

from fastapi import APIRouter, Depends, Query

from app.api.deps import get_orch
from app.services.orchestrator import FleetOrchestrator

router = APIRouter(tags=["fleet"])


@router.get("/health", summary="Liveness probe")
async def health():
    return {"status": "ok"}


@router.get("/fleet/summary", summary="Dashboard counters")
async def fleet_summary(o: FleetOrchestrator = Depends(get_orch)):
    return o.fleet.summary()


@router.get("/world", summary="Zones, obstacles, restricted areas and home base")
async def world(o: FleetOrchestrator = Depends(get_orch)):
    return o.world()


@router.get("/events", summary="Event log (newest last)")
async def events(limit: int = Query(100, ge=1, le=2000), drone_id: str | None = None,
                 mission_id: str | None = None, type: str | None = None, persisted: bool = False,
                 o: FleetOrchestrator = Depends(get_orch)):
    if persisted and o.ctx.repo:
        rows = o.ctx.repo.events(limit, drone_id, mission_id, type)
        return [{"id": r.seq, "type": r.type, "message": r.message, "severity": r.severity,
                 "drone_id": r.drone_id, "mission_id": r.mission_id, "sim_time": r.sim_time,
                 "data": r.data} for r in rows]
    return o.ctx.events.recent(limit, drone_id=drone_id, mission_id=mission_id, type=type)


@router.get("/decisions", summary="Explainable AI decision log")
async def decisions(limit: int = Query(100, ge=1, le=1000), mission_id: str | None = None,
                    o: FleetOrchestrator = Depends(get_orch)):
    return o.ctx.decisions.recent(limit, mission_id=mission_id)


@router.get("/alerts", summary="Warning/critical events")
async def alerts(limit: int = Query(100, ge=1, le=1000), o: FleetOrchestrator = Depends(get_orch)):
    return [e for e in o.ctx.events.recent(2000) if e["severity"] != "INFO"][-limit:]
