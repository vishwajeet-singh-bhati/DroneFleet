from __future__ import annotations

from fastapi import APIRouter, Depends, Request

from app.api.deps import AppState, get_orch, get_state
from app.schemas.api import InjectEventRequest, ResetRequest, ScenarioRunRequest, SimControlRequest
from app.services.orchestrator import FleetOrchestrator

router = APIRouter(prefix="/simulation", tags=["simulation"])


@router.get("/status")
async def status(s: AppState = Depends(get_state)):
    return s.runner.status()


@router.post("/control", summary="start / stop / speed / step the simulation clock")
async def control(body: SimControlRequest, s: AppState = Depends(get_state)):
    if body.action == "start":
        s.runner.running = True
    elif body.action == "stop":
        s.runner.running = False
    elif body.action == "speed":
        if body.speed is None:
            from app.safety.validator import CommandRejected
            raise CommandRejected("INVALID_PARAMS", "speed is required")
        s.runner.speed = body.speed
    elif body.action == "step":
        s.orch.tick(body.steps or 1)
    return s.runner.status()


@router.post("/events", summary="Inject a failure / world event", status_code=201)
async def inject(body: InjectEventRequest, o: FleetOrchestrator = Depends(get_orch)):
    return o.failures.inject(body.type, body.drone_id, body.params)


@router.delete("/zones/{zone_id}", summary="Clear a dynamically injected obstacle/weather zone")
async def clear_zone(zone_id: str, o: FleetOrchestrator = Depends(get_orch)):
    res = o.failures.remove_zone(zone_id)
    o.missions.on_grid_changed("zone cleared")
    return res


@router.post("/drones/{drone_id}/clear-faults")
async def clear_faults(drone_id: str, o: FleetOrchestrator = Depends(get_orch)):
    return o.failures.clear_drone_faults(drone_id)


@router.post("/reset", summary="Reset the whole simulation with a seed")
async def reset(body: ResetRequest | None = None, s: AppState = Depends(get_state)):
    body = body or ResetRequest()
    s.runner.running = False
    s.reset(body.seed, body.fleet_size)
    return s.runner.status()


# ------------------------------------------------------------------ scripted scenarios
@router.get("/scenarios", summary="List the predefined scenarios")
async def scenarios():
    from app.scenarios import list_scenarios
    return list_scenarios()


@router.post("/scenarios/{name}/run", summary="Run a scenario headless and return its pass/fail report")
async def run_scenario_headless(name: str, body: ScenarioRunRequest | None = None):
    from app.safety.validator import CommandRejected
    from app.scenarios import list_scenarios, run_scenario
    if name not in {s["id"] for s in list_scenarios()}:
        raise CommandRejected("UNKNOWN_SCENARIO", f"unknown scenario '{name}'")
    body = body or ScenarioRunRequest()
    res = run_scenario(name, body.seed, body.max_ticks)
    res.pop("orchestrator", None)
    return res


@router.post("/scenarios/{name}/load", summary="Reset the live simulation and play a scenario in it")
async def load_scenario(name: str, body: ResetRequest | None = None, s: AppState = Depends(get_state)):
    from app.safety.validator import CommandRejected
    from app.scenarios import list_scenarios, load_into
    from app.scenarios.engine import get_scenario
    if name not in {x["id"] for x in list_scenarios()}:
        raise CommandRejected("UNKNOWN_SCENARIO", f"unknown scenario '{name}'")
    body = body or ResetRequest()
    s.runner.running = False
    o = s.reset(body.seed, get_scenario(name).fleet_size)
    load_into(o, name)
    s.runner.running = True
    return {"loaded": name, **o.scenario.status()}


@router.get("/scenarios/active/status", summary="Progress of the scenario loaded in the live simulation")
async def active_scenario(o: FleetOrchestrator = Depends(get_orch)):
    return o.scenario.status() if o.scenario else {"name": None}
