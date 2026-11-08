# Architecture

## Principles
1. **Backend is the source of truth.** Position, battery, health, flight mode and mission state live in
   `simulation/` and `services/`. The frontend never mutates them; it posts commands (`take-control`,
   `manual-input`, `pause`, …) that the backend validates and may reject (`409/422` with `{error:{code,message}}`).
2. **AI recommends, deterministic code decides.** The selector, interpreter and ML models produce proposals or
   advisory scores. Every proposal passes `safety/validator.py` against the real drone state and real route.
3. **Deterministic.** Seeded `random.Random(42)`, sorted drone iteration, sim-time-derived timestamps → the same seed
   gives byte-identical event logs (tested).

## Modular monolith
| Package | Responsibility |
|---|---|
| `core` | enums, frozen config dataclasses, types, `EventLog` / `DecisionLog` (with subscribers) |
| `simulation` | `DroneState`, `SimulationEngine` (movement, battery/thermal/GPS models, takeoff/landing), fault effects |
| `mission` | `OccupancyGrid`, A*, lawnmower coverage, route planner, mission model + FSM, estimates |
| `safety` | command validation, rule-based `SafetyMonitor` |
| `control` | flight-mode FSM, safe-rejoin planner |
| `ai` | explainable selector, NL interpreter, synthetic data, maintenance/anomaly ML |
| `fleet` | availability, charging, altitude layers, summaries |
| `services` | `FleetOrchestrator` (tick pipeline), mission/control/recovery/failure services, runner, anomaly ext. |
| `database` | 9 SQLAlchemy tables + buffered repository |
| `api`, `websocket`, `schemas` | REST routers, `/ws/fleet`, pydantic request models |
| `scenarios` | 8 scripted scenarios with invariants + checks |

## Tick pipeline (`FleetOrchestrator.tick`, dt = 0.5 s)
`engine.step` → route SimEvents (arrival, inspected, landed…) → fleet housekeeping/charging → safety monitor →
extensions (ML anomaly, scenario player) → mission watchdog (completion/failure) → telemetry & flush.

## Realtime
`SimulationRunner` (asyncio) ticks at wall-clock × speed and broadcasts at 10 Hz on `/ws/fleet`:
`hello` (world, recent events/decisions), `state`, `events`, `decisions`, `world`, `pong`.

## Persistence
`drones, missions, waypoints, telemetry, events, alerts, assignments, control_sessions, ai_decisions` (SQLite).
Writes are buffered and flushed from the tick; the in-memory state remains authoritative.

## REST surface (see `/docs`)
`/health /fleet/summary /world /events /decisions /alerts /missions[/{id}[/start|pause|resume|abort|report]] /missions/interpret
/drones[/{id}[/take-control|manual-input|release-control|pause|resume|return-home|emergency|reset|telemetry]]
/simulation/{status,control,events,reset,zones/{id},scenarios[...]} /ai/{maintenance,model-info,selection/{id}}`
