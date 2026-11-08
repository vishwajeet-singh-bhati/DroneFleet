# AI DroneFleet Commander

A **100% software** autonomous drone-fleet management and human-in-the-loop control platform.
No drone hardware: a deterministic physics-lite simulator stands in for the fleet, while everything above
it — mission planning, A* routing with live replanning, explainable drone selection, safety supervision,
human takeover with a safe rejoin path, failure recovery, predictive-maintenance ML — is real, tested code.

> **Honesty note.** The ML models are a *simulation prototype trained on synthetic data*; they are advisory
> only and make no real-world accuracy claim. The optional LLM mission interpreter is never in the control
> loop. Computer vision was **not** implemented (it was optional and is not a core dependency).

![stack](https://img.shields.io/badge/FastAPI-SQLAlchemy-green) ![ui](https://img.shields.io/badge/React-TypeScript-blue)

## Quick start

```bash
./start.sh                 # backend :8000 + UI :5173   (or: make run)
# open http://localhost:5173        API docs: http://localhost:8000/docs
```
Requirements: Python 3.11+, Node 18+. `start.sh` creates `.venv` and installs everything on first run.

```bash
make test                                              # 199 backend tests
python scripts/run_demo.py --list                      # the 8 scenarios
python scripts/run_demo.py --scenario human_takeover   # narrated headless demo with pass/fail checks
python scripts/run_demo.py --scenario takeover_rejoin --url http://localhost:5173/api   # play it in the live UI
python scripts/run_demo.py --all                       # regression run of all scenarios
```
(Run uvicorn manually with `cd backend && uvicorn app.main:create_app --factory`.)

## What you can do in the UI
* Watch 12 drones on a custom canvas map (zones, obstacles, restricted airspace, routes, waypoints, trails).
* Create inspection missions (form, or plain English through the **AI mission interpreter**).
* See *why* drones were chosen (weighted-score breakdown) in the mission card and the **AI decisions** tab.
* **TAKE CONTROL** of a drone, fly it with `W A S D` / `Q E` / `R F`, then **RESUME MISSION** — the backend plans a
  safe rejoin path (no teleport, no skipped inspection points).
* Inject failures (low battery, GPS loss, signal loss, overheating, motor fault, obstacle, weather, drone failure)
  and watch detection → recovery → reassignment in the event log.
* Load any of the 8 scripted scenarios; the Scenario tab shows live pass/fail checks.

## Architecture in one picture
```
 React UI ──REST commands──▶  FastAPI  ──▶ services (mission / control / recovery / failure)
    ▲                                         │            │
    └──── /ws/fleet (state, events) ◀── FleetOrchestrator ──┴─▶ SimulationEngine (truth: battery, position, health)
                                              │   safety monitor · A* planner · selector · ML · SQLite
```
The **backend is the single source of truth**. The UI only sends validated commands and renders what the
server pushes. Details in [docs/architecture.md](docs/architecture.md).

## Docs
[architecture](docs/architecture.md) · [mission engine](docs/mission-engine.md) · [simulation](docs/simulation.md) ·
[AI](docs/ai.md) · [control system](docs/control-system.md) · [testing](docs/testing.md)

## Layout
```
backend/app/{core,utils,simulation,mission,safety,control,ai,fleet,services,database,api,schemas,websocket,scenarios}
backend/tests/         frontend/src/{components,hooks,services,types,utils}
scripts/run_demo.py    docs/    data/models/ (cached ML models)    start.sh  Makefile
```

## Known limitations
* 2D planning on a 20 m grid with fixed per-drone altitude layers (6 layers, reused beyond 6 simultaneous drones).
* Physics is simplified (kinematic movement, linear battery model, no wind dynamics).
* Single-process, in-memory simulation; SQLite persistence is write-behind and used for history, not recovery.
* Isolation Forest detection is modest (≈56 % at ≈2.7 % FPR on synthetic data); the supervised classifier is much
  stronger but only because the synthetic faults are separable. Neither is validated on real telemetry.
* Interpreter's LLM path (Anthropic API, `ANTHROPIC_API_KEY`) is optional and was not exercised against the live API in tests; the deterministic local parser is what is tested.
* No authentication/authorization on the API (portfolio scope).
