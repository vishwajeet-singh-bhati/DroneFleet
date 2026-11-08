# Testing

`make test` → **199 tests**, ~14 s. (`cd backend && ../.venv/bin/python -m pytest -q`)

| File | Tests | Covers |
|---|---|---|
| test_geometry.py | 12 | distance, bearing, projection, path length |
| test_simulation.py | 16 | movement, no-teleport, battery/thermal models, takeoff/landing, determinism |
| test_routing.py | 29 | grid rasterisation, A* (obstacles, corners, weather cost), coverage, replanning |
| test_mission_model.py | 8 | mission FSM legal/illegal transitions |
| test_integration_missions.py | 17 | plan→fly→inspect→land→report, multi-drone, replanning, reassignment |
| test_control_and_recovery.py | 42 | flight FSM, takeover/manual validation, deadman, geofence, rejoin strategies, RTH, recovery |
| test_api.py | 16 | REST contracts, error codes, WebSocket hello/state, persistence |
| test_ai.py | 47 | selector explainability (contributions sum to score), interpreter validation & fallback, ML pipeline |
| test_scenarios.py | 12 | all 8 scenarios (with per-tick invariants), determinism, exact `line_free`, scenario API |

## Scenario invariants (checked every tick)
no teleport; drones never inside an obstacle; a MANUAL drone does not advance its route; minimum separation.
Each scenario also asserts its own outcomes (e.g. takeover logged as `AUTONOMOUS→TAKEOVER_REQUEST→MANUAL`,
mission completed with full coverage, reassignment happened, altitude layers distinct).

## End-to-end checks done by hand
Real `uvicorn` + Vite proxy: `run_demo.py --url` played `takeover_rejoin` through the live server (PASSED); in a
headless browser a drone was taken over with the UI button, flown with `W`/`D` (moved ≈165 m via the backend) and
resumed to AUTONOMOUS.

## Not covered
Frontend has no unit tests (type-checked + build + manual browser run). LLM interpreter path not tested against the live API.
