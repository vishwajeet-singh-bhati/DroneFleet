"""All eight predefined scenarios must pass end-to-end (seed 42), deterministically."""
from __future__ import annotations

import pytest

from app.mission.grid import OccupancyGrid, Zone
from app.core.config import WorldConfig
from app.core.enums import ZoneKind
from app.scenarios import list_scenarios, run_scenario
from app.scenarios.engine import ORDER


@pytest.mark.parametrize("name", ORDER)
def test_scenario_passes(name):
    r = run_scenario(name, 42)
    failed = [c for c in r["checks"] if not c["ok"]]
    assert r["passed"], f"{name}: {failed}"
    assert r["checks"], "a scenario must assert something"


def test_eight_scenarios_registered():
    assert len(list_scenarios()) == 8


def test_scenario_is_deterministic():
    a = run_scenario("takeover_rejoin", 42)
    b = run_scenario("takeover_rejoin", 42)
    assert a["ticks"] == b["ticks"]
    assert [(e["clock"], e["type"], e["message"]) for e in a["timeline"]] == \
           [(e["clock"], e["type"], e["message"]) for e in b["timeline"]]
    assert a["mission_reports"] == b["mission_reports"]


def test_line_free_is_exact_at_corners_and_endpoints():
    g = OccupancyGrid(WorldConfig(width=600, height=600, cell_size=20), margin=0.0)
    g.add_zone(Zone("OB", ZoneKind.OBSTACLE, 230, 60, 270, 140))      # cells x 11..13, y 3..6
    assert g.line_free((290, 130), (300, 100))          # ends exactly on a cell corner, nothing blocked
    assert not g.line_free((200, 100), (320, 100))      # straight through the obstacle
    assert not g.line_free((220, 50), (280, 150))       # diagonal through it
    assert g.line_free((200, 100), (220, 100))          # touching the obstacle's neighbour only
    assert not g.line_free((-5, 10), (50, 10))          # outside the world


def test_scenario_api(client):
    r = client.get("/simulation/scenarios")
    assert r.status_code == 200 and len(r.json()) == 8
    r = client.post("/simulation/scenarios/nope/run")
    assert r.status_code == 404
    r = client.post("/simulation/scenarios/normal_mission/load")
    assert r.status_code == 200 and r.json()["loaded"] == "normal_mission"
    assert client.get("/fleet/summary").status_code == 200
    st = client.get("/simulation/scenarios/active/status").json()
    assert st["name"] == "normal_mission" and st["finished"] is False
