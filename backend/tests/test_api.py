import pytest

from tests.conftest import step


def make_mission(client, **kw):
    body = {"name": "Inspect Zone A", "zone_id": "ZONE_A", "num_drones": 1, **kw}
    r = client.post("/missions", json=body)
    assert r.status_code == 201, r.text
    return r.json()


def test_openapi_documents_all_main_endpoints(client):
    paths = client.get("/openapi.json").json()["paths"]
    for p in ["/missions", "/missions/{mission_id}", "/missions/{mission_id}/start",
              "/missions/{mission_id}/pause", "/missions/{mission_id}/resume", "/drones", "/drones/{drone_id}",
              "/drones/{drone_id}/take-control", "/drones/{drone_id}/release-control",
              "/drones/{drone_id}/return-home", "/simulation/events", "/events", "/health"]:
        assert p in paths, p


def test_health_and_fleet(client):
    assert client.get("/health").json() == {"status": "ok"}
    drones = client.get("/drones").json()
    assert len(drones) == 12 and drones[0]["drone_id"] == "D01"
    s = client.get("/fleet/summary").json()
    assert s["fleet_count"] == 12 and s["active_missions"] == 0
    w = client.get("/world").json()
    assert {z["kind"] for z in w["zones"]} >= {"INSPECTION", "OBSTACLE", "RESTRICTED"}


def test_drone_not_found_is_404_with_code(client):
    r = client.get("/drones/D99")
    assert r.status_code == 404 and r.json()["error"]["code"] == "NOT_FOUND"


def test_mission_lifecycle_over_http(client):
    m = make_mission(client)
    assert m["status"] == "READY" and m["assigned_drones"]
    mid = m["mission_id"]
    assert client.get(f"/missions/{mid}").json()["routes"]
    assert client.post(f"/missions/{mid}/start").json()["status"] == "ACTIVE"
    assert client.post(f"/missions/{mid}/start").status_code == 409        # duplicate start
    step(client, 40)
    assert client.post(f"/missions/{mid}/pause").json()["status"] == "PAUSED"
    assert client.post(f"/missions/{mid}/resume").json()["status"] == "ACTIVE"
    step(client, 600)
    final = client.get(f"/missions/{mid}").json()
    assert final["status"] == "COMPLETED" and final["progress"] == 1.0
    rep = client.get(f"/missions/{mid}/report").json()
    assert rep["coverage_ratio"] == 1.0
    assert client.get("/events", params={"mission_id": mid}).json()
    assert client.get("/decisions", params={"mission_id": mid}).json()


def test_mission_validation_errors(client):
    assert client.post("/missions", json={"name": "x", "zone_id": "ZONE_A", "num_drones": 0}).status_code == 422
    assert client.post("/missions", json={"name": "", "zone_id": "ZONE_A"}).status_code == 422
    r = client.post("/missions", json={"name": "x", "zone_id": "NOPE"})
    assert r.status_code == 422 and r.json()["error"]["code"] == "PLANNING_FAILED"
    assert client.get("/missions/MISSION-999").status_code == 404


def test_planning_failure_is_returned_as_failed_mission(client):
    for _ in range(12):
        pass
    for d in client.get("/drones").json():
        client.post("/simulation/events", json={"type": "LOW_BATTERY", "drone_id": d["drone_id"],
                                                "params": {"battery": 5}})
    m = make_mission(client)
    assert m["status"] == "FAILED" and "no suitable drone" in m["failure_reason"]


def test_takeover_manual_and_release_over_http(client):
    m = make_mission(client)
    did = m["assigned_drones"][0]
    client.post(f"/missions/{m['mission_id']}/start")
    step(client, 80)
    r = client.post(f"/drones/{did}/take-control")
    assert r.status_code == 200 and r.json()["flight_mode"] == "MANUAL"
    assert client.post(f"/drones/{did}/take-control").json()["changed"] is False       # duplicate
    before = client.get(f"/drones/{did}").json()
    for _ in range(8):
        assert client.post(f"/drones/{did}/manual-input", json={"forward": 1}).status_code == 200
        step(client, 1)
    after = client.get(f"/drones/{did}").json()
    assert (after["position_x"], after["position_y"]) != (before["position_x"], before["position_y"])
    assert client.post(f"/drones/{did}/manual-input", json={"forward": "fast"}).status_code == 422
    assert client.post(f"/drones/{did}/manual-input", json={"warp": 1}).status_code == 422
    rel = client.post(f"/drones/{did}/release-control").json()
    assert rel["outcome"] == "REJOINING"
    step(client, 2500)
    assert client.get(f"/missions/{m['mission_id']}").json()["status"] == "COMPLETED"


def test_manual_input_without_takeover_is_409(client):
    r = client.post("/drones/D01/manual-input", json={"forward": 1})
    assert r.status_code == 409 and r.json()["error"]["code"] == "NOT_IN_MANUAL"


def test_return_home_on_ground_rejected(client):
    r = client.post("/drones/D01/return-home")
    assert r.status_code == 409


def test_return_home_releases_drone_and_mission_reassigns(client):
    m = make_mission(client)
    did = m["assigned_drones"][0]
    client.post(f"/missions/{m['mission_id']}/start")
    step(client, 80)
    assert client.post(f"/drones/{did}/return-home").json()["flight_mode"] == "RETURN_TO_HOME"
    cur = client.get(f"/missions/{m['mission_id']}").json()
    assert did not in cur["assigned_drones"] and cur["assigned_drones"]
    step(client, 1500)
    assert client.get(f"/missions/{m['mission_id']}").json()["status"] == "COMPLETED"


def test_injection_endpoint_and_errors(client):
    m = make_mission(client)
    did = m["assigned_drones"][0]
    client.post(f"/missions/{m['mission_id']}/start")
    step(client, 60)
    r = client.post("/simulation/events", json={"type": "LOW_BATTERY", "drone_id": did, "params": {"battery": 10}})
    assert r.status_code == 201
    step(client, 3)
    assert client.get(f"/drones/{did}").json()["flight_mode"] == "RETURN_TO_HOME"
    assert client.post("/simulation/events", json={"type": "BOGUS", "drone_id": did}).status_code == 422
    assert client.post("/simulation/events", json={"type": "LOW_BATTERY"}).status_code == 422
    assert client.post("/simulation/events", json={"type": "LOW_BATTERY", "drone_id": "D77"}).status_code == 404


def test_obstacle_injection_changes_world_and_route(client):
    m = make_mission(client)
    did = m["assigned_drones"][0]
    client.post(f"/missions/{m['mission_id']}/start")
    step(client, 60)
    v0 = client.get("/world").json()["version"]
    r = client.post("/simulation/events", json={"type": "OBSTACLE_APPEARED", "drone_id": did}).json()
    w = client.get("/world").json()
    assert w["version"] > v0 and any(z["zone_id"] == r["zone_id"] for z in w["zones"])
    assert client.get("/events", params={"type": "ROUTE_REPLANNED"}).json()
    assert client.delete(f"/simulation/zones/{r['zone_id']}").status_code == 200
    assert client.delete("/simulation/zones/OBS_1").status_code == 422     # static zones are protected


def test_simulation_control_and_reset_determinism(client):
    assert client.get("/simulation/status").json()["running"] is False
    a = step(client, 10)
    assert a["tick"] == 10
    client.post("/simulation/control", json={"action": "speed", "speed": 4})
    assert client.get("/simulation/status").json()["speed"] == 4
    assert client.post("/simulation/control", json={"action": "speed"}).status_code == 422
    assert client.post("/simulation/control", json={"action": "speed", "speed": 999}).status_code == 422
    client.post("/simulation/reset", json={"seed": 42})
    d1 = client.get("/drones").json()
    client.post("/simulation/reset", json={"seed": 42})
    assert client.get("/drones").json() == d1
    client.post("/simulation/reset", json={"seed": 5})
    assert client.get("/drones").json() != d1


def test_state_is_never_writable_from_frontend(client):
    # there is deliberately no endpoint that writes battery/position/health/mission state
    paths = client.get("/openapi.json").json()["paths"]
    for p, methods in paths.items():
        for verb in methods:
            if verb in ("put", "patch"):
                pytest.fail(f"unexpected mutating endpoint {verb.upper()} {p}")


def test_persistence_of_events_missions_decisions_and_assignments(client):
    m = make_mission(client)
    client.post(f"/missions/{m['mission_id']}/start")
    step(client, 700)
    repo = client.app.state.dc.repo
    assert any(r.mission_id == m["mission_id"] and r.status == "COMPLETED" for r in repo.missions())
    assert repo.events(500, mission_id=m["mission_id"])
    assert repo.decisions(50, mission_id=m["mission_id"])
    assert repo.assignments(m["mission_id"])
    assert repo.waypoints(m["mission_id"])
    did = m["assigned_drones"][0]
    assert client.get(f"/drones/{did}/telemetry").json()
    assert repo.drones()
    persisted = client.get("/events", params={"persisted": True, "limit": 20}).json()
    assert persisted


def test_websocket_streams_state_events_and_world(client):
    m = make_mission(client)
    client.post(f"/missions/{m['mission_id']}/start")
    runner = client.app.state.dc.runner
    with client.websocket_connect("/ws/fleet") as ws:
        hello = ws.receive_json()
        assert hello["type"] == "hello" and hello["data"]["world"]["zones"]
        state = ws.receive_json()
        assert state["type"] == "state" and len(state["data"]["drones"]) == 12
        did = m["assigned_drones"][0]
        step(client, 30)
        import asyncio
        asyncio.get_event_loop_policy()
        client.portal.call(runner.broadcast)
        seen = set()
        for _ in range(6):
            msg = ws.receive_json()
            seen.add(msg["type"])
            if msg["type"] == "state":
                d = next(x for x in msg["data"]["drones"] if x["drone_id"] == did)
                assert d["flight_mode"] in ("TAKEOFF", "AUTONOMOUS") and d["current_mission_id"] == m["mission_id"]
                assert d["current_waypoint_index"] >= 0 and "timestamp" in d
            if {"state", "events"} <= seen:
                break
        assert {"state", "events"} <= seen
        ws.send_text("ping")
        got = []
        for _ in range(4):
            got.append(ws.receive_json()["type"])
            if "pong" in got:
                break
        assert "pong" in got
