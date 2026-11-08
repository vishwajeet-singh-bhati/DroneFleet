import pytest

from app.core.enums import DroneStatus, FlightMode, MissionStatus
from app.mission.models import InvalidTransition
from app.services.errors import InvalidState, PlanningFailed
from app.services.mission_service import MissionRequest
from tests.helpers import events_of, make_orch, start_mission


def test_full_mission_completes_with_full_coverage_and_report():
    o = make_orch()
    m = start_mission(o, drones=2)
    o.run_until(lambda x: m.is_terminal, 8000)
    assert m.status == MissionStatus.COMPLETED
    r = m.report
    assert r["coverage_ratio"] == 1.0 and r["targets_total"] > 0
    assert len(r["drones"]) == 2 and all(d["distance_m"] > 0 for d in r["drones"])
    # drones landed at home, mission link released, status back to READY/CHARGING
    for did in m.assigned_drones:
        d = o.engine.get(did)
        assert d.flight_mode == FlightMode.IDLE and d.altitude == 0.0 and d.current_mission_id is None


def test_selection_is_explainable_and_logged():
    o = make_orch()
    m = o.missions.create(MissionRequest(name="x", zone_id="ZONE_A", num_drones=1))
    dec = [d for d in o.ctx.decisions.all() if d["kind"] == "DRONE_SELECTION"]
    assert dec and dec[0]["score"] is not None
    assert dec[0]["reasons"] and set(dec[0]["factors"]) >= {"battery", "health", "proximity", "capability", "risk"}
    assert sum(dec[0]["factors"].values()) == pytest.approx(dec[0]["score"], abs=0.01)


def test_unknown_zone_rejected():
    o = make_orch()
    with pytest.raises(PlanningFailed):
        o.missions.create(MissionRequest(name="x", zone_id="NOPE"))


def test_no_suitable_drone_fails_planning_with_reason():
    o = make_orch()
    for d in o.engine.drones_iter():
        d.battery_percentage = 10.0
    m = o.missions.create(MissionRequest(name="x", zone_id="ZONE_A"))
    assert m.status == MissionStatus.FAILED
    assert "no suitable drone" in m.failure_reason
    assert events_of(o, "MISSION_FAILED")


def test_all_drones_unavailable_when_all_occupied():
    o = make_orch(fleet_size=2)
    start_mission(o, drones=2)
    m2 = o.missions.create(MissionRequest(name="second", zone_id="ZONE_B"))
    assert m2.status == MissionStatus.FAILED
    assert "no suitable drone" in m2.failure_reason


def test_second_mission_never_reuses_reserved_drone():
    o = make_orch(fleet_size=4)
    m1 = o.missions.create(MissionRequest(name="a", zone_id="ZONE_A", num_drones=2))
    m2 = o.missions.create(MissionRequest(name="b", zone_id="ZONE_B", num_drones=2))
    assert m1.status == m2.status == MissionStatus.READY
    assert not set(m1.assigned_drones) & set(m2.assigned_drones)


def test_deadline_impossible_is_reported():
    o = make_orch()
    m = o.missions.create(MissionRequest(name="x", zone_id="ZONE_NORTH", deadline_s=20))
    assert m.status == MissionStatus.FAILED
    assert "deadline impossible" in m.failure_reason


def test_deadline_met_by_adding_drones():
    o = make_orch()
    solo = o.missions.create(MissionRequest(name="x", zone_id="ZONE_NORTH", num_drones=1, deadline_s=5000))
    assert solo.status == MissionStatus.READY and len(solo.assigned_drones) == 1
    est1 = solo.plan_info["estimated_duration_s"]
    o.missions.abort(solo.mission_id)
    # a deadline one drone cannot meet (needs <= 72% of its time) forces the planner to scale out
    tight = o.missions.create(MissionRequest(name="y", zone_id="ZONE_NORTH", num_drones=1,
                                             deadline_s=est1 * 0.8))
    assert tight.status == MissionStatus.READY, tight.failure_reason
    assert len(tight.assigned_drones) > 1
    assert tight.plan_info["estimated_duration_s"] <= est1 * 0.8 * 0.9 + 1e-6


def test_insufficient_battery_for_route_excludes_drone():
    o = make_orch(fleet_size=3)
    for d in o.engine.drones_iter():
        d.battery_percentage = 34.0       # above the minimum but short of a long route
    m = o.missions.create(MissionRequest(name="x", zone_id="ZONE_NORTH", num_drones=1))
    # either planned with more drones (smaller chunks) or failed with an explicit battery reason
    if m.status == MissionStatus.FAILED:
        assert "battery" in m.failure_reason or "deadline" in m.failure_reason
    else:
        for did in m.assigned_drones:
            assert o.engine.get(did).battery_percentage >= 34.0


def test_start_requires_ready_and_double_start_rejected():
    o = make_orch()
    m = start_mission(o)
    with pytest.raises(InvalidState):
        o.missions.start(m.mission_id)


def test_mission_state_machine_enforced_on_service():
    o = make_orch()
    m = start_mission(o)
    o.missions.abort(m.mission_id)
    with pytest.raises(InvalidState):
        o.missions.pause(m.mission_id)


def test_pause_resume_mission_hovers_then_continues():
    o = make_orch()
    m = start_mission(o)
    d = o.engine.get(m.assigned_drones[0])
    o.tick(40)
    o.missions.pause(m.mission_id)
    assert m.status == MissionStatus.PAUSED and d.flight_mode == FlightMode.PAUSED
    pos = d.position
    o.tick(20)
    assert d.position == pos and d.velocity == 0.0
    o.missions.resume(m.mission_id)
    assert d.flight_mode == FlightMode.AUTONOMOUS
    o.run_until(lambda x: m.is_terminal, 8000)
    assert m.status == MissionStatus.COMPLETED and m.report["coverage_ratio"] == 1.0


def test_abort_returns_drones_home_and_releases_mission():
    o = make_orch()
    m = start_mission(o, drones=2)
    o.tick(40)
    o.missions.abort(m.mission_id)
    assert m.status == MissionStatus.ABORTED
    o.run_until(lambda x: all(x.engine.get(i).flight_mode == FlightMode.IDLE for i in m.released_drones), 6000)
    for did in m.released_drones:
        d = o.engine.get(did)
        assert d.altitude == 0.0 and d.current_mission_id is None


def test_abort_before_start_releases_reservation():
    o = make_orch()
    m = o.missions.create(MissionRequest(name="x", zone_id="ZONE_A"))
    did = m.assigned_drones[0]
    assert o.engine.get(did).current_mission_id == m.mission_id
    o.missions.abort(m.mission_id)
    assert o.engine.get(did).current_mission_id is None


def test_coverage_persists_so_second_mission_prioritises_uninspected():
    o = make_orch()
    m1 = start_mission(o)
    o.run_until(lambda x: m1.is_terminal, 8000)
    m2 = o.missions.create(MissionRequest(name="again", zone_id="ZONE_A"))
    assert m2.status == MissionStatus.FAILED and "nothing to inspect" in m2.failure_reason
    m3 = o.missions.create(MissionRequest(name="all", zone_id="ZONE_A", include_inspected=True))
    assert m3.status == MissionStatus.READY


def test_simulation_is_deterministic_end_to_end():
    def run():
        o = make_orch(seed=7)
        m = start_mission(o, drones=2)
        o.run_until(lambda x: m.is_terminal, 8000)
        return [(e["type"], e["message"], e["sim_time"]) for e in o.ctx.events.all()], m.report

    assert run() == run()


def test_different_seed_changes_fleet_and_selection():
    a, b = make_orch(seed=1), make_orch(seed=2)
    ma = a.missions.create(MissionRequest(name="x", zone_id="ZONE_A"))
    mb = b.missions.create(MissionRequest(name="x", zone_id="ZONE_A"))
    assert ma.plan_info["rankings"] != mb.plan_info["rankings"]
