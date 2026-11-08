import pytest

from app.control.rejoin import plan_rejoin
from app.core.config import WorldConfig
from app.core.enums import DroneStatus, EventType, FlightMode, MissionStatus, WaypointAction, ZoneKind
from app.core.types import Waypoint
from app.mission.grid import OccupancyGrid, Zone
from app.mission.route_planner import RoutePlanner
from app.safety.validator import CommandRejected
from app.services.errors import InvalidState
from app.simulation.drone_state import DroneState
from app.utils.geometry import distance
from tests.helpers import (
    events_of, fly_manual, make_orch, max_step, run_until_inspected, start_mission,
)


# =========================================================================== takeover / manual
def test_takeover_stops_autonomy_and_logs_transition():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    run_until_inspected(o, did, 3)
    idx = d.current_waypoint_index
    r = o.control.takeover(did)
    assert r["changed"] and d.flight_mode == FlightMode.MANUAL
    msgs = [e["message"] for e in events_of(o, "MODE_CHANGED", did)]
    assert any("AUTONOMOUS → TAKEOVER_REQUEST" in x for x in msgs)
    assert any("TAKEOVER_REQUEST → MANUAL" in x for x in msgs)
    assert events_of(o, "CONTROL_TAKEOVER", did)
    pos = d.position
    o.tick(20)                                   # no input => hover, NOT following the route
    assert d.position == pos and d.current_waypoint_index == idx
    assert o.control.saved[did].index >= idx


def test_manual_commands_move_drone_and_are_validated():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    run_until_inspected(o, did, 2)
    o.control.takeover(did)
    y0, h0, alt0 = d.position_y, d.heading, d.altitude
    d.heading = 0.0
    fly_manual(o, did, 10, forward=1.0)
    assert d.position_y > y0 + 40                # north
    fly_manual(o, did, 6, yaw=1.0)
    assert d.heading > 10
    fly_manual(o, did, 6, climb=1.0)
    assert d.altitude > alt0
    fly_manual(o, did, 6, climb=-1.0)
    with pytest.raises(CommandRejected):
        o.control.manual_input(did, {"forward": "fast"})
    with pytest.raises(CommandRejected):
        o.control.manual_input(did, {"forward": float("nan")})
    with pytest.raises(CommandRejected):
        o.control.manual_input(did, {"warp": 1})
    o.control.manual_input(did, {"forward": 99})  # clamped, not rejected
    assert d.manual_input.forward == 1.0
    assert o.control.sessions[did].rejected == 3


def test_manual_input_rejected_when_not_in_manual():
    o = make_orch()
    m = start_mission(o)
    with pytest.raises(CommandRejected) as e:
        o.control.manual_input(m.assigned_drones[0], {"forward": 1})
    assert e.value.code == "NOT_IN_MANUAL"


def test_duplicate_takeover_and_release_are_idempotent():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    run_until_inspected(o, did, 2)
    assert o.control.takeover(did)["changed"]
    assert not o.control.takeover(did)["changed"]
    assert len(events_of(o, "CONTROL_TAKEOVER", did)) == 1
    assert o.control.release_control(did)["changed"]
    assert not o.control.release_control(did)["changed"]


def test_takeover_rejected_in_emergency_and_when_failed():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    o.tick(30)
    o.control.emergency(did)
    with pytest.raises(CommandRejected) as e:
        o.control.takeover(did)
    assert e.value.code == "EMERGENCY_ACTIVE"
    with pytest.raises(CommandRejected):
        o.control.takeover("D12" if did != "D12" else "D11")      # idle drone on the ground
    assert events_of(o, "COMMAND_REJECTED")


def test_manual_flight_cannot_enter_restricted_area():
    o = make_orch()
    m = start_mission(o, zone="ZONE_B")
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    run_until_inspected(o, did, 2)
    o.control.takeover(did)
    nofly = o.ctx.grid.zones["NOFLY_1"]
    d.position_x, d.position_y = nofly.x0 - 60, (nofly.y0 + nofly.y1) / 2     # west of the no-fly box
    d.heading = 90.0                                                          # facing east
    fly_manual(o, did, 60, forward=1.0)
    assert not o.ctx.grid.is_blocked_xy(d.position)
    assert any(e["data"].get("reason") == "GEOFENCE" for e in events_of(o, "COMMAND_REJECTED", did))


def test_stale_manual_input_expires_deadman():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    run_until_inspected(o, did, 2)
    o.control.takeover(did)
    o.control.manual_input(did, {"forward": 1.0})
    o.tick(10)                                   # one command, then silence for 5 s
    pos = d.position
    o.tick(10)
    assert d.position == pos


def test_drone_fails_during_manual_control_ends_session_and_reassigns():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    run_until_inspected(o, did, 3)
    o.control.takeover(did)
    o.failures.inject("DRONE_FAILURE", did)
    assert did not in o.control.sessions
    assert o.engine.get(did).status == DroneStatus.FAILED
    assert m.assigned_drones and did not in m.assigned_drones
    with pytest.raises(CommandRejected):
        o.control.manual_input(did, {"forward": 1})


# =========================================================================== resume + rejoin
def test_resume_rejoins_without_teleport_and_completes_mission():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    run_until_inspected(o, did, 3)
    o.control.takeover(did)
    d.heading = 0.0
    fly_manual(o, did, 30, forward=1.0)          # drift ~200 m north
    off = d.position
    r = o.control.release_control(did)
    assert r["outcome"] == "REJOINING" and d.flight_mode == FlightMode.REJOINING
    assert d.route_label == "REJOIN"
    assert d.position == off                     # state change did not move the drone
    assert max_step(o, did, 10) <= o.settings.sim.cruise_speed * o.settings.sim.dt + 1e-6
    o.run_until(lambda x: d.flight_mode == FlightMode.AUTONOMOUS, 3000)
    assert d.route_label == "MISSION"
    assert events_of(o, "ROUTE_REJOIN_STARTED", did) and events_of(o, "ROUTE_REJOIN_COMPLETED", did)
    o.run_until(lambda x: m.is_terminal, 8000)
    assert m.status == MissionStatus.COMPLETED and m.report["coverage_ratio"] == 1.0
    assert m.report["takeovers"] == 1 and m.report["rejoins"] == 1


def test_resume_does_not_skip_inspection_waypoints():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    run_until_inspected(o, did, 2)
    saved_idx_before = d.current_waypoint_index
    o.control.takeover(did)
    fly_manual(o, did, 20, strafe=1.0)
    o.control.release_control(did)
    entry = o.control.rejoins[did].plan.entry_index
    route = o.control.rejoins[did].plan.mission_route
    skipped = route[saved_idx_before:entry]
    assert all(w.action == WaypointAction.TRANSIT for w in skipped)


def test_resume_refused_when_battery_cannot_cover_rejoin_and_mission():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    run_until_inspected(o, did, 2)
    o.control.takeover(did)
    d.battery_percentage = 20.0
    r = o.control.release_control(did)
    assert r["outcome"] == "RETURNING_HOME" and d.flight_mode == FlightMode.RETURN_TO_HOME
    assert events_of(o, "COMMAND_REJECTED", did)


def test_release_without_mission_holds_position():
    o = make_orch()
    d = o.engine.get("D01")
    d.altitude, d.flight_mode, d.status = 30.0, FlightMode.PAUSED, DroneStatus.READY
    o.control.takeover("D01")
    r = o.control.release_control("D01")
    assert r["outcome"] == "HOLDING" and d.flight_mode == FlightMode.PAUSED


def test_takeover_while_rejoining_keeps_mission_progress():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    run_until_inspected(o, did, 3)
    o.control.takeover(did)
    fly_manual(o, did, 25, forward=1.0)
    o.control.release_control(did)
    o.tick(5)
    o.control.takeover(did)                       # operator interrupts the rejoin
    assert did in o.control.saved
    fly_manual(o, did, 5, strafe=-1.0)
    o.control.release_control(did)
    o.run_until(lambda x: m.is_terminal, 9000)
    assert m.status == MissionStatus.COMPLETED and m.report["coverage_ratio"] == 1.0


def test_pause_resume_single_drone():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    o.tick(30)
    o.control.pause(did)
    assert d.flight_mode == FlightMode.PAUSED
    assert not o.control.pause(did)["changed"]            # duplicate
    o.control.resume(did)
    assert d.flight_mode == FlightMode.AUTONOMOUS
    with pytest.raises(CommandRejected):
        o.control.resume("D12")                           # not paused (idle on ground)


# ---- pure rejoin algorithm ------------------------------------------------------------------
def _grid():
    return OccupancyGrid(WorldConfig(width=600, height=600, cell_size=20), margin=0.0)


def _route():
    pts = [(100, 100, WaypointAction.INSPECT), (200, 100, WaypointAction.TRANSIT),
           (300, 100, WaypointAction.INSPECT), (400, 100, WaypointAction.TRANSIT),
           (500, 100, WaypointAction.LAND)]
    return [Waypoint(f"W{i}", x, y, 30.0, a) for i, (x, y, a) in enumerate(pts)]


def test_rejoin_goes_straight_to_next_waypoint_when_nothing_can_be_skipped():
    """Triangle inequality: detouring via the closest point of the same leg is never shorter."""
    g, route = _grid(), _route()
    plan = plan_rejoin((150, 300), route, 1, RoutePlanner(g), (50, 50))
    assert plan.strategy == "NEXT_WAYPOINT" and plan.entry_index == 1
    assert plan.rejoin_point == pytest.approx((200, 100))
    direct = next(c for c in plan.candidates if c["kind"] == "NEXT_WAYPOINT")
    proj = next(c for c in plan.candidates if c["kind"] == "SEGMENT_PROJECTION")
    assert direct["cost"] <= proj["cost"]


def test_rejoin_projects_onto_later_leg_and_skips_only_transit_points():
    g = _grid()
    pts = [(100, 100, WaypointAction.INSPECT), (200, 100, WaypointAction.TRANSIT),
           (300, 100, WaypointAction.TRANSIT), (400, 100, WaypointAction.INSPECT),
           (500, 100, WaypointAction.LAND)]
    route = [Waypoint(f"W{i}", x, y, 30.0, a) for i, (x, y, a) in enumerate(pts)]
    plan = plan_rejoin((250, 200), route, 1, RoutePlanner(g), (50, 50))
    # hand-computed costs: next waypoint 411.8, project onto W1->W2 350.0, onto W2->W3 311.8 (optimal)
    assert plan.strategy == "SEGMENT_PROJECTION"
    assert plan.rejoin_point == pytest.approx((300, 100))
    assert plan.entry_index == 3 and plan.skipped_ids == ["W1", "W2"]      # transit points only
    feasible = sorted(c["cost"] for c in plan.candidates if c["feasible"])
    assert feasible[0] == pytest.approx(311.8, abs=0.1)
    assert all(route[i].action == WaypointAction.TRANSIT for i in (1, 2))  # inspections untouched


def test_rejoin_never_skips_inspect_waypoint():
    g, route = _grid(), _route()
    # drone is closest to the later leg but W2 (INSPECT) lies ahead and must still be flown
    plan = plan_rejoin((340, 250), route, 1, RoutePlanner(g), (50, 50))
    assert plan.entry_index <= 2
    for w in route[1:plan.entry_index]:
        assert w.action == WaypointAction.TRANSIT


def test_rejoin_may_skip_transit_points_to_save_distance():
    g, route = _grid(), _route()
    plan = plan_rejoin((250, 200), route, 1, RoutePlanner(g), (50, 50))
    assert plan.entry_index in (1, 2)
    assert set(plan.skipped_ids) <= {"W1"}


def test_rejoin_replans_when_route_ahead_is_blocked():
    g, route = _grid(), _route()
    g.add_zone(Zone("OB", ZoneKind.OBSTACLE, 230, 60, 270, 140))      # blocks W1->W2
    plan = plan_rejoin((150, 300), route, 1, RoutePlanner(g), (50, 50))
    assert plan.strategy == "REPLAN"
    cur = (150, 300)
    for w in plan.path + plan.mission_route[plan.entry_index:]:
        assert g.line_free(cur, w.xy)
        cur = w.xy


def test_rejoin_none_when_nothing_left():
    g, route = _grid(), _route()
    assert plan_rejoin((10, 10), route, len(route), RoutePlanner(g), (50, 50)) is None


def test_rejoin_path_avoids_obstacles_between_drone_and_route():
    g, route = _grid(), _route()
    g.add_zone(Zone("WALL", ZoneKind.OBSTACLE, 100, 180, 300, 220))
    plan = plan_rejoin((200, 320), route, 1, RoutePlanner(g), (50, 50))
    cur = (200, 320)
    for w in plan.path:
        assert g.line_free(cur, w.xy)
        cur = w.xy


# =========================================================================== return home / emergency
def test_return_home_flies_back_and_lands_no_teleport():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    run_until_inspected(o, did, 4)
    far = distance(d.position, (d.home_x, d.home_y))
    assert far > 100
    o.control.return_home(did, "operator command")
    assert d.flight_mode == FlightMode.RETURN_TO_HOME and d.status == DroneStatus.RETURNING
    assert max_step(o, did, 5) <= o.settings.sim.cruise_speed * o.settings.sim.dt + 1e-6
    o.run_until(lambda x: d.flight_mode == FlightMode.IDLE and d.altitude == 0, 6000)
    assert distance(d.position, (d.home_x, d.home_y)) <= o.settings.sim.waypoint_threshold + 1e-6
    assert events_of(o, "RETURN_HOME_STARTED", did) and events_of(o, "RETURN_HOME_COMPLETED", did)


def test_return_home_rejected_for_grounded_drone_and_duplicate_ignored():
    o = make_orch()
    with pytest.raises(CommandRejected):
        o.control.return_home("D01")
    m = start_mission(o)
    did = m.assigned_drones[0]
    o.tick(30)
    o.control.return_home(did)
    assert not o.control.return_home(did)["changed"]


def test_emergency_lands_in_place_and_grounds_drone():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    o.tick(40)
    o.control.emergency(did)
    o.run_until(lambda x: d.flight_mode == FlightMode.IDLE, 2000)
    assert d.altitude == 0 and d.status == DroneStatus.OFFLINE
    assert d.current_mission_id is None


def test_invalid_mode_transition_is_rejected():
    o = make_orch()
    d = o.engine.get("D01")                       # IDLE on the ground
    with pytest.raises(InvalidState):
        o.control._transition(d, FlightMode.MANUAL, "illegal")


# =========================================================================== failures / recovery
def test_low_battery_triggers_return_home_and_replacement_completes_mission():
    o = make_orch()
    m = start_mission(o)
    first = m.assigned_drones[0]
    run_until_inspected(o, first, 3)
    o.failures.inject("LOW_BATTERY", first, {"battery": 12})
    o.tick(3)
    d1 = o.engine.get(first)
    assert d1.flight_mode == FlightMode.RETURN_TO_HOME
    assert first not in m.assigned_drones and len(m.assigned_drones) == 1
    repl = m.assigned_drones[0]
    assert repl != first and o.engine.get(repl).current_mission_id == m.mission_id
    assert events_of(o, "MISSION_REASSIGNED", repl)
    assert any(x["kind"] == "MISSION_REASSIGNMENT" for x in o.ctx.decisions.all())
    o.run_until(lambda x: m.is_terminal, 12000)
    assert m.status == MissionStatus.COMPLETED
    assert m.report["coverage_ratio"] == 1.0 and m.report["reassignments"] == 1
    assert d1.flight_mode == FlightMode.IDLE and d1.altitude == 0       # original drone got home


def test_replacement_is_validated_by_safety_layer_not_just_ai():
    o = make_orch(fleet_size=3)
    m = start_mission(o)
    first = m.assigned_drones[0]
    for d in o.engine.drones_iter():
        if d.drone_id != first:
            d.battery_percentage = 20.0          # nobody else passes the battery constraint
    run_until_inspected(o, first, 3)
    o.failures.inject("DRONE_FAILURE", first)
    assert m.status == MissionStatus.FAILED
    assert "no replacement" in m.failure_reason or "no drones left" in m.failure_reason


def test_drone_failure_reassigns_remaining_work():
    o = make_orch()
    m = start_mission(o, drones=2)
    a, b = m.assigned_drones
    run_until_inspected(o, a, 3)
    o.failures.inject("DRONE_FAILURE", a)
    assert a not in m.assigned_drones and len(m.assigned_drones) == 2
    o.run_until(lambda x: m.is_terminal, 14000)
    assert m.status == MissionStatus.COMPLETED and m.report["coverage_ratio"] == 1.0
    assert o.engine.get(a).status == DroneStatus.FAILED


def test_critical_battery_rule_uses_distance_to_home():
    """Same battery level: fine near home, a return-home trigger when far away."""
    near = make_orch()
    m1 = start_mission(near)
    d1 = near.engine.get(m1.assigned_drones[0])
    run_until_inspected(near, d1.drone_id, 3)
    d1.position_x, d1.position_y = d1.home_x + 80, d1.home_y + 80
    d1.battery_percentage = 25.0
    near.tick(2)
    assert d1.flight_mode == FlightMode.AUTONOMOUS

    far = make_orch()
    m2 = start_mission(far)
    d2 = far.engine.get(m2.assigned_drones[0])
    run_until_inspected(far, d2.drone_id, 3)
    d2.position_x, d2.position_y = 930.0, 930.0
    d2.battery_percentage = 25.0
    far.tick(2)
    assert d2.flight_mode == FlightMode.RETURN_TO_HOME
    assert any(x["kind"] == "SAFETY_ACTION" for x in far.ctx.decisions.all())


def test_signal_loss_triggers_failsafe_return_after_grace_period():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    o.tick(30)
    o.failures.inject("SIGNAL_LOSS", did)
    o.tick(6)
    assert d.flight_mode == FlightMode.AUTONOMOUS          # still within the grace window
    o.tick(40)
    assert d.flight_mode in (FlightMode.RETURN_TO_HOME, FlightMode.LANDING, FlightMode.IDLE)
    assert did not in m.assigned_drones


def test_comm_loss_during_manual_control_returns_home():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    run_until_inspected(o, did, 2)
    o.control.takeover(did)
    o.failures.inject("SIGNAL_LOSS", did)
    for _ in range(50):
        try:
            o.control.manual_input(did, {"forward": 0.1})
        except CommandRejected:
            break
        o.tick()
    assert d.flight_mode in (FlightMode.RETURN_TO_HOME, FlightMode.LANDING, FlightMode.IDLE)
    assert did not in o.control.sessions


def test_high_temperature_triggers_return_home():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    o.tick(30)
    o.failures.inject("HIGH_TEMPERATURE", did)
    o.run_until(lambda x: d.flight_mode == FlightMode.RETURN_TO_HOME, 800)
    assert d.flight_mode == FlightMode.RETURN_TO_HOME


def test_motor_health_drop_degrades_and_escalates():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    o.tick(30)
    o.failures.inject("MOTOR_HEALTH_DROP", did, {"drop": 80})
    o.tick(3)
    assert d.speed_derate < 1.0
    assert d.flight_mode in (FlightMode.EMERGENCY, FlightMode.RETURN_TO_HOME)


def test_gps_degradation_derates_speed_then_lands_if_lost():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    o.tick(30)
    o.failures.inject("GPS_DEGRADATION", did, {"severity": 0.6})
    o.tick(60)
    assert d.speed_derate <= 0.5
    o.failures.inject("GPS_DEGRADATION", did, {"severity": 0.95})
    o.run_until(lambda x: d.flight_mode in (FlightMode.EMERGENCY, FlightMode.IDLE), 600)
    assert d.flight_mode in (FlightMode.EMERGENCY, FlightMode.IDLE)


def test_battery_depletion_fails_drone_and_mission_reassigns():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    o.tick(30)
    d.battery_percentage = 0.01
    d.drain_multiplier = 500
    o.tick(3)
    assert d.status == DroneStatus.FAILED
    assert did not in m.assigned_drones


def test_invalid_injection_inputs_rejected():
    o = make_orch()
    with pytest.raises(CommandRejected):
        o.failures.inject("NOT_A_REAL_EVENT", "D01")
    with pytest.raises(CommandRejected):
        o.failures.inject("LOW_BATTERY")                         # missing drone
    with pytest.raises(CommandRejected):
        o.failures.inject("LOW_BATTERY", "D01", {"battery": 500})
    with pytest.raises(CommandRejected):
        o.failures.inject("OBSTACLE_APPEARED", None, {})
    with pytest.raises(CommandRejected):
        o.failures.inject("OBSTACLE_APPEARED", None, {"x": 5000, "y": 5})


# =========================================================================== dynamic replanning
def test_obstacle_on_route_triggers_replan_and_mission_still_completes():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    run_until_inspected(o, did, 2)
    old = [(w.x, w.y) for w in d.route[d.current_waypoint_index:]]
    r = o.failures.inject("OBSTACLE_APPEARED", did)
    assert did in r["replanned_drones"]
    ev = events_of(o, "ROUTE_REPLANNED", did)[-1]
    assert ev["data"]["old"] != ev["data"]["new"]
    zone = o.ctx.grid.zones[r["zone_id"]]
    cur = d.position
    for w in d.route[d.current_waypoint_index:]:
        assert o.ctx.grid.line_free(cur, w.xy)
        assert not o.ctx.grid.is_blocked_xy(w.xy) or w.action == WaypointAction.LAND
        cur = w.xy
    assert m.route_revision >= 1 and m.route_history
    o.run_until(lambda x: m.is_terminal, 9000)
    assert m.status == MissionStatus.COMPLETED
    assert not o.ctx.grid.is_blocked_xy(d.position)
    _ = (old, zone)


def test_obstacle_over_waypoint_drops_it_and_reports_uncovered():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    run_until_inspected(o, did, 2)
    victim = next(w for w in d.route[d.current_waypoint_index:] if w.action == WaypointAction.INSPECT)
    o.failures.inject("OBSTACLE_APPEARED", None, {"x": victim.x, "y": victim.y, "size": 40})
    assert victim.id not in [w.id for w in d.route[d.current_waypoint_index:]]
    o.run_until(lambda x: m.is_terminal, 9000)
    assert m.status == MissionStatus.COMPLETED
    assert victim.id in m.report["uncovered_targets"] and m.report["coverage_ratio"] < 1.0


def test_weather_zone_is_avoided_when_cheap_detour_exists():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    o.tick(10)
    o.failures.inject("WEATHER_RISK", None, {"x": 400, "y": 360, "size": 120})
    cur = d.position
    for w in d.route[d.current_waypoint_index:]:
        if w.action == WaypointAction.TRANSIT:
            assert o.ctx.grid.risk_at(o.ctx.grid.cell_of(w.xy)) == 0
        cur = w.xy
    o.run_until(lambda x: m.is_terminal, 9000)
    assert m.status == MissionStatus.COMPLETED


def test_remove_dynamic_zone_only():
    o = make_orch()
    r = o.failures.inject("OBSTACLE_APPEARED", None, {"x": 700, "y": 800, "size": 60})
    assert o.failures.remove_zone(r["zone_id"])["removed"] == r["zone_id"]
    with pytest.raises(CommandRejected):
        o.failures.remove_zone("OBS_1")


def test_manual_drone_route_revalidated_at_rejoin_after_obstacle():
    o = make_orch()
    m = start_mission(o)
    did = m.assigned_drones[0]
    d = o.engine.get(did)
    run_until_inspected(o, did, 2)
    o.control.takeover(did)
    fly_manual(o, did, 10, forward=1.0)
    # an obstacle lands on the saved route while the operator is flying
    nxt = o.control.saved[did].route[o.control.saved[did].index]
    o.failures.inject("OBSTACLE_APPEARED", None, {"x": nxt.x, "y": nxt.y, "size": 40})
    o.control.release_control(did)
    o.run_until(lambda x: m.is_terminal, 12000)
    assert m.status == MissionStatus.COMPLETED
    cur_blocked = o.ctx.grid.is_blocked_xy(d.position)
    assert not cur_blocked


def test_replacement_drone_logs_visible_states():
    o = make_orch()
    m = start_mission(o)
    first = m.assigned_drones[0]
    run_until_inspected(o, first, 3)
    o.failures.inject("LOW_BATTERY", first, {"battery": 10})
    o.tick(2)
    msgs = [e["message"] for e in o.ctx.events.all()]
    assert any(f"{first}: RETURNING HOME" in x for x in msgs)
    assert any("TAKING OVER" in x for x in msgs)
