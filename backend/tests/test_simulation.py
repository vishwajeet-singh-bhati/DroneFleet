import pytest

from app.core.config import SimConfig
from app.core.enums import DroneStatus, EventType, FlightMode, WaypointAction
from app.core.types import Waypoint
from app.simulation.drone_state import DroneState, ManualInput
from app.simulation.engine import SimulationEngine
from app.utils.geometry import distance


def make_engine(**sim_kwargs) -> SimulationEngine:
    return SimulationEngine(sim=SimConfig(**sim_kwargs))


def airborne_drone(engine, x=0.0, y=0.0, alt=30.0, mode=FlightMode.AUTONOMOUS) -> DroneState:
    d = DroneState(drone_id="T1", position_x=x, position_y=y, altitude=alt,
                   home_x=x, home_y=y, flight_mode=mode, status=DroneStatus.ON_MISSION)
    engine.add_drone(d)
    return d


def wp(i, x, y, alt=30.0, action=WaypointAction.TRANSIT):
    return Waypoint(id=f"WP{i}", x=x, y=y, altitude=alt, action=action)


def test_drone_moves_gradually_not_teleports():
    e = make_engine(inspect_dwell_s=0)
    d = airborne_drone(e)
    d.set_route([wp(1, 200, 0)], "MISSION")
    e.step()
    # one tick = dt * cruise_speed metres at most
    assert 0 < d.position_x <= e.cfg.cruise_speed * e.cfg.dt + 1e-9
    assert d.position_x < 200


def test_waypoint_progression_in_order():
    e = make_engine(inspect_dwell_s=0)
    d = airborne_drone(e)
    d.set_route([wp(1, 50, 0), wp(2, 50, 50), wp(3, 0, 50)], "MISSION")
    reached = []
    for _ in range(2000):
        for ev in e.step():
            if ev.type == EventType.WAYPOINT_REACHED:
                reached.append(ev.data["waypoint_id"])
        if d.route_complete:
            break
    assert reached == ["WP1", "WP2", "WP3"]
    assert d.route_complete


def test_route_completed_event_emitted_once():
    e = make_engine(inspect_dwell_s=0)
    d = airborne_drone(e)
    d.set_route([wp(1, 30, 0)], "MISSION")
    count = 0
    for _ in range(500):
        count += sum(1 for ev in e.step() if ev.type == EventType.ROUTE_COMPLETED)
    assert count == 1


def test_arrival_uses_threshold():
    e = make_engine(inspect_dwell_s=0, waypoint_threshold=6.0)
    d = airborne_drone(e)
    d.set_route([wp(1, 100, 0), wp(2, 200, 0)], "MISSION")
    for _ in range(1000):
        e.step()
        if d.current_waypoint_index == 1:
            break
    assert distance(d.position, (100, 0)) <= 6.0 + 1e-9


def test_inspect_waypoint_hovers_before_advancing():
    e = make_engine(inspect_dwell_s=3.0)
    d = airborne_drone(e)
    d.set_route([wp(1, 10, 0, action=WaypointAction.INSPECT), wp(2, 40, 0)], "MISSION")
    ticks_at_wp1 = 0
    for _ in range(400):
        e.step()
        if d.current_waypoint_index == 0 and d.hover_remaining > 0:
            ticks_at_wp1 += 1
        if d.current_waypoint_index == 1:
            break
    assert ticks_at_wp1 >= 3          # dwelled for several ticks
    assert d.current_waypoint_index >= 1


def test_heading_points_toward_target():
    e = make_engine()
    d = airborne_drone(e)
    d.set_route([wp(1, 0, 100)], "MISSION")
    e.step()
    assert d.heading == pytest.approx(0.0)
    d.position_x, d.position_y = 0.0, 0.0
    d.set_route([wp(1, 100, 0)], "MISSION")
    e.step()
    assert d.heading == pytest.approx(90.0)


def test_battery_drains_with_distance():
    e = make_engine(inspect_dwell_s=0)
    d = airborne_drone(e)
    d.battery_percentage = 100.0
    d.set_route([wp(1, 500, 0)], "MISSION")
    e.step(40)
    assert d.battery_percentage < 100.0
    assert d.total_distance > 0


def test_takeoff_then_autonomous():
    e = make_engine()
    d = DroneState(drone_id="T1", position_x=0, position_y=0, home_x=0, home_y=0)
    e.add_drone(d)
    e.start_takeoff("T1")
    for _ in range(200):
        e.step()
        if d.flight_mode == FlightMode.AUTONOMOUS:
            break
    assert d.flight_mode == FlightMode.AUTONOMOUS
    assert d.altitude == pytest.approx(e.cfg.default_altitude)


def test_landing_ends_idle_on_ground():
    e = make_engine()
    d = airborne_drone(e, alt=10.0, mode=FlightMode.LANDING)
    events = []
    for _ in range(100):
        events += e.step()
        if d.flight_mode == FlightMode.IDLE:
            break
    assert d.altitude == 0.0
    assert any(ev.type == EventType.LANDED for ev in events)


def test_manual_input_moves_drone_and_expires():
    e = make_engine()
    d = airborne_drone(e, mode=FlightMode.MANUAL)
    d.manual_input = ManualInput(forward=1.0)
    d.manual_input_expires = e.time + 1.0
    e.step(2)                                  # within expiry window
    moved = d.position_y
    assert moved > 0                           # heading 0 = north
    e.step(10)                                 # input expired => hover
    assert d.velocity == 0.0
    pos = d.position
    e.step(5)
    assert d.position == pos


def test_manual_climb_is_bounded():
    e = make_engine()
    d = airborne_drone(e, mode=FlightMode.MANUAL)
    d.manual_input = ManualInput(climb=1.0)
    d.manual_input_expires = 1e9
    e.step(1000)
    assert d.altitude == pytest.approx(e.cfg.max_altitude)


def test_manual_stays_in_world_bounds():
    e = make_engine()
    d = airborne_drone(e, x=1.0, y=1.0, mode=FlightMode.MANUAL)
    d.heading = 180.0
    d.manual_input = ManualInput(forward=1.0)
    d.manual_input_expires = 1e9
    e.step(50)
    assert d.position_y >= 0.0


def test_battery_depletion_fails_drone():
    e = make_engine()
    d = airborne_drone(e)
    d.battery_percentage = 0.001
    d.drain_multiplier = 1000
    events = e.step(5)
    assert d.status == DroneStatus.FAILED
    assert any(ev.type == EventType.FAILURE_DETECTED for ev in events)


def test_determinism_same_seed_same_state():
    def run(seed):
        e = SimulationEngine(seed=seed)
        e.create_fleet(6)
        for d in e.drones_iter():
            d.altitude = 30.0
            d.flight_mode = FlightMode.AUTONOMOUS
            d.set_route([wp(1, 300, 200), wp(2, 100, 400)], "MISSION")
        e.step(300)
        return e.snapshot()

    assert run(42) == run(42)
    assert run(42) != run(7)


def test_fleet_creation():
    e = SimulationEngine(seed=42)
    fleet = e.create_fleet(12)
    assert len(fleet) == 12
    assert [d.drone_id for d in fleet][:2] == ["D01", "D02"]
    assert all(0 <= d.battery_percentage <= 100 for d in fleet)


def test_duplicate_drone_rejected():
    e = make_engine()
    airborne_drone(e)
    with pytest.raises(ValueError):
        airborne_drone(e)
