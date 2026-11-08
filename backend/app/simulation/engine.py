"""Drone simulation engine.

The engine owns the physics-ish dynamics: movement, battery, temperature, GPS and
signal. It does NOT make decisions (replanning, reassignment, takeover). It advances
state by one fixed timestep and reports what happened as SimEvents; the mission and
control layers react to those events. This keeps the simulator deterministic and the
decision logic independently testable.
"""
from __future__ import annotations

import random
from typing import Iterable

from app.core.config import SimConfig, WorldConfig
from app.core.enums import Capability, DroneStatus, EventType, FlightMode
from app.simulation.drone_state import DroneState
from app.simulation.events import SimEvent
from app.utils.geometry import (
    angle_diff, clamp, distance, heading_to, heading_vector, move_toward,
)

# Modes in which the drone follows a waypoint route on its own.
ROUTE_FOLLOWING_MODES = {FlightMode.AUTONOMOUS, FlightMode.REJOINING, FlightMode.RETURN_TO_HOME}


class SimulationEngine:
    def __init__(self, world: WorldConfig | None = None, sim: SimConfig | None = None,
                 seed: int | None = None) -> None:
        self.world = world or WorldConfig()
        self.cfg = sim or SimConfig()
        self.seed = self.cfg.seed if seed is None else seed
        self.rng = random.Random(self.seed)
        self.time: float = 0.0
        self.tick_count: int = 0
        self.drones: dict[str, DroneState] = {}
        # Mode a drone switches into after TAKEOFF completes.
        self._post_takeoff_mode: dict[str, FlightMode] = {}
        # Optional geofence: fn(x, y) -> True if the point is forbidden. Set by the orchestrator so
        # manual flight cannot enter no-fly zones / obstacles (backend keeps safety authority).
        self.collision_fn = None
        self._last_geofence_event: dict[str, float] = {}

    # ------------------------------------------------------------------ fleet
    def add_drone(self, drone: DroneState) -> None:
        if drone.drone_id in self.drones:
            raise ValueError(f"duplicate drone id {drone.drone_id}")
        self.drones[drone.drone_id] = drone

    def create_fleet(self, count: int) -> list[DroneState]:
        """Create a deterministic fleet around the home base with varied attributes."""
        hx, hy = self.world.home
        cap_pool = [
            frozenset({Capability.CAMERA}),
            frozenset({Capability.CAMERA, Capability.THERMAL}),
            frozenset({Capability.CAMERA, Capability.LIDAR}),
            frozenset({Capability.CAMERA, Capability.THERMAL, Capability.LIDAR}),
        ]
        out = []
        for i in range(count):
            did = f"D{i + 1:02d}"
            col, row = i % 4, i // 4
            x, y = hx + col * 14.0, hy + row * 14.0
            d = DroneState(
                drone_id=did, position_x=x, position_y=y,
                battery_percentage=round(self.rng.uniform(62.0, 100.0), 1),
                health_percentage=round(self.rng.uniform(80.0, 100.0), 1),
                home_x=x, home_y=y, heading=0.0,
                capabilities=cap_pool[i % len(cap_pool)],
                reliability=round(self.rng.uniform(0.82, 0.99), 3),
                flight_hours=round(self.rng.uniform(5, 400), 1),
                battery_cycles=self.rng.randint(5, 350),
                temperature=self.cfg.ambient_temp,
            )
            self.add_drone(d)
            out.append(d)
        return out

    def get(self, drone_id: str) -> DroneState:
        try:
            return self.drones[drone_id]
        except KeyError:
            raise KeyError(f"unknown drone {drone_id}") from None

    # ------------------------------------------------------------------ commands (called by control layer)
    def start_takeoff(self, drone_id: str, then: FlightMode = FlightMode.AUTONOMOUS) -> None:
        d = self.get(drone_id)
        d.flight_mode = FlightMode.TAKEOFF
        self._post_takeoff_mode[drone_id] = then

    def set_mode(self, drone_id: str, mode: FlightMode) -> None:
        self.get(drone_id).flight_mode = mode

    # ------------------------------------------------------------------ main step
    def step(self, n: int = 1) -> list[SimEvent]:
        events: list[SimEvent] = []
        for _ in range(n):
            events.extend(self._tick())
        return events

    def _tick(self) -> list[SimEvent]:
        dt = self.cfg.dt
        self.time += dt
        self.tick_count += 1
        events: list[SimEvent] = []
        for did in sorted(self.drones):          # sorted => deterministic order
            d = self.drones[did]
            if d.status in (DroneStatus.FAILED, DroneStatus.OFFLINE):
                self._fallout(d, dt, events)
                d.last_update = self.time
                continue
            moved = self._navigate(d, dt, events)
            if d.is_airborne:
                d.flight_hours += dt / 3600.0
            self._update_battery(d, dt, moved, events)
            self._update_environment(d, dt)
            d.last_update = self.time
        return events

    # ------------------------------------------------------------------ navigation
    def _navigate(self, d: DroneState, dt: float, events: list[SimEvent]) -> float:
        mode = d.flight_mode
        if mode == FlightMode.IDLE:
            d.velocity = 0.0
            return 0.0
        if mode == FlightMode.TAKEOFF:
            return self._do_takeoff(d, dt, events)
        if mode in ROUTE_FOLLOWING_MODES:
            return self._follow_route(d, dt, events)
        if mode == FlightMode.MANUAL:
            return self._apply_manual(d, dt, events)
        if mode == FlightMode.PAUSED:
            d.velocity = 0.0
            return 0.0
        if mode in (FlightMode.LANDING, FlightMode.EMERGENCY):
            return self._do_landing(d, dt, events, fast=(mode == FlightMode.EMERGENCY))
        return 0.0

    def _do_takeoff(self, d: DroneState, dt: float, events: list[SimEvent]) -> float:
        target = self.cfg.default_altitude
        d.velocity = 0.0
        d.altitude = min(target, d.altitude + self.cfg.vertical_speed * 2 * dt)
        if d.altitude >= target - 1e-9:
            nxt = self._post_takeoff_mode.pop(d.drone_id, FlightMode.AUTONOMOUS)
            d.flight_mode = nxt
            events.append(SimEvent(EventType.TAKEOFF_COMPLETED, self.time,
                                   f"{d.drone_id} reached cruise altitude", drone_id=d.drone_id,
                                   mission_id=d.current_mission_id))
        return 0.0

    def _follow_route(self, d: DroneState, dt: float, events: list[SimEvent]) -> float:
        """Move toward the active waypoint; never teleport. Handles dwell and progression."""
        # Dwell at an inspection point before moving on.
        if d.hover_remaining > 0:
            d.hover_remaining = max(0.0, d.hover_remaining - dt)
            d.velocity = 0.0
            if d.hover_remaining == 0.0:
                self._advance_waypoint(d, events)
            return 0.0

        wp = d.active_waypoint
        if wp is None:
            d.velocity = 0.0
            if d.route and d.route_complete:
                return 0.0
            return 0.0

        speed = min(self.cfg.cruise_speed, self.cfg.max_speed) * d.speed_derate
        # climb/descend toward waypoint altitude concurrently
        d.altitude = self._approach(d.altitude, wp.altitude, self.cfg.vertical_speed * dt)
        d.heading = heading_to(d.position, wp.xy) if distance(d.position, wp.xy) > 1e-9 else d.heading

        new_pos, travelled, _ = move_toward(d.position, wp.xy, speed, dt)
        d.position_x, d.position_y = new_pos
        d.velocity = travelled / dt if dt else 0.0
        d.total_distance += travelled

        if distance(d.position, wp.xy) <= self.cfg.waypoint_threshold:
            self._on_waypoint_arrival(d, wp, events)
        return travelled

    def _on_waypoint_arrival(self, d: DroneState, wp, events: list[SimEvent]) -> None:
        from app.core.enums import WaypointAction
        events.append(SimEvent(
            EventType.WAYPOINT_REACHED, self.time,
            f"{d.drone_id} reached {wp.id} ({d.current_waypoint_index + 1}/{len(d.route)})",
            drone_id=d.drone_id, mission_id=d.current_mission_id,
            data={"waypoint_id": wp.id, "index": d.current_waypoint_index,
                  "route_label": d.route_label, "action": wp.action.value,
                  "x": wp.x, "y": wp.y}))
        if wp.action == WaypointAction.INSPECT and self.cfg.inspect_dwell_s > 0:
            d.hover_remaining = self.cfg.inspect_dwell_s
            d.velocity = 0.0
        else:
            self._advance_waypoint(d, events)

    def _advance_waypoint(self, d: DroneState, events: list[SimEvent]) -> None:
        d.current_waypoint_index += 1
        if d.current_waypoint_index >= len(d.route):
            events.append(SimEvent(
                EventType.ROUTE_COMPLETED, self.time,
                f"{d.drone_id} completed {d.route_label} route",
                drone_id=d.drone_id, mission_id=d.current_mission_id,
                data={"route_label": d.route_label}))
            d.velocity = 0.0

    def _apply_manual(self, d: DroneState, dt: float, events: list[SimEvent]) -> float:
        """Operator control. Input is validated/clamped by the control layer, re-clamped here."""
        inp = d.manual_input
        if self.time > d.manual_input_expires:      # dead-man: stale input is ignored
            d.velocity = 0.0
            return 0.0
        fwd = clamp(inp.forward, -1, 1)
        strafe = clamp(inp.strafe, -1, 1)
        d.heading = (d.heading + clamp(inp.yaw, -1, 1) * self.cfg.yaw_rate_deg * dt) % 360.0
        fx, fy = heading_vector(d.heading)
        rx, ry = heading_vector(d.heading + 90.0)
        vx = (fx * fwd + rx * strafe)
        vy = (fy * fwd + ry * strafe)
        mag = (vx * vx + vy * vy) ** 0.5
        if mag > 1.0:
            vx, vy = vx / mag, vy / mag
        speed = self.cfg.manual_speed * d.speed_derate
        nx = clamp(d.position_x + vx * speed * dt, 0.0, self.world.width)
        ny = clamp(d.position_y + vy * speed * dt, 0.0, self.world.height)
        if self.collision_fn is not None and self.collision_fn(nx, ny):
            # Geofence: refuse to move into a forbidden cell, report at most once per 5 s.
            d.velocity = 0.0
            last = self._last_geofence_event.get(d.drone_id, -1e9)
            if self.time - last >= 5.0:
                self._last_geofence_event[d.drone_id] = self.time
                events.append(SimEvent(
                    EventType.COMMAND_REJECTED, self.time,
                    f"{d.drone_id} manual movement blocked by geofence (restricted area/obstacle)",
                    drone_id=d.drone_id, mission_id=d.current_mission_id, severity="WARNING",
                    data={"reason": "GEOFENCE", "x": round(nx, 1), "y": round(ny, 1)}))
            return 0.0
        travelled = distance(d.position, (nx, ny))
        d.position_x, d.position_y = nx, ny
        d.velocity = travelled / dt if dt else 0.0
        d.total_distance += travelled
        d.altitude = clamp(d.altitude + clamp(inp.climb, -1, 1) * self.cfg.vertical_speed * dt,
                           self.cfg.min_altitude, self.cfg.max_altitude)
        return travelled

    def _do_landing(self, d: DroneState, dt: float, events: list[SimEvent], fast: bool) -> float:
        rate = self.cfg.vertical_speed * (3.0 if fast else 1.5)
        d.velocity = 0.0
        d.altitude = max(0.0, d.altitude - rate * dt)
        if d.altitude <= 0.0:
            d.flight_mode = FlightMode.IDLE
            d.clear_route()
            events.append(SimEvent(EventType.LANDED, self.time, f"{d.drone_id} landed",
                                   drone_id=d.drone_id, mission_id=d.current_mission_id,
                                   data={"emergency": fast, "x": d.position_x, "y": d.position_y}))
        return 0.0

    def _fallout(self, d: DroneState, dt: float, events: list[SimEvent]) -> None:
        """A failed drone descends at a bounded rate (emergency auto-landing) and stops."""
        d.velocity = 0.0
        if d.altitude > 0.0:
            d.altitude = max(0.0, d.altitude - 8.0 * dt)
            if d.altitude == 0.0:
                d.flight_mode = FlightMode.IDLE
                d.clear_route()
                events.append(SimEvent(EventType.LANDED, self.time, f"{d.drone_id} down (failed)",
                                       drone_id=d.drone_id, mission_id=d.current_mission_id,
                                       severity="CRITICAL",
                                       data={"emergency": True, "failed": True,
                                             "x": d.position_x, "y": d.position_y}))

    @staticmethod
    def _approach(value: float, target: float, max_delta: float) -> float:
        if abs(target - value) <= max_delta:
            return target
        return value + max_delta if target > value else value - max_delta

    # ------------------------------------------------------------------ battery / environment
    def _update_battery(self, d: DroneState, dt: float, moved: float, events: list[SimEvent]) -> None:
        c = self.cfg
        drain = moved * c.battery_drain_per_metre
        if d.is_airborne:
            drain += c.battery_hover_drain_per_s * dt
        else:
            drain += c.battery_idle_drain_per_s * dt
        drain *= d.drain_multiplier
        before = d.battery_percentage
        d.battery_percentage = max(0.0, before - drain)
        if before > c.battery_warn >= d.battery_percentage:
            events.append(SimEvent(EventType.BATTERY_WARNING, self.time,
                                   f"{d.drone_id} battery low ({d.battery_percentage:.0f}%)",
                                   drone_id=d.drone_id, mission_id=d.current_mission_id,
                                   severity="WARNING", data={"battery": d.battery_percentage}))
        if before > c.battery_critical >= d.battery_percentage:
            events.append(SimEvent(EventType.BATTERY_WARNING, self.time,
                                   f"{d.drone_id} battery CRITICAL ({d.battery_percentage:.0f}%)",
                                   drone_id=d.drone_id, mission_id=d.current_mission_id,
                                   severity="CRITICAL", data={"battery": d.battery_percentage,
                                                              "critical": True}))
        if d.battery_percentage <= 0.0 and d.status != DroneStatus.FAILED:
            d.status = DroneStatus.FAILED
            d.flight_mode = FlightMode.EMERGENCY
            events.append(SimEvent(EventType.FAILURE_DETECTED, self.time,
                                   f"{d.drone_id} battery depleted", drone_id=d.drone_id,
                                   mission_id=d.current_mission_id, severity="CRITICAL",
                                   data={"reason": "BATTERY_DEPLETED"}))

    def _update_environment(self, d: DroneState, dt: float) -> None:
        """Temperature, GPS and signal relax toward targets with small seeded noise."""
        c = self.cfg
        load = (d.velocity / max(c.max_speed, 1e-9)) if d.is_airborne else 0.0
        target_temp = c.ambient_temp + 8.0 + 25.0 * load + d.temp_fault if d.is_airborne \
            else c.ambient_temp + d.temp_fault
        d.temperature += (target_temp - d.temperature) * min(1.0, 0.08 * dt * 2)
        d.temperature += self.rng.uniform(-0.05, 0.05)

        # Signal degrades with distance from home base.
        dist_home = distance(d.position, (d.home_x, d.home_y))
        base_signal = clamp(1.0 - dist_home / (self.world.width * 1.6), 0.2, 1.0)
        target_signal = clamp(base_signal - d.signal_fault, 0.0, 1.0)
        d.signal_strength += (target_signal - d.signal_strength) * min(1.0, 0.3 * dt * 2)
        d.signal_strength = clamp(d.signal_strength + self.rng.uniform(-0.003, 0.003), 0.0, 1.0)

        target_gps = clamp(0.98 - d.gps_fault, 0.0, 1.0)
        d.gps_quality += (target_gps - d.gps_quality) * min(1.0, 0.3 * dt * 2)
        d.gps_quality = clamp(d.gps_quality + self.rng.uniform(-0.003, 0.003), 0.0, 1.0)

        d.vibration = max(0.02, 0.1 + 0.25 * (1.0 - d.health_percentage / 100.0)
                          + 0.05 * (d.velocity / max(c.max_speed, 1e-9)))

    # ------------------------------------------------------------------ snapshots
    def snapshot(self) -> dict:
        return {
            "sim_time": round(self.time, 3),
            "tick": self.tick_count,
            "seed": self.seed,
            "drones": [self.drones[k].to_dict() for k in sorted(self.drones)],
        }

    def drones_iter(self) -> Iterable[DroneState]:
        return (self.drones[k] for k in sorted(self.drones))
