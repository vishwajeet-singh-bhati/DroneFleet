"""Failure/event injection API surface (operator-triggered simulated faults)."""
from __future__ import annotations

from app.core.enums import DroneStatus, EventType, FlightMode, ZoneKind
from app.mission.grid import Zone
from app.safety.validator import CommandRejected
from app.services.context import Context
from app.services.errors import NotFound, ServiceError
from app.services.mission_service import MissionService
from app.services.recovery_service import RecoveryService
from app.simulation.failures import (
    DRONE_EVENTS, WORLD_EVENTS, InjectedEventType, apply_drone_fault, clear_faults,
)
from app.utils.geometry import distance


class FailureService:
    def __init__(self, ctx: Context, missions: MissionService, recovery: RecoveryService) -> None:
        self.ctx = ctx
        self.missions = missions
        self.recovery = recovery
        self._dyn_seq = 0

    def inject(self, event_type: str, drone_id: str | None = None, params: dict | None = None) -> dict:
        params = params or {}
        try:
            etype = InjectedEventType(event_type)
        except ValueError:
            raise CommandRejected("INVALID_EVENT", f"unknown event type '{event_type}'") from None
        if etype in DRONE_EVENTS:
            return self._inject_drone(etype, drone_id, params)
        return self._inject_world(etype, drone_id, params)

    # ------------------------------------------------------------------ drone-targeted
    def _inject_drone(self, etype: InjectedEventType, drone_id: str | None, params: dict) -> dict:
        if not drone_id:
            raise CommandRejected("INVALID_EVENT", f"{etype.value} requires a drone_id")
        d = self.ctx.engine.drones.get(drone_id)
        if d is None:
            raise NotFound(f"drone {drone_id}")
        try:
            desc = apply_drone_fault(d, etype, params)
        except ValueError as e:
            raise CommandRejected("INVALID_PARAMS", str(e)) from None
        self.ctx.log(EventType.INJECTED_EVENT, f"[INJECTED] {etype.value} on {drone_id}: {desc}",
                     drone_id=drone_id, mission_id=d.current_mission_id, severity="WARNING",
                     data={"event": etype.value, "params": params, "effect": desc})
        if etype == InjectedEventType.DRONE_FAILURE:
            self._fail_drone(d)
        return {"event": etype.value, "drone_id": drone_id, "effect": desc}

    def _fail_drone(self, d) -> None:
        was_flying = d.flight_mode not in (FlightMode.IDLE,) or d.is_airborne
        if was_flying:
            try:
                self.recovery.control.emergency(d.drone_id, "DRONE_FAILURE", actor="system")
            except ServiceError:
                pass
        d.status = DroneStatus.FAILED
        self.recovery.control.on_manual_drone_failed(d)
        mission = self.ctx.missions.get(d.current_mission_id) if d.current_mission_id else None
        self.ctx.log(EventType.FAILURE_DETECTED, f"{d.drone_id} has FAILED", drone_id=d.drone_id,
                     mission_id=d.current_mission_id, severity="CRITICAL", data={"reason": "DRONE_FAILURE"})
        if mission is not None and mission.is_running and d.drone_id in mission.assigned_drones:
            self.missions.release_and_reassign(mission, d, "drone failure")
        elif mission is not None and not mission.is_running:
            d.current_mission_id = None if not was_flying else d.current_mission_id
            if mission.status.value == "READY":
                self.missions.plan(mission)         # reserved drone failed before start: replan

    # ------------------------------------------------------------------ world events
    def _inject_world(self, etype: InjectedEventType, drone_id: str | None, params: dict) -> dict:
        w = self.ctx.settings.world
        size = params.get("size", 70.0 if etype == InjectedEventType.OBSTACLE_APPEARED else 220.0)
        if isinstance(size, bool) or not isinstance(size, (int, float)) or not 20 <= size <= 600:
            raise CommandRejected("INVALID_PARAMS", "'size' must be a number between 20 and 600")
        if "x" in params and "y" in params:
            cx, cy = params["x"], params["y"]
            if not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in (cx, cy)):
                raise CommandRejected("INVALID_PARAMS", "x and y must be numbers")
        elif drone_id:
            d = self.ctx.engine.drones.get(drone_id)
            if d is None:
                raise NotFound(f"drone {drone_id}")
            spot = self._spot_ahead(d, size)
            if spot is None:
                raise CommandRejected("INVALID_PARAMS", f"{drone_id} has no route to block")
            cx, cy = spot
        else:
            raise CommandRejected("INVALID_PARAMS", "provide x and y, or a drone_id to block its route")
        if not (0 <= cx <= w.width and 0 <= cy <= w.height):
            raise CommandRejected("INVALID_PARAMS", "location outside the world")
        for d in self.ctx.engine.drones_iter():
            if etype == InjectedEventType.OBSTACLE_APPEARED and distance((cx, cy), (d.home_x, d.home_y)) < size:
                raise CommandRejected("INVALID_PARAMS", "obstacle would cover a home base")
        self._dyn_seq += 1
        kind = ZoneKind.OBSTACLE if etype == InjectedEventType.OBSTACLE_APPEARED else ZoneKind.WEATHER
        zid = f"{'OBS' if kind == ZoneKind.OBSTACLE else 'WX'}-DYN-{self._dyn_seq}"
        half = size / 2
        self.ctx.grid.add_zone(Zone(zid, kind, cx - half, cy - half, cx + half, cy + half,
                                    name="Obstacle" if kind == ZoneKind.OBSTACLE else "Weather risk"))
        self.ctx.log(EventType.INJECTED_EVENT, f"[INJECTED] {etype.value} at ({cx:.0f}, {cy:.0f}) size {size:.0f} m",
                     severity="WARNING", data={"event": etype.value, "zone_id": zid, "x": cx, "y": cy,
                                               "size": size})
        replanned = self.missions.on_grid_changed(etype.value)
        return {"event": etype.value, "zone_id": zid, "replanned_drones": replanned}

    def _spot_ahead(self, d, size: float = 70.0):
        """A point on the drone's remaining route that blocks it without sitting on an inspection
        waypoint: scan the route, require clearance from every remaining non-transit waypoint, stay
        away from home bases and from the drone itself, prefer the nearest such spot ahead."""
        from app.core.enums import WaypointAction
        remaining = d.route[d.current_waypoint_index:]
        pts = [d.position] + [w.xy for w in remaining]
        if len(pts) < 2:
            return None
        keys = [w.xy for w in remaining if w.action in (WaypointAction.INSPECT, WaypointAction.HOVER)]
        homes = [(x.home_x, x.home_y) for x in self.ctx.engine.drones_iter()]
        need = size / 2 + self.ctx.grid.margin + self.ctx.grid.cell + 5      # cell rounding + buffer
        best, best_clear, walked = None, -1.0, 0.0
        for a, b in zip(pts, pts[1:]):
            seg = distance(a, b)
            n = max(1, int(seg // 10))
            for k in range(1, n + 1):
                t = k / n
                p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
                if distance(p, d.position) <= 60 or any(distance(p, h) <= 110 for h in homes):
                    continue
                clear = min((distance(p, kp) for kp in keys), default=1e9)
                if clear >= need:
                    return p                                   # nearest acceptable spot ahead
                if clear > best_clear:
                    best, best_clear = p, clear
            walked += seg
        return best

    def remove_zone(self, zone_id: str) -> dict:
        z = self.ctx.grid.zones.get(zone_id)
        if z is None:
            raise NotFound(f"zone {zone_id}")
        if "-DYN-" not in zone_id:
            raise CommandRejected("INVALID_PARAMS", "only dynamically injected zones can be removed")
        self.ctx.grid.remove_zone(zone_id)
        self.ctx.log(EventType.INFO, f"Zone {zone_id} cleared", data={"zone_id": zone_id})
        return {"removed": zone_id}

    def clear_drone_faults(self, drone_id: str) -> dict:
        d = self.ctx.engine.drones.get(drone_id)
        if d is None:
            raise NotFound(f"drone {drone_id}")
        clear_faults(d)
        self.recovery.clear(drone_id)
        self.ctx.log(EventType.INFO, f"{drone_id} injected faults cleared", drone_id=drone_id)
        return {"drone_id": drone_id, "cleared": True}
