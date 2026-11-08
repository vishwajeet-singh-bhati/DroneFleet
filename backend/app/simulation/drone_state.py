"""Authoritative drone state. Only backend code mutates this; the frontend only reads snapshots."""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone

from app.core.enums import Capability, DroneStatus, FlightMode
from app.core.types import Waypoint

SIM_EPOCH = datetime(2026, 1, 1, tzinfo=timezone.utc)


def sim_iso(sim_time: float) -> str:
    """Deterministic ISO timestamp derived from simulated time (not wall clock)."""
    return (SIM_EPOCH + timedelta(seconds=sim_time)).isoformat()


@dataclass
class ManualInput:
    """Normalised operator input, each axis in [-1, 1]."""
    forward: float = 0.0     # +1 forward along heading, -1 backward
    strafe: float = 0.0      # +1 right, -1 left
    climb: float = 0.0       # +1 up, -1 down
    yaw: float = 0.0         # +1 clockwise, -1 counter-clockwise

    def is_zero(self) -> bool:
        return not (self.forward or self.strafe or self.climb or self.yaw)


@dataclass
class DroneState:
    drone_id: str
    position_x: float
    position_y: float
    altitude: float = 0.0
    velocity: float = 0.0
    heading: float = 0.0
    battery_percentage: float = 100.0
    health_percentage: float = 100.0
    temperature: float = 25.0
    gps_quality: float = 1.0           # 0..1
    signal_strength: float = 1.0       # 0..1
    current_mission_id: str | None = None
    current_waypoint_index: int = 0
    flight_mode: FlightMode = FlightMode.IDLE
    status: DroneStatus = DroneStatus.READY
    last_update: float = 0.0           # sim seconds

    # --- static / slow-changing attributes ---
    home_x: float = 0.0
    home_y: float = 0.0
    capabilities: frozenset[Capability] = frozenset({Capability.CAMERA})
    reliability: float = 0.95          # historical reliability 0..1
    flight_hours: float = 0.0
    battery_cycles: int = 0
    total_distance: float = 0.0

    # --- navigation ---
    route: list[Waypoint] = field(default_factory=list)
    route_label: str = "NONE"          # MISSION | REJOIN | HOME | NONE
    hover_remaining: float = 0.0
    manual_input: ManualInput = field(default_factory=ManualInput)
    manual_input_expires: float = 0.0

    # --- injected faults (set by the failure injector, read by dynamics) ---
    gps_fault: float = 0.0             # subtracted from gps quality
    signal_fault: float = 0.0          # subtracted from signal strength
    temp_fault: float = 0.0            # added target temperature
    drain_multiplier: float = 1.0
    speed_derate: float = 1.0          # <1 slows the drone (e.g. motor damage)
    vibration: float = 0.1             # synthetic vibration level (g RMS)

    # ------------------------------------------------------------------
    @property
    def position(self) -> tuple[float, float]:
        return (self.position_x, self.position_y)

    @property
    def active_waypoint(self) -> Waypoint | None:
        if 0 <= self.current_waypoint_index < len(self.route):
            return self.route[self.current_waypoint_index]
        return None

    @property
    def route_complete(self) -> bool:
        return bool(self.route) and self.current_waypoint_index >= len(self.route)

    @property
    def is_airborne(self) -> bool:
        return self.altitude > 0.5

    def set_route(self, route: list[Waypoint], label: str, start_index: int = 0) -> None:
        self.route = list(route)
        self.route_label = label
        self.current_waypoint_index = start_index
        self.hover_remaining = 0.0

    def clear_route(self) -> None:
        self.route = []
        self.route_label = "NONE"
        self.current_waypoint_index = 0
        self.hover_remaining = 0.0

    def to_dict(self) -> dict:
        wp = self.active_waypoint
        return {
            "drone_id": self.drone_id,
            "position_x": round(self.position_x, 2),
            "position_y": round(self.position_y, 2),
            "altitude": round(self.altitude, 2),
            "velocity": round(self.velocity, 2),
            "heading": round(self.heading, 1),
            "battery_percentage": round(self.battery_percentage, 2),
            "health_percentage": round(self.health_percentage, 2),
            "temperature": round(self.temperature, 1),
            "gps_quality": round(self.gps_quality, 3),
            "signal_strength": round(self.signal_strength, 3),
            "current_mission_id": self.current_mission_id,
            "current_waypoint_index": self.current_waypoint_index,
            "total_waypoints": len(self.route),
            "active_waypoint": wp.to_dict() if wp else None,
            "route_label": self.route_label,
            "flight_mode": self.flight_mode.value,
            "status": self.status.value,
            "capabilities": sorted(c.value for c in self.capabilities),
            "reliability": round(self.reliability, 3),
            "home": {"x": self.home_x, "y": self.home_y},
            "timestamp": sim_iso(self.last_update),
            "sim_time": round(self.last_update, 3),
        }
