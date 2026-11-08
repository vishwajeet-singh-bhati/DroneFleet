"""Safety authority: every command and every AI recommendation passes through here.

The AI may *recommend* ("use D05"); this module decides whether the recommendation is
allowed. It is plain deterministic rules — no ML, no LLM.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from app.core.enums import Capability, DroneStatus, FlightMode
from app.simulation.drone_state import DroneState, ManualInput

MIN_ASSIGN_BATTERY = 30.0      # % a drone needs before it may be assigned at all
MIN_ASSIGN_HEALTH = 50.0       # %
MIN_ASSIGN_SIGNAL = 0.25
MIN_ASSIGN_GPS = 0.4
MANUAL_OVERRIDE_BATTERY = 8.0  # % below which the failsafe overrides the operator

TAKEOVER_ALLOWED_MODES = {
    FlightMode.AUTONOMOUS, FlightMode.REJOINING, FlightMode.PAUSED, FlightMode.RETURN_TO_HOME,
}


class CommandRejected(Exception):
    """A command was refused by safety validation. `code` is stable for clients/tests."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


def assignment_violations(drone: DroneState, required_caps: frozenset[Capability],
                          required_battery_pct: float = 0.0, allow_mission: str | None = None) -> list[str]:
    """Reasons a drone may NOT be assigned to a mission (empty list = assignable)."""
    v: list[str] = []
    if drone.status != DroneStatus.READY:
        v.append(f"status is {drone.status.value}, not READY")
    if drone.flight_mode != FlightMode.IDLE:
        v.append(f"flight mode is {drone.flight_mode.value}, not IDLE")
    if drone.current_mission_id is not None and drone.current_mission_id != allow_mission:
        v.append(f"already occupied by {drone.current_mission_id}")
    if drone.battery_percentage < MIN_ASSIGN_BATTERY:
        v.append(f"battery {drone.battery_percentage:.0f}% below minimum {MIN_ASSIGN_BATTERY:.0f}%")
    if required_battery_pct and drone.battery_percentage < required_battery_pct:
        v.append(f"battery {drone.battery_percentage:.0f}% < {required_battery_pct:.0f}% required for route")
    if drone.health_percentage < MIN_ASSIGN_HEALTH:
        v.append(f"health {drone.health_percentage:.0f}% below minimum {MIN_ASSIGN_HEALTH:.0f}%")
    if drone.signal_strength < MIN_ASSIGN_SIGNAL:
        v.append(f"signal {drone.signal_strength:.2f} too weak")
    if drone.gps_quality < MIN_ASSIGN_GPS:
        v.append(f"GPS quality {drone.gps_quality:.2f} too low")
    missing = required_caps - drone.capabilities
    if missing:
        v.append("missing capabilities: " + ", ".join(sorted(c.value for c in missing)))
    return v


def check_takeover(drone: DroneState) -> None:
    if drone.status in (DroneStatus.FAILED, DroneStatus.OFFLINE):
        raise CommandRejected("DRONE_UNAVAILABLE", f"{drone.drone_id} is {drone.status.value}")
    if drone.flight_mode == FlightMode.EMERGENCY:
        raise CommandRejected(
            "EMERGENCY_ACTIVE",
            f"{drone.drone_id} is in EMERGENCY; the failsafe keeps control authority until it has landed")
    if drone.flight_mode not in TAKEOVER_ALLOWED_MODES and drone.flight_mode != FlightMode.MANUAL:
        raise CommandRejected(
            "INVALID_STATE",
            f"cannot take control while {drone.drone_id} is {drone.flight_mode.value}")


def validate_manual_input(raw: dict) -> ManualInput:
    """Clamp operator axes to [-1, 1]; reject non-numeric/NaN input or unknown keys."""
    allowed = {"forward", "strafe", "climb", "yaw"}
    unknown = set(raw) - allowed
    if unknown:
        raise CommandRejected("INVALID_INPUT", f"unknown control axes: {sorted(unknown)}")
    out = {}
    for k in allowed:
        val = raw.get(k, 0.0)
        if isinstance(val, bool) or not isinstance(val, (int, float)) or not math.isfinite(val):
            raise CommandRejected("INVALID_INPUT", f"axis '{k}' must be a finite number")
        out[k] = max(-1.0, min(1.0, float(val)))
    return ManualInput(**out)


@dataclass(frozen=True)
class SafetyFinding:
    kind: str            # LOW_BATTERY | CRITICAL_BATTERY | HEALTH | OVERHEAT | COMM_LOSS | GPS | DEPLETED
    severity: str        # WARNING | CRITICAL
    action: str          # NONE | RETURN_HOME | EMERGENCY_LAND | DERATE
    reason: str
