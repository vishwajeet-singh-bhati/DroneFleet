"""Controlled failure injection: pure effects on simulated telemetry.

Injected faults only *modify the simulated world/telemetry*. Detection and reaction are the
job of the safety monitor and recovery service, exactly as they would be for real faults.
"""
from __future__ import annotations

from enum import Enum

from app.core.enums import DroneStatus
from app.simulation.drone_state import DroneState


class InjectedEventType(str, Enum):
    LOW_BATTERY = "LOW_BATTERY"
    GPS_DEGRADATION = "GPS_DEGRADATION"
    SIGNAL_LOSS = "SIGNAL_LOSS"
    HIGH_TEMPERATURE = "HIGH_TEMPERATURE"
    MOTOR_HEALTH_DROP = "MOTOR_HEALTH_DROP"
    OBSTACLE_APPEARED = "OBSTACLE_APPEARED"
    WEATHER_RISK = "WEATHER_RISK"
    DRONE_FAILURE = "DRONE_FAILURE"


DRONE_EVENTS = {
    InjectedEventType.LOW_BATTERY, InjectedEventType.GPS_DEGRADATION, InjectedEventType.SIGNAL_LOSS,
    InjectedEventType.HIGH_TEMPERATURE, InjectedEventType.MOTOR_HEALTH_DROP,
    InjectedEventType.DRONE_FAILURE,
}
WORLD_EVENTS = {InjectedEventType.OBSTACLE_APPEARED, InjectedEventType.WEATHER_RISK}


def _num(params: dict, key: str, default: float, lo: float, hi: float) -> float:
    val = params.get(key, default)
    if isinstance(val, bool) or not isinstance(val, (int, float)):
        raise ValueError(f"'{key}' must be a number")
    if not lo <= val <= hi:
        raise ValueError(f"'{key}' must be between {lo} and {hi}")
    return float(val)


def apply_drone_fault(d: DroneState, etype: InjectedEventType, params: dict) -> str:
    """Mutate telemetry for a drone-targeted fault; returns a human-readable description."""
    if etype == InjectedEventType.LOW_BATTERY:
        target = _num(params, "battery", 18.0, 0.0, 100.0)
        before = d.battery_percentage
        d.battery_percentage = min(before, target)
        return f"battery {before:.0f}% → {d.battery_percentage:.0f}%"
    if etype == InjectedEventType.GPS_DEGRADATION:
        d.gps_fault = _num(params, "severity", 0.6, 0.0, 1.0)
        return f"GPS fault {d.gps_fault:.2f} (quality will fall to ~{max(0, 0.98 - d.gps_fault):.2f})"
    if etype == InjectedEventType.SIGNAL_LOSS:
        d.signal_fault = _num(params, "severity", 1.0, 0.0, 1.0)
        return f"signal fault {d.signal_fault:.2f}"
    if etype == InjectedEventType.HIGH_TEMPERATURE:
        d.temp_fault = _num(params, "delta", 55.0, 0.0, 120.0)
        return f"motor temperature target +{d.temp_fault:.0f}°C"
    if etype == InjectedEventType.MOTOR_HEALTH_DROP:
        drop = _num(params, "drop", 45.0, 0.0, 100.0)
        before = d.health_percentage
        d.health_percentage = max(0.0, before - drop)
        d.speed_derate = min(d.speed_derate, 0.7)
        d.drain_multiplier = max(d.drain_multiplier, 1.25)
        return f"health {before:.0f}% → {d.health_percentage:.0f}% (speed x{d.speed_derate}, drain x{d.drain_multiplier})"
    if etype == InjectedEventType.DRONE_FAILURE:
        d.status = DroneStatus.FAILED
        return "drone declared failed"
    raise ValueError(f"not a drone event: {etype}")


def clear_faults(d: DroneState) -> None:
    d.gps_fault = d.signal_fault = d.temp_fault = 0.0
    d.drain_multiplier = 1.0
    d.speed_derate = 1.0
