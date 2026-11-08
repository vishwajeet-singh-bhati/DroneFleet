"""Cost/feasibility estimates used by drone selection and safety checks.

Estimates use the same constants as the simulator so planning and physics agree.
"""
from __future__ import annotations

from app.core.config import SimConfig
from app.core.enums import WaypointAction
from app.core.types import Waypoint
from app.utils.geometry import path_length

SAFETY_RESERVE_PCT = 8.0      # battery that must remain on landing


def route_length(position: tuple[float, float], route: list[Waypoint]) -> float:
    return path_length([position] + [w.xy for w in route])


def estimate_duration_s(length: float, route: list[Waypoint], cfg: SimConfig,
                        speed_derate: float = 1.0) -> float:
    speed = max(0.1, cfg.cruise_speed * speed_derate)
    inspects = sum(1 for w in route if w.action == WaypointAction.INSPECT)
    return length / speed + inspects * cfg.inspect_dwell_s


def estimate_battery_pct(length: float, duration_s: float, cfg: SimConfig,
                         drain_multiplier: float = 1.0) -> float:
    return (length * cfg.battery_drain_per_metre + duration_s * cfg.battery_hover_drain_per_s) \
        * drain_multiplier


def required_battery_pct(position: tuple[float, float], route: list[Waypoint], cfg: SimConfig,
                         drain_multiplier: float = 1.0, speed_derate: float = 1.0) -> float:
    """Battery needed to fly `route` (which must end at home) plus the safety reserve."""
    length = route_length(position, route)
    dur = estimate_duration_s(length, route, cfg, speed_derate)
    return estimate_battery_pct(length, dur, cfg, drain_multiplier) + SAFETY_RESERVE_PCT
