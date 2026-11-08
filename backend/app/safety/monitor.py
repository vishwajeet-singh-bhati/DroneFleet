"""Rule-based safety monitor.

Evaluates the *state* of a drone each tick and returns findings with a required action.
It encodes safety constraints (can I still make it home? is it overheating? has the link
been lost?) and is deliberately not ML — failsafes must be predictable and auditable.
The AI layers (anomaly detection, maintenance) feed events in; they never bypass this.
"""
from __future__ import annotations

from app.core.config import SimConfig
from app.core.enums import DroneStatus, FlightMode
from app.safety.validator import MANUAL_OVERRIDE_BATTERY, SafetyFinding
from app.simulation.drone_state import DroneState
from app.utils.geometry import distance

AIRBORNE_ACTIVE_MODES = {
    FlightMode.AUTONOMOUS, FlightMode.MANUAL, FlightMode.PAUSED, FlightMode.REJOINING,
    FlightMode.TAKEOFF,
}

HEALTH_RTH = 35.0
HEALTH_EMERGENCY = 15.0
SIGNAL_LOST = 0.12
COMM_LOSS_TIMEOUT_S = 5.0
GPS_DEGRADED = 0.45
GPS_LOST = 0.15
HOME_RETURN_MARGIN = 1.35       # need 35% more battery than the bare return cost


def battery_needed_to_return(d: DroneState, cfg: SimConfig) -> float:
    """Battery (%) needed to fly straight home at cruise speed, plus a 5% landing reserve."""
    dist = distance(d.position, (d.home_x, d.home_y))
    dur = dist / max(0.1, cfg.cruise_speed * d.speed_derate)
    return (dist * cfg.battery_drain_per_metre + dur * cfg.battery_hover_drain_per_s) \
        * d.drain_multiplier + 5.0


class SafetyMonitor:
    def __init__(self, cfg: SimConfig) -> None:
        self.cfg = cfg
        self._signal_lost_since: dict[str, float] = {}

    def evaluate(self, d: DroneState, now: float) -> list[SafetyFinding]:
        if d.status in (DroneStatus.FAILED, DroneStatus.OFFLINE) or not d.is_airborne:
            self._signal_lost_since.pop(d.drone_id, None)
            return []
        if d.flight_mode in (FlightMode.LANDING, FlightMode.EMERGENCY, FlightMode.IDLE):
            return []
        out: list[SafetyFinding] = []
        returning = d.flight_mode == FlightMode.RETURN_TO_HOME
        c = self.cfg

        # --- battery: "can I still get home?" is a stronger rule than a fixed threshold
        need = battery_needed_to_return(d, c)
        if not returning:
            if d.battery_percentage <= need * HOME_RETURN_MARGIN or d.battery_percentage <= c.battery_critical:
                sev = "CRITICAL"
                out.append(SafetyFinding(
                    "CRITICAL_BATTERY", sev, "RETURN_HOME",
                    f"battery {d.battery_percentage:.0f}% <= return requirement "
                    f"{need * HOME_RETURN_MARGIN:.0f}% (or critical {c.battery_critical:.0f}%)"))
        if d.flight_mode == FlightMode.MANUAL and d.battery_percentage <= MANUAL_OVERRIDE_BATTERY:
            out.append(SafetyFinding("CRITICAL_BATTERY", "CRITICAL", "RETURN_HOME",
                                     f"battery {d.battery_percentage:.0f}% — failsafe overrides operator"))
        if d.battery_percentage <= 3.0 and not returning:
            out.append(SafetyFinding("CRITICAL_BATTERY", "CRITICAL", "EMERGENCY_LAND",
                                     "battery nearly empty — landing immediately"))

        # --- health
        if d.health_percentage <= HEALTH_EMERGENCY:
            out.append(SafetyFinding("HEALTH", "CRITICAL", "EMERGENCY_LAND",
                                     f"health {d.health_percentage:.0f}% — unsafe to continue"))
        elif d.health_percentage <= HEALTH_RTH and not returning:
            out.append(SafetyFinding("HEALTH", "CRITICAL", "RETURN_HOME",
                                     f"health {d.health_percentage:.0f}% severely degraded"))

        # --- temperature
        if d.temperature >= c.max_safe_temp and not returning:
            out.append(SafetyFinding("OVERHEAT", "CRITICAL", "RETURN_HOME",
                                     f"temperature {d.temperature:.0f}°C exceeds {c.max_safe_temp:.0f}°C"))

        # --- communication link (failsafe after a grace period)
        if d.signal_strength <= SIGNAL_LOST:
            since = self._signal_lost_since.setdefault(d.drone_id, now)
            if now - since >= COMM_LOSS_TIMEOUT_S and not returning:
                out.append(SafetyFinding("COMM_LOSS", "CRITICAL", "RETURN_HOME",
                                         f"link lost for {now - since:.0f}s — failsafe return home"))
        else:
            self._signal_lost_since.pop(d.drone_id, None)

        # --- GPS
        if d.gps_quality <= GPS_LOST:
            out.append(SafetyFinding("GPS", "CRITICAL", "EMERGENCY_LAND",
                                     f"GPS quality {d.gps_quality:.2f} — cannot navigate, landing"))
        elif d.gps_quality <= GPS_DEGRADED:
            out.append(SafetyFinding("GPS", "WARNING", "DERATE",
                                     f"GPS quality {d.gps_quality:.2f} degraded — reducing speed"))
        return out
