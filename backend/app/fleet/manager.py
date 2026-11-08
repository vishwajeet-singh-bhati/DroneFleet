"""Fleet-level queries and housekeeping (availability, charging, summary stats)."""
from __future__ import annotations

from app.core.enums import DroneStatus, FlightMode
from app.services.context import Context
from app.simulation.drone_state import DroneState
from app.utils.geometry import distance

ALTITUDE_STEP = 4.0
ALTITUDE_LAYERS = 6


def assign_altitude(ctx: Context, drone_id: str, base: float = 30.0) -> float:
    """Cruise altitude = base + 4 m * layer. A drone keeps its layer while it is busy; a new layer
    is the smallest one not held by another drone, so concurrently flying drones are vertically
    separated (up to ALTITUDE_LAYERS drones; beyond that layers are reused round-robin)."""
    layers = ctx.altitude_layers
    if drone_id not in layers:
        used = {v for k, v in layers.items() if k != drone_id}
        free = [i for i in range(ALTITUDE_LAYERS) if i not in used]
        layers[drone_id] = free[0] if free else len(layers) % ALTITUDE_LAYERS
    return base + ALTITUDE_STEP * layers[drone_id]


CHARGE_RATE_PCT_PER_S = 1.0       # accelerated for demos
READY_BATTERY = 95.0


class FleetManager:
    def __init__(self, ctx: Context) -> None:
        self.ctx = ctx

    def all(self) -> list[DroneState]:
        return list(self.ctx.engine.drones_iter())

    def get(self, drone_id: str) -> DroneState:
        return self.ctx.engine.get(drone_id)

    def available(self) -> list[DroneState]:
        return [d for d in self.all() if d.status == DroneStatus.READY and d.flight_mode == FlightMode.IDLE
                and d.current_mission_id is None]

    def tick_housekeeping(self, dt: float) -> None:
        """Charge grounded drones sitting at home. CHARGING drones become READY once charged."""
        for d in self.all():
            if d.flight_mode == FlightMode.IDLE and not d.is_airborne and d.current_mission_id is None:
                self.ctx.altitude_layers.pop(d.drone_id, None)      # release the vertical layer
            if d.flight_mode != FlightMode.IDLE or d.is_airborne:
                continue
            if d.status in (DroneStatus.FAILED, DroneStatus.OFFLINE, DroneStatus.ON_MISSION):
                continue
            at_home = distance(d.position, (d.home_x, d.home_y)) <= self.ctx.settings.sim.waypoint_threshold * 3
            if not at_home:
                continue
            if d.battery_percentage < 100.0:
                if d.status == DroneStatus.READY and d.battery_percentage >= READY_BATTERY:
                    continue
                d.status = DroneStatus.CHARGING if d.battery_percentage < READY_BATTERY else d.status
                d.battery_percentage = min(100.0, d.battery_percentage + CHARGE_RATE_PCT_PER_S * dt)
                d.temperature = max(self.ctx.settings.sim.ambient_temp, d.temperature - 0.5 * dt)
            if d.status == DroneStatus.CHARGING and d.battery_percentage >= READY_BATTERY:
                d.status = DroneStatus.READY
                d.battery_cycles += 1

    def attention_list(self) -> list[dict]:
        out = []
        for d in self.all():
            reasons = []
            if d.status == DroneStatus.FAILED:
                reasons.append("FAILED")
            if d.battery_percentage <= self.ctx.settings.sim.battery_warn and d.is_airborne:
                reasons.append(f"battery {d.battery_percentage:.0f}%")
            if d.health_percentage <= 60:
                reasons.append(f"health {d.health_percentage:.0f}%")
            if d.temperature >= self.ctx.settings.sim.max_safe_temp - 10:
                reasons.append(f"temp {d.temperature:.0f}°C")
            if d.signal_strength <= 0.3:
                reasons.append("weak signal")
            if d.gps_quality <= 0.5:
                reasons.append("poor GPS")
            if d.flight_mode in (FlightMode.EMERGENCY, FlightMode.RETURN_TO_HOME):
                reasons.append(d.flight_mode.value)
            if reasons:
                out.append({"drone_id": d.drone_id, "reasons": reasons})
        return out

    def summary(self) -> dict:
        drones = self.all()
        active = [d for d in drones if d.is_airborne or d.flight_mode not in (FlightMode.IDLE,)]
        missions = list(self.ctx.missions.values())
        running = [m for m in missions if m.is_running]
        avg_health = sum(d.health_percentage for d in drones) / len(drones) if drones else 0.0
        avg_batt = sum(d.battery_percentage for d in drones) / len(drones) if drones else 0.0
        progress = 0.0
        if running:
            progress = sum(self.mission_progress(m) for m in running) / len(running)
        return {
            "fleet_count": len(drones), "active_drones": len(active),
            "available_drones": len(self.available()),
            "active_missions": len(running), "total_missions": len(missions),
            "completed_missions": sum(1 for m in missions if m.status.value == "COMPLETED"),
            "mission_completion": round(progress, 3),
            "average_health": round(avg_health, 1), "average_battery": round(avg_batt, 1),
            "attention": self.attention_list(),
            "alerts": len([e for e in self.ctx.events.recent(200) if e["severity"] != "INFO"]),
        }

    def mission_progress(self, m) -> float:
        """Fraction (0..1) of the mission's inspection targets already inspected."""
        targets = list({t.target_id: t for per in self.ctx.mission_targets.get(m.mission_id, {}).values()
                        for t in per}.values())
        if m.status.value == "COMPLETED":
            return 1.0
        if not targets:
            return 0.0
        return self.ctx.coverage.ratio(targets)
