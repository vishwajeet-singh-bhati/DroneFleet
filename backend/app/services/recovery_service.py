"""Automatic recovery: turns safety findings into actions.

Detect abnormal state -> decide whether the mission can continue -> if not, take the drone off
the mission (return home / emergency land) -> select a replacement -> replan -> continue.
The drone-level reaction is deterministic safety logic; replacement selection reuses the
explainable selector but every candidate is validated by the safety layer first.
"""
from __future__ import annotations

from app.core.enums import EventType
from app.safety.monitor import SafetyFinding
from app.services.context import Context
from app.services.control_service import ControlService
from app.services.errors import ServiceError
from app.services.mission_service import MissionService
from app.safety.validator import CommandRejected
from app.simulation.drone_state import DroneState


class RecoveryService:
    def __init__(self, ctx: Context, control: ControlService, missions: MissionService) -> None:
        self.ctx = ctx
        self.control = control
        self.missions = missions
        self._handled: set[tuple[str, str, str]] = set()

    def clear(self, drone_id: str) -> None:
        self._handled = {k for k in self._handled if k[0] != drone_id}

    def handle_findings(self, d: DroneState, findings: list[SafetyFinding]) -> None:
        # most drastic action first so an emergency is never starved by a softer finding
        order = {"EMERGENCY_LAND": 0, "RETURN_HOME": 1, "DERATE": 2, "NONE": 3}
        for f in sorted(findings, key=lambda x: order.get(x.action, 9)):
            key = (d.drone_id, f.kind, f.action)
            if key in self._handled:
                continue
            self._handled.add(key)
            self.ctx.log(EventType.FAILURE_DETECTED, f"{d.drone_id} {f.kind}: {f.reason}",
                         drone_id=d.drone_id, mission_id=d.current_mission_id, severity=f.severity,
                         data={"kind": f.kind, "action": f.action, "reason": f.reason})
            if f.action == "DERATE":
                d.speed_derate = min(d.speed_derate, 0.5)
            elif f.action in ("RETURN_HOME", "EMERGENCY_LAND"):
                self.ctx.decide("SAFETY_ACTION",
                                f"Safety layer: {d.drone_id} → {f.action.replace('_', ' ')} ({f.reason})",
                                mission_id=d.current_mission_id, drone_id=d.drone_id,
                                reasons=[f.reason, "deterministic safety rule (not ML)"],
                                extra={"kind": f.kind})
                self.handle_drone_loss(d, f.reason, f.action)

    def handle_drone_loss(self, d: DroneState, reason: str, action: str = "RETURN_HOME",
                          actor: str = "system") -> None:
        """Drone can no longer continue: make it safe, then fix the mission around it."""
        mission = self.ctx.missions.get(d.current_mission_id) if d.current_mission_id else None
        try:
            if action == "RETURN_HOME":
                self.control.return_home(d.drone_id, reason, actor)
            else:
                self.control.emergency(d.drone_id, reason, actor)
        except (CommandRejected, ServiceError) as e:
            # e.g. already landing / already returning — still fix the mission below
            self.ctx.log(EventType.INFO, f"{d.drone_id}: {getattr(e, 'message', str(e))}",
                         drone_id=d.drone_id)
        if mission is not None and mission.is_running and d.drone_id in mission.assigned_drones:
            self.missions.release_and_reassign(mission, d, reason)
