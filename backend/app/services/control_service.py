"""Drone-level control: human takeover, manual flight, resume-with-rejoin, pause, return-to-home,
emergency, and handling of route completion / landing.

All mode changes go through the FlightModeFSM. All operator input passes the safety validator.
The frontend never mutates drone state; it sends commands that end up here.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from app.control.rejoin import RejoinPlan, plan_rejoin
from app.fleet.manager import assign_altitude
from app.control.state_machine import FlightModeFSM, InvalidModeTransition
from app.core.enums import DroneStatus, EventType, FlightMode
from app.core.types import Waypoint
from app.mission.estimates import required_battery_pct
from app.safety.validator import CommandRejected, check_takeover, validate_manual_input
from app.services.context import Context
from app.services.errors import InvalidState, NotFound
from app.simulation.drone_state import DroneState, ManualInput

MANUAL_INPUT_HOLD_S = 0.6        # operator input is valid this long (dead-man behaviour)


@dataclass
class SavedProgress:
    """Mission progress captured at takeover so the mission can be resumed exactly."""
    mission_id: str
    route: list[Waypoint]
    index: int
    prior_mode: FlightMode


@dataclass
class ControlSession:
    drone_id: str
    operator: str
    started_at: float
    accepted: int = 0
    rejected: int = 0


@dataclass
class RejoinState:
    plan: RejoinPlan
    mission_id: str


class ControlService:
    def __init__(self, ctx: Context) -> None:
        self.ctx = ctx
        self.fsm = FlightModeFSM(ctx.engine.set_mode, self._on_transition)
        self.saved: dict[str, SavedProgress] = {}
        self.sessions: dict[str, ControlSession] = {}
        self.rejoins: dict[str, RejoinState] = {}
        self.paused_from: dict[str, FlightMode] = {}
        self._handshake: set[str] = set()
        self._rejoin_seq = 0
        self.mission_service = None                    # wired by orchestrator
        # (drone, reason, action) -> None ; set by orchestrator to RecoveryService.handle_drone_loss
        self.recovery_hook: Callable[[DroneState, str, str], None] = lambda d, r, a: None

    # ------------------------------------------------------------------ helpers
    def _drone(self, drone_id: str) -> DroneState:
        try:
            return self.ctx.engine.get(drone_id)
        except KeyError:
            raise NotFound(f"drone {drone_id}") from None

    def _on_transition(self, d: DroneState, frm: FlightMode, to: FlightMode, reason: str, actor: str) -> None:
        label_from = "TAKEOVER_REQUEST" if (to == FlightMode.MANUAL and d.drone_id in self._handshake) \
            else frm.value
        self._handshake.discard(d.drone_id)
        self.ctx.log(EventType.MODE_CHANGED, f"{d.drone_id} {label_from} → {to.value} (Reason: {reason})",
                     drone_id=d.drone_id, mission_id=d.current_mission_id,
                     data={"from": label_from, "to": to.value, "reason": reason, "actor": actor})

    def _transition(self, d: DroneState, to: FlightMode, reason: str, actor: str = "system") -> bool:
        try:
            return self.fsm.transition(d, to, reason, actor)
        except InvalidModeTransition as e:
            self.ctx.log(EventType.COMMAND_REJECTED, str(e), drone_id=d.drone_id, severity="WARNING",
                         data={"reason": "INVALID_TRANSITION", "from": e.frm.value, "to": e.to.value})
            raise InvalidState(str(e)) from e

    def _reject(self, d: DroneState, code: str, message: str) -> CommandRejected:
        self.ctx.log(EventType.COMMAND_REJECTED, f"{d.drone_id}: {message}", drone_id=d.drone_id,
                     mission_id=d.current_mission_id, severity="WARNING", data={"reason": code})
        return CommandRejected(code, message)

    def _end_session(self, d: DroneState, reason: str) -> None:
        sess = self.sessions.pop(d.drone_id, None)
        if sess is None:
            return
        d.manual_input = ManualInput()
        d.manual_input_expires = 0.0
        if self.ctx.repo:
            self.ctx.repo.save_control_session(d.drone_id, sess.operator, sess.started_at, self.ctx.now,
                                               reason, sess.accepted, sess.rejected)
        self.ctx.log(EventType.CONTROL_RELEASED, f"{d.drone_id} manual control ended: {reason}",
                     drone_id=d.drone_id, mission_id=d.current_mission_id,
                     data={"reason": reason, "accepted": sess.accepted, "rejected": sess.rejected})

    # ------------------------------------------------------------------ takeover
    def takeover(self, drone_id: str, operator: str = "operator") -> dict:
        d = self._drone(drone_id)
        if d.flight_mode == FlightMode.MANUAL:
            self.ctx.log(EventType.INFO, f"{d.drone_id} duplicate takeover ignored (already MANUAL)",
                         drone_id=d.drone_id)
            return {"changed": False, "flight_mode": d.flight_mode.value,
                    "message": "already under manual control"}
        try:
            check_takeover(d)
        except CommandRejected as e:
            raise self._reject(d, e.code, e.message)

        prior = d.flight_mode
        # Capture mission progress so RESUME can continue exactly where the mission left off.
        if d.current_mission_id and d.route_label == "MISSION" and d.route:
            idx = d.current_waypoint_index + (1 if d.hover_remaining > 0 else 0)   # dwell point is done
            self.saved[d.drone_id] = SavedProgress(d.current_mission_id, list(d.route), idx, prior)
        elif d.route_label == "REJOIN":
            self.rejoins.pop(d.drone_id, None)                     # abandon the rejoin; keep saved progress
        elif d.route_label == "HOME":
            self.saved.pop(d.drone_id, None)

        self.ctx.log(EventType.MODE_CHANGED, f"{d.drone_id} {prior.value} → TAKEOVER_REQUEST "
                     f"(Reason: Operator takeover)", drone_id=d.drone_id, mission_id=d.current_mission_id,
                     data={"from": prior.value, "to": "TAKEOVER_REQUEST", "reason": "Operator takeover",
                           "actor": operator})
        self._handshake.add(d.drone_id)
        self._transition(d, FlightMode.MANUAL, "Operator takeover granted", operator)
        d.hover_remaining = 0.0
        d.manual_input = ManualInput()
        d.manual_input_expires = 0.0
        self.paused_from.pop(d.drone_id, None)
        self.sessions[d.drone_id] = ControlSession(d.drone_id, operator, self.ctx.now)
        self.ctx.log(EventType.CONTROL_TAKEOVER, f"Operator took control of {d.drone_id}",
                     drone_id=d.drone_id, mission_id=d.current_mission_id,
                     data={"operator": operator, "prior_mode": prior.value,
                           "saved_waypoint_index": self.saved[d.drone_id].index if d.drone_id in self.saved else None})
        return {"changed": True, "flight_mode": d.flight_mode.value, "prior_mode": prior.value}

    def manual_input(self, drone_id: str, raw: dict) -> dict:
        d = self._drone(drone_id)
        sess = self.sessions.get(drone_id)
        if d.flight_mode != FlightMode.MANUAL or sess is None:
            raise CommandRejected("NOT_IN_MANUAL", f"{drone_id} is {d.flight_mode.value}; take control first")
        if d.status in (DroneStatus.FAILED, DroneStatus.OFFLINE):
            raise CommandRejected("DRONE_UNAVAILABLE", f"{drone_id} is {d.status.value}")
        try:
            inp = validate_manual_input(raw)
        except CommandRejected as e:
            sess.rejected += 1
            raise self._reject(d, e.code, e.message)
        d.manual_input = inp
        d.manual_input_expires = self.ctx.now + MANUAL_INPUT_HOLD_S
        sess.accepted += 1
        return {"accepted": True, "input": {"forward": inp.forward, "strafe": inp.strafe,
                                            "climb": inp.climb, "yaw": inp.yaw}}

    # ------------------------------------------------------------------ resume mission (rejoin)
    def release_control(self, drone_id: str, operator: str = "operator") -> dict:
        d = self._drone(drone_id)
        if d.flight_mode != FlightMode.MANUAL:
            if d.flight_mode in (FlightMode.REJOINING, FlightMode.AUTONOMOUS):
                self.ctx.log(EventType.INFO, f"{d.drone_id} duplicate release ignored "
                             f"(already {d.flight_mode.value})", drone_id=d.drone_id)
                return {"changed": False, "flight_mode": d.flight_mode.value, "message": "already autonomous"}
            raise self._reject(d, "NOT_IN_MANUAL", f"{d.drone_id} is {d.flight_mode.value}, not MANUAL")

        saved = self.saved.get(d.drone_id)
        mission = self.ctx.missions.get(saved.mission_id) if saved else None
        if saved is None or mission is None or mission.is_terminal or d.drone_id not in mission.assigned_drones:
            self._end_session(d, "released by operator")
            self.saved.pop(d.drone_id, None)
            self._transition(d, FlightMode.PAUSED, "Control released with no active mission: holding position",
                             operator)
            return {"changed": True, "flight_mode": d.flight_mode.value, "outcome": "HOLDING",
                    "message": "no active mission to resume; holding position"}

        self._rejoin_seq += 1
        home = (d.home_x, d.home_y)
        plan = plan_rejoin(d.position, saved.route, saved.index, self.ctx.planner, home,
                           prefix=f"RJ{self._rejoin_seq}-")
        if plan is None:                                  # nothing left on the route
            self._end_session(d, "released by operator")
            self.saved.pop(d.drone_id, None)
            self.recovery_hook(d, "mission route already complete", "RETURN_HOME")
            return {"changed": True, "flight_mode": d.flight_mode.value, "outcome": "RETURNING_HOME"}

        # Safety authority: can we afford rejoin + the rest of the mission and still get home?
        tail = plan.mission_route[plan.entry_index:]
        need = required_battery_pct(d.position, plan.path + tail, self.ctx.settings.sim,
                                    d.drain_multiplier, d.speed_derate)
        if d.battery_percentage < need:
            msg = (f"Resume refused: rejoin + remaining route needs {need:.0f}% battery, "
                   f"{d.drone_id} has {d.battery_percentage:.0f}% — returning home and reassigning")
            self.ctx.log(EventType.COMMAND_REJECTED, msg, drone_id=d.drone_id, mission_id=mission.mission_id,
                         severity="WARNING", data={"reason": "INSUFFICIENT_BATTERY_FOR_REJOIN"})
            self.ctx.decide("REJOIN_REFUSED", msg, mission_id=mission.mission_id, drone_id=d.drone_id,
                            reasons=[f"required {need:.0f}%", f"available {d.battery_percentage:.0f}%"])
            self._end_session(d, "safety: insufficient battery to resume")
            self.recovery_hook(d, "insufficient battery to rejoin", "RETURN_HOME")
            return {"changed": True, "flight_mode": d.flight_mode.value, "outcome": "RETURNING_HOME",
                    "message": msg}

        if plan.strategy == "REPLAN":
            mission.route_revision += 1
            mission.routes[d.drone_id] = plan.mission_route
            mission.route_history.append({"revision": mission.route_revision, "drone_id": d.drone_id,
                                          "t": round(self.ctx.now, 1), "reason": "route invalid at rejoin",
                                          "old": [[w.x, w.y] for w in saved.route[saved.index:]],
                                          "new": [[w.x, w.y] for w in plan.mission_route[plan.entry_index:]],
                                          "dropped": plan.dropped_ids})
            self.ctx.log(EventType.ROUTE_REPLANNED,
                         f"{d.drone_id} mission route replanned at rejoin (old route blocked)",
                         drone_id=d.drone_id, mission_id=mission.mission_id,
                         data={"reason": "route invalid at rejoin", "dropped": plan.dropped_ids})
            self.ctx.persist_mission(mission)

        self._end_session(d, "released by operator — resuming mission")
        self.rejoins[d.drone_id] = RejoinState(plan, mission.mission_id)
        d.set_route(plan.path, "REJOIN")
        self._transition(d, FlightMode.REJOINING, "Operator resumed mission", operator)
        self.ctx.log(EventType.ROUTE_REJOIN_STARTED,
                     f"{d.drone_id} rejoining {mission.mission_id} route ({plan.strategy}, "
                     f"{plan.off_route_distance:.0f} m off route, {plan.path_length:.0f} m to rejoin point)",
                     drone_id=d.drone_id, mission_id=mission.mission_id,
                     data={"strategy": plan.strategy, "rejoin_point": list(plan.rejoin_point),
                           "path": [[w.x, w.y] for w in plan.path], "entry_index": plan.entry_index,
                           "off_route_distance": round(plan.off_route_distance, 1),
                           "skipped": plan.skipped_ids})
        best = [c for c in plan.candidates if c.get("feasible")]
        self.ctx.decide("ROUTE_REJOIN",
                        f"{d.drone_id} rejoin: {plan.strategy} at ({plan.rejoin_point[0]:.0f}, "
                        f"{plan.rejoin_point[1]:.0f}), resume at waypoint {plan.entry_index + 1}",
                        mission_id=mission.mission_id, drone_id=d.drone_id,
                        reasons=[f"{len(best)} feasible candidate(s) evaluated",
                                 f"off-route distance {plan.off_route_distance:.0f} m",
                                 f"skipped transit waypoints: {len(plan.skipped_ids)}",
                                 "no inspection waypoints skipped"],
                        extra={"candidates": plan.candidates})
        return {"changed": True, "flight_mode": d.flight_mode.value, "outcome": "REJOINING",
                "rejoin": plan.to_dict()}

    def _complete_rejoin(self, d: DroneState) -> None:
        state = self.rejoins.pop(d.drone_id, None)
        self.saved.pop(d.drone_id, None)
        if state is None:
            return
        plan = state.plan
        mission = self.ctx.missions.get(state.mission_id)
        if mission is None or mission.is_terminal or d.drone_id not in mission.assigned_drones:
            self._transition(d, FlightMode.PAUSED, "rejoin finished but mission no longer active")
            return
        d.set_route(plan.mission_route, "MISSION", start_index=plan.entry_index)
        self._transition(d, FlightMode.AUTONOMOUS, "Rejoin complete: resuming mission")
        self.ctx.log(EventType.ROUTE_REJOIN_COMPLETED,
                     f"{d.drone_id} rejoined {mission.mission_id} at waypoint {plan.entry_index + 1}/"
                     f"{len(plan.mission_route)} and resumed autonomous mission",
                     drone_id=d.drone_id, mission_id=mission.mission_id,
                     data={"entry_index": plan.entry_index, "strategy": plan.strategy})
        if self.mission_service is not None:
            self.mission_service.on_grid_changed("route revalidated after rejoin")

    # ------------------------------------------------------------------ pause / resume
    def pause(self, drone_id: str, reason: str = "operator pause", actor: str = "operator") -> dict:
        d = self._drone(drone_id)
        if d.flight_mode == FlightMode.PAUSED:
            return {"changed": False, "flight_mode": d.flight_mode.value, "message": "already paused"}
        if d.flight_mode not in (FlightMode.AUTONOMOUS, FlightMode.REJOINING, FlightMode.TAKEOFF,
                                 FlightMode.RETURN_TO_HOME, FlightMode.MANUAL):
            raise self._reject(d, "INVALID_STATE", f"cannot pause while {d.flight_mode.value}")
        prior = d.flight_mode
        if prior == FlightMode.MANUAL:
            self._end_session(d, "paused by operator")
        self.paused_from[d.drone_id] = prior
        self._transition(d, FlightMode.PAUSED, reason, actor)
        return {"changed": True, "flight_mode": d.flight_mode.value}

    def resume(self, drone_id: str, actor: str = "operator") -> dict:
        d = self._drone(drone_id)
        if d.flight_mode in (FlightMode.AUTONOMOUS, FlightMode.REJOINING, FlightMode.RETURN_TO_HOME):
            return {"changed": False, "flight_mode": d.flight_mode.value, "message": "already running"}
        if d.flight_mode != FlightMode.PAUSED:
            raise self._reject(d, "INVALID_STATE", f"cannot resume from {d.flight_mode.value}")
        target = {"MISSION": FlightMode.AUTONOMOUS, "REJOIN": FlightMode.REJOINING,
                  "HOME": FlightMode.RETURN_TO_HOME}.get(d.route_label)
        if target is None or d.route_complete:
            raise self._reject(d, "NOTHING_TO_RESUME", f"{d.drone_id} has no route to resume")
        self.paused_from.pop(d.drone_id, None)
        self._transition(d, target, "Resumed", actor)
        return {"changed": True, "flight_mode": d.flight_mode.value}

    # ------------------------------------------------------------------ return home / emergency
    def return_home(self, drone_id: str, reason: str = "operator command", actor: str = "operator") -> dict:
        d = self._drone(drone_id)
        if d.flight_mode == FlightMode.RETURN_TO_HOME:
            return {"changed": False, "flight_mode": d.flight_mode.value, "message": "already returning home"}
        if d.flight_mode in (FlightMode.IDLE, FlightMode.LANDING, FlightMode.EMERGENCY) \
                or d.status in (DroneStatus.FAILED, DroneStatus.OFFLINE):
            raise self._reject(d, "INVALID_STATE", f"cannot return home while {d.flight_mode.value}")
        plan = self.ctx.planner.return_home(
            d.position, (d.home_x, d.home_y), prefix=f"H{d.drone_id}-",
            altitude=assign_altitude(self.ctx, d.drone_id, self.ctx.settings.sim.default_altitude))
        if not plan.ok:
            self.ctx.log(EventType.FAILURE_DETECTED, f"{d.drone_id}: no route home — emergency landing",
                         drone_id=d.drone_id, severity="CRITICAL")
            return self.emergency(drone_id, "no route home", actor="system")
        if d.flight_mode == FlightMode.MANUAL:
            self._end_session(d, f"return home: {reason}")
        self.rejoins.pop(d.drone_id, None)
        self.saved.pop(d.drone_id, None)
        self.paused_from.pop(d.drone_id, None)
        d.set_route(plan.route, "HOME")
        d.status = DroneStatus.RETURNING
        self._transition(d, FlightMode.RETURN_TO_HOME, reason, actor)
        self.ctx.log(EventType.RETURN_HOME_STARTED, f"{d.drone_id} returning home ({reason})",
                     drone_id=d.drone_id, mission_id=d.current_mission_id, severity="WARNING",
                     data={"reason": reason, "path": [[w.x, w.y] for w in plan.route],
                           "length": round(plan.length, 1)})
        return {"changed": True, "flight_mode": d.flight_mode.value, "route_length": round(plan.length, 1)}

    def emergency(self, drone_id: str, reason: str = "operator emergency", actor: str = "operator") -> dict:
        d = self._drone(drone_id)
        if d.flight_mode == FlightMode.EMERGENCY:
            return {"changed": False, "flight_mode": d.flight_mode.value, "message": "already in emergency"}
        if d.flight_mode == FlightMode.IDLE and not d.is_airborne:
            raise self._reject(d, "INVALID_STATE", f"{d.drone_id} is on the ground")
        if d.flight_mode == FlightMode.MANUAL:
            self._end_session(d, f"emergency: {reason}")
        self.rejoins.pop(d.drone_id, None)
        self.saved.pop(d.drone_id, None)
        self._transition(d, FlightMode.EMERGENCY, reason, actor)
        self.ctx.log(EventType.FAILURE_DETECTED, f"{d.drone_id} EMERGENCY: {reason}", drone_id=d.drone_id,
                     mission_id=d.current_mission_id, severity="CRITICAL", data={"reason": reason})
        return {"changed": True, "flight_mode": d.flight_mode.value}

    # ------------------------------------------------------------------ simulator event handlers
    def on_route_completed(self, d: DroneState, label: str) -> None:
        if d.route_label != label:
            return                                         # stale event
        if label == "MISSION" and d.flight_mode == FlightMode.AUTONOMOUS:
            self._transition(d, FlightMode.LANDING, "Mission route complete (at home)")
        elif label == "REJOIN" and d.flight_mode == FlightMode.REJOINING:
            self._complete_rejoin(d)
        elif label == "HOME" and d.flight_mode == FlightMode.RETURN_TO_HOME:
            self.ctx.log(EventType.RETURN_HOME_COMPLETED, f"{d.drone_id} arrived home",
                         drone_id=d.drone_id, mission_id=d.current_mission_id)
            self._transition(d, FlightMode.LANDING, "Arrived home")

    def on_landed(self, d: DroneState, emergency: bool) -> None:
        mission = self.ctx.missions.get(d.current_mission_id) if d.current_mission_id else None
        if d.status != DroneStatus.FAILED:
            d.status = DroneStatus.OFFLINE if emergency else DroneStatus.READY
        self.saved.pop(d.drone_id, None)
        self.rejoins.pop(d.drone_id, None)
        self.paused_from.pop(d.drone_id, None)
        if d.drone_id in self.sessions:
            self._end_session(d, "drone landed")
        self.ctx.log(EventType.MODE_CHANGED, f"{d.drone_id} LANDING → IDLE (Reason: landed"
                     f"{' after emergency' if emergency else ''})", drone_id=d.drone_id,
                     mission_id=d.current_mission_id,
                     data={"from": "LANDING", "to": "IDLE", "reason": "landed"})
        d.current_mission_id = None
        if mission is not None and d.drone_id in mission.assigned_drones and not emergency \
                and d.status != DroneStatus.FAILED:
            mission.done_drones.add(d.drone_id)
            if self.mission_service is not None:
                self.mission_service.check_completion(mission)
        elif mission is not None and self.mission_service is not None:
            self.mission_service.check_completion(mission)

    def on_manual_drone_failed(self, d: DroneState) -> None:
        if d.drone_id in self.sessions:
            self._end_session(d, "drone failed during manual control")
