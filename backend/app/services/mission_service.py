"""Mission lifecycle: create -> plan (AI selection + routing) -> start -> run -> complete,
plus pause/resume/abort, dynamic replanning on world changes, and failure reassignment.

Layering (see docs/architecture.md): the AI selector *recommends*, this service *validates*
via the safety layer and only then assigns drones and installs routes.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from app.ai.selector import Ranking, SelectionContext, explain, rank_drones
from app.core.config import SimConfig
from app.core.enums import (
    Capability, DroneStatus, EventType, FlightMode, MissionStatus, WaypointAction, ZoneKind,
)
from app.core.types import Waypoint
from app.fleet.manager import FleetManager, assign_altitude
from app.mission.coverage import (
    InspectionTarget, lawnmower_targets, order_targets, prioritise, split_among,
)
from app.mission.estimates import estimate_duration_s, required_battery_pct, route_length
from app.mission.models import InvalidTransition, Mission
from app.mission.route_planner import HOME_ID
from app.safety.validator import assignment_violations
from app.services.context import Context
from app.services.errors import InvalidState, NotFound, PlanningFailed
from app.simulation.drone_state import DroneState
from app.utils.geometry import distance

if TYPE_CHECKING:  # pragma: no cover
    from app.services.control_service import ControlService

FLYING_MODES = {FlightMode.TAKEOFF, FlightMode.AUTONOMOUS, FlightMode.REJOINING, FlightMode.PAUSED,
                FlightMode.MANUAL}
MAX_DRONES_PER_MISSION = 6
TAKEOFF_ALLOWANCE_S = 20.0
DEADLINE_SAFETY = 0.9           # plan to finish within 90% of the deadline
INSPECT_SPACING = 60.0


@dataclass
class MissionRequest:
    name: str
    description: str = ""
    objective: str = "inspection"
    priority: int = 3
    zone_id: str = "ZONE_A"
    deadline_s: float | None = None
    required_capabilities: frozenset[Capability] = frozenset({Capability.CAMERA})
    num_drones: int = 1
    include_inspected: bool = False
    auto_start: bool = False


class MissionService:
    def __init__(self, ctx: Context, fleet: FleetManager) -> None:
        self.ctx = ctx
        self.fleet = fleet
        self.control: "ControlService" = None  # type: ignore[assignment]   # wired by orchestrator

    # =============================================================== helpers
    @property
    def cfg(self) -> SimConfig:
        return self.ctx.settings.sim

    def get(self, mission_id: str) -> Mission:
        try:
            return self.ctx.missions[mission_id]
        except KeyError:
            raise NotFound(f"mission {mission_id}") from None

    def _transition(self, m: Mission, status: MissionStatus, reason: str = "") -> None:
        try:
            m.transition(status, self.ctx.now, reason)
        except InvalidTransition as e:
            raise InvalidState(str(e)) from e
        self.ctx.persist_mission(m)

    def _release_reservations(self, m: Mission) -> None:
        for did in m.assigned_drones:
            d = self.ctx.engine.drones.get(did)
            if d and d.current_mission_id == m.mission_id and d.status == DroneStatus.READY \
                    and d.flight_mode == FlightMode.IDLE:
                d.current_mission_id = None

    def _selection_context(self, m: Mission, targets: list[InspectionTarget]) -> SelectionContext:
        xs = [t.x for t in targets]
        ys = [t.y for t in targets]
        centroid = (sum(xs) / len(xs), sum(ys) / len(ys))
        weather = sum(1 for t in targets if self.ctx.grid.risk_at(self.ctx.grid.cell_of(t.xy)) > 0)
        env = weather / len(targets)
        home = self.ctx.settings.world.home
        approx_len = distance(home, centroid) * 2 + 40.0 * len(targets)
        est_batt = approx_len * self.cfg.battery_drain_per_metre + \
            (approx_len / self.cfg.cruise_speed) * self.cfg.battery_hover_drain_per_s
        w = self.ctx.settings.world
        return SelectionContext(target=centroid, required_caps=m.required_capabilities,
                                world_diagonal=(w.width ** 2 + w.height ** 2) ** 0.5,
                                est_route_length=approx_len, est_battery_pct=est_batt, env_risk=env,
                                maintenance_risk=dict(self.ctx.maintenance_risk))

    # =============================================================== create + plan
    def create(self, req: MissionRequest) -> Mission:
        zone = self.ctx.grid.zones.get(req.zone_id)
        if zone is None or zone.kind != ZoneKind.INSPECTION:
            raise PlanningFailed(f"unknown inspection zone '{req.zone_id}'")
        if req.num_drones < 1:
            raise PlanningFailed("num_drones must be >= 1")
        if req.deadline_s is not None and req.deadline_s <= 0:
            raise PlanningFailed("deadline must be positive")
        m = Mission(mission_id=self.ctx.next_mission_id(), name=req.name, description=req.description,
                    objective=req.objective, priority=req.priority, target_zone_id=req.zone_id,
                    deadline_s=req.deadline_s, required_capabilities=req.required_capabilities,
                    num_drones=req.num_drones, created_at=self.ctx.now,
                    include_inspected=req.include_inspected)
        self.ctx.missions[m.mission_id] = m
        self.ctx.persist_mission(m)
        self.ctx.log(EventType.MISSION_CREATED, f"{m.mission_id} created: {m.name}", mission_id=m.mission_id,
                     data={"zone": req.zone_id, "num_drones": req.num_drones, "deadline_s": req.deadline_s})
        self.plan(m)
        if req.auto_start and m.status == MissionStatus.READY:
            self.start(m.mission_id)
        return m

    def _fail_planning(self, m: Mission, reason: str) -> None:
        self._release_reservations(m)
        m.assigned_drones.clear()
        m.routes.clear()
        if m.status in (MissionStatus.PLANNING,):
            self._transition(m, MissionStatus.FAILED, reason)
        self.ctx.log(EventType.MISSION_FAILED, f"{m.mission_id} planning failed: {reason}",
                     mission_id=m.mission_id, severity="WARNING", data={"reason": reason})

    def plan(self, m: Mission) -> None:
        ctx = self.ctx
        if m.status == MissionStatus.READY:
            self._release_reservations(m)
        self._transition(m, MissionStatus.PLANNING)
        m.assigned_drones, m.routes, m.target_ids = [], {}, {}

        zone = ctx.grid.zones[m.target_zone_id]
        all_targets = lawnmower_targets(zone, ctx.grid, INSPECT_SPACING)
        targets = prioritise(all_targets, ctx.coverage, m.include_inspected)
        if not targets:
            return self._fail_planning(m, "nothing to inspect (area fully inspected or blocked)")

        sctx = self._selection_context(m, targets)
        rankings = rank_drones(self.fleet.all(), sctx)
        eligible = [r for r in rankings if r.eligible]
        if not eligible:
            why = "; ".join(f"{r.drone_id}: {r.rejections[0]}" for r in rankings[:5] if r.rejections)
            return self._fail_planning(m, f"no suitable drone available ({why})")

        want = min(m.num_drones, len(eligible), MAX_DRONES_PER_MISSION)
        if want < m.num_drones:
            ctx.decide("SELECTION_NOTE", f"{m.mission_id}: {m.num_drones} drones requested but only "
                       f"{len(eligible)} eligible — planning with {want}", mission_id=m.mission_id)

        max_n = min(len(eligible), MAX_DRONES_PER_MISSION)
        last_failure = "no feasible assignment"
        accepted = None
        for n in range(want, max_n + 1):
            attempt = self._try_plan(m, eligible[:n], targets)
            if attempt["ok"]:
                accepted = attempt
                break
            last_failure = attempt["reason"]
        if accepted is None:
            return self._fail_planning(m, last_failure)

        # ---- commit (safety validation already done in _try_plan)
        for r, d, res, ordered, need, dur in accepted["assignments"]:
            m.assigned_drones.append(d.drone_id)
            m.routes[d.drone_id] = res.route
            m.target_ids[d.drone_id] = [t.target_id for t in ordered if t.target_id not in res.skipped]
            ctx.mission_targets.setdefault(m.mission_id, {})[d.drone_id] = \
                [t for t in ordered if t.target_id not in res.skipped]
            d.current_mission_id = m.mission_id            # reserve drone
            if ctx.repo:
                ctx.repo.save_assignment(m.mission_id, d.drone_id, ctx.now, "PRIMARY", "initial selection")
                ctx.repo.save_waypoints(m.mission_id, d.drone_id, 0, [w.to_dict() for w in res.route])
            ctx.decide("DRONE_SELECTION",
                       f"Selected {d.drone_id} for {m.mission_id}. Score: {r.score:.2f}. {explain(r)}",
                       mission_id=m.mission_id, drone_id=d.drone_id, score=r.score,
                       factors=r.contributions, reasons=r.reasons,
                       extra={"route_length": round(route_length(d.position, res.route), 1),
                              "estimated_battery_pct": round(need, 1),
                              "estimated_duration_s": round(dur, 1), "targets": len(ordered)})
            ctx.log(EventType.DRONE_ASSIGNED, f"{d.drone_id} assigned to {m.mission_id}",
                    drone_id=d.drone_id, mission_id=m.mission_id, data={"score": round(r.score, 3)})
            ctx.log(EventType.ROUTE_GENERATED,
                    f"Route for {d.drone_id}: {len(res.route)} waypoints, {route_length(d.position, res.route):.0f} m",
                    drone_id=d.drone_id, mission_id=m.mission_id,
                    data={"waypoints": len(res.route), "skipped": res.skipped})
        rejected = [r for r in rankings if not r.eligible]
        if rejected:
            ctx.decide("SELECTION_REJECTIONS",
                       f"{m.mission_id}: {len(rejected)} drones excluded by safety/capability constraints",
                       mission_id=m.mission_id,
                       extra={"rejected": {r.drone_id: r.rejections for r in rejected}})
        m.plan_info = {
            "targets_total": len(targets), "drones": len(accepted["assignments"]),
            "estimated_duration_s": round(accepted["max_duration"], 1),
            "skipped_targets": sorted({s for a in accepted["assignments"] for s in a[2].skipped}),
            "rankings": [r.to_dict() for r in rankings],
        }
        self._transition(m, MissionStatus.READY, "plan accepted")

    def _try_plan(self, m: Mission, chosen: list[Ranking], targets: list[InspectionTarget]) -> dict:
        ctx = self.ctx
        parts = split_among(targets, len(chosen))
        assignments = []
        max_duration = 0.0
        for r, part in zip(chosen, parts):
            d = ctx.engine.get(r.drone_id)
            if not part:
                continue
            home = (d.home_x, d.home_y)
            ordered = order_targets(d.position, part, end=home)
            res = ctx.planner.build_route(d.position, ordered, home, prefix=f"{m.mission_id[-3:]}-R0-",
                                          altitude=assign_altitude(self.ctx, d.drone_id, self.cfg.default_altitude))
            if not res.ok:
                return {"ok": False, "reason": f"{d.drone_id}: {res.reason}"}
            need = required_battery_pct(d.position, res.route, self.cfg, d.drain_multiplier, d.speed_derate)
            viol = assignment_violations(d, m.required_capabilities, need, allow_mission=m.mission_id)
            if viol:
                return {"ok": False, "reason": f"{d.drone_id} cannot fly its route: {viol[0]}"}
            dur = estimate_duration_s(route_length(d.position, res.route), res.route, self.cfg,
                                      d.speed_derate) + TAKEOFF_ALLOWANCE_S
            max_duration = max(max_duration, dur)
            assignments.append((r, d, res, ordered, need, dur))
        if not assignments:
            return {"ok": False, "reason": "no assignable work"}
        if m.deadline_s is not None and max_duration > m.deadline_s * DEADLINE_SAFETY:
            return {"ok": False, "reason": (
                f"deadline impossible: estimated {max_duration:.0f}s with {len(assignments)} drone(s) "
                f"exceeds {m.deadline_s * DEADLINE_SAFETY:.0f}s (90% of {m.deadline_s:.0f}s deadline)")}
        return {"ok": True, "assignments": assignments, "max_duration": max_duration}

    # =============================================================== start / pause / resume / abort
    def start(self, mission_id: str) -> Mission:
        ctx = self.ctx
        m = self.get(mission_id)
        if m.status != MissionStatus.READY:
            raise InvalidState(f"{m.mission_id} is {m.status.value}; only READY missions can be started")
        # Re-validate: drones may have degraded between planning and start.
        bad = []
        for did in m.assigned_drones:
            d = ctx.engine.get(did)
            need = required_battery_pct(d.position, m.routes[did], self.cfg, d.drain_multiplier, d.speed_derate)
            v = assignment_violations(d, m.required_capabilities, need, allow_mission=m.mission_id)
            if v:
                bad.append((did, v[0]))
        if bad:
            ctx.log(EventType.COMMAND_REJECTED,
                    f"{m.mission_id}: assigned drones no longer valid ({bad}); replanning before start",
                    mission_id=m.mission_id, severity="WARNING")
            self.plan(m)
            if m.status != MissionStatus.READY:
                raise PlanningFailed(m.failure_reason or "replanning failed")
        for did in m.assigned_drones:
            d = ctx.engine.get(did)
            self._launch(m, d, m.routes[did])
        self._transition(m, MissionStatus.ACTIVE, "operator start")
        ctx.log(EventType.MISSION_STARTED, f"{m.mission_id} started with {', '.join(m.assigned_drones)}",
                mission_id=m.mission_id, data={"drones": m.assigned_drones})
        return m

    def _launch(self, m: Mission, d: DroneState, route: list[Waypoint]) -> None:
        d.current_mission_id = m.mission_id
        d.status = DroneStatus.ON_MISSION
        d.set_route(route, "MISSION")
        m.baseline[d.drone_id] = {"distance": d.total_distance, "battery": d.battery_percentage,
                                  "t": self.ctx.now}
        self.ctx.engine.start_takeoff(d.drone_id, FlightMode.AUTONOMOUS)
        self.ctx.log(EventType.MODE_CHANGED, f"{d.drone_id} IDLE → TAKEOFF (started {m.mission_id})",
                     drone_id=d.drone_id, mission_id=m.mission_id,
                     data={"from": "IDLE", "to": "TAKEOFF", "reason": "mission start"})
        self.ctx.log(EventType.INFO, f"{d.drone_id} started {m.mission_id}", drone_id=d.drone_id,
                     mission_id=m.mission_id)

    def pause(self, mission_id: str) -> Mission:
        m = self.get(mission_id)
        self._transition(m, MissionStatus.PAUSED, "operator pause")
        for did in m.assigned_drones:
            d = self.ctx.engine.get(did)
            if d.flight_mode in (FlightMode.AUTONOMOUS, FlightMode.REJOINING, FlightMode.TAKEOFF):
                self.control.pause(did, "mission paused", actor="operator")
        self.ctx.log(EventType.INFO, f"{m.mission_id} paused", mission_id=m.mission_id)
        return m

    def resume(self, mission_id: str) -> Mission:
        m = self.get(mission_id)
        self._transition(m, MissionStatus.ACTIVE, "operator resume")
        for did in m.assigned_drones:
            d = self.ctx.engine.get(did)
            if d.flight_mode == FlightMode.PAUSED:
                self.control.resume(did, actor="operator")
        self.ctx.log(EventType.INFO, f"{m.mission_id} resumed", mission_id=m.mission_id)
        return m

    def abort(self, mission_id: str, reason: str = "operator cancelled") -> Mission:
        m = self.get(mission_id)
        was_reserved_only = m.status == MissionStatus.READY
        self._transition(m, MissionStatus.ABORTED, reason)
        if was_reserved_only or m.started_at is None:
            self._release_reservations(m)
        for did in list(m.assigned_drones):
            d = self.ctx.engine.get(did)
            if d.flight_mode in FLYING_MODES:
                self.control.return_home(did, f"{m.mission_id} aborted", actor="system")
            if m.started_at is not None:
                m.released_drones.append(did)
        m.assigned_drones.clear()
        self.ctx.log(EventType.INFO, f"{m.mission_id} aborted: {reason}", mission_id=m.mission_id,
                     severity="WARNING")
        m.report = self.build_report(m)
        self.ctx.persist_mission(m)
        return m

    # =============================================================== completion
    def check_completion(self, m: Mission) -> None:
        """Called when a drone lands or is released. Completes (or fails) the mission if no work is left."""
        if m.status not in (MissionStatus.ACTIVE, MissionStatus.PAUSED):
            return
        if m.assigned_drones and all(did in m.done_drones for did in m.assigned_drones):
            self._complete(m)
        elif not m.assigned_drones:
            # every drone has been released: success only if all planned targets were inspected
            targets = list({t.target_id: t for per in self.ctx.mission_targets.get(m.mission_id, {}).values()
                            for t in per}.values())
            if targets and all(self.ctx.coverage.is_inspected(t.target_id) for t in targets):
                self._complete(m)
            else:
                self._fail_running(m, "no drones left on the mission")

    def _complete(self, m: Mission) -> None:
        if m.status == MissionStatus.PAUSED:
            self._transition(m, MissionStatus.ACTIVE, "completing")
        self._transition(m, MissionStatus.COMPLETED, "all drones finished")
        m.report = self.build_report(m)
        self.ctx.persist_mission(m)
        self.ctx.log(EventType.MISSION_COMPLETED,
                     f"{m.mission_id} completed ({m.report['coverage_ratio'] * 100:.0f}% coverage)",
                     mission_id=m.mission_id, data={"coverage": m.report["coverage_ratio"]})

    def _fail_running(self, m: Mission, reason: str) -> None:
        if m.status not in (MissionStatus.ACTIVE, MissionStatus.PAUSED, MissionStatus.REPLANNING):
            return
        self._transition(m, MissionStatus.FAILED, reason)
        m.report = self.build_report(m)
        self.ctx.persist_mission(m)
        self.ctx.log(EventType.MISSION_FAILED, f"{m.mission_id} failed: {reason}", mission_id=m.mission_id,
                     severity="CRITICAL", data={"reason": reason})

    def tick(self) -> None:
        """Per-tick mission watchdog: deadline overruns."""
        for m in self.ctx.missions.values():
            if m.is_running and m.deadline_at() is not None and self.ctx.now > m.deadline_at() \
                    and not m.plan_info.get("deadline_exceeded"):
                m.plan_info["deadline_exceeded"] = True
                self.ctx.log(EventType.FAILURE_DETECTED, f"{m.mission_id} exceeded its deadline",
                             mission_id=m.mission_id, severity="WARNING", data={"reason": "DEADLINE"})

    def build_report(self, m: Mission) -> dict:
        ctx = self.ctx
        all_targets = list({t.target_id: t for per in ctx.mission_targets.get(m.mission_id, {}).values()
                            for t in per}.values())
        inspected = [t for t in all_targets if ctx.coverage.is_inspected(t.target_id)]
        uncovered = [t.target_id for t in all_targets if not ctx.coverage.is_inspected(t.target_id)]
        end = m.completed_at if m.completed_at is not None else ctx.now
        duration = (end - m.started_at) if m.started_at is not None else 0.0
        drones = []
        for did, base in m.baseline.items():
            d = ctx.engine.drones[did]
            mine = ctx.mission_targets.get(m.mission_id, {}).get(did, [])
            drones.append({
                "drone_id": did, "distance_m": round(d.total_distance - base["distance"], 1),
                "battery_used_pct": round(max(0.0, base["battery"] - d.battery_percentage), 1),
                "targets_inspected": sum(1 for t in mine if ctx.coverage.is_inspected(t.target_id)),
                "released_early": did in m.released_drones,
            })

        def count(t: EventType) -> int:
            return sum(1 for e in ctx.events.all() if e["mission_id"] == m.mission_id and e["type"] == t.value)

        return {
            "mission_id": m.mission_id, "name": m.name, "status": m.status.value,
            "duration_s": round(duration, 1), "deadline_s": m.deadline_s,
            "deadline_met": None if m.deadline_s is None else duration <= m.deadline_s,
            "targets_total": len(all_targets), "targets_inspected": len(inspected),
            "coverage_ratio": round(len(inspected) / len(all_targets), 3) if all_targets else 0.0,
            "uncovered_targets": uncovered, "drones": drones,
            "replans": count(EventType.ROUTE_REPLANNED), "takeovers": count(EventType.CONTROL_TAKEOVER),
            "rejoins": count(EventType.ROUTE_REJOIN_COMPLETED),
            "reassignments": count(EventType.MISSION_REASSIGNED),
            "anomalies": count(EventType.ANOMALY_DETECTED),
            "ai_decisions": sum(1 for x in ctx.decisions.all() if x["mission_id"] == m.mission_id),
            "route_revisions": m.route_revision,
        }

    # =============================================================== dynamic replanning
    def on_grid_changed(self, reason: str) -> list[str]:
        """The planning grid changed (obstacle/restricted/weather). Replan any affected route.

        Returns ids of drones whose route was replanned.
        """
        ctx = self.ctx
        changed: list[str] = []
        for d in ctx.engine.drones_iter():
            if not d.route or d.route_complete or d.route_label == "NONE":
                continue
            if d.flight_mode in (FlightMode.LANDING, FlightMode.EMERGENCY, FlightMode.IDLE):
                continue
            remaining = d.route[d.current_waypoint_index:]
            if d.flight_mode == FlightMode.MANUAL:
                continue      # operator is flying; routes are re-validated at rejoin time
            blocked = ctx.planner.first_blocked_segment(d.position, remaining)
            risky = self._crosses_risk(d.position, remaining)
            if blocked is None and not risky:
                continue
            if self._replan_drone(d, reason, hard_block=blocked is not None):
                changed.append(d.drone_id)
        return changed

    def _crosses_risk(self, position, remaining: list[Waypoint]) -> bool:
        grid = self.ctx.grid
        cur = position
        for w in remaining:
            n = max(1, int(distance(cur, w.xy) // (grid.cell / 2)))
            for k in range(n + 1):
                t = k / n
                p = (cur[0] + (w.x - cur[0]) * t, cur[1] + (w.y - cur[1]) * t)
                if grid.risk_at(grid.cell_of(p)) > 0:
                    return True
            cur = w.xy
        return False

    def _replan_drone(self, d: DroneState, reason: str, hard_block: bool) -> bool:
        ctx = self.ctx
        idx = d.current_waypoint_index
        old_remaining = list(d.route[idx:])
        mission = ctx.missions.get(d.current_mission_id) if d.current_mission_id else None
        rev = (mission.route_revision + 1) if mission else 1
        prefix = f"{(mission.mission_id[-3:] if mission else 'RTH')}-R{rev}-"
        home = (d.home_x, d.home_y)
        res = ctx.planner.replan_remaining(d.position, old_remaining, home, prefix)
        if not res.ok:
            ctx.log(EventType.FAILURE_DETECTED, f"{d.drone_id}: no safe route after {reason} — returning home",
                    drone_id=d.drone_id, mission_id=d.current_mission_id, severity="CRITICAL",
                    data={"reason": res.reason})
            if d.flight_mode != FlightMode.RETURN_TO_HOME:
                self.control.recovery_hook(d, f"no safe route after {reason}", "RETURN_HOME")
            return False
        # new full route keeps the completed prefix so "waypoint k/N" stays meaningful
        new_route = list(d.route[:idx]) + res.route
        if mission and d.route_label == "MISSION":
            old_xy = [[w.x, w.y] for w in old_remaining]
            mission.routes[d.drone_id] = new_route
            mission.route_revision = rev
            mission.route_history.append({"revision": rev, "drone_id": d.drone_id, "t": round(ctx.now, 1),
                                          "reason": reason, "old": old_xy,
                                          "new": [[w.x, w.y] for w in res.route], "dropped": res.skipped})
            if ctx.repo:
                ctx.repo.save_waypoints(mission.mission_id, d.drone_id, rev, [w.to_dict() for w in new_route])
            # dropped targets stay in mission_targets: they show up as *uncovered* in the report
            ctx.persist_mission(mission)
        # a drone dwelling at an inspection point keeps its dwell state
        d.route = new_route
        d.current_waypoint_index = idx
        ctx.log(EventType.ROUTE_REPLANNED,
                f"{d.drone_id} route replanned ({reason}): {len(old_remaining)} → {len(res.route)} waypoints"
                + (f", dropped {', '.join(res.skipped)}" if res.skipped else ""),
                drone_id=d.drone_id, mission_id=d.current_mission_id,
                data={"reason": reason, "old": [[w.x, w.y] for w in old_remaining],
                      "new": [[w.x, w.y] for w in res.route], "dropped": res.skipped,
                      "revision": rev, "hard_block": hard_block})
        ctx.decide("REPLAN", f"Replanned {d.drone_id} route after {reason}; "
                   f"{len(res.skipped)} waypoint(s) unavailable", mission_id=d.current_mission_id,
                   drone_id=d.drone_id,
                   reasons=[f"A* recomputed {len(res.route)} waypoints", f"new path {res.length:.0f} m"],
                   extra={"dropped": res.skipped})
        return True

    # =============================================================== failure reassignment
    def release_and_reassign(self, m: Mission, d: DroneState, reason: str) -> dict:
        """Remove `d` from the mission and give its unfinished work to a replacement drone.

        Steps: (1) compute the work still to do, (2) release the drone, (3) ask the selector for
        replacement candidates, (4) validate them with the safety layer against the *actual*
        remaining route, (5) assign + plan, else (6) fall back to other active drones, else
        (7) continue with partial coverage or fail the mission if nobody is left.
        """
        ctx = self.ctx
        result = {"replacement": None, "redistributed_to": [], "uncovered": [], "mission_status": None}
        if m.is_terminal or d.drone_id not in m.assigned_drones:
            return result
        per = ctx.mission_targets.get(m.mission_id, {}).get(d.drone_id, [])
        remaining = [t for t in per if not ctx.coverage.is_inspected(t.target_id)]
        m.assigned_drones.remove(d.drone_id)
        m.released_drones.append(d.drone_id)
        if ctx.repo:
            ctx.repo.release_assignment(m.mission_id, d.drone_id, ctx.now, reason)
        ctx.log(EventType.INFO, f"{d.drone_id}: RETURNING HOME — released from {m.mission_id} ({reason})",
                drone_id=d.drone_id, mission_id=m.mission_id, severity="WARNING")
        if not remaining:
            self.check_completion(m)
            result["mission_status"] = m.status.value
            return result

        prev_status = m.status
        if m.status in (MissionStatus.ACTIVE, MissionStatus.PAUSED):
            self._transition(m, MissionStatus.REPLANNING, f"{d.drone_id} lost: {reason}")
        ctx.log(EventType.FAILURE_DETECTED, f"{m.mission_id}: {d.drone_id} can no longer continue "
                f"({reason}); {len(remaining)} target(s) need coverage", mission_id=m.mission_id,
                drone_id=d.drone_id, severity="CRITICAL", data={"remaining": len(remaining)})

        centroid = (sum(t.x for t in remaining) / len(remaining), sum(t.y for t in remaining) / len(remaining))
        sctx = self._selection_context(m, remaining)
        sctx.target = centroid
        rankings = rank_drones(self.fleet.all(), sctx)
        replacement = None
        for r in rankings:
            if not r.eligible:
                continue
            cand = ctx.engine.get(r.drone_id)
            home = (cand.home_x, cand.home_y)
            ordered = order_targets(cand.position, remaining, end=home)
            res = ctx.planner.build_route(cand.position, ordered, home,
                                          prefix=f"{m.mission_id[-3:]}-RA{m.route_revision + 1}-",
                                          altitude=assign_altitude(self.ctx, cand.drone_id, self.cfg.default_altitude))
            if not res.ok:
                continue
            need = required_battery_pct(cand.position, res.route, self.cfg, cand.drain_multiplier,
                                        cand.speed_derate)
            if assignment_violations(cand, m.required_capabilities, need, allow_mission=None):
                continue                                   # safety layer vetoes the AI recommendation
            replacement = (r, cand, res, ordered, need)
            break

        if replacement:
            r, cand, res, ordered, need = replacement
            m.route_revision += 1
            m.assigned_drones.append(cand.drone_id)
            m.routes[cand.drone_id] = res.route
            ordered_ok = [t for t in ordered if t.target_id not in res.skipped]
            m.target_ids[cand.drone_id] = [t.target_id for t in ordered_ok]
            ctx.mission_targets[m.mission_id][cand.drone_id] = ordered_ok
            cand.current_mission_id = m.mission_id
            if ctx.repo:
                ctx.repo.save_assignment(m.mission_id, cand.drone_id, ctx.now, "REPLACEMENT",
                                         f"replaces {d.drone_id}: {reason}")
                ctx.repo.save_waypoints(m.mission_id, cand.drone_id, m.route_revision,
                                        [w.to_dict() for w in res.route])
            m.route_history.append({"revision": m.route_revision, "drone_id": cand.drone_id,
                                    "t": round(ctx.now, 1), "reason": f"replaces {d.drone_id}",
                                    "old": [], "new": [[w.x, w.y] for w in res.route], "dropped": res.skipped})
            ctx.decide("MISSION_REASSIGNMENT",
                       f"Selected {cand.drone_id} to take over {m.mission_id} from {d.drone_id}. "
                       f"Score: {r.score:.2f}. {explain(r)}",
                       mission_id=m.mission_id, drone_id=cand.drone_id, score=r.score,
                       factors=r.contributions, reasons=r.reasons + [f"replaces {d.drone_id}: {reason}"],
                       extra={"remaining_targets": len(remaining), "estimated_battery_pct": round(need, 1)})
            self._launch(m, cand, res.route)
            ctx.log(EventType.MISSION_REASSIGNED,
                    f"{cand.drone_id}: TAKING OVER {m.mission_id} from {d.drone_id}",
                    drone_id=cand.drone_id, mission_id=m.mission_id,
                    data={"from": d.drone_id, "to": cand.drone_id, "remaining": len(remaining)})
            result["replacement"] = cand.drone_id
            uncovered_ids = set(res.skipped)
        else:
            leftover = self._redistribute(m, remaining, result)
            uncovered_ids = {t.target_id for t in leftover}
            result["uncovered"] = sorted(uncovered_ids)
            if leftover:
                ctx.log(EventType.FAILURE_DETECTED,
                        f"{m.mission_id}: no replacement available — {len(leftover)} target(s) uncovered",
                        mission_id=m.mission_id, severity="CRITICAL",
                        data={"uncovered": [t.target_id for t in leftover]})
                ctx.decide("MISSION_REASSIGNMENT",
                           f"No replacement for {d.drone_id} on {m.mission_id}: "
                           f"{len(leftover)} target(s) uncovered", mission_id=m.mission_id,
                           drone_id=d.drone_id, reasons=["no eligible drone passed safety validation",
                                                         "other active drones lack battery margin"])

        # the lost drone keeps only what it finished plus anything nobody could take (reported uncovered)
        ctx.mission_targets[m.mission_id][d.drone_id] = [
            t for t in per if ctx.coverage.is_inspected(t.target_id) or t.target_id in uncovered_ids]

        if m.status == MissionStatus.REPLANNING:
            if m.assigned_drones:
                self._transition(m, MissionStatus.ACTIVE if prev_status != MissionStatus.PAUSED
                                 else MissionStatus.PAUSED, "replanning finished")
            else:
                self._fail_running(m, f"{d.drone_id} lost and no replacement drone available")
        result["mission_status"] = m.status.value
        return result

    def _redistribute(self, m: Mission, remaining: list[InspectionTarget], result: dict) -> list[InspectionTarget]:
        """Fallback: give targets to still-flying drones of this mission that can afford them."""
        ctx = self.ctx
        leftover = list(remaining)
        for did in list(m.assigned_drones):
            d = ctx.engine.get(did)
            if not leftover:
                break
            if d.flight_mode not in (FlightMode.AUTONOMOUS, FlightMode.REJOINING, FlightMode.TAKEOFF) \
                    or d.route_label != "MISSION":
                continue
            own = [t for t in ctx.mission_targets[m.mission_id].get(did, [])
                   if not ctx.coverage.is_inspected(t.target_id)]
            take = leftover[:max(1, len(leftover) // max(1, len(m.assigned_drones)))]
            ordered = order_targets(d.position, own + take, end=(d.home_x, d.home_y))
            res = ctx.planner.build_route(d.position, ordered, (d.home_x, d.home_y),
                                          prefix=f"{m.mission_id[-3:]}-RD{m.route_revision + 1}-",
                                          altitude=assign_altitude(self.ctx, d.drone_id, self.cfg.default_altitude))
            if not res.ok:
                continue
            need = required_battery_pct(d.position, res.route, self.cfg, d.drain_multiplier, d.speed_derate)
            if d.battery_percentage < need or d.health_percentage < 50:
                continue
            idx = d.current_waypoint_index
            m.route_revision += 1
            d.route = list(d.route[:idx]) + res.route
            m.routes[did] = d.route
            done_before = [t for t in ctx.mission_targets[m.mission_id].get(did, [])
                           if ctx.coverage.is_inspected(t.target_id)]
            ctx.mission_targets[m.mission_id][did] = done_before + \
                [t for t in ordered if t.target_id not in res.skipped]
            leftover = [t for t in leftover if t not in take]
            result["redistributed_to"].append(did)
            ctx.log(EventType.MISSION_REASSIGNED, f"{did} absorbs {len(take)} extra target(s) on {m.mission_id}",
                    drone_id=did, mission_id=m.mission_id, data={"extra": len(take)})
        return leftover
