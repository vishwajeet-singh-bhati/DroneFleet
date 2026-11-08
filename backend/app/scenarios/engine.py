"""Scenario engine: scripted, reproducible sessions that exercise the real system.

A scenario acts like an operator: it calls the same services the REST API calls (create mission,
take control, inject events ...). It never touches drone state directly. Each scenario declares
  * setup      - what to create/start
  * steps      - ordered actions triggered by conditions (optionally lasting some simulated seconds)
  * monitors   - per-tick invariants (e.g. "no teleporting", "never inside an obstacle")
  * checks     - assertions evaluated when the scenario finishes
The same ScenarioPlayer runs headless (tests, `run_demo.py --headless`) or attached to the live
simulation so the operator can watch a scenario play out in the UI.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

from app.core.enums import EventType, FlightMode, MissionStatus
from app.services.mission_service import MissionRequest
from app.services.orchestrator import FleetOrchestrator
from app.utils.geometry import distance

Ctx = dict[str, Any]


@dataclass
class Step:
    label: str
    when: Callable[[FleetOrchestrator, Ctx], bool]
    do: Callable[[FleetOrchestrator, Ctx], None] | None = None
    tick: Callable[[FleetOrchestrator, Ctx, float], None] | None = None   # called each tick while active
    duration: float = 0.0                                                   # simulated seconds


@dataclass
class Check:
    name: str
    fn: Callable[[FleetOrchestrator, Ctx], tuple[bool, str]]


@dataclass
class Scenario:
    name: str
    title: str
    description: str
    setup: Callable[[FleetOrchestrator, Ctx], None]
    steps: list[Step] = field(default_factory=list)
    checks: list[Check] = field(default_factory=list)
    monitors: list[Callable[[FleetOrchestrator, Ctx], None]] = field(default_factory=list)
    fleet_size: int = 12


class ScenarioPlayer:
    """Orchestrator extension that advances a scenario one tick at a time."""

    def __init__(self, scenario: Scenario) -> None:
        self.scenario = scenario
        self.ctx: Ctx = {"violations": [], "log": []}
        self._idx = 0
        self._active_since: float | None = None
        self.finished = False
        self.results: list[dict] = []
        self.started = False

    def start(self, o: FleetOrchestrator) -> None:
        self.started = True
        self.scenario.setup(o, self.ctx)
        o.ctx.log(EventType.INFO, f"Scenario '{self.scenario.title}' started")

    def __call__(self, o: FleetOrchestrator) -> None:
        if self.finished:
            return
        for mon in self.scenario.monitors:
            mon(o, self.ctx)
        steps = self.scenario.steps
        if self._idx < len(steps):
            step = steps[self._idx]
            if self._active_since is None:
                if step.when(o, self.ctx):
                    self._active_since = o.now
                    self.ctx["log"].append({"t": round(o.now, 1), "step": step.label})
                    o.ctx.log(EventType.INFO, f"[scenario] {step.label}")
                    if step.do:
                        step.do(o, self.ctx)
            if self._active_since is not None:
                elapsed = o.now - self._active_since
                if step.tick and elapsed < step.duration:
                    step.tick(o, self.ctx, elapsed)
                if elapsed >= step.duration:
                    self._idx += 1
                    self._active_since = None
        if self._idx >= len(steps) and self._all_done(o):
            self._finish(o)

    def _all_done(self, o: FleetOrchestrator) -> bool:
        ids = self.ctx.get("missions", [])
        if not ids:
            return False
        if not all(o.ctx.missions[i].is_terminal for i in ids):
            return False
        return all(d.flight_mode == FlightMode.IDLE and not d.is_airborne for d in o.engine.drones_iter())

    def _finish(self, o: FleetOrchestrator) -> None:
        self.finished = True
        self.results = []
        for c in self.scenario.checks:
            try:
                ok, detail = c.fn(o, self.ctx)
            except Exception as e:                         # a crashing check is a failed check
                ok, detail = False, f"check raised {type(e).__name__}: {e}"
            self.results.append({"name": c.name, "ok": bool(ok), "detail": detail})
        for v in self.ctx["violations"]:
            self.results.append({"name": "invariant", "ok": False, "detail": v})
        o.ctx.log(EventType.INFO, f"Scenario '{self.scenario.title}' finished: "
                  f"{'PASSED' if self.passed else 'FAILED'}")

    @property
    def passed(self) -> bool:
        return self.finished and all(r["ok"] for r in self.results)

    def status(self) -> dict:
        return {"name": self.scenario.name, "title": self.scenario.title,
                "description": self.scenario.description, "finished": self.finished,
                "passed": self.passed if self.finished else None,
                "steps_done": self._idx, "steps_total": len(self.scenario.steps),
                "checks": self.results, "log": self.ctx["log"]}


# ============================================================================ helpers
def _create_start(o: FleetOrchestrator, ctx: Ctx, zone: str = "ZONE_A", drones: int = 1, **kw) -> str:
    m = o.missions.create(MissionRequest(name=f"Inspect {zone}", zone_id=zone, num_drones=drones, **kw))
    if m.status != MissionStatus.READY:
        raise RuntimeError(f"scenario mission failed to plan: {m.failure_reason}")
    o.missions.start(m.mission_id)
    ctx.setdefault("missions", []).append(m.mission_id)
    ctx["mission"] = m.mission_id
    ctx["drones"] = list(m.assigned_drones)
    ctx["primary"] = m.assigned_drones[0]
    return m.mission_id


def _reached(key: str, n: int):
    def when(o: FleetOrchestrator, ctx: Ctx) -> bool:
        d = o.engine.get(ctx[key])
        return d.current_waypoint_index >= n and d.flight_mode == FlightMode.AUTONOMOUS
    return when


def _mission(o: FleetOrchestrator, ctx: Ctx):
    return o.ctx.missions[ctx["mission"]]


def _completed_full(o: FleetOrchestrator, ctx: Ctx) -> tuple[bool, str]:
    m = _mission(o, ctx)
    cov = (m.report or {}).get("coverage_ratio", 0)
    return m.status == MissionStatus.COMPLETED and cov == 1.0, f"status={m.status.value}, coverage={cov}"


def _landed_home(drone_key_or_id: str, by_key: bool = True):
    def fn(o: FleetOrchestrator, ctx: Ctx):
        did = ctx[drone_key_or_id] if by_key else drone_key_or_id
        d = o.engine.get(did)
        dist = distance(d.position, (d.home_x, d.home_y))
        ok = d.altitude == 0 and d.flight_mode == FlightMode.IDLE and dist <= o.settings.sim.waypoint_threshold + 1
        return ok, f"{did} altitude={d.altitude:.1f}, {dist:.1f} m from home, mode={d.flight_mode.value}"
    return fn


def _events(o: FleetOrchestrator, t: EventType, drone: str | None = None) -> list[dict]:
    return [e for e in o.ctx.events.all() if e["type"] == t.value and (drone is None or e["drone_id"] == drone)]


def monitor_no_teleport(o: FleetOrchestrator, ctx: Ctx) -> None:
    prev = ctx.setdefault("_prev_pos", {})
    limit = o.settings.sim.max_speed * o.settings.sim.dt + 1e-6
    for d in o.engine.drones_iter():
        p = prev.get(d.drone_id)
        if p is not None and distance(p, d.position) > limit:
            ctx["violations"].append(f"{d.drone_id} jumped {distance(p, d.position):.1f} m in one tick")
        prev[d.drone_id] = d.position


def monitor_no_obstacle_entry(o: FleetOrchestrator, ctx: Ctx) -> None:
    for d in o.engine.drones_iter():
        if d.is_airborne and o.ctx.grid.is_blocked_xy(d.position):
            key = f"inside:{d.drone_id}"
            if key not in ctx:
                ctx[key] = True
                ctx["violations"].append(f"{d.drone_id} entered a blocked area at ({d.position_x:.0f}, "
                                         f"{d.position_y:.0f}) t={o.now:.0f}s")


def monitor_manual_holds_route(o: FleetOrchestrator, ctx: Ctx) -> None:
    """While a drone is MANUAL its autonomous waypoint index must not advance."""
    last = ctx.setdefault("_manual_idx", {})
    for d in o.engine.drones_iter():
        if d.flight_mode == FlightMode.MANUAL:
            if d.drone_id in last and d.current_waypoint_index != last[d.drone_id]:
                ctx["violations"].append(f"{d.drone_id} advanced waypoints while under manual control")
            last[d.drone_id] = d.current_waypoint_index
        else:
            last.pop(d.drone_id, None)


def monitor_separation(o: FleetOrchestrator, ctx: Ctx) -> None:
    best = ctx.get("min_sep", 1e9)
    ds = [d for d in o.engine.drones_iter() if d.is_airborne]
    for i, a in enumerate(ds):
        for b in ds[i + 1:]:
            best = min(best, ((a.position_x - b.position_x) ** 2 + (a.position_y - b.position_y) ** 2 +
                              (a.altitude - b.altitude) ** 2) ** 0.5)
    ctx["min_sep"] = best


def _manual_script(*moves: tuple[float, dict]):
    """Return a per-tick function replaying (until_seconds, axes) pairs, e.g. (8, {'forward': 1})."""
    def tick(o: FleetOrchestrator, ctx: Ctx, elapsed: float) -> None:
        for until, axes in moves:
            if elapsed < until:
                o.control.manual_input(ctx["primary"], axes)
                return
    return tick


# ============================================================================ scenarios
def _s1() -> Scenario:
    return Scenario(
        "normal_mission", "Scenario 1: Normal mission",
        "Two drones are selected by the AI, plan routes, inspect Zone A and land.",
        setup=lambda o, c: _create_start(o, c, "ZONE_A", 2),
        checks=[Check("mission completed with full coverage", _completed_full),
                Check("both drones back home", lambda o, c: (
                    all(_landed_home(d, False)(o, c)[0] for d in c["drones"]), f"drones={c['drones']}")),
                Check("selection was explained", lambda o, c: (
                    any(x["kind"] == "DRONE_SELECTION" and x["reasons"] for x in o.ctx.decisions.all()),
                    "decision log has reasons"))],
        monitors=[monitor_no_teleport, monitor_no_obstacle_entry])


def _s2() -> Scenario:
    return Scenario(
        "battery_failure", "Scenario 2: Battery failure",
        "Mid-mission the battery of the working drone collapses to 12%. The safety layer sends it "
        "home, the AI picks a replacement, which finishes the remaining inspection.",
        setup=lambda o, c: _create_start(o, c, "ZONE_A", 1),
        steps=[Step("inject LOW_BATTERY (12%) on the working drone", _reached("primary", 3),
                    lambda o, c: o.failures.inject("LOW_BATTERY", c["primary"], {"battery": 12}))],
        checks=[Check("mission completed with full coverage", _completed_full),
                Check("original drone returned home safely", _landed_home("primary")),
                Check("original drone is not FAILED", lambda o, c: (
                    o.engine.get(c["primary"]).status.value != "FAILED", o.engine.get(c["primary"]).status.value)),
                Check("a different drone took over", lambda o, c: (
                    bool(_events(o, EventType.MISSION_REASSIGNED)) and
                    _events(o, EventType.MISSION_REASSIGNED)[0]["drone_id"] != c["primary"],
                    str([e["message"] for e in _events(o, EventType.MISSION_REASSIGNED)]))),
                Check("failover decision is explained", lambda o, c: (
                    any(x["kind"] == "MISSION_REASSIGNMENT" and x["reasons"] for x in o.ctx.decisions.all()), ""))],
        monitors=[monitor_no_teleport])


def _obstacle_step(o: FleetOrchestrator, c: Ctx) -> None:
    r = o.failures.inject("OBSTACLE_APPEARED", c["primary"])
    c["zone"] = r["zone_id"]
    c["replanned"] = r["replanned_drones"]


def _s3() -> Scenario:
    return Scenario(
        "obstacle_appears", "Scenario 3: Obstacle appears",
        "A new obstacle blocks the drone's planned route. The area is marked unavailable, A* recomputes "
        "the remaining route and the drone flies around it.",
        setup=lambda o, c: _create_start(o, c, "ZONE_A", 1),
        steps=[Step("obstacle appears on the route ahead", _reached("primary", 2), _obstacle_step)],
        checks=[Check("route was replanned", lambda o, c: (
                    c.get("primary") in c.get("replanned", []) and bool(_events(o, EventType.ROUTE_REPLANNED)),
                    f"replanned={c.get('replanned')}")),
                Check("old and new routes differ", lambda o, c: (
                    _events(o, EventType.ROUTE_REPLANNED)[0]["data"]["old"] !=
                    _events(o, EventType.ROUTE_REPLANNED)[0]["data"]["new"], "")),
                Check("mission completed with full coverage", _completed_full)],
        monitors=[monitor_no_teleport, monitor_no_obstacle_entry])


def _release(o: FleetOrchestrator, c: Ctx) -> None:
    c["release"] = o.control.release_control(c["primary"])


def _s4() -> Scenario:
    return Scenario(
        "human_takeover", "Scenario 4: Human takeover",
        "The operator takes manual control, flies the drone briefly, then resumes the autonomous mission.",
        setup=lambda o, c: _create_start(o, c, "ZONE_A", 1),
        steps=[Step("operator takes control", _reached("primary", 3), lambda o, c: o.control.takeover(c["primary"])),
               Step("operator flies manually for 12 s", lambda o, c: True, duration=12.0,
                    tick=_manual_script((6, {"forward": 1.0}), (12, {"strafe": 1.0, "yaw": 0.3}))),
               Step("operator resumes the mission", lambda o, c: True, _release)],
        checks=[Check("takeover logged as AUTONOMOUS → TAKEOVER_REQUEST → MANUAL", lambda o, c: (
                    any("AUTONOMOUS → TAKEOVER_REQUEST" in e["message"] for e in _events(o, EventType.MODE_CHANGED))
                    and any("TAKEOVER_REQUEST → MANUAL" in e["message"] for e in _events(o, EventType.MODE_CHANGED)), "")),
                Check("drone resumed autonomous mission", lambda o, c: (
                    bool(_events(o, EventType.ROUTE_REJOIN_COMPLETED, c["primary"])), "")),
                Check("mission completed with full coverage", _completed_full)],
        monitors=[monitor_no_teleport, monitor_manual_holds_route])


def _place_wall_between(o: FleetOrchestrator, c: Ctx) -> None:
    d = o.engine.get(c["primary"])
    saved = o.control.saved[c["primary"]]
    nxt = saved.route[saved.index]
    mx, my = (d.position_x + nxt.x) / 2, (d.position_y + nxt.y) / 2
    try:
        r = o.failures.inject("OBSTACLE_APPEARED", None, {"x": mx, "y": my, "size": 50})
        c["wall"] = r["zone_id"]
    except Exception as e:                                     # pragma: no cover - placement is best effort
        c["wall_error"] = str(e)


def _s5() -> Scenario:
    return Scenario(
        "takeover_rejoin", "Scenario 5: Human takeover + route rejoin",
        "The operator flies the drone far off its route and an obstacle appears in between. On RESUME the "
        "system plans a safe path back, rejoins without skipping inspection points and finishes the mission.",
        setup=lambda o, c: _create_start(o, c, "ZONE_A", 1),
        steps=[Step("operator takes control", _reached("primary", 3), lambda o, c: o.control.takeover(c["primary"])),
               Step("operator flies far off the route", lambda o, c: True, duration=22.0,
                    tick=_manual_script((22, {"forward": 1.0}))),
               Step("obstacle appears between drone and route", lambda o, c: True, _place_wall_between),
               Step("operator selects RESUME MISSION", lambda o, c: True, _release)],
        checks=[Check("drone was far from the route", lambda o, c: (
                    (_events(o, EventType.ROUTE_REJOIN_STARTED, c["primary"]) or [{"data": {"off_route_distance": 0}}])
                    [0]["data"]["off_route_distance"] > 100,
                    str([e["data"].get("off_route_distance") for e in _events(o, EventType.ROUTE_REJOIN_STARTED)]))),
                Check("rejoin planned and completed", lambda o, c: (
                    bool(_events(o, EventType.ROUTE_REJOIN_STARTED, c["primary"])) and
                    bool(_events(o, EventType.ROUTE_REJOIN_COMPLETED, c["primary"])),
                    str(c.get("release", {}).get("rejoin", {}).get("strategy")))),
                Check("mission completed with full coverage", _completed_full)],
        monitors=[monitor_no_teleport, monitor_manual_holds_route, monitor_no_obstacle_entry])


def _s6() -> Scenario:
    return Scenario(
        "drone_failure_reassignment", "Scenario 6: Drone failure + mission reassignment",
        "One of two working drones fails outright. The remaining work is reassigned to a replacement "
        "chosen by the AI and validated by the safety layer.",
        setup=lambda o, c: _create_start(o, c, "ZONE_A", 2),
        steps=[Step("DRONE_FAILURE injected", _reached("primary", 3),
                    lambda o, c: o.failures.inject("DRONE_FAILURE", c["primary"]))],
        checks=[Check("failed drone is FAILED and on the ground", lambda o, c: (
                    o.engine.get(c["primary"]).status.value == "FAILED" and o.engine.get(c["primary"]).altitude == 0, "")),
                Check("work was reassigned", lambda o, c: (len(_events(o, EventType.MISSION_REASSIGNED)) >= 1, "")),
                Check("mission completed with full coverage", _completed_full)],
        monitors=[monitor_no_teleport])


def _s7() -> Scenario:
    def uniq(o: FleetOrchestrator, c: Ctx):
        per = o.ctx.mission_targets[c["mission"]]
        ids = [t.target_id for ts in per.values() for t in ts]
        return len(ids) == len(set(ids)) and len(per) == 4, f"{len(ids)} targets across {len(per)} drones, unique={len(set(ids))}"

    return Scenario(
        "multi_drone_coordination", "Scenario 7: Multiple drone coordination",
        "Four drones split the North Zone into disjoint work packages, fly on separate altitude layers "
        "and finish together.",
        setup=lambda o, c: _create_start(o, c, "ZONE_NORTH", 4),
        checks=[Check("targets split without overlap across 4 drones", uniq),
                Check("drones used distinct altitude layers", lambda o, c: (
                    len({round(_mission(o, c).routes[d][0].altitude, 1) for d in c["drones"]}) == 4, "")),
                Check("minimum 3D separation >= 3 m", lambda o, c: (c.get("min_sep", 0) >= 3.0,
                                                                    f"min separation {c.get('min_sep', 0):.1f} m")),
                Check("mission completed with full coverage", _completed_full)],
        monitors=[monitor_no_teleport, monitor_separation])


def _s8() -> Scenario:
    return Scenario(
        "emergency_return_home", "Scenario 8: Emergency return-to-home",
        "The datalink to the working drone is lost. After the grace period the failsafe flies it home "
        "along a planned route (no teleport) while a replacement completes the mission.",
        setup=lambda o, c: _create_start(o, c, "ZONE_A", 1),
        steps=[Step("SIGNAL_LOSS injected", _reached("primary", 3),
                    lambda o, c: o.failures.inject("SIGNAL_LOSS", c["primary"]))],
        checks=[Check("failsafe return home started", lambda o, c: (
                    bool(_events(o, EventType.RETURN_HOME_STARTED, c["primary"])), "")),
                Check("drone flew home and landed there", _landed_home("primary")),
                Check("return home completed event", lambda o, c: (
                    bool(_events(o, EventType.RETURN_HOME_COMPLETED, c["primary"])), "")),
                Check("mission completed with full coverage", _completed_full)],
        monitors=[monitor_no_teleport])


_FACTORIES: dict[str, Callable[[], Scenario]] = {
    s().name: s for s in (_s1, _s2, _s3, _s4, _s5, _s6, _s7, _s8)}
ORDER = list(_FACTORIES)


def get_scenario(name: str) -> Scenario:
    if name not in _FACTORIES:
        raise KeyError(name)
    return _FACTORIES[name]()


def list_scenarios() -> list[dict]:
    out = []
    for i, n in enumerate(ORDER, 1):
        s = _FACTORIES[n]()
        out.append({"id": n, "number": i, "title": s.title, "description": s.description})
    return out


def run_scenario(name: str, seed: int = 42, max_ticks: int = 12000, build=None) -> dict:
    """Run headless on a fresh orchestrator. `build(seed, fleet_size)` may supply one (e.g. with ML)."""
    scn = get_scenario(name)
    o = build(seed, scn.fleet_size) if build else FleetOrchestrator(seed=seed, fleet_size=scn.fleet_size)
    player = ScenarioPlayer(scn)
    player.start(o)
    o.extensions.append(player)
    ticks = o.run_until(lambda x: player.finished, max_ticks)
    if not player.finished:
        player.results = [{"name": "finished within tick budget", "ok": False, "detail": f"{ticks} ticks"}]
        player.finished = True
    key = {EventType.MISSION_CREATED, EventType.DRONE_ASSIGNED, EventType.MISSION_STARTED,
           EventType.CONTROL_TAKEOVER, EventType.CONTROL_RELEASED, EventType.ROUTE_REJOIN_STARTED,
           EventType.ROUTE_REJOIN_COMPLETED, EventType.ROUTE_REPLANNED, EventType.FAILURE_DETECTED,
           EventType.MISSION_REASSIGNED, EventType.RETURN_HOME_STARTED, EventType.RETURN_HOME_COMPLETED,
           EventType.MISSION_COMPLETED, EventType.MISSION_FAILED, EventType.INJECTED_EVENT,
           EventType.MODE_CHANGED, EventType.AI_DECISION}
    return {"name": scn.name, "title": scn.title, "seed": seed, "passed": player.passed,
            "checks": player.results, "ticks": ticks, "sim_time": round(o.now, 1),
            "mission_reports": [m.report for m in o.ctx.missions.values() if m.report],
            "timeline": [{"clock": e["clock"], "type": e["type"], "message": e["message"]}
                         for e in o.ctx.events.all() if EventType(e["type"]) in key],
            "orchestrator": o}


def load_into(o: FleetOrchestrator, name: str) -> ScenarioPlayer:
    """Attach a scenario to a (fresh) live orchestrator."""
    scn = get_scenario(name)
    player = ScenarioPlayer(scn)
    player.start(o)
    o.extensions.append(player)
    o.scenario = player
    return player
