"""FleetOrchestrator: owns the simulation and wires every engine together.

Tick pipeline (one fixed timestep):
  1. simulation engine advances all drones            (physics, no decisions)
  2. engine events are routed to services             (coverage, mode changes, landing, failures)
  3. fleet housekeeping                               (charging)
  4. safety monitor evaluates every drone             (deterministic failsafes)
  5. AI anomaly detection / maintenance (if enabled)  (events only; safety keeps authority)
  6. mission watchdogs and persistence
The frontend only ever sees snapshots produced here.
"""
from __future__ import annotations

from typing import Callable

from app.core.config import DEFAULT_SETTINGS, Settings
from app.core.enums import EventType, WaypointAction, ZoneKind
from app.core.event_log import DecisionLog, EventLog
from app.database.repository import Repository
from app.fleet.manager import FleetManager
from app.mission.coverage import CoverageMap
from app.mission.grid import OccupancyGrid, Zone
from app.mission.route_planner import RoutePlanner
from app.safety.monitor import SafetyMonitor
from app.services.context import Context
from app.services.control_service import ControlService
from app.services.failure_service import FailureService
from app.services.mission_service import MissionService
from app.services.recovery_service import RecoveryService
from app.simulation.engine import SimulationEngine
from app.simulation.events import SimEvent

TELEMETRY_EVERY_TICKS = 10
FLUSH_EVERY_TICKS = 20


def default_zones() -> list[Zone]:
    K = ZoneKind
    return [
        Zone("ZONE_A", K.INSPECTION, 280, 260, 520, 460, "Zone A"),
        Zone("ZONE_B", K.INSPECTION, 600, 180, 880, 400, "Zone B"),
        Zone("ZONE_NORTH", K.INSPECTION, 240, 720, 800, 900, "North Zone"),
        Zone("ZONE_WEST", K.INSPECTION, 40, 300, 200, 640, "West Zone"),
        Zone("OBS_1", K.OBSTACLE, 390, 520, 450, 660, "Radio mast"),
        Zone("OBS_2", K.OBSTACLE, 540, 470, 600, 560, "Water tower"),
        Zone("NOFLY_1", K.RESTRICTED, 660, 520, 820, 650, "Restricted airspace"),
        Zone("NOFLY_2", K.RESTRICTED, 190, 130, 270, 220, "Hospital helipad"),
    ]


class FleetOrchestrator:
    def __init__(self, settings: Settings = DEFAULT_SETTINGS, seed: int | None = None,
                 repo: Repository | None = None, fleet_size: int | None = None,
                 zones: list[Zone] | None = None) -> None:
        self.settings = settings
        engine = SimulationEngine(settings.world, settings.sim, seed=seed)
        engine.create_fleet(fleet_size or settings.fleet_size)
        grid = OccupancyGrid(settings.world)
        for z in (zones if zones is not None else default_zones()):
            grid.add_zone(z)
        engine.collision_fn = lambda x, y: grid.is_blocked_xy((x, y))
        self.ctx = Context(settings=settings, engine=engine, grid=grid,
                           planner=RoutePlanner(grid, settings.sim.default_altitude),
                           events=EventLog(), decisions=DecisionLog(), coverage=CoverageMap(), repo=repo)
        self.fleet = FleetManager(self.ctx)
        self.missions = MissionService(self.ctx, self.fleet)
        self.control = ControlService(self.ctx)
        self.control.mission_service = self.missions
        self.missions.control = self.control
        self.recovery = RecoveryService(self.ctx, self.control, self.missions)
        self.control.recovery_hook = lambda d, reason, action: self.recovery.handle_drone_loss(d, reason, action)
        self.failures = FailureService(self.ctx, self.missions, self.recovery)
        self.monitor = SafetyMonitor(settings.sim)
        self.extensions: list[Callable[["FleetOrchestrator"], None]] = []   # e.g. anomaly detection
        self.scenario = None          # ScenarioPlayer when a scripted scenario is loaded
        self._world_version_sent = -1
        if repo:
            for d in engine.drones_iter():
                repo.upsert_drone(d.to_dict())
        self.ctx.log(EventType.INFO, f"Fleet online: {len(engine.drones)} drones, seed {engine.seed}")

    # ------------------------------------------------------------------ accessors
    @property
    def engine(self) -> SimulationEngine:
        return self.ctx.engine

    @property
    def now(self) -> float:
        return self.ctx.engine.time

    # ------------------------------------------------------------------ main loop
    def tick(self, n: int = 1) -> None:
        for _ in range(n):
            self._tick_once()

    def _tick_once(self) -> None:
        ctx = self.ctx
        events = ctx.engine.step(1)
        for ev in events:
            self._handle_sim_event(ev)
        self.fleet.tick_housekeeping(ctx.settings.sim.dt)
        for d in list(ctx.engine.drones_iter()):
            findings = self.monitor.evaluate(d, ctx.now)
            if findings:
                self.recovery.handle_findings(d, findings)
        for ext in self.extensions:
            ext(self)
        self.missions.tick()
        if ctx.repo:
            if ctx.engine.tick_count % TELEMETRY_EVERY_TICKS == 0:
                ctx.repo.save_telemetry([d.to_dict() for d in ctx.engine.drones_iter()], ctx.now)
            if ctx.engine.tick_count % FLUSH_EVERY_TICKS == 0:
                ctx.repo.flush()

    def run_until(self, predicate: Callable[["FleetOrchestrator"], bool], max_ticks: int = 20000) -> int:
        """Advance until predicate is true (or max_ticks). Returns ticks executed."""
        for i in range(max_ticks):
            if predicate(self):
                return i
            self._tick_once()
        return max_ticks

    # ------------------------------------------------------------------ event routing
    def _handle_sim_event(self, ev: SimEvent) -> None:
        ctx = self.ctx
        d = ctx.engine.drones.get(ev.drone_id) if ev.drone_id else None
        t = ev.type
        if t == EventType.WAYPOINT_REACHED:
            label = ev.data.get("route_label")
            if label == "MISSION" and ev.data.get("action") == WaypointAction.INSPECT.value:
                ctx.coverage.mark(ev.data["waypoint_id"])
            if ev.data.get("action") != WaypointAction.TRANSIT.value or label == "REJOIN":
                ctx.log(t, ev.message, drone_id=ev.drone_id, mission_id=ev.mission_id, data=ev.data)
        elif t == EventType.ROUTE_COMPLETED:
            ctx.log(t, ev.message, drone_id=ev.drone_id, mission_id=ev.mission_id, data=ev.data)
            if d:
                self.control.on_route_completed(d, ev.data["route_label"])
        elif t == EventType.TAKEOFF_COMPLETED:
            ctx.log(EventType.MODE_CHANGED, f"{ev.drone_id} TAKEOFF → AUTONOMOUS (Reason: cruise altitude reached)",
                    drone_id=ev.drone_id, mission_id=ev.mission_id,
                    data={"from": "TAKEOFF", "to": "AUTONOMOUS", "reason": "cruise altitude reached"})
        elif t == EventType.LANDED:
            if d:
                self.control.on_landed(d, bool(ev.data.get("emergency")))
                self.recovery.clear(d.drone_id)
        elif t == EventType.FAILURE_DETECTED:
            ctx.log(t, ev.message, drone_id=ev.drone_id, mission_id=ev.mission_id,
                    severity=ev.severity, data=ev.data)
            if d and ev.data.get("reason") == "BATTERY_DEPLETED":
                self.control.on_manual_drone_failed(d)
                self.recovery.handle_drone_loss(d, "battery depleted", "EMERGENCY_LAND")
        else:
            ctx.log(t, ev.message, drone_id=ev.drone_id, mission_id=ev.mission_id,
                    severity=ev.severity, data=ev.data)

    # ------------------------------------------------------------------ views
    def world(self) -> dict:
        w = self.ctx.grid.to_dict()
        w["home"] = {"x": self.settings.world.home[0], "y": self.settings.world.home[1]}
        return w

    @staticmethod
    def _drone_view(d) -> dict:
        """Telemetry plus, for non-mission routes (rejoin / return-home), the remaining path so the
        UI can draw it. Mission routes come from the mission object instead."""
        out = d.to_dict()
        if d.route and d.route_label not in ("MISSION", "NONE"):
            out["active_route"] = [[w.x, w.y] for w in d.route[d.current_waypoint_index:]]
        return out

    def snapshot(self, mission_detail: bool = False) -> dict:
        ctx = self.ctx
        mission_list = []
        for m in ctx.missions.values():
            md = m.to_dict()
            md["progress"] = round(self.fleet.mission_progress(m), 3)
            if not mission_detail:
                md.pop("history", None)
                md["route_history"] = md["route_history"][-3:]
            mission_list.append(md)
        return {
            "sim_time": round(ctx.now, 3), "tick": ctx.engine.tick_count, "seed": ctx.engine.seed,
            "drones": [self._drone_view(d) for d in ctx.engine.drones_iter()],
            "missions": mission_list, "summary": self.fleet.summary(),
            "world_version": ctx.grid.version,
            "control": {did: {"operator": s.operator, "since": round(s.started_at, 1),
                              "accepted": s.accepted, "rejected": s.rejected}
                        for did, s in self.control.sessions.items()},
            "coverage": self.ctx.coverage.snapshot(),
            "maintenance": self.ctx.maintenance_view,
            "scenario": self.scenario.status() if self.scenario else None,
        }
