"""Route planner: turns inspection targets into an ordered, obstacle-free waypoint route,
detects when a route has become invalid, and replans the remaining part.

Responsibilities are deliberately separated from the simulator: the planner only works on
the OccupancyGrid and returns Waypoints; it never touches drone state.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from app.core.enums import WaypointAction
from app.core.types import Waypoint
from app.mission.astar import plan_xy
from app.mission.coverage import InspectionTarget
from app.mission.grid import OccupancyGrid, Point
from app.utils.geometry import path_length

HOME_ID = "HOME"


@dataclass
class PlanResult:
    route: list[Waypoint]
    skipped: list[str] = field(default_factory=list)     # ids of targets that could not be planned
    length: float = 0.0
    ok: bool = True
    reason: str = ""

    def to_dict(self) -> dict:
        return {"route": [w.to_dict() for w in self.route], "skipped": self.skipped,
                "length": round(self.length, 1), "ok": self.ok, "reason": self.reason}


class RoutePlanner:
    def __init__(self, grid: OccupancyGrid, altitude: float = 30.0) -> None:
        self.grid = grid
        self.altitude = altitude

    # ------------------------------------------------------------------ building
    def _leg(self, a: Point, b: Point, prefix: str, counter: list[int]) -> list[Point] | None:
        """Intermediate transit points strictly between a and b (empty if direct)."""
        pts = plan_xy(self.grid, a, b)
        if pts is None:
            return None
        mids = pts[1:-1]
        return mids

    def build_route(self, start: Point, targets: list[InspectionTarget], home: Point,
                    prefix: str = "R0-", altitude: float | None = None) -> PlanResult:
        """start -> (A* legs) -> each target (INSPECT) -> home (LAND). Unreachable targets are skipped."""
        alt = self.altitude if altitude is None else altitude
        route: list[Waypoint] = []
        skipped: list[str] = []
        cur = start
        n = [0]

        def emit_transit(points: list[Point]) -> None:
            for p in points:
                n[0] += 1
                route.append(Waypoint(f"{prefix}T{n[0]:03d}", round(p[0], 2), round(p[1], 2), alt,
                                      WaypointAction.TRANSIT))

        for t in targets:
            if self.grid.is_blocked_xy(t.xy):
                skipped.append(t.target_id)
                continue
            mids = self._leg(cur, t.xy, prefix, n)
            if mids is None:
                skipped.append(t.target_id)
                continue
            emit_transit(mids)
            route.append(Waypoint(t.target_id, t.x, t.y, alt, WaypointAction.INSPECT))
            cur = t.xy

        mids = self._leg(cur, home, prefix, n)
        if mids is None:
            return PlanResult([], skipped, 0.0, ok=False, reason="no route home")
        emit_transit(mids)
        route.append(Waypoint(HOME_ID, home[0], home[1], alt, WaypointAction.LAND))
        length = path_length([start] + [w.xy for w in route])
        return PlanResult(route, skipped, length)

    def plan_to(self, start: Point, goal: Point, prefix: str, action: WaypointAction,
                goal_id: str, altitude: float | None = None) -> PlanResult:
        """Single destination route (return-home, rejoin leg)."""
        alt = self.altitude if altitude is None else altitude
        pts = plan_xy(self.grid, start, goal)
        if pts is None:
            return PlanResult([], [], 0.0, ok=False, reason="no path")
        route = [Waypoint(f"{prefix}T{i + 1:03d}", round(p[0], 2), round(p[1], 2), alt,
                          WaypointAction.TRANSIT) for i, p in enumerate(pts[1:-1])]
        route.append(Waypoint(goal_id, goal[0], goal[1], alt, action))
        return PlanResult(route, [], path_length([start] + [w.xy for w in route]))

    def return_home(self, start: Point, home: Point, prefix: str = "RTH-",
                    altitude: float | None = None) -> PlanResult:
        return self.plan_to(start, home, prefix, WaypointAction.LAND, HOME_ID, altitude)

    # ------------------------------------------------------------------ validation / replanning
    def first_blocked_segment(self, position: Point, remaining: list[Waypoint]) -> int | None:
        """Index (into `remaining`) of the first waypoint whose approach segment is blocked,
        or whose position itself is blocked. None if the whole remaining route is clear."""
        cur = position
        for i, w in enumerate(remaining):
            if self.grid.is_blocked_xy(w.xy) and w.action != WaypointAction.LAND:
                return i
            if not self.grid.line_free(cur, w.xy):
                return i
            cur = w.xy
        return None

    def route_is_clear(self, position: Point, remaining: list[Waypoint]) -> bool:
        return self.first_blocked_segment(position, remaining) is None

    def replan_remaining(self, position: Point, remaining: list[Waypoint], home: Point,
                         prefix: str) -> PlanResult:
        """Keep the *intent* (inspect/hover/land waypoints, in order), drop the old transit
        points, drop key waypoints that are now unavailable, and re-run A* between survivors.
        """
        key = [w for w in remaining if w.action != WaypointAction.TRANSIT]
        route: list[Waypoint] = []
        skipped: list[str] = []
        cur = position
        n = 0
        has_land = any(w.action == WaypointAction.LAND for w in key)
        for w in key:
            if w.action != WaypointAction.LAND and self.grid.is_blocked_xy(w.xy):
                skipped.append(w.id)
                continue
            pts = plan_xy(self.grid, cur, w.xy)
            if pts is None:
                if w.action == WaypointAction.LAND:
                    return PlanResult([], skipped, 0.0, ok=False, reason="no route home")
                skipped.append(w.id)
                continue
            for p in pts[1:-1]:
                n += 1
                route.append(Waypoint(f"{prefix}T{n:03d}", round(p[0], 2), round(p[1], 2),
                                      w.altitude, WaypointAction.TRANSIT))
            route.append(w)
            cur = w.xy
        if not has_land:
            pts = plan_xy(self.grid, cur, home)
            if pts is None:
                return PlanResult([], skipped, 0.0, ok=False, reason="no route home")
            for p in pts[1:-1]:
                n += 1
                route.append(Waypoint(f"{prefix}T{n:03d}", round(p[0], 2), round(p[1], 2),
                                      self.altitude, WaypointAction.TRANSIT))
            route.append(Waypoint(HOME_ID, home[0], home[1], self.altitude, WaypointAction.LAND))
        return PlanResult(route, skipped, path_length([position] + [w.xy for w in route]))
