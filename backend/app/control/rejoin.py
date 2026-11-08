"""Mission rejoin planning: bring a drone that left its route (manual flight, pause drift,
replan) back onto the mission route safely, without teleporting and without skipping work.

Inputs : current position, the mission route, index of the next waypoint still to do.
Output : a RejoinPlan - the path to fly now, and the index at which to resume the mission route.

Algorithm
---------
1. Candidate rejoin points. A drone may re-enter the route at
     (a) the next waypoint itself (always legal), or
     (b) the closest point on the *current leg* (prev -> next), skipping nothing, or
     (c) the closest point on later legs, but ONLY while every waypoint it would skip is a
         plain TRANSIT point. INSPECT / HOVER / LAND waypoints are never skipped.
2. A candidate is feasible if the rejoin point is free, the drone can reach it with an A* path,
   and the remainder of the route from that point on is still obstacle-free.
3. Cost of a candidate = A* path length to the point + length of the route still to fly from it.
   Minimising this trades a short way back against not abandoning useful route (skipping transit
   points makes the remaining route shorter, which the cost rewards).
4. If no candidate is feasible (e.g. an obstacle now blocks the old route) the remaining route
   is replanned from the current position (strategy REPLAN).
The function is pure (grid + planner in, plan out) so it can be unit-tested exhaustively.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from app.core.enums import WaypointAction
from app.core.types import Waypoint
from app.mission.astar import plan_xy
from app.mission.grid import Point
from app.mission.route_planner import RoutePlanner
from app.utils.geometry import distance, path_length, point_segment_projection

REJOIN_ID = "REJOIN"


@dataclass
class RejoinPlan:
    strategy: str                     # NEXT_WAYPOINT | SEGMENT_PROJECTION | REPLAN
    rejoin_point: Point
    entry_index: int                  # index in `mission_route` to resume at after rejoin
    path: list[Waypoint]              # waypoints to fly now; last one is at rejoin_point
    off_route_distance: float         # straight-line distance from route at plan time
    path_length: float
    remaining_length: float           # route length still to fly after the rejoin point
    mission_route: list[Waypoint]     # route to install on completion (== input unless REPLAN)
    skipped_ids: list[str] = field(default_factory=list)       # transit waypoints bypassed
    dropped_ids: list[str] = field(default_factory=list)       # key waypoints dropped by REPLAN
    candidates: list[dict] = field(default_factory=list)       # for explainability / tests

    def to_dict(self) -> dict:
        return {
            "strategy": self.strategy, "rejoin_point": [round(self.rejoin_point[0], 1),
                                                         round(self.rejoin_point[1], 1)],
            "entry_index": self.entry_index, "path": [w.to_dict() for w in self.path],
            "off_route_distance": round(self.off_route_distance, 1),
            "path_length": round(self.path_length, 1),
            "remaining_length": round(self.remaining_length, 1),
            "skipped_ids": self.skipped_ids, "dropped_ids": self.dropped_ids,
            "candidates": self.candidates,
        }


def distance_to_route(position: Point, route: list[Waypoint], next_index: int) -> float:
    """Straight-line distance from `position` to the nearest point of the unfinished route."""
    pts: list[Point] = [w.xy for w in route[max(0, next_index - 1):]]
    if not pts:
        return 0.0
    if len(pts) == 1:
        return distance(position, pts[0])
    return min(distance(position, point_segment_projection(position, a, b)[0])
               for a, b in zip(pts, pts[1:]))


def _suffix_clear(planner: RoutePlanner, route: list[Waypoint]) -> list[bool]:
    """clear[i] is True if every leg route[i] -> route[i+1] -> ... -> route[-1] is obstacle free."""
    n = len(route)
    clear = [True] * n
    ok = True
    for i in range(n - 2, -1, -1):
        ok = ok and planner.grid.line_free(route[i].xy, route[i + 1].xy)
        clear[i] = ok
    return clear


def _length_from(point: Point, route: list[Waypoint], entry_index: int) -> float:
    return path_length([point] + [w.xy for w in route[entry_index:]])


def plan_rejoin(position: Point, route: list[Waypoint], next_index: int, planner: RoutePlanner,
                home: Point, prefix: str = "RJ-") -> RejoinPlan | None:
    """Return a RejoinPlan, or None if the mission route has nothing left to fly."""
    if next_index >= len(route):
        return None
    k = next_index
    grid = planner.grid
    off_route = distance_to_route(position, route, k)
    suffix_clear = _suffix_clear(planner, route)

    # ---- 1. candidate legs: (leg_start_point, index of the waypoint the leg ends at)
    legs: list[tuple[Point, int]] = []
    if k > 0:
        legs.append((route[k - 1].xy, k))
    j = k
    while j < len(route) - 1 and route[j].action == WaypointAction.TRANSIT:
        legs.append((route[j].xy, j + 1))
        j += 1

    candidates: list[dict] = []
    best: tuple[float, int, str, Point, int, list[Point]] | None = None

    def consider(label: str, q: Point, entry_index: int) -> None:
        nonlocal best
        # the old route from q onward must be usable
        if not suffix_clear[entry_index] or grid.is_blocked_xy(q):
            candidates.append({"kind": label, "entry_index": entry_index, "feasible": False,
                               "reason": "route ahead or rejoin point blocked"})
            return
        if not grid.line_free(q, route[entry_index].xy):
            candidates.append({"kind": label, "entry_index": entry_index, "feasible": False,
                               "reason": "leg to next waypoint blocked"})
            return
        pts = plan_xy(grid, position, q)
        if pts is None:
            candidates.append({"kind": label, "entry_index": entry_index, "feasible": False,
                               "reason": "no path to rejoin point"})
            return
        to_q = path_length(pts)
        rest = _length_from(q, route, entry_index)
        cost = to_q + rest
        candidates.append({"kind": label, "entry_index": entry_index, "feasible": True,
                           "point": [round(q[0], 1), round(q[1], 1)], "path_length": round(to_q, 1),
                           "remaining": round(rest, 1), "cost": round(cost, 1)})
        key = (cost, entry_index)
        if best is None or key < (best[0], best[1]):
            best = (cost, entry_index, label, q, entry_index, pts)

    consider("NEXT_WAYPOINT", route[k].xy, k)
    for a, bidx in legs:
        q, _ = point_segment_projection(position, a, route[bidx].xy)
        consider("SEGMENT_PROJECTION", q, bidx)

    if best is not None:
        cost, _, label, q, entry, pts = best
        path_wps = [Waypoint(f"{prefix}T{i + 1:03d}", round(p[0], 2), round(p[1], 2),
                             route[entry].altitude, WaypointAction.TRANSIT)
                    for i, p in enumerate(pts[1:-1])]
        path_wps.append(Waypoint(REJOIN_ID, round(q[0], 2), round(q[1], 2), route[entry].altitude,
                                 WaypointAction.TRANSIT))
        skipped = [w.id for w in route[k:entry] if w.action == WaypointAction.TRANSIT]
        return RejoinPlan(label, q, entry, path_wps, off_route, path_length([position] + [w.xy for w in path_wps]),
                          _length_from(q, route, entry), list(route), skipped, [], candidates)

    # ---- 4. fall back: replan whatever is left from where the drone is now
    res = planner.replan_remaining(position, route[k:], home, prefix)
    if not res.ok or not res.route:
        return None
    new_full = list(route[:k]) + res.route
    first = res.route[0]
    path_wps = [Waypoint(REJOIN_ID, first.x, first.y, first.altitude, WaypointAction.TRANSIT)]
    return RejoinPlan("REPLAN", first.xy, k, path_wps, off_route,
                      distance(position, first.xy), path_length([w.xy for w in res.route]),
                      new_full, [], list(res.skipped), candidates)
