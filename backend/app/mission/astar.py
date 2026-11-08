"""A* grid pathfinding with string-pulling smoothing.

Why A*: the environment is a small, static-between-events 2D occupancy grid with positive edge
costs and a cheap admissible heuristic, and replanning happens on demand when zones change.
A* returns an optimal path on that graph (optimal w.r.t. grid cost), is easy to explain and
test, and needs no training data. Dijkstra explores far more cells; RRT*/D* Lite add complexity
we don't need here (D* Lite would be the next step for very frequent replanning).

Details:
  * 8-connected moves, no corner cutting (a diagonal needs both orthogonal neighbours free).
  * cost = step length * (1 + cell risk), so weather zones are avoided when a cheap detour exists.
  * octile-distance heuristic (admissible and consistent because risk >= 0).
  * deterministic tie-breaking via an insertion counter.
  * `smooth_path` removes redundant waypoints using line-of-sight checks on the grid.
"""
from __future__ import annotations

import heapq
import math
from itertools import count

from app.mission.grid import Cell, OccupancyGrid, Point

SQRT2 = math.sqrt(2.0)
_NEIGHBOURS = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


def octile(a: Cell, b: Cell) -> float:
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)


def astar_cells(grid: OccupancyGrid, start: Cell, goal: Cell) -> tuple[list[Cell], float] | None:
    """Return (cell path inclusive of start and goal, cost in cell units) or None if unreachable."""
    if grid.is_blocked_cell(start) or grid.is_blocked_cell(goal):
        return None
    if start == goal:
        return [start], 0.0

    tie = count()
    open_heap: list[tuple[float, int, Cell]] = [(octile(start, goal), next(tie), start)]
    g: dict[Cell, float] = {start: 0.0}
    parent: dict[Cell, Cell] = {}
    closed: set[Cell] = set()

    while open_heap:
        _, _, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            return path, g[goal]
        closed.add(cur)
        for dx, dy in _NEIGHBOURS:
            nb = (cur[0] + dx, cur[1] + dy)
            if nb in closed or grid.is_blocked_cell(nb):
                continue
            diagonal = dx != 0 and dy != 0
            if diagonal and (grid.is_blocked_cell((cur[0] + dx, cur[1])) or
                             grid.is_blocked_cell((cur[0], cur[1] + dy))):
                continue                       # no corner cutting
            step = (SQRT2 if diagonal else 1.0) * (1.0 + grid.risk_at(nb))
            ng = g[cur] + step
            if ng < g.get(nb, math.inf):
                g[nb] = ng
                parent[nb] = cur
                heapq.heappush(open_heap, (ng + octile(nb, goal), next(tie), nb))
    return None


def smooth_path(grid: OccupancyGrid, pts: list[Point]) -> list[Point]:
    """Greedy string pulling: keep a point only if the straight line to the next one is blocked."""
    if len(pts) <= 2:
        return list(pts)
    out = [pts[0]]
    anchor = 0
    i = 2
    while i < len(pts):
        if not grid.line_free(pts[anchor], pts[i]):
            out.append(pts[i - 1])
            anchor = i - 1
        i += 1
    out.append(pts[-1])
    return out


def plan_xy(grid: OccupancyGrid, start: Point, goal: Point, smooth: bool = True) -> list[Point] | None:
    """Plan between two world points. Start/goal are snapped to the nearest free cell if blocked.

    Returned list begins at `start` and ends at `goal` exactly (the real positions), with the
    intermediate points on cell centres. Returns None when no route exists.
    """
    sc = grid.nearest_free_cell(grid.cell_of(start))
    gc = grid.nearest_free_cell(grid.cell_of(goal))
    if sc is None or gc is None:
        return None
    res = astar_cells(grid, sc, gc)
    if res is None:
        return None
    cells, _ = res
    pts: list[Point] = [start] + [grid.center_of(c) for c in cells[1:-1]] + [goal]
    if len(cells) == 1:
        pts = [start, goal]
    if smooth:
        pts = smooth_path(grid, pts)
    # drop consecutive duplicates
    dedup = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - dedup[-1][0], p[1] - dedup[-1][1]) > 1e-6:
            dedup.append(p)
    return dedup
