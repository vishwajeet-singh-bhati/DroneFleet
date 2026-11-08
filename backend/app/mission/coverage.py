"""Inspection coverage: turn a target zone into inspection points, prioritise, split, order.

Algorithms (all deterministic, no ML):
  * lawnmower (boustrophedon) lattice over the zone, skipping blocked cells
  * CoverageMap tracks which lattice points have been inspected, so a mission can
    "prioritise areas that have not been inspected"
  * contiguous split of the snake order into k balanced chunks (one per drone)
  * nearest-neighbour + 2-opt ordering for open paths with fixed start/end
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from app.mission.grid import OccupancyGrid, Point, Zone
from app.utils.geometry import distance


@dataclass(frozen=True)
class InspectionTarget:
    target_id: str
    x: float
    y: float

    @property
    def xy(self) -> Point:
        return (self.x, self.y)


class CoverageMap:
    """Persistent record of which target ids have been inspected."""

    def __init__(self) -> None:
        self._inspected: set[str] = set()

    def mark(self, target_id: str) -> None:
        self._inspected.add(target_id)

    def is_inspected(self, target_id: str) -> bool:
        return target_id in self._inspected

    def mark_many(self, ids) -> None:
        self._inspected.update(ids)

    def ratio(self, targets: list[InspectionTarget]) -> float:
        if not targets:
            return 1.0
        return sum(1 for t in targets if t.target_id in self._inspected) / len(targets)

    def snapshot(self) -> list[str]:
        return sorted(self._inspected)


def lawnmower_targets(zone: Zone, grid: OccupancyGrid, spacing: float = 60.0) -> list[InspectionTarget]:
    """Snake-order lattice of inspection points covering `zone`; blocked points are skipped.

    Ids are stable (`<zone>:<col>-<row>`), independent of what is blocked, so coverage state
    survives replanning.
    """
    cols = max(1, int(math.floor((zone.x1 - zone.x0) / spacing)))
    rows = max(1, int(math.floor((zone.y1 - zone.y0) / spacing)))
    dx = (zone.x1 - zone.x0) / cols
    dy = (zone.y1 - zone.y0) / rows
    out: list[InspectionTarget] = []
    for c in range(cols):
        row_iter = range(rows) if c % 2 == 0 else range(rows - 1, -1, -1)
        for r in row_iter:
            x = zone.x0 + (c + 0.5) * dx
            y = zone.y0 + (r + 0.5) * dy
            if grid.is_blocked_xy((x, y)):
                continue
            out.append(InspectionTarget(f"{zone.zone_id}:{c}-{r}", round(x, 2), round(y, 2)))
    return out


def prioritise(targets: list[InspectionTarget], coverage: CoverageMap,
               include_inspected: bool = False) -> list[InspectionTarget]:
    """Uninspected targets first (stable order). Inspected ones are dropped unless requested."""
    fresh = [t for t in targets if not coverage.is_inspected(t.target_id)]
    if not include_inspected:
        return fresh
    done = [t for t in targets if coverage.is_inspected(t.target_id)]
    return fresh + done


def split_among(targets: list[InspectionTarget], k: int) -> list[list[InspectionTarget]]:
    """Split the snake-ordered list into k contiguous, size-balanced chunks."""
    if k <= 0:
        raise ValueError("k must be >= 1")
    n = len(targets)
    base, extra = divmod(n, k)
    out, idx = [], 0
    for i in range(k):
        size = base + (1 if i < extra else 0)
        out.append(targets[idx: idx + size])
        idx += size
    return out


def _path_len(start: Point, pts: list[Point], end: Point | None) -> float:
    total, cur = 0.0, start
    for p in pts:
        total += distance(cur, p)
        cur = p
    if end is not None:
        total += distance(cur, end)
    return total


def nearest_neighbour_order(start: Point, targets: list[InspectionTarget]) -> list[InspectionTarget]:
    remaining = list(targets)
    out, cur = [], start
    while remaining:
        nxt = min(remaining, key=lambda t: (distance(cur, t.xy), t.target_id))
        remaining.remove(nxt)
        out.append(nxt)
        cur = nxt.xy
    return out


def two_opt(start: Point, order: list[InspectionTarget], end: Point | None,
            max_passes: int = 20) -> list[InspectionTarget]:
    """2-opt improvement of an open path with fixed start (and optional fixed end)."""
    best = list(order)
    n = len(best)
    if n < 3:
        return best
    improved, passes = True, 0
    while improved and passes < max_passes:
        improved, passes = False, passes + 1
        for i in range(n - 1):
            for j in range(i + 1, n):
                cand = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                if _path_len(start, [t.xy for t in cand], end) + 1e-9 < \
                        _path_len(start, [t.xy for t in best], end):
                    best, improved = cand, True
    return best


def order_targets(start: Point, targets: list[InspectionTarget], end: Point | None = None
                  ) -> list[InspectionTarget]:
    """Nearest-neighbour seed + 2-opt refinement; never worse than the seed."""
    return two_opt(start, nearest_neighbour_order(start, targets), end)
