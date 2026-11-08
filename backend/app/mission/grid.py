"""Planning environment: zones (obstacles, no-fly, weather, inspection) rasterised to a grid.

The grid is the *planner's* model of the world. Zones can be activated/deactivated at
runtime (e.g. OBSTACLE_APPEARED); every change bumps `version` so callers can detect that
existing routes may be stale.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from app.core.config import WorldConfig
from app.core.enums import ZoneKind

Cell = tuple[int, int]
Point = tuple[float, float]

WEATHER_RISK_COST = 2.0   # extra cost multiplier for cells inside an active WEATHER zone


@dataclass
class Zone:
    zone_id: str
    kind: ZoneKind
    x0: float
    y0: float
    x1: float
    y1: float
    name: str = ""
    active: bool = True

    def __post_init__(self) -> None:
        if self.x1 < self.x0:
            self.x0, self.x1 = self.x1, self.x0
        if self.y1 < self.y0:
            self.y0, self.y1 = self.y1, self.y0

    @property
    def center(self) -> Point:
        return ((self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2)

    def contains(self, p: Point) -> bool:
        return self.x0 <= p[0] <= self.x1 and self.y0 <= p[1] <= self.y1

    def to_dict(self) -> dict:
        return {"zone_id": self.zone_id, "kind": self.kind.value, "name": self.name or self.zone_id,
                "x0": self.x0, "y0": self.y0, "x1": self.x1, "y1": self.y1, "active": self.active}


class OccupancyGrid:
    def __init__(self, world: WorldConfig | None = None, margin: float = 10.0) -> None:
        self.world = world or WorldConfig()
        self.cell = self.world.cell_size
        self.cols = int(math.ceil(self.world.width / self.cell))
        self.rows = int(math.ceil(self.world.height / self.cell))
        self.margin = margin            # safety buffer around blocking zones (m)
        self.zones: dict[str, Zone] = {}
        self.version = 0
        self._blocked = [[False] * self.rows for _ in range(self.cols)]
        self._risk = [[0.0] * self.rows for _ in range(self.cols)]

    # ------------------------------------------------------------- zone management
    def add_zone(self, zone: Zone) -> None:
        self.zones[zone.zone_id] = zone
        self._rasterise()

    def set_zone_active(self, zone_id: str, active: bool) -> None:
        z = self.zones[zone_id]
        if z.active != active:
            z.active = active
            self._rasterise()

    def remove_zone(self, zone_id: str) -> None:
        if self.zones.pop(zone_id, None) is not None:
            self._rasterise()

    def zones_of(self, kind: ZoneKind) -> list[Zone]:
        return [z for z in self.zones.values() if z.kind == kind]

    def _rasterise(self) -> None:
        self._blocked = [[False] * self.rows for _ in range(self.cols)]
        self._risk = [[0.0] * self.rows for _ in range(self.cols)]
        m = self.margin
        for z in self.zones.values():
            if not z.active or z.kind == ZoneKind.INSPECTION:
                continue
            pad = m if z.kind in (ZoneKind.RESTRICTED, ZoneKind.OBSTACLE) else 0.0
            i0 = max(0, int(math.floor((z.x0 - pad) / self.cell)))
            i1 = min(self.cols - 1, int(math.floor((z.x1 + pad - 1e-9) / self.cell)))
            j0 = max(0, int(math.floor((z.y0 - pad) / self.cell)))
            j1 = min(self.rows - 1, int(math.floor((z.y1 + pad - 1e-9) / self.cell)))
            for i in range(i0, i1 + 1):
                for j in range(j0, j1 + 1):
                    if z.kind == ZoneKind.WEATHER:
                        self._risk[i][j] = max(self._risk[i][j], WEATHER_RISK_COST)
                    else:
                        self._blocked[i][j] = True
        self.version += 1

    # ------------------------------------------------------------- queries
    def in_bounds(self, c: Cell) -> bool:
        return 0 <= c[0] < self.cols and 0 <= c[1] < self.rows

    def is_blocked_cell(self, c: Cell) -> bool:
        return (not self.in_bounds(c)) or self._blocked[c[0]][c[1]]

    def risk_at(self, c: Cell) -> float:
        return self._risk[c[0]][c[1]] if self.in_bounds(c) else 0.0

    def cell_of(self, p: Point) -> Cell:
        i = min(self.cols - 1, max(0, int(p[0] // self.cell)))
        j = min(self.rows - 1, max(0, int(p[1] // self.cell)))
        return (i, j)

    def center_of(self, c: Cell) -> Point:
        return ((c[0] + 0.5) * self.cell, (c[1] + 0.5) * self.cell)

    def is_blocked_xy(self, p: Point) -> bool:
        if not (0 <= p[0] <= self.world.width and 0 <= p[1] <= self.world.height):
            return True
        return self.is_blocked_cell(self.cell_of(p))

    def line_free(self, a: Point, b: Point) -> bool:
        """True if the segment a-b touches no blocked cell.

        Exact (Amanatides-Woo) grid traversal: every cell the segment passes through is checked,
        including both neighbours when it passes exactly through a cell corner, so it can never
        "tunnel" through a corner the way fixed-step sampling can.
        """
        w, h = self.world.width, self.world.height
        if not (0 <= a[0] <= w and 0 <= a[1] <= h and 0 <= b[0] <= w and 0 <= b[1] <= h):
            return False
        cs = self.cell
        i, j = self.cell_of(a)
        ie, je = self.cell_of(b)
        if self.is_blocked_cell((i, j)):
            return False
        dx, dy = b[0] - a[0], b[1] - a[1]
        inf = float("inf")
        si = 1 if dx > 0 else -1 if dx < 0 else 0
        sj = 1 if dy > 0 else -1 if dy < 0 else 0
        tmx = ((i + (1 if si > 0 else 0)) * cs - a[0]) / dx if dx else inf
        tmy = ((j + (1 if sj > 0 else 0)) * cs - a[1]) / dy if dy else inf
        tdx = cs / abs(dx) if dx else inf
        tdy = cs / abs(dy) if dy else inf
        eps = 1e-12
        for _ in range(self.cols + self.rows + 4):
            t_next = min(tmx, tmy)
            if t_next >= 1.0 - eps:                    # segment ends inside / on the edge of this cell
                if abs(tmx - tmy) < eps and (self.is_blocked_cell((i + si, j))
                                             or self.is_blocked_cell((i, j + sj))):
                    return False                       # ends exactly on a corner touching a blocked cell
                return True
            if abs(tmx - tmy) < eps:                   # exactly through a corner: both side cells count
                if self.is_blocked_cell((i + si, j)) or self.is_blocked_cell((i, j + sj)):
                    return False
                i, j, tmx, tmy = i + si, j + sj, tmx + tdx, tmy + tdy
            elif tmx < tmy:
                i, tmx = i + si, tmx + tdx
            else:
                j, tmy = j + sj, tmy + tdy
            if self.is_blocked_cell((i, j)):
                return False
        return True

    def nearest_free_cell(self, c: Cell) -> Cell | None:
        """BFS outward to the closest unblocked cell (used to snap start/goal off obstacles)."""
        c = (min(self.cols - 1, max(0, c[0])), min(self.rows - 1, max(0, c[1])))
        if not self.is_blocked_cell(c):
            return c
        seen = {c}
        q = deque([c])
        while q:
            cur = q.popleft()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nb = (cur[0] + dx, cur[1] + dy)
                    if nb in seen or not self.in_bounds(nb):
                        continue
                    seen.add(nb)
                    if not self.is_blocked_cell(nb):
                        return nb
                    q.append(nb)
        return None

    def blocked_cells(self) -> list[Cell]:
        return [(i, j) for i in range(self.cols) for j in range(self.rows) if self._blocked[i][j]]

    def to_dict(self) -> dict:
        return {"version": self.version, "cell_size": self.cell, "cols": self.cols, "rows": self.rows,
                "width": self.world.width, "height": self.world.height,
                "zones": [z.to_dict() for z in self.zones.values()]}
