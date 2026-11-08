"""Pure geometry helpers. No state, no I/O, fully unit-testable.

Conventions: x = east, y = north, heading in degrees clockwise from north
(0 = +y, 90 = +x), like a compass bearing.
"""
from __future__ import annotations

import math

Point = tuple[float, float]


def distance(a: Point, b: Point) -> float:
    """Euclidean distance between two 2D points."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def distance_3d(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def heading_to(a: Point, b: Point) -> float:
    """Compass bearing (0-360, clockwise from north) from a to b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    if dx == 0 and dy == 0:
        return 0.0
    return math.degrees(math.atan2(dx, dy)) % 360.0


def normalize_angle(deg: float) -> float:
    return deg % 360.0


def angle_diff(target: float, current: float) -> float:
    """Smallest signed difference target-current in (-180, 180]."""
    d = (target - current + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def heading_vector(heading_deg: float) -> Point:
    """Unit vector (dx, dy) for a compass heading."""
    r = math.radians(heading_deg)
    return (math.sin(r), math.cos(r))


def move_toward(current: Point, target: Point, speed: float, dt: float) -> tuple[Point, float, bool]:
    """Advance from current toward target by speed*dt metres without overshooting.

    Returns (new_position, distance_travelled, reached_exactly).
    """
    d = distance(current, target)
    step = speed * dt
    if d <= step or d == 0:
        return target, d, True
    ratio = step / d
    new = (current[0] + (target[0] - current[0]) * ratio,
           current[1] + (target[1] - current[1]) * ratio)
    return new, step, False


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def path_length(points: list[Point]) -> float:
    return sum(distance(points[i], points[i + 1]) for i in range(len(points) - 1))


def point_segment_projection(p: Point, a: Point, b: Point) -> tuple[Point, float]:
    """Closest point on segment ab to p, and the parameter t in [0,1]."""
    abx, aby = b[0] - a[0], b[1] - a[1]
    denom = abx * abx + aby * aby
    if denom == 0:
        return a, 0.0
    t = clamp(((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / denom, 0.0, 1.0)
    return (a[0] + abx * t, a[1] + aby * t), t
