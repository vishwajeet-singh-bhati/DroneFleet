import math

import pytest

from app.core.config import SimConfig, WorldConfig
from app.core.enums import WaypointAction, ZoneKind
from app.core.types import Waypoint
from app.mission.astar import SQRT2, astar_cells, octile, plan_xy, smooth_path
from app.mission.coverage import (
    CoverageMap, InspectionTarget, lawnmower_targets, nearest_neighbour_order, order_targets,
    prioritise, split_among, _path_len,
)
from app.mission.estimates import required_battery_pct, route_length
from app.mission.grid import OccupancyGrid, Zone
from app.mission.route_planner import HOME_ID, RoutePlanner
from app.utils.geometry import distance


def small_grid(margin=0.0) -> OccupancyGrid:
    # 20x20 cells of 20 m = 400 m world
    return OccupancyGrid(WorldConfig(width=400, height=400, cell_size=20), margin=margin)


def wall(grid, x0, y0, x1, y1, zid="W", kind=ZoneKind.OBSTACLE):
    grid.add_zone(Zone(zid, kind, x0, y0, x1, y1))


# ----------------------------------------------------------------------- grid
def test_zone_rasterises_to_blocked_cells():
    g = small_grid()
    wall(g, 100, 100, 140, 140)
    assert g.is_blocked_xy((110, 110))
    assert not g.is_blocked_xy((50, 50))
    assert g.is_blocked_cell((-1, 0))              # out of bounds counts as blocked


def test_margin_inflates_obstacles():
    g = small_grid(margin=20.0)
    wall(g, 100, 100, 140, 140)
    assert g.is_blocked_xy((90, 110))              # one cell of buffer


def test_deactivating_zone_unblocks_and_bumps_version():
    g = small_grid()
    wall(g, 100, 100, 140, 140)
    v = g.version
    g.set_zone_active("W", False)
    assert not g.is_blocked_xy((110, 110))
    assert g.version > v


def test_inspection_zone_never_blocks():
    g = small_grid()
    wall(g, 100, 100, 200, 200, kind=ZoneKind.INSPECTION)
    assert not g.is_blocked_xy((150, 150))


def test_nearest_free_cell_snaps_off_obstacle():
    g = small_grid()
    wall(g, 100, 100, 140, 140)
    c = g.nearest_free_cell(g.cell_of((110, 110)))
    assert c is not None and not g.is_blocked_cell(c)


# ----------------------------------------------------------------------- A*
def test_astar_open_grid_cost_equals_octile():
    g = small_grid()
    start, goal = (1, 1), (15, 8)
    path, cost = astar_cells(g, start, goal)
    assert cost == pytest.approx(octile(start, goal))
    assert path[0] == start and path[-1] == goal


def test_astar_straight_line():
    g = small_grid()
    path, cost = astar_cells(g, (2, 2), (10, 2))
    assert cost == pytest.approx(8.0)


def test_astar_goes_around_wall():
    g = small_grid()
    wall(g, 180, 0, 220, 300)                      # vertical wall, gap at the top
    res = astar_cells(g, (2, 5), (17, 5))
    assert res is not None
    path, _ = res
    assert all(not g.is_blocked_cell(c) for c in path)
    assert max(c[1] for c in path) >= 15           # had to detour through the gap


def test_astar_unreachable_returns_none():
    g = small_grid()
    wall(g, 180, 0, 220, 400)                      # full-height wall
    assert astar_cells(g, (2, 5), (17, 5)) is None


def test_astar_blocked_goal_returns_none():
    g = small_grid()
    wall(g, 100, 100, 140, 140)
    assert astar_cells(g, (1, 1), (5, 5)) is None


def test_astar_no_corner_cutting():
    g = small_grid()
    # Two diagonal-touching blocks: (5,5) and (6,6)... path from (5,6) to (6,5) must not slip between
    wall(g, 100, 100, 120, 120, zid="A")           # cell (5,5)
    wall(g, 120, 120, 140, 140, zid="B")           # cell (6,6)
    res = astar_cells(g, (5, 6), (6, 5))
    assert res is not None
    path, cost = res
    assert len(path) > 2                           # no direct diagonal squeeze
    assert cost > SQRT2


def test_astar_prefers_cheap_detour_around_weather():
    g = small_grid()
    wall(g, 100, 100, 300, 300, zid="WX", kind=ZoneKind.WEATHER)
    path, _ = astar_cells(g, (2, 10), (18, 10))
    in_weather = [c for c in path if g.risk_at(c) > 0]
    # a path that dodges the 10x10 weather block is cheaper than crossing it
    assert len(in_weather) == 0


def test_astar_deterministic():
    g = small_grid()
    wall(g, 100, 60, 140, 300)
    a = astar_cells(g, (1, 1), (18, 18))
    b = astar_cells(g, (1, 1), (18, 18))
    assert a == b


# ----------------------------------------------------------------------- smoothing / xy planning
def test_smooth_path_keeps_line_of_sight():
    g = small_grid()
    wall(g, 180, 0, 220, 300)
    pts = plan_xy(g, (30, 100), (370, 100))
    assert pts is not None
    for a, b in zip(pts, pts[1:]):
        assert g.line_free(a, b)
    assert pts[0] == (30, 100) and pts[-1] == (370, 100)


def test_plan_xy_open_space_is_direct():
    g = small_grid()
    pts = plan_xy(g, (30, 30), (350, 350))
    assert pts == [(30, 30), (350, 350)]


def test_plan_xy_unreachable():
    g = small_grid()
    wall(g, 180, 0, 220, 400)
    assert plan_xy(g, (30, 100), (370, 100)) is None


def test_smooth_reduces_points():
    g = small_grid()
    raw = [(10, 10), (30, 30), (50, 50), (70, 70), (90, 90)]
    assert smooth_path(g, raw) == [(10, 10), (90, 90)]


# ----------------------------------------------------------------------- coverage
def zone_a():
    return Zone("ZA", ZoneKind.INSPECTION, 100, 100, 340, 220)


def test_lawnmower_covers_zone_in_snake_order():
    g = small_grid()
    ts = lawnmower_targets(zone_a(), g, spacing=60)
    assert len(ts) == 4 * 2
    assert all(zone_a().contains(t.xy) for t in ts)
    # snake: second column runs the opposite direction in y
    assert ts[0].y < ts[1].y and ts[2].y > ts[3].y


def test_lawnmower_skips_blocked_points_and_keeps_ids_stable():
    g = small_grid()
    free = lawnmower_targets(zone_a(), g, spacing=60)
    wall(g, 150, 100, 200, 220)
    blocked = lawnmower_targets(zone_a(), g, spacing=60)
    assert len(blocked) < len(free)
    assert {t.target_id for t in blocked} <= {t.target_id for t in free}


def test_prioritise_uninspected():
    g = small_grid()
    ts = lawnmower_targets(zone_a(), g, spacing=60)
    cov = CoverageMap()
    cov.mark_many([t.target_id for t in ts[:3]])
    fresh = prioritise(ts, cov)
    assert [t.target_id for t in fresh] == [t.target_id for t in ts[3:]]
    both = prioritise(ts, cov, include_inspected=True)
    assert len(both) == len(ts) and both[-1].target_id == ts[2].target_id
    assert cov.ratio(ts) == pytest.approx(3 / len(ts))


def test_split_among_is_balanced_and_complete():
    ts = [InspectionTarget(f"t{i}", i, i) for i in range(10)]
    parts = split_among(ts, 3)
    assert [len(p) for p in parts] == [4, 3, 3]
    assert [t for p in parts for t in p] == ts
    with pytest.raises(ValueError):
        split_among(ts, 0)


def test_two_opt_never_worse_than_nearest_neighbour():
    pts = [InspectionTarget(f"t{i}", x, y) for i, (x, y) in enumerate(
        [(10, 90), (80, 10), (20, 20), (90, 90), (50, 50), (5, 60), (70, 30)])]
    start = (0.0, 0.0)
    nn = nearest_neighbour_order(start, pts)
    opt = order_targets(start, pts, end=(0.0, 0.0))
    nn_len = _path_len(start, [t.xy for t in nn], (0.0, 0.0))
    opt_len = _path_len(start, [t.xy for t in opt], (0.0, 0.0))
    assert opt_len <= nn_len + 1e-9
    assert sorted(t.target_id for t in opt) == sorted(t.target_id for t in pts)


# ----------------------------------------------------------------------- route planner
def test_build_route_ends_at_home_with_inspect_waypoints():
    g = small_grid()
    rp = RoutePlanner(g)
    targets = [InspectionTarget("a", 150, 150), InspectionTarget("b", 250, 150)]
    res = rp.build_route((40, 40), targets, home=(40, 40))
    assert res.ok and not res.skipped
    ids = [w.id for w in res.route]
    assert ids[-1] == HOME_ID
    inspects = [w for w in res.route if w.action == WaypointAction.INSPECT]
    assert [w.id for w in inspects] == ["a", "b"]
    assert res.route[-1].action == WaypointAction.LAND


def test_build_route_avoids_obstacle_and_skips_unreachable_targets():
    g = small_grid()
    wall(g, 180, 0, 220, 300)
    rp = RoutePlanner(g)
    res = rp.build_route((40, 100), [InspectionTarget("far", 350, 100),
                                     InspectionTarget("inside", 200, 100)], home=(40, 100))
    assert res.skipped == ["inside"]
    cur = (40, 100)
    for w in res.route:
        assert g.line_free(cur, w.xy)
        cur = w.xy


def test_first_blocked_segment_detects_new_obstacle():
    g = small_grid()
    rp = RoutePlanner(g)
    route = rp.build_route((40, 100), [InspectionTarget("t", 350, 100)], home=(40, 100)).route
    assert rp.route_is_clear((40, 100), route)
    wall(g, 180, 60, 220, 140, zid="NEW")
    assert not rp.route_is_clear((40, 100), route)
    assert rp.first_blocked_segment((40, 100), route) is not None


def test_replan_remaining_routes_around_new_obstacle_and_keeps_inspections():
    g = small_grid()
    rp = RoutePlanner(g)
    targets = [InspectionTarget("t1", 150, 100), InspectionTarget("t2", 350, 100)]
    original = rp.build_route((40, 100), targets, home=(40, 100)).route
    wall(g, 240, 60, 280, 140, zid="NEW")
    res = rp.replan_remaining((150, 100), [w for w in original if w.id != "t1"], (40, 100), "R1-")
    assert res.ok
    assert [w.id for w in res.route if w.action == WaypointAction.INSPECT] == ["t2"]
    assert res.route[-1].id == HOME_ID
    cur = (150, 100)
    for w in res.route:
        assert g.line_free(cur, w.xy)
        cur = w.xy


def test_replan_drops_waypoint_that_becomes_unavailable():
    g = small_grid()
    rp = RoutePlanner(g)
    targets = [InspectionTarget("t1", 150, 100), InspectionTarget("t2", 300, 100)]
    original = rp.build_route((40, 100), targets, home=(40, 100)).route
    wall(g, 280, 80, 320, 120, zid="ON_T2")
    res = rp.replan_remaining((40, 100), original, (40, 100), "R1-")
    assert "t2" in res.skipped
    assert [w.id for w in res.route if w.action == WaypointAction.INSPECT] == ["t1"]


def test_return_home_route_reaches_home():
    g = small_grid()
    wall(g, 100, 0, 140, 300)
    rp = RoutePlanner(g)
    res = rp.return_home((300, 100), (40, 100))
    assert res.ok and res.route[-1].xy == (40, 100) and res.route[-1].id == HOME_ID


def test_required_battery_scales_with_route_length():
    cfg = SimConfig()
    g = small_grid()
    rp = RoutePlanner(g)
    short = rp.build_route((40, 40), [InspectionTarget("a", 80, 80)], (40, 40)).route
    long_ = rp.build_route((40, 40), [InspectionTarget("a", 380, 380)], (40, 40)).route
    assert required_battery_pct((40, 40), long_, cfg) > required_battery_pct((40, 40), short, cfg)
    assert route_length((40, 40), long_) > route_length((40, 40), short)
