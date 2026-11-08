# Mission engine

## Lifecycle
`CREATED → PLANNING → READY → ACTIVE ⇄ PAUSED / REPLANNING → COMPLETED | FAILED | ABORTED`
Transitions are an explicit table (`mission/models.py`); illegal ones raise `InvalidTransition` (HTTP 409).

## Planning
1. **Targets.** `lawnmower_targets` lays a lattice (≈60 m) over the inspection zone. Ids are stable
   (`ZONE_A:3-1`) so coverage survives replanning; points inside blocked cells are skipped and reported.
2. **Selection.** `ai/selector.py` ranks drones (see ai.md); the top N are reserved (prevents double-booking).
3. **Split + order.** Targets are split contiguously among drones, ordered by nearest-neighbour + 2-opt.
4. **Route.** `RoutePlanner.build_route` chains A* legs start → targets → home, string-pulls them, adds a cruise
   altitude layer, and verifies battery feasibility (`required_battery_pct`) and capabilities.

## A*
8-connected grid, no corner cutting, octile heuristic, deterministic tie-break counter, extra cost inside WEATHER
zones (risk-aware, not forbidden), margin around obstacles/restricted zones. `line_free` is an exact
Amanatides–Woo traversal (including exact-corner and end-on-corner cases) used for smoothing and for validating
every leg, so smoothing can never cut through a blocked cell.

## Dynamic replanning
`on_grid_changed` fires when an obstacle/restricted/weather zone appears. For every active drone,
`first_blocked_segment` checks the *remaining* route; if blocked, `replan_remaining` plans only the rest, keeps the
completed prefix, records `{old, new, dropped}` in `route_history` (the UI draws old = red dashed, new = green dotted)
and emits `ROUTE_REPLANNED`. Targets that become unreachable are reported as uncovered, never silently dropped.

## Failure handling & reassignment
`SafetyMonitor` findings → `RecoveryService` actions (deduped per drone/kind/action) → RTH or emergency land →
`release_and_reassign`: remaining targets are re-ranked across available drones, each candidate route is validated
against the real battery/health, otherwise work is redistributed to drones already flying, otherwise the mission
finishes with partial coverage (or fails if nothing was covered). The mission report lists coverage, per-drone
distance/battery, replans, takeovers, rejoins and reassignments.
