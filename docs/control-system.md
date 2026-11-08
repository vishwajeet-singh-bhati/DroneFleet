# Control system

## Flight modes
`IDLE, TAKEOFF, AUTONOMOUS, TAKEOVER_REQUEST, MANUAL, REJOINING, RETURN_TO_HOME, PAUSED, LANDING, EMERGENCY`.
`control/state_machine.py` holds an explicit allowed-transition table; repeating a command that already holds is an
idempotent no-op, an illegal one is a `409`.

## Human takeover
`AUTONOMOUS → TAKEOVER_REQUEST → MANUAL` (both steps logged, a control session is stored). While MANUAL the mission
route is **held, not discarded**, so the drone can rejoin it. Manual input (`forward, strafe, climb, yaw`) is
validated (finite numbers, extra keys rejected), clamped to [-1, 1], and expires after 0.6 s without a new message
(dead-man). The engine refuses moves into obstacles/restricted cells (geofence, event `COMMAND_REJECTED`).
UI keys: `W/S` forward/back, `A/D` strafe, `Q/E` yaw, `R/F` climb — resent at 10 Hz while held.

## RESUME MISSION: safe rejoin (`control/rejoin.py`)
No teleport and no skipped inspection waypoints.
1. Candidates: the **next waypoint**, or the **projection of the drone onto a route leg**. A later leg is only
   allowed if every waypoint it skips is `TRANSIT` (never an INSPECT/HOVER point).
2. Feasibility: A* path from the drone to the candidate must exist and the suffix of the route from the candidate must
   be obstacle-free.
3. Cost = rejoin path length + remaining route length; the cheapest feasible candidate wins (`NEXT_WAYPOINT` or
   `LEG_PROJECTION`).
4. If the old route is blocked, strategy `REPLAN` rebuilds the remaining mission route and hops to its first waypoint.
5. Battery check at resume: if the drone cannot finish and get home, it returns home and the work is reassigned.
The drone flies the rejoin path in `REJOINING` (drawn dotted in the UI), then `AUTONOMOUS`; events
`ROUTE_REJOIN_STARTED/COMPLETED` and an AI-decision record explain the choice.

## Return-to-home & emergency
RTH plans an A* path home (`HOME`), pauses/aborts the mission leg, and the mission is reassigned. EMERGENCY lands
immediately where the drone is.

## Safety monitor (rule-based, `safety/monitor.py`)
"Can I make it home" battery rule (required × 1.35 or ≤ 15 %), health, temperature, comm-loss (5 s) failsafe RTH,
GPS degradation derate/land. Recovery actions are deduplicated per (drone, kind, action).
