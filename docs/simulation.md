# Simulation

* **World:** 1000 × 1000 m, 20 m cells, home base near (60, 60), +y = north, heading = compass bearing.
* **Fleet:** 12 drones (configurable 1–40), seeded battery/health/reliability, all have a camera; some also thermal and/or lidar.
* **Time:** fixed dt = 0.5 s; `sim_time` is the only clock (timestamps derived from it → deterministic).
* **Movement:** kinematic follow-the-route at 18 m/s cruise (manual 14 m/s), waypoint threshold 6 m, no
  teleporting (an invariant monitor checks per-tick displacement ≤ max speed·dt in every scenario).
* **Takeoff/landing:** vertical phases to the drone's altitude layer; `LANDED` event when on the ground.
* **Battery:** 0.012 %/m plus hover and idle drain, scaled by a fault multiplier. **Thermal:** heats under load, cools on ground.
  **GPS/signal:** degrade only through injected faults.
* **Faults** (`simulation/failures.py`, pure functions): LOW_BATTERY, GPS_DEGRADATION, SIGNAL_LOSS, HIGH_TEMPERATURE,
  MOTOR_HEALTH_DROP, DRONE_FAILURE (drone), OBSTACLE_APPEARED, WEATHER_RISK (world).
* **Zones:** inspection A/B/North/West; obstacles (radio mast, water tower); restricted airspace (2). Geofence:
  manual movement into a blocked cell is refused by the engine's `collision_fn`.
* **Altitude layers:** `base + 4 m × layer`; a layer is held while a drone is busy and released on the ground, so
  concurrent drones are vertically separated (scenario 7 checks this).
