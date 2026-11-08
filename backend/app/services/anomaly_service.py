"""Maintenance / anomaly extension: runs the ML models on live simulated telemetry.

Runs every few ticks as an orchestrator extension. It *reports* (ANOMALY_DETECTED events,
maintenance view, risk scores used by drone selection). It never commands a drone: hard
failsafes stay in the deterministic safety monitor.
"""
from __future__ import annotations

import numpy as np

from app.ai.maintenance import MaintenanceModels, features_from_drone
from app.core.enums import EventType
from app.services.orchestrator import FleetOrchestrator

CHECK_EVERY_TICKS = 10
DEDUPE_S = 60.0
LOW_SIGNAL = 0.3
DROP_WINDOW_S = 120.0


class AnomalyService:
    def __init__(self, models: MaintenanceModels) -> None:
        self.models = models
        self._low_since: dict[str, bool] = {}
        self._drops: dict[str, list[float]] = {}
        self._last_flag: dict[str, tuple[str, float]] = {}

    def __call__(self, o: FleetOrchestrator) -> None:
        eng = o.ctx.engine
        now = o.ctx.now
        # track signal-drop episodes every tick (cheap)
        for d in eng.drones_iter():
            low = d.is_airborne and d.signal_strength < LOW_SIGNAL
            if low and not self._low_since.get(d.drone_id):
                self._drops.setdefault(d.drone_id, []).append(now)
            self._low_since[d.drone_id] = low
        if eng.tick_count % CHECK_EVERY_TICKS:
            return
        drones = list(eng.drones_iter())
        X = np.vstack([features_from_drone(d, self._recent_drops(d.drone_id, now)) for d in drones])
        for d, a in zip(drones, self.models.assess_many([d.drone_id for d in drones], X)):
            o.ctx.maintenance_view[d.drone_id] = a.to_dict()
            o.ctx.maintenance_risk[d.drone_id] = a.risk_score
            if not d.is_airborne:
                continue
            flagged, conf = self.models.detection(a)
            if not flagged:
                continue
            sig = "+".join(x["feature"] for x in a.drivers[:2]) or "pattern"
            last = self._last_flag.get(d.drone_id)
            if last and last[0] == sig and now - last[1] < DEDUPE_S:
                continue
            self._last_flag[d.drone_id] = (sig, now)
            severity = "HIGH" if a.maintenance_risk == "HIGH" else "MEDIUM"
            o.ctx.log(EventType.ANOMALY_DETECTED,
                      f"{d.drone_id} anomaly: {a.reason}. Severity: {severity}. Confidence: {conf * 100:.0f}%",
                      drone_id=d.drone_id, mission_id=d.current_mission_id,
                      severity="CRITICAL" if severity == "HIGH" else "WARNING",
                      data={"severity": severity, "confidence": round(conf, 3),
                            "anomaly_probability": round(a.anomaly_probability, 3),
                            "estimated_health": round(a.estimated_health, 1), "drivers": a.drivers,
                            "isolation_flag": a.isolation_flag, "advisory": True})
            o.ctx.decide("ANOMALY", f"ML flagged {d.drone_id}: {a.reason} "
                         f"(P(anomaly)={a.anomaly_probability:.2f}, risk {a.maintenance_risk})",
                         drone_id=d.drone_id, mission_id=d.current_mission_id, score=a.anomaly_probability,
                         reasons=[a.reason, "advisory only: safety rules keep control authority",
                                  "model trained on synthetic data"],
                         extra={"drivers": a.drivers})

    def _recent_drops(self, drone_id: str, now: float) -> float:
        lst = [t for t in self._drops.get(drone_id, []) if now - t <= DROP_WINDOW_S]
        self._drops[drone_id] = lst
        return float(len(lst))
