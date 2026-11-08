"""Predictive maintenance + anomaly detection models (scikit-learn).

Models (all trained on SIMULATED data, see synthetic_data.py — a prototype, not validated for
real drones):
  * health regressor      GradientBoostingRegressor -> estimated health score 0..100
  * anomaly classifier    GradientBoostingClassifier -> P(anomalous) (supervised, uses fault labels)
  * isolation forest      IsolationForest trained on NORMAL samples only (unsupervised) -> flags
                          telemetry that looks unlike normal operation, even for unseen fault types

Maintenance risk (LOW / MEDIUM / HIGH) combines the supervised anomaly probability and the
estimated health; the explanation lists the features that deviate most from the normal baseline.
Evaluation metrics on a held-out split are produced by `train()` and saved next to the models.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import joblib
import numpy as np
from sklearn.ensemble import GradientBoostingClassifier, GradientBoostingRegressor, IsolationForest
from sklearn.metrics import mean_absolute_error, precision_score, r2_score, recall_score, roc_auc_score
from sklearn.model_selection import train_test_split

from app.ai.synthetic_data import FEATURES, Dataset, electrical_from_state, generate
from app.simulation.drone_state import DroneState

MODEL_VERSION = 3
RISK_HIGH, RISK_MEDIUM = 0.55, 0.28
DISPLAY = {"temperature": ("temperature", "°C"), "vibration": ("vibration", " g"),
           "voltage": ("voltage", " V"), "current": ("current draw", " A"), "motor_rpm": ("motor RPM", ""),
           "gps_error": ("GPS error", " m"), "signal_drops": ("signal drops", ""),
           "battery_cycles": ("battery cycles", ""), "flight_hours": ("flight hours", " h")}


@dataclass
class Assessment:
    drone_id: str
    estimated_health: float
    anomaly_probability: float
    maintenance_risk: str                 # LOW | MEDIUM | HIGH
    risk_score: float
    isolation_score: float                # < threshold => anomalous
    isolation_flag: bool
    drivers: list[dict] = field(default_factory=list)       # [{feature, z, value}]
    reason: str = ""

    def to_dict(self) -> dict:
        return {"drone_id": self.drone_id, "estimated_health": round(self.estimated_health, 1),
                "anomaly_probability": round(self.anomaly_probability, 3),
                "maintenance_risk": self.maintenance_risk, "risk_score": round(self.risk_score, 3),
                "isolation_score": round(self.isolation_score, 4), "isolation_flag": self.isolation_flag,
                "drivers": self.drivers, "reason": self.reason,
                "disclaimer": "simulation prototype trained on synthetic data"}


class MaintenanceModels:
    def __init__(self, regressor, classifier, iforest, mean, std, iso_threshold, iso_scale, metrics) -> None:
        self.regressor, self.classifier, self.iforest = regressor, classifier, iforest
        self.mean, self.std = np.asarray(mean), np.asarray(std)
        self.iso_threshold, self.iso_scale = float(iso_threshold), float(iso_scale)
        self.metrics = metrics

    # ------------------------------------------------------------- inference
    def assess_vector(self, drone_id: str, x: np.ndarray) -> Assessment:
        return self.assess_many([drone_id], np.asarray(x, dtype=float).reshape(1, -1))[0]

    def assess_many(self, drone_ids: list[str], X: np.ndarray) -> list[Assessment]:
        """Vectorised: one model call per batch (cheap enough to run every few ticks)."""
        X = np.asarray(X, dtype=float).reshape(len(drone_ids), -1)
        health = np.clip(self.regressor.predict(X), 0, 100)
        prob = self.classifier.predict_proba(X)[:, 1]
        iso = self.iforest.decision_function(X)
        out = []
        for i, did in enumerate(drone_ids):
            z = (X[i] - self.mean) / self.std
            drivers = [{"feature": f, "z": round(float(zi), 2), "value": round(float(v), 3)}
                       for f, zi, v in zip(FEATURES, z, X[i]) if abs(zi) >= 2.5 and f not in
                       ("battery_cycles", "flight_hours")]
            drivers.sort(key=lambda d: -abs(d["z"]))
            risk_score = 0.6 * float(prob[i]) + 0.4 * (1 - float(health[i]) / 100)
            risk = "HIGH" if risk_score >= RISK_HIGH else "MEDIUM" if risk_score >= RISK_MEDIUM else "LOW"
            out.append(Assessment(did, float(health[i]), float(prob[i]), risk, risk_score, float(iso[i]),
                                  bool(iso[i] < self.iso_threshold), drivers[:4], self._explain(drivers)))
        return out

    def detection(self, a: Assessment) -> tuple[bool, float]:
        """(is_anomaly, confidence). Supervised probability OR isolation-forest flag."""
        if a.anomaly_probability >= 0.8:
            return True, a.anomaly_probability
        if a.isolation_flag:
            return True, self.confidence(a.isolation_score)
        return False, 0.0

    @staticmethod
    def _explain(drivers: list[dict]) -> str:
        if not drivers:
            return "telemetry within normal operating range"
        names = [DISPLAY[d["feature"]][0] for d in drivers[:3]]
        text = "Unusual " + " + ".join(names) + " pattern"
        top = drivers[0]
        label, unit = DISPLAY[top["feature"]]
        return f"{text} ({label} {top['value']:.2f}{unit}, {top['z']:+.1f}σ from normal)"

    def confidence(self, iso_score: float) -> float:
        """Map the isolation-forest margin below the threshold onto 0.5..0.99 (monotone, heuristic)."""
        margin = (self.iso_threshold - iso_score) / self.iso_scale
        return float(np.clip(0.5 + 0.49 * np.tanh(margin * 2.0), 0.5, 0.99))

    # ------------------------------------------------------------- persistence
    def save(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        joblib.dump({"version": MODEL_VERSION, "regressor": self.regressor, "classifier": self.classifier,
                     "iforest": self.iforest, "mean": self.mean, "std": self.std,
                     "iso_threshold": self.iso_threshold, "iso_scale": self.iso_scale,
                     "metrics": self.metrics}, directory / "maintenance.joblib")
        (directory / "metrics.json").write_text(json.dumps(self.metrics, indent=2))

    @classmethod
    def load(cls, directory: Path) -> "MaintenanceModels | None":
        p = directory / "maintenance.joblib"
        if not p.exists():
            return None
        try:
            d = joblib.load(p)
            if d.get("version") != MODEL_VERSION:
                return None
            return cls(d["regressor"], d["classifier"], d["iforest"], d["mean"], d["std"],
                       d["iso_threshold"], d["iso_scale"], d["metrics"])
        except Exception:
            return None


def train(n: int = 6000, seed: int = 42, data: Dataset | None = None) -> MaintenanceModels:
    ds = data or generate(n, seed)
    X_tr, X_te, h_tr, h_te, a_tr, a_te = train_test_split(
        ds.X, ds.health, ds.anomaly, test_size=0.25, random_state=seed, stratify=ds.anomaly)

    reg = GradientBoostingRegressor(n_estimators=160, max_depth=3, learning_rate=0.08, random_state=seed)
    reg.fit(X_tr, h_tr)
    clf = GradientBoostingClassifier(n_estimators=140, max_depth=3, learning_rate=0.1, random_state=seed)
    clf.fit(X_tr, a_tr)

    normal = X_tr[a_tr == 0]
    iso = IsolationForest(n_estimators=200, contamination="auto", random_state=seed)
    iso.fit(normal)
    normal_scores = iso.decision_function(normal)
    threshold = float(np.percentile(normal_scores, 3.0))        # ~3% false-positive budget on training normals
    scale = float(np.std(normal_scores)) or 1.0

    te_prob = clf.predict_proba(X_te)[:, 1]
    te_pred = (te_prob >= 0.5).astype(int)
    iso_te = iso.decision_function(X_te)
    iso_flag = iso_te < threshold
    metrics = {
        "disclaimer": "Trained and evaluated on SIMULATED data only; not a claim about real-world accuracy.",
        "model_version": MODEL_VERSION, "seed": seed, "n_samples": int(len(ds.X)),
        "n_train": int(len(X_tr)), "n_test": int(len(X_te)), "features": FEATURES,
        "health_regressor": {"mae": round(float(mean_absolute_error(h_te, reg.predict(X_te))), 3),
                             "r2": round(float(r2_score(h_te, reg.predict(X_te))), 4)},
        "anomaly_classifier": {"roc_auc": round(float(roc_auc_score(a_te, te_prob)), 4),
                               "precision": round(float(precision_score(a_te, te_pred)), 4),
                               "recall": round(float(recall_score(a_te, te_pred)), 4)},
        "isolation_forest": {"roc_auc": round(float(roc_auc_score(a_te, -iso_te)), 4),
                             "detection_rate": round(float(iso_flag[a_te == 1].mean()), 4),
                             "false_positive_rate": round(float(iso_flag[a_te == 0].mean()), 4),
                             "threshold": round(threshold, 5)},
    }
    mean, std = normal.mean(axis=0), normal.std(axis=0) + 1e-9
    return MaintenanceModels(reg, clf, iso, mean, std, threshold, scale, metrics)


_CACHE: dict[tuple, MaintenanceModels] = {}


def get_models(model_dir: Path | None = None, n: int = 6000, seed: int = 42) -> MaintenanceModels:
    """Load from disk if a compatible model exists, otherwise train (and save when a dir is given)."""
    key = (str(model_dir), n, seed)
    if key in _CACHE:
        return _CACHE[key]
    models = MaintenanceModels.load(model_dir) if model_dir else None
    if models is None or models.metrics.get("n_samples") != n or models.metrics.get("seed") != seed:
        models = train(n, seed)
        if model_dir:
            models.save(model_dir)
    _CACHE[key] = models
    return models


def features_from_drone(d: DroneState, signal_drops: float = 0.0) -> np.ndarray:
    """Map live simulated telemetry onto the model's feature vector."""
    volt, cur, rpm = electrical_from_state(d.battery_percentage, d.velocity, d.health_percentage)
    gps_error = (1.0 - d.gps_quality) * 6.0
    return np.array([d.temperature, d.vibration, float(d.battery_cycles), volt, cur, d.flight_hours,
                     gps_error, rpm, signal_drops], dtype=float)
