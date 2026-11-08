"""Synthetic historical telemetry for the predictive-maintenance prototype.

THIS IS SIMULATED DATA. It is generated from hand-written physical-ish relationships and fault
patterns, so any model trained on it demonstrates the ML workflow (features, training,
evaluation, inference) — it does NOT predict real-world drone failures and no real-world
accuracy is claimed.

Feature order (also used at inference time from live simulated telemetry):
  temperature (C), vibration (g RMS), battery_cycles, voltage (V), current (A),
  flight_hours, gps_error (m), motor_rpm, signal_drops (episodes / 2 min)

Fault patterns injected into the abnormal class:
  OVERHEAT          motor temperature +20..45 C
  BEARING_WEAR      vibration x2..4, temperature +8..18 C, rpm drops
  BATTERY_AGING     voltage sag, higher current draw
  GPS_FAULT         gps error x5..20
  LINK_INSTABILITY  repeated signal drops
  (patterns can combine; the label is "anomalous" if any is active)
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

FEATURES = ["temperature", "vibration", "battery_cycles", "voltage", "current", "flight_hours",
            "gps_error", "motor_rpm", "signal_drops"]
FAULT_KINDS = ["OVERHEAT", "BEARING_WEAR", "BATTERY_AGING", "GPS_FAULT", "LINK_INSTABILITY"]


def electrical_from_state(battery_pct: float, speed: float, health_pct: float) -> tuple[float, float, float]:
    """(voltage, current, motor_rpm) derived from operating state (shared by training and live)."""
    current = 6.0 + 0.55 * speed + 0.05 * (100.0 - health_pct)
    voltage = 9.0 + 3.6 * (max(battery_pct, 0.0) / 100.0) ** 0.8 - 0.03 * current
    rpm = 4600.0 + 55.0 * speed - 6.0 * (100.0 - health_pct)
    return voltage, current, rpm


@dataclass
class Dataset:
    X: np.ndarray
    health: np.ndarray          # target 0..100
    anomaly: np.ndarray         # 0/1
    fault: np.ndarray           # index into FAULT_KINDS or -1
    feature_names: list[str]


def generate(n: int = 6000, seed: int = 42, anomaly_fraction: float = 0.25) -> Dataset:
    rng = np.random.default_rng(seed)
    battery = rng.uniform(25, 100, n)
    speed = np.where(rng.random(n) < 0.2, 0.0, rng.uniform(5, 19, n))        # some hover samples
    wear_health = np.clip(rng.normal(92, 7, n), 55, 100)                      # slow ageing, not a fault
    cycles = rng.integers(5, 400, n).astype(float)
    hours = rng.uniform(5, 450, n)

    temp = 25 + np.where(speed > 0, 8 + 25 * speed / 25.0, 0.0) + rng.normal(0, 2.0, n)
    vib = 0.10 + 0.25 * (1 - wear_health / 100) + 0.05 * speed / 25.0 + np.abs(rng.normal(0, 0.012, n))
    gps_err = np.abs(rng.normal(0.12, 0.06, n)) * 5
    drops = rng.poisson(0.25, n).astype(float)
    volt, cur, rpm = (np.empty(n) for _ in range(3))
    for i in range(n):
        volt[i], cur[i], rpm[i] = electrical_from_state(battery[i], speed[i], wear_health[i])
    volt += rng.normal(0, 0.05, n)
    cur += rng.normal(0, 0.4, n)
    rpm += rng.normal(0, 40, n)

    anomaly = (rng.random(n) < anomaly_fraction).astype(int)
    fault = np.full(n, -1)
    penalty = np.zeros(n)
    for i in np.flatnonzero(anomaly):
        kinds = rng.choice(len(FAULT_KINDS), size=rng.integers(1, 3), replace=False)
        fault[i] = kinds[0]
        for k in kinds:
            kind = FAULT_KINDS[k]
            if kind == "OVERHEAT":
                d = rng.uniform(20, 45); temp[i] += d; penalty[i] += 0.5 * d
            elif kind == "BEARING_WEAR":
                m = rng.uniform(2, 4); vib[i] *= m; temp[i] += rng.uniform(8, 18)
                rpm[i] -= rng.uniform(150, 500); penalty[i] += 9 * m
            elif kind == "BATTERY_AGING":
                s = rng.uniform(0.6, 1.6); volt[i] -= s; cur[i] += rng.uniform(3, 9); penalty[i] += 14 * s
            elif kind == "GPS_FAULT":
                g = rng.uniform(5, 20); gps_err[i] *= g; penalty[i] += 1.2 * g
            elif kind == "LINK_INSTABILITY":
                c = rng.integers(3, 9); drops[i] += c; penalty[i] += 3 * c

    health = np.clip(wear_health - penalty + rng.normal(0, 1.5, n), 0, 100)
    X = np.column_stack([temp, vib, cycles, volt, cur, hours, gps_err, rpm, drops])
    return Dataset(X, health, anomaly, fault, list(FEATURES))
