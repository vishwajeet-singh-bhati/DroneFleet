"""Explainable drone-suitability ranking.

Input      : candidate drones + a mission context (target location, required capabilities,
             estimated route length/duration, environment risk).
Processing : each drone is mapped to normalised features in [0, 1]; the suitability score is a
             weighted linear combination minus a risk penalty. Hard constraints (status,
             minimum battery/health, capabilities) are applied first and make a drone ineligible
             regardless of score.
Output     : a ranked list; each entry carries the score, the per-factor contribution
             (weight * feature, so contributions sum exactly to the score) and human-readable
             reasons. This is what gets written to the AI decision log.

Why a transparent model instead of a black box: the decision must be explainable and auditable,
we have no real outcome data to fit a high-capacity model on, and a linear multi-criteria model
is easy to test, tune and reason about. Weights are explicit constants (see WEIGHTS), not magic.
The AI *recommends*; the safety validator (app.safety.validator) has the final say.

Evaluation: scripts/evaluate_selector.py compares top-ranked selection against random eligible
selection on simulated missions (battery margin, mission success, duration).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from app.core.enums import Capability
from app.safety.validator import assignment_violations
from app.simulation.drone_state import DroneState
from app.utils.geometry import clamp, distance

WEIGHTS: dict[str, float] = {
    "battery": 0.22,
    "health": 0.20,
    "proximity": 0.16,
    "capability": 0.14,
    "reliability": 0.10,
    "endurance": 0.10,
    "workload": 0.08,
}
RISK_PENALTY_WEIGHT = 0.15
assert abs(sum(WEIGHTS.values()) - 1.0) < 1e-9


@dataclass
class SelectionContext:
    target: tuple[float, float]                  # mission centroid / first target
    required_caps: frozenset[Capability]
    world_diagonal: float = 1414.0
    est_route_length: float = 600.0              # metres a typical drone would fly
    est_battery_pct: float = 12.0                # battery that route would consume
    env_risk: float = 0.0                        # 0..1, e.g. weather over the target area
    busy_drones: frozenset[str] = frozenset()    # drones with active work (workload=0)
    maintenance_risk: dict[str, float] = field(default_factory=dict)   # drone_id -> 0..1 (ML extension)


@dataclass
class Ranking:
    drone_id: str
    eligible: bool
    score: float | None = None
    features: dict[str, float] = field(default_factory=dict)
    contributions: dict[str, float] = field(default_factory=dict)   # includes "risk" (negative)
    reasons: list[str] = field(default_factory=list)                # why selected / why good
    rejections: list[str] = field(default_factory=list)             # why ineligible

    def to_dict(self) -> dict:
        return {"drone_id": self.drone_id, "eligible": self.eligible,
                "score": None if self.score is None else round(self.score, 4),
                "features": {k: round(v, 3) for k, v in self.features.items()},
                "contributions": {k: round(v, 4) for k, v in self.contributions.items()},
                "reasons": self.reasons, "rejections": self.rejections}


def drone_features(d: DroneState, ctx: SelectionContext) -> dict[str, float]:
    dist = distance(d.position, ctx.target)
    margin = d.battery_percentage - ctx.est_battery_pct
    matched = len(ctx.required_caps & d.capabilities)
    cap = 1.0 if not ctx.required_caps else matched / len(ctx.required_caps)
    return {
        "battery": clamp(d.battery_percentage / 100.0, 0, 1),
        "health": clamp(d.health_percentage / 100.0, 0, 1),
        "proximity": clamp(1.0 - dist / ctx.world_diagonal, 0, 1),
        "capability": cap,
        "reliability": clamp(d.reliability, 0, 1),
        "endurance": clamp(margin / 60.0, 0, 1),            # 60% spare battery == full marks
        "workload": 0.0 if d.drone_id in ctx.busy_drones or d.current_mission_id else 1.0,
    }


def env_risk_for(d: DroneState, ctx: SelectionContext, max_safe_temp: float = 75.0) -> float:
    link = 0.5 * (1.0 - d.signal_strength) + 0.3 * (1.0 - d.gps_quality)
    heat = 0.2 * clamp((d.temperature - 40.0) / (max_safe_temp - 40.0), 0, 1)
    maint = 0.3 * clamp(ctx.maintenance_risk.get(d.drone_id, 0.0), 0, 1)
    return clamp(link + heat + maint + ctx.env_risk * 0.5, 0, 1)


def _reasons(f: dict[str, float], d: DroneState, ctx: SelectionContext) -> list[str]:
    out = []
    if f["battery"] >= 0.8:
        out.append(f"high battery ({d.battery_percentage:.0f}%)")
    elif f["battery"] >= 0.6:
        out.append(f"adequate battery ({d.battery_percentage:.0f}%)")
    if f["health"] >= 0.9:
        out.append(f"high health ({d.health_percentage:.0f}%)")
    if f["proximity"] >= 0.8:
        out.append("close to mission area")
    if f["capability"] >= 1.0:
        out.append("compatible capabilities")
    if f["reliability"] >= 0.93:
        out.append(f"strong reliability history ({d.reliability:.2f})")
    if f["endurance"] >= 0.6:
        out.append("comfortable battery margin for the route")
    if f["workload"] >= 1.0:
        out.append("idle, no current workload")
    return out


def rank_drones(drones: list[DroneState], ctx: SelectionContext) -> list[Ranking]:
    """Rank all drones. Eligible ones first (descending score, then id); ineligible after."""
    rankings: list[Ranking] = []
    for d in drones:
        viol = assignment_violations(d, ctx.required_caps)
        f = drone_features(d, ctx)
        if viol:
            rankings.append(Ranking(d.drone_id, False, None, f, {}, [], viol))
            continue
        contrib = {k: WEIGHTS[k] * f[k] for k in WEIGHTS}
        risk = env_risk_for(d, ctx)
        contrib["risk"] = -RISK_PENALTY_WEIGHT * risk
        score = clamp(sum(contrib.values()), 0.0, 1.0)
        rankings.append(Ranking(d.drone_id, True, score, f, contrib, _reasons(f, d, ctx), []))
    rankings.sort(key=lambda r: (not r.eligible, -(r.score or 0.0), r.drone_id))
    return rankings


def explain(r: Ranking) -> str:
    """One-line human-readable explanation used in the decision log."""
    if not r.eligible:
        return f"{r.drone_id} rejected: " + "; ".join(r.rejections)
    parts = ", ".join(f"{k} {v:+.2f}" for k, v in r.contributions.items())
    why = "; ".join(r.reasons) or "meets constraints"
    return f"{r.drone_id} score {r.score:.2f} ({parts}) — {why}"
