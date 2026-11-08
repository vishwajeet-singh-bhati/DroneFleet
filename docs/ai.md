# AI components

All of them are *advisory or translating*; none directly commands a drone.

## 1. Explainable drone selection (`ai/selector.py`)
Weighted linear score over normalised features:
`battery .22, health .20, proximity .16, capability .14, reliability .10, endurance .10, workload .08`,
minus a risk penalty (≤ 0.15) from maintenance risk and active faults. **Contributions sum exactly to the score**,
and every ranking returns human-readable `reasons`/`rejections`. Hard constraints (failed/offline, missing
capability, battery insufficient for the route, busy) make a drone *ineligible* regardless of score. Each decision is
stored in the AI decision log and shown in the UI. There is no learned model here by design: a transparent formula is
easier to audit than a black box for a safety-relevant choice.

## 2. Mission interpreter (`ai/interpreter.py`)
Text → structured `MissionCreate`. A deterministic regex parser always works offline ("inspect the north zone with 2
drones within 15 minutes, high priority"). If `ANTHROPIC_API_KEY` is set, the Anthropic Messages API is tried first;
**its output is untrusted**: zone ids whitelisted, numbers clamped, unknown keys dropped, then re-validated by the same
pydantic schema as `POST /missions`. Any LLM failure falls back to the local parser. The result is a *proposal*; the
mission still has to be planned and passes all safety checks. (The LLM path is implemented but only the local parser is
covered by automated tests.)

## 3. Predictive maintenance & anomaly detection (`ai/maintenance.py`, scikit-learn)
**Synthetic data only** (`ai/synthetic_data.py`, 9 telemetry features, seed 42, 6000 samples, 75/25 split).
| Model | Task | Held-out result (synthetic) |
|---|---|---|
| GradientBoostingRegressor | health estimate | MAE 2.8, R² 0.93 |
| GradientBoostingClassifier | anomaly probability | ROC-AUC 0.997, precision 1.00, recall 0.95 |
| IsolationForest (normals only) | unsupervised outlier | ROC-AUC 0.93; at a 3 % FPR threshold detects ≈ 56 % (actual FPR 2.7 %) |

These numbers describe separability of *generated* faults, not real-world performance. Output feeds (a) the UI
maintenance tab/alerts and (b) a small selector risk penalty. It cannot ground or command a drone. Models are cached in
`data/models/` (`MODEL_VERSION` invalidates the cache).

## 4. Computer vision
Not implemented (optional in the brief; not a core dependency).
