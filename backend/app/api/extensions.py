"""Optional feature routers/hooks (AI interpreter, maintenance ML, scenarios).
Kept separate so the core API still works if an optional component fails to load."""
from __future__ import annotations

import logging
from pathlib import Path

from fastapi import FastAPI

from app.api.deps import AppState

log = logging.getLogger("dronefleet")
MODEL_DIR = Path(__file__).resolve().parents[3] / "data" / "models"


def register_extensions(state: AppState) -> None:
    """Attach ML anomaly detection to every orchestrator built by `state` (if models are available)."""
    try:
        from app.ai.maintenance import get_models
        from app.services.anomaly_service import AnomalyService
        models = get_models(state.model_dir, n=state.ml_samples)
        state.maintenance_models = models
        state.orch_hooks.append(lambda o: o.extensions.append(AnomalyService(models)))
    except Exception as e:                                  # ML is optional: core system must still run
        log.warning("maintenance ML disabled: %s", e)
        state.maintenance_models = None


def include_extension_routers(app: FastAPI) -> None:
    from app.ai.interpreter import MissionInterpreter
    from app.api import routes_ai
    app.state.interpreter = MissionInterpreter.from_env()
    app.state.maintenance_models = app.state.dc.maintenance_models
    app.include_router(routes_ai.router)
