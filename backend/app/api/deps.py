"""Application state and FastAPI dependencies."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from fastapi import Request

from app.core.config import DEFAULT_SETTINGS, Settings
from app.database.repository import Repository
from app.services.orchestrator import FleetOrchestrator
from app.services.runner import SimulationRunner
from app.websocket.manager import ConnectionManager


@dataclass
class AppState:
    settings: Settings
    repo: Repository
    manager: ConnectionManager = field(default_factory=ConnectionManager)
    orch: FleetOrchestrator = None  # type: ignore[assignment]
    runner: SimulationRunner = None  # type: ignore[assignment]
    # hooks run on every freshly built orchestrator (e.g. attach ML anomaly detection)
    orch_hooks: list[Callable[[FleetOrchestrator], None]] = field(default_factory=list)
    ml_samples: int = 6000
    model_dir: "Path | None" = None
    maintenance_models: object = None

    def build_orchestrator(self, seed: int | None = None, fleet_size: int | None = None) -> FleetOrchestrator:
        o = FleetOrchestrator(self.settings, seed=seed, repo=self.repo, fleet_size=fleet_size)
        for hook in self.orch_hooks:
            hook(o)
        return o

    def reset(self, seed: int = 42, fleet_size: int | None = None) -> FleetOrchestrator:
        self.repo.reset()
        self.orch = self.build_orchestrator(seed, fleet_size)
        if self.runner:
            self.runner.reset_cursors()
        return self.orch


def get_state(request: Request) -> AppState:
    return request.app.state.dc


def get_orch(request: Request) -> FleetOrchestrator:
    return request.app.state.dc.orch
