"""FastAPI application factory."""
from __future__ import annotations

from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from app.api import routes_drones, routes_fleet, routes_missions, routes_simulation
from app.api.deps import AppState
from app.api.errors import install_error_handlers
from app.core.config import DEFAULT_SETTINGS, Settings
from app.database.repository import Repository
from app.services.runner import SimulationRunner
from app.utils.logging import configure_logging


def create_app(settings: Settings = DEFAULT_SETTINGS, database_url: str | None = None,
               autostart: bool = True, seed: int = 42, log: bool = True, ml_samples: int = 6000,
               model_dir: "Path | None" = None) -> FastAPI:
    if log:
        configure_logging()
    repo = Repository(url=database_url or settings.database_url)
    from app.api.extensions import MODEL_DIR, register_extensions
    state = AppState(settings=settings, repo=repo, ml_samples=ml_samples,
                     model_dir=model_dir if model_dir is not None else (MODEL_DIR if ml_samples == 6000 else None))
    register_extensions(state)
    state.reset(seed)
    state.runner = SimulationRunner(lambda: state.orch, state.manager)
    state.runner.running = autostart

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        state.runner.start_loop()
        yield
        await state.runner.stop_loop()
        repo.flush()

    app = FastAPI(
        title="AI DroneFleet Commander", version="1.0.0", lifespan=lifespan,
        description="Software-only autonomous drone fleet management with human-in-the-loop control. "
                    "All drone behaviour comes from a deterministic simulation engine; the backend is the "
                    "single source of truth and the frontend only sends validated commands.")
    app.state.dc = state
    app.add_middleware(CORSMiddleware, allow_origins=["http://localhost:5173", "http://127.0.0.1:5173"],
                       allow_methods=["*"], allow_headers=["*"])
    install_error_handlers(app)
    for r in (routes_fleet.router, routes_missions.router, routes_drones.router, routes_simulation.router):
        app.include_router(r)
    from app.api.extensions import include_extension_routers
    include_extension_routers(app)

    @app.websocket("/ws/fleet")
    async def ws_fleet(ws: WebSocket):
        await state.manager.connect(ws)
        try:
            await state.manager.send(ws, state.runner.hello_message())
            await state.manager.send(ws, state.runner.state_message())
            while True:
                msg = await ws.receive_text()      # clients only send keep-alive pings
                if msg == "ping":
                    await state.manager.send(ws, {"type": "pong"})
        except WebSocketDisconnect:
            pass
        finally:
            state.manager.disconnect(ws)

    return app

