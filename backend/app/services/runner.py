"""Real-time simulation runner: advances the orchestrator on a wall-clock schedule and
broadcasts state over WebSocket. Everything runs on the asyncio event loop thread, so request
handlers (which are async) never race with the tick loop."""
from __future__ import annotations

import asyncio
import contextlib
import time
from typing import Callable

from app.services.orchestrator import FleetOrchestrator
from app.websocket.manager import ConnectionManager

BROADCAST_INTERVAL_S = 0.1
MAX_TICKS_PER_CYCLE = 400


class SimulationRunner:
    def __init__(self, get_orch: Callable[[], FleetOrchestrator], manager: ConnectionManager) -> None:
        self.get_orch = get_orch
        self.manager = manager
        self.running = False
        self.speed = 1.0
        self._task: asyncio.Task | None = None
        self._last_event_id = 0
        self._last_decision_id = 0
        self._last_world_version = -1

    # ------------------------------------------------------------------ lifecycle
    def start_loop(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.get_running_loop().create_task(self._loop())

    async def stop_loop(self) -> None:
        if self._task:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None

    def status(self) -> dict:
        o = self.get_orch()
        return {"running": self.running, "speed": self.speed, "sim_time": round(o.now, 3),
                "tick": o.engine.tick_count, "dt": o.settings.sim.dt, "seed": o.engine.seed}

    def reset_cursors(self) -> None:
        self._last_event_id = self._last_decision_id = 0
        self._last_world_version = -1

    # ------------------------------------------------------------------ loop
    async def _loop(self) -> None:
        last = time.monotonic()
        acc = 0.0
        while True:
            await asyncio.sleep(BROADCAST_INTERVAL_S)
            now = time.monotonic()
            wall, last = now - last, now
            o = self.get_orch()
            if self.running:
                acc += wall * self.speed
                n = 0
                while acc >= o.settings.sim.dt and n < MAX_TICKS_PER_CYCLE:
                    o.tick()
                    acc -= o.settings.sim.dt
                    n += 1
                if n == MAX_TICKS_PER_CYCLE:
                    acc = 0.0
            await self.broadcast()

    async def broadcast(self) -> None:
        if not self.manager.connections:
            return
        o = self.get_orch()
        await self.manager.broadcast(self.state_message())
        events = o.ctx.events.recent(500, since_id=self._last_event_id)
        if events:
            self._last_event_id = events[-1]["id"]
            await self.manager.broadcast({"type": "events", "data": events})
        decisions = [d for d in o.ctx.decisions.all() if d["id"] > self._last_decision_id]
        if decisions:
            self._last_decision_id = decisions[-1]["id"]
            await self.manager.broadcast({"type": "decisions", "data": decisions})
        if o.ctx.grid.version != self._last_world_version:
            self._last_world_version = o.ctx.grid.version
            await self.manager.broadcast({"type": "world", "data": o.world()})

    def state_message(self) -> dict:
        o = self.get_orch()
        return {"type": "state", "data": {**o.snapshot(), "running": self.running, "speed": self.speed}}

    def hello_message(self) -> dict:
        o = self.get_orch()
        return {"type": "hello", "data": {
            "world": o.world(), "events": o.ctx.events.recent(200),
            "decisions": o.ctx.decisions.recent(100), "status": self.status()}}
