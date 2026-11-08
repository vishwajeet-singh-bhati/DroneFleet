"""WebSocket connection manager. Broadcasts server state to every connected client."""
from __future__ import annotations

import asyncio
import json

from fastapi import WebSocket


class ConnectionManager:
    def __init__(self) -> None:
        self.connections: set[WebSocket] = set()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self.connections.add(ws)

    def disconnect(self, ws: WebSocket) -> None:
        self.connections.discard(ws)

    async def send(self, ws: WebSocket, message: dict) -> bool:
        try:
            await ws.send_text(json.dumps(message, default=str))
            return True
        except Exception:
            self.disconnect(ws)
            return False

    async def broadcast(self, message: dict) -> None:
        if not self.connections:
            return
        payload = json.dumps(message, default=str)
        dead = []
        results = await asyncio.gather(*[self._safe_send(ws, payload) for ws in list(self.connections)])
        for ws, ok in zip(list(self.connections), results):
            if not ok:
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)

    @staticmethod
    async def _safe_send(ws: WebSocket, payload: str) -> bool:
        try:
            await ws.send_text(payload)
            return True
        except Exception:
            return False
