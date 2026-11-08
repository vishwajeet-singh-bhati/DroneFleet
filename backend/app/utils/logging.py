"""Structured (JSON-line) logging for major operations."""
from __future__ import annotations

import json
import logging
import sys

EVENT_LOGGER = "dronefleet.events"


class JsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        payload = getattr(record, "structured", None)
        if payload is None:
            payload = {"message": record.getMessage()}
        payload = {"level": record.levelname, "logger": record.name, **payload}
        return json.dumps(payload, default=str, sort_keys=True)


def configure_logging(level: int = logging.INFO, stream=None) -> None:
    logger = logging.getLogger("dronefleet")
    if any(getattr(h, "_dronefleet", False) for h in logger.handlers):
        return
    handler = logging.StreamHandler(stream or sys.stdout)
    handler.setFormatter(JsonFormatter())
    handler._dronefleet = True  # type: ignore[attr-defined]
    logger.addHandler(handler)
    logger.setLevel(level)
    logger.propagate = False


def log_structured(logger_name: str, level: int, **fields) -> None:
    logging.getLogger(logger_name).log(level, fields.get("message", ""), extra={"structured": fields})
