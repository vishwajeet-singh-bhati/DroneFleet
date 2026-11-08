"""Map domain errors to HTTP responses with a stable `code` field."""
from __future__ import annotations

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

from app.control.state_machine import InvalidModeTransition
from app.mission.models import InvalidTransition
from app.safety.validator import CommandRejected
from app.services.errors import ServiceError

_SERVICE_STATUS = {"NOT_FOUND": 404, "INVALID_STATE": 409, "PLANNING_FAILED": 422}
_REJECT_STATUS = {"UNKNOWN_SCENARIO": 404, "INVALID_INPUT": 422, "INVALID_PARAMS": 422, "INVALID_EVENT": 422}


def _body(code: str, message: str) -> dict:
    return {"error": {"code": code, "message": message}}


def install_error_handlers(app: FastAPI) -> None:
    @app.exception_handler(ServiceError)
    async def _service(_: Request, e: ServiceError):
        return JSONResponse(_body(e.code, e.message), status_code=_SERVICE_STATUS.get(e.code, 400))

    @app.exception_handler(CommandRejected)
    async def _rejected(_: Request, e: CommandRejected):
        return JSONResponse(_body(e.code, e.message), status_code=_REJECT_STATUS.get(e.code, 409))

    @app.exception_handler(InvalidTransition)
    async def _bad_transition(_: Request, e: InvalidTransition):
        return JSONResponse(_body("INVALID_STATE", str(e)), status_code=409)

    @app.exception_handler(InvalidModeTransition)
    async def _bad_mode(_: Request, e: InvalidModeTransition):
        return JSONResponse(_body("INVALID_STATE", str(e)), status_code=409)
