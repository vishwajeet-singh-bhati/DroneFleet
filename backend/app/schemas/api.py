"""Pydantic request schemas. Responses are plain dicts produced by the domain layer
(the same shapes are pushed over the WebSocket), documented through OpenAPI examples."""
from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

from app.core.enums import Capability


class MissionCreate(BaseModel):
    model_config = ConfigDict(json_schema_extra={"example": {
        "name": "Inspect Zone A", "zone_id": "ZONE_A", "num_drones": 2, "priority": 3,
        "deadline_s": 900, "required_capabilities": ["CAMERA"], "include_inspected": False}})
    name: str = Field(min_length=1, max_length=200)
    description: str = Field(default="", max_length=2000)
    objective: str = Field(default="inspection", max_length=64)
    priority: int = Field(default=3, ge=1, le=5)
    zone_id: str = Field(default="ZONE_A", max_length=64)
    deadline_s: float | None = Field(default=None, gt=0, le=86400)
    required_capabilities: list[Capability] = Field(default_factory=lambda: [Capability.CAMERA])
    num_drones: int = Field(default=1, ge=1, le=6)
    include_inspected: bool = False
    auto_start: bool = False


class InterpretRequest(BaseModel):
    model_config = ConfigDict(json_schema_extra={"example": {
        "text": "Inspect the north zone and prioritize uninspected areas. Complete within 15 minutes.",
        "create": False}})
    text: str = Field(min_length=3, max_length=600)
    create: bool = Field(default=False, description="If true, create the validated mission immediately")


class TakeControlRequest(BaseModel):
    operator: str = Field(default="operator", min_length=1, max_length=64)


class ManualInputRequest(BaseModel):
    """Operator axes, each nominally in [-1, 1] (the backend clamps and validates)."""
    model_config = ConfigDict(extra="forbid")
    forward: float = Field(default=0.0, allow_inf_nan=False)
    strafe: float = Field(default=0.0, allow_inf_nan=False)
    climb: float = Field(default=0.0, allow_inf_nan=False)
    yaw: float = Field(default=0.0, allow_inf_nan=False)


class InjectEventRequest(BaseModel):
    model_config = ConfigDict(json_schema_extra={"example": {
        "type": "LOW_BATTERY", "drone_id": "D01", "params": {"battery": 18}}})
    type: str
    drone_id: str | None = None
    params: dict[str, Any] = Field(default_factory=dict)


class SimControlRequest(BaseModel):
    action: Literal["start", "stop", "step", "speed"]
    speed: float | None = Field(default=None, gt=0, le=60)
    steps: int | None = Field(default=None, ge=1, le=20000)


class ResetRequest(BaseModel):
    seed: int = 42
    fleet_size: int | None = Field(default=None, ge=1, le=40)


class ScenarioRunRequest(BaseModel):
    seed: int = 42
    max_ticks: int = Field(default=12000, ge=1, le=100000)
