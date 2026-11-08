"""Explicit flight-mode state machine.

All mode changes requested by the control, safety and recovery layers go through
`FlightModeFSM.transition`, which validates against the table below and records the change.
(The physics engine itself may move a drone to IDLE after it has physically landed; that is a
consequence of physics, not a command, and is mirrored into the event log by the services.)

The human takeover handshake is modelled with an intermediate TAKEOVER_REQUEST phase:

    AUTONOMOUS --request--> TAKEOVER_REQUEST --granted--> MANUAL

The request is validated by the safety layer synchronously, so the intermediate phase is
logged but does not persist between ticks.
"""
from __future__ import annotations

from typing import Callable

from app.core.enums import FlightMode
from app.simulation.drone_state import DroneState

M = FlightMode

ALLOWED: dict[FlightMode, set[FlightMode]] = {
    M.IDLE: {M.TAKEOFF},
    M.TAKEOFF: {M.AUTONOMOUS, M.PAUSED, M.RETURN_TO_HOME, M.LANDING, M.EMERGENCY},
    M.AUTONOMOUS: {M.MANUAL, M.PAUSED, M.RETURN_TO_HOME, M.LANDING, M.EMERGENCY},
    M.MANUAL: {M.REJOINING, M.AUTONOMOUS, M.PAUSED, M.RETURN_TO_HOME, M.LANDING, M.EMERGENCY},
    M.PAUSED: {M.AUTONOMOUS, M.REJOINING, M.MANUAL, M.RETURN_TO_HOME, M.LANDING, M.EMERGENCY},
    M.REJOINING: {M.AUTONOMOUS, M.MANUAL, M.PAUSED, M.RETURN_TO_HOME, M.LANDING, M.EMERGENCY},
    M.RETURN_TO_HOME: {M.LANDING, M.MANUAL, M.PAUSED, M.EMERGENCY},
    M.LANDING: {M.IDLE, M.EMERGENCY},
    M.EMERGENCY: {M.IDLE},
}


class InvalidModeTransition(Exception):
    def __init__(self, drone_id: str, frm: FlightMode, to: FlightMode) -> None:
        super().__init__(f"{drone_id}: {frm.value} -> {to.value} is not a valid flight-mode transition")
        self.drone_id, self.frm, self.to = drone_id, frm, to


# callback(drone, from_mode, to_mode, reason, actor)
TransitionListener = Callable[[DroneState, FlightMode, FlightMode, str, str], None]


class FlightModeFSM:
    def __init__(self, set_mode: Callable[[str, FlightMode], None],
                 on_transition: TransitionListener | None = None) -> None:
        self._set_mode = set_mode
        self._on_transition = on_transition

    @staticmethod
    def can_transition(frm: FlightMode, to: FlightMode) -> bool:
        return to in ALLOWED.get(frm, set())

    def transition(self, drone: DroneState, to: FlightMode, reason: str, actor: str = "system") -> bool:
        """Return True if the mode changed, False for a duplicate (idempotent no-op).

        Raises InvalidModeTransition for illegal changes.
        """
        frm = drone.flight_mode
        if frm == to:
            return False
        if not self.can_transition(frm, to):
            raise InvalidModeTransition(drone.drone_id, frm, to)
        self._set_mode(drone.drone_id, to)
        if self._on_transition:
            self._on_transition(drone, frm, to, reason, actor)
        return True
