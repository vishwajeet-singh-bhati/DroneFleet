import pytest

from app.core.enums import MissionStatus as S
from app.mission.models import ALLOWED_TRANSITIONS, InvalidTransition, Mission


def m():
    return Mission(mission_id="M1", name="Inspect Zone A")


def walk(mission, *states):
    for i, s in enumerate(states):
        mission.transition(s, sim_time=float(i))


def test_happy_path():
    x = m()
    walk(x, S.PLANNING, S.READY, S.ACTIVE, S.COMPLETED)
    assert x.status == S.COMPLETED and x.is_terminal
    assert x.started_at == 2.0 and x.completed_at == 3.0
    assert [h["to"] for h in x.history] == ["PLANNING", "READY", "ACTIVE", "COMPLETED"]


def test_pause_resume_and_replanning_loop():
    x = m()
    walk(x, S.PLANNING, S.READY, S.ACTIVE, S.PAUSED, S.ACTIVE, S.REPLANNING, S.ACTIVE)
    assert x.status == S.ACTIVE


def test_invalid_transitions_rejected():
    x = m()
    with pytest.raises(InvalidTransition):
        x.transition(S.ACTIVE, 0)                   # CREATED -> ACTIVE
    walk(x, S.PLANNING, S.READY, S.ACTIVE, S.COMPLETED)
    for s in S:
        if s != S.COMPLETED:
            with pytest.raises(InvalidTransition):
                x.transition(s, 9)                  # terminal state is final


def test_duplicate_transition_rejected():
    x = m()
    x.transition(S.PLANNING, 0)
    with pytest.raises(InvalidTransition):
        x.transition(S.PLANNING, 1)


def test_failed_records_reason_and_abort_possible_from_active():
    x = m()
    walk(x, S.PLANNING, S.READY, S.ACTIVE)
    x.transition(S.FAILED, 5, reason="no drones left")
    assert x.failure_reason == "no drones left"
    y = m()
    walk(y, S.PLANNING, S.READY, S.ACTIVE, S.ABORTED)
    assert y.status == S.ABORTED


def test_started_at_set_once():
    x = m()
    walk(x, S.PLANNING, S.READY, S.ACTIVE)
    first = x.started_at
    x.transition(S.PAUSED, 10)
    x.transition(S.ACTIVE, 20)
    assert x.started_at == first


def test_deadline_at():
    x = m()
    x.deadline_s = 900
    assert x.deadline_at() is None
    walk(x, S.PLANNING, S.READY, S.ACTIVE)
    assert x.deadline_at() == x.started_at + 900


def test_every_state_has_transition_entry():
    assert set(ALLOWED_TRANSITIONS) == set(S)
