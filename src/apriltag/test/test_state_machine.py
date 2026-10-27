"""Transition table and goal rules of apriltag_control's TrackingState machine."""

import pytest

from apriltag.control_node import (
    ACTIVE_STATES,
    ALLOWED_TRANSITIONS,
    BODY_STATUS,
    CANCEL_RULES,
    GOAL_RULES,
    STATUS_STATE,
    Step,
    TrackingState as S,
    is_goal_accepted,
    is_transition_allowed,
)

EXPECTED = {
    S.IDLE: {S.STAGE1, S.LEAVING},
    S.STAGE1: {S.STAGE2, S.IDLE},
    S.STAGE2: {S.IDLE},
    S.LEAVING: {S.IDLE},
}

# cmd -> states in which it is accepted (stopping is data=false on the same service)
EXPECTED_GOALS = {
    'attach': {S.IDLE},
    'detach': {S.IDLE},
}
# cmd -> states in which data=false cancels it
EXPECTED_CANCELS = {
    'attach': {S.STAGE1, S.STAGE2},
    'detach': {S.LEAVING},
}


def test_table_matches_design():
    assert ALLOWED_TRANSITIONS == EXPECTED


def test_every_state_has_transitions():
    assert set(ALLOWED_TRANSITIONS) == set(S)


@pytest.mark.parametrize('src', list(S))
@pytest.mark.parametrize('dst', list(S))
def test_is_transition_allowed(src, dst):
    assert is_transition_allowed(src, dst) == (dst in EXPECTED[src])


@pytest.mark.parametrize('src, dst', [
    (S.IDLE, S.STAGE2),       # Stage 2 only follows Stage 1
    (S.IDLE, S.IDLE),
    (S.STAGE1, S.LEAVING),    # detach only starts from IDLE
    (S.STAGE2, S.STAGE1),
    (S.STAGE2, S.LEAVING),    # Stage 2 aligned goes back to IDLE first
    (S.LEAVING, S.STAGE1),
])
def test_rejected_transitions(src, dst):
    assert not is_transition_allowed(src, dst)


def test_active_states_and_steps():
    assert ACTIVE_STATES == {S.STAGE1, S.STAGE2, S.LEAVING}
    assert [s.value for s in Step] == ['preparing', 'running', 'ending']


def test_goal_rules_match_design():
    assert GOAL_RULES == EXPECTED_GOALS


@pytest.mark.parametrize('action', list(EXPECTED_GOALS))
@pytest.mark.parametrize('state', list(S))
def test_is_goal_accepted(action, state):
    assert is_goal_accepted(action, state) == (state in EXPECTED_GOALS[action])


def test_cancel_rules_match_design():
    assert CANCEL_RULES == EXPECTED_CANCELS


def test_each_active_state_cancelled_by_one_service():
    assert not CANCEL_RULES['attach'] & CANCEL_RULES['detach']
    assert CANCEL_RULES['attach'] | CANCEL_RULES['detach'] == ACTIVE_STATES


def test_unknown_cmd_is_rejected():
    assert not is_goal_accepted('start_tracking', S.IDLE)


def test_body_status_values():
    assert BODY_STATUS == {S.IDLE: 0, S.STAGE1: 1, S.LEAVING: 2}


def test_status_state_values():
    assert STATUS_STATE == {S.IDLE: 'IDLE', S.STAGE1: 'Attaching(STAGE1)',
                            S.STAGE2: 'Attaching(STAGE2)', S.LEAVING: 'Detaching'}
