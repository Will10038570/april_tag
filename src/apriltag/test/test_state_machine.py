"""Transition table and goal rules of apriltag_control's TrackingState machine."""

import pytest

from apriltag.control_node import (
    ACTIVE_STATES,
    ALLOWED_TRANSITIONS,
    GOAL_RULES,
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

# (action, start) -> states in which the goal is accepted
EXPECTED_GOALS = {
    ('start_tracking', True): {S.IDLE},
    ('start_tracking', False): {S.STAGE1, S.STAGE2},
    ('leave_cs', True): {S.IDLE},
    ('leave_cs', False): {S.LEAVING},
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
    (S.STAGE1, S.LEAVING),    # leave_cs only starts from IDLE
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


@pytest.mark.parametrize('action, start', list(EXPECTED_GOALS))
@pytest.mark.parametrize('state', list(S))
def test_is_goal_accepted(action, start, state):
    assert is_goal_accepted(action, start, state) == (state in EXPECTED_GOALS[(action, start)])


def test_unknown_action_is_rejected():
    assert not is_goal_accepted('dock', True, S.IDLE)
