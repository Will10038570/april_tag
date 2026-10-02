"""Transition table of apriltag_control's TrackingState machine."""

import pytest

from apriltag.control_node import (
    ALLOWED_TRANSITIONS,
    CONTROL_ACTIVE_STATES,
    STOPPABLE_STATES,
    TrackingState as S,
    is_transition_allowed,
)

EXPECTED = {
    S.IDLE: {S.STARTING},
    S.STARTING: {S.STAGE1, S.FINISHING},
    S.STAGE1: {S.STAGE2, S.FINISHING},
    S.STAGE2: {S.IN_POSITION, S.FINISHING},
    S.IN_POSITION: {S.LEAVING, S.FINISHING},
    S.LEAVING: {S.FINISHING},
    S.FINISHING: {S.IDLE},
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
    (S.IDLE, S.FINISHING),         # STARTING ends through FINISHING, IDLE never does
    (S.IDLE, S.STAGE1),            # must go through STARTING
    (S.STAGE2, S.IDLE),            # cleanup always goes through FINISHING
    (S.STAGE2, S.LEAVING),
    (S.FINISHING, S.IN_POSITION),  # Stage 2 success goes straight to IN_POSITION
    (S.IN_POSITION, S.IDLE),
    (S.LEAVING, S.IN_POSITION),
])
def test_rejected_transitions(src, dst):
    assert not is_transition_allowed(src, dst)


def test_control_and_stoppable_states():
    assert CONTROL_ACTIVE_STATES == {S.STAGE1, S.STAGE2, S.LEAVING}
    assert STOPPABLE_STATES == {S.STARTING, S.STAGE1, S.STAGE2, S.LEAVING}
