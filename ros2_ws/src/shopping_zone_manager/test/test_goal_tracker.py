"""Lifecycle tests require no ROS installation."""
from types import SimpleNamespace
import pytest
from shopping_zone_manager.goal_tracker import GoalTracker, State


A = SimpleNamespace(name='a')
B = SimpleNamespace(name='b')


def kinds(effects):
    return [effect.kind for effect in effects]


def active(tracker):
    tracker.request(A, now=10)
    tracker.response(tracker.generation, True)
    return tracker.generation


@pytest.mark.parametrize('status,expected', [(4, 'SUCCEEDED'), (5, 'CANCELED'),
                                             (6, 'ABORTED'), (0, 'ABORTED')])
def test_results(status, expected):
    tracker = GoalTracker()
    assert tracker.state == State.IDLE
    generation = active(tracker)
    assert tracker.state == State.ACTIVE
    effects = tracker.result(generation, status)
    assert tracker.state.value == expected
    assert effects[0].status.code == expected
    assert not tracker.busy
    assert kinds(tracker.request(B)) == ['status', 'send']


def test_rejection_unknown_unavailable():
    tracker = GoalTracker()
    assert tracker.request(None)[0].status.code == 'UNKNOWN_ZONE'
    assert tracker.state == State.UNKNOWN_ZONE
    assert tracker.request(A, available=False)[0].status.code == 'UNAVAILABLE'
    assert tracker.state == State.UNAVAILABLE
    tracker.request(A)
    assert tracker.state == State.PENDING
    assert tracker.response(1, False)[0].status.code == 'REJECTED'
    assert not tracker.busy


def test_timeout_late_acceptance_is_canceled_and_quarantined():
    tracker = GoalTracker()
    tracker.request(A, now=10)
    assert not tracker.tick(14.99)
    assert tracker.tick(15)[0].status.code == 'TIMEOUT'
    assert tracker.state == State.TIMEOUT and tracker.busy
    assert not tracker.tick(20)
    assert tracker.request(B)[0].status.code == 'BUSY'
    assert tracker.request(B, preempt=True)[0].status.code == 'BUSY'
    assert tracker.cancel_request()[0].status.code == 'TIMEOUT'
    assert kinds(tracker.response(1, True)) == ['cancel']
    assert tracker.cancel_response(1, True) == []
    assert tracker.result(1, 5)[0].status.code == 'TIMEOUT'
    assert not tracker.busy
    assert 'send' in kinds(tracker.request(B))


def test_timeout_late_rejection_and_stale_generations():
    tracker = GoalTracker()
    tracker.request(A)
    tracker.tick(5)
    tracker.response(1, False)
    assert not tracker.busy
    tracker.request(B)
    assert tracker.generation == 2
    assert tracker.response(1, True) == []
    assert tracker.result(1, 4) == []
    assert tracker.cancel_response(1, False) == []
    assert tracker.state == State.PENDING
    assert tracker.result(2, 4) == []  # Results cannot precede acceptance.
    tracker.response(2, True)
    assert tracker.response(2, False) == []
    assert tracker.tick(500) == []


@pytest.mark.parametrize('pending', [True, False])
def test_busy_and_cancel(pending):
    tracker = GoalTracker()
    tracker.request(A)
    if not pending:
        tracker.response(1, True)
    state = tracker.state
    assert tracker.request(B)[0].status.code == 'BUSY'
    assert tracker.state == state and tracker.zone == A
    assert tracker.request(None)[0].status.code == 'UNKNOWN_ZONE'
    assert tracker.state == state
    effects = tracker.cancel_request()
    assert tracker.state == State.CANCELING
    assert ('cancel' in kinds(effects)) == (not pending)
    assert kinds(tracker.cancel_request()) == (['status'] if pending else ['status', 'cancel'])
    if pending:
        assert kinds(tracker.response(1, True)) == ['cancel']
    assert tracker.cancel_response(1, True) == []
    assert tracker.state == State.CANCELING
    tracker.result(1, 5)
    assert tracker.state == State.CANCELED


@pytest.mark.parametrize('pending', [True, False])
@pytest.mark.parametrize('result', [4, 5, 6])
def test_preempt_waits_for_old_result(pending, result):
    tracker = GoalTracker()
    tracker.request(A)
    if not pending:
        tracker.response(1, True)
    effects = tracker.request(B, preempt=True)
    assert 'send' not in kinds(effects)
    assert tracker.queued == B
    if pending:
        tracker.response(1, True)
    assert tracker.cancel_response(1, True) == []
    effects = tracker.result(1, result)
    assert kinds(effects) == ['status', 'next']
    assert effects[-1].zone == B
    tracker.request(effects[-1].zone, now=20)
    assert tracker.generation == 2 and tracker.deadline == 25
    assert tracker.result(1, 5) == []


def test_preempt_pending_rejection_and_cancel_failure():
    tracker = GoalTracker()
    tracker.request(A)
    tracker.request(B, preempt=True)
    assert kinds(tracker.response(1, False)) == ['status', 'next']
    active(tracker)
    tracker.request(B, preempt=True)
    assert tracker.cancel_response(2, False)[0].status.code == 'CANCEL_FAILED'
    assert tracker.state == State.ACTIVE and tracker.busy and tracker.queued is None
    assert 'cancel' in kinds(tracker.cancel_request())


def test_pending_cancel_timeout_and_idle_noop():
    tracker = GoalTracker()
    assert tracker.cancel_request()[0].status.code == 'IDLE'
    assert tracker.state == State.IDLE and not tracker.busy
    tracker.request(A)
    tracker.cancel_request()
    assert tracker.tick(5)[0].status.code == 'TIMEOUT'
    assert kinds(tracker.response(1, True)) == ['cancel']
    assert tracker.cancel_response(1, False)[0].status.code == 'CANCEL_FAILED'
    assert tracker.state == State.TIMEOUT and tracker.busy
    tracker.result(1, 5)
    assert tracker.cancel_request()[0].status.code == 'IDLE'


@pytest.mark.parametrize('timeout', [0, -1, float('nan'), float('inf')])
def test_invalid_timeout(timeout):
    with pytest.raises(ValueError):
        GoalTracker(timeout)


def test_configurable_timeout_and_status_text():
    tracker = GoalTracker(2)
    effect = tracker.request(A, now=42)[0]
    assert str(effect.status) == 'PENDING: Requested a'
    assert tracker.tick(43) == []
    assert tracker.tick(44)[0].status.code == 'TIMEOUT'


def test_cancel_can_retry_after_lost_cancel_response_even_after_timeout():
    tracker = GoalTracker()
    tracker.request(A)
    tracker.tick(5)
    tracker.response(1, True)
    assert kinds(tracker.cancel_request()) == ['status', 'cancel']
    assert tracker.busy and tracker.state == State.TIMEOUT
    tracker.result(1, 5)
    assert tracker.cancel_request()[0].status.code == 'IDLE'


def test_explicit_cancel_discards_preempted_goal():
    tracker = GoalTracker()
    generation = active(tracker)
    tracker.request(B, preempt=True)
    assert tracker.queued == B
    tracker.cancel_request()
    assert tracker.queued is None
    assert kinds(tracker.result(generation, 5)) == ['status']
    assert not tracker.busy
