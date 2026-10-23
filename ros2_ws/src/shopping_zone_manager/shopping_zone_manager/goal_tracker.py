"""ROS-free navigation lifecycle. Effects are executed by the ROS adapter."""
from dataclasses import dataclass
from enum import Enum
import math


class State(str, Enum):
    IDLE = 'IDLE'
    PENDING = 'PENDING'
    ACTIVE = 'ACTIVE'
    CANCELING = 'CANCELING'
    SUCCEEDED = 'SUCCEEDED'
    ABORTED = 'ABORTED'
    CANCELED = 'CANCELED'
    REJECTED = 'REJECTED'
    TIMEOUT = 'TIMEOUT'
    UNAVAILABLE = 'UNAVAILABLE'
    UNKNOWN_ZONE = 'UNKNOWN_ZONE'


TERMINAL = frozenset(s.value for s in State) - {'IDLE', 'PENDING', 'ACTIVE', 'CANCELING'}


@dataclass(frozen=True)
class Status:
    code: str
    text: str

    def __str__(self):
        return f'{self.code}: {self.text}'


@dataclass(frozen=True)
class Effect:
    kind: str
    generation: int
    zone: object = None
    status: Status = None


class GoalTracker:
    def __init__(self, response_timeout=5.0):
        if not math.isfinite(response_timeout) or response_timeout <= 0:
            raise ValueError('response_timeout must be positive and finite')
        self.response_timeout = response_timeout
        self.state = State.IDLE
        self.generation = 0
        self.zone = None
        self.outstanding = False
        self.accepted = False
        self.deadline = None
        self.queued = None
        self.status = Status('IDLE', 'No navigation goal')

    @property
    def busy(self):
        return self.outstanding

    def emit(self, code, text):
        self.status = Status(code, text)
        return [Effect('status', self.generation, status=self.status)]

    def request(self, zone, now=0.0, preempt=False, available=True):
        if zone is None:
            if not self.busy:
                self.state = State.UNKNOWN_ZONE
            return self.emit('UNKNOWN_ZONE', 'Unknown zone; use list_zones')
        if self.busy:
            if preempt and self.state != State.TIMEOUT:
                self.queued = zone
                return self.cancel_request(preserve_queue=True)
            return self.emit('BUSY', 'Current goal has not settled; cancel or wait')
        if not available:
            self.state = State.UNAVAILABLE
            return self.emit('UNAVAILABLE', 'NavigateToPose server unavailable')
        self.generation += 1
        self.zone = zone
        self.outstanding = True
        self.accepted = False
        self.deadline = now + self.response_timeout
        self.state = State.PENDING
        return self.emit('PENDING', f'Requested {zone.name}') + [Effect('send', self.generation, zone)]

    def response(self, generation, accepted):
        if generation != self.generation or not self.outstanding or self.accepted:
            return []
        self.deadline = None
        if not accepted:
            return self._finish(State.REJECTED)
        self.accepted = True
        if self.state in (State.TIMEOUT, State.CANCELING):
            return [Effect('cancel', generation)]
        self.state = State.ACTIVE
        return self.emit('ACTIVE', self.zone.name)

    def result(self, generation, status):
        if generation != self.generation or not self.outstanding or not self.accepted:
            return []
        outcome = {4: State.SUCCEEDED, 5: State.CANCELED, 6: State.ABORTED}.get(status)
        if outcome is None:
            # An unexpected result must not silently become success.
            outcome = State.ABORTED
        return self._finish(outcome)

    def _finish(self, outcome):
        self.outstanding = False
        self.deadline = None
        self.accepted = False
        if self.state == State.TIMEOUT:
            return self.emit('TIMEOUT', 'Late goal settled after response timeout')
        self.state = outcome
        effects = self.emit(outcome.value, self.zone.name)
        if self.queued is not None:
            effects.append(Effect('next', self.generation, self.queued))
            self.queued = None
        return effects

    def cancel_request(self, preserve_queue=False):
        if not preserve_queue:
            self.queued = None
        if not self.busy:
            return self.emit('IDLE', 'No navigation goal to cancel')
        if self.state == State.TIMEOUT:
            effects = self.emit('TIMEOUT', 'Awaiting late response or final cancellation result')
            if self.accepted:
                effects.append(Effect('cancel', self.generation))
            return effects
        if self.state == State.CANCELING:
            effects = self.emit('CANCELING', 'Cancellation requested again')
            if self.accepted:
                effects.append(Effect('cancel', self.generation))
            return effects
        self.state = State.CANCELING
        effects = self.emit('CANCELING', f'Canceling {self.zone.name}')
        if self.accepted:
            effects.append(Effect('cancel', self.generation))
        return effects

    def cancel_response(self, generation, accepted):
        if generation != self.generation or not self.busy:
            return []
        if accepted:
            return []  # Only the action result establishes CANCELED.
        self.queued = None
        if self.state != State.TIMEOUT:
            self.state = State.ACTIVE
        return self.emit('CANCEL_FAILED', 'Cancel not acknowledged; goal may still be active')

    def tick(self, now):
        if self.deadline is not None and now >= self.deadline:
            self.deadline = None
            self.queued = None
            self.state = State.TIMEOUT
            return self.emit('TIMEOUT', 'No goal response; late acceptance will be canceled')
        return []
