"""ROS-free mode transitions, velocity arbitration and output expiry."""
from dataclasses import dataclass
import math


ZERO = (0.0, 0.0)
ALLOWED = {'IDLE': (), 'GUIDE': ('nav',), 'FOLLOW': ('follow', 'nav'),
           'TELEOP': ('teleop',), 'ESTOP': ()}


class ModeManager:
    def __init__(self):
        self.state = 'IDLE'
        self.pending = None
        self.zero_pending = True

    def request(self, request):
        target = request.upper()
        if target == 'RESET':
            target = 'IDLE'
        elif self.state == 'ESTOP':
            return ()
        if target not in ALLOWED:
            raise ValueError('Unknown mode')
        effects = []
        if self.state in ('GUIDE', 'FOLLOW') or self.pending in ('GUIDE', 'FOLLOW'):
            effects.append('cancel_nav_goals')
        effects.append('disable_follow')
        self.pending = None if target == 'ESTOP' else target
        self.state = 'ESTOP' if target == 'ESTOP' else 'IDLE'
        self.zero_pending = True
        return tuple(effects)

    def step(self, effects_complete=True):
        # Every request requires a complete stopped output tick before activation.
        if self.zero_pending:
            self.zero_pending = False
            return ()
        if self.pending is None or not effects_complete:
            return ()
        self.state, self.pending = self.pending, None
        return ('enable_follow',) if self.state == 'FOLLOW' else ()


@dataclass(frozen=True)
class MuxResult:
    velocity: tuple
    source: str
    invalid: bool = False


class VelocityMux:
    def __init__(self, timeouts=None, priorities=None, max_linear=0.6,
                 max_angular=1.2, zero_period=0.2, allow_reverse=False):
        self.timeouts = dict(timeouts or {'safety_stop': 0.5, 'teleop': 0.5,
                                        'follow': 0.3, 'nav': 0.5})
        self.priorities = dict(priorities or {'safety_stop': 100, 'teleop': 30,
                                            'follow': 20, 'nav': 10})
        if set(self.timeouts) != set(self.priorities):
            raise ValueError('Every input needs a timeout and priority')
        if any(not math.isfinite(v) or v <= 0 for v in
               [*self.timeouts.values(), max_linear, max_angular]):
            raise ValueError('Limits and timeouts must be positive and finite')
        if zero_period is not None and (not math.isfinite(zero_period) or zero_period <= 0):
            raise ValueError('zero_period must be positive or None')
        self.max_linear, self.max_angular = max_linear, max_angular
        self.allow_reverse = allow_reverse
        self.zero_period = zero_period
        self.inputs = {}
        self.last_zero = None

    def clear(self):
        self.inputs.clear()
        self.last_zero = None

    def receive(self, source, linear, angular, now):
        if source not in self.timeouts or not math.isfinite(now):
            raise ValueError('Unknown source or invalid timestamp')
        invalid = not all(math.isfinite(v) for v in (linear, angular))
        velocity = ZERO if invalid or source == 'safety_stop' else (
            max(-self.max_linear if self.allow_reverse else 0.0, min(self.max_linear, linear)),
            max(-self.max_angular, min(self.max_angular, angular)))
        self.inputs[source] = (now, velocity, invalid)

    def select(self, mode, now):
        allowed = ALLOWED[mode]
        fresh = [source for source, (stamp, _, _) in self.inputs.items()
                 if (source in allowed or source == 'safety_stop')
                 and 0 <= now-stamp < self.timeouts[source]]
        if mode in ('IDLE', 'ESTOP') or not fresh:
            return MuxResult(ZERO, mode.lower() if mode in ('IDLE', 'ESTOP') else 'none')
        source = max(fresh, key=lambda key: self.priorities[key])
        _, velocity, invalid = self.inputs[source]
        return MuxResult(velocity, source, invalid)

    def output(self, mode, now):
        result = self.select(mode, now)
        if result.velocity != ZERO:
            self.last_zero = None
            return result
        if self.last_zero is None or (self.zero_period is not None
                                      and now-self.last_zero >= self.zero_period):
            self.last_zero = now
            return result
        return None


class OutputWatchdog:
    def __init__(self, timeout=0.25):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Watchdog timeout must be positive and finite')
        self.timeout = timeout
        self.stamp = None
        self.velocity = ZERO

    def receive(self, velocity, now):
        self.velocity = velocity if all(math.isfinite(v) for v in velocity) else ZERO
        self.stamp = now

    def output(self, now):
        if self.stamp is None or not 0 <= now-self.stamp < self.timeout:
            return ZERO
        return self.velocity


class VelocityLimiter:
    """Bound normal acceleration; safety zeros bypass the ramp."""
    def __init__(self, linear_acceleration=0.6, angular_acceleration=1.2):
        self.limits = (linear_acceleration, angular_acceleration)
        if any(not math.isfinite(v) or v <= 0 for v in self.limits):
            raise ValueError('Acceleration limits must be positive and finite')
        self.velocity = ZERO
        self.stamp = None

    def step(self, velocity, now):
        dt = 0.0 if self.stamp is None else now-self.stamp
        self.stamp = now
        if velocity == ZERO or not math.isfinite(dt) or dt <= 0 or dt > 0.2:
            self.velocity = ZERO
        else:
            self.velocity = tuple(0.0 if new == 0.0 else old+max(-limit*dt, min(limit*dt, new-old))
                                  for old, new, limit in zip(self.velocity, velocity, self.limits))
        return self.velocity


class SafetyVelocityLimiter(VelocityLimiter):
    """Ramp recovery with one scale; never exceed a safety-filtered command.

    Reductions apply immediately. A common factor preserves the guard's tested
    curvature, including when either component changes sign.
    """
    def step(self, velocity, now):
        dt = 0.0 if self.stamp is None else now-self.stamp
        self.stamp = now
        if velocity == ZERO or not math.isfinite(dt) or dt <= 0 or dt > 0.2:
            self.velocity = ZERO
        else:
            scale = 1.0
            for old, new, limit in zip(self.velocity, velocity, self.limits):
                if new:
                    previous = abs(old) if old*new > 0 else 0.0
                    scale = min(scale, (previous+limit*dt)/abs(new))
            self.velocity = tuple(v*scale for v in velocity)
        return self.velocity
