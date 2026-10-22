import math
from dataclasses import dataclass


def front_obstacle_distance(ranges, angle_min, angle_increment, sector=math.pi/3):
    values = [r for i, r in enumerate(ranges)
              if math.isfinite(r) and r > 0 and
              abs(math.atan2(math.sin(angle_min+i*angle_increment),
                             math.cos(angle_min+i*angle_increment))) <= sector/2]
    return min(values, default=float('inf'))


@dataclass
class ControllerParams:
    follow_distance: float = 1.0
    stop_distance: float = 0.6
    obstacle_stop_distance: float = 0.65
    lost_timeout: float = 0.5
    max_linear: float = 0.5
    max_angular: float = 1.0
    max_linear_accel: float = 0.4
    max_angular_accel: float = 1.5
    kp_distance: float = 0.7
    kp_bearing: float = 1.5


class FollowController:
    def __init__(self, params=None):
        self.p = params or ControllerParams()
        if any(not math.isfinite(v) or v <= 0 for v in vars(self.p).values()):
            raise ValueError('Controller parameters must be positive and finite')
        if self.p.stop_distance >= self.p.follow_distance:
            raise ValueError('Stop distance must be below follow distance')
        self.v = self.w = 0.0

    def step(self, target, age, obstacle, dt, enabled=True):
        p = self.p
        state, reason = 'FOLLOWING', 'Tracking target'
        if not enabled:
            state, reason = 'IDLE', 'Disabled'
        elif target is None or not math.isfinite(age) or age < 0 or age > p.lost_timeout or not all(math.isfinite(t) for t in target):
            state, reason = 'LOST', 'Target missing or stale'
        elif math.hypot(*target) <= p.stop_distance:
            state, reason = 'TOO_CLOSE', 'Target inside stop distance'
        elif math.isnan(obstacle) or obstacle <= p.obstacle_stop_distance:
            state, reason = 'OBSTACLE_STOP', 'Obstacle or unavailable scan'
        if state != 'FOLLOWING':
            self.v = self.w = 0.0
            return self.v, self.w, state, reason
        distance, bearing = math.hypot(*target), math.atan2(target[1], target[0])
        v = min(p.max_linear, max(0.0, p.kp_distance*(distance-p.follow_distance))) * max(0.0, math.cos(bearing))
        w = max(-p.max_angular, min(p.max_angular, p.kp_bearing*bearing))
        dt = min(0.2, max(0.0, dt)) if math.isfinite(dt) else 0.0
        self.v += max(-p.max_linear_accel*dt, min(p.max_linear_accel*dt, v-self.v))
        self.w += max(-p.max_angular_accel*dt, min(p.max_angular_accel*dt, w-self.w))
        return self.v, self.w, state, reason
