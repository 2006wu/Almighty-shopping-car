import math
from dataclasses import dataclass


def front_obstacle_distance(ranges, angle_min, angle_increment, sector=math.pi/3,
                            range_min=0.0, range_max=float('inf')):
    """Return front clearance; NaN means unavailable, +inf means clear.

    Zero and positive readings below the sensor minimum trigger a stop.
    Invalid readings never establish a clear sector by themselves.
    """
    if (not all(math.isfinite(v) for v in (angle_min, angle_increment, sector, range_min))
            or not 0 < sector <= 2*math.pi or range_min < 0
            or math.isnan(range_max) or range_max <= range_min):
        return float('nan')
    values = []
    for i, value in enumerate(ranges):
        angle = angle_min+i*angle_increment
        if not math.isfinite(angle):
            return float('nan')
        if abs(math.atan2(math.sin(angle), math.cos(angle))) > sector/2+1e-12:
            continue
        if value == float('inf'):
            values.append(value)
        elif math.isfinite(value) and 0 <= value <= range_max:
            values.append(0.0 if value < range_min else value)
    return min(values, default=float('nan'))


def side_clearance(ranges, angle_min, angle_increment, sector_min, sector_max):
    """Minimum lateral distance in symmetric side sectors (angles in radians)."""
    values = [math.inf if value == math.inf else value*abs(math.sin(angle_min+i*angle_increment))
              for i, value in enumerate(ranges)
              if sector_min <= abs(math.atan2(math.sin(angle_min+i*angle_increment),
                                               math.cos(angle_min+i*angle_increment))) <= sector_max
              and (value == math.inf or math.isfinite(value) and value >= 0)]
    return min(values, default=math.nan)


def side_speed_factor(clearance, slow=0.45, free=0.9, minimum=0.4):
    """Interpolate the speed cap; unavailable side coverage uses the slow cap."""
    if (not all(math.isfinite(v) for v in (slow, free, minimum))
            or not 0 <= slow < free or not 0 <= minimum <= 1):
        raise ValueError('Invalid side slowdown parameters')
    if math.isnan(clearance):
        return minimum
    return minimum+(1-minimum)*min(1.0, max(0.0, (clearance-slow)/(free-slow)))


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
    los_uncertain_speed_factor: float = 0.6
    side_sector_min: float = math.pi/6
    side_sector_max: float = math.radians(100)
    side_slow_clearance: float = 0.35
    side_free_clearance: float = 0.75
    side_min_speed_factor: float = 0.4


class FollowController:
    def __init__(self, params=None):
        self.p = params or ControllerParams()
        nonnegative = {'side_slow_clearance', 'side_sector_min',
                       'side_min_speed_factor', 'los_uncertain_speed_factor'}
        if any(not math.isfinite(v) or (v < 0 if k in nonnegative else v <= 0)
               for k, v in vars(self.p).items()):
            raise ValueError('Controller parameters must be finite and within their positive/nonnegative bounds')
        if self.p.stop_distance >= self.p.follow_distance:
            raise ValueError('Stop distance must be below follow distance')
        side_speed_factor(0, self.p.side_slow_clearance, self.p.side_free_clearance,
                          self.p.side_min_speed_factor)
        if (self.p.los_uncertain_speed_factor > 1
                or not 0 <= self.p.side_sector_min < self.p.side_sector_max <= math.pi):
            raise ValueError('Invalid slowdown factor or side sectors')
        self.v = self.w = 0.0

    def step(self, target, age, obstacle, dt, enabled=True, speed_factor=1.0):
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
        if state not in ('FOLLOWING', 'OBSTACLE_STOP') or math.isnan(obstacle):
            self.v = self.w = 0.0
            return self.v, self.w, state, reason
        if not math.isfinite(dt) or dt <= 0:
            self.v = self.w = 0.0
            return 0.0, 0.0, 'IDLE', 'Invalid control interval'
        distance, bearing = math.hypot(*target), math.atan2(target[1], target[0])
        v = (min(p.max_linear, max(0.0, p.kp_distance*(distance-p.follow_distance)))
             * max(0.0, min(1.0, speed_factor)) * max(0.0, math.cos(bearing)))
        if state == 'OBSTACLE_STOP':
            self.v = v = 0.0
            reason = 'Obstacle: translation stopped, turning toward target'
        w = max(-p.max_angular, min(p.max_angular, p.kp_bearing*bearing))
        dt = min(0.2, max(0.0, dt)) if math.isfinite(dt) else 0.0
        self.v += max(-p.max_linear_accel*dt, min(p.max_linear_accel*dt, v-self.v))
        self.w += max(-p.max_angular_accel*dt, min(p.max_angular_accel*dt, w-self.w))
        return self.v, self.w, state, reason
