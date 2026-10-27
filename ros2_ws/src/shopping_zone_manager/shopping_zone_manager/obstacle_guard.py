"""ROS-free conservative swept-footprint velocity guard (metres and seconds)."""
from dataclasses import dataclass
import math

import yaml


@dataclass(frozen=True)
class GuardResult:
    velocity: tuple
    state: str
    scale: float
    clearance: float


def load_nav_footprint(path):
    """Read the installed Nav2 footprints, requiring the supported centred box."""
    with open(path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    footprints = []
    for name in ('local_costmap', 'global_costmap'):
        value = config[name][name]['ros__parameters']['footprint']
        points = yaml.safe_load(value) if isinstance(value, str) else value
        if (not isinstance(points, list) or len(points) != 4
                or any(not isinstance(p, list) or len(p) != 2 for p in points)
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for p in points for v in p)):
            raise ValueError('Expected a finite four-corner footprint')
        corners = set(map(tuple, points))
        hx = max(abs(x) for x, y in points)
        hy = max(abs(y) for x, y in points)
        if hx <= 0 or hy <= 0 or corners != {(x, y) for x in (-hx, hx) for y in (-hy, hy)}:
            raise ValueError('Only a centred rectangular footprint is supported')
        footprints.append((hx, hy))
    if footprints[0] != footprints[1]:
        raise ValueError('Local and global footprints must agree')
    return footprints[0]


def scan_fresh(now, stamp, wall, received, timeout=0.5):
    """Check both acquisition time and wall-clock delivery, rejecting resets."""
    return (received is not None
            and all(math.isfinite(v) for v in (now, stamp, wall, received, timeout))
            and timeout > 0 and 0 <= now-stamp < timeout
            and 0 <= wall-received < timeout)


def scan_points(ranges, angle_min, increment, range_min, range_max, pose):
    """Require full-circle coverage; transform planar laser returns to base."""
    if (not ranges or not all(math.isfinite(v) for v in
            (angle_min, increment, range_min, range_max, *pose))
            or range_min < 0 or range_max <= range_min or increment == 0
            or abs(increment)*len(ranges) < 2*math.pi-2*abs(increment)
            or abs(increment) > 0.05):
        raise ValueError('Invalid or incomplete scan')
    points = []
    x, y, yaw = pose
    for i, r in enumerate(ranges):
        if r == math.inf:
            continue
        if not math.isfinite(r) or not range_min <= r <= range_max:
            raise ValueError('Invalid laser return')
        a = angle_min+i*increment+yaw
        points.append((x+r*math.cos(a), y+r*math.sin(a)))
    return points


def braking_travel(speed, time, reaction, deceleration):
    """Signed travel: reaction at initial speed, then constant braking to rest."""
    braking_time = min(max(0.0, time-reaction), abs(speed)/deceleration)
    travel = abs(speed)*min(time, reaction)
    travel += abs(speed)*braking_time-deceleration*braking_time**2/2
    return math.copysign(travel, speed)


class ObstacleGuard:
    def __init__(self, half_length=0.30, half_width=0.25, margin=0.10,
                 reaction=0.25, deceleration=1.2, angular_deceleration=1.2,
                 slow_extra=0.50, hysteresis=0.05, clear_time=0.30):
        values = locals().copy()
        values.pop('self')
        if any(not math.isfinite(v) or v <= 0 for v in values.values()):
            raise ValueError('Guard parameters must be positive and finite')
        self.__dict__.update(values)
        self.scale = 0.0
        self.clear_since = self.last_time = None

    def clearance(self, points, velocity):
        v, w = velocity
        # Full rotation disk safely bounds all intermediate skid-steer headings.
        if abs(v) < 1e-6 and abs(w) > 1e-6:
            radius = math.hypot(self.half_length, self.half_width)
            return min((math.hypot(x, y)-radius for x, y in points), default=math.inf)
        if abs(w) < 1e-6 and abs(v) > 1e-6:
            # Ignore returns behind travel unless they touch the safety footprint.
            points = [(x, y) for x, y in points
                      if x*v >= 0 or abs(x) <= self.half_length+self.margin]
        horizon = self.reaction+max(abs(v)/self.deceleration,
                                   abs(w)/self.angular_deceleration)
        # 1.2 m/s^2 is below Fortress DiffDrive's 2.0 m/s^2 braking limit.
        # Real hardware must re-measure braking before selecting this value.
        # Integrate linear and angular braking independently, including reaction.
        steps = max(1, math.ceil(horizon/0.025))
        dt = horizon/steps
        radius = math.hypot(self.half_length, self.half_width)
        padding = (abs(v)+abs(w)*radius)*dt/2
        minimum = math.inf
        px = py = integration_padding = 0.0
        previous_travel = previous_yaw = 0.0
        for i in range(steps+1):
            t = i*dt
            yaw = braking_travel(w, t, self.reaction, self.angular_deceleration)
            travel = braking_travel(v, t, self.reaction, self.deceleration)
            delta = travel-previous_travel
            midpoint_yaw = (yaw+previous_yaw)/2
            px += delta*math.cos(midpoint_yaw)
            py += delta*math.sin(midpoint_yaw)
            # Bound integration error using the full interval's heading range.
            # Combined with sampling padding this encloses the continuous sweep.
            integration_padding += abs(delta)*abs(yaw-previous_yaw)/2
            previous_travel, previous_yaw = travel, yaw
            c, s = math.cos(yaw), math.sin(yaw)
            for x, y in points:
                dx, dy = x-px, y-py
                distance = math.hypot(max(abs(c*dx+s*dy)-self.half_length, 0),
                                      max(abs(-s*dx+c*dy)-self.half_width, 0))
                minimum = min(minimum, distance-padding-integration_padding)
        return minimum

    def step(self, points, velocity, now, fresh=True):
        if (not fresh or points is None or not math.isfinite(now)
                or not all(math.isfinite(v) for v in velocity)
                or self.last_time is not None and now < self.last_time):
            self.scale, self.clear_since, self.last_time = 0.0, None, now
            return GuardResult((0.0, 0.0), 'STALE', 0.0, math.nan)
        self.last_time = now
        clearance = self.clearance(points, velocity)
        rotation = abs(velocity[0]) < 1e-6 and abs(velocity[1]) > 1e-6
        # A clear full rotation disk bounds every heading at any allowed angular
        # speed. Scaling an in-place turn can strand a skid-steer below friction.
        raw = (float(clearance > self.margin) if rotation else
               max(0.0, min(1.0, (clearance-self.margin)/self.slow_extra)))
        if raw < self.scale:
            self.scale, self.clear_since = raw, None
        elif raw > self.scale:
            release_margin = self.margin+self.hysteresis
            if not rotation:
                release_margin += self.scale*self.slow_extra
            wide = clearance > release_margin
            self.clear_since = (now if self.clear_since is None else self.clear_since) if wide else None
            if self.clear_since is not None and now-self.clear_since >= self.clear_time:
                self.scale, self.clear_since = raw, None
        else:
            self.clear_since = None
        state = 'STOP' if self.scale == 0 else 'SLOW' if self.scale < 1 else 'CLEAR'
        return GuardResult(tuple(v*self.scale for v in velocity), state, self.scale, clearance)
