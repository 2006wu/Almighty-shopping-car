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
    follow_distance: float = 1.2
    follow_time_headway: float = 0.4
    braking_reaction: float = 0.25
    braking_deceleration: float = 1.2
    stop_distance: float = 0.7
    too_close_margin: float = 0.1
    stop_release_margin: float = 0.15
    stop_release_time: float = 0.3
    obstacle_stop_distance: float = 0.65
    obstacle_stop_rotate: bool = True
    lost_timeout: float = 0.5
    max_linear: float = 1.0
    constrained_max_linear: float = 0.5
    open_side_clearance: float = 0.5
    near_person_distance: float = 1.2
    near_person_max_linear: float = 0.35
    max_angular: float = 1.5
    max_linear_accel: float = 0.8
    max_angular_accel: float = 1.5
    kp_distance: float = 1.0
    kp_bearing: float = 1.5
    # Absolute target velocity feedforward; hysteresis remains optional.
    velocity_feedforward: float = 0.9
    distance_hysteresis_enter: float = 0.0
    distance_hysteresis_exit: float = 0.0
    rotate_to_target_min_angle: float = math.pi/4
    rotate_to_target_release_angle: float = 0.25
    los_uncertain_speed_factor: float = 0.6
    side_sector_min: float = math.pi/6
    side_sector_max: float = math.radians(100)
    side_slow_clearance: float = 0.35
    side_free_clearance: float = 0.75
    side_min_speed_factor: float = 0.4


class FollowController:
    def __init__(self, params=None):
        self.p = params or ControllerParams()
        nonnegative = {'follow_time_headway', 'velocity_feedforward', 'distance_hysteresis_enter',
                       'distance_hysteresis_exit', 'side_slow_clearance', 'side_sector_min',
                       'side_min_speed_factor', 'los_uncertain_speed_factor'}
        if not isinstance(self.p.obstacle_stop_rotate, bool):
            raise ValueError('obstacle_stop_rotate must be boolean')
        if any(not math.isfinite(v) or (v < 0 if k in nonnegative else v <= 0)
               for k, v in vars(self.p).items() if k != 'obstacle_stop_rotate'):
            raise ValueError('Controller parameters must be finite and within their positive/nonnegative bounds')
        if not 0 < self.p.rotate_to_target_release_angle < self.p.rotate_to_target_min_angle <= math.pi:
            raise ValueError('Invalid target alignment angles')
        if self.p.stop_distance+self.p.too_close_margin >= self.p.follow_distance:
            raise ValueError('Stop distance plus near margin must be below follow distance')
        side_speed_factor(0, self.p.side_slow_clearance, self.p.side_free_clearance,
                          self.p.side_min_speed_factor)
        if (self.p.los_uncertain_speed_factor > 1
                or not 0 <= self.p.side_sector_min < self.p.side_sector_max <= math.pi):
            raise ValueError('Invalid slowdown factor or side sectors')
        if not 0 <= self.p.distance_hysteresis_enter <= self.p.distance_hysteresis_exit:
            raise ValueError('Hysteresis requires enter <= exit')
        self.holding_distance = False
        self.v = self.w = 0.0
        self.near_latched = False
        self.near_clear_time = 0.0
        self.aligning = False

    def open_area(self, clearance=math.nan, scan_valid=True, los_clear=True):
        """Open area: valid scan, clear LOS and enough side clearance on both sides."""
        return (scan_valid and los_clear and not math.isnan(clearance)
                and clearance >= self.p.open_side_clearance)

    def linear_cap(self, obstacle, clearance=math.nan, scan_valid=True, los_clear=True):
        """Absolute caps remain authoritative regardless of guard slowdown state."""
        p = self.p
        cap = (p.max_linear if self.open_area(clearance, scan_valid, los_clear)
               else min(p.max_linear, p.constrained_max_linear))
        if obstacle < p.near_person_distance:
            cap = min(cap, p.near_person_max_linear)
        return cap

    def step(self, target, age, obstacle, dt, enabled=True, speed_factor=1.0,
             target_velocity=(0.0, 0.0), rotation_clear=False, velocity_age=0.0,
             feedforward_enabled=True, clearance=math.nan, scan_valid=True, los_clear=True):
        p = self.p
        valid_target = (target is not None and math.isfinite(age) and 0 <= age <= p.lost_timeout
                        and all(math.isfinite(t) for t in target))
        if not enabled or not valid_target:
            self.near_clear_time = 0.0
        elif math.hypot(*target) <= p.stop_distance+p.too_close_margin+1e-12:
            self.near_latched = True
            self.near_clear_time = 0.0
        elif self.near_latched:
            clear = (math.hypot(*target) > p.stop_distance+p.too_close_margin+p.stop_release_margin
                     and math.isfinite(dt) and dt > 0)
            self.near_clear_time = self.near_clear_time+min(dt, 0.2) if clear else 0.0
            if self.near_clear_time >= p.stop_release_time:
                self.near_latched = False
                self.near_clear_time = 0.0
        state, reason = 'FOLLOWING', 'Tracking target'
        if not enabled:
            state, reason = 'IDLE', 'Disabled'
        elif target is None or not math.isfinite(age) or age < 0 or age > p.lost_timeout or not all(math.isfinite(t) for t in target):
            state, reason = 'LOST', 'Target missing or stale'
        elif self.near_latched:
            state, reason = 'TOO_CLOSE', 'Target near; waiting for sustained release clearance'
        elif math.isnan(obstacle) or obstacle <= p.obstacle_stop_distance:
            state, reason = 'OBSTACLE_STOP', 'Obstacle or unavailable scan'
        if (state == 'OBSTACLE_STOP' and math.isfinite(obstacle)
                and (not math.isfinite(dt) or dt <= 0)):
            state, reason = 'IDLE', 'Invalid control interval'
        if state != 'FOLLOWING':
            self.holding_distance = False
            if (state == 'OBSTACLE_STOP' and p.obstacle_stop_rotate
                    and math.hypot(*target) > p.stop_distance+p.too_close_margin
                    and rotation_clear and math.isfinite(obstacle) and obstacle >= 0
                    and math.isfinite(dt) and dt > 0):
                self.v = 0.0
                w = max(-p.max_angular, min(p.max_angular,
                        p.kp_bearing*math.atan2(target[1], target[0])))
                limit = p.max_angular_accel*min(dt, 0.2)
                self.w += max(-limit, min(limit, w-self.w))
                return self.v, self.w, state, 'Forward blocked; rotating toward target'
            self.v = self.w = 0.0
            return self.v, self.w, state, reason
        if not math.isfinite(dt) or dt <= 0:
            self.v = self.w = 0.0
            return 0.0, 0.0, 'IDLE', 'Invalid control interval'
        distance, bearing = math.hypot(*target), math.atan2(target[1], target[0])
        # Velocity is absolute target velocity expressed in base_link.
        velocity_valid = (target_velocity is not None and len(target_velocity) == 2
                          and math.isfinite(velocity_age) and 0 <= velocity_age <= p.lost_timeout
                          and all(math.isfinite(v) for v in target_velocity))
        # Near shelves, crowds or with uncertain LOS drop the time headway so the
        # robot stays close enough not to lose the person around shelf corners.
        headway = p.follow_time_headway if self.open_area(clearance, scan_valid, los_clear) else 0.0
        estimated_forward = target_velocity[0] if velocity_valid else 0.0
        effective_distance = p.follow_distance+headway*max(0.0, estimated_forward)
        error = distance-effective_distance
        if abs(error) <= p.distance_hysteresis_enter:
            self.holding_distance = True
        elif abs(error) >= p.distance_hysteresis_exit:
            self.holding_distance = False
        correction = 0.0 if self.holding_distance else p.kp_distance*error
        forward = estimated_forward if feedforward_enabled and distance >= p.follow_distance else 0.0
        demand = max(0.0, p.velocity_feedforward*forward+correction)
        absolute_cap = self.linear_cap(obstacle, clearance, scan_valid, los_clear)
        cap = min(absolute_cap, p.max_linear*max(0.0, min(1.0, speed_factor)))
        heading = max(0.0, math.cos(bearing))
        v = min(p.max_linear, demand)*max(0.0, min(1.0, speed_factor))*heading
        if abs(bearing) >= p.rotate_to_target_min_angle:
            self.aligning = True
        elif abs(bearing) <= p.rotate_to_target_release_angle:
            self.aligning = False
        if self.aligning:
            v = 0.0
        # Invert reaction*v + v^2/(2*a) using the nearest measured clearance.
        # The downstream footprint guard remains authoritative for all geometry.
        available = max(0.0, min(distance, obstacle)-p.stop_distance)
        ar = p.braking_deceleration*p.braking_reaction
        braking_cap = math.sqrt(ar*ar+2*p.braking_deceleration*available)-ar
        v = min(v, braking_cap, cap)
        w = max(-p.max_angular, min(p.max_angular, p.kp_bearing*bearing))
        dt = min(0.2, max(0.0, dt)) if math.isfinite(dt) else 0.0
        limits = []
        if demand > cap:
            limits.append('speed cap')
        if demand > braking_cap:
            limits.append('braking clearance')
        if heading < 0.95:
            limits.append('heading')
        if abs(v-self.v) > p.max_linear_accel*dt:
            limits.append('linear acceleration')
        if abs(w-self.w) > p.max_angular_accel*dt:
            limits.append('angular acceleration')
        if self.holding_distance:
            limits.append('distance hold')
        if self.aligning:
            limits.append('Aligning with target before translating')
        reason = ', '.join(limits) or 'Tracking target'
        self.v += max(-p.max_linear_accel*dt, min(p.max_linear_accel*dt, v-self.v))
        self.w += max(-p.max_angular_accel*dt, min(p.max_angular_accel*dt, w-self.w))
        # Clearance reductions must cap even a ramp-down command immediately.
        self.v = min(self.v, braking_cap, cap)
        # Do not carry ramp-down motion into the person's requested separation.
        # Keep turning toward the person (as develop does) so the cart is not left
        # facing a shelf when the person walks past; the TOO_CLOSE latch at
        # stop_distance+too_close_margin still zeroes both axes.
        if distance < p.follow_distance:
            self.v = 0.0
        return self.v, self.w, state, reason


def rotation_scan_clear(ranges, angle_min, angle_increment, range_min, range_max,
                        base_in_scan, radius):
    """Require a full scan to clear a disk around base_link, including margin.

    Test the far intersection of each ray with the disk, rather than distances
    to return endpoints: an occluder inside the disk cannot establish free space.
    The scan origin must lie inside the disk. Missing/partial coverage fails shut.
    """
    from .follow_handoff import scan_valid

    if (not scan_valid(ranges, angle_min, angle_increment, range_min, range_max)
            or len(base_in_scan) != 3
            or not all(math.isfinite(v) for v in (*base_in_scan, radius))
            or radius <= math.hypot(*base_in_scan[:2])
            or abs(angle_increment) > 0.05
            or abs(angle_increment)*(len(ranges)-1) < 2*math.pi-abs(angle_increment)-1e-6):
        return False
    x, y = base_in_scan[:2]
    for i, value in enumerate(ranges):
        angle = angle_min+i*angle_increment
        projection = x*math.cos(angle)+y*math.sin(angle)
        cross = x*math.sin(angle)-y*math.cos(angle)
        boundary = projection+math.sqrt(max(0.0, radius*radius-cross*cross))
        if value < range_min or min(value, range_max) <= boundary:
            return False
    return True


class NavigationGuard:
    """Filter Nav2 commands without commanding an independent escape motion."""
    def __init__(self, params):
        self.p = params
        self.v = self.w = 0.0

    def step(self, command, obstacle, rotation_clear, dt, safe=True, *, near_latched=False):
        p = self.p
        if (near_latched or not safe or command is None or math.isnan(obstacle) or obstacle < 0
                or not all(math.isfinite(v) for v in (*command, dt)) or dt <= 0):
            self.v = self.w = 0.0
            return self.v, self.w
        dt = min(dt, 0.2)
        blocked = obstacle <= p.obstacle_stop_distance
        v = 0.0 if blocked else max(0.0, min(p.max_linear, command[0]))
        w = (0.0 if blocked and not rotation_clear else
             max(-p.max_angular, min(p.max_angular, command[1])))
        # Stops bypass ramps. In particular no residual forward velocity is
        # permitted at a shelf while a costmap-aware controller turns away.
        self.v = (0.0 if v == 0 else min(p.max_linear, max(0.0,
                  self.v+max(-p.max_linear_accel*dt, min(p.max_linear_accel*dt, v-self.v)))))
        self.w = (0.0 if w == 0 else
                  self.w+max(-p.max_angular_accel*dt, min(p.max_angular_accel*dt, w-self.w)))
        return self.v, self.w
