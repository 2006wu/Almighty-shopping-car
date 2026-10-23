"""ROS-free line-of-sight checks and direct/Nav2 ownership decisions."""
import math
from dataclasses import dataclass


def transform_xy(point, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x+c*point[0]-s*point[1], y+s*point[0]+c*point[1]


def valid_xy(target):
    return target is not None and len(target) == 2 and all(math.isfinite(v) for v in target)


def scan_valid(ranges, angle_min, angle_increment, range_min, range_max):
    """Reject malformed scans, including invalid readings outside the corridor."""
    return (len(ranges) > 0
            and all(math.isfinite(v) for v in (angle_min, angle_increment, range_min))
            and math.isfinite(angle_min+(len(ranges)-1)*angle_increment)
            and angle_increment != 0 and range_min >= 0
            and not math.isnan(range_max) and range_max > range_min
            and all(v == math.inf or (math.isfinite(v) and 0 <= v <= range_max)
                    for v in ranges))


def line_of_sight_clear(ranges, angle_min, angle_increment, target_xy,
                        robot_half_width, margin, range_min, range_max):
    """Check the target-width angular corridor in the scan frame.

    Rays must intersect the forward strip; finite returns are tested against
    its lateral boundary. The strip starts at the scan origin.
    Include the nearest bearing ray even when the strip is sub-resolution.
    All distances use the scan origin; the caller transforms the target.
    """
    if (not valid_xy(target_xy)
            or not all(math.isfinite(v) and v > 0 for v in (robot_half_width, margin))
            or not scan_valid(ranges, angle_min, angle_increment, range_min, range_max)):
        return False
    distance = math.hypot(*target_xy)
    if distance <= margin:
        return False
    bearing = math.atan2(target_xy[1], target_xy[0])
    errors = [abs(math.atan2(math.sin(angle_min+i*angle_increment-bearing),
                             math.cos(angle_min+i*angle_increment-bearing)))
              for i in range(len(ranges))]
    nearest = min(range(len(errors)), key=errors.__getitem__)
    if errors[nearest] > abs(angle_increment)/2+1e-9:
        return False
    for i, value in enumerate(ranges):
        error = errors[i]
        if error > math.pi/2 and i != nearest:
            continue
        if value < range_min:
            return False
        if value == math.inf or value >= distance-margin:
            continue
        if i == nearest or abs(value*math.sin(error)) <= robot_half_width:
            return False
    return True


@dataclass
class HandoffParams:
    follow_distance: float = 1.0
    obstacle_stop_distance: float = 0.65
    handoff_margin: float = 0.3
    handoff_delay: float = 1.0
    goal_update_distance: float = 0.4
    goal_update_period: float = 0.5
    clear_margin: float = 0.15
    resume_delay: float = 0.5
    retry_cooldown: float = 3.0


@dataclass(frozen=True)
class HandoffDecision:
    mode: str
    action: str
    direct_publish: bool
    reason: str


class HandoffSupervisor:
    def __init__(self, params=None, route_around=True):
        self.p = params or HandoffParams()
        if any(not math.isfinite(v) or v <= 0 for v in vars(self.p).values()):
            raise ValueError('Handoff parameters must be positive and finite')
        if not isinstance(route_around, bool):
            raise ValueError('route_around must be boolean')
        self.route_around = route_around
        self.mode = 'DIRECT'
        self.blocked_since = self.clear_since = self.last_goal = None
        self.last_goal_time = self.last_time = None
        self.retry_after = -math.inf

    def record_goal(self, now, map_target):
        """Confirm the actual submission position and time."""
        self.last_goal, self.last_goal_time = tuple(map_target), now

    def step(self, now, state, target, los_clear, front_clearance, nav_status='none',
             *, map_target=None, enabled=True, scan_valid=True):
        """Targets are base-frame safety coordinates and map-frame goal coordinates."""
        action, reason = 'none', 'Direct following'
        previous = self.mode
        clock_reset = (not math.isfinite(now)
                       or self.last_time is not None and now < self.last_time)
        self.last_time = now
        safe = (enabled and state not in ('LOST', 'TOO_CLOSE', 'IDLE')
                and valid_xy(target) and scan_valid
                and not math.isnan(front_clearance) and front_clearance >= 0)
        if not self.route_around or not safe or clock_reset:
            self.mode = 'DIRECT'
            self.blocked_since = self.clear_since = None
            if clock_reset:
                self.retry_after = -math.inf
            reason = 'Direct only or safety stop'
        elif self.mode == 'NAV2':
            close = math.hypot(*target) <= self.p.follow_distance+self.p.handoff_margin
            clear = los_clear and front_clearance > self.p.obstacle_stop_distance+self.p.clear_margin
            self.clear_since = (now if self.clear_since is None else self.clear_since) if clear else None
            if nav_status in ('failed', 'canceled') or not valid_xy(map_target):
                self.mode = 'DIRECT'
                self.retry_after = now+self.p.retry_cooldown
                reason = 'Navigation failed or map transform unavailable'
            elif (nav_status == 'succeeded' or los_clear and close
                  or self.clear_since is not None and now-self.clear_since >= self.p.resume_delay):
                self.mode = 'DIRECT'
                reason = 'Direct path restored or navigation succeeded'
            elif (math.dist(map_target, self.last_goal) > self.p.goal_update_distance
                  and now-self.last_goal_time >= self.p.goal_update_period):
                action, reason = 'update_goal', 'Person moved in map frame'
            else:
                reason = 'Navigation owns velocity'
        else:
            blocked = state == 'OBSTACLE_STOP' or not los_clear
            eligible = (blocked and valid_xy(map_target)
                        and math.hypot(*target) > self.p.follow_distance+self.p.handoff_margin
                        and now >= self.retry_after)
            self.blocked_since = (now if self.blocked_since is None else self.blocked_since) if eligible else None
            if self.blocked_since is not None and now-self.blocked_since >= self.p.handoff_delay:
                self.mode, action, reason = 'NAV2', 'send_goal', 'Persistent blocked path'
        if previous == 'NAV2' and self.mode == 'DIRECT':
            action = 'cancel_goal'
            self.blocked_since = self.clear_since = None
        if action in ('send_goal', 'update_goal'):
            self.record_goal(now, map_target)
        return HandoffDecision(self.mode, action, self.mode == 'DIRECT', reason)


class GoalBook:
    """Identify callbacks belonging to the latest submitted goal only."""
    def __init__(self):
        self.generation = 0

    def begin(self):
        self.generation += 1
        return self.generation

    def is_current(self, generation):
        return generation == self.generation


class VelocityDrain:
    """Wait for terminal confirmation, then quiet/zero foreign velocity.

    The two-second fallback starts at terminal confirmation; an unconfirmed
    running goal never releases ownership. Times use the node clock.
    """
    def __init__(self):
        self.active = False
        self.started = self.terminal_at = self.last_message = None
        self.zero = False

    def start(self, now):
        if not self.active:
            self.active = True
            self.started = now
            self.terminal_at = None

    def terminal(self, now):
        if self.active and self.terminal_at is None:
            self.terminal_at = now

    def observe(self, now, linear, angular):
        self.last_message = now
        self.zero = (math.isfinite(linear) and math.isfinite(angular)
                     and abs(linear) < 0.02 and abs(angular) < 0.05)

    def ready(self, now):
        if not self.active:
            return True
        if self.terminal_at is None or now < self.terminal_at:
            return False
        last = self.started if self.last_message is None else max(self.started, self.last_message)
        return self.zero or now-last >= 0.3 or now-self.terminal_at >= 2.0

    def reset(self):
        self.__init__()
