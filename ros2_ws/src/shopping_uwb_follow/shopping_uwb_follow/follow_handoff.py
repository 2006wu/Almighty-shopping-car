"""ROS-free line-of-sight checks and direct/Nav2 ownership decisions."""
import math
from dataclasses import dataclass


def transform_xy(point, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x+c*point[0]-s*point[1], y+s*point[0]+c*point[1]


def standoff_target(target, follow_distance):
    """Place the navigation endpoint before the person, never behind the robot."""
    if (not valid_xy(target) or not math.isfinite(follow_distance)
            or follow_distance <= 0):
        raise ValueError('Standoff requires a finite target and positive distance')
    distance = math.hypot(*target)
    scale = max(0.0, 1-follow_distance/distance) if distance else 0.0
    return target[0]*scale, target[1]*scale


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
                        robot_half_width, margin, range_min, range_max, *,
                        follow_distance=None):
    """Check the target-width angular corridor in the scan frame.

    Rays must intersect the forward strip; finite returns are tested against
    its lateral boundary. The strip starts at the scan origin.
    Include the nearest bearing ray even when the strip is sub-resolution.
    All distances use the scan origin; the caller transforms the target.
    With follow_distance, stop at distance - max(margin, follow_distance).
    At or inside follow_distance, retain the distance - margin check so nearby
    returns still block. Omitting follow_distance preserves the original check.
    """
    if (not valid_xy(target_xy)
            or not all(math.isfinite(v) and v > 0 for v in (robot_half_width, margin))
            or follow_distance is not None
            and (not math.isfinite(follow_distance) or follow_distance <= 0)
            or not scan_valid(ranges, angle_min, angle_increment, range_min, range_max)):
        return False
    distance = math.hypot(*target_xy)
    if distance <= margin:
        return False
    clear_length = distance-margin
    if follow_distance is not None and distance > follow_distance:
        clear_length = distance-max(margin, follow_distance)
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
        if value == math.inf or value >= clear_length:
            continue
        if i == nearest or abs(value*math.sin(error)) <= robot_half_width:
            return False
    return True


class LosDebouncer:
    """Latch blockage after persistence; release only on a wider clear corridor."""
    def __init__(self, block_time=0.3, clear_time=0.6):
        if any(not math.isfinite(v) or v < 0 for v in (block_time, clear_time)):
            raise ValueError('LOS times must be finite and nonnegative')
        self.block_time, self.clear_time = block_time, clear_time
        self.reset()

    def reset(self):
        self.blocked = False
        self.pending_since = self.last_time = self.clear_since = self.blocked_since = None
        self.clear = False
        self.uncertain = False

    def step(self, now, raw_clear, wide_clear):
        if not math.isfinite(now):
            self.reset()
            return False
        if self.last_time is not None and now < self.last_time:
            self.reset()
        self.last_time = now
        clearing = raw_clear and (wide_clear or not self.blocked)
        self.clear_since = (now if self.clear_since is None else self.clear_since) if clearing else None
        self.clear = self.clear_since is not None and now-self.clear_since >= self.clear_time
        changing = (raw_clear and wide_clear) if self.blocked else not raw_clear
        if changing:
            if self.pending_since is None:
                self.pending_since = now
            delay = self.clear_time if self.blocked else self.block_time
            if now-self.pending_since >= delay:
                self.blocked = not self.blocked
                self.blocked_since = now if self.blocked else None
                self.pending_since = None
        else:
            self.pending_since = None
        self.uncertain = not raw_clear and not self.blocked
        return self.clear and not self.blocked


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
    los_block_time: float = 0.3
    los_clear_time: float = 0.6
    los_clear_extra_width: float = 0.10
    min_nav_hold: float = 1.5
    min_direct_hold: float = 1.0


@dataclass(frozen=True)
class HandoffDecision:
    mode: str
    action: str
    direct_publish: bool
    reason: str


class HandoffSupervisor:
    def __init__(self, params=None, route_around=True):
        self.p = params or HandoffParams()
        nonnegative = {'los_block_time', 'los_clear_time', 'los_clear_extra_width',
                       'min_nav_hold', 'min_direct_hold'}
        if any(not math.isfinite(v) or (v < 0 if k in nonnegative else v <= 0)
               for k, v in vars(self.p).items()):
            raise ValueError('Handoff parameters must be finite and within their positive/nonnegative bounds')
        if not isinstance(route_around, bool):
            raise ValueError('route_around must be boolean')
        self.los = LosDebouncer(self.p.los_block_time, self.p.los_clear_time)
        self.nav_since = self.direct_since = None
        self.route_around = route_around
        self.mode = 'DIRECT'
        self.blocked_since = self.clear_since = self.last_goal = None
        self.last_goal_time = self.last_time = None
        self.retry_after = -math.inf
        self.recovery_pending = False

    def record_goal(self, now, map_target):
        """Confirm the actual submission position and time."""
        self.last_goal, self.last_goal_time = tuple(map_target), now

    def step(self, now, state, target, los_clear, front_clearance, nav_status='none',
             *, map_target=None, enabled=True, scan_valid=True, los_clear_wide=None,
             near_latched=False, direct_clear=True):
        """Use measured base/map targets; direct_clear requires lateral space/alignment.

        Safety cancellation ends velocity ownership, not the unfinished detour.
        Recovery waits for valid inputs and the existing entry/cooldown timers.
        """
        action, reason = 'none', 'Direct following'
        previous = self.mode
        clock_reset = (not math.isfinite(now)
                       or self.last_time is not None and now < self.last_time)
        self.last_time = now
        safe = (enabled and not near_latched and state not in ('LOST', 'TOO_CLOSE', 'IDLE')
                and valid_xy(target) and scan_valid
                and not math.isnan(front_clearance) and front_clearance >= 0)
        if not safe or clock_reset:
            self.los.reset()
        else:
            self.los.step(now, los_clear, los_clear if los_clear_wide is None else los_clear_wide)
        # Raw blockage during debounce is uncertain, never a clear return signal.
        los_clear = self.los.clear and not self.los.blocked
        if not self.route_around or not safe or clock_reset:
            # Preserve the route after scan/TF faults, but never across a lost or
            # near target, disable, or clock reset.
            self.recovery_pending = (
                self.route_around and not clock_reset and enabled and not near_latched
                and state not in ('LOST', 'TOO_CLOSE', 'IDLE')
                and (previous == 'NAV2' or self.recovery_pending))
            self.mode = 'DIRECT'
            self.blocked_since = self.clear_since = None
            if clock_reset:
                self.retry_after = -math.inf
                self.direct_since = self.nav_since = None
            reason = 'Direct only or safety stop'
        elif self.mode == 'NAV2':
            distance = math.hypot(*target)
            close = distance <= self.p.follow_distance+self.p.handoff_margin
            clear = (close and los_clear and direct_clear
                     and front_clearance > self.p.obstacle_stop_distance+self.p.clear_margin)
            self.clear_since = (now if self.clear_since is None else self.clear_since) if clear else None
            if nav_status in ('failed', 'canceled') or not valid_xy(map_target):
                self.mode = 'DIRECT'
                self.recovery_pending = True
                self.retry_after = now+self.p.retry_cooldown
                reason = 'Navigation failed or map transform unavailable'
            elif (los_clear and distance <= self.p.follow_distance
                  and (nav_status == 'succeeded'
                       or now-self.nav_since >= self.p.min_nav_hold)):
                # The goal is the person's position; direct control must hold
                # follow distance even when a narrow aisle cannot clear sides.
                self.mode = 'DIRECT'
                reason = 'Visible person reached follow distance'
            elif (now-self.nav_since >= self.p.min_nav_hold
                  and self.clear_since is not None
                  and now-self.clear_since >= self.p.resume_delay):
                self.mode = 'DIRECT'
                reason = 'Direct path, side clearance and alignment restored'
            # Success at an old goal cannot release a far person through an
            # unsafe direct path. Keep the displacement/rate limits.
            elif (math.dist(map_target, self.last_goal) > self.p.goal_update_distance
                  and now-self.last_goal_time >= self.p.goal_update_period):
                action, reason = 'update_goal', 'Person moved in map frame'
            else:
                reason = 'Navigation owns velocity'
        else:
            if (self.recovery_pending and direct_clear and los_clear
                    and front_clearance > self.p.obstacle_stop_distance+self.p.clear_margin):
                self.recovery_pending = False
            blocked = state == 'OBSTACLE_STOP' or self.los.blocked or self.recovery_pending
            eligible = (blocked and valid_xy(map_target)
                        and math.hypot(*target) > self.p.follow_distance+self.p.handoff_margin
                        and now >= self.retry_after)
            self.blocked_since = (now if self.blocked_since is None else self.blocked_since) if eligible else None
            if (self.blocked_since is not None
                    and now-self.blocked_since >= self.p.handoff_delay
                    and (self.recovery_pending or self.direct_since is None
                         or now-self.direct_since >= self.p.min_direct_hold
                         or self.los.blocked_since is not None
                         and now-self.los.blocked_since >= self.p.min_direct_hold)):
                self.mode, action, reason = 'NAV2', 'send_goal', 'Persistent blocked path'
        if previous != 'NAV2' and self.mode == 'NAV2':
            self.nav_since = now
            self.recovery_pending = False
        if previous == 'NAV2' and self.mode == 'DIRECT':
            self.direct_since = now
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
