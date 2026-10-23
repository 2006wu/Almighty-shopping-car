"""Planar odometry drift for time-aligned poses sharing the spawn origin."""
from dataclasses import dataclass
import math


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2:
    time: float
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.time, self.x, self.y, self.yaw)):
            raise ValueError('Pose must be finite')


def interpolate_pose(before, after, time, max_gap=0.1):
    """Interpolate a bracketed truth pose; never extrapolate across missing data."""
    if not math.isfinite(max_gap) or max_gap <= 0 or not math.isfinite(time):
        raise ValueError('Invalid interpolation limits')
    dt = after.time-before.time
    if dt <= 0 or dt > max_gap or not before.time <= time <= after.time:
        return None
    fraction = (time-before.time)/dt
    return Pose2(time, before.x+fraction*(after.x-before.x),
                 before.y+fraction*(after.y-before.y),
                 wrap_angle(before.yaw+fraction*wrap_angle(after.yaw-before.yaw)))


@dataclass(frozen=True)
class Drift:
    translation: float
    heading: float
    distance: float
    translation_per_metre: float | None
    heading_per_metre: float | None


class DriftMetrics:
    def __init__(self):
        self.previous = None
        self.distance = 0.0

    def update(self, estimate, truth):
        if abs(estimate.time-truth.time) > 1e-6:
            raise ValueError('Poses must be time aligned')
        if self.previous is not None:
            if truth.time <= self.previous.time:
                raise ValueError('Samples must increase in time')
            self.distance += math.hypot(truth.x-self.previous.x, truth.y-self.previous.y)
        self.previous = truth
        translation = math.hypot(estimate.x-truth.x, estimate.y-truth.y)
        heading = abs(wrap_angle(estimate.yaw-truth.yaw))
        moving = self.distance > 1e-6
        return Drift(translation, heading, self.distance,
                     translation/self.distance if moving else None,
                     heading/self.distance if moving else None)
