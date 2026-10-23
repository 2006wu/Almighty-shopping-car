"""Planar evaluation of timestamp-aligned samples, without ROS dependencies."""
import math
from dataclasses import dataclass
from statistics import mean


@dataclass(frozen=True)
class FollowSample:
    time: float
    estimate: tuple  # (x, y) in base_link
    robot_pose: tuple  # (x, y, yaw) in a common world frame
    ground_truth: tuple  # (x, y) in the same world frame as robot_pose


def sample_metrics(sample):
    """Return (estimation error, true follow distance), in metres."""
    if (len(sample.estimate) != 2 or len(sample.robot_pose) != 3
            or len(sample.ground_truth) != 2
            or not all(math.isfinite(v) for v in
                       (sample.time, *sample.estimate, *sample.robot_pose, *sample.ground_truth))):
        raise ValueError('Sample must contain finite planar coordinates and time')
    x, y, yaw = sample.robot_pose
    ex, ey = sample.estimate
    gx, gy = sample.ground_truth
    wx = x+math.cos(yaw)*ex-math.sin(yaw)*ey
    wy = y+math.sin(yaw)*ex+math.cos(yaw)*ey
    return math.hypot(wx-gx, wy-gy), math.hypot(gx-x, gy-y)


def synchronized_sample(estimate, robot_pose, ground_truth, max_skew=0.15):
    """Build a sample from three (timestamp, value) pairs, or skip stale data.

    Callers must supply a common clock and aligned world/odom coordinates.
    No interpolation is performed.
    """
    if not math.isfinite(max_skew) or max_skew < 0:
        raise ValueError('Maximum timestamp skew must be finite and nonnegative')
    if any(item is None for item in (estimate, robot_pose, ground_truth)):
        return None
    times = [item[0] for item in (estimate, robot_pose, ground_truth)]
    if not all(math.isfinite(t) for t in times) or max(times)-min(times) > max_skew:
        return None
    sample = FollowSample(estimate[0], estimate[1], robot_pose[1], ground_truth[1])
    sample_metrics(sample)
    return sample


def evaluate_series(samples):
    """Compute sample-weighted statistics for strictly increasing timestamps.

    Empty series and invalid samples raise ValueError. Distances use ground truth;
    error is the Euclidean distance from the transformed estimate to ground truth.
    """
    samples = list(samples)
    if not samples or any(b.time <= a.time for a, b in zip(samples, samples[1:])):
        raise ValueError('Series must be nonempty with increasing timestamps')
    errors, distances = zip(*(sample_metrics(sample) for sample in samples))
    return {'count': len(samples), 'duration': samples[-1].time-samples[0].time,
            'estimate_error_mean': mean(errors),
            'estimate_error_rmse': math.sqrt(mean(e*e for e in errors)),
            'estimate_error_max': max(errors),
            'follow_distance_mean': mean(distances),
            'follow_distance_min': min(distances),
            'follow_distance_max': max(distances)}


def state_statistics(samples):
    """Summarize (time, state) samples, with final dwell censored at last sample.

    Times must be finite and strictly increasing. Dwell includes both boundary
    runs; no duration is extrapolated beyond the observed interval.
    """
    samples = list(samples)
    if (not samples or any(not math.isfinite(t) or not isinstance(s, str) or not s
                           for t, s in samples)
            or any(b[0] <= a[0] for a, b in zip(samples, samples[1:]))):
        raise ValueError('States require finite increasing times and nonempty names')
    runs = {}
    start, state = samples[0]
    transitions = 0
    for now, current in samples[1:]:
        if current != state:
            runs.setdefault(state, []).append(now-start)
            start, state = now, current
            transitions += 1
    runs.setdefault(state, []).append(samples[-1][0]-start)
    duration = samples[-1][0]-samples[0][0]
    return {'transitions': transitions, 'duration': duration,
            'transitions_per_minute': transitions*60/duration if duration else 0.0,
            'dwell': {state: {'count': len(values), 'total': sum(values),
                              'mean': mean(values), 'min': min(values), 'max': max(values)}
                      for state, values in sorted(runs.items())}}


def parse_state_log(lines, sample_period=0.05):
    """Parse echo data or state log lines, optionally prefixed by [seconds].

    Untimed logs use the explicitly supplied sample period (20 Hz by default).
    Mixed timing is rejected rather than silently mixing clock domains.
    """
    import re
    if not math.isfinite(sample_period) or sample_period <= 0:
        raise ValueError('Sample period must be positive and finite')
    names = 'NAV2_HANDOFF|FOLLOWING|BLOCKED_WAIT|OBSTACLE_STOP|TOO_CLOSE|LOST|IDLE'
    pattern = re.compile(r'(?:data:\s*[\'"]?|\bstate[=: ]+|^\s*|\]\s*)('
                         + names+r')\b', re.IGNORECASE)
    timestamp = re.compile(r'\[([0-9]+(?:\.[0-9]+)?)\]|^\s*([0-9]+(?:\.[0-9]+)?)\s+')
    samples, timed = [], None
    for line in lines:
        match = pattern.search(line)
        if match is None:
            continue
        stamp = timestamp.search(line[:match.end(1)-len(match.group(1))])
        has_time = stamp is not None
        if timed is not None and timed != has_time:
            raise ValueError('Do not mix timestamped and untimed state samples')
        timed = has_time
        now = float(stamp.group(1) or stamp.group(2)) if stamp else len(samples)*sample_period
        samples.append((now, match.group(1).upper()))
    return samples
