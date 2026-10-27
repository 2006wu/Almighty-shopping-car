"""ROS-free acceptance rules for simulation fault trials."""
from dataclasses import dataclass
import math

# Noise trials only require the follow command to stay within the configured
# follow limits (uwb_follow.yaml max_linear / max_angular) plus rounding slack.
NOISE_MAX_LINEAR = 1.0+0.01
NOISE_MAX_ANGULAR = 1.5+0.01


@dataclass(frozen=True)
class Sample:
    time: float
    linear: float
    angular: float


def moving(sample, translation_only=False):
    return abs(sample.linear) > 0.02 or not translation_only and abs(sample.angular) > 0.05


def planar_sample(stamp, vx, vy, angular):
    """Keep lateral truth motion visible to the stop check in either twist frame."""
    return Sample(stamp, math.hypot(vx, vy), angular)


def valid_samples(samples):
    return bool(samples) and all(
        all(math.isfinite(v) for v in (s.time, s.linear, s.angular))
        and (i == 0 or s.time > samples[i-1].time)
        for i, s in enumerate(samples))


def stop_latency(samples, start, end, deadline, max_gap=0.25):
    """Require both a sustained stop and fresh coverage through the fault end."""
    samples = [s for s in samples if start <= s.time <= end]
    if not valid_samples(samples) or end-start < deadline+0.3:
        return None
    if samples[0].time-start > max_gap or end-samples[-1].time > max_gap:
        return None
    if any(b.time-a.time > max_gap for a, b in zip(samples, samples[1:])):
        return None
    last_motion = max((i for i, s in enumerate(samples) if moving(s)), default=-1)
    tail = samples[last_motion+1:]
    if not tail or tail[-1].time-tail[0].time < 0.3:
        return None
    latency = tail[0].time-start
    return latency if latency <= deadline else None


def evaluate_trial(command, truth, baseline_command, baseline_truth,
                   recovery_command, recovery_truth, start, end, noise=False):
    """Never pass an already stopped robot, missing telemetry or absent recovery."""
    streams = (command, truth, baseline_command, baseline_truth,
               recovery_command, recovery_truth)
    if not all(valid_samples(s) for s in streams):
        return False, None, None, 'missing or invalid telemetry'
    if not all(any(moving(s) for s in stream)
               for stream in (baseline_command, baseline_truth)):
        return False, None, None, 'baseline was not moving'
    if not all(any(moving(s) for s in stream)
               for stream in (recovery_command, recovery_truth)):
        return False, None, None, 'no recovery motion'
    if noise:
        covered = all(stream[0].time <= start+0.25 and stream[-1].time >= end-0.25
                      and all(b.time-a.time <= 0.25 for a, b in zip(stream, stream[1:]))
                      for stream in (command, truth))
        bounded = all(abs(s.linear) <= NOISE_MAX_LINEAR and abs(s.angular) <= NOISE_MAX_ANGULAR
                      for s in command)
        return covered and bounded, None, None, 'noise: command limits and telemetry coverage'
    cmd = stop_latency(command, start, end, 0.8)
    gt = stop_latency(truth, start, end, 1.5)
    return cmd is not None and gt is not None, cmd, gt, 'cmd<=0.8s truth<=1.5s; hold>=0.3s'
