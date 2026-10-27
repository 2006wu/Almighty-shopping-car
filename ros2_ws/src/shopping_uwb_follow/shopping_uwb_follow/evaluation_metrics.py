"""Timestamp-based simulation evaluation; no ROS dependencies.

Traces contain (simulation seconds, tuple of finite values). Positive lag means
that the observed signal trails the reference: observed(t) ~= reference(t-lag).
"""
from bisect import bisect_left
import math
from statistics import mean

from .person_path import path_position


class Trace:
    def __init__(self, samples):
        self.samples = list(samples)
        self.times = [t for t, _ in self.samples]
        if (any(not math.isfinite(t) or not v or
                not all(math.isfinite(x) for x in v) for t, v in self.samples)
                or any(b <= a for a, b in zip(self.times, self.times[1:]))
                or len({len(v) for _, v in self.samples}) > 1):
            raise ValueError('Trace requires finite, strictly increasing, equal-sized samples')

    def at(self, time, max_gap=0.25, yaw=False):
        i = bisect_left(self.times, time)
        if i < len(self.times) and abs(self.times[i]-time) < 1e-8:
            return self.samples[i][1]
        if i == 0 or i == len(self.times) or self.times[i]-self.times[i-1] > max_gap:
            return None
        a, b = self.samples[i-1], self.samples[i]
        ratio = (time-a[0])/(b[0]-a[0])
        result = [x+ratio*(y-x) for x, y in zip(a[1], b[1])]
        if yaw:
            delta = math.atan2(math.sin(b[1][2]-a[1][2]), math.cos(b[1][2]-a[1][2]))
            result[2] = a[1][2]+ratio*delta
        return tuple(result)


def stats(values):
    values = list(values)
    return ({'count': len(values), 'mean': mean(values),
             'rmse': math.sqrt(mean(x*x for x in values)), 'max': max(values),
             'min': min(values)} if values else {'count': 0})


def best_shift(reference, observed, max_lag=0.5, step=0.01, max_gap=0.25):
    """Search vector MSE on identical support for every candidate, no extrapolation.

    Constant reference signals, insufficient support and boundary optima cannot
    establish latency. Absolute position bias can confound this estimate.
    """
    if not all(math.isfinite(x) and x > 0 for x in (max_lag, step, max_gap)) or step > max_lag:
        raise ValueError('Require positive search bounds, with step <= max_lag')
    reference, observed = Trace(reference), Trace(observed)
    if (reference.samples and observed.samples
            and len(reference.samples[0][1]) != len(observed.samples[0][1])):
        raise ValueError('Reference and observation dimensions must match')
    n = int(max_lag/step)
    shifts = [i*step for i in range(-n, n+1)]
    support = []
    for t, value in observed.samples:
        refs = [reference.at(t-shift, max_gap) for shift in shifts]
        if all(v is not None for v in refs):
            support.append((value, refs))
    if len(support) < 10:
        return {'identifiable': False, 'reason': 'insufficient common support', 'count': len(support)}
    central = [refs[n] for _, refs in support]
    center = [mean(v[d] for v in central) for d in range(len(central[0]))]
    excitation = math.sqrt(mean(sum((x-c)**2 for x, c in zip(v, center)) for v in central))
    if excitation < 0.02:
        return {'identifiable': False, 'reason': 'insufficient motion', 'count': len(support)}
    losses = [mean(sum((a-b)**2 for a, b in zip(value, refs[i]))
                   for value, refs in support) for i in range(len(shifts))]
    best = min(range(len(shifts)), key=lambda i: losses[i])
    return {'identifiable': best not in (0, len(shifts)-1),
            'reason': 'boundary optimum' if best in (0, len(shifts)-1) else 'ok',
            'lag_s': shifts[best], 'rmse_m': math.sqrt(losses[best]),
            'count': len(support), 'excitation_m': excitation}


def frame_valid(kind, parent, child=None):
    expected = {'person': ('sim_world', None), 'target': ('base_link', None),
                'robot': ('sim_world', 'base_footprint_gt'),
                'model': ('supermarket', 'person')}
    return (parent, child) == expected[kind]


def segment_kind(path, time, window=0.5, heading_rate=0.3):
    """Classify polygon corners by heading change over a centered time window."""
    a, b, c = (path_position(path, time+dt) for dt in (-window, 0, window))
    u, v = (b[0]-a[0], b[1]-a[1]), (c[0]-b[0], c[1]-b[1])
    if min(math.hypot(*u), math.hypot(*v)) < 1e-9:
        return 'unknown'
    angle = abs(math.atan2(u[0]*v[1]-u[1]*v[0], u[0]*v[0]+u[1]*v[1]))
    return 'turn' if angle/window > heading_rate else 'straight'


def relative_position(person, robot):
    dx, dy = person[0]-robot[0], person[1]-robot[1]
    c, s = math.cos(robot[2]), math.sin(robot[2])
    return c*dx+s*dy, -s*dx+c*dy


def near_stop(samples, stop_distance, speed_limit=0.03, angular_limit=0.05,
              deadline=0.5, max_gap=0.25):
    """Assess every near episode, including continued stopping after the deadline.

    Samples: (time, distance, planar speed, absolute yaw rate). Short episodes or
    recording boundaries are censored; gaps must never silently certify a stop.
    """
    events, active = [], []
    left_censored = False

    def finish(closed, closing_time=None):
        if not active:
            return
        start = active[0][0]
        late = [s for s in active if s[0]-start >= deadline-1e-8]
        gaps = (any(b[0]-a[0] > max_gap for a, b in zip(active, active[1:]))
                or (closing_time is not None and closing_time-active[-1][0] > max_gap))
        violations = [s[0] for s in late if s[2] > speed_limit or s[3] > angular_limit]
        stopped = next((s[0]-start for s in active
                        if s[2] <= speed_limit and s[3] <= angular_limit), None)
        events.append({'start_s': start, 'first_stop_s': stopped,
                       'violation_times_s': violations,
                       'status': 'FAIL' if violations else
                       'PASS' if late and not gaps and closed and not left_censored else 'NOT_COVERED'})
        active.clear()

    samples = list(samples)
    if (any(not all(math.isfinite(x) for x in s) or len(s) != 4 for s in samples)
            or any(b[0] <= a[0] for a, b in zip(samples, samples[1:]))):
        raise ValueError('Near-stop samples require finite values and increasing time')
    for i, sample in enumerate(samples):
        if sample[1] < stop_distance:
            if not active:
                left_censored = i == 0
            active.append(sample)
        else:
            finish(True, sample[0])
    finish(False)
    return events


def reacquire_events(target_times, states, end, timeout=0.5, max_gap=0.25,
                     recognized_states=('FOLLOWING',)):
    """Measure fresh-estimate -> recognized tracking state after target loss.

    Safety holds recognize a fresh target while correctly commanding zero motion.
    Preserve the later FOLLOWING time separately; never count IDLE or LOST as
    recovery. A resumed estimate must be fresh and state coverage continuous.
    """
    starts = [a+timeout for a, b in zip(target_times, target_times[1:]) if b-a > timeout]
    if target_times and end-target_times[-1] > timeout:
        starts.append(target_times[-1]+timeout)
    previous = None
    for t, state in states:
        if state == 'LOST' and previous != 'LOST':
            starts.append(t)
        previous = state
    events, until = [], -math.inf

    def fresh_at(t, first):
        i = bisect_left(target_times, t)
        if i < len(target_times) and target_times[i] == t:
            latest = target_times[i]
        elif i > 0:
            latest = target_times[i-1]
        else:
            return False
        return latest >= first and 0 <= t-latest <= timeout
    for start in sorted(starts):
        if start <= until:
            continue
        fresh = next((t for t in target_times if t > start), None)
        recognized = set(recognized_states)
        recovered = next(((t, state) for t, state in states if fresh is not None and t >= fresh
                          and state in recognized and fresh_at(t, fresh)), None)
        recovery = recovered[0] if recovered is not None else None
        following = next((t for t, state in states if fresh is not None and t >= fresh
                          and state == 'FOLLOWING' and fresh_at(t, fresh)), None)
        interval = [t for t, _ in states if start <= t <= (recovery if recovery is not None else end)]
        covered = (len(interval) >= 2 and interval[0]-start <= max_gap
                   and (recovery if recovery is not None else end)-interval[-1] <= max_gap
                   and all(b-a <= max_gap for a, b in zip(interval, interval[1:])))
        events.append({'lost_s': start, 'fresh_s': fresh, 'following_s': following,
                       'tracking_s': recovery, 'tracking_state': recovered[1] if recovered else None,
                       'reacquire_s': recovery-fresh if recovery is not None else None,
                       'outage_to_following_s': recovery-start if recovery is not None else None,
                       'state_coverage': covered,
                       'status': 'PASS' if recovery is not None and covered else 'NOT_COVERED'})
        until = recovery if recovery is not None else end
    return events


def coverage(samples, start, end, max_gap=0.25):
    times = [t for t, _ in samples]
    return (len(times) >= 10 and times[0]-start <= max_gap and end-times[-1] <= max_gap
            and all(b-a <= max_gap for a, b in zip(times, times[1:])))


def check_row(name, passed, details):
    return {'check': name, 'status': 'PASS' if passed else 'FAIL', 'details': details}


def event_row(name, events, limit=None):
    failed = any(e['status'] == 'FAIL' or
                 (limit is not None and e.get('reacquire_s') is not None
                  and e['reacquire_s'] > limit) for e in events)
    status = ('FAIL' if failed else 'PASS' if events and
              all(e['status'] == 'PASS' for e in events) else 'NOT_COVERED')
    return {'check': name, 'status': status, 'details': events}


def sync_report(person, model, path, mean_limit=0.05, lag_limit=0.1):
    reference = Trace(person)
    errors = [math.dist(v, p) for t, v in model if (p := reference.at(t)) is not None]
    rows = []
    for name, series in (('truth_vs_path', person), ('model_vs_path', model)):
        offsets = stats(math.dist(v, path_position(path, t)) for t, v in series)
        analytic = [(t, path_position(path, t)) for t, _ in series]
        lag = best_shift(analytic, series)
        rows.append(check_row(name, offsets['count'] >= 10 and offsets['mean'] < mean_limit,
                              offsets))
        rows.append(check_row(name+'_lag', lag.get('identifiable', False)
                              and abs(lag.get('lag_s', math.inf)) < lag_limit, lag))
    offsets = stats(errors)
    rows.append(check_row('model_vs_truth', offsets['count'] >= 10
                          and offsets['mean'] < mean_limit, offsets))
    lag = best_shift(person, model)
    rows.append(check_row('model_vs_truth_lag', lag.get('identifiable', False)
                          and abs(lag.get('lag_s', math.inf)) < lag_limit, lag))
    return rows


def follow_performance(distances, path, follow_distance, settle_window=5.0, follow_time_headway=0.0):
    """Informational truth metrics; settle from trace start and exclude pauses.

    Distances are timestamped ground-truth separations. No acceptance threshold
    is introduced, and missing eligible support is reported with count zero.
    """
    distances = list(distances)
    start = distances[0][0] if distances else 0.0
    errors = [abs(distance-follow_distance) for t, distance in distances
              if t-start >= settle_window
              and math.dist(path_position(path, t-0.01), path_position(path, t+0.01)) > 1e-6]
    return {'person_speed_m_s': path['speed'], 'follow_distance_m': follow_distance,
            'follow_time_headway_s': follow_time_headway,
            'moving_target_distance_m': follow_distance+follow_time_headway*path['speed'],
            'settle_window_s': settle_window, 'steady_state_distance_error_m': stats(errors),
            'max_gap_m': max((d for _, d in distances), default=None)}


def follow_report(person, robot, target, states, path, params, end,
                  latency_limit=0.3, tracking_limit=1.5, reacquire_limit=2.0,
                  stop_deadline=0.5):
    truth, poses = Trace(person), Trace(robot)
    relative, near, segments = [], [], {'straight': [], 'turn': [], 'unknown': []}
    estimates = Trace(target)
    for t, pose in poses.samples:
        p = truth.at(t)
        if p is None:
            continue
        r = relative_position(p, pose)
        relative.append((t, r))
        distance = math.hypot(*r)
        near.append((t, distance, pose[3], pose[4]))
        estimate = estimates.at(t)
        segments[segment_kind(path, t)].append(
            (distance, abs(distance-params['follow_distance']),
             math.dist(r, estimate) if estimate is not None else None))
    rows = []
    lag = best_shift(relative, target)
    rows.append(check_row('filter_latency', lag.get('identifiable', False)
                          and abs(lag.get('lag_s', math.inf)) < latency_limit, lag))
    for kind, values in segments.items():
        detail = {'distance_m': stats(v[0] for v in values),
                  'distance_error_m': stats(v[1] for v in values),
                  'estimate_error_m': stats(v[2] for v in values if v[2] is not None)}
        if kind == 'unknown' and not values:
            continue
        rows.append({'check': kind+'_tracking',
                     'status': ('NOT_COVERED' if len(values) < 10 else 'PASS'
                                if detail['distance_error_m']['rmse'] < tracking_limit else 'FAIL'),
                     'details': detail})
    rows.append({'check': 'follow_performance', 'status': 'INFO',
                 'details': follow_performance([(t, d) for t, d, _, _ in near],
                                               path, params['follow_distance'],
                                               follow_time_headway=params.get('follow_time_headway', 0.0))})
    rows.append(event_row('near_stop', near_stop(near, params['stop_distance'], deadline=stop_deadline)))
    events = reacquire_events([t for t, _ in target], states, end, params['lost_timeout'])
    for event in events:
        if (event['tracking_s'] is None and event['fresh_s'] is not None
                and end-event['fresh_s'] > reacquire_limit and event['state_coverage']):
            event['status'] = 'FAIL'
    rows.append(event_row('reacquire', events, reacquire_limit))
    # Keep the original motion-recovery gate; target recognition is distinct.
    recognized = reacquire_events([t for t, _ in target], states, end, params['lost_timeout'],
                                 recognized_states=('FOLLOWING', 'TOO_CLOSE', 'OBSTACLE_STOP',
                                                    'BLOCKED_WAIT', 'NAV2_HANDOFF'))
    rows.append(event_row('target_reacquire', recognized, reacquire_limit))
    return rows
