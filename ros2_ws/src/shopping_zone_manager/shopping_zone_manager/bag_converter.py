"""Convert plain simulation records to eval_report rows without ROS dependencies."""
from bisect import bisect_left
import math

from .eval_metrics import finite

TRUTH = '/sim/ground_truth_odom'
PERSON = '/uwb/person_ground_truth'
PEDESTRIANS = '/sim/pedestrians'
FOLLOW = '/follow/state'
ZONE = '/shopping/zone_status'


def yaw(q):
    x, y, z, w = (finite(q[k]) for k in ('x', 'y', 'z', 'w'))
    if abs(x*x+y*y+z*z+w*w-1) > 0.01:
        raise ValueError('Expected normalized quaternion')
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def convert_records(records, zones=(), map_world_transform=None, max_skew=0.15, pedestrian_radius=0.25):
    """Sort header-time records, validate duplicate truth times, interpolate bounded truth.

    zones are dictionaries from zones.yaml. The explicit (x,y,yaw) transform maps
    map coordinates into sim_world; no estimated odometry is used as truth.
    """
    max_skew = finite(max_skew)
    if max_skew <= 0:
        raise ValueError('max_skew must be positive')
    pedestrian_radius = finite(pedestrian_radius)
    if pedestrian_radius <= 0:
        raise ValueError('pedestrian_radius must be positive')
    streams = {}
    for topic, t, message in records:
        streams.setdefault(topic, []).append((finite(t), message))
    input_counts = {k: len(v) for k, v in streams.items()}
    duplicates = {}
    for topic, samples in streams.items():
        samples.sort(key=lambda item: item[0])
        if topic == ZONE:
            continue  # Lifecycle transitions may share one recorder clock tick.
        unique = dict(samples)
        count = len(samples)-len(unique)
        if count and topic in (TRUTH, PERSON, PEDESTRIANS):
            raise ValueError(f'Duplicate truth timestamp on {topic}; split resets into separate bags')
        if count:
            duplicates[topic] = count
        streams[topic] = sorted(unique.items())
    rows = []
    info = dict(input_counts=input_counts, coalesced_topic_samples=duplicates,
                missing_person_samples=0, missing_pedestrian_samples=0, cutoff_episodes=[], ignored_zone_statuses=0,
                map_world_transform=map_world_transform, max_skew=max_skew, pedestrian_radius=pedestrian_radius)

    def emit(kind, t, **fields):
        rows.append(dict(type=kind, t=t, **fields))

    truth, people = [], []
    for t, msg in streams.get(TRUTH, []):
        if msg['header']['frame_id'] != 'sim_world':
            raise ValueError('Robot truth must be in sim_world')
        pose = msg['pose']['pose']
        truth.append((t, (finite(pose['position']['x']), finite(pose['position']['y']),
                          yaw(pose['orientation']))))
        twist = msg['twist']['twist']
        emit('odom', t, linear=math.hypot(finite(twist['linear']['x']), finite(twist['linear']['y'])),
             angular=finite(twist['angular']['z']))
    for t, msg in streams.get(PERSON, []):
        if msg['header']['frame_id'] != 'sim_world':
            raise ValueError('Person truth must be in sim_world')
        people.append((t, (finite(msg['point']['x']), finite(msg['point']['y']))))

    def interpolator(samples):
        times = [s[0] for s in samples]

        def at(t):
            index = bisect_left(times, t)
            if index < len(times) and times[index] == t:
                return samples[index][1]
            if index == 0 or index == len(times):
                return None
            before, after = samples[index-1], samples[index]
            if t-before[0] > max_skew or after[0]-t > max_skew:
                return None
            fraction = (t-before[0])/(after[0]-before[0])
            values = [a+(b-a)*fraction for a, b in zip(before[1], after[1])]
            if len(values) == 3:
                delta = math.atan2(math.sin(after[1][2]-before[1][2]),
                                   math.cos(after[1][2]-before[1][2]))
                values[2] = before[1][2]+delta*fraction
            return tuple(values)
        return at

    pedestrian_streams = []
    for t, msg in streams.get(PEDESTRIANS, []):
        if msg['header']['frame_id'] != 'sim_world':
            raise ValueError('Pedestrian truth must be in sim_world')
        poses = msg['poses']
        if not pedestrian_streams:
            pedestrian_streams = [[] for _ in poses]
        if not poses or len(poses) != len(pedestrian_streams):
            raise ValueError('Expected a fixed nonempty pedestrian order per bag')
        for samples, pose in zip(pedestrian_streams, poses):
            samples.append((t, (finite(pose['position']['x']), finite(pose['position']['y']))))
    pedestrians_at = [interpolator(samples) for samples in pedestrian_streams]
    robot_at, person_at = interpolator(truth), interpolator(people)
    info['max_truth_gap'] = max((b[0]-a[0] for a, b in zip(truth, truth[1:])), default=None)
    previous_distance_time = None
    distance_started = False
    distance_gap = False
    for t, (x, y, heading) in truth:
        fields = dict(x=x, y=y, yaw=heading)
        if pedestrians_at:
            positions = [at(t) for at in pedestrians_at]
            if all(position is not None for position in positions):
                fields['pedestrians'] = [dict(x=p[0], y=p[1], radius=pedestrian_radius) for p in positions]
            else:
                info['missing_pedestrian_samples'] += 1
        person = person_at(t)
        if person is not None:
            fields.update(person_x=person[0], person_y=person[1])
            if streams.get(FOLLOW):
                if distance_gap or (previous_distance_time is not None and
                                    t-previous_distance_time > 2*max_skew):
                    raise ValueError('Interior person truth gap; split the evaluation interval')
                previous_distance_time = t
                distance_started = True
                emit('distance', t, distance=math.hypot(person[0]-x, person[1]-y))
        else:
            info['missing_person_samples'] += 1
            distance_gap = distance_started
        emit('pose', t, **fields)
    for t, msg in streams.get('/cmd_vel', []):
        emit('cmd', t, linear=math.hypot(finite(msg['linear']['x']), finite(msg['linear']['y'])),
             angular=finite(msg['angular']['z']))
    previous = None
    for t, msg in streams.get(FOLLOW, []):
        state = msg['data'].split(':', 1)[0].strip()
        emit('state', t, state=state)
        if state != previous and state in ('LOST', 'TOO_CLOSE', 'OBSTACLE_STOP'):
            emit('trigger', t, event=state.lower())
        previous = state
    previous = None
    for t, msg in streams.get('/mode_manager/state', []):
        state = msg['data']
        if state == 'ESTOP' and previous != state:
            emit('trigger', t, event='estop')
        previous = state

    by_name = {z['name']: z for z in zones}
    active = None
    episode_number = 0

    def finish(end, success, cutoff=False):
        nonlocal active
        start, goal, identity = active
        if end < start:
            raise ValueError('Navigation ends before it starts')
        times = sorted({start, end, *(t for t, _ in truth if start < t < end)})
        for t in times:
            pose = robot_at(t)
            if pose is None:
                raise ValueError(f'Missing truth at navigation boundary {identity} t={t}')
            emit('navigation', t, episode=identity, success=success,
                 goal_x=goal[0], goal_y=goal[1], goal_yaw=goal[2],
                 x=pose[0], y=pose[1], yaw=pose[2])
        if cutoff:
            info['cutoff_episodes'].append(identity)
        active = None

    for t, msg in streams.get(ZONE, []):
        code, _, text = msg['data'].partition(':')
        if code == 'PENDING' and text.strip().startswith('Requested '):
            if active is not None:
                raise ValueError('Overlapping navigation episodes')
            name = text.strip()[len('Requested '):]
            if name not in by_name:
                raise ValueError(f'Unknown recorded zone: {name}')
            if map_world_transform is None:
                raise ValueError('Navigation requires explicit map_world_transform')
            zone = by_name[name]
            if zone.get('frame_id') != 'map':
                raise ValueError('Only map-frame zone goals are supported')
            dx, dy, angle = (finite(v) for v in map_world_transform)
            x, y, heading = (finite(zone[k]) for k in ('x', 'y', 'yaw'))
            goal = (dx+math.cos(angle)*x-math.sin(angle)*y,
                    dy+math.sin(angle)*x+math.cos(angle)*y, heading+angle)
            episode_number += 1
            active = (t, goal, f'zone-{episode_number}-{name}')
        elif code in ('SUCCEEDED', 'ABORTED', 'CANCELED', 'REJECTED', 'TIMEOUT'):
            if active is not None:
                finish(t, code == 'SUCCEEDED')
            else:
                info['ignored_zone_statuses'] += 1
        elif code in ('ACTIVE', 'CANCELING') and active is None:
            raise ValueError('Recording started during navigation; record before goto')
    if active is not None:
        if not truth:
            raise ValueError('Unfinished navigation has no truth samples')
        finish(truth[-1][0], False, cutoff=True)
    # Independent trigger sources can coincide; merge reasons into one event.
    triggers = {}
    other_rows = []
    for row in rows:
        if row['type'] == 'trigger':
            triggers.setdefault(row['t'], set()).add(row['event'])
        else:
            other_rows.append(row)
    other_rows.extend(dict(type='trigger', t=t, event='+'.join(sorted(events)))
                      for t, events in triggers.items())
    return sorted(other_rows, key=lambda row: row['t']), info
