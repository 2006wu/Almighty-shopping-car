"""Offline metrics over plain mappings; all timestamps are seconds on one clock."""
import math
import statistics
import xml.etree.ElementTree as ET


def finite(value):
    """Convert a numeric value, rejecting booleans and non-finite data."""
    if isinstance(value, bool):
        raise ValueError('Boolean numeric values are invalid')
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError('Expected a finite number') from exc
    if not math.isfinite(result):
        raise ValueError('Expected a finite number')
    return result


def positive(value, allow_zero=False):
    value = finite(value)
    if value < 0 or (value == 0 and not allow_zero):
        raise ValueError('Invalid nonnegative/positive parameter')
    return value


def series(records, fields=()):
    """Copy and validate a strictly increasing stream without sorting it."""
    result, previous = [], -math.inf
    for record in records:
        row = dict(record)
        for key in ('t', *fields):
            try:
                row[key] = finite(row[key])
            except KeyError as exc:
                raise ValueError(f'Missing field: {key}') from exc
        if row['t'] <= previous:
            raise ValueError('Timestamps must be strictly increasing within a stream')
        previous = row['t']
        result.append(row)
    return result


def summary(values):
    """Sample-weighted statistics with linearly interpolated (n-1)*0.95 p95."""
    values = sorted(finite(v) for v in values)
    if not values:
        return dict(count=0, mean=None, median=None, p95=None, max=None)
    index = (len(values)-1)*0.95
    lower, upper = math.floor(index), math.ceil(index)
    percentile = values[lower] + (values[upper]-values[lower])*(index-lower)
    return dict(count=len(values), mean=statistics.mean(values),
                median=statistics.median(values), p95=percentile, max=values[-1])


def navigation_metrics(episodes):
    """Each episode has id, success (bool), goal=(x,y,yaw), and pose samples."""
    results = []
    for episode in episodes:
        if not isinstance(episode['success'], bool):
            raise ValueError('Episode success must be boolean')
        goal = tuple(finite(v) for v in episode['goal'])
        if len(goal) != 3:
            raise ValueError('Goal must contain x, y, yaw')
        poses = series(episode['samples'], ('x', 'y', 'yaw'))
        row = dict(id=episode['id'], success=episode['success'], duration=None,
                   path_length=None, path_efficiency=None,
                   final_position_error=None, final_yaw_error=None)
        if poses:
            first, last = poses[0], poses[-1]
            length = sum(math.hypot(b['x']-a['x'], b['y']-a['y'])
                         for a, b in zip(poses, poses[1:]))
            straight = math.hypot(goal[0]-first['x'], goal[1]-first['y'])
            row.update(duration=last['t']-first['t'], path_length=length,
                       path_efficiency=straight/length if length > 0 else None,
                       final_position_error=math.hypot(goal[0]-last['x'], goal[1]-last['y']),
                       final_yaw_error=abs(math.atan2(math.sin(goal[2]-last['yaw']),
                                                     math.cos(goal[2]-last['yaw']))))
        results.append(row)
    successes = sum(row['success'] for row in results)
    return dict(episodes=results, count=len(results), successes=successes,
                success_rate=successes/len(results) if results else None,
                time_to_goal=summary(row['duration'] for row in results
                                     if row['success'] and row['duration'] is not None))


def load_store_boxes(path):
    """Read actual store SDF collisions; reject unsupported geometry/transforms."""
    world = ET.parse(path).getroot().find('world')
    if world is None or world.find('include') is not None:
        raise ValueError('Expected an explicit SDF world without includes')
    boxes = []
    for model in world.findall('model'):
        if model.get('name') in ('floor', 'person'):
            continue
        pose_node = model.find('pose')
        pose = [finite(v) for v in model.findtext('pose', '0 0 0 0 0 0').split()]
        links = model.findall('link')
        if (len(pose) != 6 or any(pose[3:]) or
                (pose_node is not None and pose_node.attrib) or
                model.findtext('static') not in ('true', '1') or
                model.find('model') is not None or model.find('include') is not None or
                len(links) != 1 or links[0].find('pose') is not None):
            raise ValueError('Unsupported store model transform or structure')
        collisions = links[0].findall('collision')
        if len(collisions) != 1 or collisions[0].find('pose') is not None:
            raise ValueError('Expected one untransformed collision box')
        size = [positive(v) for v in collisions[0].findtext('geometry/box/size', '').split()]
        if len(size) != 3:
            raise ValueError('Expected a collision box with three dimensions')
        boxes.append((pose[0]-size[0]/2, pose[1]-size[1]/2,
                      pose[0]+size[0]/2, pose[1]+size[1]/2))
    if not boxes:
        raise ValueError('No store obstacles found')
    return boxes


def _rectangle(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*a-s*b, y+s*a+c*b) for a, b in
            [(-length/2, -width/2), (length/2, -width/2),
             (length/2, width/2), (-length/2, width/2)]]


def _edges(polygon):
    return list(zip(polygon, polygon[1:]+polygon[:1]))


def _point_segment(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    u = max(0, min(1, ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(point[0]-a[0]-u*dx, point[1]-a[1]-u*dy)


def _polygon_distance(a, b):
    # SAT detects intersections, including crossing edges and containment.
    separated = False
    for p, q in _edges(a)+_edges(b):
        nx, ny = -(q[1]-p[1]), q[0]-p[0]
        pa = [x*nx+y*ny for x, y in a]
        pb = [x*nx+y*ny for x, y in b]
        if max(pa) < min(pb) or max(pb) < min(pa):
            separated = True
            break
    if not separated:
        return 0.0
    return min(_point_segment(p, u, v) for src, dst in ((a, b), (b, a))
               for p in src for u, v in _edges(dst))


def clearance_metrics(records, boxes, length=0.6, width=0.5, person_radius=0.25):
    """Pose rows may also contain a simultaneous person_x/person_y pair."""
    length, width, radius = positive(length), positive(width), positive(person_radius)
    obstacles = []
    for box in boxes:
        values = tuple(finite(v) for v in box)
        if len(values) != 4 or values[0] >= values[2] or values[1] >= values[3]:
            raise ValueError('Expected box xmin, ymin, xmax, ymax')
        x0, y0, x1, y1 = values
        obstacles.append([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
    rows = series(records, ('x', 'y', 'yaw'))
    static, person = [], []
    for row in rows:
        robot = _rectangle(row['x'], row['y'], row['yaw'], length, width)
        if obstacles:
            static.append(min(_polygon_distance(robot, obstacle) for obstacle in obstacles))
        people = list(row.get('pedestrians', []))
        if 'person_x' in row or 'person_y' in row:
            try:
                people.append(dict(x=row['person_x'], y=row['person_y'], radius=radius))
            except KeyError as exc:
                raise ValueError('Person coordinates must be a pair') from exc
        distances = []
        for pedestrian in people:
            dx, dy = finite(pedestrian['x'])-row['x'], finite(pedestrian['y'])-row['y']
            r = positive(pedestrian['radius'])
            c, s = math.cos(row['yaw']), math.sin(row['yaw'])
            px, py = c*dx+s*dy, -s*dx+c*dy
            distances.append(max(0, math.hypot(max(abs(px)-length/2, 0),
                                              max(abs(py)-width/2, 0))-r))
        if distances:
            person.append(min(distances))
    return dict(pose_count=len(rows), static_sample_count=len(static),
                person_sample_count=len(person), static_min=min(static, default=None),
                person_min=min(person, default=None), minimum=min(static+person, default=None))


def follow_metrics(records, target=1.0, band=0.2, too_close=0.6):
    """Distance statistics are sample weighted; durations use left-hold intervals."""
    target, band, too_close = positive(target), positive(band, True), positive(too_close)
    rows = series(records, ('distance',))
    if any(row['distance'] < 0 for row in rows):
        raise ValueError('Distance cannot be negative')
    errors = [row['distance']-target for row in rows]
    duration = rows[-1]['t']-rows[0]['t'] if rows else 0.0
    in_band = close_duration = 0.0
    events, was_close = 0, False
    for index, row in enumerate(rows):
        dt = rows[index+1]['t']-row['t'] if index+1 < len(rows) else 0.0
        close = row['distance'] <= too_close
        events += int(close and not was_close)
        was_close = close
        close_duration += dt if close else 0
        in_band += dt if target-band <= row['distance'] <= target+band else 0
    return dict(sample_count=len(rows), duration=duration,
                mean_error=statistics.mean(errors) if errors else None,
                mean_absolute_error=statistics.mean(map(abs, errors)) if errors else None,
                rmse=math.sqrt(statistics.mean(e*e for e in errors)) if errors else None,
                p95_absolute_error=summary(map(abs, errors))['p95'],
                time_fraction_in_band=in_band/duration if duration else None,
                too_close_count=events, too_close_duration=close_duration)


def state_metrics(records):
    """Count LOST entries and subsequent FOLLOWING recoveries; repeats do not reset."""
    rows = series(records)
    lost_at, events, recovered, durations = None, 0, 0, []
    for row in rows:
        state = row.get('state')
        if not isinstance(state, str) or not state.strip():
            raise ValueError('Expected a nonempty state string')
        if state == 'LOST' and lost_at is None:
            lost_at = row['t']
            events += 1
        elif state == 'FOLLOWING' and lost_at is not None:
            durations.append(row['t']-lost_at)
            recovered += 1
            lost_at = None
    return dict(lost_count=events, reacquire_count=recovered,
                pending_lost_count=int(lost_at is not None), reacquire_time=summary(durations))


def stop_metrics(triggers, commands, odometry, linear_threshold=0.02,
                 angular_threshold=0.05, zero_tolerance=1e-9):
    """First qualifying sample at/after each trigger, without interpolation."""
    linear_threshold = positive(linear_threshold)
    angular_threshold = positive(angular_threshold)
    zero_tolerance = positive(zero_tolerance, True)
    triggers = series(triggers)
    commands = series(commands, ('linear', 'angular'))
    odometry = series(odometry, ('linear', 'angular'))
    results = []
    for trigger in triggers:
        t = trigger['t']
        command = next((row['t']-t for row in commands if row['t'] >= t and
                        abs(row['linear']) <= zero_tolerance and
                        abs(row['angular']) <= zero_tolerance), None)
        measured = next((row['t']-t for row in odometry if row['t'] >= t and
                         abs(row['linear']) < linear_threshold and
                         abs(row['angular']) < angular_threshold), None)
        results.append(dict(t=t, event=trigger.get('event', 'unspecified'),
                            commanded_latency=command, measured_latency=measured))
    return dict(events=results, count=len(results),
                commanded_pending=sum(r['commanded_latency'] is None for r in results),
                measured_pending=sum(r['measured_latency'] is None for r in results),
                commanded=summary(r['commanded_latency'] for r in results
                                  if r['commanded_latency'] is not None),
                measured=summary(r['measured_latency'] for r in results
                                 if r['measured_latency'] is not None))
