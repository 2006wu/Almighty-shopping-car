"""ROS-independent geometry and measurement guards for simulation checks."""
import math
import xml.etree.ElementTree as ET

import yaml


def navfn_tolerance(path):
    with open(path, encoding='utf-8') as stream:
        params = yaml.safe_load(stream)['planner_server']['ros__parameters']
    tolerances = [float(params[name]['tolerance']) for name in params['planner_plugins']
                  if params[name]['plugin'] == 'nav2_navfn_planner/NavfnPlanner']
    if len(tolerances) != 1 or not math.isfinite(tolerances[0]) or tolerances[0] < 0:
        raise ValueError('Expected one NavFn planner with a finite nonnegative tolerance')
    return tolerances[0]


def store_bounds(path):
    """Conservatively enclose the store using the outer edges of its wall boxes."""
    world = ET.parse(path).getroot().find('world')
    boxes = []
    for name in ('north', 'west', 'east', 'south_left', 'south_right'):
        model = world.find(f"model[@name='{name}']")
        pose = list(map(float, model.findtext('pose').split()))
        size = list(map(float, model.findtext('link/collision/geometry/box/size').split()))
        # Fail closed if the world no longer uses these axis-aligned model-local boxes.
        if (not all(math.isfinite(v) for v in pose + size)
                or any(v != 0 for v in pose[3:]) or any(v <= 0 for v in size)
                or model.find('link/pose') is not None
                or model.find('link/collision/pose') is not None):
            raise ValueError('Expected finite axis-aligned wall boxes without local poses')
        boxes.append((pose[0]-size[0]/2, pose[1]-size[1]/2,
                      pose[0]+size[0]/2, pose[1]+size[1]/2))
    return (min(b[0] for b in boxes), min(b[1] for b in boxes),
            max(b[2] for b in boxes), max(b[3] for b in boxes))


def outside_distance(x, y, bounds):
    """Distance to a closed rectangle, hence a lower bound to any cell within it."""
    xmin, ymin, xmax, ymax = bounds
    if not all(math.isfinite(v) for v in (x, y, *bounds)) or xmin >= xmax or ymin >= ymax:
        raise ValueError('Invalid position or map bounds')
    return math.hypot(max(xmin-x, 0, x-xmax), max(ymin-y, 0, y-ymax))


def unreachable_clearances(x, y, tolerance, world_bounds, costmap_bounds):
    if not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError('Invalid NavFn tolerance')
    distances = [outside_distance(x, y, bounds) for bounds in (world_bounds, costmap_bounds)]
    if any(distance <= tolerance + 1.0 for distance in distances):
        raise ValueError('Unreachable goal must exceed NavFn tolerance + 1 m from store and costmap')
    return distances


def final_position_detail(x, y, zone, max_error):
    if zone.frame_id != 'map':
        raise ValueError('Tolerance check requires a map-frame zone')
    if not all(math.isfinite(v) for v in (x, y, zone.x, zone.y, max_error)) or max_error <= 0:
        raise ValueError('Invalid final position or error limit')
    error = math.hypot(x-zone.x, y-zone.y)
    detail = f'final error={error:.3f} m; limit={max_error:.3f} m'
    if error > max_error:
        raise ValueError(detail)
    return detail
