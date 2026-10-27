import math
from dataclasses import dataclass
import yaml


@dataclass(frozen=True)
class Zone:
    name: str
    aliases: tuple
    x: float
    y: float
    yaw: float
    frame_id: str


def load_zones(path):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get('zones'), list) or not data['zones']:
        raise ValueError('Expected a nonempty zones list')
    result, seen = [], set()
    for entry in data['zones']:
        if not isinstance(entry, dict):
            raise ValueError('Zone must be a mapping')
        try:
            name, aliases, frame = entry['name'], entry.get('aliases', []), entry['frame_id']
            if any(isinstance(entry[k], bool) for k in ('x', 'y', 'yaw')):
                raise ValueError('Boolean coordinates are invalid')
            values = [float(entry[k]) for k in ('x', 'y', 'yaw')]
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('Invalid zone fields') from exc
        if not isinstance(aliases, list) or not isinstance(frame, str) or not frame.strip() or not all(math.isfinite(v) for v in values):
            raise ValueError('Invalid zone coordinates, frame or aliases')
        for key in [name]+aliases:
            if not isinstance(key, str) or not key.strip() or key.strip().casefold() in seen:
                raise ValueError('Empty or duplicate zone name/alias')
            seen.add(key.strip().casefold())
        result.append(Zone(name.strip(), tuple(a.strip() for a in aliases), *values, frame.strip()))
    return result


def lookup(zones, name):
    for zone in zones:
        if name.strip().casefold() in [s.strip().casefold() for s in (zone.name, *zone.aliases)]:
            return zone
    raise ValueError('Unknown zone; valid names: '+', '.join(z.name for z in zones))


def nearest(zones, x, y):
    if not zones:
        raise ValueError('No zones available')
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError('Query coordinates must be finite')
    return min(zones, key=lambda z: math.hypot(z.x-x, z.y-y))


def arrived(zone, pose, age, xy_tolerance=0.15, yaw_tolerance=0.25, max_age=0.5):
    """Validate a fresh localized (x, y, yaw) in the zone frame, never truth."""
    if pose is None:
        return False
    values = (*pose, age, xy_tolerance, yaw_tolerance, max_age)
    if (len(pose) != 3 or not all(math.isfinite(v) for v in values)
            or min(xy_tolerance, yaw_tolerance, max_age) <= 0
            or not 0 <= age <= max_age):
        return False
    x, y, yaw = pose
    return (math.hypot(x-zone.x, y-zone.y) <= xy_tolerance
            and abs(math.atan2(math.sin(yaw-zone.yaw), math.cos(yaw-zone.yaw)))
            <= yaw_tolerance)
