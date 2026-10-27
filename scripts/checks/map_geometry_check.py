#!/usr/bin/env python3
"""Compare saved map cell centers with axis-aligned SDF collision faces (no ROS)."""
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / 'ros2_ws/src'


def read_pgm(path):
    """Read one P2/P5 image, including big-endian 16-bit P5 samples."""
    data = Path(path).read_bytes()
    pos = 0

    def token():
        nonlocal pos
        while pos < len(data):
            if data[pos:pos+1] == b'#':
                end = data.find(b'\n', pos)
                pos = len(data) if end < 0 else end+1
            elif data[pos] in b' \t\r\n\v\f':
                pos += 1
            else:
                break
        start = pos
        while pos < len(data) and data[pos] not in b' \t\r\n\v\f#':
            pos += 1
        if start == pos:
            raise ValueError('Truncated PGM header or pixels')
        return data[start:pos]

    magic = token()
    if magic not in (b'P2', b'P5'):
        raise ValueError('Expected P2 or P5 PGM')
    width, height, maximum = (int(token()) for _ in range(3))
    if width <= 0 or height <= 0 or not 1 <= maximum <= 65535:
        raise ValueError('Invalid PGM dimensions or maximum')
    count = width*height
    if magic == b'P2':
        pixels = [int(token()) for _ in range(count)]
        # Permit trailing whitespace/comments but reject extra samples.
        try:
            token()
        except ValueError:
            pass
        else:
            raise ValueError('Extra PGM samples')
    else:
        # Consume exactly the raster separator, never strip binary pixel bytes.
        if pos >= len(data) or data[pos] not in b' \t\r\n\v\f':
            raise ValueError('Missing P5 raster separator')
        pos += 2 if data[pos:pos+2] == b'\r\n' else 1
        size = 1 if maximum < 256 else 2
        raster = data[pos:]
        if len(raster) != count*size:
            raise ValueError('Wrong P5 raster length')
        pixels = [int.from_bytes(raster[i:i+size], 'big')
                  for i in range(0, len(raster), size)]
    if any(p < 0 or p > maximum for p in pixels):
        raise ValueError('PGM sample outside declared range')
    return width, height, maximum, pixels


@dataclass
class Grid:
    width: int
    height: int
    resolution: float
    origin: tuple
    occupied: set

    def center(self, col, row):
        x, y = (col+0.5)*self.resolution, (self.height-row-0.5)*self.resolution
        ox, oy, yaw = self.origin
        return (ox+math.cos(yaw)*x-math.sin(yaw)*y,
                oy+math.sin(yaw)*x+math.cos(yaw)*y)


def load_grid(path):
    path = Path(path)
    meta = yaml.safe_load(path.read_text())
    w, h, maximum, pixels = read_pgm(path.parent/meta['image'])
    resolution = float(meta['resolution'])
    origin = tuple(map(float, meta['origin']))
    if resolution <= 0 or not math.isfinite(resolution) or len(origin) != 3 or not all(map(math.isfinite, origin)):
        raise ValueError('Invalid map coordinates')
    if meta.get('mode', 'trinary') != 'trinary':
        raise ValueError('Only trinary occupancy maps are supported')
    occupied = {(i % w, i // w) for i, p in enumerate(pixels)
                if (p/maximum if meta.get('negate', 0) else 1-p/maximum) > meta['occupied_thresh']}
    return Grid(w, h, resolution, origin, occupied)


def load_boxes(path):
    boxes = []
    world = ET.parse(path).getroot().find('world')
    for model in world.findall('model'):
        name = model.get('name')
        if name in ('floor', 'person'):
            continue
        pose = list(map(float, model.findtext('pose', '0 0 0 0 0 0').split()))
        link = model.find('link')
        collisions = link.findall('collision')
        if pose[3:] != [0, 0, 0] or link.find('pose') is not None or len(collisions) != 1 or collisions[0].find('pose') is not None:
            raise ValueError(f'Unsupported collision transform: {name}')
        size = list(map(float, collisions[0].findtext('geometry/box/size').split()))
        boxes.append((name, pose[0], pose[1], size[0], size[1]))
    return boxes


def line_fit(points):
    """Return intercept, slope and residual RMS for (independent, dependent) pairs."""
    if len(points) < 3:
        return None
    mx, my = (statistics.mean(p[i] for p in points) for i in (0, 1))
    variance = sum((x-mx)**2 for x, _ in points)
    if variance < 1e-12:
        return None
    slope = sum((x-mx)*(y-my) for x, y in points)/variance
    intercept = my-slope*mx
    rms = math.sqrt(statistics.mean((y-intercept-slope*x)**2 for x, y in points))
    return intercept, slope, rms


def estimate_edge(grid, axis, face, low, high, search=0.3):
    """Nearest occupied center per tangential cell bin; ignore corners and missing bins.

    Offsets use occupied centers, so rasterization contributes about half a cell.
    A missing/occluded face is reported as missing rather than filled with zeros.
    """
    if axis not in (0, 1) or search <= 0:
        raise ValueError('Invalid edge search')
    trim = min(0.15, (high-low)/4)
    bins = {}
    for col, row in grid.occupied:
        p = grid.center(col, row)
        normal, tangent = p[axis], p[1-axis]
        if low+trim <= tangent <= high-trim and abs(normal-face) <= search:
            key = math.floor(tangent/grid.resolution)
            candidate = (tangent, normal-face)
            if key not in bins or abs(candidate[1]) < abs(bins[key][1]):
                bins[key] = candidate
    points = list(bins.values())
    fit = line_fit(points)
    coverage = min(1.0, len(points)*grid.resolution/(high-low-2*trim))
    if fit is None:
        return {'offset': None, 'samples': len(points), 'coverage': coverage}
    intercept, slope, rms = fit
    return {'offset': statistics.median(p[1] for p in points),
            'center_offset': intercept+slope*(low+high)/2,
            'rotation_deg': math.degrees(math.atan(-slope if axis == 0 else slope)),
            'rms': rms, 'samples': len(points), 'coverage': coverage}


def analyze_box(grid, box, search):
    name, x, y, width, height = box
    edges = {}
    for label, axis, face, low, high in (
            ('left', 0, x-width/2, y-height/2, y+height/2),
            ('right', 0, x+width/2, y-height/2, y+height/2),
            ('bottom', 1, y-height/2, x-width/2, x+width/2),
            ('top', 1, y+height/2, x-width/2, x+width/2)):
        edges[label] = estimate_edge(grid, axis, face, low, high, min(search, (width if axis == 0 else height)/2-grid.resolution/2))
    result = {'name': name, 'edges': edges}
    # Require both faces and reasonable coverage for region translation/size estimates.
    for axis, a, b, size in (('x', 'left', 'right', width), ('y', 'bottom', 'top', height)):
        ea, eb = edges[a], edges[b]
        valid = all(e['offset'] is not None and e['coverage'] >= 0.5 for e in (ea, eb))
        result[f'd{axis}'] = (ea['center_offset']+eb['center_offset'])/2 if valid else None
        result[f'scale_{axis}'] = 1+(eb['center_offset']-ea['center_offset'])/size if valid else None
    angles = [e['rotation_deg'] for e in edges.values() if e['offset'] is not None and e['coverage'] >= 0.5]
    result['rotation_deg'] = statistics.median(angles) if angles else None
    return result


def analyze_aisle(grid, boxes, zone, search):
    """Measure local lateral geometry only; long parallel shelves do not constrain y."""
    x, y = zone['x'], zone['y']
    faces = []
    for name, bx, by, width, height in boxes:
        if name.startswith('shelf_') and by-height/2 <= y-0.35 and y+0.35 <= by+height/2:
            faces.extend((bx-width/2, bx+width/2))
    left = max((f for f in faces if 0 < x-f < 1.5), default=None)
    right = min((f for f in faces if 0 < f-x < 1.5), default=None)
    if left is None or right is None:
        return None
    edges = [estimate_edge(grid, 0, f, y-0.5, y+0.5, search) for f in (left, right)]
    valid = all(e['offset'] is not None and e['coverage'] >= 0.5 for e in edges)
    return {'name': zone['name'], 'left': edges[0], 'right': edges[1],
            'dx': sum(e['center_offset'] for e in edges)/2 if valid else None,
            'rotation_deg': statistics.mean(e['rotation_deg'] for e in edges) if valid else None,
            'scale_x': 1+(edges[1]['center_offset']-edges[0]['center_offset'])/(right-left) if valid else None}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=SRC/'shopping_robot_navigation/maps/supermarket.yaml')
    parser.add_argument('--world', type=Path, default=SRC/'shopping_robot_gazebo/worlds/supermarket.sdf')
    parser.add_argument('--zones', type=Path, default=SRC/'shopping_zone_manager/config/zones.yaml')
    parser.add_argument('--search', type=float, default=0.3)
    parser.add_argument('--json', type=Path)
    args = parser.parse_args(argv)
    if not math.isfinite(args.search) or args.search <= 0:
        parser.error('--search must be finite and positive')
    grid = load_grid(args.map)
    boxes = load_boxes(args.world)
    results = [analyze_box(grid, b, args.search) for b in boxes]
    aisles = [r for z in yaml.safe_load(args.zones.read_text())['zones']
              if (r := analyze_aisle(grid, boxes, z, args.search)) is not None]
    def fmt(v):
        return 'missing' if v is None else f'{v:+.3f}'
    print('Occupied cell-center offsets (m); rotation (deg); scale is mapped/true size.')
    print('region               left   right  bottom     top      dx      dy     rot      sx      sy')
    for r in results:
        values = [r['edges'][s]['offset'] for s in ('left', 'right', 'bottom', 'top')]
        values += [r[k] for k in ('dx', 'dy', 'rotation_deg', 'scale_x', 'scale_y')]
        print(f"{r['name']:20s} " + ' '.join(f'{fmt(v):>7s}' for v in values))
        print('  coverage L/R/B/T: ' + '/'.join(f"{r['edges'][s]['coverage']:.0%}" for s in ('left', 'right', 'bottom', 'top')))
    print('Local zone aisle fits (lateral only; longitudinal translation is unconstrained):')
    for r in aisles:
        print(f"  {r['name']}: dx={fmt(r['dx'])} m, rotation={fmt(r['rotation_deg'])} deg, scale_x={fmt(r['scale_x'])}")
    print('Missing/outer wall faces, occlusion, thick/double edges and nearest-edge association limit inference.')
    if args.json:
        args.json.write_text(json.dumps({'resolution': grid.resolution, 'regions': results, 'aisles': aisles}, indent=2, allow_nan=False)+'\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
