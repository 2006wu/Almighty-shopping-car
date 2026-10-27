#!/usr/bin/env python3
"""Reproduce static-map inflation costs without ROS; exclude live scan obstacles."""
import ast
import json
import math
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.map_geometry_check import load_grid

NAV = ROOT / 'ros2_ws/src/shopping_robot_navigation'


def inflation_cost(distance_cells, resolution, inscribed, radius, scaling):
    """Match computeCost plus enqueue's ceil(radius / resolution) cutoff."""
    if distance_cells == 0:
        return 254
    if distance_cells > math.ceil(radius / resolution):
        return 0
    distance = distance_cells * resolution
    if distance <= inscribed:
        return 253
    return int(252 * math.exp(-scaling * (distance - inscribed)))


def effective_radius(layer, requested=None):
    """The minimum radius limits support, not the inscribed-cost plateau."""
    return max(layer['inflation_radius'] if requested is None else requested,
               layer['min_inflation_radius'], layer['inscribed_radius_floor'])


def fork_navfn_cost(raw):
    """Match setCostmap: even unknown=255 hits the first branch."""
    return 253 if raw >= 253 else max(1, raw)


def report():
    grid = load_grid(NAV / 'maps/supermarket.yaml')
    assert grid.origin[2] == 0, 'This probe expects an axis-aligned map'
    fork = yaml.safe_load((NAV / 'config/nav2_params_fork.yaml').read_text())
    apt = yaml.safe_load((NAV / 'config/nav2_params.yaml').read_text())
    fork = fork['global_costmap']['global_costmap']['ros__parameters']
    apt = apt['global_costmap']['global_costmap']['ros__parameters']
    layer, baseline = fork['inflation_layer'], apt['inflation_layer']
    # The rectangle's padded half-width is its inscribed radius. Humble's
    # default footprint_padding is 0.01 m when omitted from YAML.
    footprint = ast.literal_eval(apt['footprint'])
    apt_inscribed = min(min(abs(x), abs(y)) for x, y in footprint) + apt.get('footprint_padding', 0.01)
    occupied = {(x, grid.height - 1 - row) for x, row in grid.occupied}
    rows = []
    for name, world, logged in [('household', (0.0, 4.5), (118, 151)),
                                ('drinks', (-2.4, 2.0), (70, 101))]:
        # NavFn worldToMap uses round, unlike Costmap2D's truncation.
        expected = tuple(math.floor((v-o)/grid.resolution + 0.5)
                         for v, o in zip(world, grid.origin))
        for label, cell in [('logged', logged), ('saved_map_world', expected)]:
            distance, nearest = min((math.hypot(cell[0]-x, cell[1]-y), (x, y))
                                    for x, y in occupied)
            args = (distance, grid.resolution)
            rows.append(dict(
                zone=name, coordinates=label, cell=cell, nearest_occupied=nearest,
                distance_m=distance * grid.resolution,
                fork_cost=inflation_cost(*args, layer['inscribed_radius_floor'],
                                         effective_radius(layer), layer['cost_scaling_factor']),
                fork_floor_040_cost=inflation_cost(*args, 0.40, effective_radius(layer),
                                                   layer['cost_scaling_factor']),
                apt_cost=inflation_cost(*args, apt_inscribed, baseline['inflation_radius'],
                                       baseline['cost_scaling_factor']),
                fork_shrink_cost=inflation_cost(*args, layer['inscribed_radius_floor'],
                                                effective_radius(layer, layer['shrink_to']),
                                                layer['cost_scaling_factor'])))
    return dict(resolution=grid.resolution, origin=grid.origin,
                fork_inscribed=layer['inscribed_radius_floor'], apt_inscribed=apt_inscribed,
                normal_radius=effective_radius(layer),
                shrink_radius=effective_radius(layer, layer['shrink_to']), goals=rows)


if __name__ == '__main__':
    print(json.dumps(report(), indent=2))
