from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from shopping_zone_manager.nav_check_logic import (
    final_position_detail, planner_tolerance, outside_distance, store_bounds,
    unreachable_clearances,
)
from shopping_zone_manager.zones import Zone


SRC = Path(__file__).resolve().parents[2]
WORLD = SRC / 'shopping_robot_gazebo/worlds/supermarket.sdf'
CONFIG = SRC / 'shopping_robot_navigation/config/nav2_params.yaml'
ZONE = Zone('drinks', (), -2.4, 2.0, 0.0, 'map')


def test_unreachable_goal_with_current_world_and_planner():
    bounds = store_bounds(WORLD)
    assert bounds == pytest.approx((-6.1, -3.1, 6.1, 7.1))
    tolerance = planner_tolerance(CONFIG)
    distances = unreachable_clearances(50, 50, tolerance, bounds, (-10, -10, 10, 10))
    assert min(distances) > tolerance + 1
    # The bounding rectangle encloses every cell, including free and unknown cells.
    assert outside_distance(50, 50, (-10, -10, 10, 10)) == pytest.approx(40 * 2**0.5)


@pytest.mark.parametrize('plugin', ['nav2_navfn_planner/NavfnPlanner', 'nav2_smac_planner/SmacPlanner2D'])
@pytest.mark.parametrize('tolerance', [0.3, 0.8, 2.0])
def test_tolerance_is_loaded_from_selected_grid_plugin(tmp_path, tolerance, plugin):
    config = yaml.safe_load(CONFIG.read_text())
    params = config['planner_server']['ros__parameters']
    params['planner_plugins'] = ['RenamedPlanner']
    params['RenamedPlanner'] = params.pop('GridBased')
    params['RenamedPlanner']['tolerance'] = tolerance
    params['RenamedPlanner']['plugin'] = plugin
    path = tmp_path / 'params.yaml'
    path.write_text(yaml.safe_dump(config))
    assert planner_tolerance(path) == tolerance
    unreachable_clearances(50, 50, planner_tolerance(path), store_bounds(WORLD), (-8, -8, 8, 8))


@pytest.mark.parametrize('tolerance', [-1, float('nan'), float('inf')])
def test_invalid_planner_tolerance(tmp_path, tolerance):
    config = yaml.safe_load(CONFIG.read_text())
    config['planner_server']['ros__parameters']['GridBased']['tolerance'] = tolerance
    path = tmp_path / 'params.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):
        planner_tolerance(path)


@pytest.mark.parametrize('bounds', [(-6, -3, 6, 7), (-100, -100, 100, 100),
                                  (49, 49, 49.5, 49.5)])
def test_unreachable_goal_rejects_nearby_store_or_costmap(bounds):
    remote = (-10, -10, 10, 10)
    # The first rectangle contains a store goal; the others contain or approach (50, 50).
    x, y = (0, 0) if bounds[0] == -6 else (50, 50)
    for world, costmap in [(bounds, remote), (remote, bounds)]:
        with pytest.raises(ValueError, match='tolerance'):
            unreachable_clearances(x, y, 0.8, world, costmap)


def test_unreachable_margin_is_strict_and_tolerance_changes_are_honoured():
    bounds = (-1, -1, 1, 1)
    with pytest.raises(ValueError):
        unreachable_clearances(2.8, 0, 0.8, bounds, bounds)
    with pytest.raises(ValueError):
        unreachable_clearances(50, 50, 100, bounds, bounds)


@pytest.mark.parametrize('mutation', ['rotated', 'link_pose', 'collision_pose'])
def test_changed_wall_geometry_fails_closed(tmp_path, mutation):
    tree = ET.parse(WORLD)
    model = tree.getroot().find("world/model[@name='north']")
    if mutation == 'rotated':
        model.find('pose').text = '0 7 1 0 0 0.2'
    else:
        parent = model.find('link' if mutation == 'link_pose' else 'link/collision')
        ET.SubElement(parent, 'pose').text = '1 0 0 0 0 0'
    path = tmp_path / 'world.sdf'
    tree.write(path)
    with pytest.raises(ValueError, match='wall boxes'):
        store_bounds(path)


def test_reachable_zone_final_error_and_custom_limit():
    detail = final_position_detail(-2.1, 2.0, ZONE, 0.35)
    assert 'final error=0.300 m' in detail
    assert 'limit=0.350 m' in detail
    with pytest.raises(ValueError, match='final error=0.800 m'):
        final_position_detail(-1.6, 2.0, ZONE, 0.35)
    assert 'final error=0.800 m' in final_position_detail(-1.6, 2.0, ZONE, 0.9)


@pytest.mark.parametrize('x,y,limit', [(float('nan'), 0, 0.35), (0, float('inf'), 0.35),
                                     (0, 0, 0), (0, 0, float('inf'))])
def test_invalid_final_measurements_cannot_pass(x, y, limit):
    with pytest.raises(ValueError):
        final_position_detail(x, y, ZONE, limit)


def test_final_error_requires_map_zone():
    with pytest.raises(ValueError, match='map-frame'):
        final_position_detail(0, 0, Zone('bad', (), 0, 0, 0, 'odom'), 0.35)
