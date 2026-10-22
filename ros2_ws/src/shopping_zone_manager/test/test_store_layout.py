"""Check goals and the complete person loop against actual SDF collisions."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml
import pytest
from shopping_zone_manager.zones import load_zones


SRC = Path(__file__).resolve().parents[2]
WORLD = ET.parse(SRC/'shopping_robot_gazebo/worlds/supermarket.sdf').getroot().find('world')
PATH = yaml.safe_load((SRC/'shopping_robot_gazebo/config/person_path.yaml').read_text())


def box(model):
    pose = list(map(float, model.findtext('pose', '0 0 0 0 0 0').split()))
    size = list(map(float, model.findtext('link/collision/geometry/box/size').split()))
    # Fail rather than silently ignoring future collision transforms.
    assert pose[3:] == [0, 0, 0]
    assert model.find('link/pose') is None
    assert model.find('link/collision/pose') is None
    assert len(model.findall('link/collision')) == 1
    return pose, size


OBSTACLES = [box(m) for m in WORLD.findall('model') if m.get('name') not in ('floor', 'person')]
FLOOR_POSE, FLOOR_SIZE = box(WORLD.find("model[@name='floor']"))


def assert_free(x, y, clearance):
    assert abs(x-FLOOR_POSE[0])+clearance < FLOOR_SIZE[0]/2
    assert abs(y-FLOOR_POSE[1])+clearance < FLOOR_SIZE[1]/2
    for pose, size in OBSTACLES:
        dx = max(abs(x-pose[0])-size[0]/2, 0)
        dy = max(abs(y-pose[1])-size[1]/2, 0)
        assert math.hypot(dx, dy) > clearance, (x, y, pose, size)


def test_four_rows_three_wide_aisles_and_entrance():
    assert 10 <= FLOOR_SIZE[0] <= 12 and 8 <= FLOOR_SIZE[1] <= 10
    shelves = sorted((box(m) for m in WORLD.findall('model') if m.get('name').startswith('shelf_')),
                     key=lambda item: item[0][0])
    assert len(shelves) == 4
    for (a, sa), (b, sb) in zip(shelves, shelves[1:]):
        assert b[0]-sb[0]/2-(a[0]+sa[0]/2) >= 1.2
        assert a[1] == b[1] and sa[1] == sb[1]
    # Entrance gap at the south edge must fit the footprint and inflation.
    assert_free(0, -2.3, 0.55)
    assert WORLD.find("model[@name='checkout']") is not None


@pytest.mark.parametrize('zone', load_zones(SRC/'shopping_zone_manager/config/zones.yaml'), ids=lambda z: z.name)
def test_zone_free_with_robot_clearance(zone):
    assert zone.frame_id == 'map'
    assert_free(zone.x, zone.y, 0.55)


def test_spawn_pose_is_origin_and_free():
    launch = (SRC/'shopping_robot_gazebo/launch/sim.launch.py').read_text()
    assert "'-x', '0', '-y', '0', '-z', '0.02', '-Y', '0'" in launch
    assert_free(0, 0, 0.7)
    # Include the laser offset and the full front sector out to 0.7 m.
    for degrees in range(-30, 31):
        angle = math.radians(degrees)
        assert_free(0.18+0.7*math.cos(angle), 0.7*math.sin(angle), 0.01)


def test_entire_person_loop_and_initial_model_pose():
    points = PATH['waypoints']
    initial = list(map(float, WORLD.findtext("model[@name='person']/pose").split()))
    assert points[0] == [1.5, 0.0]
    assert points[1][0] > points[0][0] and points[1][1] == 0
    # The disabled person must be outside the store and the lidar range.
    assert abs(initial[0]-FLOOR_POSE[0]) > FLOOR_SIZE[0]/2+0.25
    assert abs(initial[1]-FLOOR_POSE[1]) > FLOOR_SIZE[1]/2+0.25
    assert math.hypot(*initial[:2])-0.25 > 12
    assert PATH['speed'] > 0
    # Sampling every <=2 cm checks connecting segments and the closing edge too.
    for a, b in zip(points, points[1:]+points[:1]):
        steps = max(1, math.ceil(math.dist(a, b)/0.02))
        for i in range(steps+1):
            t = i/steps
            assert_free(a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]), 0.30)
    for center in (-2.4, 0, 2.4):
        assert any(x == center and 1 < y < 6 for x, y in points)
