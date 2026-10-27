"""Conservative geometry, fail-safe and hysteresis contracts without ROS."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from shopping_zone_manager.obstacle_guard import ObstacleGuard, scan_points


def ready(points, velocity=(0.4, 0.0)):
    guard = ObstacleGuard()
    guard.step(points, velocity, 0)
    return guard, guard.step(points, velocity, 0.31)


def test_speed_direction_and_slow_zone():
    assert ready([(0.70, 0)], (0.6, 0))[1].state == 'STOP'
    assert ready([(0.70, 0)], (0.1, 0))[1].state == 'SLOW'
    assert ready([(0.70, 0)], (-0.4, 0))[1].state == 'CLEAR'
    assert ready([(-0.70, 0)], (-0.6, 0))[1].state == 'STOP'
    assert ready([(0.7, 1.2)])[1].state == 'CLEAR'


def test_rotation_disk_and_curved_sweep():
    assert ready([(0.6, 0)], (0, 0.8))[1].velocity == (0, 0.8)
    assert ready([(0.32, 0.25)], (0, -0.8))[1].state == 'STOP'
    # Point on the commanded turning arc, outside the initial footprint.
    assert ready([(0.55, 0.45)], (0.6, 1.2))[1].state == 'STOP'


def test_safe_rotation_releases_prior_translation_slowdown_without_lowering_margin():
    guard, result = ready([(0.7, 0)], (0.1, 0))
    assert result.state == 'SLOW'
    assert guard.step([(0.6, 0)], (0, .22), .32).state == 'SLOW'
    assert guard.step([(0.6, 0)], (0, .22), .63).velocity == (0, .22)
    radius = math.hypot(guard.half_length, guard.half_width)
    assert guard.step([(radius+.09, 0)], (0, .22), .64).velocity == (0, 0)
    assert guard.step([(radius+.14, 0)], (0, .22), 1.).state == 'STOP'
    assert guard.step([(radius+.14, 0)], (0, .22), 2.).state == 'STOP'
    assert guard.step([(radius+.16, 0)], (0, .22), 3.).state == 'STOP'
    assert guard.step([(radius+.16, 0)], (0, .22), 3.31).velocity == (0, .22)
    assert guard.step([(radius+.3, 0)], (0, .22), 3.32, fresh=False).state == 'STALE'


def test_immediate_stop_delayed_wider_release_and_stale():
    guard, result = ready([])
    assert result.state == 'CLEAR'
    assert guard.step([(0.4, 0)], (0.4, 0), 0.32).velocity == (0, 0)
    assert guard.step([], (0.4, 0), 0.33).state == 'STOP'
    assert guard.step([(0.4, 0)], (0.4, 0), 0.5).state == 'STOP'
    assert guard.step([], (0.4, 0), 0.6).state == 'STOP'
    assert guard.step([], (0.4, 0), 0.91).state == 'CLEAR'
    assert guard.step([], (0.4, 0), 0.92, False).state == 'STALE'
    assert guard.step([], (0.4, 0), 0.93).velocity == (0, 0)
    assert guard.step([], (0.4, 0), 0.2).state == 'STALE'
    assert guard.step([], (math.nan, 0), 1).velocity == (0, 0)


def test_hysteresis_does_not_release_near_boundary():
    guard = ObstacleGuard()
    for t in (0, 1, 2):
        assert guard.step([(0.42, 0)], (0, 0), t).state == 'STOP'


def test_scan_transform_and_validation():
    ranges = [math.inf]*720
    ranges[360] = 1
    points = scan_points(ranges, -math.pi, 2*math.pi/720, .12, 12, (.18, 0, 0))
    assert points == pytest.approx([(1.18, 0)])
    for value in (math.nan, -math.inf, 0, 13):
        ranges[0] = value
        with pytest.raises(ValueError):
            scan_points(ranges, -math.pi, .01, .12, 12, (0, 0, 0))
    with pytest.raises(ValueError):
        scan_points([math.inf]*100, 0, .01, .12, 12, (0, 0, 0))


def test_footprint_matches_description_and_nav2():
    root = Path(__file__).resolve().parents[4]
    if not (root/'docs').is_dir():
        root = Path('/repo')
    if not (root/'docs').is_dir():
        pytest.skip('Repository sources are not mounted')
    src = root/'ros2_ws/src'
    model = ET.parse(src/'shopping_robot_description/urdf/shopping_robot.urdf.xacro')
    props = {e.attrib['name']: e.attrib['value'] for e in model.getroot()
             if e.tag.endswith('property')}
    g = ObstacleGuard()
    assert float(props['length']) == 2*g.half_length
    assert float(props['width']) == 2*g.half_width
    nav = yaml.safe_load((src/'shopping_robot_navigation/config/nav2_params.yaml').read_text())
    footprint = yaml.safe_load(nav['local_costmap']['local_costmap']['ros__parameters']['footprint'])
    assert max(abs(x) for x, y in footprint) == g.half_length
    assert max(abs(y) for x, y in footprint) == g.half_width


def test_final_recovery_limiter_preserves_curvature_and_immediate_clamps():
    from shopping_zone_manager.mode_control import SafetyVelocityLimiter
    limiter = SafetyVelocityLimiter()
    assert limiter.step((0.5, 1), 0) == (0, 0)
    v, w = limiter.step((0.5, 1), .1)
    assert 0 < v <= .08 and w == 2*v
    v, w = limiter.step((.01, .02), .15)
    assert (v, w) == (.01, .02)
    assert limiter.step((0, 0), .16) == (0, 0)
    v, w = limiter.step((-.5, 1), .2)
    assert -.0301 <= v < 0 and w == -2*v


@pytest.mark.parametrize('now,stamp,wall,received,expected', [
    (1, 1, 2, 2, True), (1, .49, 2, 2, False),
    (1, 1, 2, 1.49, False), (.5, 1, 2, 2, False),
    (1, 1, 1, 2, False), (1, 1, 2, None, False),
    (1, math.nan, 2, 2, False)])
def test_scan_header_freeze_delivery_timeout_and_clock_reset(now, stamp, wall, received, expected):
    from shopping_zone_manager.obstacle_guard import scan_fresh
    assert scan_fresh(now, stamp, wall, received) is expected


def test_load_nav_footprint_and_reject_unsupported_shapes(tmp_path):
    from shopping_zone_manager.obstacle_guard import load_nav_footprint
    def write(local, global_):
        config = {name: {name: {'ros__parameters': {'footprint': value}}}
                  for name, value in (('local_costmap', local), ('global_costmap', global_))}
        path = tmp_path/'nav.yaml'
        path.write_text(yaml.safe_dump(config))
        return path
    box = [[.4, .3], [.4, -.3], [-.4, -.3], [-.4, .3]]
    assert load_nav_footprint(write(str(box), box)) == (.4, .3)
    for invalid in (box[:3], [[1, 1]]*4, [[True, 1]]*4,
                    [[math.nan, 1]]*4, [[.4, .3], [.5, -.3], [-.4, -.3], [-.4, .3]]):
        with pytest.raises(ValueError):
            load_nav_footprint(write(invalid, invalid))
    with pytest.raises(ValueError):
        load_nav_footprint(write(box, [[.3, .3], [.3, -.3], [-.3, -.3], [-.3, .3]]))
