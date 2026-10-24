"""Offline calibration and map geometry behavior without ROS."""
import importlib.util
import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'scripts'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


geom = load('map_geometry_check')
yaw = load('yaw_rate_check')


@pytest.mark.parametrize('magic,raster,maximum,expected', [
    (b'P2', b'0 10 # inline\n32 255\n', 255, [0, 10, 32, 255]),
    (b'P5', bytes([10, 32, 35, 255]), 255, [10, 32, 35, 255]),
    (b'P5', bytes([0, 10, 1, 0, 2, 0, 3, 255]), 1023, [10, 256, 512, 1023]),
])
def test_pgm_comments_binary_whitespace_and_16bit(tmp_path, magic, raster, maximum, expected):
    p = tmp_path/'grid.pgm'
    p.write_bytes(magic+b'\n# comment\n2 2\n'+str(maximum).encode()+b'\n'+raster)
    assert geom.read_pgm(p) == (2, 2, maximum, expected)


@pytest.mark.parametrize('data', [b'P6\n1 1\n255\n\x00', b'P5\n0 1\n255\n',
                                  b'P5\n2 1\n255\n\x00', b'P2\n1 1\n10\n11',
                                  b'P2\n1 1\n10\n0 1', b'P2\n1 1\n10\n'])
def test_bad_pgm(tmp_path, data):
    p = tmp_path/'bad.pgm'
    p.write_bytes(data)
    with pytest.raises(ValueError):
        geom.read_pgm(p)


def test_map_origin_y_flip_negate_and_unknown(tmp_path):
    (tmp_path/'grid.pgm').write_bytes(b'P5\n2 2\n255\n'+bytes([0, 205, 254, 0]))
    p = tmp_path/'grid.yaml'
    p.write_text('image: grid.pgm\nresolution: 0.5\norigin: [1, 2, 1.5707963267948966]\noccupied_thresh: 0.65\n')
    grid = geom.load_grid(p)
    assert grid.occupied == {(0, 0), (1, 1)}
    assert grid.center(0, 0) == pytest.approx((0.25, 2.25))
    p.write_text(p.read_text()+'negate: 1\n')
    assert geom.load_grid(p).occupied == {(0, 1), (1, 0)}


def rectangle_grid(dx=0, dy=0, angle=0, sx=1, sy=1):
    grid = geom.Grid(400, 400, 0.01, (-2, -2, 0), set())
    # Rasterize a thin occupied boundary around a transformed rectangle.
    for row in range(grid.height):
        for col in range(grid.width):
            x, y = grid.center(col, row)
            x, y = x-dx, y-dy
            u, v = math.cos(angle)*x+math.sin(angle)*y, -math.sin(angle)*x+math.cos(angle)*y
            if ((abs(abs(u)-0.5*sx) <= 0.007 and abs(v) <= sy)
                    or (abs(abs(v)-sy) <= 0.007 and abs(u) <= 0.5*sx)):
                grid.occupied.add((col, row))
    return grid


def test_edge_translation_and_missing():
    grid = rectangle_grid(dx=0.12, dy=-0.08)
    r = geom.analyze_box(grid, ('synthetic', 0, 0, 1, 2), 0.3)
    assert r['dx'] == pytest.approx(0.12, abs=0.011)
    assert r['dy'] == pytest.approx(-0.08, abs=0.011)
    assert r['scale_x'] == pytest.approx(1, abs=0.02)
    assert geom.estimate_edge(grid, 0, 1.5, -1, 1)['offset'] is None


def test_edge_rotation_and_scale():
    grid = rectangle_grid(angle=0.03, sx=1.08, sy=0.96)
    r = geom.analyze_box(grid, ('synthetic', 0, 0, 1, 2), 0.3)
    assert r['rotation_deg'] == pytest.approx(math.degrees(0.03), abs=0.5)
    assert r['scale_x'] == pytest.approx(1.08, abs=0.02)
    assert r['scale_y'] == pytest.approx(0.96, abs=0.02)


def test_thin_wall_backface_not_reused():
    grid = geom.Grid(20, 20, 0.05, (0, 0, 0), {(i, 9) for i in range(20)})
    r = geom.analyze_box(grid, ('wall', 0.5, 0.625, 1, 0.2), 0.3)
    assert r['edges']['bottom']['offset'] == pytest.approx(0)
    assert r['edges']['top']['offset'] is None
    assert r['scale_y'] is None


def samples(rate=0.855, speed=0.2):
    return [yaw.Sample(i/50, speed*i/50, 0,
                       math.atan2(math.sin(3+rate*i/50), math.cos(3+rate*i/50))) for i in range(151)]


def test_yaw_unwrap_source_time_and_speed():
    assert yaw.summarize(samples(), 3) == pytest.approx((0.855, 0.2))
    assert yaw.summarize(samples(rate=-0.855), 3)[0] == pytest.approx(-0.855)


@pytest.mark.parametrize('bad', [[], samples()[:20], samples()[::25],
                                list(reversed(samples())), samples()+[samples()[-1]],
                                [yaw.Sample(0, math.nan, 0, 0)]+samples()[1:]])
def test_measurement_rejects_missing_stale_nonfinite_and_clock_reset(bad):
    with pytest.raises(ValueError):
        yaw.summarize(bad, 3)


def test_fit_parameter_direction_and_moving_exclusion():
    fitted = yaw.fit_factor(2, [(0, 0.6, 0.855), (0, -0.4, -0.57), (0.4, 0.3, 0.1)])
    assert fitted['mean_ratio'] == pytest.approx(1.425)
    assert fitted['fixed_wheel_rate_separation'] == pytest.approx(2.85)
    assert fitted['fortress_command_factor'] == pytest.approx(2/1.425)


@pytest.mark.parametrize('rows', [[], [(0, 0.2, 0)], [(0, 0.2, -0.1)], [(0, 0, 0.1)], [(0, 0.2, math.nan)]])
def test_fit_rejects_invalid_or_stalled_rotation(rows):
    with pytest.raises(ValueError):
        yaw.fit_factor(2, rows)


def test_dev_entry():
    assert 'yaw-check) run python3 /repo/scripts/yaw_rate_check.py "$@" ;;' in (ROOT/'scripts/dev.sh').read_text()


@pytest.mark.parametrize('failure,expected', [(KeyboardInterrupt, 130), (RuntimeError, 1)])
def test_runtime_abort_always_publishes_zero(monkeypatch, failure, expected):
    from types import ModuleType, SimpleNamespace
    events = []

    class Twist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0.0)
            self.angular = SimpleNamespace(z=0.0)

    class Publisher:
        def publish(self, msg):
            events.append(('publish', msg.linear.x, msg.angular.z))

    class Node:
        def __init__(self, *a, **kw):
            pass

        def create_publisher(self, *a):
            return Publisher()

        def create_subscription(self, *a):
            return object()

        def destroy_node(self):
            events.append('destroy')

    def spin(*a, **kw):
        if 'failed' not in events:
            events.append('failed')
            raise failure('injected')

    modules = {
        'rclpy': dict(init=lambda **kw: None, spin_once=spin, shutdown=lambda: events.append('shutdown')),
        'rclpy.node': dict(Node=Node),
        'rclpy.parameter': dict(Parameter=lambda *a, **kw: None),
        'rclpy.qos': dict(qos_profile_sensor_data=object()),
        'geometry_msgs.msg': dict(Twist=Twist),
        'nav_msgs.msg': dict(Odometry=object),
        'sensor_msgs.msg': dict(LaserScan=object),
    }
    for name, attrs in modules.items():
        mod = ModuleType(name)
        mod.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, mod)
    assert yaw.main([]) == expected
    stop_events = events[events.index('failed')+1:]
    assert stop_events[:10] == [('publish', 0.0, 0.0)]*10
    assert stop_events[-2:] == ['destroy', 'shutdown']


def test_local_aisle_uses_geometry_and_only_constrains_lateral_position():
    grid = geom.Grid(400, 400, 0.01, (-2, -2, 0), set())
    for row in range(50, 350):
        for col in (129, 289):
            grid.occupied.add((col, row))
    boxes = [('shelf_a', -1.2, 0, 0.8, 3), ('shelf_b', 1.2, 0, 0.8, 3)]
    result = geom.analyze_aisle(grid, boxes, {'name': 'test', 'x': 0, 'y': 0}, 0.3)
    assert result['dx'] == pytest.approx(0.095)
    assert result['scale_x'] == pytest.approx(1)
    assert 'dy' not in result
    assert geom.analyze_aisle(grid, boxes, {'name': 'outside', 'x': 0, 'y': 3}, 0.3) is None
