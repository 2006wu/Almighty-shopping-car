"""Fault engine, message immutability and conservative check acceptance."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from shopping_robot_sim.faults import FaultEngine, parse_command
from shopping_robot_sim.fault_check import Sample, evaluate_trial, stop_latency


@pytest.mark.parametrize('command', ['uwb_stale', 'uwb_dropout:3.0', 'uwb_noise:0.5',
                                     'lidar_drop', 'lidar_freeze', 'odom_freeze', 'clear'])
def test_commands(command):
    assert parse_command(command).name == command.split(':')[0]


@pytest.mark.parametrize('command', ['', 'oops', 'clear:1', 'uwb_noise',
                                     'uwb_noise:nan', 'uwb_noise:inf',
                                     'uwb_dropout:-1', 'uwb_dropout:0', 'uwb_stale:2'])
def test_reject_commands(command):
    with pytest.raises(ValueError):
        parse_command(command)


def test_schedule_expiry_overlap_clear_and_clock_reset():
    engine = FaultEngine([{'time': 2, 'fault': 'lidar_drop', 'duration': 1},
                          {'time': 1, 'fault': 'odom_freeze', 'duration': 4}])
    engine.advance(0)
    assert not engine.active
    engine.advance(2)
    assert set(engine.active) == {'lidar_drop', 'odom_freeze'}
    engine.advance(3)
    assert set(engine.active) == {'odom_freeze'}
    engine.command('clear', 3)
    assert not engine.active
    engine.advance(0)
    engine.advance(2)
    assert len(engine.active) == 2
    engine.advance(10)
    assert not engine.active


@pytest.mark.parametrize('field,value', [('time', -1), ('duration', float('inf'))])
def test_invalid_schedule(field, value):
    row = dict(time=1, duration=1, fault='lidar_drop')
    row[field] = value
    with pytest.raises(ValueError):
        FaultEngine([row])


@pytest.mark.parametrize('stream,fault', [('target', 'uwb_stale'),
                                         ('scan', 'lidar_freeze'), ('odom', 'odom_freeze')])
def test_freeze_preserves_stamp_and_recovers(stream, fault):
    engine = FaultEngine()
    first = NS(header=NS(stamp=1), data=[2.0])
    engine.command(fault, 1)
    frozen = engine.mutate(stream, first)
    frozen.data[0] = 99
    second = NS(header=NS(stamp=2), data=[3.0])
    assert engine.mutate(stream, second).header.stamp == 1
    assert engine.mutate(stream, second).data == [2.0]
    assert first.data == [2.0]
    engine.command('clear', 2)
    assert engine.mutate(stream, second).header.stamp == 2


def test_dropout_expiry_noise_seed_and_no_input_mutation():
    msg = NS(data=[1.0, 2.0, float('nan')])
    engine = FaultEngine()
    engine.command('uwb_dropout:3', 0)
    assert engine.mutate('ranges', msg) is None
    engine.advance(3)
    assert engine.mutate('ranges', msg) is not None
    engine.command('lidar_drop', 3)
    assert engine.mutate('scan', msg) is None
    other = FaultEngine()
    for instance in (engine, other):
        instance.command('uwb_noise:0.5', 3)
    assert engine.mutate('ranges', msg).data[:2] == other.mutate('ranges', msg).data[:2]
    assert msg.data[:2] == [1.0, 2.0]


def series(start, end, stop=0.6):
    return [Sample(i/10, 0.2 if i/10 < stop else 0.0, 0.0)
            for i in range(round(start*10), round(end*10)+1)]


def trial(command=None, truth=None, recovery=None, noise=False):
    return evaluate_trial(command if command is not None else series(0, 4),
                          truth if truth is not None else series(0, 4, 1.2),
                          series(-1, -0.1, 10), series(-1, -0.1, 10),
                          recovery if recovery is not None else series(5, 6, 10),
                          series(5, 6, 10), 0, 4, noise)


def test_pass_and_stop_thresholds():
    ok, cmd, gt, _ = trial()
    assert ok and cmd == 0.6 and gt == 1.2
    assert not trial(command=series(0, 4, 0.9))[0]
    assert not trial(truth=series(0, 4, 1.6))[0]


@pytest.mark.parametrize('bad', [[], series(0, 1), series(1, 4),
                               [Sample(0, 0, 0), Sample(4, 0, 0)],
                               [Sample(0, float('nan'), 0)]])
def test_missing_or_gapped_telemetry_fails(bad):
    assert not trial(command=bad)[0]


def test_transient_zero_late_motion_and_no_recovery_fail():
    samples = series(0, 4)
    samples[-1] = Sample(4, 0.1, 0)
    assert not trial(command=samples)[0]
    assert not trial(recovery=series(5, 6))[0]
    assert stop_latency([Sample(0, 0, 0), Sample(0, 0, 0)], 0, 4, 0.8) is None


def test_noise_checks_limits_not_stop():
    assert trial(command=series(0, 4, 10), noise=True)[0]
    assert trial(command=[Sample(i/10, 0.6, 0) for i in range(41)], noise=True)[0]
    assert not trial(command=[Sample(i/10, 1.05, 0) for i in range(41)], noise=True)[0]
    assert not trial(command=[Sample(i/10, 0.5, 1.6) for i in range(41)], noise=True)[0]
    assert not trial(command=series(0, 1, 10), noise=True)[0]


def test_launch_opt_in_and_raw_producer_wiring():
    root = Path(__file__).resolve().parents[1]/'ros2_ws/src'
    for package, filename in [('shopping_robot_gazebo', 'sim'),
                               ('shopping_robot_bringup', 'follow')]:
        source = (root/package/f'launch/{filename}.launch.py').read_text()
        tree = ast.parse(source)
        declarations = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                        and isinstance(n.func, ast.Name) and n.func.id == 'DeclareLaunchArgument'
                        and n.args[0].value == 'fault_injection']
        assert len(declarations) == 1
        assert next(k.value.value for k in declarations[0].keywords if k.arg == 'default_value') == 'false'
        assert '/sim/raw' in source
    sim = (root/'shopping_robot_gazebo/launch/sim.launch.py').read_text()
    assert "condition=IfCondition(fault)" in sim
    assert "('/scan', output('/scan'))" in sim
    assert "('/odom', output('/odom'))" in sim


def test_already_stationary_baseline_cannot_pass():
    still = series(-1, -0.1, -2)
    assert not evaluate_trial(series(0, 4), series(0, 4), still, still,
                              series(5, 6, 10), series(5, 6, 10), 0, 4)[0]


def test_phantom_front_preserves_stamp_and_other_rays():
    engine = FaultEngine()
    engine.command('lidar_front:0.2', 0)
    scan = NS(ranges=[5.0]*720, angle_min=-3.141592653589793,
              angle_increment=2*3.141592653589793/720, header=NS(stamp=123))
    changed = engine.mutate('scan', scan)
    assert changed.ranges[360] == 0.2
    assert changed.ranges[0] == 5.0
    assert changed.header.stamp == 123
    assert scan.ranges[360] == 5.0
    engine.command('clear', 1)
    assert engine.mutate('scan', scan).ranges[360] == 5.0


@pytest.mark.parametrize('command', ['lidar_front', 'lidar_front:nan', 'lidar_front:0'])
def test_invalid_phantom_distance(command):
    with pytest.raises(ValueError):
        parse_command(command)


def test_translation_baseline_excludes_in_place_rotation():
    from shopping_robot_sim.fault_check import moving
    turning = Sample(0, 0, .2)
    assert moving(turning) and not moving(turning, translation_only=True)
    assert moving(Sample(0, .1, 0), translation_only=True)


def test_stop_check_detects_truth_motion_along_either_planar_axis():
    from shopping_robot_sim.fault_check import planar_sample
    for vx, vy in ((.1, 0), (0, .1), (0, -.1)):
        samples = [planar_sample(i*.1, vx, vy, 0) for i in range(41)]
        assert stop_latency(samples, 0, 4, .8) is None


def test_noise_bounds_track_follow_config():
    import yaml
    from shopping_robot_sim import fault_check
    root = Path(__file__).resolve().parents[1]
    cfg = yaml.safe_load((root/'ros2_ws/src/shopping_uwb_follow/config/uwb_follow.yaml').read_text())
    params = next(v['ros__parameters'] for v in cfg.values() if 'max_linear' in v.get('ros__parameters', {}))
    assert fault_check.NOISE_MAX_LINEAR == pytest.approx(params['max_linear']+0.01)
    assert fault_check.NOISE_MAX_ANGULAR == pytest.approx(params['max_angular']+0.01)
