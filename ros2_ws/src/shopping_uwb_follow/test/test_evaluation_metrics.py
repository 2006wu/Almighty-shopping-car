"""Synthetic-trace checks for live simulation measurement tools."""
import ast
import importlib
import math
from pathlib import Path
import random
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

from shopping_uwb_follow.evaluation_metrics import (
    Trace, best_shift, coverage, event_row, follow_report, frame_valid,
    near_stop, reacquire_events, relative_position, segment_kind, sync_report,
)
from shopping_uwb_follow.evaluation_node import parser, result_code

# Package-relative so the test also runs from colcon (/ws/src) where the repo root is absent.
SRC = Path(__file__).resolve().parents[2]
ROOT = SRC.parents[1]
PATH = {'speed': 1.0, 'waypoints': [[0, 0], [4, 0], [4, 4], [0, 4]]}


@pytest.mark.parametrize('lag', [-0.23, 0.0, 0.17, 0.31])
def test_known_noisy_shift(lag):
    rng = random.Random(27)
    def signal(t):
        return math.sin(1.7*t)+0.3*math.cos(0.43*t), math.cos(0.9*t)
    reference = [(i/50, signal(i/50)) for i in range(600)]
    observed = [(i/20, tuple(v+rng.gauss(0, 0.006) for v in signal(i/20-lag)))
                for i in range(240)]
    value = best_shift(reference, observed)
    assert value['identifiable']
    assert value['lag_s'] == pytest.approx(lag, abs=0.011)
    assert value['rmse_m'] < 0.02


def test_unidentifiable_lag_and_gaps():
    assert not best_shift([], [])['identifiable']
    flat = [(i/10, (1.0, 2.0)) for i in range(100)]
    assert best_shift(flat, flat)['reason'] == 'insufficient motion'
    signal = [(i/10, (i/10, 0)) for i in range(100)]
    delayed = [(t, (v[0]-1, 0)) for t, v in signal]
    assert best_shift(signal, delayed)['reason'] == 'boundary optimum'
    assert not best_shift(signal[::10], signal)['identifiable']
    with pytest.raises(ValueError):
        best_shift(signal, signal, step=0)
    with pytest.raises(ValueError):
        best_shift(signal, [(t, (1,)) for t, _ in signal])


def test_trace_rejects_invalid_data_and_no_extrapolation():
    for values in ([(1, (0,)), (1, (1,))], [(1, (math.nan,))], [(1, (1, 2)), (2, (1,))]):
        with pytest.raises(ValueError):
            Trace(values)
    trace = Trace([(1, (0, 0)), (1.2, (2, 4)), (2, (3, 5))])
    assert trace.at(1.1) == pytest.approx((1, 2))
    assert trace.at(0.9) is None
    assert trace.at(1.5) is None
    assert trace.at(2.1) is None
    yaw = Trace([(1, (0, 0, 3.1)), (1.2, (0, 0, -3.1))])
    assert yaw.at(1.1, yaw=True)[2] == pytest.approx(math.pi)


def test_segment_classification_at_corners_and_cycle():
    assert segment_kind(PATH, 2.0) == 'straight'
    assert segment_kind(PATH, 4.0) == 'turn'
    assert segment_kind(PATH, 4.2) == 'turn'
    assert segment_kind(PATH, 16.0) == 'turn'
    assert segment_kind(PATH, 0) == 'unknown'
    assert relative_position((1, 3), (1, 1, math.pi/2)) == pytest.approx((2, 0))


def test_near_stop_deadline_hold_and_censoring():
    safe = [(i/10, 0.4 if i < 10 else 1.0, 0.2 if i < 4 else 0.0, 0.0) for i in range(12)]
    event = near_stop([(-0.1, 1.0, 0.2, 0.0)]+safe, 0.6)[0]
    assert event['status'] == 'PASS'
    assert event['first_stop_s'] == 0.4
    unsafe = [(t, d, 0.1 if t == 0.8 else v, w) for t, d, v, w in safe]
    assert near_stop(unsafe, 0.6)[0]['violation_times_s'] == [0.8]
    rotating = [(t, d, v, 0.1) for t, d, v, _ in safe]
    assert near_stop(rotating, 0.6)[0]['status'] == 'FAIL'
    assert near_stop(safe, 0.6)[0]['status'] == 'NOT_COVERED'
    assert near_stop(safe[:3]+safe[10:], 0.6)[0]['status'] == 'NOT_COVERED'
    assert near_stop(safe[:10], 0.6)[0]['status'] == 'NOT_COVERED'
    assert near_stop([s for s in safe if not 0.3 <= s[0] <= 0.7], 0.6)[0]['status'] == 'NOT_COVERED'
    assert near_stop([(0, 0.6, 1, 0)], 0.6) == []
    assert event_row('near', [])['status'] == 'NOT_COVERED'


def test_reacquire_fresh_to_following_and_deduplication():
    targets = [i/10 for i in range(31) if not 0.5 <= i/10 < 1.5]
    states = [(i/10, 'FOLLOWING' if i < 9 or i >= 19 else 'LOST') for i in range(31)]
    events = reacquire_events(targets, states, 3.0)
    assert len(events) == 1
    event = events[0]
    assert event['fresh_s'] == 1.5
    assert event['following_s'] == 1.9
    assert event['reacquire_s'] == pytest.approx(0.4)
    assert event['status'] == 'PASS'
    assert event_row('reacquire', events, 0.3)['status'] == 'FAIL'
    assert event_row('reacquire', events, 0.5)['status'] == 'PASS'
    assert reacquire_events(targets, states[:15], 3.0)[0]['status'] == 'NOT_COVERED'
    assert reacquire_events([0, 0.1], [(0, 'FOLLOWING'), (1, 'LOST')], 2)[0]['following_s'] is None
    assert reacquire_events([0, 0.1, 0.2], [(0, 'FOLLOWING')], 0.3) == []


@pytest.mark.parametrize('kind,parent,child', [
    ('person', 'sim_world', None), ('target', 'base_link', None),
    ('robot', 'sim_world', 'base_footprint_gt'), ('model', 'supermarket', 'person')])
def test_frames(kind, parent, child):
    assert frame_valid(kind, parent, child)
    for incorrect in ('odom', 'map', '', 'world'):
        assert not frame_valid(kind, incorrect, child)
    assert not frame_valid('model', 'supermarket', 'person::body')


def test_sync_reports_detect_model_offset_and_latency():
    from shopping_uwb_follow.person_path import path_position
    truth = [(i/10+1, path_position(PATH, i/10+1)) for i in range(120)]
    assert all(row['status'] == 'PASS' for row in sync_report(truth, truth, PATH))
    shifted = [(t, (p[0]+0.2, p[1])) for t, p in truth]
    rows = sync_report(truth, shifted, PATH)
    assert next(r for r in rows if r['check'] == 'model_vs_truth')['status'] == 'FAIL'
    delayed = [(t, path_position(PATH, t-0.2)) for t, _ in truth]
    row = next(r for r in sync_report(truth, delayed, PATH) if r['check'] == 'model_vs_truth_lag')
    assert row['status'] == 'FAIL'
    assert row['details']['lag_s'] == pytest.approx(0.2, abs=0.01)
    assert all(r['status'] == 'FAIL' for r in sync_report([], [], PATH))


def test_follow_truth_independent_of_estimator_and_missing_scenarios():
    from shopping_uwb_follow.person_path import path_position
    person = [(i/20+1, path_position(PATH, i/20+1)) for i in range(200)]
    robot = [(t, (p[0]-1, p[1], 0, 0.2, 0)) for t, p in person]
    target = [(t, (1, 0)) for t, _ in person]
    rows = follow_report(person, robot, target, [], PATH,
                         {'follow_distance': 1, 'stop_distance': 0.6, 'lost_timeout': 0.5}, 11)
    for kind in ('turn_tracking', 'straight_tracking'):
        row = next(r for r in rows if r['check'] == kind)
        assert row['status'] == 'PASS'
        assert row['details']['distance_m']['mean'] == 1
    assert next(r for r in rows if r['check'] == 'filter_latency')['status'] == 'FAIL'
    assert next(r for r in rows if r['check'] == 'near_stop')['status'] == 'NOT_COVERED'
    assert next(r for r in rows if r['check'] == 'reacquire')['status'] == 'NOT_COVERED'
    assert coverage(person, 1, 11)
    assert not coverage(person[:10]+person[20:], 1, 11)
    assert result_code([{'status': 'PASS'}]) == 0
    assert result_code([{'status': 'NOT_COVERED'}]) == 2
    assert result_code([{'status': 'NOT_COVERED'}, {'status': 'FAIL'}]) == 1
    # Truth safety evidence must survive complete estimator loss.
    near_person = [(i/10, (1.0 if i < 2 or i >= 15 else 0.4, 0)) for i in range(20)]
    moving_robot = [(t, (0, 0, 0, 0.2, 0)) for t, _ in near_person]
    failed = follow_report(near_person, moving_robot, [], [], PATH,
                           {'follow_distance': 1, 'stop_distance': 0.6, 'lost_timeout': 0.5}, 2)
    assert next(r for r in failed if r['check'] == 'near_stop')['status'] == 'FAIL'
    # Fully observed failure to return to FOLLOWING is a failure, not censoring.
    times = [i/10 for i in range(100)]
    interrupted = [(t, (1, 0)) for t in times if t < 1 or t >= 2]
    lost = [(t, 'FOLLOWING' if t < 1.4 else 'LOST') for t in times]
    failed = follow_report([], [], interrupted, lost, PATH,
                           {'follow_distance': 1, 'stop_distance': 0.6, 'lost_timeout': 0.5}, 10)
    assert next(r for r in failed if r['check'] == 'reacquire')['status'] == 'FAIL'



def test_cli_entry_points_and_dev_forwarding():
    setup = ast.parse((SRC/'shopping_uwb_follow/setup.py').read_text())
    call = next(n for n in ast.walk(setup) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'setup')
    entries = ast.literal_eval(next(k.value for k in call.keywords if k.arg == 'entry_points'))['console_scripts']
    for command in ('person_sync_check', 'follow_eval'):
        entry = next(e for e in entries if e.startswith(command+' = '))
        module, function = entry.split(' = ')[1].split(':')
        assert callable(getattr(importlib.import_module(module), function))
    dev_path = ROOT/'scripts/dev.sh'
    if not dev_path.exists():
        dev_path = Path('/repo/scripts/dev.sh')
    if not dev_path.exists():
        pytest.skip('repository scripts are not mounted')
    dev = dev_path.read_text()
    for command, executable in [('person-sync-check', 'person_sync_check'), ('follow-eval', 'follow_eval')]:
        assert f'{command}) run ros2 run shopping_uwb_follow {executable} "$@" --ros-args -p use_sim_time:=true ;;' in dev
    for mode in ('sync', 'follow'):
        assert parser(mode).parse_args(['--duration', '10', '--output', '/tmp/report.json']).duration == 10
        for value in ('0', '-1', 'nan', 'inf'):
            with pytest.raises(SystemExit):
                parser(mode).parse_args(['--duration', value])
    # Entry point help must work without importing rclpy.
    import os
    env = dict(os.environ, PYTHONPATH=str(SRC/'shopping_uwb_follow'))
    result = subprocess.run([sys.executable, '-c',
                             'from shopping_uwb_follow.evaluation_node import person_sync_main; person_sync_main(["--help"])'],
                            env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert '--lag-limit' in result.stdout


def test_pure_modules_and_humble_callback_signatures():
    for name in ('evaluation_metrics.py', 'person_path.py', 'follow_metrics.py'):
        tree = ast.parse((SRC/'shopping_uwb_follow/shopping_uwb_follow'/name).read_text())
        imports = [n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
        imports += [a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names]
        assert not any(n and n.startswith(('rclpy', 'geometry_msgs', 'nav_msgs', 'tf2', 'std_msgs')) for n in imports)
    tree = ast.parse((SRC/'shopping_uwb_follow/shopping_uwb_follow/evaluation_node.py').read_text())
    callbacks = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name.startswith('on_')]
    assert len(callbacks) == 5
    assert all([a.arg for a in n.args.args] == ['self', 'msg'] for n in callbacks)


def test_follow_only_bridge_and_fortress_observer_contract():
    config = yaml.safe_load((SRC/'shopping_robot_gazebo/config/bridge_person.yaml').read_text())
    assert config == [{'ros_topic_name': '/simulation/person_pose',
                       'gz_topic_name': '/model/person/pose',
                       'ros_type_name': 'tf2_msgs/msg/TFMessage',
                       'gz_type_name': 'ignition.msgs.Pose_V', 'direction': 'GZ_TO_ROS'}]
    world = ET.parse(SRC/'shopping_robot_gazebo/worlds/supermarket.sdf').getroot()
    plugin = world.find(".//model[@name='person']/plugin")
    assert plugin.get('name') == 'ignition::gazebo::systems::PosePublisher'
    for key in ('publish_model_pose', 'publish_nested_model_pose', 'use_pose_vector_msg'):
        assert plugin.findtext(key) == 'true'
    assert plugin.findtext('publish_link_pose') == 'false'
    assert plugin.findtext('static_publisher') == 'false'
    launch = (SRC/'shopping_robot_bringup/launch/follow.launch.py').read_text()
    assert "'/config/bridge_person.yaml'" in launch
    assert 'person_bridge, *nodes' in launch
    assert 'bridge_person.yaml' not in (SRC/'shopping_robot_gazebo/launch/sim.launch.py').read_text()


@pytest.mark.parametrize('bad_parent,zero_stamp', [(False, False), (True, False), (False, True)])
def test_live_collector_with_fake_humble_callbacks(monkeypatch, tmp_path, bad_parent, zero_stamp):
    """Exercise the read-only ROS adapter, clock window and JSON without ROS."""
    from types import ModuleType, SimpleNamespace as NS
    from shopping_uwb_follow.evaluation_node import run
    from shopping_uwb_follow.person_path import path_position

    def module(name, **attrs):
        mod = ModuleType(name)
        mod.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, mod)
        return mod

    class FakeNode:
        def __init__(self, name, parameter_overrides):
            self.clock_time = 0.0
            self.callbacks = {}
            assert parameter_overrides[0].value is True

        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=round(self.clock_time*1e9)))

        def get_parameter(self, key):
            assert key == 'use_sim_time'
            return NS(value=True)

        def create_subscription(self, kind, topic, callback, qos):
            self.callbacks[topic] = callback

        def get_logger(self):
            return NS(info=lambda message: None)

        def destroy_node(self):
            pass

    def spin_once(node, timeout_sec):
        node.clock_time = round(node.clock_time+0.1, 8)
        t = node.clock_time
        p = path_position(PATH, t)
        stamp = NS(sec=int(t), nanosec=round((t-int(t))*1e9))
        node.callbacks['/uwb/person_ground_truth'](
            NS(header=NS(stamp=stamp, frame_id='sim_world'), point=NS(x=p[0], y=p[1])))
        transform = NS(header=NS(stamp=NS(sec=0, nanosec=0) if zero_stamp else stamp,
                                  frame_id='odom' if bad_parent else 'supermarket'),
                       child_frame_id='person', transform=NS(translation=NS(x=p[0], y=p[1])))
        node.callbacks['/simulation/person_pose'](NS(transforms=[transform]))

    module('rclpy', init=lambda args: None, ok=lambda: True, spin_once=spin_once, shutdown=lambda: None)
    module('rclpy.node', Node=FakeNode)
    module('rclpy.parameter', Parameter=lambda name, value: NS(value=value))
    module('rclpy.qos', qos_profile_sensor_data=object())
    module('ament_index_python.packages', get_package_share_directory=lambda package: str(SRC/package))
    for name, symbol in [('geometry_msgs.msg', 'PointStamped'), ('nav_msgs.msg', 'Odometry'),
                         ('std_msgs.msg', 'String'), ('tf2_msgs.msg', 'TFMessage')]:
        module(name, **{symbol: object})
    path_file = tmp_path/'path.yaml'
    path_file.write_text(yaml.safe_dump(PATH))
    output = tmp_path/'nested/report.json'
    code = run('sync', ['--duration', '5', '--person-path', str(path_file), '--output', str(output)])
    import json
    report = json.loads(output.read_text())
    assert report['mode'] == 'sync'
    assert report['end_sim_s']-report['start_sim_s'] == 5
    assert code == (1 if bad_parent or zero_stamp else 0)
    assert report['status'] == ('FAIL' if code else 'PASS')


@pytest.mark.parametrize('safe_state', ['TOO_CLOSE', 'OBSTACLE_STOP', 'BLOCKED_WAIT', 'NAV2_HANDOFF'])
def test_reacquisition_preserves_safety_hold_and_separate_motion_time(safe_state):
    targets = [i/10 for i in range(70) if not 1 <= i/10 < 2]
    states = [(i/10, 'FOLLOWING' if i < 14 or i >= 60 else
               'LOST' if i < 21 else safe_state) for i in range(70)]
    event = reacquire_events(targets, states, 7, recognized_states=('FOLLOWING', safe_state))[0]
    assert event['tracking_state'] == safe_state
    assert event['tracking_s'] == 2.1
    assert event['following_s'] == 6
    assert event['reacquire_s'] == pytest.approx(.1)
    assert event_row('target_reacquire', [event], 2)['status'] == 'PASS'
    motion_event = reacquire_events(targets, states, 7)[0]
    assert event_row('reacquire', [motion_event], 2)['status'] == 'FAIL'


def test_disabled_or_stale_target_never_certifies_reacquisition():
    targets = [0, .1, 2]
    states = [(i/10, 'LOST' if 2 <= i <= 22 else 'IDLE') for i in range(70)]
    event = reacquire_events(targets, states, 7)[0]
    assert event['tracking_s'] is None
    assert event['status'] == 'NOT_COVERED'



def test_informational_performance_uses_config_distance_excludes_settle_and_pauses():
    from shopping_uwb_follow.evaluation_metrics import follow_performance
    path = dict(speed=1., waypoints=[[0, 0], [20, 0]], pauses=[10, 0])
    data = [(i/10, 8. if i < 100 else 1.4) for i in range(200)]
    result = follow_performance(data, path, 1.2)
    assert result['max_gap_m'] == 8.
    assert result['person_speed_m_s'] == 1.
    assert result['steady_state_distance_error_m']['mean'] == pytest.approx(.2)
    assert result['steady_state_distance_error_m']['rmse'] == pytest.approx(.2)
    assert follow_performance([], path, 1.2)['steady_state_distance_error_m'] == {'count': 0}
    assert result_code([{'status': 'PASS'}, {'status': 'INFO'}]) == 0
    assert result_code([{'status': 'FAIL'}, {'status': 'INFO'}]) == 1
