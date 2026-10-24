"""Synthetic bag pipeline validation without ROS or a running simulator."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest
import yaml

from shopping_zone_manager.bag_converter import convert_records, TRUTH, PERSON, FOLLOW, ZONE
from shopping_zone_manager.eval_report import load_log, evaluate_log
from shopping_zone_manager.bag_to_eval import plain_message

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/'ros2_ws/src/shopping_zone_manager'
ZONES = [dict(name='entrance', x=1., y=0., yaw=0., frame_id='map')]


def robot(t, x=0., speed=0.):
    return (TRUTH, t, dict(header=dict(frame_id='sim_world'),
                          pose=dict(pose=dict(position=dict(x=x, y=0.),
                                              orientation=dict(x=0., y=0., z=0., w=1.))),
                          twist=dict(twist=dict(linear=dict(x=speed, y=0.), angular=dict(z=0.)))))


def person(t, x=1.):
    return PERSON, t, dict(header=dict(frame_id='sim_world'), point=dict(x=x, y=0.))


def state(topic, t, text):
    return topic, t, dict(data=text)


def command(t, v=0., w=0.):
    return '/cmd_vel', t, dict(linear=dict(x=v, y=0.), angular=dict(z=w))


@pytest.mark.parametrize('terminal,success', [('SUCCEEDED', True), ('ABORTED', False),
                                            ('CANCELED', False), ('REJECTED', False),
                                            ('TIMEOUT', False)])
def test_navigation_boundaries_and_round_trip(tmp_path, terminal, success):
    records = [robot(0), robot(.1, .5), robot(.2, 1),
               state(ZONE, .05, 'PENDING: Requested entrance'),
               state(ZONE, .15, terminal+': entrance')]
    rows, info = convert_records(reversed(records), ZONES, (0, 0, 0))
    nav = [row for row in rows if row['type'] == 'navigation']
    assert [row['t'] for row in nav] == [.05, .1, .15]
    assert nav[0]['x'] == pytest.approx(.25)
    assert nav[-1]['x'] == pytest.approx(.75)
    assert all(row['success'] is success for row in nav)
    path = tmp_path/'observations.jsonl'
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    report = evaluate_log(load_log(path), [])
    assert report['navigation']['success_rate'] == float(success)
    assert info['cutoff_episodes'] == []


def test_follow_and_stop_latency_and_state_normalization():
    records = [robot(0, speed=.3), robot(.1, speed=.1), robot(.2),
               person(0, 1), person(.2, 1.2),
               state(FOLLOW, 0, 'FOLLOWING: Tracking'),
               state(FOLLOW, .05, 'LOST: Target missing or stale'),
               state(FOLLOW, .1, 'LOST: Target missing or stale'),
               state(FOLLOW, .2, 'FOLLOWING: Tracking'),
               command(0, .3), command(.1, 0, .1), command(.15)]
    rows, _ = convert_records(records)
    assert [r['distance'] for r in rows if r['type'] == 'distance'] == pytest.approx([1, 1.1, 1.2])
    report = evaluate_log(rows, [])
    assert report['states']['lost_count'] == 1
    event = report['stop']['events'][0]
    assert event['commanded_latency'] == pytest.approx(.1)
    assert event['measured_latency'] == pytest.approx(.15)


def test_estop_same_time_triggers_and_pending_stop():
    rows, _ = convert_records([state(FOLLOW, 1, 'TOO_CLOSE: Close'),
                               state('/mode_manager/state', 1, 'ESTOP'), command(0)])
    triggers = [r for r in rows if r['type'] == 'trigger']
    assert triggers == [dict(type='trigger', t=1., event='estop+too_close')]
    assert evaluate_log(rows, [])['stop']['commanded_pending'] == 1


def test_empty_and_missing_person_are_not_fabricated():
    assert convert_records([])[0] == []
    rows, info = convert_records([robot(0), state(FOLLOW, 0, 'LOST: Missing')])
    assert not any(r['type'] == 'distance' for r in rows)
    assert info['missing_person_samples'] == 1
    evaluate_log(rows, [])


def test_internal_person_gap_rejected():
    records = [robot(t) for t in (0, .1, .5, .9, 1.)]
    records += [person(0), person(1), state(FOLLOW, 0, 'FOLLOWING: Tracking')]
    with pytest.raises(ValueError, match='Interior person'):
        convert_records(records)


def test_duplicate_truth_rejected_but_receive_time_commands_coalesced():
    with pytest.raises(ValueError, match='Duplicate truth'):
        convert_records([robot(0), robot(0)])
    rows, info = convert_records([command(0, .1), command(0, .2)])
    assert rows[0]['linear'] == .2
    assert info['coalesced_topic_samples']['/cmd_vel'] == 1


def test_navigation_cutoff_and_explicit_transform():
    records = [robot(0), robot(.1), state(ZONE, 0, 'PENDING: Requested entrance')]
    with pytest.raises(ValueError, match='explicit'):
        convert_records(records, ZONES)
    rows, info = convert_records(records, ZONES, (1, 2, 0))
    nav = [r for r in rows if r['type'] == 'navigation']
    assert info['cutoff_episodes'] == ['zone-1-entrance']
    assert nav[-1]['success'] is False
    assert (nav[-1]['goal_x'], nav[-1]['goal_y']) == (2, 2)


def test_partial_navigation_and_missing_boundary_fail():
    with pytest.raises(ValueError, match='during navigation'):
        convert_records([state(ZONE, 1, 'ACTIVE: entrance')])
    with pytest.raises(ValueError, match='Missing truth'):
        convert_records([robot(0), state(ZONE, 1, 'PENDING: Requested entrance'),
                         state(ZONE, 2, 'SUCCEEDED: entrance')], ZONES, (0, 0, 0))


def test_frame_mismatch_rejected():
    record = person(0)
    record[2]['header']['frame_id'] = 'odom'
    with pytest.raises(ValueError, match='sim_world'):
        convert_records([record])


def test_manifest_and_entry_points_and_pure_imports():
    topics = yaml.safe_load((PACKAGE/'config/record_topics.yaml').read_text())['topics']
    assert len(topics) == len(set(topics))
    sources = '\n'.join(p.read_text() for p in (ROOT/'ros2_ws/src').rglob('*')
                        if p.suffix in ('.py', '.yaml', '.xacro') and
                        p.name not in ('record_topics.yaml', 'bag_converter.py', 'bag_to_eval.py') and
                        'test' not in p.parts)
    assert all(topic in sources for topic in topics)
    assert '/scan' not in topics
    dev = (ROOT/'scripts/dev.sh').read_text()
    assert '  record)' in dev and '  eval-bag)' in dev
    assert 'bag_to_eval = shopping_zone_manager.bag_to_eval:main' in (PACKAGE/'setup.py').read_text()
    for name in ('bag_converter.py', 'eval_metrics.py', 'eval_report.py'):
        tree = ast.parse((PACKAGE/'shopping_zone_manager'/name).read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                modules = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                modules = [node.module or '']
            else:
                continue
            assert not any(m.split('.')[0] in ('rclpy', 'rosbag2_py', 'rosidl_runtime_py') for m in modules)
    subprocess.run(['bash', '-n', str(ROOT/'scripts/dev.sh')], check=True)


def test_recorder_uses_manifest_and_graceful_duration(monkeypatch):
    spec = importlib.util.spec_from_file_location('record_bag', ROOT/'scripts/record_bag.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calls = []

    class Process:
        def __init__(self, argv):
            calls.append(argv)

        def wait(self, timeout=None):
            if timeout is not None:
                raise subprocess.TimeoutExpired('recorder', timeout)
            return 0

        def poll(self):
            return None

        def send_signal(self, sig):
            calls.append(sig)

    monkeypatch.setattr(module.subprocess, 'Popen', Process)
    monkeypatch.setattr(module.Path, 'mkdir', lambda *a, **kw: None)
    assert module.main(['synthetic-test-bag', '--duration', '1', '/extra']) == 0
    topics = yaml.safe_load((PACKAGE/'config/record_topics.yaml').read_text())['topics']
    assert calls[0][-len(topics)-1:] == topics+['/extra']
    assert '--use-sim-time' in calls[0]
    assert 'sqlite3' in calls[0]
    assert calls[1] == module.signal.SIGINT
    for args in (['../escape'], ['run', '--duration', 'nan'], ['run', '--duration', '0']):
        with pytest.raises(SystemExit):
            module.main(args)


def test_plain_ros_message_conversion():
    class Message:
        data = [1, 2]

        def get_fields_and_field_types(self):
            return {'data': 'sequence<int32>'}
    assert plain_message(Message()) == {'data': [1, 2]}


def test_same_tick_navigation_transitions_keep_lifecycle_order():
    records = [robot(0), robot(.1), state(ZONE, 0, 'PENDING: Requested entrance'),
               state(ZONE, 0, 'ACTIVE: entrance'), state(ZONE, .1, 'SUCCEEDED: entrance')]
    rows, _ = convert_records(records, ZONES, (0, 0, 0))
    assert evaluate_log(rows, [])['navigation']['success_rate'] == 1


def test_humble_reader_header_receive_time_and_clock_reset(monkeypatch):
    import sys
    import types
    from shopping_zone_manager.bag_to_eval import read_records
    messages = [('/clock', dict(clock=dict(sec=1, nanosec=0)), 1_000_000_000),
                ('/cmd_vel', dict(data='plain'), 1_100_000_000),
                ('/stamped', dict(header=dict(stamp=dict(sec=1, nanosec=500))), 1_200_000_000)]
    opened = []

    class Reader:
        def __init__(self):
            self.records = iter(messages)
            self.remaining = len(messages)

        def open(self, storage, converter):
            opened.append((storage, converter))

        def get_all_topics_and_types(self):
            return [types.SimpleNamespace(name=topic, type='fake/Message') for topic, _, _ in messages]

        def has_next(self):
            return self.remaining > 0

        def read_next(self):
            self.remaining -= 1
            return next(self.records)

    monkeypatch.setitem(sys.modules, 'rosbag2_py', types.SimpleNamespace(
        SequentialReader=Reader, StorageOptions=lambda **kw: kw,
        ConverterOptions=lambda *args: args))
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', types.SimpleNamespace(
        deserialize_message=lambda data, cls: data))
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py.utilities', types.SimpleNamespace(
        get_message=lambda name: object))
    result = list(read_records('/bag'))
    assert opened == [({'uri': '/bag', 'storage_id': 'sqlite3'}, ('cdr', 'cdr'))]
    assert result[1][1] == pytest.approx(1.1)
    assert result[2][1] == pytest.approx(1.0000005)
    messages.append(('/clock', dict(clock=dict(sec=0, nanosec=0)), 1_300_000_000))
    with pytest.raises(ValueError, match='clock reset'):
        list(read_records('/bag'))


def test_missing_robot_truth_interval_cannot_bridge_follow_time_fraction():
    with pytest.raises(ValueError, match='Interior person truth gap'):
        convert_records([robot(0), robot(1), person(0), person(1),
                         state(FOLLOW, 0, 'FOLLOWING: Tracking')])


def test_navigation_yaw_interpolation_uses_shortest_arc():
    import math
    first, last = robot(0), robot(.1)
    for record, angle in ((first, math.radians(179)), (last, math.radians(-179))):
        record[2]['pose']['pose']['orientation'].update(z=math.sin(angle/2), w=math.cos(angle/2))
    rows, _ = convert_records([first, last, state(ZONE, .05, 'PENDING: Requested entrance'),
                               state(ZONE, .1, 'SUCCEEDED: entrance')], ZONES, (0, 0, 0))
    nav = [r for r in rows if r['type'] == 'navigation']
    assert abs(nav[0]['yaw']) == pytest.approx(math.pi)


def test_multiple_navigation_episodes_preserve_failed_denominator():
    records = [robot(t) for t in (0, .1, .2, .3)]
    records += [state(ZONE, 0, 'PENDING: Requested entrance'),
                state(ZONE, .1, 'SUCCEEDED: entrance'),
                state(ZONE, .2, 'PENDING: Requested entrance'),
                state(ZONE, .3, 'ABORTED: entrance')]
    rows, _ = convert_records(records, ZONES, (0, 0, 0))
    assert evaluate_log(rows, [])['navigation']['success_rate'] == .5

def test_plain_message_handles_numpy_like_scalars_and_arrays():
    class Scalar(float):
        def tolist(self):
            return float(self)

    class Array:
        def __init__(self, items):
            self.items = items

        def tolist(self):
            return list(self.items)

    class Message:
        covariance = Array([Scalar(0.5), Scalar(0.0)])
        x = Scalar(1.25)

        def get_fields_and_field_types(self):
            return {'covariance': 'double[2]', 'x': 'double'}
    assert plain_message(Message()) == {'covariance': [0.5, 0.0], 'x': 1.25}


def test_guard_triggers_use_simulation_time_and_do_not_repeat():
    records = [state('/safety/obstacle_state', 100+i,
                     json.dumps(dict(t=t, state=s)))
               for i, (t, s) in enumerate(((0, 'CLEAR'), (.1, 'STOP'),
                                           (.2, 'STOP'), (.3, 'STALE')))]
    records += [command(.15), robot(.25)]
    rows, _ = convert_records(records)
    triggers = [r for r in rows if r['type'] == 'trigger']
    assert [(r['t'], r['event']) for r in triggers] == [
        (.1, 'obstacle_guard_stop'), (.3, 'obstacle_guard_stale')]
    report = evaluate_log(rows, [])
    assert report['stop']['events'][0]['commanded_latency'] == pytest.approx(.05)
    assert report['stop']['commanded_pending'] == 1
