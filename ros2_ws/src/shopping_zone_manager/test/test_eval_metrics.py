"""Hand-computed offline evaluation cases, including missing observations."""
import csv
import json
import math
from pathlib import Path

import pytest

from shopping_zone_manager.eval_metrics import (
    clearance_metrics, follow_metrics, load_store_boxes, navigation_metrics,
    state_metrics, stop_metrics, summary,
)
from shopping_zone_manager.eval_report import evaluate_log, load_log, main, markdown_report


WORLD = Path(__file__).resolve().parents[2]/'shopping_robot_gazebo/worlds/supermarket.sdf'


def pose(t, x=0, y=0, yaw=0, **kwargs):
    return dict(t=t, x=x, y=y, yaw=yaw, **kwargs)


def episode(identity='a', success=True, goal=(3, 4, 0), samples=None):
    return dict(id=identity, success=success, goal=goal,
                samples=[pose(0), pose(4, 3, 0), pose(10, 3, 4)] if samples is None else samples)


def speed(t, linear=0, angular=0):
    return dict(t=t, linear=linear, angular=angular)


def test_navigation_geometry_success_denominator_and_success_only_time():
    result = navigation_metrics([episode(), episode('failed', False, samples=[pose(0), pose(50)])])
    assert result['count'] == 2
    assert result['success_rate'] == 0.5
    assert result['time_to_goal'] == dict(count=1, mean=10, median=10, p95=10, max=10)
    row = result['episodes'][0]
    assert row['path_length'] == 7
    assert row['path_efficiency'] == pytest.approx(5/7)
    assert row['final_position_error'] == row['final_yaw_error'] == 0
    assert result['episodes'][1]['final_position_error'] == 5
    assert result['episodes'][1]['path_efficiency'] is None


def test_navigation_wrapped_yaw_and_stationary_goal():
    result = navigation_metrics([episode(goal=(0, 0, -math.pi+0.1),
                                         samples=[pose(2, yaw=math.pi-0.1)])])['episodes'][0]
    assert result['final_yaw_error'] == pytest.approx(0.2)
    assert result['duration'] == result['path_length'] == 0
    assert result['path_efficiency'] is None


def test_percentiles_and_navigation_time_distribution():
    durations = [1, 2, 3, 10]
    result = navigation_metrics([episode(str(t), samples=[pose(0), pose(t)]) for t in durations])
    assert result['time_to_goal'] == dict(count=4, mean=4, median=2.5, p95=pytest.approx(8.95), max=10)
    assert summary([]) == dict(count=0, mean=None, median=None, p95=None, max=None)


def test_empty_inputs_are_explicitly_undefined():
    nav = navigation_metrics([])
    assert nav['success_rate'] is None and nav['count'] == 0
    assert navigation_metrics([episode(samples=[])])['episodes'][0]['duration'] is None
    assert clearance_metrics([], [])['minimum'] is None
    assert clearance_metrics([pose(0)], [])['minimum'] is None
    follow = follow_metrics([])
    assert follow['rmse'] is follow['time_fraction_in_band'] is None
    assert follow['too_close_count'] == 0
    assert state_metrics([])['lost_count'] == 0
    assert stop_metrics([], [], [])['count'] == 0


def test_clearance_edges_corners_contact_and_overlap():
    assert clearance_metrics([pose(0)], [(1, -1, 2, 1)])['static_min'] == pytest.approx(0.7)
    assert clearance_metrics([pose(0)], [(0.6, 0.65, 1, 1)])['static_min'] == pytest.approx(0.5)
    for box in [(0.3, -1, 2, 1), (-1, -1, 1, 1), (-0.1, -1, 0.1, 1),
                (-0.1, -0.1, 0.1, 0.1)]:
        assert clearance_metrics([pose(0)], [box])['minimum'] == 0


def test_clearance_uses_rotated_footprint_and_configurable_dimensions():
    box = [(1, -1, 2, 1)]
    assert clearance_metrics([pose(0, yaw=math.pi/2)], box)['minimum'] == pytest.approx(0.75)
    assert clearance_metrics([pose(0, yaw=math.pi/4)], box)['minimum'] == pytest.approx(1-0.55/math.sqrt(2))
    assert clearance_metrics([pose(0)], box, length=1, width=0.2)['minimum'] == 0.5
    # A rotated rectangle crosses a narrow box although no vertex is contained.
    assert clearance_metrics([pose(0, yaw=math.pi/4)], [(-1, -0.01, 1, 0.01)])['minimum'] == 0


def test_person_circle_clearance_and_missing_person_counts():
    rows = [pose(0), pose(1, person_x=1, person_y=0), pose(2, person_x=0.6, person_y=0.65)]
    result = clearance_metrics(rows, [(10, 10, 11, 11)])
    assert result['person_sample_count'] == 2 and result['pose_count'] == 3
    assert result['person_min'] == result['minimum'] == pytest.approx(0.25)
    assert clearance_metrics([pose(0, person_x=0, person_y=0)], [])['person_min'] == 0
    assert clearance_metrics([pose(0, person_x=0.3, person_y=0)], [])['person_min'] == 0
    assert clearance_metrics([pose(0, yaw=math.pi/2, person_x=1, person_y=0)], [],
                             person_radius=0.1)['person_min'] == pytest.approx(0.65)


def test_actual_store_geometry_is_loaded_without_a_second_layout():
    boxes = load_store_boxes(WORLD)
    assert len(boxes) == 14
    result = clearance_metrics([pose(0, y=2)], boxes)
    assert result['static_min'] == pytest.approx(0.5)


@pytest.mark.parametrize('change', [
    lambda s: s.replace('<pose>0 7 1 0 0 0</pose>', '<pose>0 7 1 0 0 1</pose>'),
    lambda s: s.replace('<link name="link">', '<link name="link"><pose>0 0 0 0 0 0</pose>', 1),
    lambda s: s.replace('<size>12 0.2 2</size>', '<size>12 nan 2</size>'),
    lambda s: s.replace('<static>true</static>', '<static>false</static>'),
    lambda s: s.replace('<size>12 0.2 2</size>', '<size>12 0 2</size>'),
])
def test_store_loader_rejects_unsupported_layout(tmp_path, change):
    # Change floor and obstacle link poses, since floor is deliberately excluded.
    text = change(WORLD.read_text())
    if '<link name="link"><pose>' in text:
        text = text.replace('<link name="link">', '<link name="link"><pose>0 0 0 0 0 0</pose>')
    path = tmp_path/'world.sdf'
    path.write_text(text)
    with pytest.raises(ValueError):
        load_store_boxes(path)


def test_follow_sample_errors_and_time_weighting():
    result = follow_metrics([dict(t=0, distance=0.5), dict(t=1, distance=1),
                             dict(t=4, distance=1.5)])
    assert result['mean_error'] == 0
    assert result['mean_absolute_error'] == pytest.approx(1/3)
    assert result['rmse'] == pytest.approx(math.sqrt(1/6))
    assert result['p95_absolute_error'] == 0.5
    assert result['time_fraction_in_band'] == 0.75
    assert result['too_close_count'] == 1
    assert result['too_close_duration'] == 1


def test_follow_band_boundaries_close_runs_and_terminal_sample():
    rows = [dict(t=t, distance=d) for t, d in [(0, 0.6), (2, 0.5), (3, 1.2), (4, 0.6)]]
    result = follow_metrics(rows)
    assert result['too_close_count'] == 2
    assert result['too_close_duration'] == 3
    assert result['time_fraction_in_band'] == 0.25
    assert follow_metrics([dict(t=4, distance=1)])['time_fraction_in_band'] is None


def test_state_reacquisition_counts_pending_and_repeated_lost():
    states = ['LOST', 'LOST', 'OBSTACLE_STOP', 'FOLLOWING', 'FOLLOWING', 'LOST', 'IDLE']
    result = state_metrics([dict(t=t, state=s) for t, s in enumerate(states)])
    assert result == dict(lost_count=2, reacquire_count=1, pending_lost_count=1,
                          reacquire_time=dict(count=1, mean=3, median=3, p95=3, max=3))
    assert state_metrics([dict(t=0, state='FOLLOWING')])['reacquire_count'] == 0


def test_stop_requires_both_linear_and_angular_and_ignores_earlier_samples():
    triggers = [dict(t=1, event='estop'), dict(t=3, event='timeout')]
    cmd = [speed(0), speed(1, angular=0.2), speed(1.2), speed(4, linear=0.1)]
    odom = [speed(0), speed(1.2, linear=-0.02), speed(1.4, angular=0.05), speed(1.7)]
    result = stop_metrics(triggers, cmd, odom)
    assert result['events'][0]['commanded_latency'] == pytest.approx(0.2)
    assert result['events'][0]['measured_latency'] == pytest.approx(0.7)
    assert result['events'][1]['commanded_latency'] is None
    assert result['events'][1]['measured_latency'] is None
    assert result['commanded_pending'] == result['measured_pending'] == 1
    assert result['commanded']['count'] == 1


def test_stop_exact_trigger_zero_and_custom_thresholds():
    result = stop_metrics([dict(t=2)], [speed(2, linear=1e-10)],
                          [speed(2, linear=-0.03, angular=-0.1)],
                          linear_threshold=0.04, angular_threshold=0.2)
    assert result['events'][0]['commanded_latency'] == result['events'][0]['measured_latency'] == 0
    pending = stop_metrics([dict(t=2)], [], [])
    assert pending['commanded_pending'] == pending['measured_pending'] == 1


@pytest.mark.parametrize('times', [[2, 1], [1, 1]])
@pytest.mark.parametrize('metric', ['navigation', 'clearance', 'follow', 'state', 'trigger', 'cmd', 'odom'])
def test_reject_unsorted_or_duplicate_timestamps(times, metric):
    with pytest.raises(ValueError):
        if metric == 'navigation':
            navigation_metrics([episode(samples=[pose(t) for t in times])])
        elif metric == 'clearance':
            clearance_metrics([pose(t) for t in times], [])
        elif metric == 'follow':
            follow_metrics([dict(t=t, distance=1) for t in times])
        elif metric == 'state':
            state_metrics([dict(t=t, state='LOST') for t in times])
        elif metric == 'trigger':
            stop_metrics([dict(t=t) for t in times], [], [])
        elif metric == 'cmd':
            stop_metrics([], [speed(t) for t in times], [])
        else:
            stop_metrics([], [], [speed(t) for t in times])


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf, True])
def test_nonfinite_data_rejected_in_all_numeric_streams(bad):
    calls = [lambda: navigation_metrics([episode(goal=(bad, 0, 0))]),
             lambda: navigation_metrics([episode(samples=[pose(0, yaw=bad)])]),
             lambda: clearance_metrics([pose(0, person_x=bad, person_y=0)], []),
             lambda: clearance_metrics([pose(0)], [(0, 0, bad, 1)]),
             lambda: follow_metrics([dict(t=0, distance=bad)]),
             lambda: state_metrics([dict(t=bad, state='LOST')]),
             lambda: stop_metrics([dict(t=bad)], [], []),
             lambda: stop_metrics([], [speed(0, angular=bad)], []),
             lambda: stop_metrics([], [], [speed(0, linear=bad)])]
    for call in calls:
        with pytest.raises(ValueError):
            call()


@pytest.mark.parametrize('call', [
    lambda: clearance_metrics([], [], width=0),
    lambda: clearance_metrics([], [(2, 0, 1, 1)]),
    lambda: clearance_metrics([pose(0, person_x=1)], []),
    lambda: follow_metrics([], band=-1),
    lambda: follow_metrics([dict(t=0, distance=-1)]),
    lambda: state_metrics([dict(t=0, state='')]),
    lambda: stop_metrics([], [], [], linear_threshold=0),
    lambda: navigation_metrics([episode(success='false')]),
])
def test_invalid_parameters_and_fields(call):
    with pytest.raises(ValueError):
        call()


def log_rows():
    return [dict(type='navigation', t=0, episode='trip', success=True, goal_x=3, goal_y=4,
                 goal_yaw=0, x=0, y=0, yaw=0),
            dict(type='pose', t=0, x=0, y=0, yaw=0, person_x=1, person_y=0),
            dict(type='distance', t=0, distance=1), dict(type='state', t=0, state='LOST'),
            dict(type='trigger', t=0, event='estop'), dict(type='cmd', t=0.2, linear=0, angular=0),
            dict(type='odom', t=0.6, linear=0, angular=0),
            dict(type='state', t=1, state='FOLLOWING'),
            dict(type='navigation', t=5, episode='trip', success=True, goal_x=3, goal_y=4,
                 goal_yaw=0, x=3, y=4, yaw=0)]


@pytest.mark.parametrize('extension', ['csv', 'jsonl'])
def test_loader_and_cli_write_equivalent_strict_json_and_markdown(tmp_path, extension):
    path = tmp_path/f'log.{extension}'
    rows = log_rows()
    if extension == 'jsonl':
        path.write_text('\n'.join(json.dumps(row) for row in rows))
    else:
        fields = list(dict.fromkeys(k for row in rows for k in row))
        with path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fields)
            writer.writeheader()
            writer.writerows({k: ('true' if v else 'false') if isinstance(v, bool) else v
                              for k, v in row.items()} for row in rows)
    prefix = tmp_path/'summary'
    assert main([str(path), '--world', str(WORLD), '--output', str(prefix)]) == 0
    result = json.loads((tmp_path/'summary.json').read_text())
    assert result['navigation']['success_rate'] == 1
    assert result['navigation']['episodes'][0]['path_length'] == 5
    assert result['clearance']['person_min'] == pytest.approx(0.45)
    assert result['states']['reacquire_time']['mean'] == 1
    assert result['stop']['commanded']['max'] == 0.2
    assert result['stop']['measured']['max'] == 0.6
    assert result['parameters']['target'] == 1
    markdown = (tmp_path/'summary.md').read_text()
    assert 'navigation.success_rate | 1.0' in markdown
    assert '無資料' in markdown
    assert 'NaN' not in (tmp_path/'summary.json').read_text()


@pytest.mark.parametrize('rows', [
    [dict(type='distance', t=2, distance=1), dict(type='distance', t=1, distance=1)],
    [dict(type='bogus', t=0)],
    [dict(type='distance', t=0, distance='NaN')],
    [dict(type='pose', t=0, x=0, y=0)],
    [dict(type='trigger', t=0)],
    [dict(type='navigation', t=0, episode='a', success='maybe')],
])
def test_invalid_logs_fail_before_writing_reports(tmp_path, rows):
    path = tmp_path/'log.jsonl'
    path.write_text('\n'.join(json.dumps(row) for row in rows))
    with pytest.raises(SystemExit) as exc:
        main([str(path), '--world', str(WORLD), '--output', str(tmp_path/'report')])
    assert exc.value.code == 2
    assert not (tmp_path/'report.json').exists()


def test_conflicting_navigation_metadata_rejected():
    rows = [row for row in log_rows() if row['type'] == 'navigation']
    rows[-1]['success'] = False
    with pytest.raises(ValueError):
        evaluate_log(rows, [])


def test_empty_log_and_markdown_escapes(tmp_path):
    path = tmp_path/'empty.jsonl'
    path.write_text('')
    report = evaluate_log(load_log(path), [])
    assert report['navigation']['count'] == 0
    assert report['stop']['commanded']['mean'] is None
    assert 'a\\|b c' in markdown_report({'event': 'a|b\nc'})
