"""Exercise bounded retries, timeout handling and experiment configuration."""
import ast
from pathlib import Path
import subprocess

import pytest

from shopping_zone_manager.tour_logic import Sequence, load_tour, map_prefix

ROOT = Path(__file__).resolve().parents[4]
CONFIG = ROOT/'ros2_ws/src/shopping_zone_manager/config/map_tour.yaml'


def test_sequence_retries_skips_and_finishes():
    goals = load_tour(CONFIG)[:3]
    seq = Sequence(goals, started=10, timeout=50, max_failures=2)
    assert seq.current(11) == goals[0]
    seq.record(False)
    assert seq.current(12) == goals[0]
    seq.record(True)
    seq.record(False)
    seq.record(False)
    assert seq.current(13) == goals[2]
    seq.record(True)
    assert seq.current(14) is None
    assert seq.summary() == {'reached': [goals[0].name, goals[2].name],
                             'skipped': [goals[1].name], 'pending': [],
                             'timed_out': False, 'success': False}
    with pytest.raises(RuntimeError):
        seq.record(True)


def test_overall_timeout_includes_unfinished_goals():
    goals = load_tour(CONFIG)
    seq = Sequence(goals, started=10, timeout=5)
    seq.record(True)
    assert seq.current(15) is None
    assert seq.summary()['timed_out']
    assert seq.summary()['pending'] == [g.name for g in goals[1:]]
    assert not seq.summary()['success']


def test_success_and_single_failure_skip():
    goals = load_tour(CONFIG)[:1]
    seq = Sequence(goals, 0, max_failures=1)
    seq.record(True)
    assert seq.current(1) is None and seq.summary()['success']
    seq = Sequence(goals, 0, max_failures=1)
    seq.record(False)
    assert seq.current(1) is None and seq.summary()['skipped'] == [goals[0].name]


@pytest.mark.parametrize('timeout,failures', [(0, 2), (-1, 2), (float('nan'), 2), (10, 0)])
def test_invalid_sequence_limits(timeout, failures):
    with pytest.raises(ValueError):
        Sequence(load_tour(CONFIG), 0, timeout, failures)


@pytest.mark.parametrize('name', ['../supermarket', 'a/b', 'a b', '', 'a;touch'])
def test_map_names_are_bounded(tmp_path, name):
    with pytest.raises(ValueError):
        map_prefix(tmp_path, name)


def test_entry_points_and_shell_commands():
    setup = ast.parse((ROOT/'ros2_ws/src/shopping_zone_manager/setup.py').read_text())
    call = next(n for n in ast.walk(setup) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'setup')
    entries = ast.literal_eval(next(k.value for k in call.keywords if k.arg == 'entry_points'))['console_scripts']
    assert 'map_tour = shopping_zone_manager.map_tour:main' in entries
    assert 'localization_check = shopping_zone_manager.localization_check:main' in entries
    shell = (ROOT/'scripts/dev.sh').read_text()
    assert '  map-tour)' in shell and '  loc-check)' in shell
    assert 'map_name:=$name' in shell
    assert 'localization_check --ros-args -p use_sim_time:=true "$@"' in shell


@pytest.mark.parametrize('suffix', ['yaml', 'pgm', 'posegraph', 'data'])
def test_canonical_maps_can_be_added_but_other_maps_stay_ignored(suffix):
    directory = 'ros2_ws/src/shopping_robot_navigation/maps/'
    for name, expected in [('supermarket', 1), ('other_test_map', 0)]:
        result = subprocess.run(['git', 'check-ignore', '--no-index', '-q', directory+name+'.'+suffix], cwd=ROOT)
        assert result.returncode == expected


def test_pure_modules_do_not_import_ros():
    directory = ROOT/'ros2_ws/src/shopping_zone_manager/shopping_zone_manager'
    for name in ('tour_logic.py', 'localization_metrics.py'):
        tree = ast.parse((directory/name).read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                assert all(not alias.name.startswith(('rclpy', 'geometry_msgs', 'nav_msgs')) for alias in node.names)
            if isinstance(node, ast.ImportFrom):
                assert not node.module.startswith(('rclpy', 'geometry_msgs', 'nav_msgs'))


@pytest.mark.parametrize('content', ['waypoints: []',
                                     'waypoints: [{name: a, x: .nan, y: 0, yaw: 0}]',
                                     'waypoints: [{name: a, x: true, y: 0, yaw: 0}]',
                                     'waypoints: [{name: a, x: 0, y: 0, yaw: 0}, {name: a, x: 1, y: 0, yaw: 0}]'])
def test_invalid_tour_configuration(tmp_path, content):
    config = tmp_path/'tour.yaml'
    config.write_text(content)
    with pytest.raises(ValueError):
        load_tour(config)
