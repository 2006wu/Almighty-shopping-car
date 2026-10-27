"""Execute the navigator result adapter without ROS, including confidence failure."""
import ast
from concurrent.futures import Future
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from shopping_zone_manager.goal_tracker import GoalTracker, State
from shopping_zone_manager.localization_health import amcl_healthy
from shopping_zone_manager.zones import Zone, arrived


def navigator(sample, require=True):
    path = Path(__file__).resolve().parents[1]/'ros2_ws/src/shopping_zone_manager/shopping_zone_manager/node.py'
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    cls.bases = []

    class Time:
        @staticmethod
        def from_msg(stamp):
            return NS(nanoseconds=stamp)

    env = dict(math=math, Time=Time, arrived=arrived, amcl_healthy=amcl_healthy,
               TransformException=LookupError)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), env)
    node = env['ZoneNavigator'].__new__(env['ZoneNavigator'])
    node.tracker = GoalTracker()
    node.tracker.request(Zone('goal', (), 1., 2., 0., 'map'))
    node.tracker.response(1, True)
    node.handles, node.result_futures = {1: object()}, {1: object()}
    node.amcl_sample, node.require_amcl = sample, require
    node.amcl_xy_std, node.amcl_yaw_std, node.amcl_max_age = .15, .35, 5.
    node.arrival_xy_tolerance, node.arrival_yaw_tolerance, node.arrival_max_age = .25, .35, .5
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
    node.pose_buffer = NS(lookup_transform=lambda *args: NS(
        header=NS(stamp=9_900_000_000), transform=NS(
            translation=NS(x=1., y=2.), rotation=NS(x=0., y=0., z=0., w=1.))))
    node.get_logger = lambda: NS(warning=lambda msg: None)
    node.apply = lambda effects: None
    return node


@pytest.mark.parametrize('variance,stamp,required,expected', [
    (.008, 9., True, State.SUCCEEDED),
    (.02761, 9., True, State.ABORTED),
    (1.65, 9., True, State.ABORTED),
    (.008, 4., True, State.ABORTED),
    (.008, 11., True, State.ABORTED),
    (None, 9., True, State.ABORTED),
    (None, 9., False, State.SUCCEEDED),
    (.02761, 9., False, State.ABORTED),
])
def test_confidence_gates_actual_success_callback(variance, stamp, required, expected):
    cov = [0.]*36
    cov[0], cov[7], cov[35] = .001, variance, .003
    node = navigator(None if variance is None else (stamp, cov), required)
    future = Future()
    future.set_result(NS(status=4))
    node.result(1, future)
    assert node.tracker.state == expected
    assert not node.tracker.busy
    assert node.handles == node.result_futures == {}


def test_guide_requires_amcl_only_for_amcl_localization():
    from test_mode_launch import load, resolve
    actions = load('shopping_robot_bringup/launch/guide.launch.py')['generate_launch_description']()
    node = next(a for a in actions if a.kind == 'Node' and a.executable == 'zone_navigator')
    expression = node.parameters[0]['require_amcl'].args[0]
    for mode in ('mapping', 'localization', 'map_server'):
        assert resolve(expression, {'slam_mode': mode}) == (mode == 'map_server')


def test_guide_forwards_explicit_actor_yield_flag():
    from test_mode_launch import load, resolve
    actions = load('shopping_robot_bringup/launch/guide.launch.py')['generate_launch_description']()
    declaration = next(a for a in actions if a.kind == 'DeclareLaunchArgument' and a.args[0] == 'pedestrian_yield')
    assert declaration.default_value == 'true'
    sim = next(a for a in actions if a.kind == 'IncludeLaunchDescription' and a.args[0].endswith('/sim.launch.py'))
    argument = dict(sim.launch_arguments)['pedestrian_yield']
    for value in ('true', 'false'):
        assert resolve(argument, {'pedestrian_yield': value}) == value
