"""Evaluate launch wiring with small substitution/action fakes, without ROS."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

SRC = Path(__file__).resolve().parents[1]/'ros2_ws/src'


class Configuration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


class Expression:
    def __init__(self, parts):
        self.parts = parts

    def perform(self, context):
        return eval(''.join(resolve(p, context) for p in self.parts), {'__builtins__': {}})


def resolve(value, context):
    return value.perform(context) if hasattr(value, 'perform') else value


def action(kind):
    return lambda *args, **kwargs: NS(kind=kind, args=args, **kwargs)


def load(relative):
    path = SRC/relative
    tree = ast.parse(path.read_text())
    tree.body = [n for n in tree.body if not isinstance(n, (ast.Import, ast.ImportFrom))]
    env = {name: action(name) for name in (
        'DeclareLaunchArgument', 'IncludeLaunchDescription', 'Node', 'GroupAction',
        'SetRemap', 'OpaqueFunction', 'ParameterValue', 'IfCondition')}
    env.update(LaunchDescription=lambda values: values, LaunchConfiguration=Configuration,
               PythonExpression=Expression, PythonLaunchDescriptionSource=lambda path: path,
               get_package_share_directory=lambda package: '/share/'+package)
    exec(compile(tree, str(path), 'exec'), env)
    return env


@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_follow_wiring_for_both_mux_modes(enabled):
    actions = load('shopping_robot_bringup/launch/follow.launch.py')['generate_launch_description']()
    context = {'use_mux': enabled}
    follow = next(a for a in actions if a.kind == 'Node' and a.executable == 'follow_controller_node')
    parameters = follow.parameters[-1]
    assert resolve(parameters['enabled'].args[0], context) == (enabled == 'false')
    assert resolve(parameters['observed_cmd_vel_topic'].args[0], context) == (
        '/cmd_vel/nav' if enabled == 'true' else '/cmd_vel')
    assert resolve(follow.remappings[0][1], context) == (
        '/cmd_vel/follow' if enabled == 'true' else '/cmd_vel')
    navigation = next(a for a in actions if a.kind == 'IncludeLaunchDescription'
                      and a.args[0].endswith('/shopping_robot_navigation/launch/navigation.launch.py'))
    assert resolve(dict(navigation.launch_arguments)['use_mux'], context) == enabled


@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_nav2_only_smoother_output_is_remapped(enabled):
    env = load('shopping_robot_navigation/launch/navigation.launch.py')
    actions = env['start']({'use_mux': enabled, 'slam_mode': 'mapping', 'map': ''})
    group = actions[-1]
    remaps = [a for a in group.actions if a.kind == 'SetRemap']
    assert [(a.src, a.dst) for a in remaps] == (
        [('cmd_vel_smoothed', '/cmd_vel/nav')] if enabled == 'true' else [])
    include = group.actions[-1]
    assert include.args[0].endswith('/nav2_bringup/launch/navigation_launch.py')
    assert dict(include.launch_arguments)['use_composition'] == 'False'


@pytest.mark.parametrize('name, initial', [('guide', 'guide'), ('follow', 'follow'), ('mode_mux', 'idle')])
def test_launch_defaults_and_manager_inclusion(name, initial):
    actions = load(f'shopping_robot_bringup/launch/{name}.launch.py')['generate_launch_description']()
    declarations = {a.args[0]: a.default_value for a in actions if a.kind == 'DeclareLaunchArgument'}
    assert declarations['use_mux'] == 'true'
    assert declarations['initial_mode'] == initial
    if name != 'mode_mux':
        manager = next(a for a in actions if a.kind == 'IncludeLaunchDescription'
                       and a.args[0].endswith('/mode_mux.launch.py'))
        assert dict(manager.launch_arguments)['initial_mode'].name == 'initial_mode'
    else:
        nodes = [a for a in actions if a.kind == 'Node']
        assert {n.executable for n in nodes} == {'cmd_vel_mux', 'cmd_vel_watchdog'}
        for node in nodes:
            assert node.condition.kind == 'IfCondition'
            assert node.condition.args[0].name == 'use_mux'
            assert node.parameters[0]['use_sim_time'].name == 'use_sim_time'
