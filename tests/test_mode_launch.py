"""Evaluate launch wiring with small substitution/action fakes, without ROS."""
import ast
import math
import os
from shopping_robot_navigation.stack_selection import select_stack, apt_environment
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

SRC = Path(__file__).resolve().parents[1]/'ros2_ws/src'


class Configuration:
    def __init__(self, name, default=None):
        self.name = name
        self.default = default

    def perform(self, context):
        return context.get(self.name, self.default)


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
        'SetRemap', 'OpaqueFunction', 'ParameterValue', 'IfCondition', 'LogInfo', 'ExecuteProcess', 'RewrittenYaml')}
    env['prepare_scenario'] = lambda context: []
    env['pedestrian_actions'] = lambda context: []
    env.update(LaunchDescription=lambda values: values, LaunchConfiguration=Configuration,
               PythonExpression=Expression, PythonLaunchDescriptionSource=lambda path: path,
               get_package_share_directory=lambda package: '/share/'+package,
               get_packages_with_prefixes=lambda: {}, select_stack=select_stack,
               apt_environment=apt_environment, os=os, Path=Path, math=math)
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


@pytest.mark.parametrize('name', ['guide', 'follow'])
@pytest.mark.parametrize('stack', ['apt', 'fork', 'auto'])
def test_scenarios_preserve_nav2_and_optional_mcu_wiring(name, stack):
    actions = load(f'shopping_robot_bringup/launch/{name}.launch.py')['generate_launch_description']()
    declarations = {a.args[0]: a for a in actions if a.kind == 'DeclareLaunchArgument'}
    assert declarations['scenario'].default_value == 'default'
    assert declarations['nav2_stack'].default_value == 'auto'
    assert declarations['nav2_stack'].choices == ['auto', 'fork', 'apt']
    assert declarations['mcu_comm'].default_value == 'false'
    assert declarations['use_mux'].default_value == 'true'
    includes = {a.args[0].split('/')[-1]: a for a in actions
                if a.kind == 'IncludeLaunchDescription'}
    context = {'scenario': 'alt_start', 'nav2_stack': stack, 'use_sim_time': 'true'}
    for filename, argument in [('sim.launch.py', 'scenario'),
                               ('navigation.launch.py', 'nav2_stack')]:
        arguments = dict(includes[filename].launch_arguments)
        assert resolve(arguments[argument], context) == context[argument]
        assert resolve(arguments['use_sim_time'], context) == 'true'
    comm = includes['comm.launch.py']
    assert comm.condition.kind == 'IfCondition'
    assert comm.condition.args[0].name == 'mcu_comm'
    assert resolve(dict(comm.launch_arguments)['use_sim_time'], context) == 'true'
    if name == 'follow':
        bridge = next(a for a in actions if a.kind == 'Node' and
                      getattr(a, 'name', '') == 'person_pose_bridge')
        assert bridge.parameters[0]['config_file'].endswith('/config/bridge_person.yaml')
        uwb = next(a for a in actions if a.kind == 'Node' and a.executable == 'uwb_sim_node')
        assert uwb.parameters[1]['person_path'].name == 'scenario_person_path'


@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_nav2_velocity_outputs_are_remapped(enabled, monkeypatch):
    monkeypatch.delenv('AMENT_PREFIX_PATH', raising=False)
    env = load('shopping_robot_navigation/launch/navigation.launch.py')
    actions = env['start']({'use_mux': enabled, 'slam_mode': 'mapping', 'map': '', 'nav2_stack': 'apt'})
    group = actions[-1]
    remaps = [a for a in group.actions if a.kind == 'SetRemap']
    assert [(a.src, a.dst) for a in remaps] == (
        [('cmd_vel_smoothed', '/cmd_vel/nav'), ('behavior_server:cmd_vel', '/cmd_vel/nav'),
         ('bt_navigator_navigate_to_pose_rclcpp_node:/cmd_vel', '/cmd_vel/nav'),
         ('bt_navigator_navigate_through_poses_rclcpp_node:/cmd_vel', '/cmd_vel/nav')] if enabled == 'true' else [])
    include = group.actions[-1]
    assert include.args[0].endswith('/nav2_bringup/launch/navigation_launch.py')
    assert dict(include.launch_arguments)['use_composition'] == 'False'


@pytest.mark.parametrize('spawn', [{}, dict(spawn_x='', spawn_y='', spawn_yaw=''),
                                  dict(spawn_x='-2.4', spawn_y='0', spawn_yaw='1.5707963267948966')])
def test_slam_localization_uses_scenario_pose_only_when_supplied(spawn, tmp_path):
    class Context(dict):
        @property
        def launch_configurations(self):
            return self

    env = load('shopping_robot_slam/launch/slam.launch.py')
    prefix = str(tmp_path/'map')
    for suffix in ('.posegraph', '.data'):
        Path(prefix+suffix).write_bytes(b'test')
    actions = env['start'](Context(mode='localization', map=prefix, use_sim_time='true', **spawn))
    parameters = actions[0].parameters[-1]
    if spawn.get('spawn_x'):
        assert parameters['map_start_at_dock'] is False
        assert parameters['map_start_pose'] == [-2.4, 0.0, 1.5707963267948966]
    else:
        assert 'map_start_pose' not in parameters
        assert 'map_start_at_dock' not in parameters


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
        assert {n.executable for n in nodes} == {
            'cmd_vel_mux', 'cmd_vel_watchdog', 'obstacle_guard',
            'collision_monitor', 'lifecycle_manager'}
        nodes = [n for n in nodes if n.executable.startswith('cmd_vel_')]
        for node in nodes:
            assert node.condition.kind == 'IfCondition'
            assert node.condition.args[0].name == 'use_mux'
            assert node.parameters[0]['use_sim_time'].name == 'use_sim_time'


@pytest.mark.parametrize('guard', ['true', 'false'])
@pytest.mark.parametrize('collision', ['true', 'false'])
def test_safety_chain_has_no_final_velocity_bypass(guard, collision):
    actions = load('shopping_robot_bringup/launch/mode_mux.launch.py')['generate_launch_description']()
    context = {'use_mux': 'true', 'obstacle_guard': guard, 'collision_monitor': collision}
    nodes = {a.executable: a for a in actions if a.kind == 'Node'}
    watchdog = nodes['cmd_vel_watchdog']
    assert resolve(watchdog.remappings[0][1], context) == (
        '/cmd_vel_collision/output' if collision == 'true' else
        '/cmd_vel_guard/output' if guard == 'true' else '/cmd_vel_mux/output')
    monitor = nodes['collision_monitor']
    assert resolve(monitor.parameters[-1]['cmd_vel_in_topic'].args[0], context) == (
        '/cmd_vel_guard/output' if guard == 'true' else '/cmd_vel_mux/output')
    assert resolve(nodes['obstacle_guard'].condition.args[0], context) == (guard == 'true')
    assert resolve(monitor.condition.args[0], context) == (collision == 'true')
    assert resolve(nodes['lifecycle_manager'].condition.args[0], context) == (collision == 'true')
    declarations = {a.args[0]: a for a in actions if a.kind == 'DeclareLaunchArgument'}
    assert declarations['obstacle_guard'].default_value == 'true'
    assert declarations['collision_monitor'].default_value == 'false'
    assert declarations['use_mux'].choices == ['true']
    # Launch arguments must exist before nodes evaluate their substitutions.
    first_node = next(i for i, a in enumerate(actions) if a.kind == 'Node')
    assert all(a.kind == 'DeclareLaunchArgument' for a in actions[:first_node])


@pytest.mark.parametrize('name', ['guide', 'follow'])
@pytest.mark.parametrize('stack', ['apt', 'fork', 'auto'])
@pytest.mark.parametrize('guard,collision', [
    ('true', 'false'), ('true', 'true'), ('false', 'false'), ('false', 'true')])
def test_stack_selection_and_safety_arguments_are_forwarded_together(name, stack, guard, collision):
    actions = load(f'shopping_robot_bringup/launch/{name}.launch.py')['generate_launch_description']()
    declarations = {a.args[0]: a for a in actions if a.kind == 'DeclareLaunchArgument'}
    expected = {'mcu_comm', 'fault_injection', 'fault_schedule', 'use_sim_time',
                'obstacle_guard', 'collision_monitor', 'use_mux', 'initial_mode',
                'odom_source', 'headless', 'nav2_stack', 'slam_mode', 'map', 'scenario',
                'route_around' if name == 'follow' else 'person',
                # Container-split role switches.
                'start_sim', 'start_navigation', 'start_app', 'start_nav_localization',
                'start_sim_localization', 'mcu_transport'} | ({'start_uwb_sim'} if name == 'follow' else {'pedestrian_yield'})
    assert set(declarations) == expected
    assert declarations['nav2_stack'].default_value == 'auto'
    assert set(declarations['nav2_stack'].choices) == {'apt', 'fork', 'auto'}
    assert declarations['use_mux'].choices == ['true']
    assert declarations['obstacle_guard'].default_value == 'true'
    assert declarations['collision_monitor'].default_value == 'false'
    assert declarations['mcu_comm'].default_value == 'false'
    context = {key: a.default_value for key, a in declarations.items()}
    context.update(nav2_stack=stack, obstacle_guard=guard, collision_monitor=collision,
                   slam_mode='map_server', map='/maps/shop.yaml', use_sim_time='false')
    includes = {Path(a.args[0]).name: dict(a.launch_arguments)
                for a in actions if a.kind == 'IncludeLaunchDescription'}
    for key in ('nav2_stack', 'use_mux', 'slam_mode', 'map', 'use_sim_time'):
        assert resolve(includes['navigation.launch.py'][key], context) == context[key]
    for key in ('obstacle_guard', 'collision_monitor', 'use_mux', 'initial_mode', 'use_sim_time'):
        assert resolve(includes['mode_mux.launch.py'][key], context) == context[key]
    for key in ('odom_source', 'headless', 'fault_injection', 'fault_schedule', 'use_sim_time'):
        assert resolve(includes['sim.launch.py'][key], context) == context[key]
    assert resolve(includes['comm.launch.py']['use_sim_time'], context) == 'false'
    if name == 'follow':
        assert includes['mode_mux.launch.py']['manage_follow'] == 'true'
    else:
        assert resolve(includes['sim.launch.py']['person'], context) == context['person']


def test_follow_has_navigation_catchup_headroom_with_guide_default_preserved(monkeypatch):
    monkeypatch.delenv('AMENT_PREFIX_PATH', raising=False)
    actions = load('shopping_robot_bringup/launch/follow.launch.py')['generate_launch_description']()
    navigation = next(a for a in actions if a.kind == 'IncludeLaunchDescription'
                      and a.args[0].endswith('/shopping_robot_navigation/launch/navigation.launch.py'))
    assert dict(navigation.launch_arguments)['rpp_desired_speed'] == '0.5'
    start = load('shopping_robot_navigation/launch/navigation.launch.py')['start']
    base = dict(use_mux='true', slam_mode='mapping', map='', nav2_stack='apt')
    guide = start(base)[-1].actions[-1]
    assert isinstance(dict(guide.launch_arguments)['params_file'], str)
    follow = start({**base, 'rpp_desired_speed': '0.5'})[-1].actions[-1]
    assert dict(follow.launch_arguments)['params_file'].param_rewrites == {'desired_linear_vel': '0.5'}


@pytest.mark.parametrize('speed', ['0', '-0.1', '0.6', 'nan', 'inf', 'invalid'])
def test_navigation_speed_override_rejects_unsafe_values(speed):
    start = load('shopping_robot_navigation/launch/navigation.launch.py')['start']
    with pytest.raises(ValueError):
        start(dict(use_mux='true', slam_mode='mapping', map='', nav2_stack='apt', rpp_desired_speed=speed))


@pytest.mark.parametrize('missing', ['.posegraph', '.data'])
def test_slam_localization_rejects_incomplete_or_empty_graph(tmp_path, missing):
    env = load('shopping_robot_slam/launch/slam.launch.py')
    prefix = str(tmp_path/'map')
    for suffix in ('.posegraph', '.data'):
        Path(prefix+suffix).write_bytes(b'' if suffix == missing else b'test')
    with pytest.raises(ValueError, match='Missing or empty SLAM artifact'):
        env['start'](dict(mode='localization', map=prefix, use_sim_time='true'))
