"""Guard role ownership, Compose safety and shell dispatch without ROS/Docker."""
from collections import Counter
import json
import os
from pathlib import Path
import subprocess

import pytest
import yaml

from test_mode_launch import load, action, resolve

ROOT = Path(__file__).resolve().parents[1]


def test_compose_roles_share_namespace_image_and_dds():
    services = yaml.safe_load((ROOT/'docker/docker-compose.yaml').read_text())['services']
    assert {k for k, v in services.items() if not v.get('profiles')} == {'compute', 'display'}
    for name in ('sim', 'localization', 'navigation', 'app', 'record'):
        service = services[name]
        assert service['profiles'] == (['record'] if name == 'record' else ['split'])
        assert service['image'] == services['compute']['image']
        assert service['build'] == services['compute']['build']
    for name, service in services.items():
        assert not any(service.get(key) for key in ('devices', 'privileged', 'platform'))
        assert '/dev/tty' not in str(service)
        if name != 'compute':
            assert service['network_mode'] == 'service:compute'
            assert not service.get('ports') and not service.get('networks')
        assert service['environment']['ROS_DOMAIN_ID'] == '${ROS_DOMAIN_ID:-28}'
        if name == 'microros_client':
            assert service['profiles'] == ['comm']
            continue  # XRCE is not a DDS participant; the agent owns its DDS side.
        for key in ('RMW_IMPLEMENTATION', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'ROS_LOCALHOST_ONLY'):
            assert service['environment'][key] == services['compute']['environment'][key]
        assert './fastdds_udp.xml:/etc/fastdds_udp.xml:ro' in service['volumes']
    for name in ('localization', 'navigation', 'app'):
        assert services[name]['depends_on']['sim']['condition'] == 'service_healthy'
    assert 'ros2' not in str(services['sim']['healthcheck'])
    serial = yaml.safe_load((ROOT/'docker/docker-compose.serial.yaml').read_text())['services']['microros_agent']
    assert serial['command'][0] == 'serial'
    assert '${MICROROS_SERIAL_DEVICE:?' in str(serial['command'])
    assert not serial.get('devices')


def enabled(condition, context):
    if condition is None:
        return True
    if condition.kind == 'LaunchConfigurationEquals':
        return context[condition.args[0]] == condition.args[1]
    value = str(resolve(condition.args[0], context)).lower() in ('true', '1')
    return not value if condition.kind == 'UnlessCondition' else value


class Scope(dict):
    """Dict context that also exposes launch_configurations like LaunchContext."""

    @property
    def launch_configurations(self):
        return self


def nodes_for(filename, context):
    env = load(filename)
    env.update({name: action(name) for name in (
        'UnlessCondition', 'LaunchConfigurationEquals', 'SetEnvironmentVariable')})

    def walk(actions, scope):
        for item in actions:
            if item.kind == 'DeclareLaunchArgument':
                if item.args[0] not in scope and hasattr(item, 'default_value'):
                    scope[item.args[0]] = resolve(item.default_value, scope)
                continue
            if not enabled(getattr(item, 'condition', None), scope):
                continue
            if item.kind == 'OpaqueFunction':
                yield from walk(item.function(Scope(scope)), scope)
            elif item.kind == 'GroupAction':
                yield from walk(item.actions, dict(scope))
            elif item.kind == 'IncludeLaunchDescription':
                path = item.args[0]
                if path.startswith('/share/shopping_') and '/shopping_robot_description/' not in path:
                    args = {k: resolve(v, scope) for k, v in item.launch_arguments}
                    yield from nodes_for(path.removeprefix('/share/'), {**scope, **args})
            elif item.kind == 'Node':
                yield item
    # The generic loader returns an action list for LaunchDescription.
    yield from walk(env['generate_launch_description'](), dict(context))


@pytest.mark.parametrize('mode', ['guide', 'follow'])
@pytest.mark.parametrize('odom_source', ['ekf', 'ground_truth'])
@pytest.mark.parametrize('slam_mode', ['mapping', 'localization', 'map_server'])
def test_role_node_ownership(mode, odom_source, slam_mode, monkeypatch, tmp_path):
    monkeypatch.delenv('AMENT_PREFIX_PATH', raising=False)
    prefix = str(tmp_path/'map')
    for suffix in ('.posegraph', '.data'):
        Path(prefix+suffix).write_bytes(b'test')
    roles = {role: list(nodes_for(f'shopping_robot_bringup/launch/{role}.launch.py', {
        'mode': mode, 'odom_source': odom_source, 'slam_mode': slam_mode, 'map': prefix}))
        for role in ('sim', 'localization', 'navigation', 'app')}
    names = {role: Counter(n.executable for n in nodes) for role, nodes in roles.items()}
    assert names['app']['cmd_vel_watchdog'] == names['app']['cmd_vel_mux'] == 1
    for role in ('sim', 'localization', 'navigation'):
        assert names[role]['cmd_vel_watchdog'] == names[role]['cmd_vel_mux'] == 0
    assert names['sim']['sim_mcu_firmware'] == 1
    assert names['sim']['ekf_node'] == 0
    assert names['localization']['ekf_node'] == (odom_source == 'ekf')
    assert names['localization']['async_slam_toolbox_node'] == (slam_mode == 'mapping')
    assert names['localization']['localization_slam_toolbox_node'] == (slam_mode == 'localization')
    assert not names['navigation']  # Nav2's existing external launch owns its nodes.
    if mode == 'follow':
        assert names['sim']['uwb_sim_node'] == 1
        assert names['app']['uwb_sim_node'] == 0
        assert names['app']['uwb_estimator_node'] == names['app']['follow_controller_node'] == 1
    else:
        assert names['app']['zone_navigator'] == 1
    for nodes in roles.values():
        for node in nodes:
            for params in getattr(node, 'parameters', []):
                if isinstance(params, dict) and 'use_sim_time' in params:
                    # The role launch fixes a single clock for every included launch.
                    assert resolve(params['use_sim_time'], {'use_sim_time': 'true'}) == 'true'


def test_split_commands_and_literal_arguments(tmp_path):
    fake = tmp_path/'docker'
    log = tmp_path/'calls'
    fake.write_text('#!/usr/bin/env python3\nimport json, os, sys\n'
                    'with open(os.environ["CALLS"], "a") as f:\n'
                    ' f.write(json.dumps([sys.argv[1:], os.getenv("SPLIT_MODE"), '
                    'os.getenv("SPLIT_LAUNCH_ARGS")])+"\\n")\n')
    fake.chmod(0o755)
    env = dict(os.environ, DOCKER=str(fake), CALLS=str(log))
    arg = 'map:=/ws/map with spaces;$(touch nope).yaml'
    subprocess.run(['bash', str(ROOT/'scripts/dev.sh'), 'split-up', 'follow', arg], env=env, check=True)
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    assert calls[0][0][-1] == 'down'
    assert calls[1][0][-8:] == ['up', '-d', 'compute', 'sim', 'localization', 'navigation', 'app', 'display']
    assert calls[1][1:] == ['follow', arg]
    for bad in ('use_mux:=false', 'use_sim_time:=false', 'mcu_transport:=serial_loopback', 'map:=a\nb'):
        result = subprocess.run(['bash', str(ROOT/'scripts/dev.sh'), 'split-up', bad], env=env,
                                capture_output=True)
        assert result.returncode == 2
    assert len(log.read_text().splitlines()) == 2
    for command in (['split-status'], ['split-logs', 'navigation'], ['split-down']):
        subprocess.run(['bash', str(ROOT/'scripts/dev.sh'), *command], env=env, check=True)
    help_text = subprocess.check_output(['bash', str(ROOT/'scripts/dev.sh'), 'help'], text=True)
    for name in ('split-up', 'split-down', 'split-status', 'split-logs'):
        assert name in help_text


def test_role_runner_preserves_spaces_and_never_evaluates_arguments(tmp_path):
    fake = tmp_path/'ros2'
    fake.write_text('#!/usr/bin/env python3\nimport json, sys\nprint(json.dumps(sys.argv[1:]))\n')
    fake.chmod(0o755)
    marker = tmp_path/'unsafe'
    arg = f'map:=/map with spaces/$(touch {marker}).yaml'
    env = dict(os.environ, PATH=str(tmp_path)+os.pathsep+os.environ['PATH'],
               SPLIT_MODE='guide', SPLIT_LAUNCH_ARGS=arg)
    result = subprocess.check_output(['bash', str(ROOT/'scripts/container/split_role.sh'), 'navigation'],
                                     env=env, text=True)
    assert json.loads(result) == ['launch', 'shopping_robot_bringup', 'navigation.launch.py', 'mode:=guide', arg]
    assert not marker.exists()


@pytest.mark.parametrize('mode, suffix', [
    ('mapping', '/shopping_robot_slam/launch/slam.launch.py'),
    ('localization', '/shopping_robot_slam/launch/slam.launch.py'),
    ('map_server', '/nav2_bringup/launch/localization_launch.py'),
])
def test_localization_and_navigation_includes_have_separate_switches(mode, suffix, monkeypatch):
    monkeypatch.delenv('AMENT_PREFIX_PATH', raising=False)
    actions = load('shopping_robot_navigation/launch/navigation.launch.py')['start']({
        'slam_mode': mode, 'map': '/ws/map.yaml', 'nav2_stack': 'apt', 'use_mux': 'true'})
    assert actions[1].args[0].endswith(suffix)
    assert actions[1].condition.args[0].name == 'start_localization'
    assert actions[-1].condition.args[0].name == 'start_navigation'


def test_apt_isolation_keeps_role_switches(monkeypatch):
    monkeypatch.setenv('AMENT_PREFIX_PATH', '/nav2_ws/install/nav2_msgs:/opt/ros/humble')
    start = load('shopping_robot_navigation/launch/navigation.launch.py')['start']
    for role in ('localization', 'navigation'):
        args = {'slam_mode': 'mapping', 'map': '', 'nav2_stack': 'apt',
                'use_mux': 'true', 'use_sim_time': 'true',
                'start_localization': str(role == 'localization').lower(),
                'start_navigation': str(role == 'navigation').lower()}
        process = start(args)[-1]
        assert 'map:=' not in process.cmd
        for name in ('start_localization', 'start_navigation'):
            assert f'{name}:={args[name]}' in process.cmd


def test_sim_health_requires_world_clock_without_ros(tmp_path):
    fake = tmp_path/'ign'
    fake.write_text('#!/bin/sh\nprintf "%s\\n" "$FAKE_TOPICS"\n')
    fake.chmod(0o755)
    env = dict(os.environ, PATH=str(tmp_path)+os.pathsep+os.environ['PATH'])
    for topics, status in (('', 1), ('/other/clock', 1),
                           ('/world/supermarket/clock\n/scan', 0)):
        result = subprocess.run([os.sys.executable, str(ROOT/'scripts/container/sim_health.py')],
                                env=dict(env, FAKE_TOPICS=topics))
        assert result.returncode == status
