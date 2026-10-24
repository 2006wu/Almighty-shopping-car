"""Inspect and execute launch ASTs with fakes; no ROS graph or devices are used."""
import ast
import itertools
import os
from pathlib import Path
import pytest
from test_mode_launch import load, action, resolve


def sim_actions():
    env = load('shopping_robot_gazebo/launch/sim.launch.py')
    env.update(os=os, UnlessCondition=action('UnlessCondition'),
               LaunchConfigurationEquals=action('LaunchConfigurationEquals'),
               ExecuteProcess=action('ExecuteProcess'), SetEnvironmentVariable=action('SetEnvironmentVariable'))
    return env['generate_launch_description']()


@pytest.mark.parametrize('flags', list(itertools.product(('false', 'true'), repeat=4)))
def test_each_real_source_disables_its_simulation(flags):
    context = dict(zip(('use_real_lidar', 'use_real_imu', 'use_real_uwb', 'use_real_base'), flags))
    context['fault_injection'] = 'false'
    actions = sim_actions()
    bridges = {a.name: a for a in actions if a.kind == 'Node' and hasattr(a, 'name')}
    for kind in ('lidar', 'imu', 'base'):
        bridge = bridges[kind+'_bridge']
        assert bridge.condition.kind == 'UnlessCondition'
        assert resolve(bridge.condition.args[0], context) == context['use_real_'+kind]
        assert bridge.parameters[0]['config_file'].endswith('/bridge_'+kind+'.yaml')
    noise = next(a for a in actions if a.kind == 'Node' and a.executable == 'wheel_odom_noise')
    for parameter, flag in [('simulate_wheel', 'use_real_base'), ('simulate_imu', 'use_real_imu')]:
        assert resolve(noise.parameters[0][parameter].args[0], context) == (context[flag] == 'false')
    env = load('shopping_robot_bringup/launch/hardware.launch.py')
    env['UnlessCondition'] = action('UnlessCondition')
    harness = env['generate_launch_description']()
    declarations = {a.args[0]: a.default_value for a in harness if a.kind == 'DeclareLaunchArgument'}
    for flag in context.keys() - {'fault_injection'}:
        assert declarations[flag] == 'false'
    for node in [a for a in harness if a.kind == 'Node']:
        assert node.condition.kind == 'UnlessCondition'
        assert node.condition.args[0].name == 'use_real_uwb'
    sim = next(a for a in harness if a.kind == 'IncludeLaunchDescription' and a.args[0].endswith('/sim.launch.py'))
    forwarded = dict(sim.launch_arguments)
    assert forwarded['odom_source'] == 'ekf'
    for kind in ('lidar', 'imu', 'base'):
        assert resolve(forwarded['use_real_'+kind], context) == context['use_real_'+kind]
    drivers = [a for a in harness if a.kind == 'IncludeLaunchDescription' and 'placeholder' in a.args[0]]
    assert len(drivers) == 4
    for driver in drivers:
        assert driver.condition.kind == 'IfCondition'
        assert driver.condition.args[0].name in context
    mux = next(a for a in harness if a.kind == 'IncludeLaunchDescription' and a.args[0].endswith('/mode_mux.launch.py'))
    assert dict(mux.launch_arguments)['use_mux'] == 'true'
    assert dict(mux.launch_arguments)['initial_mode'] == 'idle'


@pytest.mark.parametrize('kind', ['lidar', 'imu', 'uwb', 'base'])
def test_placeholder_requires_explicit_pair(kind, tmp_path):
    env = load(f'shopping_robot_bringup/launch/{kind}_driver_placeholder.launch.py')
    env.update(Path=Path, LogError=action('LogError'), get_package_share_directory=lambda p: str(tmp_path/p))
    for package, filename in [('', ''), ('driver', ''), ('', 'test.launch.py'),
                              ('driver', '../test.launch.py'), ('driver', '/tmp/test.launch.py'),
                              ('driver', 'missing.launch.py')]:
        actions = env['start']({'driver_package': package, 'driver_launch': filename})
        assert [a.kind for a in actions] == ['LogError']
    path = tmp_path/'driver/launch/test.launch.py'
    path.parent.mkdir(parents=True)
    path.write_text('')
    include = env['start']({'driver_package': 'driver', 'driver_launch': path.name})[0]
    assert include.kind == 'IncludeLaunchDescription'
    assert include.args[0] == str(path)
    assert dict(include.launch_arguments)['use_sim_time'].name == 'use_sim_time'


def test_noise_publishers_are_inside_explicit_channel_guards():
    path = Path(__file__).resolve().parents[1]/'ros2_ws/src/shopping_uwb_follow/shopping_uwb_follow/wheel_odom_noise_node.py'
    tree = ast.parse(path.read_text())
    guards = [n for n in ast.walk(tree) if isinstance(n, ast.If)]
    for parameter, topic in [('simulate_wheel', '/wheel_odom_noisy'), ('simulate_imu', '/imu/with_covariance')]:
        guard = next(n for n in guards if parameter in ast.unparse(n.test))
        assert topic in ast.unparse(guard)


@pytest.mark.parametrize('kind', ['lidar', 'imu', 'base'])
def test_simulator_truth_cannot_replace_real_source_odometry(kind):
    env = load('shopping_robot_gazebo/launch/sim.launch.py')
    context = {'odom_source': 'ground_truth', 'fault_injection': 'false', 'use_real_lidar': 'false',
               'use_real_imu': 'false', 'use_real_base': 'false'}
    assert env['validate_sources'](context) == []
    context['use_real_'+kind] = 'true'
    with pytest.raises(RuntimeError, match='require odom_source:=ekf'):
        env['validate_sources'](context)
    context['odom_source'] = 'ekf'
    assert env['validate_sources'](context) == []
    context['fault_injection'] = 'true'
    with pytest.raises(RuntimeError, match='simulated fault publishers'):
        env['validate_sources'](context)


def test_missing_explicit_driver_package_logs_error():
    env = load('shopping_robot_bringup/launch/lidar_driver_placeholder.launch.py')
    def missing(package):
        raise LookupError(package)
    env.update(Path=Path, LogError=action('LogError'), get_package_share_directory=missing)
    actions = env['start']({'driver_package': 'missing', 'driver_launch': 'driver.launch.py'})
    assert [a.kind for a in actions] == ['LogError']
