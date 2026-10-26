"""Scenario schemas, geometry, trajectories and launch contracts without ROS."""
import ast
from copy import deepcopy
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

from shopping_robot_sim.scenarios import load_scenario, generate_world, validate_path
from shopping_robot_sim.trajectory import trajectory_position, tracking_velocity
from shopping_uwb_follow.person_path import path_position

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT/'ros2_ws/src'
CONFIG = SRC/'shopping_robot_gazebo/config'
WORLD = SRC/'shopping_robot_gazebo/worlds/supermarket.sdf'
NAMES = sorted(p.stem for p in (CONFIG/'scenarios').glob('*.yaml'))


def test_required_scenarios_and_canonical_default():
    assert set(NAMES) == {'default', 'aisle_crossing', 'narrow_pass', 'checkout_queue',
                          'follow_turns', 'follow_stop_go', 'alt_start'}
    assert load_scenario(CONFIG, 'default')['person_path'] == yaml.safe_load((CONFIG/'person_path.yaml').read_text())
    for t in (0, 1, 10, 100, 1000):
        assert path_position(load_scenario(CONFIG, 'default')['person_path'], t) == trajectory_position(load_scenario(CONFIG, 'default')['person_path'], t)


@pytest.mark.parametrize('name', ['../default', '', 'Default', '/default', None])
def test_bad_names(name):
    with pytest.raises(ValueError):
        load_scenario(CONFIG, name)


@pytest.mark.parametrize('mutate', [
    lambda d: d.update(unknown=0),
    lambda d: d.pop('robot_pose'),
    lambda d: d['robot_pose'].update(yaw=float('nan')),
    lambda d: d['robot_pose'].update(x=True),
    lambda d: d.update(person_path='../person_path.yaml'),
    lambda d: d.update(person_path=dict(speed=0, waypoints=[[0, 0], [1, 0]])),
    lambda d: d.update(person_path=dict(speed=1, waypoints=[[0, 0], [1, 0]], pauses=[0])),
    lambda d: d.update(person_path=dict(speed=1, waypoints=[[0, 0], [1, 0]], pauses=[0, -1])),
    lambda d: d.update(person_path=dict(speed=1, waypoints=[[0, 0, 0], [1, 0]])),
    lambda d: d.update(pedestrians={}),
    lambda d: d['pedestrians'][0].update(radius=0),
    lambda d: d['pedestrians'][0].update(speed=-1),
    lambda d: d['pedestrians'][0].update(start_delay=float('inf')),
    lambda d: d['pedestrians'][0].update(name='person'),
    lambda d: d['pedestrians'].append(deepcopy(d['pedestrians'][0])),
])
def test_schema_rejects_invalid_data(tmp_path, mutate):
    (tmp_path/'scenarios').mkdir()
    data = yaml.safe_load((CONFIG/'scenarios/aisle_crossing.yaml').read_text())
    data['person_path'] = yaml.safe_load((CONFIG/'person_path.yaml').read_text())
    mutate(data)
    (tmp_path/'scenarios/bad.yaml').write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_scenario(tmp_path, 'bad')


def test_trajectory_loop_speed_delay_and_static():
    path = dict(speed=2., waypoints=[[0, 0], [4, 0]])
    assert trajectory_position(path, 1, 2) == (0, 0)
    assert trajectory_position(path, 3, 2) == (2, 0)
    assert trajectory_position(path, 5, 2) == (2, 0)
    assert trajectory_position(path, 6, 2) == (0, 0)
    assert trajectory_position(dict(speed=0, waypoints=[[3, 2]]), 100) == (3, 2)
    path['pauses'] = [1, 2]
    assert trajectory_position(path, .5) == (0, 0)
    assert trajectory_position(path, 2) == (2, 0)
    assert trajectory_position(path, 4) == (4, 0)
    assert trajectory_position(path, 6) == (2, 0)
    assert trajectory_position(path, 7) == (0, 0)


def test_tracking_velocity_is_bounded_and_body_relative():
    p = dict(speed=.3, waypoints=[[0, 0], [4, 0]], start_delay=2)
    assert tracking_velocity(p, 1, (0, 0), 0) == (0, 0)
    assert tracking_velocity(p, 3, (0, 0), 0) == pytest.approx((.3, 0))
    assert tracking_velocity(p, 3, (0, 0), math.pi/2) == pytest.approx((0, -.3))
    assert math.hypot(*tracking_velocity(p, 100, (100, 100), 0)) <= .3+1e-12


def test_stop_go_pauses_and_near_start():
    path = load_scenario(CONFIG, 'follow_stop_go')['person_path']
    assert trajectory_position(path, 1) == (.58, 0)
    assert trajectory_position(path, 12) == (2.4, 0)
    assert trajectory_position(path, 25) == (2.4, 2)
    assert .3+.25 < path["waypoints"][0][0] < .6


@pytest.mark.parametrize('name', NAMES)
def test_generated_sdf_and_native_plugin(name):
    scenario = load_scenario(CONFIG, name)
    world = ET.fromstring(generate_world(WORLD, scenario)).find('world')
    for plugin in world.findall('.//plugin'):
        assert plugin.get('filename').startswith('ignition-gazebo-')
        assert plugin.get('name').startswith('ignition::gazebo::systems::')
    for p in scenario['pedestrians']:
        model = world.find(f"model[@name='{p['name']}']")
        assert model.findtext('static', 'false') == 'false'
        assert model.findtext('link/gravity') == 'false'
        assert float(model.findtext('link/collision/geometry/cylinder/radius')) == p['radius']
        assert model.find('link/visual/geometry/cylinder') is not None
        assert model.findtext('plugin/topic') == f"/model/{p['name']}/cmd_vel"


def test_launch_scenario_and_single_person_file_contract():
    for package, name in [('shopping_robot_gazebo', 'sim'), *[('shopping_robot_bringup', n) for n in ('mapping', 'guide', 'follow')]]:
        text = (SRC/package/f'launch/{name}.launch.py').read_text()
        tree = ast.parse(text)
        assert "DeclareLaunchArgument('scenario', default_value='default')" in text
        assert 'OpaqueFunction(function=prepare_scenario)' in text
        if name != 'sim':
            assert "'scenario': LaunchConfiguration('scenario')" in text
        if name in ('sim', 'follow'):
            assert "'person_path': LaunchConfiguration('scenario_person_path')" in text
            assert '/config/person_path.yaml' not in text
        assert tree
    sim = (SRC/'shopping_robot_gazebo/launch/sim.launch.py').read_text()
    assert "'initial_state': ParameterValue(LaunchConfiguration('scenario_initial_state'))" in sim
    assert "LaunchConfiguration('scenario_world')" in sim
    slam = (SRC/'shopping_robot_slam/launch/slam.launch.py').read_text()
    assert "overrides['map_start_pose']" in slam
    loc = (SRC/'shopping_zone_manager/shopping_zone_manager/localization_check.py').read_text()
    assert "self.parameter('scenario', 'default')" in loc


def test_scenario_list_without_docker():
    import os
    result = subprocess.run(['bash', str(ROOT/'scripts/dev.sh'), 'scenario-list'],
                            env={**os.environ, 'DOCKER': 'this-must-not-run',
                                 'PYTHON': sys.executable},
                            capture_output=True, text=True, check=True)
    assert all(name+':' in result.stdout for name in NAMES)


def test_materialization_is_once_per_launch_and_spawn_state_matches(tmp_path):
    """Execute the real launch callback with action fakes and verify its artifacts."""
    from types import SimpleNamespace as NS
    path = SRC/'shopping_robot_gazebo/shopping_robot_sim/scenario_launch.py'
    tree = ast.parse(path.read_text())
    tree.body = [node for node in tree.body if not isinstance(node, (ast.Import, ast.ImportFrom))]

    class Configuration:
        def __init__(self, name, default=None):
            self.name, self.default = name, default

        def perform(self, context):
            return context.launch_configurations.get(self.name, self.default)

    def action(kind):
        return lambda *args, **kwargs: NS(kind=kind, args=args, **kwargs)

    import json
    import shutil
    env = dict(Path=Path, json=json, shutil=shutil, yaml=yaml,
               tempfile=NS(mkdtemp=lambda **kw: str(tmp_path)),
               get_package_share_directory=lambda package: str(SRC/package),
               LaunchConfiguration=Configuration, load_scenario=load_scenario, generate_world=generate_world,
               **{name: action(name) for name in ('SetLaunchConfiguration', 'RegisterEventHandler',
                                                 'OnShutdown', 'OpaqueFunction', 'Node')})
    exec(compile(tree, str(path), 'exec'), env)
    context = NS(launch_configurations=dict(scenario='alt_start', use_sim_time='true'))
    actions = env['prepare_scenario'](context)
    for a in actions:
        if a.kind == 'SetLaunchConfiguration':
            context.launch_configurations[a.args[0]] = a.args[1]
    values = context.launch_configurations
    assert yaml.safe_load(Path(values['scenario_person_path']).read_text()) == load_scenario(CONFIG, 'alt_start')['person_path']
    state = json.loads(values['scenario_initial_state'])
    assert state[:6] == [-2.4, 0, 0, 0, 0, math.pi/2]
    assert len(state) == 15
    assert all(isinstance(value, float) for value in state)
    assert ET.parse(values['scenario_world']).find("world[@name='supermarket']") is not None
    assert env['prepare_scenario'](context) == []
    assert values['scenario_person_path'] == str(tmp_path/'person_path.yaml')


def test_pedestrian_watchdog_stops_on_missing_and_stale_truth():
    from types import SimpleNamespace as NS
    path = SRC/'shopping_robot_gazebo/shopping_robot_sim/pedestrian_mover.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    cls.bases = []
    scope = dict(Twist=lambda: 'zero')
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), scope)
    mover = scope['PedestrianMover'].__new__(scope['PedestrianMover'])
    commands = []
    mover.publishers_by_name = {'ped_test': NS(publish=commands.append)}
    mover.get_clock = lambda: NS(now=lambda: NS(nanoseconds=2_000_000_000))
    mover.last = None
    mover.watchdog()
    mover.last = 1.0
    mover.watchdog()
    mover.last = 3.0
    mover.watchdog()
    assert commands == ['zero']*3
    mover.last = 1.9
    mover.watchdog()
    assert commands == ['zero']*3


def test_cmake_program_entry_points_and_runtime_dependencies():
    cmake = (SRC/'shopping_robot_gazebo/CMakeLists.txt').read_text()
    for filename, executable in [('pedestrian_mover.py', 'pedestrian_mover'),
                                 ('initial_pose_node.py', 'scenario_initial_pose')]:
        text = (SRC/'shopping_robot_gazebo/shopping_robot_sim'/filename).read_text()
        assert text.startswith('#!/usr/bin/env python3')
        tree = ast.parse(text)
        assert any(isinstance(n, ast.If) and ast.unparse(n.test) == "__name__ == '__main__'"
                   and any(isinstance(c, ast.Call) and isinstance(c.func, ast.Name) and c.func.id == 'main'
                           for c in ast.walk(n)) for n in tree.body)
        assert f'RENAME {executable}' in cmake
    dependencies = {e.text for e in ET.parse(SRC/'shopping_robot_gazebo/package.xml').findall('exec_depend')}
    assert {'rclpy', 'geometry_msgs', 'tf2_msgs', 'ros_gz_bridge', 'shopping_uwb_follow'} <= dependencies


def test_initial_pose_targets_amcl_once_and_uses_scenario_heading():
    from types import SimpleNamespace as NS
    path = SRC/'shopping_robot_gazebo/shopping_robot_sim/initial_pose_node.py'
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    cls.bases = []

    def message():
        return NS(header=NS(), pose=NS(pose=NS(position=NS(), orientation=NS()), covariance=[0.]*36))

    scope = dict(math=math, PoseWithCovarianceStamped=message)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), scope)
    node = scope['InitialPose'].__new__(scope['InitialPose'])
    messages = []
    node.publisher = NS(publish=messages.append, get_subscription_count=lambda: 1)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1, to_msg=lambda: 'stamp'))
    node.get_subscriptions_info_by_topic = lambda topic: [NS(node_name='slam_toolbox')]
    node.sent = False
    node.pose = dict(x=-2.4, y=0, yaw=math.pi/2)
    node.get_logger = lambda: NS(info=lambda text: None)
    node.tick()
    assert messages == []
    node.get_subscriptions_info_by_topic = lambda topic: [NS(node_name='amcl')]
    node.tick()
    node.tick()
    assert len(messages) == 1
    assert messages[0].pose.pose.position.x == -2.4
    assert messages[0].pose.pose.orientation.z == pytest.approx(math.sin(math.pi/4))
    assert messages[0].pose.pose.orientation.w == pytest.approx(math.cos(math.pi/4))
    assert messages[0].header.frame_id == 'map'


def test_pedestrian_truth_has_individual_simulator_timestamps():
    scenario = load_scenario(CONFIG, 'checkout_queue')
    world = ET.fromstring(generate_world(WORLD, scenario)).find('world')
    for person in scenario['pedestrians']:
        model = world.find(f"model[@name='{person['name']}']")
        plugin = next(p for p in model.findall('plugin') if p.get('name').endswith('PosePublisher'))
        assert plugin.find('topic') is None  # Fortress uses the model-scoped pose topic.
        assert plugin.findtext('publish_model_pose') == 'true'
        assert plugin.findtext('publish_nested_model_pose') == 'true'
        assert plugin.findtext('update_frequency') == '30'


def test_pedestrian_merges_stamped_model_messages_without_using_scene_time():
    from types import SimpleNamespace as NS
    path = SRC/'shopping_robot_gazebo/shopping_robot_sim/pedestrian_mover.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    cls.bases = []
    scope = dict(math=math, tracking_velocity=tracking_velocity,
                 Twist=lambda: NS(linear=NS(x=0., y=0.)),
                 Pose=lambda: NS(position=NS(x=0., y=0., z=0.)),
                 PoseArray=lambda: NS(header=NS(), poses=[]))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), scope)
    mover = scope['PedestrianMover'].__new__(scope['PedestrianMover'])
    mover.people = [dict(name=name, speed=.3, start_delay=0,
                         waypoints=[[0, 0], [1, 0]]) for name in ('a', 'b')]
    commands, arrays = [], []
    mover.publishers_by_name = {name: NS(publish=commands.append) for name in ('a', 'b')}
    mover.truth = NS(publish=arrays.append)
    mover.last, mover.measured = None, {}

    def message(name, sec, nsec=0):
        return NS(transforms=[NS(child_frame_id=name, header=NS(stamp=NS(sec=sec, nanosec=nsec)),
                                transform=NS(translation=NS(x=0., y=0., z=.85),
                                             rotation=NS(w=1., x=0., y=0., z=0.)))])

    mover.on_poses(message('a', 0))
    mover.on_poses(message('a', 2))
    assert not arrays and not commands
    mover.on_poses(message('b', 2, 20_000_000))
    assert len(arrays) == 1 and len(arrays[0].poses) == 2
    assert arrays[0].header.stamp.sec == 2
    assert arrays[0].header.stamp.nanosec == 0
    assert len(commands) == 2
    assert all(math.hypot(c.linear.x, c.linear.y) <= .3 for c in commands)
    mover.on_poses(message('a', 3))
    assert len(arrays) == 1  # Never combine old and new measurements as fresh truth.
