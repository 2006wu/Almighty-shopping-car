"""Launch-only scenario materialization shared by sim and bringup."""
import json
from pathlib import Path
import shutil
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from .scenarios import generate_world, load_scenario


def prepare_scenario(context):
    # Parent bringup resolves once; the included sim inherits these exact files.
    if context.launch_configurations.get('scenario_person_path'):
        return []
    share = Path(get_package_share_directory('shopping_robot_gazebo'))
    name = LaunchConfiguration('scenario').perform(context)
    scenario = load_scenario(share/'config', name)
    world = context.launch_configurations.get('world', str(share/'worlds/supermarket.sdf'))
    content = generate_world(world, scenario)
    directory = Path(tempfile.mkdtemp(prefix='shopping-scenario-'))
    path = share/'config/person_path.yaml'
    if name != 'default':
        path = directory/'person_path.yaml'
        path.write_text(yaml.safe_dump(scenario['person_path']), encoding='utf-8')
    world_path = directory/'world.sdf'
    world_path.write_text(content, encoding='utf-8')
    scenario_path = directory/'scenario.yaml'
    scenario_path.write_text(yaml.safe_dump(scenario), encoding='utf-8')
    pose = scenario['robot_pose']
    state = [float(pose['x']), float(pose['y']), 0.0, 0.0, 0.0, float(pose['yaw'])]+[0.0]*9
    actions = [SetLaunchConfiguration('scenario_person_path', str(path)),
               SetLaunchConfiguration('scenario_world', str(world_path)),
               SetLaunchConfiguration('scenario_file', str(scenario_path)),
               SetLaunchConfiguration('scenario_initial_state', json.dumps(state))]
    actions += [SetLaunchConfiguration('spawn_'+key, str(value)) for key, value in pose.items()]
    actions.append(Node(package='shopping_robot_gazebo', executable='scenario_initial_pose',
                        parameters=[{'scenario_file': str(scenario_path),
                                     'use_sim_time': LaunchConfiguration('use_sim_time', default='true')}],
                        output='screen'))

    def cleanup(_context, **_kwargs):
        shutil.rmtree(directory, ignore_errors=True)
        return []
    actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
    return actions


def pedestrian_actions(context):
    file = LaunchConfiguration('scenario_file').perform(context)
    scenario = yaml.safe_load(Path(file).read_text(encoding='utf-8'))
    people = scenario['pedestrians']
    if not people:
        return []
    time = LaunchConfiguration('use_sim_time')
    # Only longstanding Humble Twist topic conversion is required.
    arguments = [f"/model/{p['name']}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist" for p in people]
    return [Node(package='ros_gz_bridge', executable='parameter_bridge', name='pedestrian_bridge',
                 arguments=arguments, parameters=[{'use_sim_time': time}], output='screen'),
            Node(package='shopping_robot_gazebo', executable='pedestrian_mover',
                 parameters=[{'scenario_file': file, 'use_sim_time': time}], output='screen')]
