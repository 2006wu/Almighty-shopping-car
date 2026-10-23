from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_directory


def start(context):
    mode = LaunchConfiguration('slam_mode').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    if mode not in ('mapping', 'localization', 'map_server'):
        raise ValueError('slam_mode must be mapping, localization or map_server')
    if mode != 'mapping' and not map_file:
        raise ValueError('Selected localization mode requires map:=...')
    time = LaunchConfiguration('use_sim_time')
    params = get_package_share_directory('shopping_robot_navigation')+'/config/nav2_params.yaml'
    nav = get_package_share_directory('nav2_bringup')+'/launch/'
    common = {'use_sim_time': time, 'params_file': params, 'autostart': 'true', 'use_composition': 'False'}
    actions = []
    if mode == 'map_server':
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(nav+'localization_launch.py'), launch_arguments={**common, 'map': map_file}.items()))
    else:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('shopping_robot_slam')+'/launch/slam.launch.py'), launch_arguments={'use_sim_time': time, 'mode': mode, 'map': map_file}.items()))
    navigation = IncludeLaunchDescription(PythonLaunchDescriptionSource(nav+'navigation_launch.py'), launch_arguments=common.items())
    # Override only the smoother output; preserve Nav2's internal cmd_vel_nav.
    remaps = [SetRemap(src='cmd_vel_smoothed', dst='/cmd_vel/nav')] if LaunchConfiguration('use_mux').perform(context).lower() == 'true' else []
    actions.append(GroupAction(actions=[*remaps, navigation]))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('use_mux', default_value='true', choices=['true', 'false']),
                              DeclareLaunchArgument('use_sim_time', default_value='true'),
                              DeclareLaunchArgument('map', default_value=''),
                              DeclareLaunchArgument('slam_mode', default_value='mapping'), OpaqueFunction(function=start)])
