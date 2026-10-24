from launch import LaunchDescription
from shopping_robot_sim.scenario_launch import prepare_scenario
from launch.conditions import IfCondition
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def include(package, filename, args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory(package)+'/launch/'+filename), launch_arguments=args.items())


def generate_launch_description():
    time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='default'),
        OpaqueFunction(function=prepare_scenario),
        DeclareLaunchArgument('mcu_comm', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('fault_injection', default_value='false'),
        DeclareLaunchArgument('fault_schedule', default_value='[]'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('shopping_mcu_bridge')+'/launch/comm.launch.py'),
            condition=IfCondition(LaunchConfiguration('mcu_comm')),
            launch_arguments={'use_sim_time': time}.items()),
        DeclareLaunchArgument('obstacle_guard', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('collision_monitor', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_mux', default_value='true', choices=['true']),
        DeclareLaunchArgument('initial_mode', default_value='guide'),
        DeclareLaunchArgument('odom_source', default_value='ekf', choices=['ekf', 'ground_truth']),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('person', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('nav2_stack', default_value='apt', choices=['auto', 'fork', 'apt']),
        DeclareLaunchArgument('slam_mode', default_value='mapping'),
        include('shopping_robot_gazebo', 'sim.launch.py', {'fault_injection': LaunchConfiguration('fault_injection'), 'fault_schedule': LaunchConfiguration('fault_schedule'), 'scenario': LaunchConfiguration('scenario'), 'odom_source': LaunchConfiguration('odom_source'), 'use_sim_time': time, 'headless': LaunchConfiguration('headless'), 'person': LaunchConfiguration('person')}),
        include('shopping_robot_navigation', 'navigation.launch.py', {'use_mux': LaunchConfiguration('use_mux'), 'nav2_stack': LaunchConfiguration('nav2_stack'), 'slam_mode': LaunchConfiguration('slam_mode'), 'map': LaunchConfiguration('map'), 'use_sim_time': time}),
        include('shopping_robot_bringup', 'mode_mux.launch.py', {'use_mux': LaunchConfiguration('use_mux'), 'obstacle_guard': LaunchConfiguration('obstacle_guard'), 'collision_monitor': LaunchConfiguration('collision_monitor'), 'initial_mode': LaunchConfiguration('initial_mode'), 'use_sim_time': time}),
        Node(package='shopping_zone_manager', executable='zone_navigator', parameters=[{'use_sim_time': time}], output='screen')])
