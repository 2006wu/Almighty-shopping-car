from launch import LaunchDescription
from shopping_robot_sim.scenario_launch import prepare_scenario
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
        DeclareLaunchArgument('fault_injection', default_value='false'),
        DeclareLaunchArgument('fault_schedule', default_value='[]'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('odom_source', default_value='ekf', choices=['ekf', 'ground_truth']),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('person', default_value='false'),
        include('shopping_robot_gazebo', 'sim.launch.py', {'fault_injection': LaunchConfiguration('fault_injection'), 'fault_schedule': LaunchConfiguration('fault_schedule'), 'scenario': LaunchConfiguration('scenario'), 'odom_source': LaunchConfiguration('odom_source'), 'use_sim_time': time, 'headless': LaunchConfiguration('headless'), 'person': LaunchConfiguration('person')}),
        include('shopping_robot_slam', 'slam.launch.py', {'mode': 'mapping', 'use_sim_time': time})])
