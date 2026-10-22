from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def include(package, filename, args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory(package)+'/launch/'+filename), launch_arguments=args.items())


def generate_launch_description():
    time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('person', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('slam_mode', default_value='mapping'),
        include('shopping_robot_gazebo', 'sim.launch.py', {'use_sim_time': time, 'headless': LaunchConfiguration('headless'), 'person': LaunchConfiguration('person')}),
        include('shopping_robot_navigation', 'navigation.launch.py', {'slam_mode': LaunchConfiguration('slam_mode'), 'map': LaunchConfiguration('map'), 'use_sim_time': time}),
        Node(package='shopping_zone_manager', executable='zone_navigator', parameters=[{'use_sim_time': time}], output='screen')])
