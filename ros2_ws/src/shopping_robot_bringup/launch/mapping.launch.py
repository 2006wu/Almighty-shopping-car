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
        include('shopping_robot_gazebo', 'sim.launch.py', {'use_sim_time': time, 'headless': LaunchConfiguration('headless')}),
        include('shopping_robot_slam', 'slam.launch.py', {'mode': 'mapping', 'use_sim_time': time})])
