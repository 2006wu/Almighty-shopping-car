import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('shopping_robot_gazebo')
    description = get_package_share_directory('shopping_robot_description')
    time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=share+'/worlds/supermarket.sdf'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', share+'/worlds:'+description+':'+os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')),
        ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', LaunchConfiguration('world')], condition=IfCondition(LaunchConfiguration('headless')), output='screen'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')], condition=UnlessCondition(LaunchConfiguration('headless')), output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description+'/launch/display.launch.py'), launch_arguments={'use_sim_time': time}.items()),
        Node(package='ros_gz_sim', executable='create', arguments=['-world', 'supermarket', '-name', 'shopping_robot', '-topic', '/robot_description', '-x', '0', '-y', '0', '-z', '0.02', '-Y', '0'], parameters=[{'use_sim_time': time}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{'config_file': share+'/config/bridge.yaml', 'use_sim_time': time}], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['0', '0', '0', '0', '0', '0', 'odom', 'sim_world'], parameters=[{'use_sim_time': time}]),
        Node(package='shopping_uwb_follow', executable='person_path_node', parameters=[{'use_sim_time': time, 'person_path': share+'/config/person_path.yaml'}], output='screen')])
