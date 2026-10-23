from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    parameters = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_mux', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('initial_mode', default_value='idle'),
        DeclareLaunchArgument('manage_follow', default_value='false'),
        Node(package='shopping_zone_manager', executable='cmd_vel_mux',
             condition=IfCondition(LaunchConfiguration('use_mux')),
             parameters=[parameters, {'initial_mode': LaunchConfiguration('initial_mode'),
                                      'manage_follow': ParameterValue(LaunchConfiguration('manage_follow'), value_type=bool)}], output='screen'),
        Node(package='shopping_zone_manager', executable='cmd_vel_watchdog',
             condition=IfCondition(LaunchConfiguration('use_mux')),
             parameters=[parameters], output='screen')])
