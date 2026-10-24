from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    parameters = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    guard = LaunchConfiguration('obstacle_guard')
    collision = LaunchConfiguration('collision_monitor')
    use_mux = LaunchConfiguration('use_mux')
    def both(flag):
        return IfCondition(PythonExpression(["'", use_mux, "' == 'true' and '", flag, "' == 'true'"]))
    guard_output = PythonExpression(["'/cmd_vel_guard/output' if '", guard,
                                     "' == 'true' else '/cmd_vel_mux/output'"])
    watchdog_input = PythonExpression(["'/cmd_vel_collision/output' if '", collision,
        "' == 'true' else '/cmd_vel_guard/output' if '", guard,
        "' == 'true' else '/cmd_vel_mux/output'"])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_mux', default_value='true', choices=['true']),
        DeclareLaunchArgument('initial_mode', default_value='idle'),
        DeclareLaunchArgument('manage_follow', default_value='false'),
        DeclareLaunchArgument('obstacle_guard', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('collision_monitor', default_value='false', choices=['true', 'false']),
        Node(package='shopping_zone_manager', executable='obstacle_guard',
             condition=both(guard), parameters=[parameters, {'footprint_file':
                 get_package_share_directory('shopping_robot_navigation')+'/config/nav2_params.yaml'}],
             output='screen'),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', condition=both(collision),
             parameters=[get_package_share_directory('shopping_robot_bringup')+
                         '/config/collision_monitor.yaml', parameters,
                         {'cmd_vel_in_topic': ParameterValue(guard_output, value_type=str)}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='collision_monitor_lifecycle', condition=both(collision),
             parameters=[parameters, {'autostart': True, 'node_names': ['collision_monitor']}],
             output='screen'),
        Node(package='shopping_zone_manager', executable='cmd_vel_mux',
             condition=IfCondition(LaunchConfiguration('use_mux')),
             parameters=[parameters, {'initial_mode': LaunchConfiguration('initial_mode'),
                                      'manage_follow': ParameterValue(LaunchConfiguration('manage_follow'), value_type=bool)}], output='screen'),
        Node(package='shopping_zone_manager', executable='cmd_vel_watchdog',
             condition=IfCondition(LaunchConfiguration('use_mux')),
             parameters=[parameters], remappings=[('/cmd_vel_mux/output', watchdog_input)], output='screen')])
