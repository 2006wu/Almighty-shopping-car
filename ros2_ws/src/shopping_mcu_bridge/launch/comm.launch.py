from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    time = LaunchConfiguration('use_sim_time')
    transport = LaunchConfiguration('transport')
    params = {'use_sim_time': time}
    return LaunchDescription([
        DeclareLaunchArgument('transport', default_value='sim_ros', choices=['sim_ros', 'serial_loopback', 'agent']),
        DeclareLaunchArgument('estop_on_link_loss', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('standalone', default_value='false', choices=['true', 'false']),
        Node(package='shopping_zone_manager', executable='cmd_vel_mux',
             condition=IfCondition(LaunchConfiguration('standalone')),
             parameters=[params, {'initial_mode': 'idle'}], output='screen'),
        Node(package='shopping_zone_manager', executable='cmd_vel_watchdog',
             condition=IfCondition(LaunchConfiguration('standalone')),
             parameters=[params], output='screen'),
        Node(package='shopping_mcu_bridge', executable='sim_mcu_firmware',
             condition=IfCondition(PythonExpression(["'", transport, "' == 'sim_ros'"])),
             parameters=[params], output='screen'),
        Node(package='shopping_mcu_bridge', executable='serial_bridge',
             condition=IfCondition(PythonExpression(["'", transport, "' == 'serial_loopback'"])),
             parameters=[params, {'use_hardware': False, 'port': ''}], output='screen'),
        Node(package='shopping_mcu_bridge', executable='mcu_link_monitor', parameters=[params,
             {'allow_simulation_faults': ParameterValue(PythonExpression([
                 "'", transport, "' == 'agent' and '", LaunchConfiguration('standalone'), "' == 'true'"]), value_type=bool)},
             {'estop_on_link_loss': ParameterValue(LaunchConfiguration('estop_on_link_loss'), value_type=bool)}],
             output='screen')])
