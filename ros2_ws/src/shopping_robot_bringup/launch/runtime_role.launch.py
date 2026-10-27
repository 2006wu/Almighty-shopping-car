"""Compose existing launches into mutually exclusive runtime roles."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def include(package, filename, args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(
        get_package_share_directory(package)+'/launch/'+filename), launch_arguments=args.items())


def start(context):
    role = LaunchConfiguration('role').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    common = {name: LaunchConfiguration(name) for name in (
        'odom_source', 'headless', 'nav2_stack', 'slam_mode', 'map',
        'fault_injection', 'fault_schedule', 'route_around', 'person', 'mcu_transport')}
    common.update(use_sim_time='true', use_mux='true',
                  rpp_desired_speed='0.8')
    if role == 'localization':
        return [include('shopping_robot_gazebo', 'sim.launch.py', {
            **common, 'start_sim': 'false', 'start_localization': 'true'}),
            include('shopping_robot_navigation', 'navigation.launch.py', {
                **common, 'start_localization': 'true', 'start_navigation': 'false'})]
    if role == 'navigation':
        return [include('shopping_robot_navigation', 'navigation.launch.py', {
            **common, 'start_localization': 'false', 'start_navigation': 'true'})]
    return [include('shopping_robot_bringup', mode+'.launch.py', {
        **common, 'start_sim': str(role == 'sim').lower(),
        'start_uwb_sim': str(role == 'sim').lower(),
        'start_navigation': 'false', 'start_app': str(role == 'app').lower(),
        'start_sim_localization': 'false', 'start_nav_localization': 'false',
        'initial_mode': mode, 'mcu_comm': str(role == 'sim').lower()})]


def generate_launch_description():
    defaults = {'odom_source': 'ekf', 'headless': 'true', 'nav2_stack': 'auto',
                'slam_mode': 'mapping', 'map': '', 'fault_injection': 'false',
                'mcu_transport': 'sim_ros', 'fault_schedule': '[]', 'route_around': 'true', 'person': 'false'}
    return LaunchDescription([
        DeclareLaunchArgument('role', choices=['sim', 'localization', 'navigation', 'app']),
        DeclareLaunchArgument('mode', default_value='guide', choices=['guide', 'follow']),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=start)])
