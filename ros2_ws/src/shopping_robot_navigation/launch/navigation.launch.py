from launch import LaunchDescription
import os

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction, LogInfo, ExecuteProcess
from ament_index_python.packages import get_packages_with_prefixes
from shopping_robot_navigation.stack_selection import select_stack, apt_environment
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
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
    stack = select_stack(LaunchConfiguration('nav2_stack').perform(context), get_packages_with_prefixes())
    share = get_package_share_directory('shopping_robot_navigation')
    if stack == 'apt' and '/nav2_ws/' in os.environ.get('AMENT_PREFIX_PATH', ''):
        # Re-exec so both Python ament lookup and spawned binaries use apt.
        args = [f'{name}:={LaunchConfiguration(name, default="true").perform(context)}'
                for name in ('slam_mode', 'map', 'use_sim_time', 'use_mux', 'start_localization', 'start_navigation')]
        # The child process cannot inherit launch configurations from bringup.
        for name in ('spawn_x', 'spawn_y', 'spawn_yaw'):
            value = LaunchConfiguration(name, default='').perform(context)
            if value != '':
                args.append(f'{name}:={value}')
        return [LogInfo(msg='Nav2 stack: apt (isolated from fork underlay)'), ExecuteProcess(
            cmd=['bash', '-c', 'source /opt/ros/humble/setup.bash; '
                 'if [[ -f /ws/install/local_setup.bash ]]; then source /ws/install/local_setup.bash; fi; '
                 'exec ros2 launch shopping_robot_navigation navigation.launch.py nav2_stack:=apt "$@"',
                 'nav2-apt', *args], env=apt_environment(os.environ), output='screen')]
    time = LaunchConfiguration('use_sim_time')
    params = share + ('/config/nav2_params_fork.yaml' if stack == 'fork' else '/config/nav2_params.yaml')
    if stack == 'fork':
        from nav2_common.launch import RewrittenYaml
        params = RewrittenYaml(source_file=params, root_key='', param_rewrites={
            'default_nav_to_pose_bt_xml': share + '/behavior_trees/navigate_to_pose_shopping.xml',
            'default_nav_through_poses_bt_xml': share + '/behavior_trees/navigate_through_poses_shopping.xml'},
            convert_types=True)
    nav = get_package_share_directory('nav2_bringup')+'/launch/'
    common = {'use_sim_time': time, 'params_file': params, 'autostart': 'true', 'use_composition': 'False'}
    actions = [LogInfo(msg=f'Nav2 stack: {stack}')]
    if mode == 'map_server':
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(nav+'localization_launch.py'), condition=IfCondition(LaunchConfiguration('start_localization')), launch_arguments={**common, 'map': map_file}.items()))
    else:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('shopping_robot_slam')+'/launch/slam.launch.py'), condition=IfCondition(LaunchConfiguration('start_localization')), launch_arguments={'use_sim_time': time, 'mode': mode, 'map': map_file}.items()))
    navigation = IncludeLaunchDescription(PythonLaunchDescriptionSource(nav+'navigation_launch.py'), launch_arguments=common.items())
    # Override the smoother output and the behavior_server recoveries (spin/backup/drive_on_heading/
    # assisted_teleop publish cmd_vel directly); preserve Nav2's internal cmd_vel_nav. The node-scoped
    # rules match behavior_server and the fork BT client, preserving the controller-to-smoother link.
    remaps = [SetRemap(src='cmd_vel_smoothed', dst='/cmd_vel/nav'),
              SetRemap(src='behavior_server:cmd_vel', dst='/cmd_vel/nav'),
              SetRemap(src='bt_navigator_navigate_to_pose_rclcpp_node:/cmd_vel', dst='/cmd_vel/nav')] if LaunchConfiguration('use_mux').perform(context).lower() == 'true' else []
    actions.append(GroupAction(condition=IfCondition(LaunchConfiguration('start_navigation')), actions=[*remaps, navigation]))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('start_localization', default_value='true'),
                              DeclareLaunchArgument('start_navigation', default_value='true'),
                              DeclareLaunchArgument('use_mux', default_value='true', choices=['true', 'false']),
                              DeclareLaunchArgument('use_sim_time', default_value='true'),
                              DeclareLaunchArgument('nav2_stack', default_value='apt', choices=['auto', 'fork', 'apt']),
                              DeclareLaunchArgument('map', default_value=''),
                              DeclareLaunchArgument('spawn_x', default_value=''),
                              DeclareLaunchArgument('spawn_y', default_value=''),
                              DeclareLaunchArgument('spawn_yaw', default_value=''),
                              DeclareLaunchArgument('slam_mode', default_value='mapping'), OpaqueFunction(function=start)])
