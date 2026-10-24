import os
from launch import LaunchDescription
from shopping_robot_sim.scenario_launch import prepare_scenario, pedestrian_actions
from launch.actions import OpaqueFunction, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('shopping_robot_gazebo')
    description = get_package_share_directory('shopping_robot_description')
    time = LaunchConfiguration('use_sim_time')
    fault = LaunchConfiguration('fault_injection')
    def output(topic):
        return PythonExpression(["'/sim/raw", topic, "' if '", fault, "'.lower() == 'true' else '", topic, "'"])
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='default'),
        DeclareLaunchArgument('world', default_value=share+'/worlds/supermarket.sdf'),
        OpaqueFunction(function=prepare_scenario),
        DeclareLaunchArgument('fault_injection', default_value='false'),
        DeclareLaunchArgument('fault_schedule', default_value='[]'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('person', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('odom_source', default_value='ekf', choices=['ekf', 'ground_truth']),
        OpaqueFunction(function=pedestrian_actions),
        Node(package='shopping_robot_gazebo', executable='fault_injector',
             condition=IfCondition(fault), parameters=[{'use_sim_time': time,
             'schedule': ParameterValue(LaunchConfiguration('fault_schedule'), value_type=str)}], output='screen'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', share+'/worlds:'+description+':'+os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')),
        ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', LaunchConfiguration('scenario_world')], condition=IfCondition(LaunchConfiguration('headless')), output='screen'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', LaunchConfiguration('scenario_world')], condition=UnlessCondition(LaunchConfiguration('headless')), output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description+'/launch/display.launch.py'), launch_arguments={'use_sim_time': time, 'odom_source': LaunchConfiguration('odom_source')}.items()),
        Node(package='ros_gz_sim', executable='create', arguments=['-world', 'supermarket', '-name', 'shopping_robot', '-topic', '/robot_description', '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'), '-z', '0.02', '-Y', LaunchConfiguration('spawn_yaw')], parameters=[{'use_sim_time': time}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='sensor_bridge', remappings=[('/scan', output('/scan'))], parameters=[{'config_file': share+'/config/bridge.yaml', 'use_sim_time': time}], output='screen'),
        # Only the legacy mode bridges simulator TF; wheel_tf and truth TF stay isolated.
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='truth_odom_bridge', remappings=[('/odom', output('/odom'))],
             condition=LaunchConfigurationEquals('odom_source', 'ground_truth'),
             parameters=[{'config_file': share+'/config/bridge_ground_truth.yaml', 'use_sim_time': time}], output='screen'),
        Node(package='shopping_uwb_follow', executable='wheel_odom_noise',
             condition=LaunchConfigurationEquals('odom_source', 'ekf'),
             parameters=[{'use_sim_time': time}], output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             condition=LaunchConfigurationEquals('odom_source', 'ekf'),
             parameters=[share+'/config/ekf.yaml', {'use_sim_time': time, 'initial_state': ParameterValue(LaunchConfiguration('scenario_initial_state'))}],
             remappings=[('odometry/filtered', output('/odom'))], output='screen'),
        Node(package='shopping_uwb_follow', executable='odom_drift_monitor',
             parameters=[{'use_sim_time': time}], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', condition=LaunchConfigurationEquals('odom_source', 'ground_truth'), arguments=['0', '0', '0', '0', '0', '0', 'odom', 'sim_world'], parameters=[{'use_sim_time': time}]),
        Node(package='shopping_uwb_follow', executable='person_path_node', condition=IfCondition(LaunchConfiguration('person')), parameters=[{'use_sim_time': time, 'person_path': LaunchConfiguration('scenario_person_path')}], output='screen')])
