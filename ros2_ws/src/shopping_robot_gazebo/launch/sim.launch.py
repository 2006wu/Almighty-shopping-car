import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def validate_sources(context):
    real_sources = any(LaunchConfiguration('use_real_'+kind).perform(context).lower() == 'true'
                       for kind in ('lidar', 'imu', 'base'))
    if real_sources and LaunchConfiguration('odom_source').perform(context) == 'ground_truth':
        raise RuntimeError('Real source options require odom_source:=ekf; simulator truth cannot own real robot TF.')
    if real_sources and LaunchConfiguration('fault_injection').perform(context).lower() == 'true':
        raise RuntimeError('Real source options cannot be combined with simulated fault publishers.')
    return []


def generate_launch_description():
    share = get_package_share_directory('shopping_robot_gazebo')
    description = get_package_share_directory('shopping_robot_description')
    time = LaunchConfiguration('use_sim_time')
    fault = LaunchConfiguration('fault_injection')
    def output(topic):
        return PythonExpression(["'/sim/raw", topic, "' if '", fault, "'.lower() == 'true' else '", topic, "'"])
    return LaunchDescription([
        DeclareLaunchArgument('use_real_lidar', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_real_imu', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_real_base', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('fault_injection', default_value='false'),
        DeclareLaunchArgument('fault_schedule', default_value='[]'),
        DeclareLaunchArgument('world', default_value=share+'/worlds/supermarket.sdf'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('person', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('odom_source', default_value='ekf', choices=['ekf', 'ground_truth']),
        OpaqueFunction(function=validate_sources),
        Node(package='shopping_robot_gazebo', executable='fault_injector',
             condition=IfCondition(fault), parameters=[{'use_sim_time': time,
             'schedule': ParameterValue(LaunchConfiguration('fault_schedule'), value_type=str)}], output='screen'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', share+'/worlds:'+description+':'+os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')),
        ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', LaunchConfiguration('world')], condition=IfCondition(LaunchConfiguration('headless')), output='screen'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')], condition=UnlessCondition(LaunchConfiguration('headless')), output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description+'/launch/display.launch.py'), launch_arguments={'use_sim_time': time, 'odom_source': LaunchConfiguration('odom_source')}.items()),
        Node(package='ros_gz_sim', executable='create', arguments=['-world', 'supermarket', '-name', 'shopping_robot', '-topic', '/robot_description', '-x', '0', '-y', '0', '-z', '0.02', '-Y', '0'], parameters=[{'use_sim_time': time}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='sensor_bridge',
             parameters=[{'config_file': share+'/config/bridge_common.yaml', 'use_sim_time': time}], output='screen'),
        *[Node(package='ros_gz_bridge', executable='parameter_bridge', name=kind+'_bridge',
             condition=UnlessCondition(LaunchConfiguration('use_real_'+kind)),
             remappings=[('/scan', output('/scan'))] if kind == 'lidar' else [],
             parameters=[{'config_file': share+'/config/bridge_'+kind+'.yaml', 'use_sim_time': time}], output='screen')
          for kind in ('lidar', 'imu', 'base')],
        # Only the legacy mode bridges simulator TF; wheel_tf and truth TF stay isolated.
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='truth_odom_bridge', remappings=[('/odom', output('/odom'))],
             condition=LaunchConfigurationEquals('odom_source', 'ground_truth'),
             parameters=[{'config_file': share+'/config/bridge_ground_truth.yaml', 'use_sim_time': time}], output='screen'),
        Node(package='shopping_uwb_follow', executable='wheel_odom_noise',
             condition=LaunchConfigurationEquals('odom_source', 'ekf'),
             parameters=[{'use_sim_time': time,
                          'simulate_wheel': ParameterValue(PythonExpression(["'", LaunchConfiguration('use_real_base'), "'.lower() != 'true'"]), value_type=bool),
                          'simulate_imu': ParameterValue(PythonExpression(["'", LaunchConfiguration('use_real_imu'), "'.lower() != 'true'"]), value_type=bool)}], output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             condition=LaunchConfigurationEquals('odom_source', 'ekf'),
             parameters=[share+'/config/ekf.yaml', {'use_sim_time': time}],
             remappings=[('odometry/filtered', output('/odom'))], output='screen'),
        Node(package='shopping_uwb_follow', executable='odom_drift_monitor',
             parameters=[{'use_sim_time': time}], output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher', condition=LaunchConfigurationEquals('odom_source', 'ground_truth'), arguments=['0', '0', '0', '0', '0', '0', 'odom', 'sim_world'], parameters=[{'use_sim_time': time}]),
        Node(package='shopping_uwb_follow', executable='person_path_node', condition=IfCondition(LaunchConfiguration('person')), parameters=[{'use_sim_time': time, 'person_path': share+'/config/person_path.yaml'}], output='screen')])
