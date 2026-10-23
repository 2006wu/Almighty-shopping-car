from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    time = LaunchConfiguration('use_sim_time')
    config = get_package_share_directory('shopping_uwb_follow')+'/config/uwb_follow.yaml'
    use_mux = LaunchConfiguration('use_mux')
    follow_topic = PythonExpression(["'/cmd_vel/follow' if '", use_mux, "'.lower() == 'true' else '/cmd_vel'"])
    observed_topic = PythonExpression(["'/cmd_vel/nav' if '", use_mux, "'.lower() == 'true' else '/cmd_vel'"])
    enabled = ParameterValue(PythonExpression(["'", use_mux, "'.lower() != 'true'"]), value_type=bool)
    nodes = [Node(package='shopping_uwb_follow', executable=name,
                  parameters=[config, {'use_sim_time': time, 'person_path': get_package_share_directory('shopping_robot_gazebo')+'/config/person_path.yaml'}, *([{'enabled': enabled, 'observed_cmd_vel_topic': ParameterValue(observed_topic, value_type=str), 'route_around': ParameterValue(LaunchConfiguration('route_around'), value_type=bool)}] if name == 'follow_controller_node' else [])], remappings=[('/cmd_vel', follow_topic)] if name == 'follow_controller_node' else [], output='screen')
             for name in ['uwb_sim_node', 'uwb_estimator_node', 'uwb_eval_node', 'follow_controller_node']]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_mux', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('initial_mode', default_value='follow'),
        DeclareLaunchArgument('odom_source', default_value='ekf', choices=['ekf', 'ground_truth']),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('slam_mode', default_value='mapping'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('route_around', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('shopping_robot_bringup')+'/launch/mode_mux.launch.py'), launch_arguments={'use_mux': use_mux, 'initial_mode': LaunchConfiguration('initial_mode'), 'manage_follow': 'true', 'use_sim_time': time}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('shopping_robot_navigation')+'/launch/navigation.launch.py'), launch_arguments={'use_mux': use_mux, 'slam_mode': LaunchConfiguration('slam_mode'), 'map': LaunchConfiguration('map'), 'use_sim_time': time}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('shopping_robot_gazebo')+'/launch/sim.launch.py'), launch_arguments={'odom_source': LaunchConfiguration('odom_source'), 'use_sim_time': time, 'headless': LaunchConfiguration('headless'), 'person': 'true'}.items()), *nodes])
