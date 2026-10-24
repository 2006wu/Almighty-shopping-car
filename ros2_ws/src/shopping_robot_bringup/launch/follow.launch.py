from launch import LaunchDescription
from launch.conditions import IfCondition
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
    fault = LaunchConfiguration('fault_injection')
    def raw(topic):
        return PythonExpression(["'/sim/raw", topic, "' if '", fault, "'.lower() == 'true' else '", topic, "'"])
    person_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                         name='person_pose_bridge', condition=IfCondition(LaunchConfiguration('start_sim')), parameters=[{'use_sim_time': time,
                         'config_file': get_package_share_directory('shopping_robot_gazebo')+'/config/bridge_person.yaml'}],
                         output='screen')
    nodes = [Node(package='shopping_uwb_follow', executable=name,
                  condition=IfCondition(LaunchConfiguration('start_uwb_sim' if name == 'uwb_sim_node' else 'start_app')),
                  parameters=[config, {'use_sim_time': time, 'person_path': get_package_share_directory('shopping_robot_gazebo')+'/config/person_path.yaml'}, *([{'enabled': enabled, 'monitor_odom': ParameterValue(fault, value_type=bool), 'observed_cmd_vel_topic': ParameterValue(observed_topic, value_type=str), 'route_around': ParameterValue(LaunchConfiguration('route_around'), value_type=bool)}] if name == 'follow_controller_node' else [])],
                  remappings=([('/cmd_vel', follow_topic)] if name == 'follow_controller_node' else [('/uwb/ranges', raw('/uwb/ranges'))] if name == 'uwb_sim_node' else [('/uwb/target', raw('/uwb/target'))] if name == 'uwb_estimator_node' else []), output='screen')
             for name in ['uwb_sim_node', 'uwb_estimator_node', 'uwb_eval_node', 'follow_controller_node']]
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='true'),
        DeclareLaunchArgument('start_navigation', default_value='true'),
        DeclareLaunchArgument('start_app', default_value='true'),
        DeclareLaunchArgument('start_uwb_sim', default_value='true'),
        DeclareLaunchArgument('start_sim_localization', default_value='true'),
        DeclareLaunchArgument('start_nav_localization', default_value='true'),
        DeclareLaunchArgument('mcu_transport', default_value='sim_ros', choices=['sim_ros', 'agent']),
        DeclareLaunchArgument('mcu_comm', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('fault_injection', default_value='false'),
        DeclareLaunchArgument('fault_schedule', default_value='[]'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('shopping_mcu_bridge')+'/launch/comm.launch.py'),
            condition=IfCondition(LaunchConfiguration('mcu_comm')),
            launch_arguments={'use_sim_time': time, 'transport': LaunchConfiguration('mcu_transport')}.items()),
        DeclareLaunchArgument('use_mux', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('initial_mode', default_value='follow'),
        DeclareLaunchArgument('odom_source', default_value='ekf', choices=['ekf', 'ground_truth']),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('nav2_stack', default_value='apt', choices=['auto', 'fork', 'apt']),
        DeclareLaunchArgument('slam_mode', default_value='mapping'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('route_around', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('shopping_robot_bringup')+'/launch/mode_mux.launch.py'), condition=IfCondition(LaunchConfiguration('start_app')), launch_arguments={'use_mux': use_mux, 'initial_mode': LaunchConfiguration('initial_mode'), 'manage_follow': 'true', 'use_sim_time': time}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('shopping_robot_navigation')+'/launch/navigation.launch.py'), condition=IfCondition(LaunchConfiguration('start_navigation')), launch_arguments={'use_mux': use_mux, 'nav2_stack': LaunchConfiguration('nav2_stack'), 'slam_mode': LaunchConfiguration('slam_mode'), 'map': LaunchConfiguration('map'), 'use_sim_time': time, 'start_localization': LaunchConfiguration('start_nav_localization')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('shopping_robot_gazebo')+'/launch/sim.launch.py'), condition=IfCondition(LaunchConfiguration('start_sim')), launch_arguments={'start_localization': LaunchConfiguration('start_sim_localization'), 'odom_source': LaunchConfiguration('odom_source'), 'use_sim_time': time, 'headless': LaunchConfiguration('headless'), 'person': 'true', 'fault_injection': fault, 'fault_schedule': LaunchConfiguration('fault_schedule')}.items()), person_bridge, *nodes])
