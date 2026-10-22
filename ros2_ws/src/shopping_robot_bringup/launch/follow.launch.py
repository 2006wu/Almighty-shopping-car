from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    time = LaunchConfiguration('use_sim_time')
    config = get_package_share_directory('shopping_uwb_follow')+'/config/uwb_follow.yaml'
    nodes = [Node(package='shopping_uwb_follow', executable=name,
                  parameters=[config, {'use_sim_time': time, 'person_path': get_package_share_directory('shopping_robot_gazebo')+'/config/person_path.yaml'}, *([{'enabled': True}] if name == 'follow_controller_node' else [])], output='screen')
             for name in ['uwb_sim_node', 'uwb_estimator_node', 'follow_controller_node']]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('shopping_robot_gazebo')+'/launch/sim.launch.py'), launch_arguments={'use_sim_time': time, 'headless': LaunchConfiguration('headless')}.items()), *nodes])
