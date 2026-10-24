"""Interface harness; no driver or motion producer is selected by default."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('shopping_robot_bringup')
    gazebo = get_package_share_directory('shopping_robot_gazebo')
    uwb = get_package_share_directory('shopping_uwb_follow')
    time = LaunchConfiguration('use_sim_time')
    kinds = ('lidar', 'imu', 'uwb', 'base')
    actions = [DeclareLaunchArgument('use_sim_time', default_value='true'),
               DeclareLaunchArgument('headless', default_value='true')]
    for kind in kinds:
        actions.extend([
            DeclareLaunchArgument('use_real_'+kind, default_value='false', choices=['true', 'false']),
            DeclareLaunchArgument(kind+'_driver_package', default_value=''),
            DeclareLaunchArgument(kind+'_driver_launch', default_value=''),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                share+'/launch/'+kind+'_driver_placeholder.launch.py'),
                condition=IfCondition(LaunchConfiguration('use_real_'+kind)),
                launch_arguments={'driver_package': LaunchConfiguration(kind+'_driver_package'),
                                  'driver_launch': LaunchConfiguration(kind+'_driver_launch'),
                                  'use_sim_time': time}.items())])
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo+'/launch/sim.launch.py'),
        launch_arguments={'use_sim_time': time, 'headless': LaunchConfiguration('headless'),
                          'odom_source': 'ekf', 'person': 'true', 'fault_injection': 'false',
                          **{'use_real_'+kind: LaunchConfiguration('use_real_'+kind)
                             for kind in ('lidar', 'imu', 'base')}}.items()))
    # Existing planar UWB messages are kept separate from the future stamped interface.
    for executable in ('uwb_sim_node', 'uwb_estimator_node'):
        actions.append(Node(package='shopping_uwb_follow', executable=executable,
            condition=UnlessCondition(LaunchConfiguration('use_real_uwb')),
            parameters=[uwb+'/config/uwb_follow.yaml', {'use_sim_time': time,
                        'person_path': gazebo+'/config/person_path.yaml'}], output='screen'))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(share+'/launch/mode_mux.launch.py'),
        launch_arguments={'use_mux': 'true', 'initial_mode': 'idle', 'use_sim_time': time}.items()))
    return LaunchDescription(actions)
