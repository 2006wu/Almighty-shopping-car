"""Start only the navigation role; reuse the shared launch decomposition."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(
        get_package_share_directory('shopping_robot_bringup')+'/launch/runtime_role.launch.py'),
        launch_arguments={'role': 'navigation'}.items())])
