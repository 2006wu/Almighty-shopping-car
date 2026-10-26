from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='true'),
                              DeclareLaunchArgument('config', default_value=get_package_share_directory('shopping_robot_bringup')+'/rviz/shopping_robot.rviz'),
                              Node(package='rviz2', executable='rviz2', arguments=['-d', LaunchConfiguration('config')], parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])])
