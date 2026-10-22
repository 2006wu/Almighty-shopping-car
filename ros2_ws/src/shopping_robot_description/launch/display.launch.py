from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model = PathJoinSubstitution([FindPackageShare('shopping_robot_description'), 'urdf', 'shopping_robot.urdf.xacro'])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'robot_description': ParameterValue(Command(['xacro ', model]), value_type=str)}])])
