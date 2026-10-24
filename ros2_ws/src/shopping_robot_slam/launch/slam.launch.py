from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def start(context):
    mode = LaunchConfiguration('mode').perform(context)
    graph = LaunchConfiguration('map').perform(context)
    if mode not in ('mapping', 'localization'):
        raise ValueError('mode must be mapping or localization')
    if mode == 'localization' and not graph:
        raise ValueError('localization requires map:=/path/to/serialized_posegraph_prefix')
    filename = 'mapper_params_online_async.yaml' if mode == 'mapping' else 'localization_params.yaml'
    overrides = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    if mode == 'localization':
        overrides['map_file_name'] = graph
        if 'spawn_x' in context.launch_configurations:
            overrides['map_start_at_dock'] = False
            overrides['map_start_pose'] = [float(context.launch_configurations['spawn_'+key])
                                           for key in ('x', 'y', 'yaw')]
    return [Node(package='slam_toolbox', executable='async_slam_toolbox_node' if mode == 'mapping' else 'localization_slam_toolbox_node', name='slam_toolbox',
                 parameters=[get_package_share_directory('shopping_robot_slam')+'/config/'+filename, overrides], output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode', default_value='mapping'),
                              DeclareLaunchArgument('map', default_value=''),
                              DeclareLaunchArgument('use_sim_time', default_value='true'), OpaqueFunction(function=start)])
