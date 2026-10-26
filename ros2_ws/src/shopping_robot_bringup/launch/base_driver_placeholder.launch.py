"""PLACEHOLDER: includes only an explicitly supplied future driver launch."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogError, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def start(context):
    package = LaunchConfiguration('driver_package').perform(context)
    filename = LaunchConfiguration('driver_launch').perform(context)
    if not package or not filename:
        return [LogError(msg='PLACEHOLDER: driver_package and driver_launch must both be explicitly supplied; no driver started.')]
    # Only a launch file inside the named package launch directory is accepted.
    if Path(filename).name != filename or not filename.endswith('.launch.py'):
        return [LogError(msg='PLACEHOLDER: driver_launch must be a plain *.launch.py filename.')]
    try:
        path = Path(get_package_share_directory(package))/'launch'/filename
    except LookupError:
        return [LogError(msg='PLACEHOLDER: explicit driver package is unavailable; no driver started.')]
    if not path.is_file():
        return [LogError(msg='PLACEHOLDER: explicit driver launch does not exist; no driver started.')]
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('driver_package', default_value=''),
        DeclareLaunchArgument('driver_launch', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=start)])
