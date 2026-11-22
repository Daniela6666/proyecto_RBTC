"""Robot, optional EKF, map/AMCL, Nav2 and mission in one launch."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package = Path(get_package_share_directory('dif_bot'))
    def include(name, args=None, condition=None):
        kwargs = {'condition': condition} if condition is not None else {}
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch' / name)),
            launch_arguments=(args or {}).items(), **kwargs)
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        DeclareLaunchArgument('map', default_value=str(package / 'maps/rbtc_room.yaml')),
        include('sim.launch.py', {'headless': LaunchConfiguration('headless'),
                                 'use_ekf': LaunchConfiguration('use_ekf')}),
        include('ekf.launch.py', {'launch_static_map_odom': 'false'},
                IfCondition(LaunchConfiguration('use_ekf'))),
        include('localization.launch.py', {'map': LaunchConfiguration('map')}),
        include('navigation.launch.py'),
        include('mission.launch.py'),
    ])
