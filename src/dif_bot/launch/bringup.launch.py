from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    p=Path(get_package_share_directory('dif_bot'))
    def include(name,args=None):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(p/'launch'/name)),launch_arguments=(args or {}).items())
    return LaunchDescription([DeclareLaunchArgument('map',default_value=str(p/'maps/rbtc_room.yaml')),include('sim.launch.py'),include('localization.launch.py',{'map':LaunchConfiguration('map')}),include('navigation.launch.py'),include('mission.launch.py')])
