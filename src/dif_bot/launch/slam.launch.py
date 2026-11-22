from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    p = Path(get_package_share_directory('dif_bot'))
    s = Path(get_package_share_directory('slam_toolbox'))
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(s / 'launch/online_async_launch.py')), launch_arguments={'use_sim_time': 'true', 'slam_params_file': str(p / 'config/slam.yaml')}.items())])
