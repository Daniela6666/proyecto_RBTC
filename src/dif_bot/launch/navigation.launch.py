from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    p=Path(get_package_share_directory('dif_bot'))
    n=Path(get_package_share_directory('nav2_bringup'))
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(n/'launch/navigation_launch.py')),launch_arguments={'use_sim_time':'true','autostart':'true','use_composition':'False','params_file':str(p/'config/nav2_params.yaml')}.items())])
