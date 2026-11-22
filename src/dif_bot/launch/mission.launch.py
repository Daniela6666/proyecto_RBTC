from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    p=Path(get_package_share_directory('dif_bot'))
    return LaunchDescription([Node(package='dif_bot',executable='mission_node.py',output='screen',parameters=[{'use_sim_time':True,'waypoints_file':str(p/'config/waypoints.yaml'),'goal_timeout':300.0}])])
