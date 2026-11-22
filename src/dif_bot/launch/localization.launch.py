from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    p = Path(get_package_share_directory('dif_bot'))
    cfg = str(p / 'config/amcl.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Ruta absoluta al mapa YAML guardado'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', parameters=[cfg, {'yaml_filename': LaunchConfiguration('map')}], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[cfg], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server', 'amcl']}], output='screen')])
