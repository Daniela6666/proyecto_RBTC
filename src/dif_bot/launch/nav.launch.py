"""Navegación completa (Persona C: Santiago Chacón): simulación + controladores + SLAM/AMCL + Nav2 + misión.

Uso:
  ros2 launch dif_bot nav.launch.py slam:=true            # mapea mientras navega
  ros2 launch dif_bot nav.launch.py slam:=false map:=...  # mapa guardado + AMCL
  ros2 service call /start_mission std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('dif_bot')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    waypoints_file = LaunchConfiguration('waypoints_file')

    # Con ros2_control la odometría sale por /diff_drive_controller/odom;
    # con el DiffDrive nativo de Gazebo, por /odom (ver README raíz).
    odom_topic = PythonExpression([
        "'/diff_drive_controller/odom' if '", use_ros2_control, "' == 'true' else '/odom'"])

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'odom_topic': odom_topic,
            # Con SLAM el mapa crece poco a poco: costmap global en ventana móvil 12x12 m.
            'global_costmap.global_costmap.ros__parameters.rolling_window': slam,
        },
        convert_types=True,
    )

    declare = [
        DeclareLaunchArgument('slam', default_value='true',
                              description='true: slam_toolbox; false: map_server + AMCL'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'map.yaml')),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('waypoints_file',
                              default_value=os.path.join(pkg, 'config', 'waypoints.yaml')),
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('world', default_value='empty_world.sdf'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'sim.launch.py')),
        launch_arguments={
            'use_ros2_control': use_ros2_control,
            'headless': headless,
            'world': world,
        }.items(),
    )

    # sim.launch.py no carga los controladores con ros2_control: se cargan aquí.
    controllers = TimerAction(
        period=5.0,
        condition=IfCondition(use_ros2_control),
        actions=[
            Node(package='controller_manager', executable='spawner', output='screen',
                 arguments=['joint_state_broadcaster', '--controller-manager',
                            '/controller_manager', '--controller-manager-timeout', '60']),
            Node(package='controller_manager', executable='spawner', output='screen',
                 arguments=['diff_drive_controller', '--controller-manager',
                            '/controller_manager', '--controller-manager-timeout', '60']),
            # Nav2 publica en /cmd_vel; diff_drive_controller escucha otro topic.
            # Sin use_sim_time: no necesita /clock (1 kHz) y así no consume CPU.
            Node(package='dif_bot', executable='cmd_vel_relay.py', output='screen'),
        ],
    )

    slam_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        output='screen', condition=IfCondition(slam),
        parameters=[configured_params, {'use_sim_time': True}],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'localization_launch.py')),
        condition=UnlessCondition(slam),
        launch_arguments={'map': map_yaml, 'use_sim_time': 'true',
                          'params_file': configured_params, 'autostart': 'true'}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'params_file': configured_params,
                          'autostart': 'true'}.items(),
    )

    mission = Node(
        package='dif_bot', executable='mission_node.py', output='screen',
        # Sin use_sim_time: los tiempos de la misión se miden en tiempo de pared.
        parameters=[{
            'waypoints_file': waypoints_file,
            'localizer': PythonExpression(["'none' if '", slam, "' == 'true' else 'amcl'"]),
        }],
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', output='screen', condition=IfCondition(rviz),
        arguments=['-d', os.path.join(nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    # Nav2 arranca cuando el robot y los controladores ya publican TF y odometría.
    nav_stack = TimerAction(period=12.0, actions=[
        GroupAction([slam_node, localization, navigation, mission, rviz_node]),
    ])

    return LaunchDescription(declare + [sim, controllers, nav_stack])
