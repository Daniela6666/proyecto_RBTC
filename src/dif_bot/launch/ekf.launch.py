import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def launch_setup(context, *args, **kwargs):
    use_sim_time = _is_true(context, 'use_sim_time')
    use_imu = _is_true(context, 'use_imu')
    use_static_map = _is_true(context, 'launch_static_map_odom')
    odom_topic = LaunchConfiguration('odom_topic').perform(context)

    ekf_yaml = os.path.join(get_package_share_directory('dif_bot'), 'config', 'ekf.yaml')

    # Sobreescrituras sobre ekf.yaml (el diccionario va despues del yaml y gana).
    overrides = {
        'use_sim_time': use_sim_time,
        'odom0': odom_topic,
    }
    if not use_imu:
        # Prueba sin IMU: ningun campo de la IMU entra en el filtro.
        overrides['imu0_config'] = [False] * 15

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml, overrides],
        # La salida de robot_localization se publica en odometry/filtered; la llevamos a /odom_filtered
        remappings=[('odometry/filtered', '/odom_filtered')],
    )

    actions = [ekf_node]

    # map -> odom PROVISIONAL: identidad, estatica.
    # Sirve para que el arbol TF llegue de map a base_link (map -> odom -> base_link).
    # Es provisional: cuando SLAM o AMCL publiquen map -> odom, desactivar esto con
    # launch_static_map_odom:=false para no tener dos publicadores de map -> odom.
    if use_static_map:
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Usar el reloj de simulacion (/clock)',
        ),
        DeclareLaunchArgument(
            'odom_topic', default_value='/odom',
            description='Odometria de entrada. Con ros2_control: /diff_drive_controller/odom',
        ),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='false: el EKF ignora la IMU (imu0_config todo False)',
        ),
        DeclareLaunchArgument(
            'launch_static_map_odom', default_value='true',
            description='true: publica map->odom identidad (provisional). false si SLAM/AMCL ya publican map->odom',
        ),
        OpaqueFunction(function=launch_setup),
    ])
