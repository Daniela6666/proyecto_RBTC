import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_ros2_control = LaunchConfiguration('use_ros2_control').perform(context).lower() == 'true'
    use_ekf = LaunchConfiguration('use_ekf').perform(context).lower() == 'true'
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    world = LaunchConfiguration('world').perform(context)

    pkg_share = get_package_share_directory('dif_bot')
    world_path = os.path.join(pkg_share, 'worlds', world)

    # headless: solo servidor, render por EGL (sin ventana); sin esto Ogre2 falla en WSL sin GPU
    gz_args = f'-r {"-s --headless-rendering " if headless else ""}{world_path}'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'dif_bot', '-z', '0.1'],
        output='screen',
    )

    # Mismo xacro para ambos modos; cambian los argumentos use_ros2_control y use_ekf
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([FindPackageShare('dif_bot'), 'urdf', 'robot.urdf.xacro']),
            ' use_ros2_control:=', LaunchConfiguration('use_ros2_control'),
            ' use_ekf:=', LaunchConfiguration('use_ekf'),
        ]),
        value_type=str,
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
    )

    # Puente comun a ambos modos
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
    ]
    # Solo sin ros2_control: DiffDrive nativo de Gazebo
    if not use_ros2_control:
        bridge_args += [
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ]
        # /tf (TF odom->base_link del plugin) solo sin EKF; con EKF lo publica ekf_filter_node
        if not use_ekf:
            bridge_args.append('/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    actions = [gz_sim, spawn, rsp, bridge]

    # Con ros2_control: no se lanzan spawners. Los controladores se cargan a mano.
    # (ver README.md raiz)
    return actions


def generate_launch_description():
    # Permite que Gazebo encuentre los plugins de ign_ros2_control y demas libs de ROS
    ament_prefixes = os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
    lib_paths = [os.path.join(p, 'lib') for p in ament_prefixes if p]
    existing = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    plugin_path = os.pathsep.join([p for p in lib_paths + [existing] if p])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ros2_control', default_value='false',
            description='true: hardware ign_ros2_control; false: DiffDrive nativo de Gazebo',
        ),
        DeclareLaunchArgument(
            'use_ekf', default_value='false',
            description='true: el EKF (ekf.launch.py) publica odom->base_link; la odometria de rueda no publica ese TF',
        ),
        DeclareLaunchArgument(
            'world', default_value='empty_world.sdf',
            description='Archivo del mundo dentro de worlds/',
        ),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true: gz sim solo servidor (sin GUI)',
        ),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugin_path),
        OpaqueFunction(function=launch_setup),
    ])
