from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    p = Path(get_package_share_directory('dif_bot'))
    gz = Path(get_package_share_directory('ros_gz_sim'))
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz / 'launch/gz_sim.launch.py')), launch_arguments={'gz_args': '-r ' + str(p / 'worlds/rbtc_room.sdf')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'use_sim_time': True, 'robot_description': (p / 'urdf/robot.urdf').read_text()}]),
        Node(package='ros_gz_sim', executable='create', arguments=['-name', 'dif_bot', '-topic', 'robot_description', '-z', '0.16'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', parameters=[{'use_sim_time': True}], arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'], output='screen')])
