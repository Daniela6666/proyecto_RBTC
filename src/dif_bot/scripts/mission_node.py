#!/usr/bin/env python3
"""Nodo de misión (Santiago Chacón, proyecto final).

Lee una lista de waypoints desde un YAML y, al llamar al servicio
/start_mission (std_srvs/Trigger), los recorre en orden con
nav2_simple_commander. El estado se publica en /mission_status.

Parámetros:
  waypoints_file (str): ruta al YAML de waypoints.
  localizer (str): nodo de localización por el que esperar antes de
      navegar. 'amcl' con mapa guardado; 'none' con SLAM (slam_toolbox
      en Humble no es un nodo lifecycle).
  waypoint_timeout (float): segundos máximos por waypoint (tiempo de pared).
"""
import math
import threading

import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('localizer', 'none')
        self.declare_parameter('waypoint_timeout', 120.0)

        self.status_pub = self.create_publisher(String, 'mission_status', 10)
        self.srv = self.create_service(Trigger, 'start_mission', self.on_start)
        self.navigator = BasicNavigator(node_name='mission_navigator')
        self.mission_thread = None
        self.publish_status('IDLE')
        self.get_logger().info('Listo. Llamar a /start_mission para iniciar.')

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(f'[estado] {text}')

    def load_waypoints(self):
        path = self.get_parameter('waypoints_file').value
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        frame = data.get('frame_id', 'map')
        waypoints = []
        for wp in data['waypoints']:
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.pose.position.x = float(wp['x'])
            pose.pose.position.y = float(wp['y'])
            yaw = math.radians(float(wp.get('yaw_deg', 0.0)))
            q = yaw_to_quaternion(yaw)
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = q
            waypoints.append((wp.get('name', f'wp{len(waypoints)}'), pose))
        return waypoints

    def on_start(self, request, response):
        if self.mission_thread is not None and self.mission_thread.is_alive():
            response.success = False
            response.message = 'Ya hay una misión en curso'
            return response
        try:
            waypoints = self.load_waypoints()
        except Exception as e:  # noqa: BLE001
            response.success = False
            response.message = f'No se pudieron leer los waypoints: {e}'
            return response
        self.mission_thread = threading.Thread(
            target=self.run_mission, args=(waypoints,), daemon=True)
        self.mission_thread.start()
        response.success = True
        response.message = f'Misión iniciada con {len(waypoints)} waypoints'
        return response

    def run_mission(self, waypoints):
        nav = self.navigator
        localizer = self.get_parameter('localizer').value
        timeout = Duration(seconds=self.get_parameter('waypoint_timeout').value)

        self.publish_status('WAITING_NAV2')
        if localizer == 'none':
            nav._waitForNodeToActivate('bt_navigator')
        else:
            nav.waitUntilNav2Active(localizer=localizer)

        reached = 0
        for i, (name, pose) in enumerate(waypoints, start=1):
            # stamp = 0: Nav2 usa la TF más reciente (el navegador no usa sim time)
            self.publish_status(
                f'GOING {i}/{len(waypoints)} {name} '
                f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
            start = nav.get_clock().now()
            if not nav.goToPose(pose):
                self.publish_status(f'REJECTED {name}')
                continue
            while not nav.isTaskComplete():
                if nav.get_clock().now() - start > timeout:
                    nav.cancelTask()
                    break
            result = nav.getResult()
            elapsed = (nav.get_clock().now() - start).nanoseconds / 1e9
            if result == TaskResult.SUCCEEDED:
                reached += 1
                self.publish_status(f'REACHED {name} en {elapsed:.1f} s')
            elif result == TaskResult.CANCELED:
                self.publish_status(f'TIMEOUT {name} tras {elapsed:.1f} s')
            else:
                self.publish_status(f'FAILED {name} tras {elapsed:.1f} s')

        self.publish_status(f'FINISHED {reached}/{len(waypoints)} waypoints alcanzados')


def main():
    rclpy.init()
    node = MissionNode()
    # Executor propio: BasicNavigator usa el executor global en su hilo
    # (spin_until_future_complete); compartirlo da 'generator already executing'.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.navigator.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
