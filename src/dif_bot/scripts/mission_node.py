#!/usr/bin/env python3
"""Nonblocking mission services; BasicNavigator is owned by one worker thread."""
import json
import math
import threading
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def load_waypoints(filename):
    data=yaml.safe_load(Path(filename).read_text())
    if not isinstance(data,dict) or data.get('frame_id') != 'map':
        raise ValueError('waypoints must use frame_id: map')
    points=data.get('waypoints')
    if not isinstance(points,list) or not points:
        raise ValueError('at least one waypoint is required')
    for point in points:
        if not isinstance(point,dict):
            raise ValueError('each waypoint must be a mapping')
        for key in ('x','y','yaw'):
            if isinstance(point.get(key),bool) or not isinstance(point.get(key),(int,float)) or not math.isfinite(point[key]):
                raise ValueError('x, y, yaw must be finite numbers')
    return points


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('waypoints_file','')
        self.declare_parameter('goal_timeout',120.0)
        self.lock=threading.Lock()
        self.cancel_event=threading.Event()
        self.running=False
        self.worker=None
        self.status=self.create_publisher(String,'/mission/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger,'/mission/start',self.start)
        self.create_service(Trigger,'/mission/cancel',self.cancel)
        self.report('idle')

    def report(self,state,**details):
        message=json.dumps(dict(state=state,**details),ensure_ascii=False)
        self.status.publish(String(data=message))
        self.get_logger().info(message)

    def start(self,request,response):
        with self.lock:
            if self.running:
                response.success=False; response.message='Mission already running'; return response
            try:
                points=load_waypoints(self.get_parameter('waypoints_file').value)
                timeout=float(self.get_parameter('goal_timeout').value)
                if not math.isfinite(timeout) or timeout<=0: raise ValueError('goal_timeout must be positive')
            except Exception as exc:
                response.success=False; response.message=str(exc); return response
            self.running=True; self.cancel_event.clear()
            self.worker=threading.Thread(target=self.execute,args=(points,timeout),daemon=True)
            self.worker.start()
        response.success=True; response.message='Accepted; observe /mission/status for completion'
        return response

    def cancel(self,request,response):
        with self.lock:
            response.success=self.running
            if self.running: self.cancel_event.set()
        response.message='Cancellation requested' if response.success else 'No active mission'
        return response

    def execute(self,points,timeout):
        nav=None
        try:
            nav=BasicNavigator(node_name='mission_navigator')
            nav.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
            self.report('waiting_for_nav2')
            deadline=time.monotonic()+30.0
            while not nav.nav_to_pose_client.wait_for_server(timeout_sec=0.5):
                if self.cancel_event.is_set(): self.report('canceled'); return
                if time.monotonic()>deadline: raise RuntimeError('Nav2 action server unavailable after 30 seconds')
            for index,point in enumerate(points):
                if self.cancel_event.is_set(): self.report('canceled'); return
                pose=PoseStamped(); pose.header.frame_id='map'; pose.header.stamp=nav.get_clock().now().to_msg()
                pose.pose.position.x=float(point['x']); pose.pose.position.y=float(point['y'])
                pose.pose.orientation.z=math.sin(point['yaw']/2); pose.pose.orientation.w=math.cos(point['yaw']/2)
                self.report('navigating',waypoint=index+1,name=point.get('name',str(index+1)),total=len(points))
                if not nav.goToPose(pose): raise RuntimeError('Navigation goal rejected')
                deadline=time.monotonic()+timeout
                while not nav.isTaskComplete():
                    if self.cancel_event.is_set() or time.monotonic()>deadline:
                        nav.cancelTask()
                        self.report('canceled' if self.cancel_event.is_set() else 'timeout',waypoint=index+1)
                        return
                    time.sleep(0.05)
                if nav.getResult()!=TaskResult.SUCCEEDED:
                    raise RuntimeError('Waypoint failed: '+str(index+1))
                self.report('waypoint_reached',waypoint=index+1)
            self.report('succeeded',completed=len(points))
        except Exception as exc:
            if nav is not None:
                try: nav.cancelTask()
                except Exception: pass
            if rclpy.ok(): self.report('failed',reason=str(exc))
        finally:
            if nav is not None: nav.destroy_node()
            with self.lock: self.running=False


def main():
    rclpy.init(); node=MissionManager()
    executor=SingleThreadedExecutor(); executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.cancel_event.set()
        if node.worker is not None: node.worker.join(timeout=5.0)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
