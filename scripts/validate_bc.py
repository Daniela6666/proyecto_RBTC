#!/usr/bin/env python3
"""Measured Gazebo trials. Run using scripts/humble.sh python3 scripts/validate_bc.py CASE."""
import os
os.environ['ROS_DOMAIN_ID']='43'
os.environ['IGN_PARTITION']='rbtc_batch'
os.environ['LIBGL_ALWAYS_SOFTWARE']='1'
os.environ.setdefault('DISPLAY',':0')
import sys, math, time, signal, subprocess, json, csv, shutil
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'validation'; OUT.mkdir(exist_ok=True)
PROCS=[]

def log(*args): print(*args,flush=True)
def launch(label,args):
    f=open('/tmp/rbtc_'+label+'.log','w')
    proc=subprocess.Popen(args,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    PROCS.append((proc,f)); return proc

def cleanup():
    for proc,f in reversed(PROCS):
        try:os.killpg(proc.pid,signal.SIGINT)
        except ProcessLookupError:pass
    for proc,f in reversed(PROCS):
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:os.killpg(proc.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            proc.wait(timeout=3)
        f.close()
    PROCS.clear()

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
class Probe(Node):
    def __init__(self):
        super().__init__('validation_probe')
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.odom=None; self.truth=None; self.scan=None; self.grid=None; self.rows=[]; self.t0=None
        self.cmd=self.create_publisher(Twist,'/cmd_vel',10)
        self.initial=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'odom',m),qos_profile_sensor_data)
        self.create_subscription(PoseArray,'/ground_truth',self.on_truth,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',lambda m:setattr(self,'scan',m),qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'grid',m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_amcl,10)
    def on_truth(self,m):
        if m.poses:self.truth=m.poses[0]
    def on_amcl(self,m):
        if self.truth is None or self.t0 is None:return
        p=m.pose.pose; t=self.truth
        self.rows.append([self.now()-self.t0,p.position.x,p.position.y,yaw(p.orientation),t.position.x,t.position.y,yaw(t.orientation),math.hypot(p.position.x-t.position.x,p.position.y-t.position.y),abs(wrap(yaw(p.orientation)-yaw(t.orientation)))])
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def spin(self):rclpy.spin_once(self,timeout_sec=.02)
    def wait(self,predicate,timeout=40):
        end=time.monotonic()+timeout
        while not predicate():
            self.spin()
            if time.monotonic()>end:raise TimeoutError('Waiting for ROS data')
    def velocity(self,v=0.,w=0.):
        m=Twist();m.linear.x=float(v);m.angular.z=float(w);self.cmd.publish(m)
    def hold(self,seconds,w=0.):
        start=self.now(); end=time.monotonic()+max(40,seconds*8)
        while self.now()-start<seconds:
            self.spin();self.velocity(0,w)
            if time.monotonic()>end:raise TimeoutError('Simulation clock stopped')
        self.velocity()
    def drive(self,points,speed):
        for index,(tx,ty) in enumerate(points):
            end=time.monotonic()+300; last=0
            while True:
                self.spin();p=self.odom.pose.pose
                dx,dy=tx-p.position.x,ty-p.position.y; d=math.hypot(dx,dy)
                if d<.12:break
                a=wrap(math.atan2(dy,dx)-yaw(p.orientation))
                v=min(speed,.8*d)*max(0,math.cos(a)) if abs(a)<.45 else 0.
                self.velocity(v,max(-.8,min(.8,1.8*a)))
                if time.monotonic()-last>20:
                    log('DRIVE',index+1,'target',tx,ty,'distance',round(d,3),'sim',round(self.now(),1));last=time.monotonic()
                if time.monotonic()>end:raise TimeoutError('Waypoint not reached')
            self.velocity();self.hold(.5)
            log('REACHED',index+1,tx,ty,'sim',round(self.now(),1))

def setup(noise,integrated=False):
    n=Probe()
    launch('batch_sim',['ros2','launch','dif_bot','bringup.launch.py','headless:=true'] if integrated else ['ros2','launch','dif_bot','sim.launch.py','headless:=true','world:=rbtc_room.sdf','lidar_noise:='+str(noise)])
    launch('batch_truth',['ros2','run','ros_gz_bridge','parameter_bridge','/world/rbtc_room/dynamic_pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V','--ros-args','-r','/world/rbtc_room/dynamic_pose/info:=/ground_truth'])
    n.wait(lambda:n.odom is not None and n.scan is not None and n.truth is not None)
    n.hold(2)
    log('READY','scan',n.scan.header.frame_id,'truth',n.truth.position.x,n.truth.position.y)
    return n

def map_metrics(prefix):
    meta=yaml.safe_load(prefix.with_suffix('.yaml').read_text()); a=np.array(Image.open(prefix.with_suffix('.pgm')))
    h,w=a.shape; yy,xx=np.indices(a.shape); r=meta['resolution']; ox,oy,_=meta['origin']; x=ox+(xx+.5)*r;y=oy+(h-yy-.5)*r
    roi=(abs(x)<3.9)&(abs(y)<3.9)
    obstacles=[(1.1,1.9,.6,1.8),(-2,-1,1.35,2.05),(1.5,2.1,-2.1,-1.5)]
    expected=np.zeros(a.shape,dtype=bool)
    for x0,x1,y0,y1 in obstacles:expected|=(x>=x0)&(x<=x1)&(y>=y0)&(y<=y1)
    free_roi=roi&~expected
    occupied=a<100; distance=np.minimum(np.maximum(3.9-abs(x),0),np.maximum(3.9-abs(y),0))
    for x0,x1,y0,y1 in obstacles:
        d=np.hypot(np.maximum(np.maximum(x0-x,x-x1),0),np.maximum(np.maximum(y0-y,y-y1),0));distance=np.minimum(distance,d)
    values=distance[occupied&roi]
    return dict(resolution_m=r,width=w,height=h,known_free_space_fraction=float(np.mean(a[free_roi]!=205)),occupied_distance_mean_m=float(np.mean(values)) if len(values) else None,occupied_distance_p95_m=float(np.percentile(values,95)) if len(values) else None)

def mapping(case,speed):
    n=setup(.01)
    try:
        launch('batch_slam',['ros2','launch','dif_bot','slam.launch.py'])
        n.wait(lambda:n.grid is not None,45)
        start=n.now()
        points=[(0,-3),(-3,-3),(-3,3),(3,3),(3,-3),(0,-3),(0,0)]
        n.drive(points,speed);n.hold(4)
        prefix=OUT/case
        subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-f',str(prefix),'--ros-args','-p','use_sim_time:=true','-p','save_map_timeout:=15.0'],check=True,timeout=30,stdout=open('/tmp/rbtc_map_save.log','w'),stderr=subprocess.STDOUT)
        result=map_metrics(prefix);result.update(case=case,speed_m_s=speed,noise_stddev_m=.01,duration_sim_s=n.now()-start,route=points)
        (OUT/(case+'.json')).write_text(json.dumps(result,indent=2));log('RESULT',result)
        if case=='mapping_slow':
            shutil.copy2(prefix.with_suffix('.pgm'),ROOT/'src/dif_bot/maps/rbtc_room.pgm')
            meta=yaml.safe_load(prefix.with_suffix('.yaml').read_text());meta['image']='rbtc_room.pgm'
            (ROOT/'src/dif_bot/maps/rbtc_room.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
    finally:n.velocity();n.destroy_node()

def localization(case,noise,wrong):
    n=setup(noise)
    try:
        launch('batch_amcl',['ros2','launch','dif_bot','localization.launch.py','map:='+str(ROOT/'src/dif_bot/maps/rbtc_room.yaml')])
        n.wait(lambda:n.initial.get_subscription_count()>0,40);n.hold(3)
        t=n.truth; m=PoseWithCovarianceStamped();m.header.frame_id='map';m.pose.pose.position.x=t.position.x+(1.0 if wrong else 0);m.pose.pose.position.y=t.position.y+(1.0 if wrong else 0)
        angle=yaw(t.orientation)+(.6 if wrong else 0);m.pose.pose.orientation.z=math.sin(angle/2);m.pose.pose.orientation.w=math.cos(angle/2)
        m.pose.covariance[0]=.5;m.pose.covariance[7]=.5;m.pose.covariance[35]=.25
        n.t0=n.now();n.initial.publish(m);n.hold(22,w=.3)
        n.drive([(0,-1.0),(-.7,-1.0),(-.7,0),(0,0)],.2);n.hold(3)
        if len(n.rows)<10:raise RuntimeError('Insufficient AMCL samples')
        with open(OUT/(case+'.csv'),'w') as f:
            writer=csv.writer(f);writer.writerow(['sim_s','amcl_x','amcl_y','amcl_yaw','truth_x','truth_y','truth_yaw','position_error_m','yaw_error_rad']);writer.writerows(n.rows)
        a=np.array(n.rows);ok=(a[:,7]<.2)&(a[:,8]<.15);conv=None
        for i in range(len(ok)-4):
            if all(ok[i:i+5]):conv=float(a[i,0]);break
        result=dict(case=case,noise_stddev_m=noise,incorrect_initial_pose=wrong,samples=len(a),convergence_sim_s=conv,final_position_error_m=float(np.median(a[-10:,7])),final_yaw_error_rad=float(np.median(a[-10:,8])),p95_position_error_m=float(np.percentile(a[:,7],95)))
        (OUT/(case+'.json')).write_text(json.dumps(result,indent=2));log('RESULT',result)
    finally:n.velocity();n.destroy_node()

def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case',choices=['mapping_slow','mapping_fast','amcl_correct_low','amcl_wrong_low','amcl_correct_high','amcl_wrong_high'])
    case=parser.parse_args().case;rclpy.init()
    try:
        if case=='mapping_slow':mapping(case,.2)
        elif case=='mapping_fast':mapping(case,.5)
        else:
            parts=case.split('_');localization(case,.05 if 'high' in parts else .01,'wrong' in parts)
    finally:cleanup();rclpy.shutdown()
if __name__=='__main__':main()
