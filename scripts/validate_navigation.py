#!/usr/bin/env python3
"""Cold-start integration test; run with scripts/humble.sh after colcon build."""
import json, math, time
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from validate_bc import ROOT, OUT, setup, launch, cleanup, yaw, log

def main():
    rclpy.init(); n=setup(.01,integrated=True); states=[]; result={}
    n.create_subscription(String,'/mission/status',lambda m:states.append(json.loads(m.data)),10)
    def request(client,req,timeout=40):
        if not client.wait_for_service(timeout_sec=timeout):raise TimeoutError(client.srv_name)
        f=client.call_async(req); n.wait(f.done,timeout); return f.result()
    def trigger(name):return request(n.create_client(Trigger,name),Trigger.Request())
    def wait_status(status,offset=0,timeout=600):
        n.wait(lambda:any(s.get('state')==status for s in states[offset:]),timeout)
    try:
        n.wait(lambda:n.initial.get_subscription_count()>0,50);n.hold(3)
        p=n.truth;m=PoseWithCovarianceStamped();m.header.frame_id='map'
        m.pose.pose.position.x=p.position.x;m.pose.pose.position.y=p.position.y
        a=yaw(p.orientation);m.pose.pose.orientation.z=math.sin(a/2);m.pose.pose.orientation.w=math.cos(a/2)
        m.pose.covariance[0]=.1;m.pose.covariance[7]=.1;m.pose.covariance[35]=.1
        n.initial.publish(m);n.hold(5)
        for node in ['amcl','planner_server','controller_server','bt_navigator','behavior_server']:
            c=n.create_client(GetState,'/'+node+'/get_state');deadline=time.monotonic()+60
            while request(c,GetState.Request()).current_state.id!=3:
                if time.monotonic()>deadline:raise TimeoutError(node+' inactive')
                n.spin()
        log('LIFECYCLE_ACTIVE')
        ac=ActionClient(n,NavigateToPose,'navigate_to_pose')
        if not ac.wait_for_server(timeout_sec=30):raise TimeoutError('navigate_to_pose')
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.pose.position.x=-.85;goal.pose.pose.orientation.w=1.
        f=ac.send_goal_async(goal);n.wait(f.done,30);handle=f.result()
        if not handle.accepted:raise RuntimeError('manual goal rejected')
        f=handle.get_result_async();n.wait(f.done,180);result['single_goal_status']=f.result().status;result['single_goal_xy']=[-.85,0.0]
        if f.result().status!=4:raise RuntimeError('single goal failed')
        log('SINGLE_GOAL_SUCCEEDED')
        # A goal inside box_C exercises the configured recovery tree.
        bad=NavigateToPose.Goal();bad.pose.header.frame_id='map'
        bad.pose.pose.position.x=1.8;bad.pose.pose.position.y=-1.8;bad.pose.pose.orientation.w=1.
        f=ac.send_goal_async(bad);n.wait(f.done,30);bad_handle=f.result()
        if not bad_handle.accepted:raise RuntimeError('blocked test goal rejected before planning')
        f=bad_handle.get_result_async();until=time.monotonic()+75
        while not f.done() and time.monotonic()<until:n.spin()
        if not f.done():
            cf=bad_handle.cancel_goal_async();n.wait(cf.done,20);n.wait(f.done,30)
        result['blocked_goal_status']=f.result().status
        if f.result().status==4:raise RuntimeError('goal inside obstacle unexpectedly succeeded')
        navlog=__import__('pathlib').Path('/tmp/rbtc_batch_sim.log').read_text()
        recovery_lines=[line for line in navlog.splitlines() if any(t in line.lower() for t in ['running spin','running wait','running backup','clearing entirely'])]
        (OUT/'recovery_events.log').write_text('\n'.join(recovery_lines)+'\n')
        result['observed_recovery_events']=len(recovery_lines)
        log('BLOCKED_GOAL_STATUS',result['blocked_goal_status'],'RECOVERY_EVENTS',len(recovery_lines))

        offset=len(states);start=trigger('/mission/start');result['start_accepted']=start.success
        duplicate=trigger('/mission/start');result['duplicate_rejected']=not duplicate.success
        cancel=trigger('/mission/cancel');result['cancel_accepted']=cancel.success
        if not all(result[k] for k in ['start_accepted','duplicate_rejected','cancel_accepted']):raise RuntimeError('service contract failed')
        wait_status('canceled',offset,60)
        until=time.monotonic()+2
        while time.monotonic()<until:n.spin()
        offset=len(states);start=trigger('/mission/start')
        if not start.success:raise RuntimeError(start.message)
        wait_status('succeeded',offset,600)
        result['mission']=next(s for s in reversed(states) if s.get('state')=='succeeded')
        result['use_ekf']=True;result['robot']='robot.urdf.xacro';result['map']='rbtc_room.yaml';result['noise_stddev_m']=.01
        (OUT/'navigation.json').write_text(json.dumps(result,indent=2))
        (OUT/'mission_states.json').write_text(json.dumps(states,indent=2))
        log('RESULT',result)
    finally:n.velocity();n.destroy_node();cleanup();rclpy.shutdown()
if __name__=='__main__':
    try: main()
    finally:
        cleanup()
        if rclpy.ok(): rclpy.shutdown()
