"""JSON-lines process relay across a network-none container boundary.

Exports only gated mock pose/speed plus shared simulation clock. No MAVLink.
Ingress uses real ROS messages; drops stale input rather than replaying it.
"""
import argparse
import json
import queue
import sys
import threading
import time
from pathlib import Path


def validate(row):
    if row['kind'] not in ('clock','pose','speed'):
        raise ValueError('Unknown relay kind')
    if type(row['stamp_ns']) is not int or row['stamp_ns']<=0:
        raise ValueError('Invalid timestamp')
    if row['kind']!='clock':
        from vio_adapter import validate_covariance
        import math
        if row['frame']!='odom' or not all(math.isfinite(v) for v in row['values']):
            raise ValueError('Invalid frame/values')
        if len(row['values'])!=(7 if row['kind']=='pose' else 6):
            raise ValueError('Invalid payload length')
        validate_covariance(row['covariance'])
    return row


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['export','ingress'])
    p.add_argument('--log',type=Path,required=True);a=p.parse_args()
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped,TwistWithCovarianceStamped
    from rosgraph_msgs.msg import Clock
    rclpy.init();node=rclpy.create_node('mavros_isolated_'+a.mode)
    log=a.log.open('x');last_clock=[0];clock=[None];pending=queue.Queue(maxsize=1024);ended=threading.Event()
    types={'clock':Clock,'pose':PoseWithCovarianceStamped,'speed':TwistWithCovarianceStamped}
    def serialize(kind,msg):
        stamp=msg.clock if kind=='clock' else msg.header.stamp
        ns=stamp.sec*10**9+stamp.nanosec
        if kind=='clock' and ns-last_clock[0]<10_000_000 and ns>=last_clock[0]:return
        if kind=='clock':last_clock[0]=ns
        row=dict(kind=kind,stamp_ns=ns,export_wall=time.monotonic())
        if kind!='clock':
            row['frame']=msg.header.frame_id
            if kind=='pose':
                v=msg.pose.pose.position;q=msg.pose.pose.orientation
                row['values']=[v.x,v.y,v.z,q.x,q.y,q.z,q.w];row['covariance']=list(msg.pose.covariance)
            else:
                v=msg.twist.twist.linear;w=msg.twist.twist.angular
                row['values']=[v.x,v.y,v.z,w.x,w.y,w.z];row['covariance']=list(msg.twist.covariance)
        line=json.dumps(row,allow_nan=False);log.write(line+'\n');log.flush();print(line,flush=True)
    if a.mode=='export':
        topics={'clock':'/clock','pose':'/integration/mock_mavros/vision_pose/pose_cov',
                'speed':'/integration/mock_mavros/vision_speed/speed_twist_cov'}
        for kind in topics:node.create_subscription(types[kind],topics[kind],lambda m,k=kind:serialize(k,m),4096 if kind=='clock' else 100)
    else:
        topics={'clock':'/clock','pose':'/mavros/vision_pose/pose_cov','speed':'/mavros/vision_speed/speed_twist_cov'}
        pubs={k:node.create_publisher(types[k],topic,100) for k,topic in topics.items()}
        def reader():
            try:
                for line in sys.stdin:pending.put(json.loads(line),timeout=1)
            finally:ended.set()
        threading.Thread(target=reader,daemon=True).start()
        def drain():
            for _ in range(256):
                try:row=validate(pending.get_nowait())
                except queue.Empty:break
                row=dict(row,ingress_wall=time.monotonic());kind=row['kind'];ns=row['stamp_ns']
                reason=None
                # Same Docker-host monotonic clock, including transport queue residence.
                if row['ingress_wall']-row['export_wall']>.2:reason='stale relay arrival'
                if kind!='clock' and (clock[0] is None or not -1e-9<=(clock[0]-ns)/1e9<=.2+1e-9):reason='stale/future sim stamp'
                if reason:row['rejected']=reason
                else:
                    msg=types[kind]();stamp=msg.clock if kind=='clock' else msg.header.stamp
                    stamp.sec,stamp.nanosec=divmod(ns,10**9)
                    if kind=='clock':clock[0]=ns
                    else:
                        msg.header.frame_id=row['frame'];v=row['values']
                        if kind=='pose':
                            msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=v[:3]
                            msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w=v[3:]
                            msg.pose.covariance=row['covariance']
                        else:
                            msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z=v[:3]
                            msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z=v[3:]
                            msg.twist.covariance=row['covariance']
                    pubs[kind].publish(msg);row['published']=True
                log.write(json.dumps(row,allow_nan=False)+'\n');log.flush()
        node.create_timer(.002,drain)
    try:
        while rclpy.ok() and not (a.mode=='ingress' and ended.is_set() and pending.empty()):
            rclpy.spin_once(node,timeout_sec=.01)
    except KeyboardInterrupt:pass
    finally:
        log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
