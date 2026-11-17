"""Independent input/output observer. No bridge conversion helper, publishers or GT."""
import argparse
import json
from pathlib import Path
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory', type=Path, required=True)
    p.add_argument('--duration', type=float, default=65)
    p.add_argument('--input-topic', default='/vio/odom')
    p.add_argument('--output-topic', default='/integration/vio/odometry_enu')
    a = p.parse_args()
    a.directory.mkdir(exist_ok=False)
    import numpy as np
    import rclpy
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    rclpy.init()
    node = rclpy.create_node('isolated_frame_bridge_observer')
    clock = [None, None]
    last = dict(input=None, output=None)
    logs = {name: (a.directory/(name+'.jsonl')).open('x') for name in last}
    def on_clock(msg):
        clock[:] = [msg.clock.sec+msg.clock.nanosec*1e-9, time.monotonic()]
    def callback(side):
        def receive(msg):
            wall = time.monotonic()
            t = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            def vec(v): return [v.x, v.y, v.z]
            q = msg.pose.pose.orientation
            quaternion = vec(q)+[q.w]
            position = vec(msg.pose.pose.position)
            linear, angular = vec(msg.twist.twist.linear), vec(msg.twist.twist.angular)
            pose, twist = [np.asarray(c).reshape(6,6) for c in (msg.pose.covariance,msg.twist.covariance)]
            covariance_valid = all(np.isfinite(c).all() and np.allclose(c,c.T,atol=1e-10,rtol=0)
                                   and np.linalg.eigvalsh(c).min() >= -1e-10 for c in (pose,twist))
            frames = ('odom_ned','base_link_frd') if side=='input' else ('odom','base_link')
            age = None if clock[0] is None else clock[0]-t
            valid = (np.isfinite(position+quaternion+linear+angular).all() and covariance_valid
                     and abs(sum(x*x for x in quaternion)-1)<1e-5
                     and (msg.header.frame_id,msg.child_frame_id)==frames
                     and (last[side] is None or t>last[side])
                     and age is not None and -1e-9<=age<=.2+1e-9
                     and 0<=wall-clock[1]<=.5)
            row=dict(t=t,sec=msg.header.stamp.sec,nanosec=msg.header.stamp.nanosec,
                     arrival_monotonic_s=wall,valid=bool(valid),age_sim_s=age,
                     frame=msg.header.frame_id,child_frame=msg.child_frame_id,
                     position=position,q=quaternion,linear=linear,angular=angular,
                     pose_covariance=list(msg.pose.covariance),twist_covariance=list(msg.twist.covariance))
            logs[side].write(json.dumps(row,allow_nan=False)+'\n');logs[side].flush()
            last[side]=t
        return receive
    node.create_subscription(Clock,'/clock',on_clock,4096)
    node.create_subscription(Odometry,a.input_topic,callback('input'),100)
    node.create_subscription(Odometry,a.output_topic,callback('output'),100)
    start=time.monotonic()
    try:
        while time.monotonic()-start<a.duration:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        for log in logs.values():log.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
