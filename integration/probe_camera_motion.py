#!/usr/bin/env python3
"""Real Gazebo cmd_vel motion and tracked pixel displacements; never launches SITL/VIO."""
import argparse
import csv
import json
from pathlib import Path
import time

PHASES = [('stationary', 2.5, 0, 0), ('world_x', 2.5, .2, 0),
          ('pause', 1.5, 0, 0), ('world_y', 2.5, 0, .2), ('stop', 2.5, 0, 0)]


def phase_at(elapsed):
    end = 0
    for name, duration, vx, vy in PHASES:
        end += duration
        if elapsed < end:
            return name, vx, vy
    return 'stop', 0, 0


def main():
    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    frames_dir = args.output_dir/'frames'
    frames_dir.mkdir(exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('integration_camera_motion_probe')
    pub = node.create_publisher(Twist, '/model/camera_drone/cmd_vel', 10)
    frames, flows, odometry, commands = [], [], [], []
    state = {'active': False, 'start': 0, 'previous': None, 'clock': None}
    def stamp(s):
        return s.sec + s.nanosec * 1e-9
    def image(msg):
        if not state['active']:
            return
        wall = time.monotonic()-state['start']
        phase = phase_at(wall)[0]
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        if msg.encoding not in ('rgb8', 'bgr8', 'mono8'):
            raise RuntimeError('Unsupported real image encoding: '+msg.encoding)
        if msg.encoding == 'mono8':
            gray = raw[:, :msg.width].copy()
        else:
            pixels = raw[:, :msg.width*3].reshape(msg.height, msg.width, 3)
            gray = cv2.cvtColor(pixels, cv2.COLOR_RGB2GRAY if msg.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
        index = len(frames)
        name = f'frames/{index:04d}.png'
        if not cv2.imwrite(str(args.output_dir/name), gray):
            raise OSError('Could not save actual frame')
        ts = stamp(msg.header.stamp)
        frames.append(dict(index=index, phase=phase, wall_s=wall, stamp_s=ts,
                           sim_clock_s=state['clock'], file=name))
        previous = state['previous']
        if previous is not None:
            old, old_ts, old_phase = previous
            points = cv2.goodFeaturesToTrack(old, maxCorners=300, qualityLevel=.01, minDistance=7)
            if points is not None and len(points)>=4:
                tracked, ok, _ = cv2.calcOpticalFlowPyrLK(old, gray, points, None)
                if tracked is not None:
                    back, back_ok, _ = cv2.calcOpticalFlowPyrLK(gray, old, tracked, None)
                    mask = (ok[:,0]>0) & (back_ok[:,0]>0) & (np.linalg.norm(back[:,0]-points[:,0], axis=1)<.5)
                    displacement = (tracked-points)[:,0][mask]
                    if len(displacement)>=4:
                        du, dv = np.median(displacement, axis=0)
                        flows.append(dict(frame=index, phase=phase, previous_phase=old_phase,
                            stamp_s=ts, dt_s=ts-old_ts, tracks=len(displacement),
                            du_px=float(du), dv_px=float(dv)))
        state['previous'] = (gray, ts, phase)
    def odom(msg):
        if not state['active']:
            return
        p=msg.pose.pose.position; q=msg.pose.pose.orientation; v=msg.twist.twist.linear
        odometry.append(dict(wall_s=time.monotonic()-state['start'], phase=phase_at(time.monotonic()-state['start'])[0],
            stamp_s=stamp(msg.header.stamp), frame_id=msg.header.frame_id, child_frame_id=msg.child_frame_id,
            x=p.x,y=p.y,z=p.z,qx=q.x,qy=q.y,qz=q.z,qw=q.w,vx=v.x,vy=v.y,vz=v.z))
    def clock(msg):
        state['clock'] = stamp(msg.clock)
    def send(vx=0,vy=0):
        command=Twist(); command.linear.x=float(vx); command.linear.y=float(vy)
        pub.publish(command)
    node.create_subscription(Image, '/camera/image_raw', image, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/model/camera_drone/odometry', odom, qos_profile_sensor_data)
    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    try:
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            send()
            rclpy.spin_once(node, timeout_sec=.05)
        if pub.get_subscription_count()==0:
            raise RuntimeError('No cmd_vel bridge subscriber')
        state['start']=time.monotonic(); state['active']=True
        last_send=-1
        duration=sum(p[1] for p in PHASES)
        while time.monotonic()-state['start']<duration:
            elapsed=time.monotonic()-state['start']
            if elapsed-last_send>=.05:
                phase,vx,vy=phase_at(elapsed); send(vx,vy)
                commands.append(dict(wall_s=elapsed,phase=phase,vx=vx,vy=vy,sim_clock_s=state['clock']))
                last_send=elapsed
            rclpy.spin_once(node, timeout_sec=.005)
    finally:
        # Send zero repeatedly while transport is alive, even on a processing error.
        state['active']=False
        for _ in range(10):
            send(); rclpy.spin_once(node, timeout_sec=.02)
        for name, rows in [('frames',frames),('flow',flows),('odometry',odometry),('commands',commands)]:
            with (args.output_dir/(name+'.csv')).open('w',newline='') as f:
                if rows:
                    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        (args.output_dir/'capture.json').write_text(json.dumps(dict(frames=len(frames),flow_pairs=len(flows),
            odometry=len(odometry),commands=len(commands),phases=PHASES,
            scope='Gazebo kinematic camera only; independent image tracking, no VIO/ArduPilot'),indent=2)+'\n')
        node.destroy_node(); rclpy.shutdown()


if __name__=='__main__':
    main()
