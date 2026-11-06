#!/usr/bin/env python3
"""Real Gazebo cmd_vel motion and tracked pixel displacements; never launches SITL/VIO."""
import argparse
import csv
import json
import math
import statistics
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


def evaluate_motion(frames, odometry, flows, min_displacement=.1):
    """Verdict from measurements, never command counts or successful process exit.

    Threshold is metres per commanded axis. Pixel checks cover signs only at
    identity attitude; insufficient tracks leave that check NOT TESTED.
    """
    if not math.isfinite(min_displacement) or min_displacement <= 0:
        raise ValueError('Minimum displacement must be finite and positive')
    failures = []
    measured = {}
    for phase, axis in [('world_x', 'x'), ('world_y', 'y')]:
        images = [f for f in frames if f['phase'] == phase]
        poses = [o for o in odometry if o['phase'] == phase]
        if len(images) < 2:
            failures.append(f'{phase}: missing camera frames')
        if len(poses) < 2:
            failures.append(f'{phase}: missing odometry')
            continue
        values = [float(o[axis]) for o in poses]
        delta = values[-1] - values[0]
        measured[phase] = delta
        if not all(math.isfinite(v) for v in values) or delta < min_displacement:
            failures.append(f'{phase}: insufficient positive measured displacement')
    movement_status = 'FAIL' if failures else 'PASS'
    optical = {'status': 'NOT TESTED', 'reason': 'Insufficient consistent feature tracks or unverified attitude'}
    identity = bool(odometry) and all(
        all(k in o and math.isfinite(float(o[k])) and abs(float(o[k])-expected) < 1e-3
            for k, expected in [('qx', 0), ('qy', 0), ('qz', 0), ('qw', 1)])
        for o in odometry)
    directions = {}
    for phase, pixel in [('world_x', 'dv_px'), ('world_y', 'du_px')]:
        pairs = [f for f in flows if f['phase'] == phase and f.get('previous_phase') == phase
                 and float(f.get('dt_s', 0)) > 0 and int(f.get('tracks', 0)) >= 4
                 and math.isfinite(float(f[pixel]))]
        if len(pairs) >= 3:
            directions[phase] = statistics.median(float(f[pixel]) for f in pairs)
    if identity and len(directions) == 2:
        optical = {'status': 'PASS' if all(v > .01 for v in directions.values()) else 'FAIL',
                   'median_dominant_shift_px': directions,
                   'scope': 'Positive-axis pixel signs only; not flow magnitude or VIO accuracy'}
        if optical['status'] == 'FAIL':
            failures.append('Measured pixel direction does not match positive-axis motion')
    return dict(status='FAIL' if failures else 'PASS' if optical['status'] == 'PASS' else 'NOT TESTED',
                movement_status=movement_status, optical_flow=optical,
                measured_displacement_m=measured, minimum_displacement_m=min_displacement,
                failures=failures)


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
    completed = False
    result = None
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
        completed = True
    finally:
        # Send zero repeatedly while transport is alive, even on a processing error.
        state['active']=False
        for _ in range(10):
            send(); rclpy.spin_once(node, timeout_sec=.02)
        for name, rows in [('frames',frames),('flow',flows),('odometry',odometry),('commands',commands)]:
            with (args.output_dir/(name+'.csv')).open('w',newline='') as f:
                if rows:
                    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        result = evaluate_motion(frames, odometry, flows)
        if not completed:
            result['status'] = 'FAIL'
            result['failures'].append('Capture interrupted or failed')
        (args.output_dir/'capture.json').write_text(json.dumps(dict(frames=len(frames),flow_pairs=len(flows),
            odometry=len(odometry),commands=len(commands),phases=PHASES,
            status=result['status'], verdict=result,
            scope='Gazebo kinematic camera only; independent image tracking, no VIO/ArduPilot'),indent=2)+'\n')
        node.destroy_node(); rclpy.shutdown()
    return {'PASS': 0, 'FAIL': 1, 'NOT TESTED': 2}[result['status']]


if __name__=='__main__':
    raise SystemExit(main())
