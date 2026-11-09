"""Passive live ROS/Gazebo observation. No navigation/clock publishers, FCU sockets or commands.

A discovery timeout/missing stream is HOLD, never mock success. Pose is witness
only, never estimator input. No capture-to-FCU latency is inferred.
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import time
import threading


def timing(rows):
    gaps = [b['t']-a['t'] for a, b in zip(rows, rows[1:])]
    wall = [b['wall']-a['wall'] for a, b in zip(rows, rows[1:])]
    span = rows[-1]['t']-rows[0]['t'] if rows else 0
    return dict(count=len(rows), sim_span_s=span,
                sim_hz=(len(rows)-1)/span if span > 0 else None,
                wall_hz=(len(rows)-1)/(rows[-1]['wall']-rows[0]['wall']) if len(rows)>1 and rows[-1]['wall']>rows[0]['wall'] else None,
                max_sim_gap_s=max(gaps, default=None), max_wall_gap_s=max(wall, default=None),
                nonincreasing_stamps=sum(g <= 0 for g in gaps), gaps_over_200ms=sum(g > .2 for g in gaps))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--reliable-camera', action='store_true')
    parser.add_argument('--sdf', type=Path, default=Path('/opt/ardupilot_gazebo/models/gimbal_small_3d/model.sdf'))
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be finite and positive')
    args.output.mkdir(parents=True, exist_ok=False)
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, Imu
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    from gz.transport13 import Node
    from gz.msgs10.clock_pb2 import Clock as NativeClock
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.image_pb2 import Image as NativeImage
    import xml.etree.ElementTree as ET
    camera = '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'
    imu = '/world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu'
    paths = dict(image=camera, imu=imu, odom='/vio/odom', epoch='/integration/vio/sample_epoch', clock='/clock')
    records = {key: [] for key in [*paths, 'native_clock', 'native_image', 'poses']}
    latest_clock = [None]
    latest_native_clock = [None]
    started = time.monotonic()
    rclpy.init()
    node = rclpy.create_node('passive_live_vio_preflight')
    def stamp(t):
        return t.sec+t.nanosec*1e-9
    def base(t):
        return dict(t=t, wall=time.monotonic(), clock=latest_clock[0])
    def image_cb(msg):
        row = base(stamp(msg.header.stamp))
        row['native_clock_at_callback'] = latest_native_clock[0]
        data = np.frombuffer(msg.data, np.uint8)
        row.update(width=msg.width, height=msg.height, encoding=msg.encoding,
                   content_std=float(data.std()), nonzero=bool(data.any()))
        records['image'].append(row)
    def imu_cb(msg):
        records['imu'].append(base(stamp(msg.header.stamp)))
    def clock_cb(msg):
        latest_clock[0] = stamp(msg.clock)
        records['clock'].append(base(latest_clock[0]))
    def odom_cb(msg):
        row = base(stamp(msg.header.stamp))
        row.update(frame=msg.header.frame_id, child=msg.child_frame_id,
                   position=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z],
                   covariance=list(msg.pose.covariance), twist_covariance=list(msg.twist.covariance))
        records['odom'].append(row)
    def epoch_cb(msg):
        try:
            value = json.loads(msg.data)
            records['epoch'].append(dict(wall=time.monotonic(), value=value, t=value['stamp_ns']/1e9))
        except (ValueError, KeyError, TypeError):
            records['epoch'].append(dict(wall=time.monotonic(), error='malformed token', t=0))
    for key, cls, cb in [('image', Image, image_cb), ('imu', Imu, imu_cb), ('clock', Clock, clock_cb), ('odom', Odometry, odom_cb), ('epoch', String, epoch_cb)]:
        node.create_subscription(cls, paths[key], cb, QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE) if key=='image' and args.reliable_camera else qos_profile_sensor_data)
    native = Node()
    native_lock = threading.Lock()
    native_active = threading.Event()
    native_active.set()
    def native_clock_cb(msg):
        with native_lock:
            if native_active.is_set():
                latest_native_clock[0] = msg.sim.sec+msg.sim.nsec*1e-9
                records['native_clock'].append(dict(t=msg.sim.sec+msg.sim.nsec*1e-9, wall=time.monotonic()))
    def native_image_cb(msg):
        with native_lock:
            if native_active.is_set():
                records['native_image'].append(dict(t=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,wall=time.monotonic()))
    def pose_cb(msg):
        with native_lock:
            if native_active.is_set():
                records['poses'].append(dict(t=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9, wall=time.monotonic(),
                    poses=[dict(name=p.name, p=[p.position.x,p.position.y,p.position.z],q=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]) for p in msg.pose]))
    native.subscribe(NativeClock, '/clock', native_clock_cb)
    native.subscribe(NativeImage, camera, native_image_cb)
    native.subscribe(Pose_V, '/world/iris_runway/dynamic_pose/info', pose_cb)
    try:
        while time.monotonic()-started < args.duration:
            rclpy.spin_once(node, timeout_sec=.01)
        endpoint_info = {key: [dict(node=e.node_name, namespace=e.node_namespace, type=e.topic_type, qos=str(e.qos_profile)) for e in node.get_publishers_info_by_topic(topic)] for key, topic in paths.items()}
        (args.output/'endpoints.json').write_text(json.dumps(endpoint_info, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    with native_lock:
        native_active.clear()
    for kind, rows in records.items():
        with (args.output/(kind+'.jsonl')).open('x') as out:
            for row in rows:
                out.write(json.dumps(row, allow_nan=False)+'\n')
    summary = dict(scope='PASSIVE live observation; no FCU forwarding', duration_wall_s=time.monotonic()-started,
                   streams={k:timing(v) for k,v in records.items()}, reliable_camera=args.reliable_camera, forwarding_enabled=False, flight_commands_sent=False,
                   motion_test='NOT TESTED: no motion commands permitted', fcu_clock_mapping='NOT TESTED', capture_to_ekf_latency='NOT TESTED')
    if args.sdf.exists():
        (args.output/'camera_sensor.sdf').write_text(args.sdf.read_text())
        roll, pitch, yaw = map(float, ET.parse(args.sdf).find(".//sensor[@name='camera']/pose").text.split()[3:])
        cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
        sensor=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
        def rotation(q):
            x,y,z,w=q
            if abs(sum(v*v for v in q)-1)>1e-5:
                raise ValueError('nonunit pose quaternion')
            return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        angles=[]
        positions=[]
        for row in records['poses']:
            poses={v['name']:v for v in row['poses']}
            if not all(k in poses for k in ['iris_with_gimbal','iris_with_standoffs','gimbal','pitch_link']):
                continue
            r=rotation(poses['iris_with_standoffs']['q']).T@rotation(poses['gimbal']['q'])@rotation(poses['pitch_link']['q'])@sensor
            axes=[r[:,0],-r[:,1],-r[:,2]]
            expected=[[0,0,-1],[0,-1,0],[-1,0,0]]
            angles.append([float(np.degrees(np.arccos(np.clip(np.dot(a,b),-1,1)))) for a,b in zip(axes,expected)])
            positions.append(poses['iris_with_gimbal']['p'])
        summary['mount'] = dict(samples=len(angles), max_deviation_deg=np.max(angles,axis=0).tolist() if angles else None,
                                status='PASS' if angles and np.max(angles)<=1 else 'HOLD')
        summary['observed_motion_extent_enu_m']=np.ptp(positions,axis=0).tolist() if positions else None
    ages=[r['native_clock_at_callback']-r['t'] for r in records['image'] if r.get('native_clock_at_callback') is not None]
    summary['image_header_to_latest_native_clock_s'] = dict(minimum=min(ages), maximum=max(ages), mean=statistics.mean(ages), meaning='Gazebo-domain arrival-paired diagnostic, not FCU/capture latency') if ages else None
    holds=[]
    for key in ['odom','epoch','clock','image','imu']:
        if not records[key]:
            holds.append('No received ROS '+key+' messages')
    if summary.get('mount',{}).get('status')!='PASS':
        holds.append('Actual camera mounting not confirmed within1degree')
    holds.extend(['No verified acquisition-clock to independent FCU mapping','No motion quality/coverage measurement in this passive no-flight observation'])
    summary.update(status='HOLD', holds=holds)
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
