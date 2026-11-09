"""Live ROS-only VIO with fail-closed gate. GT Z only enters estimation.

Run after measured gimbal preflight. No MAVLink connection or flight commands.
Fault schedule is LOCAL INPUT INJECTION, never a physical sensor outage claim.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import signal
import hashlib
import uuid
from stage_timing import activate,stage,sample,mark,FaultFile


def recovery_deadline(stamp):
    """Before any camera time is available the estimator stays uninitialized."""
    return None if stamp is None else stamp+.25


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--vio-dir', type=Path, required=True)
    p.add_argument('--duration', type=float, default=70)
    p.add_argument('--ground-z', type=float, required=True)
    p.add_argument('--faults', action='store_true', help='Local receive/quality fault injections during static test')
    p.add_argument('--fault-offset',type=float,default=0,help='Keep a normal observation phase before labelled local faults')
    p.add_argument('--profile',action='store_true')
    p.add_argument('--async-diagnostics',action='store_true',help='Opt-in: move process-load sampling off frame consumer')
    p.add_argument('--reliable-camera',action='store_true',help='Opt-in: reliable subscription on our RosFrameSource node')
    p.add_argument('--min-odom-subscribers',type=int,default=0)
    p.add_argument('--min-epoch-subscribers',type=int,default=0)
    p.add_argument('--mount-calibration',type=Path,help='Opt-in fixed measured body lever; use IMU, never world GT attitude, for camera depth')
    p.add_argument('--fault-file',type=Path,help='Labelled local injection control, for isolated capture tests only')
    a = p.parse_args()
    if not all(math.isfinite(x) for x in (a.duration, a.ground_z,a.fault_offset)) or a.duration <= 0 or a.fault_offset<0:
        p.error('Invalid finite configuration')
    if min(a.min_odom_subscribers,a.min_epoch_subscribers)<0:p.error('Negative discovery requirement')
    a.output_dir.mkdir(parents=True, exist_ok=False)
    timing=activate(a.output_dir/'stages.jsonl') if a.profile else None
    fault_control=FaultFile(a.fault_file)
    def stop(signum, frame):
        raise KeyboardInterrupt('controlled stream shutdown')
    signal.signal(signal.SIGTERM, stop)
    sys.path.insert(0, str(a.vio_dir))
    import numpy as np
    import cv2
    import rclpy
    from rclpy.time import Time
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from vio_ros_out import RosOdomPublisher, quat_to_rot
    from vio_frontend import FlowFrontend
    from vio_core import VioEKF
    from vio_sensor_capture import capture
    from vio_geometry import camera_to_body_xy
    from vio_estimation_inputs import fixed_mount, vertical_inputs
    from vio_quality_gate import QualityGate, covariance_ok, distributed_flow, fresh, ekf_ready
    cv2.setNumThreads(1)
    rclpy.init()
    ros_node = rclpy.create_node('vio_gated_output')
    epoch_pub=ros_node.create_publisher(String,'/integration/vio/sample_epoch',100)
    session=str(uuid.uuid4())
    epoch_start=[None]
    log = (a.output_dir/'stream.jsonl').open('x')
    publications = (a.output_dir/'publications.jsonl').open('x')
    gate = QualityGate()
    ekf = VioEKF()
    frontend = FlowFrontend(205.4696)
    n = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.]])
    b = np.diag([1., -1, -1])
    calibrated_lever=None if a.mount_calibration is None else np.asarray(fixed_mount(a.mount_calibration))
    sensor_r = np.array([[math.cos(-1.57), math.sin(-1.57)**2, math.sin(-1.57)*math.cos(-1.57)],
                         [0, math.cos(-1.57), -math.sin(-1.57)],
                         [-math.sin(-1.57), math.cos(-1.57)*math.sin(-1.57), math.cos(-1.57)**2]])
    state = dict(last_imu=None, filter_t=None, previous=None, previous_t=None,
                 imu_index=0, initialized=False, reset_after=None, last_frame_wall=None,
                 last_seen_imu=None, last_imu_wall=None, frame_stamp=None, emitted=0,
                 accepted=0, rejected=0, gaps=0, recoveries=0)
    started = time.monotonic()
    fault_events = []
    active_fault = [None]
    schedule = [('camera_loss', 8, 10), ('imu_loss', 16, 18),
                ('low_contrast', 24, 26), ('bad_covariance', 32, 34)] if a.faults else []
    schedule=[(name,lo+a.fault_offset,hi+a.fault_offset) for name,lo,hi in schedule]
    config = dict(duration=a.duration, ground_z=a.ground_z, faults=schedule, started_monotonic_s=started,
                  profile=a.profile,async_diagnostics=a.async_diagnostics,reliable_camera=a.reliable_camera,session=session,
                  min_odom_subscribers=a.min_odom_subscribers,min_epoch_subscribers=a.min_epoch_subscribers,
                  horizontal_gt_input=False, external_nav=False, fx=205.4696,
                  gate='3 frames + 250ms, <=200ms sim image age, 500ms wall watchdog',
                  publication_max_age_sim_s=.18, observer_max_age_sim_s=.2,
                  delivery_budget_note='20ms simulation-age reserve at publication, not a guaranteed DDS deadline',
                  angular_covariance_policy='Measured aligned IMU angular velocity FRD, diagonal variance100(rad/s)^2 is conservative uncalibrated compatibility policy, not sensor noise calibration',
                  limitations='GT height, known sensor initial yaw=90deg/Rx(pi); distributed audit is not semantic shadow detection')
    config['source_sha256'] = {str(path.name): hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in [Path(__file__), Path(__file__).with_name('vio_quality_gate.py'),
                                            Path(__file__).with_name('vio_sensor_capture.py')]
                               + [a.vio_dir/name for name in ('vio_core.py', 'vio_frontend.py', 'vio_ros_source.py', 'vio_ros_out.py')]}
    config.update(fixed_mount_calibration=None if a.mount_calibration is None else str(a.mount_calibration),
                  fault_file=None if a.fault_file is None else str(a.fault_file),
                  world_gt_attitude_used_for_camera_depth=calibrated_lever is None,
                  attitude_source='Gazebo IMU orientation relative to initial sensor, known SDF spawn reference',
                  vertical_aid='Gazebo GT Z only',horizontal_gt_input=False)
    (a.output_dir/'configuration.json').write_text(json.dumps(config, indent=2)+'\n')

    def write(row):
        log.write(json.dumps(row, allow_nan=False)+'\n')
        log.flush()

    def fault():
        if a.fault_file is not None:
            value=fault_control.read().get('fault')
            return None if value=='stale' else value  # stale is injected at transport queue only
        elapsed = time.monotonic()-started
        return next((name for name, lo, hi in schedule if lo <= elapsed < hi), None)

    def rotation(q):
        q = np.asarray(q, float)
        if not np.isfinite(q).all() or abs(q@q-1) > 1e-4:
            raise ValueError('invalid quaternion')
        return quat_to_rot(q)

    class MeasurementNode:
        """Only publisher clock is acquisition time; ROS simulation clock untouched."""
        t = 0.
        def create_publisher(self, msg_type, topic, qos):
            real = ros_node.create_publisher(msg_type, topic, qos)
            if topic != '/vio/odom':
                return real
            self.odom_publisher=real
            class CheckedPublisher:
                def publish(_, msg):
                    # RosOdomPublisher leaves angular rate unknown (-1 covariance).
                    # Odometry has no IMU-style -1 sentinel. Supply ACTUAL aligned
                    # gyro and explicit large uncertainty, without changing vio/.
                    angular = state['last_imu']['gyro']
                    msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = angular
                    for i in range(3, 6):
                        msg.twist.covariance[i*6+i] = 100.
                    stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                    pose = np.asarray(msg.pose.covariance).reshape(6, 6)
                    twist = np.asarray(msg.twist.covariance).reshape(6, 6)
                    q = msg.pose.pose.orientation
                    fields = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                              msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
                              q.x, q.y, q.z, q.w]
                    if (not gate.ready or abs(stamp-output_node.t) > 1e-8
                            or msg.header.frame_id != 'odom_ned' or msg.child_frame_id != 'base_link_frd'
                            or not np.isfinite(fields+list(angular)).all() or abs(sum(v*v for v in fields[-4:])-1) > 1e-5
                            or not covariance_ok(pose, 6) or not covariance_ok(twist, 6, 100.)):
                        raise ValueError('outgoing odometry validation failed')
                    wall = time.monotonic()
                    epoch_pub.publish(String(data=json.dumps(dict(session=session,epoch=state['recoveries'],
                        stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,
                        epoch_start_ns=epoch_start[0],ready=True))))
                    with sample(round(stamp*1e9)),stage('vio_ros_enqueue'):
                        real.publish(msg)
                    publications.write(json.dumps(dict(t=stamp, publish_monotonic_s=wall))+'\n')
                    publications.flush()
                    state['emitted'] += 1
            return CheckedPublisher()
        def get_clock(self):
            class AcquisitionClock:
                def now(_):
                    return Time(nanoseconds=round(output_node.t*1e9))
            return AcquisitionClock()
    output_node = MeasurementNode()
    odom = RosOdomPublisher(node=output_node)

    def block(reason, t, reset=False):
        gate.block(reason, t, time.monotonic())
        if reset and state['reset_after'] is None:
            state['reset_after'] = recovery_deadline(t)
            if state['reset_after'] is not None:state['gaps'] += 1

    def tick(buffers):
        wall = time.monotonic()
        imu = buffers['imu'].latest()
        if imu and fault() != 'imu_loss' and imu['t'] != state['last_seen_imu']:
            state['last_seen_imu'] = imu['t']
            state['last_imu_wall'] = wall
        for key, reason in [('last_frame_wall', 'camera wall watchdog'), ('last_imu_wall', 'IMU wall watchdog')]:
            if state[key] is not None and wall-state[key] > .5:
                block(reason, state['frame_stamp'], reset=(key == 'last_imu_wall'))

    def handle(frame, buffers):
        nonlocal ekf, frontend
        gray, t, _, source_age = frame
        wall = time.monotonic()
        injected = fault()
        if injected != active_fault[0]:
            fault_events.append(dict(fault=injected, previous=active_fault[0],
                                     applied_monotonic_s=wall, frame_stamp=t))
            active_fault[0] = injected
        if injected == 'camera_loss':
            return
        state['last_frame_wall'] = wall
        state['frame_stamp'] = t
        row = dict(t=t, wall=wall, fault=injected, published=False)
        phase=time.monotonic()
        try:
            if state['previous_t'] is not None and t <= state['previous_t']:
                block('camera timestamp reset', t, reset=True)
                raise ValueError('camera timestamp reset')
            imu = buffers['imu'].snapshot()
            poses = buffers['poses'].snapshot()
            clocks = buffers['clocks'].snapshot()
            if any(buf.dropped for buf in buffers.values()):
                raise ValueError('buffer overflow')
            if injected == 'imu_loss':
                block('injected IMU input loss', t, reset=True)
                raise ValueError('injected IMU input loss')
            if not imu or not poses or not clocks or source_age is None:
                raise ValueError('sensor/clock warmup')
            clock = clocks[-1]
            if not fresh(clock['t']-t, .2) or wall-clock['arrival_monotonic_s'] > .5:
                raise ValueError('stale/uncomparable clock')
            causal = next((v for v in reversed(poses) if v['t'] <= t), None)
            if causal is None or t-causal['t'] > .05:
                raise ValueError('height timestamp unavailable')
            entries = {v['name']: v for v in causal['poses']}
            root, body, gimbal, pitch = [entries[k] for k in ('iris_with_gimbal', 'iris_with_standoffs', 'gimbal', 'pitch_link')]
            rg, rp, rb = [rotation(v['q']) for v in (gimbal, pitch, body)]
            mount = rb.T@rg@rp@sensor_r
            targets = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0.]])
            angles = np.degrees(np.arccos(np.clip(np.sum(mount*targets, axis=0), -1, 1)))
            row['mount_deg'] = angles.tolist()
            if max(angles) > 1:
                raise ValueError('camera mount')
            if state['reset_after'] is not None:
                from vio_sensor_capture import recovery_ready
                if not recovery_ready(imu, t, state['reset_after']):
                    raise ValueError('waiting for continuous IMU recovery')
                ekf, frontend = VioEKF(), FlowFrontend(205.4696)
                state.update(last_imu=None, filter_t=None, previous=None, previous_t=None, initialized=False,
                             reset_after=None, recoveries=state['recoveries']+1)
                epoch_start[0]=round(t*1e9)
                row['position_epoch_reset'] = True
            if state['last_imu'] is None:
                state['imu_index'] = next((i for i in range(len(imu)-1, -1, -1) if imu[i]['t'] <= t), -1)
                if state['imu_index'] < 0:
                    raise ValueError('waiting for first causal IMU')
                state['last_imu'] = imu[state['imu_index']]
                state['filter_t'] = state['last_imu']['t']
                state['imu_index'] += 1
            gyro_samples = []
            imu_begin=time.monotonic()
            max_gap = .001
            while state['imu_index'] < len(imu) and imu[state['imu_index']]['t'] <= t:
                m = imu[state['imu_index']]
                last = state['last_imu']
                gap = m['t']-last['t']
                max_gap = max(max_gap, gap)
                if not 0 < gap <= .02 or not np.isfinite(m['f']+m['gyro']).all():
                    block('IMU gap/reset/nonfinite', t, reset=True)
                    raise ValueError('IMU gap/reset/nonfinite')
                dt = m['t']-state['filter_t']
                if dt > 0:
                    ekf.predict(np.asarray(last['f']), rotation(last['q']), dt)
                    gyro_samples.append((dt, np.asarray(last['gyro'])))
                state.update(last_imu=m, filter_t=m['t'], imu_index=state['imu_index']+1)
            last = state['last_imu']
            imu_age = t-last['t']
            if not 0 <= imu_age <= .005:
                raise ValueError('IMU watermark/alignment')
            r = rotation(last['q'])  # Verified spawn ENU yaw90 + sensor Rx(pi) -> NED identity.
            dt = t-state['filter_t']
            if dt > 0:
                ekf.predict(np.asarray(last['f']), r, dt)
                gyro_samples.append((dt, np.asarray(last['gyro'])))
            state['filter_t'] = t
            row['imu_processing_wall_s']=time.monotonic()-imu_begin
            gyro = sum(dt*g for dt, g in gyro_samples)/sum(dt for dt, g in gyro_samples) if gyro_samples else np.asarray(last['gyro'])
            lever = np.asarray(gimbal['p'])+rg@np.asarray(pitch['p'])
            if calibrated_lever is None:
                height = root['p'][2]-a.ground_z
                camera_height = height+(rotation(root['q'])@lever)[2]
                lever_frd=b@rb.T@(lever-np.asarray(body['p']))
            else:
                height,camera_height=vertical_inputs(root['p'][2],a.ground_z,r,calibrated_lever)
                lever_frd=calibrated_lever
            if camera_height <= .05:
                raise ValueError('camera height below near clip')
            if not state['initialized']:
                ekf.x[2] = -height
                ekf.P[2, :] = 0
                ekf.P[:, 2] = 0
                ekf.P[2, 2] = ekf.sigma_z**2
                state['initialized'] = True
            else:
                ekf.update_height(-height)
            if injected in ('low_contrast','tracking_loss'):
                gray = np.zeros_like(gray)  # Explicit local fault, never presented as real frame.
            contrast = float(gray.std())
            previous, previous_t = state['previous'], state['previous_t']
            frame_gap = .1 if previous_t is None else t-previous_t
            if not fresh(frame_gap,.3):
                # Re-prime image pair instead of fusing a long-baseline correction.
                frontend = FlowFrontend(205.4696)
                previous = None
                block('camera gap', t)
            flow_begin=time.monotonic()
            result = frontend.process(gray, t, camera_height, gyro, r)
            row['frontend_wall_s']=time.monotonic()-flow_begin
            state.update(previous=gray, previous_t=t)
            audit = {'ok': False, 'reason': 'frontend unavailable'}
            accepted = False
            if result is not None:
                audit_begin=time.monotonic()
                audit = distributed_flow(previous, gray, t-previous_t, camera_height, 205.4696, gyro, r, result.v_xy)
                row['audit_wall_s']=time.monotonic()-audit_begin
                velocity = camera_to_body_xy(result.v_xy, r, gyro, lever_frd)
                row.update(flow_xy=velocity.tolist(), flow_camera_xy=result.v_xy.tolist())
                if audit['ok']:
                    accepted, nis = ekf.update_velocity_xy(velocity, result.sigma_scale)
                    row.update(nis=float(nis))
                state['accepted' if accepted else 'rejected'] += 1
                row.update(inliers=result.n_inliers, sigma_v=result.sigma_v)
            covariance = ekf.P.copy()
            if injected == 'bad_covariance':
                covariance[0, 0] = -1  # Gate injection only, estimator state remains unaltered.
            ready = gate.assess(t, wall, imu_age=imu_age, frame_age=source_age,
                                clock_age=clock['t']-t, max_imu_gap=max_gap,
                                inliers=result.n_inliers if result is not None else 0,
                                contrast=contrast, covariance=covariance,
                                state_finite=bool(np.isfinite(ekf.x).all()),
                                audit_ok=audit['ok'], ekf_ok=bool(accepted and ekf_ready(ekf) and state['reset_after'] is None), frame_gap=frame_gap)
            row.update(state=ekf.x.tolist(), covariance=ekf.P.tolist(), audit=audit,
                       contrast=contrast, flow_accepted=bool(accepted), ekf_internal_resets=ekf.n_resets, imu_age=imu_age,
                       source_age=source_age, clock_age=clock['t']-t,
                       gt_ned=(n@np.asarray(root['p'])).tolist(),
                       gt_height_only_input=True, ready=ready)
            row.update(estimator_output_monotonic_s=time.monotonic(),
                       estimator_output_clock_s=buffers['clocks'].latest()['t'],
                       frame_gap_s=frame_gap,frontend_available=result is not None,
                       session=session,epoch=state['recoveries'])
            mark('vio_estimator_output',source_ns=round(t*1e9),source_clock_s=buffers['clocks'].latest()['t'])
            if epoch_start[0] is None:epoch_start[0]=round(t*1e9)
            # GT XY above is LOGGING ONLY; never used in filter or gate.
            if ready:
                current_clock = buffers['clocks'].latest()
                current_wall = time.monotonic()
                odom_matches=output_node.odom_publisher.get_subscription_count()
                epoch_matches=epoch_pub.get_subscription_count()
                row.update(odom_subscribers=odom_matches,epoch_subscribers=epoch_matches)
                if odom_matches<a.min_odom_subscribers or epoch_matches<a.min_epoch_subscribers:
                    block('waiting for downstream DDS discovery',t);row['ready']=False
                elif not fresh(current_clock['t']-t, .18) or current_wall-current_clock['arrival_monotonic_s'] > .5:
                    block('stale at publication', t)
                    row['ready'] = False
                    row['error'] = 'stale at publication'
                else:
                    output_node.t = t
                    odom.publish(r, ekf.x[:3], ekf.x[3:6], ekf.P[:3, :3], ekf.P[3:6, 3:6], 3e-4)
                    row['published'] = True
            row['processing_s'] = time.monotonic()-wall
        except (ValueError, KeyError) as exc:
            block(str(exc), t)
            row['error'] = str(exc)
        write(row)

    camera = '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'
    imu_topic = '/world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu'
    try:
        data = capture(camera, imu_topic, a.duration, a.output_dir, handle, tick,
                       profile=a.profile,async_diagnostics=a.async_diagnostics,reliable_camera=a.reliable_camera)
        for key in ('imu', 'native_imu', 'poses', 'clocks'):
            with (a.output_dir/(key+'.jsonl')).open('x') as f:
                for row in data[key]:
                    f.write(json.dumps(row)+'\n')
        (a.output_dir/'capture_meta.json').write_text(json.dumps(dict(qos=data['qos'], overflows=data['buffer_overflows'],camera_profile=data['camera_profile']), indent=2)+'\n')
    finally:
        # capture owns shutdown; destroy publisher node before process exits.
        ros_node.destroy_node()
        log.close()
        publications.close()
        (a.output_dir/'gate_transitions.json').write_text(json.dumps(gate.transitions, indent=2)+'\n')
        (a.output_dir/'fault_events.json').write_text(json.dumps(fault_events, indent=2)+'\n')
        summary = {k: state[k] for k in ('emitted', 'accepted', 'rejected', 'gaps', 'recoveries')}
        (a.output_dir/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        if timing is not None:timing.close()
        print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
