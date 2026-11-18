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
import os
import threading
import contextvars
from concurrent.futures import ThreadPoolExecutor
from stage_timing import activate,stage,sample,mark,FaultFile,critical_stage
from skip_log import SkipLog
from vio_admission import Admission, keep_as_reference
import vio_flow_consistency
from vio_imu_preintegration import (new_pending, frame_step, idle_step, observe_frame_stamp, advance,
                                    ImuGapError, PreintegrationOverrun, PROPAGATION_STEP_S)


# Source publication age gate (simulation seconds), derived from the transport's
# unchanged limits so a published sample can always be paired (vio_latency_contract).
from vio_latency_contract import SOURCE_PUBLICATION_MAX_AGE_S as PUBLICATION_MAX_AGE_S, describe as latency_contract


def recovery_deadline(stamp):
    """Before any camera time is available the estimator stays uninitialized."""
    return None if stamp is None else stamp+.25


def clock_status(stamp, clock, now=None, limit=.2):
    """Describe the existing sim-age and host-receipt checks separately."""
    from vio_quality_gate import fresh
    # The callback may run after frame entry but before its clock snapshot.
    # Compare receipt to validation time, never an earlier frame-entry instant.
    now=time.monotonic() if now is None else now
    age=clock['t']-stamp
    wall_age=now-clock['arrival_monotonic_s']
    reasons=[]
    if not fresh(age,limit):
        reasons.append('simulation clock behind frame' if age<0 else 'simulation age exceeded/nonfinite')
    if not 0<=wall_age<=.5:
        reasons.append('callback receipt stale/future/nonfinite')
    return dict(valid=not reasons,reasons=reasons,frame_sim_s=stamp,clock_sim_s=clock['t'],validation_host_s=now,
        simulation_age_s=age,callback_age_wall_s=wall_age,
        ipc_delivery_s=None if 'ipc_received_monotonic_s' not in clock else clock['ipc_received_monotonic_s']-clock['arrival_monotonic_s'])


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
    p.add_argument('--isolated-imu',action='store_true',help='Opt-in: bounded spawned ROS IMU/clock receiver; no timestamp replacement')
    p.add_argument('--probe-owned-bridge',action='store_true',help='Bounded probe only: keep sensor bridge in the dedicated source process group')
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
    from vio_quality_gate import QualityGate, covariance_ok, fresh, ekf_ready
    from vio_quality_gate import measure_distributed_flow, judge_distributed_flow
    from vio_scale_compensation import predicted_scale, compensate_previous
    cv2.setNumThreads(1)
    rclpy.init()
    ros_node = rclpy.create_node('vio_gated_output')
    epoch_pub=ros_node.create_publisher(String,'/integration/vio/sample_epoch',100)
    session=str(uuid.uuid4())
    epoch_start=[None]
    log = (a.output_dir/'stream.jsonl').open('x')
    publications = (a.output_dir/'publications.jsonl').open('x')
    skips_file = (a.output_dir/'skips.jsonl').open('x')
    def emit_skip(row):
        # Rate-limited by SkipLog; mirrored to stderr (captured as console evidence).
        line=json.dumps(dict(row,host_monotonic_s=time.monotonic()),allow_nan=False)
        skips_file.write(line+'\n');skips_file.flush()
        print(line,file=sys.stderr,flush=True)
    skip_log = SkipLog(emit_skip)
    # The audit's image work runs here while the frame thread runs the frontend.
    # Flight evidence: at real-time factor ~1 (airborne) the 100 ms frame budget
    # was exceeded (median 122 ms), the backlog grew until 'stale at publication'.
    audit_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='vio-audit')
    admission = Admission(PUBLICATION_MAX_AGE_S)
    gate = QualityGate()
    ekf = VioEKF()
    frontend = FlowFrontend(205.4696)
    n = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.]])
    b = np.diag([1., -1, -1])
    calibrated_lever=None if a.mount_calibration is None else np.asarray(fixed_mount(a.mount_calibration))
    sensor_r = np.array([[math.cos(-1.57), math.sin(-1.57)**2, math.sin(-1.57)*math.cos(-1.57)],
                         [0, math.cos(-1.57), -math.sin(-1.57)],
                         [-math.sin(-1.57), math.cos(-1.57)*math.sin(-1.57), math.cos(-1.57)**2]])
    state = dict(last_imu=None, filter_t=None, previous=None, previous_t=None, previous_depth=None,
                 imu_index=0, initialized=False, reset_after=None, last_frame_wall=None,
                 last_seen_imu=None, last_imu_wall=None, frame_stamp=None, emitted=0,
                 accepted=0, rejected=0, gaps=0, recoveries=0, camera_period=None,
                 superseded_since_frame=0, preintegration_wall_s=0., **new_pending())
    publication_health=[None]
    started = time.monotonic()
    fault_events = []
    active_fault = [None]
    schedule = [('camera_loss', 8, 10), ('imu_loss', 16, 18),
                ('low_contrast', 24, 26), ('bad_covariance', 32, 34)] if a.faults else []
    schedule=[(name,lo+a.fault_offset,hi+a.fault_offset) for name,lo,hi in schedule]
    config = dict(duration=a.duration, ground_z=a.ground_z, faults=schedule, started_monotonic_s=started,
                  profile=a.profile,async_diagnostics=a.async_diagnostics,reliable_camera=a.reliable_camera,session=session,
                  isolated_imu=a.isolated_imu,probe_owned_bridge=a.probe_owned_bridge,
                  min_odom_subscribers=a.min_odom_subscribers,min_epoch_subscribers=a.min_epoch_subscribers,
                  horizontal_gt_input=False, external_nav=False, fx=205.4696,
                  gate='3 frames + 250ms, <=200ms sim image age, 500ms wall watchdog',
                  publication_max_age_sim_s=PUBLICATION_MAX_AGE_S, latency_contract=latency_contract(),
                  imu_propagation_step_s=PROPAGATION_STEP_S,
                  flow_consistency=dict(nis_limit=vio_flow_consistency.CHI2_2DOF_999, alias_k_sigma=vio_flow_consistency.K_1DOF_999,
                                        period_min_peak=vio_flow_consistency.PERIOD_MIN_PEAK,
                                        period_crop_px=vio_flow_consistency.PERIOD_CROP_PX,
                                        period_max_offset_px=vio_flow_consistency.PERIOD_MAX_OFFSET_PX),
                  observer_max_age_sim_s=.2,
                  delivery_budget_note='Publication age leaves the transport its 20 ms mapping uncertainty plus measured delivery and pairing reserves (vio_latency_contract); not a guaranteed DDS deadline',
                  angular_covariance_policy='Measured aligned IMU angular velocity FRD, diagonal variance100(rad/s)^2 is conservative uncalibrated compatibility policy, not sensor noise calibration',
                  limitations='GT height, known sensor initial yaw=90deg/Rx(pi); distributed audit is not semantic shadow detection')
    config.update(profile_pid=os.getpid(),frame_thread_native_id=threading.get_native_id())
    config['source_sha256'] = {str(path.name): hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in [Path(__file__), Path(__file__).with_name('vio_quality_gate.py'),
                                            Path(__file__).with_name('vio_sensor_capture.py')]
                               + [a.vio_dir/name for name in ('vio_core.py', 'vio_frontend.py', 'vio_ros_source.py', 'vio_ros_out.py')]}
    if a.isolated_imu:
        config['source_sha256']['isolated_imu.py']=hashlib.sha256(Path(__file__).with_name('isolated_imu.py').read_bytes()).hexdigest()
    config.update(fixed_mount_calibration=None if a.mount_calibration is None else str(a.mount_calibration),
                  fault_file=None if a.fault_file is None else str(a.fault_file),
                  world_gt_attitude_used_for_camera_depth=calibrated_lever is None,
                  attitude_source='Gazebo IMU orientation relative to initial sensor, known SDF spawn reference',
                  vertical_aid='Gazebo GT Z only',horizontal_gt_input=False)
    (a.output_dir/'configuration.json').write_text(json.dumps(config, indent=2)+'\n')

    def write(row):
        with critical_stage('source_diagnostic_write'):
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
                    if publication_health[0] is not None:
                        publication_health[0]()
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
        try:
            imu = buffers['imu'].latest()
        except ValueError as exc:
            block(str(exc),state['frame_stamp'],reset=True)
            return
        if imu and fault() != 'imu_loss' and imu['t'] != state['last_seen_imu']:
            state['last_seen_imu'] = imu['t']
            state['last_imu_wall'] = imu['arrival_monotonic_s']
        for key, reason in [('last_frame_wall', 'camera wall watchdog'), ('last_imu_wall', 'IMU wall watchdog')]:
            if state[key] is not None and wall-state[key] > .5:
                block(reason, state['frame_stamp'], reset=(key == 'last_imu_wall'))
        preintegrate(buffers)

    def preintegrate(buffers):
        # Idle-time IMU propagation: takes the per-frame IMU burst off the
        # acquisition->publish path. Never past the next expected frame, never
        # while uninitialized/resetting, and never consumes a bad sample (the
        # frame path keeps every fail-closed check).
        if fault() == 'imu_loss' or buffers['imu'].dropped:
            return
        begin = time.monotonic()
        try:
            slot = getattr(buffers['frames'], 'slot', None)
            newest = None if slot is None else slot.newest_arrived_ns()
            idle_step(state, buffers['imu'].tail_from, ekf.predict, rotation,
                      newest_arrived=None if newest is None else newest*1e-9, step_s=PROPAGATION_STEP_S)
        except ValueError:
            return  # health/quaternion failures are raised again, fail-closed, on the frame path
        finally:
            state['preintegration_wall_s'] += time.monotonic()-begin

    def camera_depth_at(buffers, t):
        # Same inputs as the frame path (GT height aid, IMU attitude); None if unavailable.
        try:
            if state['last_imu'] is None:
                return None
            r = rotation(state['last_imu']['q'])
            causal = next((v for v in reversed(buffers['poses'].snapshot()) if v['t'] <= t), None)
            if causal is None or t-causal['t'] > .05:
                return None
            entries = {v['name']: v for v in causal['poses']}
            root, gimbal, pitch = entries['iris_with_gimbal'], entries['gimbal'], entries['pitch_link']
            if calibrated_lever is None:
                lever = np.asarray(gimbal['p'])+rotation(gimbal['q'])@np.asarray(pitch['p'])
                camera_height = root['p'][2]-a.ground_z+(rotation(root['q'])@lever)[2]
            else:
                _, camera_height = vertical_inputs(root['p'][2], a.ground_z, r, calibrated_lever)
            return camera_height/max(float(r[2, 2]), .5) if camera_height > .05 else None
        except (ValueError, KeyError):
            return None

    def skip_reference(gray, t, buffers, injected):
        # Skipped frame: no estimate, but it becomes the tracking reference so the
        # next pair stays one camera period apart (vio_admission.keep_as_reference).
        if (state['last_imu'] is None or state['reset_after'] is not None or buffers['imu'].dropped
                or state['filter_t'] is None or state['filter_t'] > t):
            return False
        try:
            _, bad = advance(state, buffers['imu'].tail_from(state['imu_index']), state['imu_index'], t,
                             ekf.predict, rotation, step_s=PROPAGATION_STEP_S)
        except ValueError:
            return False
        if bad is not None:
            return False   # the next frame's IMU step fails closed on it, as before
        if injected in ('low_contrast', 'tracking_loss'):
            gray = np.zeros_like(gray)
        return keep_as_reference(state, frontend, gray, t, camera_depth_at(buffers, t))

    def on_superseded(replaced, frame):
        # Latest-only slot: the consumer was busy and newer frames replaced these.
        state['superseded_since_frame'] += len(replaced)
        for ns in replaced:
            skip_log('superseded in latest-only camera slot (consumer busy)', ns, 'vio_source',
                     replaced_by_acquisition_ns=round(frame[1]*1e9), last_processed_t=state['frame_stamp'])

    def process_frame(frame, buffers):
        nonlocal ekf, frontend
        gray, t, _, source_age = frame
        wall = time.monotonic()
        frame_cpu = time.thread_time()
        injected = fault()
        if injected != active_fault[0]:
            fault_events.append(dict(fault=injected, previous=active_fault[0],
                                     applied_monotonic_s=wall, frame_stamp=t))
            active_fault[0] = injected
        if injected == 'camera_loss':
            skip_log('injected camera loss', round(t*1e9), 'vio_source', t=t, fault=injected)
            return
        state['last_frame_wall'] = wall
        previous_stamp = state['frame_stamp']
        missing = observe_frame_stamp(state, t)
        if missing:
            skip_log('camera frames not received by source', round(t*1e9), 'vio_source',
                     missing_frames=missing, previous_t=previous_stamp, t=t)
        # Process the most recent frame when behind (vio_admission): a frame that
        # cannot be published within the unchanged publication age is skipped,
        # never two in a row, so the IMU backlog and the next frame get the time.
        try:
            clock_start = buffers['clocks'].latest()
        except ValueError:
            clock_start = None   # health failure: the frame path below fails closed as before
        if clock_start is not None and not admission.admit(clock_start['t']-t):
            reason = 'skipped: predicted to exceed publication age (consumer behind)'
            skip_log(reason, round(t*1e9), 'vio_source', t=t, age_at_start_s=clock_start['t']-t,
                     expected_processing_s=admission.expected(), publication_max_age_s=PUBLICATION_MAX_AGE_S)
            write(dict(t=t, wall=wall, fault=injected, published=False, source_age=source_age,
                       skip_reason=reason, age_at_start_s=clock_start['t']-t,
                       expected_processing_s=admission.expected(), kept_as_reference=skip_reference(gray, t, buffers, injected)))
            return
        row = dict(t=t, wall=wall, fault=injected, published=False,source_age=source_age)
        try:
            if state['previous_t'] is not None and t <= state['previous_t']:
                block('camera timestamp reset', t, reset=True)
                raise ValueError('camera timestamp reset')
            with critical_stage('source_snapshot'):
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
            if a.profile:
                row['queue_depths']=dict(imu=len(imu),poses=len(poses),clocks=len(clocks),imu_consumer_index=state['imu_index'])
            clock = clocks[-1]
            row['clock_check']=clock_status(t,clock)
            row['clock_check'].update(newest_imu_sim_s=imu[-1]['t'],clock_rows=len(clocks),imu_rows=len(imu))
            row['clock_check']['publishers']=getattr(buffers['clocks'],'publishers',[])
            if not row['clock_check']['valid']:
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
                state.update(new_pending())
                state.update(last_imu=None, filter_t=None, previous=None, previous_t=None, previous_depth=None, initialized=False,
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
                state.update(new_pending())
            imu_start_index=state['imu_index']
            imu_process_begin=time.monotonic()
            imu_begin=time.monotonic()
            imu_cpu_begin=time.thread_time()
            preintegrated=state['pending_count']
            with critical_stage('source_imu_processing'):
                try:
                    # r: verified spawn ENU yaw90 + sensor Rx(pi) -> NED identity.
                    gyro_samples, max_gap, last, r, imu_age = frame_step(state, imu, t, ekf.predict, rotation,
                                                                         step_s=PROPAGATION_STEP_S)
                except (ImuGapError, PreintegrationOverrun) as exc:
                    block(str(exc), t, reset=True)  # new estimator epoch, as before for gaps
                    raise ValueError(str(exc))
                row['imu_processing_wall_s']=time.monotonic()-imu_begin
                row['imu_processing_thread_cpu_s']=time.thread_time()-imu_cpu_begin
                row['imu_preintegrated']=preintegrated
                row['imu_preintegration_wall_s']=state['preintegration_wall_s']
                state['preintegration_wall_s']=0.
            if a.profile:
                processed=imu[imu_start_index:state['imu_index']]
                row['imu_batch']=dict(count=len(processed),preintegrated=preintegrated,pending=len(imu)-state['imu_index'],
                    max_receipt_wait_s=max((imu_process_begin-v['arrival_monotonic_s'] for v in processed),default=None))
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
            # Same depth the frontend uses; GT-derived height is already an input.
            depth = camera_height/max(float(r[2, 2]), .5)
            scale = None
            if previous is not None and frontend.prev is not None:
                scale = predicted_scale(state['previous_depth'], depth)
                # Both the pinned frontend (unmodified code, compensated input)
                # and the audit see the same compensated previous frame.
                frontend.prev = compensate_previous(frontend.prev, scale)
            row['scale_compensation'] = scale
            audit_future = None
            if previous is not None:
                def timed_measure(previous, gray, dt, scale):
                    begin=time.monotonic()
                    with critical_stage('source_spatial_audit'):
                        measured=measure_distributed_flow(previous, gray, dt, scale)
                    with critical_stage('source_texture_periods'):
                        # Image-only, on the audit worker, off the frame thread.
                        periods=vio_flow_consistency.texture_periods(previous)
                    return measured, periods, time.monotonic()-begin
                audit_future = audit_pool.submit(contextvars.copy_context().run, timed_measure,
                                                 previous, gray, t-previous_t, scale)
            flow_begin=time.monotonic()
            with critical_stage('source_frontend'):
                result = frontend.process(gray, t, camera_height, gyro, r)
            row['frontend_wall_s']=time.monotonic()-flow_begin
            state.update(previous=gray, previous_t=t, previous_depth=depth)
            audit = {'ok': False, 'reason': 'frontend unavailable' if previous is not None else 'no previous comparable frame'}
            accepted = False
            measured = None
            periods = None
            consistency = None
            prior_xy, prior_cov = ekf.x[3:5].copy(), ekf.P[3:5, 3:5].copy()   # IMU-propagated, before fusion
            if audit_future is not None:
                wait_begin=time.monotonic()
                measured, periods, row['audit_wall_s'] = audit_future.result()
                row['audit_wait_s']=time.monotonic()-wait_begin
            if result is not None:
                if measured is not None:
                    audit = judge_distributed_flow(measured, camera_height, 205.4696, gyro, r, result.v_xy)
                velocity = camera_to_body_xy(result.v_xy, r, gyro, lever_frd)
                row.update(flow_xy=velocity.tolist(), flow_camera_xy=result.v_xy.tolist())
                pair_dt = t-previous_t
                # Diagnostics (logging only): what the frontend subtracted and measured.
                row.update(gyro_frd=np.asarray(gyro, float).tolist(), depth_m=float(depth), pair_dt_s=pair_dt,
                           flow_rot_px_s=(205.4696*np.asarray(gyro, float)[:2]).tolist(),
                           flow_px_s=vio_flow_consistency.flow_px(result.v_xy, gyro, r, depth, 205.4696).tolist(),
                           prior_velocity_xy=prior_xy.tolist(), prior_velocity_cov=prior_cov.tolist(),
                           gt_body_q=list(root['q']))   # GT attitude: LOGGING ONLY
                if audit['ok'] and periods is not None:
                    # Same frame as result.v_xy: add the lever term back to the prior.
                    lever_xy = result.v_xy-velocity
                    consistency = vio_flow_consistency.check(result.v_xy, result.sigma_v, prior_xy+lever_xy, prior_cov,
                                                             gyro, r, depth, pair_dt, 205.4696, periods)
                    row['flow_consistency'] = consistency
                if audit['ok'] and consistency is not None and consistency['ok']:
                    accepted, nis = ekf.update_velocity_xy(velocity, result.sigma_scale)
                    row.update(nis=float(nis))
                state['accepted' if accepted else 'rejected'] += 1
                row.update(inliers=result.n_inliers, sigma_v=result.sigma_v)
            covariance = ekf.P.copy()
            if injected == 'bad_covariance':
                covariance[0, 0] = -1  # Gate injection only, estimator state remains unaltered.
            with critical_stage('source_quality_checks'):
                ready = gate.assess(t, wall, imu_age=imu_age, frame_age=source_age,
                                    clock_age=clock['t']-t, max_imu_gap=max_gap,
                                    inliers=result.n_inliers if result is not None else 0,
                                    contrast=contrast, covariance=covariance,
                                    state_finite=bool(np.isfinite(ekf.x).all()),
                                    audit_ok=audit['ok'], ekf_ok=bool(accepted and ekf_ready(ekf) and state['reset_after'] is None), frame_gap=frame_gap,
                                    consistency_ok=consistency is None or consistency['ok'])
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
                row['publication_clock_check']=clock_status(t,current_clock,current_wall,limit=PUBLICATION_MAX_AGE_S)
                odom_matches=output_node.odom_publisher.get_subscription_count()
                epoch_matches=epoch_pub.get_subscription_count()
                row.update(odom_subscribers=odom_matches,epoch_subscribers=epoch_matches)
                if a.profile:row['publication_check_age_sim_s']=current_clock['t']-t
                if odom_matches<a.min_odom_subscribers or epoch_matches<a.min_epoch_subscribers:
                    row['downstream_status'] = 'insufficient_dds_subscribers'
                    block('waiting for downstream DDS discovery',t);row['ready']=False
                elif not row['publication_clock_check']['valid']:
                    block('stale at publication', t)
                    row['ready'] = False
                    row['error'] = 'stale at publication'
                else:
                    output_node.t = t
                    with critical_stage('source_odom_publish'):
                        odom.publish(r, ekf.x[:3], ekf.x[3:6], ekf.P[:3, :3], ekf.P[3:6, 3:6], 3e-4)
                    row['published'] = True
                    if a.profile:row['post_enqueue_age_sim_s']=buffers['clocks'].latest()['t']-t
            row['processing_s'] = time.monotonic()-wall
        except (ValueError, KeyError) as exc:
            block(str(exc), t)
            row['error'] = str(exc)
        row.setdefault('processing_s',time.monotonic()-wall)
        row['processing_thread_cpu_s']=time.thread_time()-frame_cpu
        if clock_start is not None:
            try:
                admission.processed(buffers['clocks'].latest()['t']-clock_start['t'])
            except ValueError:
                pass
        if not row['published']:
            # Every unpublished frame names its reason in stream.jsonl; the
            # rate-limited skip log summarises them without flooding.
            row['skip_reason'] = row.get('error') or gate.reason or 'not ready'
            audit_row = row.get('audit') or {}
            skip_log(row['skip_reason'], round(t*1e9), 'vio_source', t=t, gate_ready=gate.ready,
                     gate_count=gate.count, inliers=row.get('inliers'), frontend_available=row.get('frontend_available'),
                     audit_reason=audit_row.get('reason'), clock_age_s=row.get('clock_age'),
                     downstream_status=row.get('downstream_status'),
                     odom_subscribers=row.get('odom_subscribers'),
                     epoch_subscribers=row.get('epoch_subscribers'),
                     consistency_reason=(row.get('flow_consistency') or {}).get('reason'),
                     publication_age_s=(row.get('publication_clock_check') or {}).get('simulation_age_s'),
                     processing_s=row['processing_s'], scale_compensation=row.get('scale_compensation'),
                     epoch=state['recoveries'])
        write(row)

    def handle(frame,buffers):
        publication_health[0]=buffers['clocks'].check_health
        with sample(round(frame[1]*1e9)),critical_stage('source_frame'):
            return process_frame(frame,buffers)

    camera = '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'
    imu_topic = '/world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu'
    try:
        data = capture(camera, imu_topic, a.duration, a.output_dir, handle, tick, superseded_handler=on_superseded,
                       profile=a.profile,async_diagnostics=a.async_diagnostics,reliable_camera=a.reliable_camera,
                       isolated_imu=a.isolated_imu,probe_owned_bridge=a.probe_owned_bridge)
        for key in ('imu', 'native_imu', 'poses', 'clocks'):
            with (a.output_dir/(key+'.jsonl')).open('x') as f:
                for row in data[key]:
                    f.write(json.dumps(row)+'\n')
        (a.output_dir/'capture_meta.json').write_text(json.dumps(dict(qos=data['qos'], overflows=data['buffer_overflows'],camera_slot=data['camera_slot'],camera_profile=data['camera_profile'],isolated_imu=data.get('isolated_imu')), indent=2)+'\n')
    finally:
        # capture owns shutdown; destroy publisher node before process exits.
        audit_pool.shutdown(wait=True)
        ros_node.destroy_node()
        log.close()
        publications.close()
        skip_log.flush('vio_source')
        skips_file.close()
        (a.output_dir/'gate_transitions.json').write_text(json.dumps(gate.transitions, indent=2)+'\n')
        (a.output_dir/'fault_events.json').write_text(json.dumps(fault_events, indent=2)+'\n')
        summary = {k: state[k] for k in ('emitted', 'accepted', 'rejected', 'gaps', 'recoveries')}
        (a.output_dir/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        if timing is not None:timing.close()
        print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
