"""Passive source-only experiment support; no MAVROS, FCU writes or ROS output.

Linux monotonic seconds and simulation labels remain separate. Offline analysis
uses independent ROS reception, never publication counters alone. No ROS/Gazebo
imports occur on the offline path.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import sys
import threading
import time

ROOT = Path(__file__).resolve().parent
CAMERA = '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'
IMU = '/world/iris_runway/model/iris_with_gimbal/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu'
LIMITS = dict(publication_age_sim_s=.18, observer_age_sim_s=.2,
              clock_age_sim_s=.2, host_receipt_age_s=.5, availability_wall_s=.5,
              imu_gap_sim_s=.02)


def save(path, value):
    with Path(path).open('x') as output:
        json.dump(value, output, indent=2, allow_nan=False)
        output.write('\n')


def rows(path):
    with Path(path).open() as source:
        return [json.loads(line) for line in source if line.strip()]


def stats(values):
    values = sorted(values)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite timing evidence')
    if not values:
        return dict(n=0, p50=None, p95=None, p99=None, max=None)
    def percentile(p):
        index = (len(values)-1)*p
        lo = int(index)
        return values[lo]+(values[min(lo+1, len(values)-1)]-values[lo])*(index-lo)
    return dict(n=len(values), p50=percentile(.5), p95=percentile(.95),
                p99=percentile(.99), max=values[-1])


def gaps(values):
    return stats([b-a for a, b in zip(values, values[1:])])


def hardware_renderer(identity):
    renderer=next((line.split('=',1)[1] for line in identity.splitlines()
                   if line.startswith(('renderer=','GL_RENDERER='))), '')
    bad=('llvmpipe','softpipe','swrast','software','microsoft basic render','warp','swiftshader','lavapipe')
    if (not renderer or any(word in renderer.lower() for word in bad)
            or not any(word in renderer.lower() for word in ('nvidia','radeon','intel','amd'))):
        raise ValueError('Reviewed hardware renderer not established: '+renderer)
    return renderer


def resource_sample():
    def read(path):
        try:
            return path.read_text().strip()
        except OSError:
            return None
    processes = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry/'stat').read_text().rsplit(')', 1)[1].split()
            processes.append(dict(pid=int(entry.name), comm=read(entry/'comm'),
                utime=int(fields[11]), stime=int(fields[12]),
                schedstat=read(entry/'schedstat')))
        except (OSError, IndexError, ValueError):
            continue
    return dict(kind='resources', host=time.monotonic(), processes=processes,
        ticks_per_second=os.sysconf('SC_CLK_TCK'),
        cpu_stat=read(Path('/sys/fs/cgroup/cpu.stat')),
        cpu_pressure=read(Path('/sys/fs/cgroup/cpu.pressure')),
        cpu_max=read(Path('/sys/fs/cgroup/cpu.max')))


def witness(directory):
    """Native receipt witness throughout all phases, including passive heartbeat."""
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.imu_pb2 import IMU as Imu
    import importlib.util
    spec = importlib.util.spec_from_file_location('probe_wire', ROOT/'mavros_live_runs/20261108/mavlink_v20.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    parser = module.MAVLink(None)
    records = []
    stopped = threading.Event()
    failure = []
    seen = set()
    lock = threading.Lock()
    def record(kind, **value):
        with lock:
            if len(records) >= 300000:
                failure.append('native witness capacity exceeded')
                stopped.set()
                return
            records.append(dict(kind=kind, host=time.monotonic(),
                thread_cpu_s=time.thread_time(), thread_id=threading.get_native_id(), **value))
            seen.add(kind)
    def camera(msg):
        record('camera', sim=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,
            width=msg.width, height=msg.height, bytes=len(msg.data), pixel_format=int(msg.pixel_format_type))
    def imu(msg):
        record('imu', sim=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9)
    def clock(msg):
        record('clock', sim=msg.sim.sec+msg.sim.nsec*1e-9)
    node = Node()
    for message, topic, callback in ((Image,CAMERA,camera),(Imu,IMU,imu),(Clock,'/clock',clock)):
        if not node.subscribe(message, topic, callback):
            raise RuntimeError('Native subscription failed: '+topic)
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    started = time.monotonic()
    last_heartbeat = started
    resources = []
    ready = False
    try:
        # Receive only: no MAVLink commands, heartbeats, interval requests or FCU writes.
        with socket.create_connection(('127.0.0.1',5760), timeout=2) as connection:
            connection.settimeout(.1)
            next_resource = started
            while not stopped.is_set() and time.monotonic()-started < 180:
                try:
                    data = connection.recv(65535)
                    if not data:
                        raise RuntimeError('FCU heartbeat connection closed')
                    for message in parser.parse_buffer(data) or []:
                        if message.get_type() == 'HEARTBEAT':
                            if message.autopilot != module.MAV_AUTOPILOT_ARDUPILOTMEGA:
                                continue
                            armed = bool(message.base_mode & 128)
                            record('heartbeat', armed=armed,system=message.get_srcSystem(),component=message.get_srcComponent())
                            last_heartbeat = time.monotonic()
                            if armed:
                                raise RuntimeError('FCU armed: abort source experiment')
                except socket.timeout:
                    pass
                if time.monotonic()-last_heartbeat > 3:
                    raise RuntimeError('Passive DISARMED heartbeat stale >3s')
                if time.monotonic() >= next_resource:
                    resources.append(resource_sample())
                    next_resource = time.monotonic()+.5
                if not ready and {'camera','clock','imu','heartbeat'} <= seen:
                    save(directory/'witness.ready.json', dict(host=time.monotonic()))
                    ready = True
                if not ready and time.monotonic()-started > 10:
                    raise RuntimeError('Native camera/clock/IMU/heartbeat readiness missing')
            if not stopped.is_set():
                failure.append('Native witness duration bound reached')
    except Exception as exc:
        failure.append(str(exc))
    finally:
        for topic in (CAMERA, IMU, '/clock'):
            node.unsubscribe(topic)
        time.sleep(.15)
        with (directory/'native.jsonl').open('x') as output:
            for row in records:
                output.write(json.dumps(row)+'\n')
        save(directory/'resources.json', resources)
        save(directory/'witness.json', dict(start=started, end=time.monotonic(), failure=failure,
            cap=300000, count=len(records), time_domain='Linux CLOCK_MONOTONIC seconds'))
    return 1 if failure else 0


def preflight(directory, vio_dir):
    import cv2
    import numpy as np
    import rclpy
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    import xml.etree.ElementTree as ET
    from linux_gpu.prepare_vio import FILES
    sys.path.insert(0, str(ROOT/'isolated_blockers/20261109_a/code'))
    from mount_math import measure_mount, mount_pass
    for name, digest in FILES.items():
        if hashlib.sha256((vio_dir/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Pinned VIO dependency mismatch: '+name)
    # Fail early if the supplied domain already carries another source/clock.
    rclpy.init()
    ros = rclpy.create_node('source_probe_preflight')
    try:
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            rclpy.spin_once(ros, timeout_sec=.1)
        for topic in ('/clock','/vio/odom','/integration/vio/sample_epoch'):
            if ros.get_publishers_info_by_topic(topic):
                raise ValueError('Require unused ROS domain; existing publisher: '+topic)
    finally:
        ros.destroy_node();rclpy.shutdown()
    identity=Path('/tmp/render_gl_identity.txt').read_text()
    renderer=hardware_renderer(identity)
    sensor=ET.parse('/opt/ardupilot_gazebo/models/gimbal_small_3d/model.sdf').find('.//sensor[@name="camera"]')
    p=list(map(float,sensor.findtext('pose').split()))
    r,pitch,y=p[3:]
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(pitch),math.sin(pitch),math.cos(y),math.sin(y)
    rotation=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    samples=[]
    errors=[]
    def pose(msg):
        if samples and time.monotonic()-samples[-1]['host'] < .1:
            return
        try:
            def entry(name):
                found=[v for v in msg.pose if v.name==name]
                if len(found)!=1:
                    raise ValueError('Ambiguous/missing native pose: '+name)
                v=found[0]
                return dict(position=[v.position.x,v.position.y,v.position.z],
                    quaternion_xyzw=[v.orientation.x,v.orientation.y,v.orientation.z,v.orientation.w])
            result=measure_mount(*(entry(n) for n in ('iris_with_standoffs','gimbal','pitch_link')),rotation,p[:3])
            samples.append(dict(result,host=time.monotonic(),sim=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9))
        except Exception as exc:
            errors.append(str(exc))
    node=Node();topic='/world/iris_runway/dynamic_pose/info'
    if not node.subscribe(Pose_V,topic,pose):
        raise RuntimeError('Native pose subscription failed')
    time.sleep(5);node.unsubscribe(topic);time.sleep(.15)
    save(directory/'mount_samples.json', samples)
    if errors or len(samples)<10 or samples[-1]['sim']<=samples[0]['sim'] or not mount_pass(samples):
        raise ValueError('HOLD: measured camera mount not <=1degree: '+str(errors[:3]))
    save(directory/'mount.json',dict(mount_status='PASS',settled_snapshots=len(samples),
        max_angles_deg=[max(v['angles_deg'][i] for v in samples) for i in range(3)],
        final_lever_body_flu_m=samples[-1]['lever_body_flu_m']))
    files=[ROOT/name for name in ('run_vio_stream.py','vio_sensor_capture.py','isolated_imu.py',
        'observe_frame_bridge.py','stage_timing.py','vio_source_probe.py','run_vio_source_probe.sh')]
    save(directory/'preflight.json',dict(execution_allowed=False,forwarding_enabled=False,
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        monotonic_clock=time.get_clock_info('monotonic').implementation,
        renderer=renderer,identity=identity,ros_domain=os.environ['ROS_DOMAIN_ID'],
        partition=os.environ['GZ_PARTITION'],python=sys.version,opencv=cv2.__version__,numpy=np.__version__,
        source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},
        vio_sha256=FILES))


def analyze(directory):
    """An incomplete/missing capture cannot produce PASS. No inferred CPU cause."""
    checks=[]
    measurements={}
    def check(name, condition, detail=None):
        checks.append(dict(name=name,status='PASS' if condition else 'FAIL',detail=detail))
    try:
        check('collection_exits_and_cleanup',int((directory/'collection.exit').read_text())==0)
        phases=rows(directory/'phases.jsonl')
        bounds={r['phase']:r['host'] for r in phases}
        check('phases_complete', all(n in bounds for n in ('before','on','after','end')))
        check('phase_order', bounds['before']<bounds['on']<bounds['after']<bounds['end'])
        witness_meta=json.loads((directory/'witness.json').read_text())
        native=rows(directory/'native.jsonl')
        camera_rows=[r for r in native if r['kind']=='camera']
        check('camera_shape_preserved',bool(camera_rows) and all(
            r['width']==640 and r['height']==480 and r['bytes']==640*480*3 for r in camera_rows))
        check('witness_health', not witness_meta['failure'], witness_meta['failure'])
        check('witness_covers_phases',witness_meta['start']<=bounds['before'] and witness_meta['end']>=bounds['end'])
        source=directory/'source'
        config=json.loads((source/'configuration.json').read_text())
        meta=json.loads((source/'capture_meta.json').read_text())
        summary=json.loads((source/'summary.json').read_text())
        stream=rows(source/'stream.jsonl')
        pubs=rows(source/'publications.jsonl')
        observed=rows(directory/'observer/input.jsonl')
        stages=rows(source/'stages.jsonl')
        starts=[r['host_ns']/1e9 for r in stages if r['stage']=='sensor_capture_started']
        ends=[r['host_ns']/1e9 for r in stages if r['stage']=='sensor_capture_finished']
        check('capture_bounds',len(starts)==len(ends)==1 and ends[0]-starts[0]>=60)
        end=ends[0]
        start=starts[0]
        measurements['capture_duration_wall_s']=end-start
        check('isolated_profile_enabled',config['isolated_imu'] and config['profile'] and config['min_odom_subscribers']>=1)
        check('publication_limit_unchanged',config['publication_max_age_sim_s']==LIMITS['publication_age_sim_s'])
        health=meta['isolated_imu']
        check('worker_health',health is not None and not health['failure'] and not health['worker_fault'] and health['worker_stopped'])
        check('profile_complete',len([r for r in stages if r['stage']=='profile_summary'])==1)
        check('no_record_overflow',not any(meta['overflows'].values()) and not health['metric_overflows'] and
              not meta['camera_profile']['ros_record_overflows'] and not meta['camera_profile']['native_record_overflows'] and
              all(not r.get('dropped',0) for r in stages if r['stage']=='profile_summary'))
        check('no_recovery_or_imu_gap',summary['gaps']==summary['recoveries']==0)
        publishers=meta['qos']['clock_publishers']
        check('one_clock_publisher',bool(publishers) and all(len(r['publishers'])==1 for r in publishers))
        pt=[r['t'] for r in pubs]; ot=[r['t'] for r in observed]
        check('source_and_independent_messages',bool(pt) and bool(ot))
        check('stamp_order_unique',all(b>a for values in (pt,ot) for a,b in zip(values,values[1:])))
        check('independent_delivery',pt==ot,dict(source=len(pt),received=len(ot)))
        check('independent_data_valid',bool(observed) and all(r['valid'] for r in observed))
        published=[r for r in stream if r['published']]
        check('source_rows_match_publications',[r['t'] for r in published]==pt)
        check('publication_checks',bool(published) and all(r.get('ready') and not r.get('error') and
              r['publication_clock_check']['valid'] and 0<=r['publication_clock_check']['simulation_age_s']<=.18 and
              0<=r['publication_clock_check']['callback_age_wall_s']<=.5 and r['odom_subscribers']>=1 for r in published))
        first=pubs[0]['publish_monotonic_s'] if pubs else end
        check('no_post_ready_clock_errors',not any('clock' in r.get('error','') for r in stream if r['wall']>=first))
        for name,values in (('publication',[r['publish_monotonic_s'] for r in pubs]),
                            ('subscriber',[r['arrival_monotonic_s'] for r in observed])):
            check(name+'_host_order',all(math.isfinite(v) for v in values) and all(b>a for a,b in zip(values,values[1:])))
            # Include trailing silence up to capture end. Report startup separately.
            intervals=[b-a for a,b in zip(values,values[1:])]+([max(0,end-values[-1])] if values else [end-start])
            measurements[name+'_gaps_wall_s']=stats(intervals)
            check(name+'_availability',bool(values) and max(intervals)<=.5,measurements[name+'_gaps_wall_s'])
            boundaries=values+[max(end,values[-1])] if values else [start,end]
            measurements[name+'_first_outage']=next((dict(last_host=a,next_host=b,
                availability_deadline_host=a+.5,gap_wall_s=b-a) for a,b in zip(boundaries,boundaries[1:]) if b-a>.5),None)
        measurements['startup_wall_s']=first-start
        check('bounded_readiness_and_observation',first-start<=10 and end-first>=50,
              dict(startup_wall_s=first-start,post_ready_wall_s=end-first))
        measurements['counts']=dict(source=len(pt),independent=len(ot),frames=len(stream))
        if len(pt)>1 and pt[-1]>pt[0]:
            measurements['source_rate_sim_hz']=(len(pt)-1)/(pt[-1]-pt[0])
            measurements['source_rate_wall_hz']=(len(pt)-1)/(pubs[-1]['publish_monotonic_s']-pubs[0]['publish_monotonic_s'])
        measurements['camera_profile']=meta['camera_profile']
        measurements['source_errors']=[dict(t=r['t'],host=r['wall'],error=r['error']) for r in stream if r.get('error')]
        post_ready=[r for r in stream if pt and r['t']>=pt[0]]
        measurements['flow_coverage']=dict(
            full_window=dict(accepted=sum(bool(r.get('flow_accepted')) for r in stream),frames=len(stream)),
            after_first_publication=dict(accepted=sum(bool(r.get('flow_accepted')) for r in post_ready),frames=len(post_ready)))
        measurements['publication_age_sim_s']=stats([r['publication_clock_check']['simulation_age_s'] for r in published])
        measurements['publication_to_subscriber_wall_s']=stats([b['arrival_monotonic_s']-a['publish_monotonic_s'] for a,b in zip(pubs,observed) if a['t']==b['t']])
        for field in ('processing_s','processing_thread_cpu_s','imu_processing_wall_s','imu_processing_thread_cpu_s','frontend_wall_s','audit_wall_s'):
            measurements[field]=stats([r[field] for r in stream if field in r])
        measurements['ipc_delivery_s']=stats([r['ipc_delivery_s'] for r in health['ipc_metrics']])
        check('ipc_trace_present',bool(health['ipc_metrics']))
        cameras=rows(source/'ros_camera.jsonl')
        native_cameras=rows(source/'native_camera.jsonl')
        callbacks={round(r['t']*1e9):r['arrival_monotonic_s'] for r in cameras}
        native_arrivals={round(r['t']*1e9):r['arrival_monotonic_s'] for r in native_cameras}
        measurements['camera_callback_to_consumer_wall_s']=stats([r['wall']-callbacks[round(r['t']*1e9)] for r in stream if round(r['t']*1e9) in callbacks])
        measurements['native_arrival_to_callback_wall_s']=stats([r['arrival_monotonic_s']-native_arrivals[round(r['t']*1e9)] for r in cameras if round(r['t']*1e9) in native_arrivals])
        measurements['native_arrival_to_publication_wall_s']=stats([r['publish_monotonic_s']-native_arrivals[round(r['t']*1e9)] for r in pubs if round(r['t']*1e9) in native_arrivals])
        measurements['camera_callback_gaps_wall_s']=gaps([r['arrival_monotonic_s'] for r in cameras])
        measurements['camera_queue_wait_s']=stats([r['queue_wait_ns']/1e9 for r in stages if r['stage']=='camera_queue_dequeue' and r.get('queue_wait_ns') is not None])
        measurements['camera_queue_replacements']=sum(r.get('replaced_source_ns') is not None for r in stages if r['stage']=='camera_queue_insert')
        for stage_name in ('source_imu_processing','source_frontend','source_spatial_audit'):
            matching=[r for r in stages if r['stage']==stage_name]
            measurements[stage_name]=dict(wall_s=stats([(r['end_host_ns']-r['start_host_ns'])/1e9 for r in matching]),
                thread_cpu_s=stats([r['thread_cpu_ns']/1e9 for r in matching]),
                runqueue_wait_s=stats([r['schedule_delta'][1]/1e9 for r in matching if r.get('schedule_delta') is not None]))
        transitions=json.loads((source/'gate_transitions.json').read_text())
        def ready_at(wall):
            prior=[r for r in transitions if r['wall']<=wall]
            return bool(prior) and prior[-1]['ready']
        check('no_publication_while_gate_blocked',all(ready_at(r['publish_monotonic_s']) for r in pubs))
        measurements['gate_blocks_after_first_publication']=[r for r in transitions if not r['ready'] and r['wall']>=first]
        resources=json.loads((directory/'resources.json').read_text())
        for phase,lo,hi in (('before',bounds['before'],bounds['on']),('on',bounds['on'],bounds['after']),('after',bounds['after'],bounds['end'])):
            measurements[phase]={}
            for kind in ('camera','clock','imu'):
                selected=[r for r in native if r['kind']==kind and lo<=r['host']<hi]
                check(phase+'_'+kind+'_present',len(selected)>1)
                labels=[r['sim'] for r in selected]
                check(phase+'_'+kind+'_order',all(b>=a if kind=='clock' else b>a for a,b in zip(labels,labels[1:])))
                measurements[phase][kind]=dict(count=len(selected),arrival_gap_wall_s=gaps([r['host'] for r in selected]),stamp_gap_sim_s=gaps(labels))
                if len(selected)>1 and labels[-1]>labels[0]:
                    measurements[phase][kind].update(
                        rate_sim_hz=(len(selected)-1)/(labels[-1]-labels[0]),
                        rate_wall_hz=(len(selected)-1)/(selected[-1]['host']-selected[0]['host']),
                        simulation_progress_per_arrival_wall_s=(labels[-1]-labels[0])/(selected[-1]['host']-selected[0]['host']))
            selected=[r for r in resources if lo<=r['host']<hi]
            check(phase+'_resources_present',len(selected)>1)
            check(phase+'_cpu_stat_available',bool(selected) and all(r['cpu_stat'] for r in selected))
            if len(selected)>1:
                def cpu(row):
                    return {k:int(v) for k,v in (line.split() for line in (row['cpu_stat'] or '').splitlines())}
                a,b=map(cpu,(selected[0],selected[-1]))
                measurements[phase]['cgroup']=dict(start_host=selected[0]['host'],end_host=selected[-1]['host'],
                    cpu_max=selected[0]['cpu_max'],delta={k:b[k]-a[k] for k in a if k in b})
        imu=rows(source/'imu.jsonl')
        measurements['imu_stamp_gaps_sim_s']=gaps([r['t'] for r in imu])
        check('imu_order_and_max_gap',len(imu)>1 and all(0<b['t']-a['t']<=.02+1e-9 for a,b in zip(imu,imu[1:])))
        clocks=rows(source/'clocks.jsonl')
        check('ros_clock_order',len(clocks)>1 and all(b['t']>=a['t'] for a,b in zip(clocks,clocks[1:])))
        for kind, data in (('imu',imu),('clock',clocks)):
            check(kind+'_ipc_receipt_order',bool(data) and all(
                math.isfinite(r['arrival_monotonic_s']) and math.isfinite(r['ipc_received_monotonic_s']) and
                0<=r['arrival_monotonic_s']<=r['ipc_received_monotonic_s'] for r in data) and all(
                b['ipc_sequence']>=a['ipc_sequence'] and b['arrival_monotonic_s']>=a['arrival_monotonic_s']
                for a,b in zip(data,data[1:])))
        # Require raw evidence to exist and be nonempty, not just aggregate counters.
        for name in ('clocks.jsonl','native_imu.jsonl','ros_camera.jsonl','native_camera.jsonl','process_load.json'):
            content=(source/name).read_text().strip()
            check('evidence_'+name,bool(content) and content!='[]')
    except (OSError,KeyError,IndexError,TypeError,ValueError) as exc:
        check('complete_parseable_evidence',False,str(exc))
    status='PASS' if checks and all(r['status']=='PASS' for r in checks) else 'FAIL'
    return dict(source_continuity=status,overall_decision='HOLD',execution_allowed=False,
        forwarding_enabled=False,limits=LIMITS,checks=checks,measurements=measurements,
        fault_injection='NOT TESTED: normal OFF/ON/OFF only',
        camera_generation_and_bridge_internal_timing='NOT TESTED: native receipt is not generation/bridge publication',
        fcu_latency='NOT TESTED: no MAVROS/FCU vision route')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['preflight','witness','analyze','mark'])
    parser.add_argument('--directory',type=Path,required=True)
    parser.add_argument('--vio-dir',type=Path)
    parser.add_argument('--phase')
    parser.add_argument('--result-name',default='SOURCE_RESULT.json')
    args=parser.parse_args()
    if args.action=='preflight':
        preflight(args.directory,args.vio_dir)
    elif args.action=='witness':
        return witness(args.directory)
    elif args.action=='analyze':
        result=analyze(args.directory)
        if Path(args.result_name).name!=args.result_name:
            parser.error('Result name must be a filename, never a path')
        save(args.directory/args.result_name,result)
        print(json.dumps(result,indent=2))
        return 0 if result['source_continuity']=='PASS' else 1
    else:
        with (args.directory/'phases.jsonl').open('a') as output:
            output.write(json.dumps(dict(phase=args.phase,host=time.monotonic()))+'\n')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
