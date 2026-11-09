"""One real ROS -> patched MAVROS -> wire interlock, CAPTURE ONLY.

Only MAVROS TIMESYNC packets may reach the new FCU. This probe also sends its
own fixed read-only parameter queries and SYSTEM_TIME requests. There is no
vision-forward switch. Shadow reference authorization is diagnostic-only and
cannot become an FCU reference or flight GO. Persistent stores survive restart.
"""
import argparse
from collections import OrderedDict
import hashlib
import importlib.util
import json
from pathlib import Path
import signal
import os
import socket
import time
from stage_timing import activate,sample,stage,mark,FaultFile,AsyncAudit
import uuid

from physics_clock_mapping import AppliedClockMapping,PATCHED_SHA
from navigation_lifecycle import NavigationLifecycle
from navigation_egress import VisionEgress
from vio_frame_bridge import convert
from vio_adapter import convert_pair,expand_upper_triangle,validate_covariance


def fcu_allowed(kind):
    return kind=='TIMESYNC'


def common_watermark(anchors,ros_clock_ns):
    """Newest APPLIED anchor also observed by ROS, not necessarily newest AP anchor."""
    if ros_clock_ns is None:return None
    return next((a for a in reversed(anchors) if a['source_ns']<=ros_clock_ns),None)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--duration',type=float,default=150)
    p.add_argument('--output-name',default='transport')
    p.add_argument('--initialize-capture-reference',help='Explicit NEW diagnostic reference UUID; never FCU authorization')
    p.add_argument('--fault-file',type=Path,help='Labelled local capture fault control')
    p.add_argument('--mavros-pid-file',type=Path,help='Owned local MAVROS PID, monitored fail-closed on process death')
    a=p.parse_args()
    if Path(a.output_name).name!=a.output_name:raise ValueError('Output name must be a basename')
    out=a.run/'evidence'/a.output_name;out.mkdir(exist_ok=False)
    recorder=activate(out/'stages.jsonl')
    fault_control=FaultFile(a.fault_file)
    if a.initialize_capture_reference is not None:uuid.UUID(a.initialize_capture_reference)
    (out/'pid').write_text(str(os.getpid()))
    binary=Path('/opt/integration/isolated_blockers/20261109_a/evidence/arducopter_patched')
    digest=hashlib.sha256(binary.read_bytes()).hexdigest()
    identity=json.loads((a.run/'artifacts/run_identity.json').read_text())
    mapping=AppliedClockMapping(identity['clock_epoch'],digest)
    gate=NavigationLifecycle(a.run/'state/capture_reference.json',mapping)
    egress=VisionEgress(a.run/'state/capture_reference.json')
    # A used reference means restart: never generate another automatically.
    initial_allowed=not gate.durable.state['used_fcu'] or a.initialize_capture_reference is not None
    spec=importlib.util.spec_from_file_location('wire','/opt/integration/mavros_live_runs/20261108/mavlink_v20.py')
    wire=importlib.util.module_from_spec(spec);spec.loader.exec_module(wire)
    import rclpy
    from rclpy.parameter import Parameter
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import PoseWithCovarianceStamped,TwistWithCovarianceStamped
    from mavros_msgs.msg import TimesyncStatus
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
    # Header times are explicitly mapped below. This wrapper does not use the
    # node clock; avoid a second, unused TimeSource subscription/executor load.
    rclpy.init();node=rclpy.create_node('phase1_capture_only_contract')
    poses=node.create_publisher(PoseWithCovarianceStamped,'/mavros/vision_pose/pose_cov',100)
    speeds=node.create_publisher(TwistWithCovarianceStamped,'/mavros/vision_speed/speed_twist_cov',100)
    converted=node.create_publisher(Odometry,'/integration/phase1/odometry_enu',100)
    logs={name:(out/(name+'.jsonl')).open('x') for name in ('events','wire','mapping','fcuwitness','odom','timesync','params')}
    raw=(out/'vision.mavlink').open('xb');tcpraw=(out/'fcu_tcp.bin').open('xb')
    audit_writer=AsyncAudit(dict(logs,raw=raw,tcpraw=tcpraw))
    def log(name,**row):
        audit_writer.put(name,dict(host_ns=time.monotonic_ns(),**row))
    running=[True];clock=[None,None];pending=OrderedDict();tokens={};expected={};last_odom=[None]
    stats=dict(odom_received=0,mapped_pairs=0,vision_received=0,validated_capture_packets=0,vision_sent_to_fcu=0,
               control_sent_to_fcu=0,heartbeat=0,armed_samples=0,clock_proof_checks=0,system_time_requests=0)
    def block(reason):
        changed=gate.reason!=reason or not gate.durable.state['blocked']
        if changed:gate.block(reason)
        if not egress.authorization.state['blocked']:egress.block()
        if changed:log('events',event='latched_closed',reason=reason)
    def stop(*_):running[0]=False
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    def on_clock(msg):
        ns=msg.clock.sec*10**9+msg.clock.nanosec
        if clock[0] is not None and ns<clock[0]:block('ROS simulation clock reset')
        if clock[0] is None or ns>clock[0]:clock[:]=[ns,time.monotonic_ns()]
    def on_odom(msg):
        ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        callback_entry=time.monotonic_ns()
        last_odom[0]=callback_entry
        try:
            with sample(ns),stage('callback_validation'):
                convert(msg)  # invalidate immediately, before waiting for a time bracket
            if clock[0] is not None and clock[0]-ns>200_000_000:
                raise ValueError('stale VIO input')
        except ValueError as exc:
            block(str(exc));return
        pending[ns]=(msg,time.monotonic_ns());stats['odom_received']+=1
        if len(pending)>32:block('bounded odometry buffer overflow');pending.popitem(last=False)
        pos=msg.pose.pose.position;vel=msg.twist.twist.linear
        log('odom',acquisition_ns=ns,callback_entry_host_ns=callback_entry,position_ned=[pos.x,pos.y,pos.z],velocity_body=[vel.x,vel.y,vel.z],
            pose_covariance=list(msg.pose.covariance),twist_covariance=list(msg.twist.covariance))
    def on_token(msg):
        try:
            row=json.loads(msg.data)
            if row.get('ready') is not True or type(row.get('epoch')) is not int or type(row.get('stamp_ns')) is not int:
                raise ValueError('invalid quality/epoch token')
            if not initial_allowed and (row['session']!=gate.durable.state['producer'] or row['epoch']!=0):
                block('producer session/epoch reset');return
            tokens[row['stamp_ns']]=row
            if len(tokens)>64:tokens.pop(next(iter(tokens)))
        except (ValueError,KeyError,TypeError):block('malformed epoch token')
    # Only the latest clock matters. A reliable 4096-deep queue can make a
    # healthy sensor appear future-dated when this executor shares a CPU cap.
    node.create_subscription(Clock,'/clock',on_clock,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(Odometry,'/vio/odom',on_odom,100)
    node.create_subscription(String,'/integration/vio/sample_epoch',on_token,100)
    node.create_subscription(TimesyncStatus,'/mavros/timesync_status',lambda m:log('timesync',remote=m.remote_timestamp_ns,
        observed=m.observed_offset_ns,estimated=m.estimated_offset_ns,rtt=m.round_trip_time_ms),qos_profile_sensor_data)
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.bind(('127.0.0.1',14890));udp.setblocking(False)
    tcp=socket.create_connection(('127.0.0.1',5760),timeout=10)
    tcp.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
    tcp.setblocking(False)
    dec_tcp=wire.MAVLink(None);dec_udp=wire.MAVLink(None)
    dec_tcp.robust_parsing=dec_udp.robust_parsing=True
    enc=wire.MAVLink(None,srcSystem=255,srcComponent=191)
    sent={};witness=[];offset=0;last_ping=0;watermark=None;started=time.monotonic()
    parameter_names=['GPS_TYPE','GPS_TYPE2','AHRS_EKF_TYPE','EK3_ENABLE','EK3_SRC_OPTIONS','EK3_SRC1_POSXY',
        'EK3_SRC1_VELXY','EK3_SRC1_POSZ','EK3_SRC1_VELZ','EK3_SRC1_YAW','VISO_TYPE','VISO_DELAY_MS',
        'VISO_POS_M_NSE','VISO_VEL_M_NSE','VISO_YAW_M_NSE','SERIAL0_PROTOCOL']
    last_param=0
    system_pending=None;last_system=0
    trace=a.run/'evidence/applied.csv'
    def refresh():
        nonlocal offset,watermark
        if trace.exists():
            with trace.open() as f:
                f.seek(offset)
                while True:
                    start=f.tell();line=f.readline()
                    if not line or not line.endswith('\n'):offset=start;break
                    offset=f.tell();v=line.strip().split(',')
                    try:
                        mapping.feed(round(float(v[0])*1e9),int(v[4])*1000,int(v[6]),parsed_no_time_sync=int(v[1]),use_time_sync=int(v[2]))
                    except ValueError as exc:
                        block(str(exc));continue
        for row in list(witness):
            try:
                if mapping.crosscheck(row['fcu_ns'],row['sent_ns'],row['received_ns']):
                    log('fcuwitness',**row,within_applied_bracket=True);witness.remove(row);stats['clock_proof_checks']+=1
            except ValueError as exc:
                log('fcuwitness',**row,within_applied_bracket=False,reason=str(exc))
                block(str(exc));witness.remove(row)
        if mapping.anchors and clock[0] is not None:
            last=common_watermark(mapping.anchors,clock[0])
            if last is not None and last['source_ns']!=watermark:
                gate.clock_tick(last['source_ns'],last['host_ns'],validation_host_ns=time.monotonic_ns());watermark=last['source_ns']

    def drain():
        nonlocal initial_allowed
        now=time.monotonic_ns()
        if not mapping.verified or clock[0] is None:return
        for ns in sorted(list(pending)):
            now=time.monotonic_ns()
            if ns not in tokens:continue
            msg,arrival=pending[ns]
            if fault_control.read().get('fault')=='stale' and now-arrival<300_000_000:
                continue  # delay the ACTUAL sample; never retimestamp or synthesize it
            if now-arrival>200_000_000:
                block('stale queued VIO');pending.pop(ns);tokens.pop(ns,None);continue
            if gate.clock is None or ns>gate.clock:continue  # wait for future bracket, never extrapolate
            token=tokens.pop(ns);pending.pop(ns)
            try:
                if initial_allowed and poses.get_subscription_count() and speeds.get_subscription_count():
                    # Explicit diagnostic contract only; there is NO FCU egress route.
                    convert_pair(convert(msg),PoseWithCovarianceStamped,TwistWithCovarianceStamped,clock[0])
                    mapping.map(ns,now,mapping.source_epoch)
                    calibration=json.loads((a.run/'evidence/integrated/summary.json').read_text())
                    if calibration['mount_status']!='PASS':raise ValueError('actual gimbal preflight failed')
                    reference='CAPTURE_ONLY:'+(a.initialize_capture_reference or identity['reference_nonce'])
                    gate.authorize(reference,token,'PASS',now)
                    egress.authorize(reference,token['session'],'PASS')
                    initial_allowed=False
                    log('events',event='diagnostic_reference_initialized',reference=reference,
                        authorization_scope='isolated capture contract only; NOT full FCU preflight',forwarding_enabled=False)
                    continue
                with sample(ns),stage('lifecycle_pair'):
                    pair=gate.pair(msg,token,now,True,PoseWithCovarianceStamped,TwistWithCovarianceStamped)
                if pair is None:
                    egress.block();log('events',event='pair_blocked',reason=gate.reason,acquisition_ns=ns);continue
                pose,speed,meta=pair
                mark('permission_created',source_ns=ns,approved_host_ns=meta['approved_host_ns'],
                     deadline_host_ns=meta['allowed_host_deadline_ns'],remaining_ns=meta['allowed_host_deadline_ns']-time.monotonic_ns())
                with sample(ns),stage('egress_approve'):
                    if not egress.approve(meta):raise ValueError('wire permission rejected')
                if time.monotonic_ns()>meta['allowed_host_deadline_ns']:
                    mark('permission_expired_before_enqueue',source_ns=ns,deadline_host_ns=meta['allowed_host_deadline_ns'])
                    raise ValueError('permission expired during conversion; never extend it')
                with sample(ns),stage('mavros_enqueue'):
                    poses.publish(pose);speeds.publish(speed)
                expected[meta['fcu_ns']//1000]=dict(position=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z],
                    covariance=list(msg.pose.covariance),meta=meta,diagnostic=gate.last_transformed,valid_seen=set())
                stats['mapped_pairs']+=1
                log('mapping',**meta,source_clock_ns=clock[0],publish_host_ns=time.monotonic_ns(),forwarding_enabled=False)
                # Reuse validated conversion, publish diagnostic only after BOTH
                # packets pass the final wire guard; no repeated matrix work.
            except (ValueError,KeyError,FileNotFoundError) as exc:block(str(exc))
    try:
        while running[0] and time.monotonic()-started<a.duration:
            now=time.monotonic_ns()
            if now-last_ping>100_000_000:
                request=wire.MAVLink_timesync_message(0,now)
                tcp.sendall(request.pack(enc));enc.seq=(enc.seq+1)%256;sent[now]=now;last_ping=now
            if stats['heartbeat'] and parameter_names and now-last_param>200_000_000:
                name=parameter_names.pop(0)
                request=wire.MAVLink_param_request_read_message(1,1,name.encode(),-1)
                tcp.sendall(request.pack(enc));enc.seq=(enc.seq+1)%256;last_param=now
            if stats['heartbeat'] and (system_pending is None or now-system_pending>1_000_000_000) and now-last_system>200_000_000:
                # Fixed read-only REQUEST_MESSAGE(SYSTEM_TIME); never arm/mode.
                request=wire.MAVLink_command_long_message(1,1,512,0,2,0,0,0,0,0,0)
                tcp.sendall(request.pack(enc));enc.seq=(enc.seq+1)%256
                system_pending=now;last_system=now;stats['system_time_requests']+=1
            try:
                data=tcp.recv(65535)
                if not data:raise RuntimeError('FCU stream closed')
            except BlockingIOError:data=b''
            if data:
                audit_writer.put('tcpraw',data)
                received=time.monotonic_ns()
                for msg in dec_tcp.parse_buffer(data) or []:
                    kind=msg.get_type()
                    if kind=='BAD_DATA':continue
                    if kind=='HEARTBEAT':
                        stats['heartbeat']+=1
                        if msg.base_mode&128:stats['armed_samples']+=1;raise RuntimeError('Unexpected ARMED FCU')
                    if kind=='TIMESYNC' and msg.tc1>0 and msg.ts1 in sent:
                        log('fcuwitness',kind='TIMESYNC_UART_CONSTRAINT',fcu_ns=msg.tc1,
                            sent_ns=sent.pop(msg.ts1),received_ns=received,used_for_clock_proof=False)
                        continue  # our monotonic token must NEVER reach MAVROS
                    if kind=='SYSTEM_TIME' and system_pending is not None:
                        witness.append(dict(kind='SYSTEM_TIME',fcu_ns=msg.time_boot_ms*1_000_000,
                            sent_ns=system_pending,received_ns=received))
                        system_pending=None
                    if kind=='PARAM_VALUE':log('params',message=msg.to_dict())
                    udp.sendto(bytes(msg.get_msgbuf()),('127.0.0.1',14891))
            try:data,_=udp.recvfrom(65535)
            except BlockingIOError:data=b''
            if data:
                wire_arrival=time.monotonic_ns()
                for msg in dec_udp.parse_buffer(data) or []:
                    kind=msg.get_type()
                    if kind=='BAD_DATA':continue
                    if fcu_allowed(kind):tcp.sendall(bytes(msg.get_msgbuf()))
                    elif kind in ('VISION_POSITION_ESTIMATE','VISION_SPEED_ESTIMATE'):
                        stats['vision_received']+=1;audit_writer.put('raw',bytes(msg.get_msgbuf()))
                        valid=False;reason=None;usec=getattr(msg,'usec',None);wire_checked=None
                        try:
                            covariance=list(msg.covariance)
                            validate_covariance(expand_upper_triangle(covariance) if kind=='VISION_POSITION_ESTIMATE' else covariance,
                                                6 if kind=='VISION_POSITION_ESTIMATE' else 3)
                            usec=msg.usec if kind=='VISION_POSITION_ESTIMATE' else msg.usec
                            with stage('wire_guard',usec=usec,kind=kind):
                                valid=egress.check(kind,usec,msg.reset_counter,time.monotonic_ns(),clock=time.monotonic_ns)
                            wire_checked=time.monotonic_ns()
                            if not valid:
                                reason='wire permission absent, revoked or expired';block(reason)
                            if msg.get_msgbuf()[0]!=253:raise ValueError('Not MAVLink2')
                            ex=expected.get(usec)
                            if valid and kind=='VISION_POSITION_ESTIMATE' and ex:
                                if max(abs(a-b) for a,b in zip([msg.x,msg.y,msg.z],ex['position']))>1e-5:raise ValueError('Wire frame/position mismatch')
                        except (ValueError,AttributeError) as exc:reason=str(exc);block(reason);valid=False
                        wire_checked=time.monotonic_ns()
                        ex=expected.get(usec)
                        if valid and ex and wire_checked>ex['meta']['allowed_host_deadline_ns']:
                            reason='permission expired at final wire boundary';block(reason);valid=False
                        stats['validated_capture_packets']+=int(valid)
                        log('wire',message=msg.to_dict(),valid_capture=valid,reason=reason,
                            arrival_host_ns=wire_arrival,checked_host_ns=wire_checked,
                            deadline_host_ns=None if expected.get(usec) is None else expected[usec]['meta']['allowed_host_deadline_ns'],
                            forwarded_to_fcu=False,global_forwarding_enabled=False)
                        if valid and ex:
                            ex['valid_seen'].add(kind)
                            if len(ex['valid_seen'])==2:
                                with stage('diagnostic_ros_publish',usec=usec):converted.publish(ex['diagnostic'])
            rclpy.spin_once(node,timeout_sec=.002)
            refresh();drain()
            if not initial_allowed and not gate.durable.state['blocked']:
                if not poses.get_subscription_count() or not speeds.get_subscription_count():block('MAVROS disconnected')
                elif last_odom[0] is not None and time.monotonic_ns()-last_odom[0]>500_000_000:
                    block('VIO availability watchdog')
                elif a.mavros_pid_file is not None:
                    try:
                        pid=int(a.mavros_pid_file.read_text())
                        if pid<=1 or Path('/proc/'+str(pid)+'/stat').read_text().split(') ')[1][0]=='Z':raise ValueError('MAVROS exited')
                        if not gate.watchdog(time.monotonic_ns()):egress.block()
                    except (OSError,ValueError,IndexError):block('owned MAVROS process stopped')
                elif not gate.watchdog(time.monotonic_ns()):egress.block()
    finally:
        gate.close();egress.close();tcp.close();udp.close()
        node.destroy_node();rclpy.shutdown()
        audit_writer.close();raw.close();tcpraw.close()
        for f in logs.values():f.close()
        recorder.close()
        with (out/'summary.json').open('x') as f:json.dump(dict(stats,clock_mapping_verified=mapping.verified,
            clock_latched=mapping.broken,gate_reason=gate.reason,decision='HOLD',execution_allowed=False,
            forwarding_enabled=False,scope='Live candidate-wire capture; no FCU vision reception/fusion'),f,indent=2)


if __name__=='__main__':main()
