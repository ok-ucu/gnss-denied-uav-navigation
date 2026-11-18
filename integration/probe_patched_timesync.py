"""Network-none MAVROS timing experiment with RECORDED real ROS inputs.

Not live VIO, not an FCU. Artificial signed peer-clock offset is a TIMESYNC control fixture;
no pose/velocity/covariance value or acquisition stamp is synthesized/shifted.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--dialect',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--mode',choices=['NONE','MAVLINK'],required=True)
    p.add_argument('--peer-offset-ns',type=int,default=1_000_000_000)
    p.add_argument('--jitter-ms',type=float,default=0)
    p.add_argument('--drift-ppm',type=float,default=0)
    p.add_argument('--domain-offset-ns',type=int,default=0)
    p.add_argument('--header-only-offset-ns',type=int,default=0,help='NEGATIVE raw-MAVROS clock-domain fixture, no FCU')
    p.add_argument('--patched',action='store_true')
    p.add_argument('--reset-test',action='store_true')
    a=p.parse_args();a.output.mkdir(exist_ok=False,parents=True)
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped,TwistWithCovarianceStamped
    from rosgraph_msgs.msg import Clock
    from mavros_msgs.msg import TimesyncStatus
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    spec=importlib.util.spec_from_file_location('wire',a.dialect);wire=importlib.util.module_from_spec(spec);spec.loader.exec_module(wire)
    rclpy.init();node=rclpy.create_node('recorded_timing_probe')
    pose=node.create_publisher(PoseWithCovarianceStamped,'/mavros/vision_pose/pose_cov',100)
    speed=node.create_publisher(TwistWithCovarianceStamped,'/mavros/vision_speed/speed_twist_cov',100)
    clockpub=node.create_publisher(Clock,'/clock',100)
    statuses=[];node.create_subscription(TimesyncStatus,'/mavros/timesync_status',lambda m:statuses.append(message_to_ordereddict(m)),qos_profile_sensor_data)
    rows=[json.loads(s) for s in a.input.read_text().splitlines()]
    poses=[r for r in rows if r['kind']=='pose'][:20];lo=poses[0]['stamp_ns']-500_000_000;hi=poses[-1]['stamp_ns']+500_000_000
    allowed={r['stamp_ns'] for r in poses}
    replay=[r for r in rows if r.get('published') and ((r['kind']=='clock' and lo<=r['stamp_ns']<=hi) or (r['kind']!='clock' and r['stamp_ns'] in allowed))]
    if a.domain_offset_ns:
        replay=[dict(r,stamp_ns=r['stamp_ns']+a.domain_offset_ns) for r in replay]
    if a.header_only_offset_ns:
        replay=[dict(r,stamp_ns=r['stamp_ns']+(a.header_only_offset_ns if r['kind']!='clock' else 0)) for r in replay]
    base_clock=replay[0]['stamp_ns']
    clock_ns=[0];stop=threading.Event();received=[];responses=[]
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',14590));sock.settimeout(.02)
    def receiver():
        dec=wire.MAVLink(None);dec.robust_parsing=True;enc=wire.MAVLink(None,srcSystem=1,srcComponent=1);last=0
        def send(msg):sock.sendto(msg.pack(enc),('127.0.0.1',14591));enc.seq=(enc.seq+1)%256
        while not stop.is_set():
            now=time.monotonic()
            if now-last>=1:send(wire.MAVLink_heartbeat_message(2,3,81,9,3,3));last=now
            try:data,_=sock.recvfrom(65535)
            except socket.timeout:continue
            for m in dec.parse_buffer(data) or []:
                received.append(dict(arrival_wall=time.monotonic(),message=m.to_dict()))
                if m.get_type()=='TIMESYNC' and m.tc1==0 and clock_ns[0]>0:
                    if a.jitter_ms:time.sleep(a.jitter_ms*.001*(len(responses)%4)/3)
                    drift=round((clock_ns[0]-base_clock)*a.drift_ppm/1e6)
                    tc=clock_ns[0]+a.peer_offset_ns+drift
                    send(wire.MAVLink_timesync_message(tc,m.ts1));responses.append(dict(tc1=tc,ts1=m.ts1,expected_offset_ns=-a.peer_offset_ns-drift))
    thread=threading.Thread(target=receiver);thread.start()
    cfg=a.config.read_text().replace('timesync_mode: NONE','timesync_mode: '+a.mode)
    if a.patched:cfg=cfg.replace('sys_time,','sys_time_patched,')
    cfg+='    timesync_rate: 10.0\n    convergence_window: 5\n'
    path=a.output/'config.yaml';path.write_text(cfg)
    console=(a.output/'mavros.console.txt').open('x')
    process=subprocess.Popen(['ros2','run','mavros','mavros_node','--ros-args','--params-file',str(path)],stdout=console,stderr=subprocess.STDOUT,start_new_session=True)
    sent=[];reset_events=[];sample_index=0;decisions={}
    from reset_safety import ResetSafety
    guard=ResetSafety(a.output/'authorization.json')
    guard.initialize('isolated-reference-A','producer-A')
    try:
        until=time.monotonic()+8
        while not (pose.get_subscription_count() and speed.get_subscription_count()):
            if process.poll() is not None or time.monotonic()>until:raise RuntimeError('MAVROS subscriptions absent')
            rclpy.spin_once(node,timeout_sec=.01)
        start=time.monotonic();origin=replay[0]['ingress_wall']
        for row in replay:
            target=start+row['ingress_wall']-origin
            while time.monotonic()<target:rclpy.spin_once(node,timeout_sec=.001)
            ns=row['stamp_ns'];kind=row['kind']
            if kind=='clock':msg=Clock();stamp=msg.clock;clock_ns[0]=ns
            else:
                if a.reset_test:
                    if ns not in decisions:
                        sample_index+=1
                        if sample_index==13:
                            guard.initialize('isolated-reference-B','producer-B')
                            reset_events.append(dict(event='explicit_new_reference_initialization',stamp_ns=ns))
                        producer='producer-A' if sample_index<7 else 'producer-B'
                        epoch=0 if sample_index<7 or sample_index>=13 else 1
                        decisions[ns]=guard.admit(producer,epoch,ns)
                        reset_events.append(dict(event='pair_decision',stamp_ns=ns,allowed=decisions[ns],epoch=epoch))
                    if not decisions[ns]:continue
                msg=PoseWithCovarianceStamped() if kind=='pose' else TwistWithCovarianceStamped();stamp=msg.header.stamp;msg.header.frame_id='odom';v=row['values']
                if kind=='pose':
                    msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z=v[:3]
                    msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w=v[3:];msg.pose.covariance=row['covariance']
                else:
                    msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z=v[:3]
                    msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z=v[3:];msg.twist.covariance=row['covariance']
                sent.append(dict(kind=kind,stamp_ns=ns))
            stamp.sec,stamp.nanosec=divmod(ns,10**9)
            {'clock':clockpub,'pose':pose,'speed':speed}[kind].publish(msg);rclpy.spin_once(node,timeout_sec=0)
        end=time.monotonic()+1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    finally:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=6)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        stop.set();thread.join(timeout=1);sock.close();console.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    vision=[r['message'] for r in received if r['message']['mavpackettype'] in ['VISION_POSITION_ESTIMATE','VISION_SPEED_ESTIMATE']]
    expected={k:{r['stamp_ns']//1000 for r in sent if r['kind']==k} for k in ['pose','speed']}
    actual={k:{r['usec'] for r in vision if r['mavpackettype']==t} for k,t in [('pose','VISION_POSITION_ESTIMATE'),('speed','VISION_SPEED_ESTIMATE')]}
    offset_errors=[r['observed_offset_ns']-next((x['expected_offset_ns'] for x in responses if x['tc1']==r['remote_timestamp_ns']),-a.peer_offset_ns) for r in statuses]
    result=dict(header_only_offset_ns=a.header_only_offset_ns,clock_domain_status='FAIL' if a.header_only_offset_ns else 'PASS',patched_overlay=a.patched,domain_offset_ns=a.domain_offset_ns,jitter_ms=a.jitter_ms,drift_ppm=a.drift_ppm,
        offset_errors_ns=offset_errors,reset_events=reset_events,mode=a.mode,scope='RECORDED ROS replay, artificial TIMESYNC peer clock; NOT FCU synchronization proof',
        sent_counts={k:len(v) for k,v in expected.items()},wire_counts={k:len(v) for k,v in actual.items()},
        timestamps_unchanged=expected==actual,reset_counters=sorted({v['reset_counter'] for v in vision}),
        timesync_responses=len(responses),timesync_statuses=statuses,child_exit=process.returncode,
        peer_offset_ns=a.peer_offset_ns,expected_offset_ns=-a.peer_offset_ns,
        offset_check='NOT TESTED' if a.mode=='NONE' else ('PASS' if statuses and all(abs(v)<=20_000_000 for v in offset_errors) else 'FAIL'))
    (a.output/'wire.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in received))
    (a.output/'responses.json').write_text(json.dumps(responses,indent=2)+'\n');(a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='timesync_statuses'},indent=2))
    if result['clock_domain_status']=='FAIL':raise SystemExit(2)
    if not result['timestamps_unchanged'] or result['offset_check']=='FAIL':raise SystemExit(1)


if __name__=='__main__':main()
