"""Offline PCAP/pose audit. Static input consistency is not dynamic fusion proof."""
import argparse
import bisect
import json
import math
from pathlib import Path
import socket
import struct
import sys
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from vio_frame_bridge import mm,vec,transpose,quat_matrix


def packets(path):
    with Path(path).open('rb') as source:
        magic=source.read(4)
        formats={b'\xd4\xc3\xb2\xa1':('<',1000),b'\xa1\xb2\xc3\xd4':('>',1000),
                 b'\x4d\x3c\xb2\xa1':('<',1),b'\xa1\xb2\x3c\x4d':('>',1)}
        if magic not in formats:raise ValueError('Expected classic PCAP, not pcapng')
        endian,scale=formats[magic]
        header=source.read(20)
        if len(header)!=20:raise ValueError('Truncated PCAP header')
        link=struct.unpack(endian+'HHIIII',header)[-1]
        if link!=1:raise ValueError('Only captured Ethernet/loopback EN10MB supported')
        while True:
            header=source.read(16)
            if not header:break
            if len(header)!=16:raise ValueError('Truncated packet header')
            seconds,fraction,size,original=struct.unpack(endian+'IIII',header)
            if size>262144:raise ValueError('Unexpected packet size')
            frame=source.read(size)
            if len(frame)!=size or size!=original:raise ValueError('Truncated packet payload')
            if len(frame)<14 or frame[12:14]!=b'\x08\x00':continue
            ip=frame[14:]
            if len(ip)<20 or ip[0]>>4!=4 or ip[9]!=17:continue
            ihl=(ip[0]&15)*4;total=struct.unpack('!H',ip[2:4])[0]
            if ihl<20 or total>len(ip) or struct.unpack('!H',ip[6:8])[0]&0x3fff:raise ValueError('Invalid/fragmented IPv4')
            udp=ip[ihl:total]
            if len(udp)<8:raise ValueError('Truncated UDP')
            sport,dport,length,_=struct.unpack('!HHHH',udp[:8])
            if length<8 or length>len(udp):raise ValueError('Invalid UDP size')
            yield dict(receive_unix_ns=seconds*10**9+fraction*scale,source_ip=socket.inet_ntoa(ip[12:16]),
                       destination_ip=socket.inet_ntoa(ip[16:20]),source_port=sport,destination_port=dport,
                       payload=udp[8:length])


def validate_physics(value):
    for key,size in [('position',3),('velocity',3),('quaternion',4)]:
        if len(value[key])!=size or not all(type(x) in (int,float) and math.isfinite(x) for x in value[key]):
            raise ValueError('Malformed '+key)
    if type(value['timestamp']) not in (int,float) or not math.isfinite(value['timestamp']):raise ValueError('Invalid timestamp')
    for key in ('gyro','accel_body'):
        if len(value['imu'][key])!=3 or not all(math.isfinite(x) for x in value['imu'][key]):raise ValueError('Invalid IMU')
    if abs(sum(x*x for x in value['quaternion'])-1)>1e-5:raise ValueError('Nonunit quaternion')


def transform(pose):
    if pose is None or not pose.strip():pose='0 0 0 0 0 0'
    x,y,z,roll,pitch,yaw=map(float,pose.split())
    cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
    return [x,y,z],[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]


def compose(a,b):
    p,r=a;q,s=b
    return [x+y for x,y in zip(p,vec(r,q))],mm(r,s)


def inverse(a):
    p,r=a;t=transpose(r)
    return [-x for x in vec(t,p)],t


def sdf_transform(element):
    values=list(map(float,element.text.split()))
    if element.get('degrees')=='true':values[3:]=[math.radians(x) for x in values[3:]]
    return transform(' '.join(map(str,values)))


def expected_json(root,body,imu,world_transform,body_transform):
    pose=([0.,0.,0.],[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
    for entry in (root,body,imu):pose=compose(pose,(entry['position'],quat_matrix(entry['quaternion_xyzw'])))
    return compose(compose(inverse(world_transform),pose),body_transform)


def analyze(capture,model_sdf,write_outputs=True):
    capture=Path(capture)
    raw=[];physics=[];servos=[]
    for row in packets(capture/'physics.pcap'):
        payload=row.pop('payload');row['payload_hex']=payload.hex();raw.append(row)
        if row['source_port']==9002:
            value=json.loads(payload.strip(b'\x00 \n\r\t'))
            validate_physics(value)
            physics.append(dict(receive_unix_ns=row['receive_unix_ns'],source_port=row['source_port'],destination_port=row['destination_port'],physics=value))
        elif row['destination_port']==9002 and len(payload)==40:
            magic,rate,count,*pwm=struct.unpack('<HHI16H',payload)
            servos.append(dict(receive_unix_ns=row['receive_unix_ns'],magic=magic,frame_rate=rate,frame_count=count,pwm=pwm))
    # Every generated output is exclusive; never overwrite a prior capture.
    if write_outputs:
        for name,rows in [('udp_packets.jsonl',raw),('physics_parsed.jsonl',physics),('servo_parsed.jsonl',servos)]:
            with (capture/name).open('x') as out:
                for row in rows:out.write(json.dumps(row,allow_nan=False)+'\n')
    tree=ET.parse(model_sdf);plugin=tree.find(".//plugin[@filename='ArduPilotPlugin']")
    world=sdf_transform(plugin.find('gazeboXYZToNED'));body=sdf_transform(plugin.find('modelXYZToAirplaneXForwardZDown'))
    poses=[json.loads(s) for s in (capture/'poses_clock.jsonl').read_text().splitlines()]
    poses=[r for r in poses if r['kind']=='pose'];times=[r['stamp_ns'] for r in poses]
    comparisons=[]
    for row in physics:
        t=round(row['physics']['timestamp']*10**9);index=bisect.bisect_left(times,t)
        choices=[i for i in (index-1,index) if 0<=i<len(poses)]
        if not choices:continue
        i=min(choices,key=lambda i:abs(times[i]-t));pose=poses[i]
        if abs(times[i]-t)>20_000_000:continue
        entries={name:[p for p in pose['poses'] if p['name']==name] for name in ('iris_with_gimbal','iris_with_standoffs','imu_link')}
        if any(len(values)!=1 for values in entries.values()):raise ValueError('Ambiguous pose hierarchy')
        p,r=expected_json(*(entries[k][0] for k in entries),world,body)
        w,x,y,z=row['physics']['quaternion'];actual=quat_matrix([x,y,z,w]);delta=mm(transpose(r),actual)
        angle=math.degrees(math.acos(max(-1,min(1,(sum(delta[j][j] for j in range(3))-1)/2))))
        comparisons.append(dict(json_timestamp_ns=t,pose_timestamp_ns=pose['stamp_ns'],delta_sim_ns=pose['stamp_ns']-t,
                                position_error_m=math.sqrt(sum((a-b)**2 for a,b in zip(p,row['physics']['position']))),rotation_error_deg=angle))
    if write_outputs:
        with (capture/'pose_comparisons.jsonl').open('x') as out:
            for row in comparisons:out.write(json.dumps(row)+'\n')
    stamps=[row['physics']['timestamp'] for row in physics]
    roots=[[x for x in row['poses'] if x['name']=='iris_with_gimbal'][0]['position'] for row in poses if any(x['name']=='iris_with_gimbal' for x in row['poses'])]
    extent=max((math.dist(p,roots[0]) for p in roots),default=None) if roots else None
    static_pass=bool(physics) and len(comparisons)==len(physics) and extent is not None and extent<1e-6 and all(r['position_error_m']<1e-5 and r['rotation_error_deg']<.001 for r in comparisons)
    return dict(decision='HOLD',clock_mapping='HOLD',execution_allowed=False,forwarding_enabled=False,
                static_input_consistency='PASS' if static_pass else 'HOLD',model_motion_extent_m=extent,
                udp_packets=len(raw),json_packets=len(physics),servo_packets=len(servos),
                json_span_s=stamps[-1]-stamps[0] if len(stamps)>1 else None,
                json_timestamp_gaps_s=[b-a for a,b in zip(stamps,stamps[1:])],
                no_time_sync_wire_values=sorted({r['physics']['no_time_sync'] for r in physics}),
                no_lockstep_wire_values=sorted({r['physics']['no_lockstep'] for r in physics}),
                comparison_count=len(comparisons),max_pair_delta_sim_ms=max((abs(r['delta_sim_ns'])/1e6 for r in comparisons),default=None),
                max_position_error_m=max((r['position_error_m'] for r in comparisons),default=None),
                max_rotation_error_deg=max((r['rotation_error_deg'] for r in comparisons),default=None),
                same_physical_state_scope='Static JSON input/IMU-link pose comparison only; dynamic equivalence NOT TESTED',
                dynamic_equivalence_verified=False,fcu_estimator_agreement='NOT TESTED',
                proof_limit='PCAP proves UDP state presence; not individual packet consumption, VIO fusion or bounded capture-to-FCU delay')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture');p.add_argument('--model-sdf',required=True)
    p.add_argument('--read-only',action='store_true',help='Recompute without rewriting parsed capture evidence')
    p.add_argument('--summary-output',type=Path)
    args=p.parse_args();result=analyze(args.capture,args.model_sdf,not args.read_only)
    with (args.summary_output or Path(args.capture)/'summary.json').open('x') as out:json.dump(result,out,indent=2);out.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='json_timestamp_gaps_s'},indent=2))
