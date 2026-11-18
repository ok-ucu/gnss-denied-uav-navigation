"""Bounded disarmed-only camera/pose/clock test in a NEW partition.

Only gimbal setpoints and world pause/resume are sent. No MAVLink/ExternalNav.
All image bytes are real Gazebo sensor output. No image rotation or VIO.
"""
import argparse
import json
import math
from pathlib import Path
import threading
import time
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.image_pb2 import Image
from gz.msgs10.double_pb2 import Double
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.boolean_pb2 import Boolean
from mount_math import measure_mount, mount_pass

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--run',type=Path,required=True)
parser.add_argument('--variant',required=True)
parser.add_argument('--duration',type=float,default=30)
args=parser.parse_args()
out=args.run/'evidence'/args.variant
out.mkdir(exist_ok=False)
lock=threading.Lock()
node=Node()
stream=(out/'native.jsonl').open('x')
commands=(out/'commands.jsonl').open('x')
started=time.monotonic()
poses=[];images=[];clocks=[];observed=[];publishers={}
corrected_saved=[False]

def stamp(h):return h.stamp.sec*10**9+h.stamp.nsec
def record(row):
    with lock:
        stream.write(json.dumps(row,allow_nan=False)+'\n')
        stream.flush()
def rotation(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    if abs(x*x+y*y+z*z+w*w-1)>1e-5:raise ValueError('Nonunit pose quaternion')
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
sensor=ET.parse(args.run/'evidence'/f'{args.variant}_gimbal_original.sdf').find('.//sensor[@name="camera"]')
sensor_pose=list(map(float,sensor.findtext('pose').split()))
r,p,y=sensor_pose[3:];cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
rs=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
targets=[np.array(a) for a in ([0,0,-1],[0,-1,0],[-1,0,0])]

def on_pose(msg):
    elapsed=time.monotonic()-started
    # Keep complete raw hierarchy; sample at <=20Hz wall to limit evidence volume.
    if poses and elapsed-poses[-1]['elapsed_wall_s']<.05:return
    def unique(name):
        found=[p for p in msg.pose if p.name==name]
        if len(found)!=1:raise ValueError('Ambiguous/missing '+name)
        return found[0]
    try:
        body,gimbal,pitch=(unique(n) for n in ('iris_with_standoffs','gimbal','pitch_link'))
        def xyz(m):return np.array([m.position.x,m.position.y,m.position.z])
        def entry(p):return dict(position=xyz(p).tolist(),quaternion_xyzw=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
        mount=measure_mount(entry(body),entry(gimbal),entry(pitch),rs.tolist(),sensor_pose[:3])
        row=dict(kind='pose',elapsed_wall_s=elapsed,stamp_ns=stamp(msg.header),
                 receive_monotonic_ns=time.monotonic_ns(),**mount,
                 poses=[dict(id=p.id,name=p.name,position=xyz(p).tolist(),
                             quaternion_xyzw=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]) for p in msg.pose])
        poses.append(row);record(row)
    except ValueError as exc:record(dict(kind='pose_error',error=str(exc),elapsed_wall_s=elapsed))

def on_clock(msg):
    row=dict(kind='clock',stamp_ns=msg.sim.sec*10**9+msg.sim.nsec,
             receive_monotonic_ns=time.monotonic_ns(),elapsed_wall_s=time.monotonic()-started)
    # Full 1000Hz native clock trace retained, separate from receive time.
    clocks.append(row);record(row)

def on_image(msg):
    data=bytes(msg.data)
    row=dict(kind='image',stamp_ns=stamp(msg.header),receive_monotonic_ns=time.monotonic_ns(),
             elapsed_wall_s=time.monotonic()-started,width=msg.width,height=msg.height,
             step=msg.step,pixel_format=int(msg.pixel_format_type),byte_count=len(data),
             nonzero_bytes=sum(b!=0 for b in data))
    # Selected raw PPM frames, without cropping/rotation or synthetic content.
    if len(images) in (0,20,60,100) and len(data)==msg.width*msg.height*3:
        path=out/f'frame_{len(images):04d}.ppm'
        path.write_bytes(f'P6\n{msg.width} {msg.height}\n255\n'.encode()+data)
        row['raw_frame']=path.name
    if not corrected_saved[0] and poses and mount_pass([poses[-1]]) and len(data)==msg.width*msg.height*3:
        path=out/'frame_corrected.ppm'
        path.write_bytes(f'P6\n{msg.width} {msg.height}\n255\n'.encode()+data)
        row['corrected_frame']=path.name
        row['orientation_pose_stamp_ns']=poses[-1]['stamp_ns']
        corrected_saved[0]=True
    images.append(row);record(row)

topics=['/world/iris_runway/dynamic_pose/info','/clock',
 '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image']
node.subscribe(Pose_V,topics[0],on_pose);node.subscribe(Clock,topics[1],on_clock)
node.subscribe(Image,topics[2],on_image)
setpoints={'yaw':.000796326795,'roll':-.000796326795,'pitch':1.570796326795}
for axis in setpoints:
    topic='/gimbal/cmd_'+axis
    def observed_command(msg,a=axis):
        row=dict(kind='observed_command',axis=a,value_rad=msg.data,elapsed_wall_s=time.monotonic()-started)
        observed.append(row);record(row)
    node.subscribe(Double,topic,observed_command)
    publishers[axis]=node.advertise(topic,Double)

def pause(value):
    # CLI uses a separate transport client; preserve actual service response.
    cmd=['gz','service','-s','/world/iris_runway/control','--reqtype','gz.msgs.WorldControl',
         '--reptype','gz.msgs.Boolean','--timeout','5000','--req','pause: '+str(value).lower()]
    request_start=time.monotonic_ns()
    reply=subprocess.run(cmd,capture_output=True,text=True,timeout=8)
    row=dict(event='world_pause' if value else 'world_resume',
             success=reply.returncode==0 and 'data: true' in reply.stdout,
             elapsed_wall_s=time.monotonic()-started,command=cmd,
             request_start_monotonic_ns=request_start,reply_monotonic_ns=time.monotonic_ns(),
             stdout=reply.stdout,stderr=reply.stderr,returncode=reply.returncode)
    commands.write(json.dumps(row)+'\n');commands.flush()
    if not row['success']:raise RuntimeError('World control not acknowledged')

try:
    time.sleep(2)
    for axis,value in setpoints.items():
        deadline=time.monotonic()+2
        while not publishers[axis].has_connections() and time.monotonic()<deadline:
            time.sleep(.02)
        if not publishers[axis].has_connections():
            raise RuntimeError('Gimbal subscriber missing: '+axis)
        publishers[axis].publish(Double(data=value))
        commands.write(json.dumps(dict(event='setpoint',axis=axis,value_rad=value,
                                      elapsed_wall_s=time.monotonic()-started))+'\n')
    commands.flush()
    while time.monotonic()-started<12:time.sleep(.05)
    pause(True)
    time.sleep(3)
    pause(False)
    while time.monotonic()-started<args.duration:time.sleep(.05)
finally:
    # This service belongs ONLY to this new isolated partition; resume on error.
    try:pause(False)
    except Exception as exc:commands.write(json.dumps(dict(event='cleanup_error',error=str(exc)))+'\n')
    for t in topics+['/gimbal/cmd_'+a for a in setpoints]:node.unsubscribe(t)
    time.sleep(.15)
    stream.close();commands.close()
settled=[r for r in poses if r['elapsed_wall_s']>=9]
summary=dict(execution_allowed=False,forwarding_enabled=False,
    mount_status='PASS' if mount_pass(settled) else 'FAIL',
    settled_snapshots=len(settled),
    max_angles_deg=[max((r['angles_deg'][i] for r in settled),default=None) for i in range(3)],
    final_angles_deg=settled[-1]['angles_deg'] if settled else None,
    final_lever_body_flu_m=settled[-1]['lever_body_flu_m'] if settled else None,
    image_count=len(images),nonzero_image_count=sum(r['nonzero_bytes']>0 for r in images),
    pose_count=len(poses),clock_count=len(clocks),setpoints_rad=setpoints,
    unexpected_commands=[r for r in observed if abs(r['value_rad']-setpoints[r['axis']])>1e-10],
    optical_flow_accuracy='NOT TESTED',dynamic_mount_hold='NOT TESTED',
    acquisition_to_fcu_uncertainty_bound='NOT ESTABLISHED')
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
