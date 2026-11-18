"""CPU-only hardware bring-up policy/estimator. No ROS, devices or FCU writes.

Body origin is the IMU sensing point, axes FRD. Attitude is an independently
measured IMU-sensor -> ENU quaternion, NOT estimated by this velocity EKF.
All raw acquisition stamps remain integer nanoseconds in a verified system clock.
"""
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import platform
import sys

import numpy as np

from linux_gpu.prepare_vio import FILES
from vio_adapter import validate_covariance
from vio_geometry import camera_to_body_xy
from vio_quality_gate import QualityGate, distributed_flow, ekf_ready

ENU_NED=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
CAMERA_FRD=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
IMU_GAP_NS=20_000_000
ALIGN_NS=5_000_000
FRAME_GAP_NS=300_000_000
PUBLICATION_NS=180_000_000
WATCHDOG_NS=500_000_000


def finite(values,shape):
    a=np.asarray(values,dtype=float)
    if a.shape!=shape or not np.isfinite(a).all():raise ValueError('Invalid shape/nonfinite measurement')
    return a


def rotation(values):
    r=finite(values,(3,3))
    if not np.allclose(r.T@r,np.eye(3),rtol=0,atol=1e-6) or abs(np.linalg.det(r)-1)>1e-6:
        raise ValueError('Require SO(3) rotation, not a reflection')
    return r


def quaternion(q):
    x,y,z,w=finite(q,(4,))
    if abs(x*x+y*y+z*z+w*w-1)>1e-5:raise ValueError('Unnormalized measured attitude')
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def covariance(values):
    a=finite(values,(3,3));validate_covariance(a.ravel().tolist(),3)
    if np.min(np.diag(a))<=0:raise ValueError('Unknown/zero sensor covariance')
    return a


def stamp_ns(stamp):
    if type(stamp.sec) is not int or type(stamp.nanosec) is not int or stamp.sec<0 or not 0<=stamp.nanosec<10**9:
        raise ValueError('Invalid integer acquisition timestamp')
    ns=stamp.sec*10**9+stamp.nanosec
    if ns<=0:raise ValueError('Missing acquisition timestamp')
    return ns


def validate_config(config,base):
    """No driver/device auto-discovery or inferred calibration/clock claims."""
    c=config
    if (c.get('schema')!=1 or c.get('mode')!='physical-cpu-only' or
            c.get('execution_allowed') is not False or c.get('forwarding_enabled') is not False):
        raise ValueError('Physical bring-up scope and disabled safety flags required')
    if c.get('clock_domain')!='ros-system-unix-ns' or c.get('stamp_semantics')!='acquisition':
        raise ValueError('Require verified acquisition -> ROS system clock mapping')
    if c.get('calibration_verified') is not True:raise ValueError('Physical calibration NOT VERIFIED')
    for key in ('calibration_evidence','clock_evidence'):
        ref=c[key];path=(Path(base)/ref['path']).resolve()
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=ref['sha256']:
            raise ValueError('Missing/mismatched '+key)
    uncertainty=c['clock_uncertainty_ns']
    if type(uncertainty) is not int or not 0<=uncertainty<ALIGN_NS:
        raise ValueError('Clock uncertainty must fit existing 5ms alignment budget')
    topics=c['topics']
    if set(topics)!={'camera','imu','range'} or len(set(topics.values()))!=3:
        raise ValueError('Explicit independent camera/IMU/range topics required')
    for topic in topics.values():
        if (not isinstance(topic,str) or not topic.startswith('/') or any(s in topic.lower() for s in
                ('gazebo','world/','mavros','ground_truth','/clock','/vio/')) or topic=='/' or ' ' in topic):
            raise ValueError('Only explicit physical sensor adapter topics allowed')
    frames=c['frames']
    if set(frames)!={'camera','imu','range'} or not all(isinstance(v,str) and v and not v.startswith('/') for v in frames.values()):
        raise ValueError('Explicit calibrated sensor frame IDs required')
    if c.get('acceleration')!='specific-force-m/s2' or c.get('angular_velocity')!='rad/s' or c.get('attitude')!='imu-to-ENU':
        raise ValueError('Require measured specific force, rad/s and independent ENU attitude')
    rotation(c['R_body_imu'])
    rc=rotation(c['R_body_camera'])
    if np.max(np.degrees(np.arccos(np.clip(np.sum(rc*CAMERA_FRD,axis=0),-1,1))))>1:
        raise ValueError('Camera optical axes incompatible with FlowFrontend (1 degree mount limit)')
    finite(c['camera_lever_frd_m'],(3,));finite(c['range_lever_frd_m'],(3,))
    if not np.allclose(finite(c['range_axis_frd'],(3,)),[0,0,1],rtol=0,atol=1e-6):
        raise ValueError('Range adapter requires body-down slant range')
    if c.get('surface')!='locally-static-horizontal-plane':raise ValueError('Range scale needs a verified local ground plane')
    camera=c['camera'];k=finite(camera['K'],(3,3));finite(camera['D'],(5,))
    if (camera.get('distortion_model')!='plumb_bob' or k[0,0]<=0 or k[1,1]<=0 or
            not np.array_equal(k[2],[0,0,1]) or k[0,1]!=0 or k[1,0]!=0 or
            type(camera['width']) is not int or type(camera['height']) is not int or min(camera['width'],camera['height'])<=0):
        raise ValueError('Require calibrated pinhole/plumb_bob camera dimensions')
    for key in ('range_sigma_m','max_range_age_s','max_tilt_deg'):
        if type(c[key]) not in (int,float) or not math.isfinite(c[key]) or c[key]<=0:raise ValueError('Missing sensor contract: '+key)
    if c['max_tilt_deg']>60:raise ValueError('Frontend depth clamp is not a valid high-tilt model')
    if c['max_range_age_s']>.18:raise ValueError('Range age contract must fit the existing180ms publication limit')
    return c


def host_admission(machine=None,system=None,python=None,ros=None):
    import os
    if (machine or platform.machine()).lower() not in ('aarch64','arm64') or (system or platform.system())!='Linux':
        raise ValueError('Hardware subscription requires Linux ARM64')
    if tuple(python or sys.version_info[:2])!=(3,12) or (ros or os.environ.get('ROS_DISTRO'))!='jazzy':
        raise ValueError('Supported native bring-up: ROS2 Jazzy with system Python3.12')


def load_vio(directory):
    directory=Path(directory).resolve()
    for name,digest in FILES.items():
        if hashlib.sha256((directory/name).read_bytes()).hexdigest()!=digest:raise ValueError('Unpinned VIO source: '+name)
    # Fresh process only. Reject a preimported module from another source tree.
    for name in ('vio_core','vio_frontend','vio_ros_source','vio_ros_out'):
        if name in sys.modules and Path(sys.modules[name].__file__).resolve()!=directory/(name+'.py'):
            raise ValueError('VIO import provenance conflict')
    sys.path.insert(0,str(directory))
    import vio_core, vio_frontend, vio_ros_source, vio_ros_out
    return vio_core, vio_frontend, vio_ros_source, vio_ros_out


def imu_measurement(c,force,gyro,q,force_cov,gyro_cov,att_cov):
    b=rotation(c['R_body_imu']);r=ENU_NED@quaternion(q)@b.T
    # VioEKF.predict adds NED gravity itself. Never subtract gravity here.
    return dict(f=b@finite(force,(3,)),gyro=b@finite(gyro,(3,)),r=r,
                force_cov=b@covariance(force_cov)@b.T,
                gyro_cov=b@covariance(gyro_cov)@b.T,att_cov=covariance(att_cov))


def range_heights(c,slant,min_range,max_range,r):
    if not all(math.isfinite(x) for x in (slant,min_range,max_range)) or not 0<min_range<slant<max_range:
        raise ValueError('Invalid/out-of-range physical range')
    tilt=math.degrees(math.acos(float(np.clip(r[2,2],-1,1))))
    if tilt>c['max_tilt_deg']:raise ValueError('Tilt exceeds physical range contract')
    root=slant*r[2,2]+float((r@np.asarray(c['range_lever_frd_m']))[2])
    # Lever in FRD points DOWN. Sensor closer to ground -> smaller height.
    camera=root-float((r@np.asarray(c['camera_lever_frd_m']))[2])
    if root<=0 or camera<=0:raise ValueError('No positive measured camera height')
    return root,camera


class HardwareVio:
    """Timestamp-ordered causal IMU integration; no invented samples or GT input.

    Critical input failures latch until process restart/new reference. Images may
    be unavailable/low-contrast: no publication, readiness must pass again.
    """
    def __init__(self,c,core,frontend):
        self.c=c;self.ekf=core.VioEKF(sigma_z=c['range_sigma_m'])
        self.frontend=frontend.FlowFrontend(c['camera']['K'][0][0])
        self.gate=QualityGate();self.imu=deque();self.ranges=deque();self.attitudes=deque(maxlen=8192)
        self.last_ns={};self.receipts={};self.failure=None;self.started=None
        self.last_imu=None;self.filter_ns=None;self.reference_ns=None
        self.previous=None;self.frame_ns=None;self.last_range_ns=None;self.initial_height=None
        self.map=None;self.counts=dict(imu=0,range=0,camera=0,estimates=0,ready=0)
        self.maximum_imu_gap_ns=0

    def fail(self,reason):
        self.failure=self.failure or str(reason);self.gate.ready=False
        raise ValueError(self.failure)

    def accept(self,kind,ns,arrival_ns):
        if self.failure:self.fail(self.failure)
        if type(ns) is not int or ns<=0 or type(arrival_ns) is not int or arrival_ns<0:self.fail('Invalid integer timestamp')
        if kind in self.last_ns and ns<=self.last_ns[kind]:self.fail(kind+' timestamp reset/duplicate')
        if kind=='imu' and kind in self.last_ns:
            gap=ns-self.last_ns[kind]
            if gap>IMU_GAP_NS:self.fail('IMU gap >20ms')
            self.maximum_imu_gap_ns=max(self.maximum_imu_gap_ns,gap)
        self.last_ns[kind]=ns;self.receipts[kind]=arrival_ns;self.counts[kind]+=1
        if self.started is None:self.started=arrival_ns

    def add_imu(self,ns,arrival_ns,**data):
        self.accept('imu',ns,arrival_ns)
        if len(self.imu)>=8192:self.fail('IMU buffer overflow')
        try:m=imu_measurement(self.c,**data)
        except ValueError as exc:self.fail(exc)
        self.imu.append(dict(ns=ns,**m))
        self.attitudes.append(dict(ns=ns,r=m['r']))

    def add_range(self,ns,arrival_ns,slant,min_range,max_range):
        self.accept('range',ns,arrival_ns)
        if len(self.ranges)>=128:self.fail('Range buffer overflow')
        if not all(math.isfinite(v) for v in (slant,min_range,max_range)) or not 0<min_range<slant<max_range:
            self.fail('Invalid/out-of-range physical range')
        self.ranges.append(dict(ns=ns,slant=slant,min_range=min_range,max_range=max_range))

    def check_alive(self,now):
        if self.failure:self.fail(self.failure)
        if self.started is None:self.started=now
        for kind in ('imu','range','camera'):
            receipt=self.receipts.get(kind,self.started)
            if not 0<=now-receipt<=WATCHDOG_NS:self.fail('Missing/stale '+kind+' receipt >500ms')

    def rectified(self,gray):
        import cv2
        camera=self.c['camera'];h,w=camera['height'],camera['width']
        if gray.dtype!=np.uint8 or gray.shape!=(h,w):self.fail('Camera dimensions/encoding differ from calibration')
        if self.map is None:
            fx=camera['K'][0][0];out=np.array([[fx,0,w/2],[0,fx,h/2],[0,0,1.]])
            self.map=cv2.initUndistortRectifyMap(np.asarray(camera['K'],float),np.asarray(camera['D'],float),
                np.eye(3),out,(w,h),cv2.CV_32FC1)
        return cv2.remap(gray,*self.map,cv2.INTER_LINEAR)

    def process(self,gray,ns,arrival_ns,now_system_ns,now_mono_ns):
        self.accept('camera',ns,arrival_ns);self.check_alive(now_mono_ns)
        uncertainty=self.c['clock_uncertainty_ns']
        if not 0<=now_system_ns-ns or now_system_ns-ns+uncertainty>PUBLICATION_NS:self.fail('Stale/future camera acquisition')
        if self.frame_ns is not None and ns-self.frame_ns>FRAME_GAP_NS:self.fail('Camera gap >300ms')
        if not self.imu or self.imu[-1]['ns']<ns:self.fail('Missing IMU acquisition watermark')
        if self.last_imu is None:
            causal=[m for m in self.imu if m['ns']<=ns]
            if not causal:self.fail('No causal IMU')
            self.reference_ns=ns;self.filter_ns=ns;self.last_imu=causal[-1]
            # Before the reference exists, retain only the latest causal range
            # for initial height. Older startup ranges are not filter updates.
            while len(self.ranges)>=2 and self.ranges[1]['ns']<=ns:self.ranges.popleft()
        weighted=np.zeros(3);weight=0
        def propagate(target):
            nonlocal weighted,weight
            if target<=self.filter_ns:return
            dt=(target-self.filter_ns)*1e-9;m=self.last_imu
            self.ekf.sigma_acc=max(self.ekf.sigma_acc,float(np.sqrt(np.linalg.eigvalsh(m['force_cov']).max())))
            self.ekf.predict(m['f'],m['r'],dt)
            weighted+=dt*m['gyro'];weight+=dt;self.filter_ns=target
        # Merge causal IMU and range events at their actual acquisition times.
        # Do not fuse an old range measurement as if it were acquired now.
        while (self.imu and self.imu[0]['ns']<=ns) or (self.ranges and self.ranges[0]['ns']<=ns):
            imu_ns=self.imu[0]['ns'] if self.imu else math.inf
            range_ns=self.ranges[0]['ns'] if self.ranges else math.inf
            if imu_ns<=range_ns:
                m=self.imu.popleft();propagate(m['ns']);self.last_imu=m
            else:
                height=self.ranges.popleft()
                aligned=next((a for a in reversed(self.attitudes) if a['ns']<=height['ns']),None)
                if aligned is None or height['ns']-aligned['ns']+uncertainty>ALIGN_NS:self.fail('Range acquisition attitude unavailable/stale')
                root,_=range_heights(self.c,height['slant'],height['min_range'],height['max_range'],aligned['r'])
                if self.initial_height is not None and height['ns']<self.filter_ns:self.fail('Late range behind estimator time; no rewind')
                propagate(height['ns'])
                if self.initial_height is None:
                    self.initial_height=root;self.ekf.x[2]=0.
                else:self.ekf.update_height(-(root-self.initial_height),sigma=self.c['range_sigma_m'])
                self.last_range_ns=height['ns'];self.height=dict(height,root=root)
        m=self.last_imu
        if ns-m['ns']+uncertainty>ALIGN_NS:self.fail('IMU acquisition alignment exceeds5ms')
        propagate(ns)
        gyro=weighted/weight if weight else m['gyro'];r=m['r']
        height=getattr(self,'height',None)
        if height is None or not 0<=ns-height['ns']+uncertainty<=self.c['max_range_age_s']*1e9:self.fail('Missing/stale timestamped range')
        root=height['root']
        camera=root-float((r@np.asarray(self.c['camera_lever_frd_m']))[2])
        if camera<=0:self.fail('Nonpositive measured camera height')
        gray=self.rectified(gray);t=(ns-self.reference_ns)*1e-9
        result=self.frontend.process(gray,t,camera,gyro,r)
        audit={'ok':False,'reason':'frontend unavailable'};accepted=False;nis=None
        if result is not None:
            audit=distributed_flow(self.previous,gray,(ns-self.frame_ns)*1e-9,camera,self.frontend.fx,gyro,r,result.v_xy)
            if audit['ok']:
                velocity=camera_to_body_xy(result.v_xy,r,gyro,self.c['camera_lever_frd_m'])
                accepted,nis=self.ekf.update_velocity_xy(velocity,result.sigma_scale)
                self.counts['estimates']+=1
        gap=.1 if self.frame_ns is None else (ns-self.frame_ns)*1e-9
        self.previous=gray;self.frame_ns=ns
        ready=self.gate.assess(t,now_mono_ns*1e-9,imu_age=(ns-m['ns']+uncertainty)*1e-9,
            frame_age=(now_system_ns-ns+uncertainty)*1e-9,clock_age=(now_system_ns-ns+uncertainty)*1e-9,
            max_imu_gap=self.maximum_imu_gap_ns*1e-9,frame_gap=gap,inliers=result.n_inliers if result else 0,
            contrast=float(gray.std()),covariance=self.ekf.P,state_finite=bool(np.isfinite(self.ekf.x).all()),
            audit_ok=bool(accepted and audit['ok']),ekf_ok=ekf_ready(self.ekf))
        if self.ekf.n_resets:self.fail('Estimator reset; explicit reference restart required')
        if ready:self.counts['ready']+=1
        return dict(stamp_ns=ns,ready=ready,r=r,gyro=m['gyro'],gyro_cov=m['gyro_cov'],
            att_var=float(np.linalg.eigvalsh(m['att_cov']).max()),p=self.ekf.x[:3].copy(),
            v=self.ekf.x[3:6].copy(),P_pos=self.ekf.P[:3,:3].copy(),P_vel=self.ekf.P[3:6,3:6].copy(),
            nis=nis,inliers=result.n_inliers if result else 0,audit=audit,
            height_m=root,camera_height_m=camera,range_age_ns=ns-height['ns'],
            max_imu_gap_ns=self.maximum_imu_gap_ns,clock_uncertainty_ns=uncertainty)


def load_config(path):
    path=Path(path)
    c=json.loads(path.read_text(),parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
    return validate_config(c,path.parent)


def check_publication(row,now_system_ns):
    if (row['ready'] is not True or type(row['stamp_ns']) is not int or
            not 0<=now_system_ns-row['stamp_ns'] or
            now_system_ns-row['stamp_ns']+row['clock_uncertainty_ns']>PUBLICATION_NS):
        raise ValueError('Publication gate closed/stale/future acquisition')
    rotation(row['r'])
    for key,shape in (('p',(3,)),('v',(3,)),('gyro',(3,))):finite(row[key],shape)
    for key in ('P_pos','P_vel','gyro_cov'):covariance(row[key])
    if not math.isfinite(row['att_var']) or row['att_var']<=0:raise ValueError('Unknown attitude covariance')
