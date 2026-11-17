"""Physical ROS sensor adapters -> pinned CPU VIO; isolated candidate output only.

Default is offline config/dependency validation. --receive opts into sensor
subscriptions, never FCU/MAVROS. Candidate Odometry additionally needs --publish.
No device paths, drivers, Gazebo, CUDA, Docker or automatic reference recovery.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import threading
import time
import uuid

from pi5_vio import (HardwareVio,check_publication,host_admission,load_config,
                     load_vio,stamp_ns,WATCHDOG_NS)
from vio_adapter import validate_covariance

TOPIC='/integration/pi5/odom_candidate'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--vio-dir',type=Path,required=True)
    p.add_argument('--receive',action='store_true');p.add_argument('--publish',action='store_true')
    p.add_argument('--duration',type=float,default=30.)
    p.add_argument('--output',type=Path,help='New JSONL file required for bounded receive')
    a=p.parse_args()
    if a.publish and not a.receive:p.error('--publish requires --receive')
    if not 0<a.duration<=300:p.error('Finite bounded duration required (0,300]s')
    c=load_config(a.config);core,frontend,images,out=load_vio(a.vio_dir)
    if not a.receive:
        print(json.dumps(dict(status='OFFLINE_CONFIG_PASS',hardware='NOT TESTED',
            execution_allowed=False,forwarding_enabled=False)));return
    host_admission()
    if a.output is None:p.error('--output NEW.jsonl required')
    import numpy as np
    import cv2
    cv2.ocl.setUseOpenCL(False)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import Image,Imu,Range

    log=a.output.open('x');session=str(uuid.uuid4())
    def record(**row):
        log.write(json.dumps(dict(session=session,host_ns=time.monotonic_ns(),execution_allowed=False,
            forwarding_enabled=False,**row),allow_nan=False,default=lambda v:v.tolist())+'\n');log.flush()
    rclpy.init();node=rclpy.create_node('pi5_cpu_vio_bringup')
    if node.get_parameter('use_sim_time').value:
        node.destroy_node();rclpy.shutdown();log.close()
        raise ValueError('Physical hardware must not consume simulation /clock')
    executor=SingleThreadedExecutor();executor.add_node(node)
    queues={k:deque() for k in ('imu','range','camera')};capacities=dict(imu=8192,range=128,camera=2)
    lock=threading.Lock();errors=[];camera_drops=[0];last_received={};last_stamp={}
    engine=HardwareVio(c,core,frontend);row=None
    # Receiver callbacks only validate framing/stamps and enqueue raw messages.
    # Conversion, EKF and image audit run outside this lock/executor thread.
    def callback(kind,msg):
        try:
            ns=stamp_ns(msg.header.stamp);arrival=time.monotonic_ns()
            if msg.header.frame_id!=c['frames'][kind]:raise ValueError(kind+' frame differs from calibrated adapter')
            now=node.get_clock().now().nanoseconds
            if not 0<=now-ns or now-ns+c['clock_uncertainty_ns']>180_000_000:
                raise ValueError(kind+' stamp not fresh in verified system time')
            with lock:
                if errors:return
                if kind in last_stamp and ns<=last_stamp[kind]:raise ValueError(kind+' acquisition reset/duplicate')
                last_stamp[kind]=ns;last_received[kind]=arrival
                if len(queues[kind])>=capacities[kind]:
                    if kind!='camera':raise ValueError(kind+' receive buffer overflow')
                    queues[kind].popleft();camera_drops[0]+=1
                queues[kind].append((ns,arrival,msg))
        except Exception as exc:
            with lock:errors.append(str(exc))
    for kind,typ in (('camera',Image),('imu',Imu),('range',Range)):
        node.create_subscription(typ,c['topics'][kind],lambda msg,k=kind:callback(k,msg),qos_profile_sensor_data)

    class OutputNode:
        def get_clock(self):
            class Clock:
                def now(_):return Time(nanoseconds=row['stamp_ns'])
            return Clock()
        def create_publisher(self,typ,topic,qos):
            real=node.create_publisher(typ,topic,qos)
            if topic!=TOPIC:return real
            class Checked:
                def publish(_,msg):
                    with lock:
                        if errors:raise ValueError(errors[0])
                        if any(not 0<=time.monotonic_ns()-last_received.get(k,0)<=WATCHDOG_NS for k in queues):
                            raise ValueError('Physical adapter receipt watchdog')
                    check_publication(row,node.get_clock().now().nanoseconds)
                    if (stamp_ns(msg.header.stamp)!=row['stamp_ns'] or msg.header.frame_id!='pi5_odom_ned' or
                            msg.child_frame_id!='pi5_imu_frd'):raise ValueError('Output stamp/frame mismatch')
                    angular=msg.twist.twist.angular
                    angular.x,angular.y,angular.z=map(float,row['gyro'])
                    for i in range(3):
                        for j in range(3):msg.twist.covariance[(i+3)*6+j+3]=float(row['gyro_cov'][i,j])
                    validate_covariance(msg.pose.covariance);validate_covariance(msg.twist.covariance)
                    real.publish(msg)
            return Checked()
    publisher=out.RosOdomPublisher(node=OutputNode(),topic=TOPIC,status_topic='/integration/pi5/status',
        frame_id='pi5_odom_ned',child_frame_id='pi5_imu_frd') if a.publish else None
    def spin():
        try:executor.spin()
        except Exception as exc:
            with lock:errors.append('Receiver executor failed: '+str(exc))
    thread=threading.Thread(target=spin,daemon=True);thread.start()
    started=time.monotonic_ns();deadline=started+int(a.duration*1e9);pending=deque();verdict='FAIL'
    record(event='configuration',config=c,output_topic=TOPIC if a.publish else None,reference='local IMU-origin NED; not FCU reference',
           versions=dict(python=__import__('sys').version,numpy=np.__version__,opencv=__import__('cv2').__version__,ros='jazzy'))
    try:
        while time.monotonic_ns()<deadline:
            with lock:
                if errors:raise ValueError(errors[0])
                batch={k:list(v) for k,v in queues.items()}
                for queue in queues.values():queue.clear()
                depths={k:len(v) for k,v in batch.items()}
                receipts=dict(last_received)
            now=time.monotonic_ns()
            for k in queues:
                if not 0<=now-receipts.get(k,started)<=WATCHDOG_NS:raise ValueError('Missing/stale '+k+' adapter >500ms')
            for ns,arrival,msg in batch['imu']:
                vec=lambda v:[v.x,v.y,v.z]
                engine.add_imu(ns,arrival,force=vec(msg.linear_acceleration),gyro=vec(msg.angular_velocity),
                    q=[msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w],
                    force_cov=np.asarray(msg.linear_acceleration_covariance).reshape(3,3),
                    gyro_cov=np.asarray(msg.angular_velocity_covariance).reshape(3,3),
                    att_cov=np.asarray(msg.orientation_covariance).reshape(3,3))
            for ns,arrival,msg in batch['range']:engine.add_range(ns,arrival,msg.range,msg.min_range,msg.max_range)
            pending.extend(batch['camera'])
            while len(pending)>2:pending.popleft();camera_drops[0]+=1
            if pending and engine.imu and engine.imu[-1]['ns']>=pending[0][0]:
                ns,arrival,msg=pending.popleft();before=time.monotonic_ns();cpu=time.thread_time_ns()
                if engine.reference_ns is None and (engine.imu[0]['ns']>ns or
                        not any(v['ns']<=ns for v in engine.ranges)):
                    record(event='startup_frame_rejected',stamp_ns=ns,reason='No causal IMU/range overlap yet')
                    continue
                gray=images.image_to_gray(msg)
                if gray is None:raise ValueError('Unsupported physical image encoding')
                row=engine.process(gray,ns,arrival,node.get_clock().now().nanoseconds,before)
                if row['ready']:check_publication(row,node.get_clock().now().nanoseconds)
                published=False
                if row['ready'] and publisher:
                    publisher.publish(row['r'],row['p'],row['v'],row['P_pos'],row['P_vel'],row['att_var']);published=True
                record(event='frame',**row,published=published,queue_depths=depths,receipt_ns=arrival,
                       processing_wall_ns=time.monotonic_ns()-before,processing_cpu_ns=time.thread_time_ns()-cpu,
                       camera_drops=camera_drops[0])
            if pending and node.get_clock().now().nanoseconds-pending[0][0]>180_000_000:
                raise ValueError('Camera waiting for missing acquisition watermark >180ms')
            if not thread.is_alive():raise ValueError('Receiver thread stopped')
            time.sleep(.001)
        if not engine.counts['ready']:raise ValueError('No ready estimates: missing inputs/texture/calibration')
        verdict='BOUNDED_BRINGUP_COMPLETE'  # Never hardware navigation/flight acceptance.
    except BaseException as exc:
        record(event='failed',reason=str(exc));raise
    finally:
        executor.shutdown(timeout_sec=2);thread.join(timeout=2)
        record(event='summary',verdict=verdict,counts=engine.counts,camera_drops=camera_drops[0],
               failure=engine.failure,receiver_joined=not thread.is_alive())
        node.destroy_node();rclpy.shutdown();log.close()
        if thread.is_alive():raise RuntimeError('Receiver cleanup incomplete')


if __name__=='__main__':main()
