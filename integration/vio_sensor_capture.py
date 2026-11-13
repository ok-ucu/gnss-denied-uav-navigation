"""Dedicated ROS IMU executor plus independent native IMU witness. No output VIO."""
from collections import deque
import os
import signal
import subprocess
import threading
import time
from stage_timing import sample,stage,mark,critical_stage,trace_span,FrameQueueLock


class BoundedRecords:
    def __init__(self, capacity, label=None):
        if capacity <= 0:
            raise ValueError('capacity must be positive')
        self.items = deque(maxlen=capacity)
        self.dropped = 0
        self.lock = threading.Lock()
        self.label=label
        self.check_health = None

    def append(self, item):
        with self.lock:
            if len(self.items) == self.items.maxlen:
                self.dropped += 1
            self.items.append(item)

    def snapshot(self):
        if self.check_health is not None:
            self.check_health()
        with critical_stage('source_buffer_snapshot',buffer=self.label) as metrics:
            before=time.monotonic_ns() if metrics else None
            with self.lock:
                if metrics:metrics.update(lock_wait_ns=time.monotonic_ns()-before,size=len(self.items))
                return list(self.items)

    def latest(self):
        if self.check_health is not None:
            self.check_health()
        with self.lock:
            return self.items[-1] if self.items else None


def recovery_ready(samples, now, not_before, stable_span=.25, max_gap=.02):
    import math
    if not all(math.isfinite(v) for v in (now,not_before,stable_span,max_gap)) or stable_span<=0 or max_gap<=0:
        raise ValueError('Invalid recovery timing')
    if now<not_before:
        return False
    recent=[m['t'] for m in samples if now-stable_span-max_gap<=m['t']<=now]
    return (len(recent)>=2 and recent[-1]-recent[0]>=stable_span-1e-9
            and now-recent[-1]<=.005
            and all(0<b-a<=max_gap+1e-9 for a,b in zip(recent,recent[1:])))


def capture(camera, imu_topic, duration, output_dir, frame_handler=None, tick_handler=None,
            profile=False, async_diagnostics=False, reliable_camera=False, isolated_imu=False):
    import cv2
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Imu, Image
    from rosgraph_msgs.msg import Clock
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.imu_pb2 import IMU
    from vio_ros_source import RosFrameSource
    cv2.setNumThreads(1)
    buffers = {key: BoundedRecords(size,key) for key, size in
               [('imu',120000),('native_imu',120000),('poses',20000),('clocks',120000),('frames',1500)]}
    dedicated_topic='/integration/vio_imu'
    bridge_config=output_dir/'sensor_bridge.yaml'
    bridge_config.write_text(f"""- ros_topic_name: {dedicated_topic}
  gz_topic_name: {imu_topic}
  ros_type_name: sensor_msgs/msg/Imu
  gz_type_name: gz.msgs.IMU
  direction: GZ_TO_ROS
  publisher_queue: 4096
  subscriber_queue: 4096
- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
  publisher_queue: 4096
  subscriber_queue: 4096
""")
    bridge_log=(output_dir/'clock_bridge.log').open('x')
    bridge=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',
                              '--ros-args','-r','__node:=vio_sensor_bridge','-p','config_file:='+str(bridge_config)],
                             stdout=bridge_log,stderr=subprocess.STDOUT,start_new_session=True)
    source=None; receiver=None; executor=None; thread=None; isolated=None
    load_records=[]; next_load=0.; endpoints=[];camera_endpoints=[];clock_endpoints=[];clock_graph=[]
    buffers['clocks'].publishers=[]
    native=Node()
    camera_records=BoundedRecords(10000)
    native_camera_records=BoundedRecords(10000)
    class MeasuredSource(RosFrameSource):
        def _cb(self, msg):
            begin=time.monotonic()
            ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            mark('camera_callback_entry',source_ns=ns)
            with sample(ns),critical_stage('source_camera_callback'),stage('camera_callback'):
                super()._cb(msg)
            camera_records.append(dict(t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                                       arrival_monotonic_s=begin,callback_end_monotonic_s=time.monotonic()))
    class ReliableSource(MeasuredSource if profile else RosFrameSource):
        def __init__(self,topic,node_name='vio_camera_capture',use_sim_time=True):
            # RosFrameSource starts spin() inside its constructor. Destroying its
            # subscription afterwards races the active wait-set (InvalidHandle).
            # Construct our ROS endpoint before starting its OWN executor; reuse
            # teammate _cb/image conversion/poll unchanged. No global monkeypatch.
            from rclpy.parameter import Parameter
            self._rclpy=rclpy
            if not rclpy.ok():rclpy.init()
            overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,use_sim_time)]
            self.node=rclpy.create_node(node_name,parameter_overrides=overrides)
            self._latest=None
            self._lock=FrameQueueLock(self) if profile else threading.Lock()
            self.n_frames=self.n_unsupported=self.n_bad_age=0
            self.node.create_subscription(Image,topic,self._cb,
                QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE))
            self._executor=SingleThreadedExecutor();self._executor.add_node(self.node)
            self._thread=threading.Thread(target=self._executor.spin,daemon=True)
            self._thread.start()

        def close(self):
            self._executor.shutdown();self._thread.join(timeout=3)
            self.node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
    if profile:
        from gz.msgs10.image_pb2 import Image as NativeImage
        def camera_cb(msg):
            mark('native_camera_arrival',source_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nsec)
            native_camera_records.append(dict(t=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,
                                               arrival_monotonic_s=time.monotonic()))
        native.subscribe(NativeImage,camera,camera_cb)
    sampler_stop=threading.Event();sampler_thread=None;stack_thread=None
    def sample_load():
        begin=time.monotonic();ticks=[]
        from pathlib import Path
        for entry in Path('/proc').iterdir():
            if entry.name.isdigit():
                try:
                    fields=(entry/'stat').read_text().rsplit(')',1)[1].split()
                    ticks.append({'pid':int(entry.name),'utime':int(fields[11]),'stime':int(fields[12]),'rss_pages':int(fields[21])})
                except (OSError,ValueError,IndexError):pass
        proc=subprocess.run(['ps','-eo','pid,pcpu,pmem,rss,args'],capture_output=True,text=True,timeout=3)
        load_records.append(dict(cpu_ticks=ticks,clock_ticks_per_second=os.sysconf('SC_CLK_TCK'),
                                 arrival_monotonic_s=begin,processes=proc.stdout,
                                 sampling_wall_s=time.monotonic()-begin))
    def background_load():
        while not sampler_stop.is_set():
            try:sample_load()
            except (OSError,subprocess.TimeoutExpired) as exc:
                load_records.append(dict(error=str(exc),arrival_monotonic_s=time.monotonic()))
            sampler_stop.wait(1.)

    def consumer_stacks(tid):
        import sys
        while not sampler_stop.wait(.02):
            frame=sys._current_frames().get(tid)
            stack=[]
            for _ in range(10):
                if frame is None:break
                stack.append([frame.f_code.co_filename.rsplit('/',1)[-1],frame.f_code.co_name,frame.f_lineno])
                frame=frame.f_back
            mark('source_consumer_stack',stack=stack)

    def q(q):return [q.x,q.y,q.z,q.w]
    def vec(v):return [v.x,v.y,v.z]
    def pose_cb(msg):
        buffers['poses'].append({'t':msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,
                                'arrival_monotonic_s':time.monotonic(),
                                'poses':[{'name':p.name,'id':p.id,'p':vec(p.position),
                                          'q':q(p.orientation)} for p in msg.pose]})
    def native_cb(msg):
        buffers['native_imu'].append({'t':msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,
                                      'arrival_monotonic_s':time.monotonic(),
                                      'f':vec(msg.linear_acceleration),
                                      'gyro':vec(msg.angular_velocity),'q':q(msg.orientation)})
    def imu_cb(msg):
        start=time.monotonic()
        cpu_start=time.thread_time_ns() if profile else None
        row={'t':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
             'arrival_monotonic_s':start,'frame':msg.header.frame_id,
             'f':vec(msg.linear_acceleration),'gyro':vec(msg.angular_velocity),'q':q(msg.orientation)}
        row['callback_build_s']=time.monotonic()-start
        buffers['imu'].append(row)
        if profile:
            row.update(callback_end_host_ns=time.monotonic_ns(),callback_thread_cpu_ns=time.thread_time_ns()-cpu_start,
                       receipt_thread_id=threading.get_native_id())
    def clock_cb(msg):
        buffers['clocks'].append({'t':msg.clock.sec+msg.clock.nanosec*1e-9,
                                  'arrival_monotonic_s':time.monotonic()})
    native.subscribe(Pose_V,'/world/iris_runway/dynamic_pose/info',pose_cb)
    native.subscribe(IMU,imu_topic,native_cb)
    try:
        source=(ReliableSource if reliable_camera else MeasuredSource if profile else RosFrameSource)(camera,node_name='vio_camera_capture',use_sim_time=True)
        receiver=rclpy.create_node('vio_dedicated_imu_receiver')
        qos=QoSProfile(depth=4096,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)
        if isolated_imu:
            from isolated_imu import IsolatedImuReceiver
            isolated=IsolatedImuReceiver(buffers,dedicated_topic,profile)
            isolated.start()
            for name in ('imu','clocks'):
                buffers[name].check_health=isolated.check
        else:
            sub=receiver.create_subscription(Imu,dedicated_topic,imu_cb,qos)
            clock_sub=receiver.create_subscription(Clock,'/clock',clock_cb,qos)
            executor=SingleThreadedExecutor();executor.add_node(receiver)
            thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        if async_diagnostics:
            sampler_thread=threading.Thread(target=background_load,daemon=True);sampler_thread.start()
        if profile:
            stack_thread=threading.Thread(target=consumer_stacks,args=(threading.get_ident(),),daemon=True)
            stack_thread.start()
        start=time.monotonic()
        mark('sensor_capture_started')
        while time.monotonic()-start<duration:
            with trace_span('source_poll') as poll_trace:
                frame=source.poll()
                if frame is not None:poll_trace['source_ns']=round(frame[1]*1e9)
            if frame is not None:
                buffers['frames'].append(frame)
                if frame_handler is not None:
                    frame_handler(frame, buffers)
            if tick_handler is not None:
                with trace_span('source_tick'):
                    tick_handler(buffers)
            if time.monotonic()>=next_load:
                observed_clock=[dict(node=e.node_name,namespace=e.node_namespace,
                    gid=bytes(e.endpoint_gid).hex(),qos=str(e.qos_profile))
                    for e in receiver.get_publishers_info_by_topic('/clock')]
                if observed_clock!=clock_endpoints:
                    clock_endpoints=observed_clock
                    clock_graph.append(dict(host_monotonic_s=time.monotonic(),publishers=clock_endpoints))
                    buffers['clocks'].publishers=clock_endpoints
                if not endpoints:
                    endpoints=[{'node':e.node_name,'qos':str(e.qos_profile),'depth':e.qos_profile.depth,
                                'reliability':str(e.qos_profile.reliability)}
                               for e in receiver.get_publishers_info_by_topic(dedicated_topic)]
                if not camera_endpoints:
                    camera_endpoints=[dict(node=e.node_name,reliability=str(e.qos_profile.reliability))
                                      for e in receiver.get_publishers_info_by_topic(camera)]
                if not async_diagnostics:sample_load()
                next_load=time.monotonic()+1.
            with trace_span('source_loop_sleep',slow_ns=10_000_000):
                time.sleep(.002)
    finally:
        mark('sensor_capture_finished')
        shutdown_error=None
        sampler_stop.set()
        if sampler_thread:sampler_thread.join(timeout=5)
        if stack_thread:stack_thread.join(timeout=2)
        if isolated is not None:
            try:
                isolated.close()
            except RuntimeError as exc:
                # Still stop the owned ROS/bridge resources below. The failed
                # worker session remains unhealthy and cannot publish again.
                isolated.reason=str(exc)
                isolated.fault.set()
                shutdown_error=exc
            finally:
                # Capture has finished: evidence export only.
                for name in ('imu','clocks'):
                    buffers[name].check_health=None
        if executor:executor.shutdown()
        if thread:thread.join(timeout=3)
        if receiver:receiver.destroy_node()
        if source:source.close()
        os.killpg(bridge.pid,signal.SIGTERM);bridge.wait(timeout=5);bridge_log.close()
        if shutdown_error is not None:
            with (output_dir/'isolated_shutdown_error.txt').open('x') as error:
                error.write(str(shutdown_error)+'\n')
            raise shutdown_error
    import json
    (output_dir/'process_load.json').write_text(json.dumps(load_records,indent=2)+'\n')
    if profile:
        for name,buffer in [('ros_camera',camera_records),('native_camera',native_camera_records)]:
            with (output_dir/(name+'.jsonl')).open('x') as f:
                for row in buffer.snapshot():f.write(json.dumps(row)+'\n')
    result={name:buffer.snapshot() for name,buffer in buffers.items()}
    result['buffer_overflows']={name:buffer.dropped for name,buffer in buffers.items()}
    result['qos']={'depth':4096,'reliability':'RELIABLE','publisher_endpoints':endpoints,'imu_topic':dedicated_topic,'publisher_queue':4096,'subscriber_queue':4096,'executor':'dedicated IMU+clock, separate from image conversion'}
    result['qos']['isolated_imu_process']=isolated_imu
    result['qos']['clock_publishers']=clock_graph
    result['isolated_imu']=None if isolated is None else dict(
        worker_pid=isolated.process.pid,batches=isolated.sequence,rows=isolated.row_counts,
        failure=isolated.reason,worker_fault=isolated.fault.is_set(),
        produced_batches=isolated.produced.value,
        queued_unconsumed_batches=max(0,isolated.produced.value-isolated.sequence),
        shutdown_tail_flushed=False,
        queue_batch_bound=32,flush_target_s=.003,delivery_bound_established=False,
        ipc_metrics=list(isolated.metrics),metric_overflows=isolated.metric_overflows,
        worker_stopped=not isolated.process.is_alive())
    result['camera_profile']=dict(enabled=profile,ros_callbacks=source.n_frames,
        unsupported=source.n_unsupported,bad_age=source.n_bad_age,
        reader_reliability='RELIABLE' if reliable_camera else 'BEST_EFFORT',reader_depth=32 if reliable_camera else 5,
        publisher_endpoints=camera_endpoints,
        ros_record_overflows=camera_records.dropped,native_record_overflows=native_camera_records.dropped)
    return result
