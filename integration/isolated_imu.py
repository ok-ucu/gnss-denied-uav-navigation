"""Bounded, ordered ROS IMU/clock ingestion in a spawned interpreter.

Sensor acquisition and callback receipt timestamps are never replaced by IPC
receipt time. Queue overflow, worker exit and protocol errors latch unhealthy.
This module imports ROS only inside the real worker; offline tests need no ROS.
"""
import math
import multiprocessing
import queue
import threading
import time
import uuid
from collections import deque

QUEUE_BATCHES = 32
MAX_BATCH_ROWS = 128


def isolated_imu_args(enabled):
    return ['--isolated-imu'] if enabled else []


def put_batch(output, fault, produced, packet):
    """No blocking producer I/O or silent drop when the consumer falls behind."""
    try:
        output.put_nowait(packet)
    except queue.Full:
        fault.set()
        return False
    with produced.get_lock():
        produced.value += 1
    return True


def ros_imu_clock_worker(output, stop, fault, produced, session, imu_topic, profile):
    node = executor = None
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import Imu
        from rosgraph_msgs.msg import Clock

        rclpy.init()
        node = rclpy.create_node('vio_isolated_imu_clock_receiver')
        qos = QoSProfile(depth=4096, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        pending = {'imu': [], 'clocks': []}

        def imu_cb(msg):
            arrived = time.monotonic()
            cpu = time.thread_time_ns() if profile else None
            a, g, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
            row = dict(t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                       stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                       arrival_monotonic_s=arrived, frame=msg.header.frame_id,
                       f=[a.x, a.y, a.z], gyro=[g.x, g.y, g.z], q=[q.x, q.y, q.z, q.w])
            row['callback_build_s'] = time.monotonic()-arrived
            if profile:
                row['callback_thread_cpu_ns'] = time.thread_time_ns()-cpu
            pending['imu'].append(row)

        def clock_cb(msg):
            pending['clocks'].append(dict(t=msg.clock.sec+msg.clock.nanosec*1e-9,
                stamp_ns=msg.clock.sec*10**9+msg.clock.nanosec,
                arrival_monotonic_s=time.monotonic()))

        node.create_subscription(Imu, imu_topic, imu_cb, qos)
        node.create_subscription(Clock, '/clock', clock_cb, qos)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        sequence = 0
        sent = time.monotonic()
        while not stop.is_set():
            executor.spin_once(timeout_sec=.002)
            now = time.monotonic()
            size = sum(map(len, pending.values()))
            # 3ms is a flush target, NOT a guaranteed delivery bound.
            if size and (now-sent >= .003 or size >= MAX_BATCH_ROWS):
                packet = dict(session=session, sequence=sequence, sent_monotonic_s=now, **pending)
                if not put_batch(output, fault, produced, packet):
                    break
                sequence += 1
                pending = {'imu': [], 'clocks': []}
                sent = now
    except BaseException:
        if not stop.is_set():
            fault.set()
        raise
    finally:
        # Shutdown never blocks trying to flush a full IPC channel. No unconsumed
        # tail is represented as an accepted sample or silently resumed later.
        if not stop.is_set():
            fault.set()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if 'rclpy' in locals() and rclpy.ok():
            rclpy.shutdown()
        output.cancel_join_thread()
        output.close()


class IsolatedImuReceiver:
    def __init__(self, buffers, imu_topic, profile=False, *, target=ros_imu_clock_worker):
        self.buffers = buffers
        self.session = str(uuid.uuid4())
        self.sequence = 0
        self.last = {}
        self.reason = None
        self.closed = False
        self.profile = profile
        self.metrics = deque(maxlen=20000)
        self.metric_overflows = 0
        self.row_counts = dict(imu=0, clocks=0)
        self.context = multiprocessing.get_context('spawn')
        self.output = self.context.Queue(maxsize=QUEUE_BATCHES)
        self.stop = self.context.Event()
        self.fault = self.context.Event()
        self.produced = self.context.Value('Q', 0)
        self.process = self.context.Process(target=target, args=(self.output, self.stop, self.fault,
            self.produced, self.session, imu_topic, profile), daemon=True)
        self.thread = None

    def start(self):
        self.process.start()
        self.thread = threading.Thread(target=self._drain, daemon=True)
        self.thread.start()

    def check(self):
        if self.reason or self.closed or self.fault.is_set() or not self.process.is_alive():
            self.reason = self.reason or 'isolated IMU worker failed/exited/overflowed'
            raise ValueError(self.reason)

    def accept(self, packet, received=None):
        if self.reason or self.fault.is_set() or self.closed:
            raise ValueError(self.reason or 'isolated IMU receiver closed/failed')
        received = time.monotonic() if received is None else received
        try:
            if packet['session'] != self.session or packet['sequence'] != self.sequence:
                raise ValueError('isolated IMU IPC sequence/session mismatch')
            sent = packet['sent_monotonic_s']
            if not all(math.isfinite(v) for v in (sent,received)) or min(sent,received)<0 or sent > received:
                raise ValueError('isolated IMU IPC receipt clock mismatch')
            if not 0 < len(packet['imu'])+len(packet['clocks']) <= MAX_BATCH_ROWS:
                raise ValueError('isolated IMU malformed/oversize batch')
            latest = dict(self.last)
            for kind in ('imu', 'clocks'):
                for row in packet[kind]:
                    t, arrived = row['t'], row['arrival_monotonic_s']
                    if not all(math.isfinite(v) for v in (t, arrived)) or min(t, arrived)<0 or arrived > sent:
                        raise ValueError('isolated IMU malformed timestamp')
                    if kind in latest:
                        previous_t, previous_arrival = latest[kind]
                        # /clock duplicates are retained for audit, not promoted
                        # into new acquisition labels. Reset/out-of-order is fatal.
                        if t < previous_t or (kind == 'imu' and t == previous_t) or arrived < previous_arrival:
                            raise ValueError('isolated IMU/clock reset or unordered samples')
                    latest[kind] = (t, arrived)
            for kind in ('imu', 'clocks'):
                for row in packet[kind]:
                    self.buffers[kind].append(dict(row, ipc_received_monotonic_s=received,
                                                  ipc_sequence=self.sequence))
                    self.row_counts[kind] += 1
            self.last = latest
            if self.profile:
                if len(self.metrics) == self.metrics.maxlen:
                    self.metric_overflows += 1
                self.metrics.append(dict(sequence=self.sequence, received_monotonic_s=received,
                    sent_monotonic_s=sent, ipc_delivery_s=received-sent,
                    oldest_callback_age_s=received-min(r['arrival_monotonic_s'] for kind in ('imu','clocks') for r in packet[kind]),
                    rows=len(packet['imu'])+len(packet['clocks']),
                    pending_batches=max(0,self.produced.value-self.sequence-1)))
            self.sequence += 1
        except (KeyError, TypeError, ValueError) as exc:
            self.reason = 'isolated IMU IPC invalid: '+str(exc)
            self.fault.set()
            raise ValueError(self.reason) from exc

    def _drain(self):
        while not self.stop.is_set():
            try:
                packet=self.output.get(timeout=.05)
                if self.stop.is_set():
                    return
                self.accept(packet)
            except queue.Empty:
                if not self.process.is_alive():
                    self.reason = 'isolated IMU worker exited'
                    self.fault.set()
                    return
            except (ValueError, EOFError, OSError) as exc:
                if self.stop.is_set():
                    return
                self.reason = self.reason or str(exc)
                self.fault.set()
                return

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop.set()
        if self.process.pid is not None:
            self.process.join(timeout=3)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=2)
            if self.process.is_alive():
                self.process.kill()
                self.process.join(timeout=2)
        if self.thread is not None:
            self.thread.join(timeout=1)
        self.output.close()
        self.output.cancel_join_thread()
        if self.process.pid is not None and self.process.is_alive():
            raise RuntimeError('isolated IMU worker did not stop')
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError('isolated IMU drain did not stop; source must exit')
