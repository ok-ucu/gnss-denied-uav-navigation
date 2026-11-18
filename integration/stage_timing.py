"""Opt-in in-memory stage profiling; no diagnostic disk writes on the hot path.

All *_host_ns are Linux CLOCK_MONOTONIC. cpu_ns/process_cpu_ns measure process
CPU; thread_cpu_ns measures the calling thread. Acquisition labels remain
simulation nanoseconds. The two time domains are never subtracted.
"""
from collections import deque
from contextlib import contextmanager
from contextvars import ContextVar
import json
from pathlib import Path
import threading
import time
import queue
import resource

_recorder = None
_sample = ContextVar('acquisition_ns', default=None)


class Recorder:
    def __init__(self, path, capacity=100000):
        self.path = Path(path)
        if self.path.exists():
            raise ValueError('Never overwrite stage evidence')
        self.rows = deque(maxlen=capacity)
        self.dropped = 0
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._sample_cpu, daemon=True)
        self.thread.start()

    def add(self, row):
        if len(self.rows)==self.rows.maxlen:
            self.dropped += 1
        self.rows.append(row)

    def _sample_cpu(self):
        while not self.stop.is_set():
            try:
                values = dict(line.split() for line in Path('/sys/fs/cgroup/cpu.stat').read_text().splitlines())
                self.add(dict(stage='cgroup_cpu',host_ns=time.monotonic_ns(),values={k:int(v) for k,v in values.items()}))
            except OSError:
                pass
            self.stop.wait(.5)

    def close(self):
        self.stop.set();self.thread.join(timeout=2)
        with self.path.open('x') as output:
            for row in self.rows:
                output.write(json.dumps(row,allow_nan=False)+'\n')
            output.write(json.dumps(dict(stage='profile_summary',dropped=self.dropped))+'\n')


def activate(path):
    global _recorder
    if _recorder is not None:
        raise RuntimeError('Profile already active')
    _recorder = Recorder(path)
    return _recorder


@contextmanager
def sample(acquisition_ns):
    token = _sample.set(acquisition_ns)
    try:
        yield
    finally:
        _sample.reset(token)


@contextmanager
def stage(name, **metadata):
    recorder = _recorder
    if recorder is None:
        yield
        return
    start = time.monotonic_ns();cpu = time.process_time_ns()
    try:
        yield
    finally:
        recorder.add(dict(stage=name,acquisition_ns=_sample.get(),start_host_ns=start,
                          end_host_ns=time.monotonic_ns(),cpu_ns=time.process_time_ns()-cpu,**metadata))


def mark(name, **metadata):
    if _recorder is not None:
        _recorder.add(dict(stage=name,acquisition_ns=_sample.get(),host_ns=time.monotonic_ns(),**metadata))


def thread_schedule():
    """Actual Linux thread counters, or unavailable; never substitute zeros."""
    try:
        return tuple(map(int,Path('/proc/self/task/'+str(threading.get_native_id())+'/schedstat').read_text().split()))
    except (OSError,ValueError):
        return None


@contextmanager
def critical_stage(name, **metadata):
    """Opt-in CPU/wall profiling for coarse source stages, not every IMU sample.

    Scheduler reads bracket the timed work and have their own overhead. Runqueue
    counters are not futex/IO wait; wall minus thread CPU alone cannot name a wait.
    Existing stage() process-CPU fields/transport instrumentation stay unchanged.
    """
    recorder=_recorder
    detail={}
    if recorder is None:
        yield detail
        return
    detail['profiled']=True
    overhead=time.monotonic_ns();schedule=thread_schedule()
    usage=resource.getrusage(resource.RUSAGE_THREAD) if hasattr(resource,'RUSAGE_THREAD') else None
    start=time.monotonic_ns();cpu=time.thread_time_ns();process=time.process_time_ns()
    try:
        yield detail
    finally:
        end=time.monotonic_ns();thread_cpu=time.thread_time_ns()-cpu;process_cpu=time.process_time_ns()-process
        after=thread_schedule()
        final_usage=resource.getrusage(resource.RUSAGE_THREAD) if usage is not None else None
        delta=None if schedule is None or after is None else [b-a for a,b in zip(schedule,after)]
        recorder.add(dict(stage=name,acquisition_ns=_sample.get(),start_host_ns=start,end_host_ns=end,
            thread_id=threading.get_native_id(),thread_cpu_ns=thread_cpu,process_cpu_ns=process_cpu,cpu_ns=process_cpu,
            schedule_delta=delta,voluntary_switches=None if usage is None else final_usage.ru_nvcsw-usage.ru_nvcsw,
            involuntary_switches=None if usage is None else final_usage.ru_nivcsw-usage.ru_nivcsw,
            entry_probe_ns=start-overhead,exit_probe_ns=time.monotonic_ns()-end,**metadata,**detail))


@contextmanager
def trace_span(name, slow_ns=5_000_000, **metadata):
    """Cheap optional loop probe; no proc/file reads or scheduler guesses.

    Retain slow calls and calls carrying an actual frame label. Fast empty polls
    are omitted, not reported as zero queue wait. CPU is measured, not inferred.
    """
    recorder=_recorder
    detail={}
    if recorder is None:
        yield detail
        return
    start=time.monotonic_ns();cpu=time.thread_time_ns();process=time.process_time_ns()
    try:
        yield detail
    finally:
        end=time.monotonic_ns();thread_cpu=time.thread_time_ns()-cpu
        process_cpu=time.process_time_ns()-process
        if end-start>=slow_ns or 'source_ns' in detail:
            recorder.add(dict(stage=name,acquisition_ns=_sample.get(),start_host_ns=start,end_host_ns=end,
                thread_id=threading.get_native_id(),thread_cpu_ns=thread_cpu,process_cpu_ns=process_cpu,
                schedule_delta=None,**metadata,**detail))


class FrameQueueLock:
    """Profile the existing latest-only slot, preserving its exact lock/contents.

    The owner's callback/poll still do all insertion/dequeue operations. Logging
    occurs after unlocking; no file I/O or change to dropped-frame policy.
    """
    def __init__(self, owner):
        self.owner=owner
        self.lock=threading.Lock()
        self.local=threading.local()
        self.inserted_ns=None

    def __enter__(self):
        start=time.monotonic_ns();cpu=time.thread_time_ns()
        self.lock.acquire()
        acquired=time.monotonic_ns()
        self.local.before=self.owner._latest
        self.local.start=start;self.local.cpu=cpu;self.local.acquired=acquired
        return self

    def __exit__(self, *exc):
        # Inspect the unchanged slot while held, then release before recording.
        before=self.local.before;after=self.owner._latest
        end=time.monotonic_ns();queue_wait=None;replaced=None;kind=None;stamp=None
        if after is not None and after is not before:
            kind='camera_queue_insert';stamp=round(after[1]*1e9)
            replaced=None if before is None else round(before[1]*1e9)
            self.inserted_ns=end
        elif before is not None and after is None:
            kind='camera_queue_dequeue';stamp=round(before[1]*1e9)
            queue_wait=None if self.inserted_ns is None else self.local.acquired-self.inserted_ns
            self.inserted_ns=None
        cpu=time.thread_time_ns()-self.local.cpu
        self.lock.release()
        wait=self.local.acquired-self.local.start
        if kind is not None or wait>=5_000_000:
            mark(kind or 'camera_queue_lock_wait',source_ns=stamp,event_host_ns=end,
                lock_start_host_ns=self.local.start,lock_acquired_host_ns=self.local.acquired,
                lock_wait_ns=wait,lock_thread_cpu_ns=cpu,queue_wait_ns=queue_wait,
                depth_before=int(before is not None),depth_after=int(after is not None),
                replaced_source_ns=replaced,thread_id=threading.get_native_id())


class FaultFile:
    """Explicit local fault injection, never a navigation-data fallback."""
    def __init__(self,path):
        self.path=Path(path) if path else None
        self.last=0.;self.value={}

    def read(self):
        now=time.monotonic()
        if self.path is None:return {}
        if now-self.last>=.02:
            with trace_span('fault_control_read',slow_ns=0,path=str(self.path)) as timing:
                timing['json_loaded']=False
                value=json.loads(self.path.read_text())
                timing['json_loaded']=True
            if value.get('fault') not in (None,'camera_loss','imu_loss','tracking_loss','low_contrast','bad_covariance','stale'):
                raise ValueError('Invalid fault injection control')
            self.value=value;self.last=now
        return self.value


class AsyncAudit:
    """Bounded diagnostic I/O only. Authorization writes NEVER use this queue.

    Overflow or writer failure raises; the transport's finally closes its guard.
    SIGTERM shutdown drains the queue before files close. Hard-kill may lose the
    diagnostic tail; this does not grant or persist any authorization.
    """
    def __init__(self,files,capacity=10000):
        self.files=files;self.queue=queue.Queue(capacity);self.error=None
        self.thread=threading.Thread(target=self._write,daemon=True);self.thread.start()

    def put(self,name,value):
        if self.error is not None:raise RuntimeError('Diagnostic writer failed') from self.error
        try:self.queue.put_nowait((name,value))
        except queue.Full:raise RuntimeError('Diagnostic queue full: fail closed')

    def _write(self):
        while True:
            item=self.queue.get()
            if item is None:return
            if self.error is not None:continue
            try:
                name,value=item
                with stage('diagnostic_write',stream=name):
                    self.files[name].write(value if isinstance(value,bytes) else json.dumps(value,allow_nan=False,default=str)+'\n')
                    self.files[name].flush()
            except Exception as error:self.error=error

    def close(self):
        self.queue.put(None);self.thread.join(timeout=10)
        if self.thread.is_alive() or self.error is not None:
            raise RuntimeError('Diagnostic drain failed') from self.error
