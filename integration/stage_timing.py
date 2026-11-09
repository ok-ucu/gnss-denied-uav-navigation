"""Opt-in in-memory stage profiling; no diagnostic disk writes on the hot path.

All *_host_ns are Linux CLOCK_MONOTONIC, *_cpu_ns process CPU time. Acquisition
labels remain simulation nanoseconds. The two domains are never subtracted.
"""
from collections import deque
from contextlib import contextmanager
from contextvars import ContextVar
import json
from pathlib import Path
import threading
import time
import queue

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


class FaultFile:
    """Explicit local fault injection, never a navigation-data fallback."""
    def __init__(self,path):
        self.path=Path(path) if path else None
        self.last=0.;self.value={}

    def read(self):
        now=time.monotonic()
        if self.path is None:return {}
        if now-self.last>=.02:
            value=json.loads(self.path.read_text())
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
