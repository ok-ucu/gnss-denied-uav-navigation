"""Offline-testable RX collector and witness-only byte suppression.

No connect(), upstream writer, FCU default address or runtime entrypoint. A
reviewed bootstrap must supply descriptors AFTER admission. The native network
namespace/firewall and separate-process bootstrap are not integrated yet.
"""
from collections import deque
from dataclasses import dataclass
import os
import time


class ProxyFailure(RuntimeError):
    def __init__(self, code): self.code=code; super().__init__(code)


class ReadPipe:
    """Only the read descriptor crosses into a consumer process."""
    def __init__(self, fd): self.fd=fd; os.set_blocking(fd,False)
    def recv(self, size): return os.read(self.fd,size)
    def close(self):
        if self.fd is not None:os.close(self.fd);self.fd=None


class WritePipe:
    def __init__(self, fd): self.fd=fd; os.set_blocking(fd,False)
    def send(self, data): return os.write(self.fd,data)
    def close(self):
        if self.fd is not None:os.close(self.fd);self.fd=None


def one_way_pipe():
    reader,writer=os.pipe()
    return ReadPipe(reader),WritePipe(writer)


class RxOnlyIngress:
    """Minimal capability wrapper; no descriptor/socket is exposed to consumers.

    Native OS isolation is still mandatory: Python privacy cannot sandbox a
    malicious process. Attempts through this API latch FAIL before any TX syscall.
    """
    def __init__(self, stream, record=lambda *a,**k:None):
        self.__stream=stream;stream.setblocking(False)
        self.record=record;self.bytes_received=0;self.application_writes=0
        self.write_attempts=0;self.failure=None

    def recv(self,size):
        if self.failure:raise ProxyFailure(self.failure)
        data=self.__stream.recv(size);self.bytes_received+=len(data);return data

    def _deny(self,*_args,**_kwargs):
        self.write_attempts+=1;self.failure='UPSTREAM_WRITE_ATTEMPT'
        self.record('upstream_write_attempt',attempts=self.write_attempts)
        raise ProxyFailure(self.failure)

    send=_deny
    sendall=_deny
    write=_deny

    def close(self):self.__stream.close()


class Pending:
    def __init__(self,limit):
        if type(limit) is not int or limit<=0:raise ValueError('Explicit positive queue byte bound required')
        self.limit=limit;self.parts=deque();self.bytes=0
    def append(self,data):
        if self.bytes+len(data)>self.limit:raise ProxyFailure('QUEUE_OVERFLOW')
        if data:self.parts.append(bytes(data));self.bytes+=len(data)
    def flush(self,send):
        if not self.parts:return 0
        try:sent=send(self.parts[0])
        except (BlockingIOError,InterruptedError):return 0
        if sent<=0:raise ProxyFailure('DOWNSTREAM_DISCONNECT')
        if sent>len(self.parts[0]):raise ProxyFailure('INVALID_SEND_RESULT')
        self.bytes-=sent
        if sent==len(self.parts[0]):self.parts.popleft()
        else:self.parts[0]=self.parts[0][sent:]
        return sent
    def clear(self):
        removed=self.bytes;self.parts.clear();self.bytes=0;return removed


class ReceiveCollector:
    """Independent bounded fanout queues: witness backpressure cannot block control.

    An overflow is latched FAIL, never successful silence. Control delivery can
    continue while the coordinator requests cleanup. No automatic recovery.
    """
    def __init__(self,ingress,control_writer,witness_writer,queue_bytes,record=lambda *a,**k:None):
        self.ingress=ingress;self.writers={'control':control_writer,'witness':witness_writer}
        self.pending={key:Pending(queue_bytes) for key in self.writers}
        self.record=record;self.offset=0;self.sequence=0;self.failures=[];self.eof=False

    def fail(self,code):
        if code not in self.failures:self.failures.append(code);self.record('failure',code=code)

    def step(self):
        # Service control FIRST, nonblocking, even if witness overflowed earlier.
        for key in ('control','witness'):
            try:self.pending[key].flush(self.writers[key].send)
            except (OSError,ProxyFailure) as exc:self.fail(key.upper()+'_'+getattr(exc,'code','DISCONNECT'))
        if self.eof:return
        try:data=self.ingress.recv(65536)
        except (BlockingIOError,InterruptedError):return
        except (OSError,ProxyFailure) as exc:
            self.fail(getattr(exc,'code','UPSTREAM_DISCONNECT'));self.eof=True;return
        if not data:self.eof=True;self.fail('UPSTREAM_EOF');return
        self.record('upstream_receive',offset=self.offset,bytes=len(data),chunk_sequence=self.sequence)
        self.offset+=len(data);self.sequence+=1
        for key in ('control','witness'):
            if any(code.startswith(key.upper()+'_') for code in self.failures):continue
            try:self.pending[key].append(data)
            except ProxyFailure:self.fail(key.upper()+'_QUEUE_OVERFLOW')


@dataclass(frozen=True)
class ReadinessProof:
    """In-process capability, not a serialized authorization or native peer proof."""
    test_id: str
    count: int
    first_ns: int
    last_ns: int
    ready_ns: int
    issuer: object


class ReadinessAuthority:
    """Retains issued proof identity AND the actual guard; fields alone cannot authorize.

    This portable capability registry is not an authenticated cross-process IPC
    service. A native implementation remains gated on the host/network backend.
    """
    def __init__(self):self.__issued={}
    def issue(self,test_id,guard,first_ns,last_ns,now_ns):
        if not guard.ready(now_ns/1e9):raise ProxyFailure('READINESS_NOT_VERIFIED')
        proof=ReadinessProof(test_id,guard.count,first_ns,last_ns,now_ns,self)
        self.__issued[id(proof)]=(proof,guard)
        return proof
    def verify(self,proof,test_id,now_ns):
        issued=self.__issued.get(id(proof))
        if not issued or issued[0] is not proof or proof.test_id!=test_id:return False
        try:return issued[1].ready(now_ns/1e9) is True
        except RuntimeError:return False


class FaultProxy:
    def __init__(self,test_id,reader,downstream,queue_bytes,controller_capability,
                 readiness_issuer,record=lambda *a,**k:None,clock=time.monotonic_ns):
        if controller_capability is None or not isinstance(readiness_issuer,ReadinessAuthority):
            raise ValueError('Private controller/readiness capabilities required')
        self.test_id=test_id;self.reader=reader;self.__downstream=downstream
        downstream.setblocking(False)
        self.pending=Pending(queue_bytes);self.__capability=controller_capability
        self.__issuer=readiness_issuer;self.record=record;self.clock=clock
        self.state='RELAY';self.failures=[];self.received=0;self.delivered=0;self.discarded=0
        self.cutoff=None;self.last_send_ns=None;self.sequence=0
        self.record('started',state='RELAY')

    def fail(self,code):
        if code not in self.failures:self.failures.append(code);self.record('failure',code=code)
        self.close()
        raise ProxyFailure(code)

    def trigger(self,test_id,capability,proof):
        now=self.clock()
        if test_id!=self.test_id or capability is not self.__capability:self.fail('UNAUTHORIZED_TRIGGER')
        if self.state!='RELAY' or self.failures:self.fail('REPEATED_OR_FAILED_TRIGGER')
        if (not isinstance(proof,ReadinessProof) or proof.issuer is not self.__issuer or
                not self.__issuer.verify(proof,self.test_id,now) or
                proof.test_id!=self.test_id or type(proof.count) is not int or proof.count<3 or
                any(type(v) is not int for v in (proof.first_ns,proof.last_ns,proof.ready_ns)) or
                proof.last_ns-proof.first_ns<2_000_000_000 or
                not proof.last_ns<=proof.ready_ns<=now or now-proof.last_ns>3_000_000_000):
            self.fail('READINESS_NOT_VERIFIED')
        dropped=self.pending.clear();self.discarded+=dropped
        self.state='SILENCE'
        self.cutoff=dict(host_monotonic_ns=now,received_offset=self.received,
                         delivered_offset=self.delivered,last_send_ns=self.last_send_ns,
                         discarded_pending=dropped,readiness_last_heartbeat_ns=proof.last_ns)
        self.record('trigger',cutoff=dict(self.cutoff))
        return self.cutoff

    def step(self):
        if self.state=='CLOSED':return
        try:data=self.reader.recv(65536)
        except (BlockingIOError,InterruptedError):data=None
        except OSError:self.fail('PROXY_INPUT_DISCONNECT')
        if data==b'':self.fail('PROXY_INPUT_EOF')
        if data:
            self.record('proxy_receive',offset=self.received,bytes=len(data),chunk_sequence=self.sequence)
            self.sequence+=1;self.received+=len(data)
            if self.state=='SILENCE':self.discarded+=len(data)
            else:
                try:self.pending.append(data)
                except ProxyFailure:self.fail('WITNESS_QUEUE_OVERFLOW')
        if self.state=='RELAY':
            try:sent=self.pending.flush(self.__downstream.send)
            except (OSError,ProxyFailure):self.fail('WITNESS_DISCONNECT')
            if sent:
                self.last_send_ns=self.clock();self.delivered+=sent
                self.record('witness_send',bytes=sent,delivered_offset=self.delivered)

    def close(self):
        if self.state!='CLOSED':
            self.state='CLOSED';self.__downstream.close();self.reader.close()
            self.record('closed',delivered=self.delivered,discarded=self.discarded)

    def summary(self,ingress,collector):
        return dict(test_id=self.test_id,state=self.state,failures=list(self.failures)+list(collector.failures),
            cutoff=self.cutoff,received=self.received,delivered=self.delivered,discarded=self.discarded,
            upstream_bytes=ingress.bytes_received,upstream_application_writes=ingress.application_writes,
            upstream_write_attempts=ingress.write_attempts,collector_chunks=collector.sequence)
