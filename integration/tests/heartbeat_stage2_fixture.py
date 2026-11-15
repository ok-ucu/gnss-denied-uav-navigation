"""Synthetic LOOPBACK TCP/pipe fixture. Never contacts FCU or starts a simulator."""
import socket
import hashlib
from pathlib import Path

from heartbeat_fault_proxy import FaultProxy,ReadinessAuthority,ReceiveCollector,RxOnlyIngress,one_way_pipe
from heartbeat_test_evidence import EventJournal
from heartbeat_test_owner import CleanupController,durable_new
from heartbeat_witness import HeartbeatConsumer
from linux_gpu.verified_disarmed import load_codec
from test_heartbeat_test_owner import fixture,FakeBackend


def tcp_pair():
    listener=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    try:
        listener.bind(('127.0.0.1',0));listener.listen(1)
        client=socket.create_connection(listener.getsockname(),timeout=1)
        server,_=listener.accept()
        for stream in (server,client):stream.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        return server,client
    finally:listener.close()


def packet(system=1,component=1,armed=False):
    wire=load_codec();sender=wire.MAVLink(None,srcSystem=system,srcComponent=component)
    return wire.MAVLink_heartbeat_message(2,3,128 if armed else 0,0,3,3).pack(sender)


class StreamFixture:
    def __init__(self,root):
        self.root=Path(root);self.clock,self.ledger=fixture();self.test_id=self.ledger.test_id
        self.ledger.publish(root);self.capability=object();self.issuer=ReadinessAuthority()
        self.watchdog_journal=EventJournal(self.root/'watchdog.jsonl',self.test_id,self.clock)
        self.watchdog_journal.emit('synthetic_manager_armed',deadline_ns=self.ledger.deadline.end_ns)
        self.upstream,self.ingress_socket=tcp_pair();self.proxy_socket,self.witness_socket=tcp_pair()
        self.control_reader,self.control_writer=one_way_pipe();self.proxy_reader,self.proxy_writer=one_way_pipe()
        self.proxy_journal=EventJournal(self.root/'proxy.jsonl',self.test_id,self.clock)
        self.collector_journal=EventJournal(self.root/'collector.jsonl',self.test_id,self.clock)
        self.rx=RxOnlyIngress(self.ingress_socket,self.collector_journal.emit)
        self.collector=ReceiveCollector(self.rx,self.control_writer,self.proxy_writer,65536,self.collector_journal.emit)
        self.proxy=FaultProxy(self.test_id,self.proxy_reader,self.proxy_socket,65536,self.capability,
            self.issuer,self.proxy_journal.emit,self.clock)
        self.witness=HeartbeatConsumer(self.witness_socket,root,self.test_id,1,1,'witness',
            self.clock,readiness_issuer=self.issuer)
        self.control=HeartbeatConsumer(self.control_reader,root,self.test_id,1,1,'control',self.clock)
        self.finished=False

    def pump(self,repeat=4,consume_control=True):
        for _ in range(repeat):
            self.collector.step();self.proxy.step();self.witness.step()
            if consume_control:self.control.step()

    def send(self,data=None,consume_control=True):
        data=packet() if data is None else data
        self.upstream.sendall(data);self.pump(8,consume_control)

    def advance(self,seconds):self.clock.now+=int(seconds*1e9)

    def ready(self):
        for index in range(3):
            raw=packet()
            for part in (raw[:4],raw[4:9],raw[9:]):self.send(part)
            if index<2:self.advance(1)
        if not self.witness.ready or not self.control.ready:raise AssertionError('Synthetic readiness missing')

    def silence(self):return self.proxy.trigger(self.test_id,self.capability,self.witness.proof)

    def stale(self):
        self.ready();self.silence()
        for dt in (1,1,1.1):self.advance(dt);self.send()
        if self.witness.failure_code!='HEARTBEAT_STALE':raise AssertionError(self.witness.failure_code)

    def finish(self,cleanup=True):
        self.proxy.close();self.witness.finish();self.control.finish()
        self.proxy_journal.close();self.collector_journal.close()
        durable_new(self.root/'proxy.json',self.proxy.summary(self.rx,self.collector))
        self.control_writer.close();self.proxy_writer.close();self.rx.close();self.upstream.close()
        self.watchdog_journal.emit('synthetic_cleanup_requested')
        durable_new(self.root/'admission.json',dict(schema=2,test_id=self.test_id,evidence_scope='synthetic',
            ledger_sha256=self.ledger.digest(),endpoint_noninterference_verified=True,
            manager_recovery_verified=True,network_isolation_verified=True,forbidden_actions=[],
            endpoint='test-owned loopback only; no FCU',scope='synthetic adapter assertions only'))
        if cleanup:
            result=CleanupController(self.ledger,FakeBackend(self.clock),self.clock).run(self.root)
            if result['status']!='PASS':raise AssertionError(result)
            self.watchdog_journal.emit('synthetic_cleanup_finished',status=result['status'],
                cleanup_receipt_sha256=hashlib.sha256((self.root/'cleanup.receipt.json').read_bytes()).hexdigest())
        self.watchdog_journal.close()
        self.finished=True

    def close(self):
        if self.finished:return
        # Fixture teardown only: no process groups/container cleanup.
        for journal in (self.witness.journal,self.control.journal,self.proxy_journal,
                        self.collector_journal,self.watchdog_journal):
            if not journal.closed:journal.close()
        for stream in (self.upstream,self.ingress_socket,self.proxy_socket,self.witness_socket):
            try:stream.close()
            except OSError:pass
        for pipe in (self.control_reader,self.control_writer,self.proxy_reader,self.proxy_writer):
            try:pipe.close()
            except OSError:pass
