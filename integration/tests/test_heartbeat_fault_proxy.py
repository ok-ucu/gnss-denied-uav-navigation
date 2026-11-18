"""Real synthetic TCP only, no FCU endpoints or Linux runtime admission."""
from dataclasses import replace
import socket
import select
import subprocess
import sys
import tempfile
import time
import unittest

from heartbeat_fault_proxy import Pending,ProxyFailure,ReceiveCollector
from heartbeat_stage2_fixture import StreamFixture,packet


class ProxyTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.stream=StreamFixture(self.temp.name);self.addCleanup(self.stream.close)

    def test_fragmented_readiness_silence_stale_and_continued_control(self):
        f=self.stream;f.ready()
        self.assertEqual(f.witness.guard.count,3);self.assertEqual(f.control.guard.count,3)
        cutoff=f.silence();delivered=f.proxy.delivered
        for step in (1,1,1.1):f.advance(step);f.send()
        self.assertEqual(f.witness.failure_code,'HEARTBEAT_STALE')
        self.assertIsNone(f.control.failure_code);self.assertEqual(f.control.guard.count,6)
        self.assertEqual(f.proxy.delivered,delivered);self.assertGreater(f.proxy.discarded,0)
        self.assertEqual(cutoff['delivered_offset'],delivered)
        self.assertEqual(f.proxy.state,'SILENCE')
        # No EOF/close injected: the witness socket remains an open idle TCP stream.
        with self.assertRaises(BlockingIOError):f.witness_socket.recv(1)

    def test_upstream_zero_bytes_and_write_api_attempts_latch_fail(self):
        f=self.stream;f.ready();f.upstream.settimeout(.01)
        with self.assertRaises(socket.timeout):f.upstream.recv(1)
        self.assertEqual(f.rx.application_writes,0)
        for name in ('send','sendall','write'):
            with self.assertRaisesRegex(ProxyFailure,'UPSTREAM_WRITE_ATTEMPT'):getattr(f.rx,name)(b'forbidden')
        self.assertEqual(f.rx.write_attempts,3)
        with self.assertRaises(socket.timeout):f.upstream.recv(1)
        f.collector.step();self.assertIn('UPSTREAM_WRITE_ATTEMPT',f.collector.failures)

    def test_unauthorized_early_wrong_id_and_repeated_trigger(self):
        f=self.stream
        with self.assertRaisesRegex(ProxyFailure,'READINESS_NOT_VERIFIED'):
            f.proxy.trigger(f.test_id,f.capability,None)
        self.assertEqual(f.proxy.state,'CLOSED')
        # Distinct fresh fixture per unauthorized/repeated path, never recovery.
        for mode in ('wrong-id','wrong-capability','repeated'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as temp:
                g=StreamFixture(temp)
                try:
                    g.ready()
                    if mode=='repeated':g.silence()
                    with self.assertRaises(ProxyFailure):
                        g.proxy.trigger('foreign' if mode=='wrong-id' else g.test_id,
                            object() if mode=='wrong-capability' else g.capability,g.witness.proof)
                    self.assertEqual(g.proxy.state,'CLOSED')
                finally:g.close()

    def test_copied_readiness_fields_and_failed_guard_cannot_authorize_trigger(self):
        f=self.stream;f.ready()
        forged=replace(f.witness.proof)  # identical fields/issuer, never issued
        with self.assertRaisesRegex(ProxyFailure,'READINESS_NOT_VERIFIED'):
            f.proxy.trigger(f.test_id,f.capability,forged)
        with tempfile.TemporaryDirectory() as temp:
            g=StreamFixture(temp)
            try:
                g.ready();g.witness.guard.failure='synthetic latched failure'
                with self.assertRaisesRegex(ProxyFailure,'READINESS_NOT_VERIFIED'):g.silence()
            finally:g.close()

    def test_eof_disconnect_and_partial_packet_are_not_expected_stale(self):
        f=self.stream;f.ready();f.upstream.close()
        for _ in range(200):
            f.collector.step()
            if f.collector.failures:break
            time.sleep(.001)
        self.assertIn('UPSTREAM_EOF',f.collector.failures)
        # Proxy/witness TCP EOF has its own cause, never a heartbeat timeout.
        f.proxy_socket.close()
        for _ in range(200):
            f.witness.step()
            if f.witness.failure_code:break
            time.sleep(.001)
        self.assertEqual(f.witness.failure_code,'DOWNSTREAM_EOF')
        with tempfile.TemporaryDirectory() as temp:
            g=StreamFixture(temp)
            try:
                g.ready();g.send(packet()[:4]);g.silence()
                g.advance(3.1);g.witness.step()
                self.assertEqual(g.witness.failure_code,'PARTIAL_MAVLINK_STALE')
            finally:g.close()

    def test_malformed_armed_wrong_id_never_reaches_readiness(self):
        for raw,code in ((b'garbage','MALFORMED_MAVLINK'),(packet(armed=True),'INVALID_HEARTBEAT'),
                         (packet(system=2),'INVALID_HEARTBEAT'),(packet(component=9),'INVALID_HEARTBEAT')):
            with self.subTest(code=code),tempfile.TemporaryDirectory() as temp:
                f=StreamFixture(temp)
                try:
                    f.send(raw);self.assertFalse(f.witness.ready)
                    self.assertEqual(f.witness.failure_code,code)
                    self.assertIsNotNone(f.control.failure_code)
                finally:f.close()

    def test_proxy_input_eof_is_distinct_and_fail_closed(self):
        f=self.stream;f.proxy_writer.close()
        with self.assertRaisesRegex(ProxyFailure,'PROXY_INPUT_EOF'):f.proxy.step()
        self.assertEqual(f.proxy.state,'CLOSED')

    def test_missing_heartbeat_and_pre_ready_stale_use_existing_guards(self):
        f=self.stream;f.advance(10.1);f.witness.step()
        self.assertEqual(f.witness.failure_code,'NO_HEARTBEAT');self.assertFalse(f.witness.ready)
        with tempfile.TemporaryDirectory() as temp:
            g=StreamFixture(temp)
            try:
                g.send();g.advance(3.1);g.witness.step()
                self.assertEqual(g.witness.failure_code,'HEARTBEAT_STALE');self.assertFalse(g.witness.ready)
                g.witness.finish()  # Valid FAILED export even without readiness.
                from heartbeat_test_evidence import verify_witness
                self.assertFalse(verify_witness(temp,g.test_id)['ready'])
            finally:g.close()

    def test_receive_disconnect_and_latched_failure_never_reestablish_readiness(self):
        f=self.stream;f.ready();proof=f.witness.proof
        class Disconnected:
            def recv(self,size):raise ConnectionResetError('synthetic reset')
        f.witness.reader=Disconnected();f.witness.step()
        self.assertEqual(f.witness.failure_code,'DOWNSTREAM_DISCONNECT')
        count=f.witness.guard.count;f.advance(1);f.witness.step()
        self.assertEqual(f.witness.guard.count,count);self.assertIs(f.witness.proof,proof)

    def test_eof_or_late_bytes_at_stale_boundary_are_not_expected_silence(self):
        for mode,code in (('eof','DOWNSTREAM_EOF'),('data','LATE_DATA_AFTER_STALE')):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as temp:
                f=StreamFixture(temp)
                try:
                    f.ready()
                    if mode=='eof':f.proxy_socket.close()
                    else:f.proxy_socket.sendall(packet())  # Delayed delivery fixture, not FCU TX.
                    self.assertTrue(select.select([f.witness_socket],[],[],1)[0])
                    f.advance(3.1);f.witness.step()
                    self.assertEqual(f.witness.failure_code,code)
                    self.assertEqual(f.witness.guard.count,3)  # Late bytes never restore validity.
                finally:f.close()

    def test_independent_bounded_queue_never_blocks_control_on_witness_overflow(self):
        class Rx:
            def __init__(self):self.payload=[b'abcd',b'efgh',b'ijkl']
            def recv(self,size):
                if self.payload:return self.payload.pop(0)
                raise BlockingIOError()
        class Fast:
            def __init__(self):self.data=b''
            def send(self,data):self.data+=data;return len(data)
        class Slow:
            def send(self,data):raise BlockingIOError()
        fast=Fast();collector=ReceiveCollector(Rx(),fast,Slow(),4)
        for _ in range(6):collector.step()
        self.assertEqual(fast.data,b'abcdefghijkl')
        self.assertIn('WITNESS_QUEUE_OVERFLOW',collector.failures)
        self.assertNotIn('CONTROL_QUEUE_OVERFLOW',collector.failures)

    def test_pending_partial_writes_preserve_order(self):
        pending=Pending(20);pending.append(b'abcdef');output=[]
        def send(data):output.append(data[:2]);return min(2,len(data))
        for _ in range(3):pending.flush(send)
        self.assertEqual(b''.join(output),b'abcdef');self.assertEqual(pending.bytes,0)

    def test_control_separate_process_receives_during_witness_silence(self):
        f=self.stream
        code='''import os,sys,time
from linux_gpu.verified_disarmed import HeartbeatGuard
fd=int(sys.argv[1]);os.set_blocking(fd,False)
guard=HeartbeatGuard(1,1,time.monotonic());deadline=time.monotonic()+4
while guard.count<8 and time.monotonic()<deadline:
    try:data=os.read(fd,65536)
    except BlockingIOError:time.sleep(.001);continue
    if not data:raise SystemExit(3)
    guard.feed(data,time.monotonic())
print(guard.count,flush=True)
raise SystemExit(0 if guard.count==8 else 4)
'''
        child=subprocess.Popen([sys.executable,'-B','-c',code,str(f.control_reader.fd)],
            pass_fds=(f.control_reader.fd,),stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            # Parent never reads this pipe. Child inherits ONLY its read end,
            # not the upstream socket or any writer capability.
            for index in range(3):
                f.send(consume_control=False)
                if index<2:f.advance(1)
            self.assertTrue(f.witness.ready);f.silence()
            for _ in range(5):f.advance(.5);f.send(consume_control=False)
            out,err=child.communicate(timeout=5)
            self.assertEqual(child.returncode,0,err);self.assertEqual(out.strip(),'8')
            self.assertIsNone(f.witness.failure_code)
        finally:
            if child.poll() is None:child.kill()
            child.communicate(timeout=2)


if __name__=='__main__':unittest.main()
