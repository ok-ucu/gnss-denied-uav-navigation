"""Real codec, synthetic transport only; no live FCU or identity authentication."""
import json
from pathlib import Path
import socket
import tempfile
import unittest
from unittest.mock import patch

from linux_gpu.verified_disarmed import HeartbeatGuard, check, load_codec


class VerifiedDisarmedTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.wire=load_codec()

    def packet(self,system=1,component=1,armed=False,v1=False,autopilot=3,vehicle=2):
        return self.wire.MAVLink_heartbeat_message(vehicle,autopilot,128 if armed else 0,0,3,3).pack(
            self.wire.MAVLink(None,srcSystem=system,srcComponent=component),force_mavlink1=v1)

    def test_prefix_108_and_crc_corruption_are_not_heartbeat(self):
        guard=HeartbeatGuard(1,1,0)
        bad=bytearray(self.packet());bad[-1]^=255
        guard.feed(b'loaded console\n'+bytes(bad),0)
        self.assertEqual(guard.count,0);self.assertGreater(guard.bad_data,0)
        self.assertFalse(guard.ready(1))
        for t in (1,2,3):guard.feed(self.packet(),t)
        self.assertTrue(guard.ready(3))
        self.assertEqual(guard.count,3)

    def test_fragments_preserve_real_codec_framing(self):
        guard=HeartbeatGuard(1,1,0)
        for t in (0,1,2):
            packet=self.packet()
            for b in packet:guard.feed(bytes([b]),t)
        self.assertTrue(guard.ready(2))

    def test_wrong_identity_role_version_or_armed_latches_failure(self):
        for changes in (dict(system=42),dict(component=42),dict(autopilot=0),
                        dict(vehicle=6),dict(v1=True),dict(armed=True)):
            with self.subTest(changes=changes):
                guard=HeartbeatGuard(1,1,0)
                with self.assertRaises(RuntimeError):guard.feed(self.packet(**changes),1)
                with self.assertRaises(RuntimeError):guard.feed(self.packet(),2)
                self.assertEqual(guard.count,0)

    def test_partial_bad_data_other_packets_and_single_hb_cannot_pass(self):
        for data in (b'junk\n',self.packet()[:-1],self.wire.MAVLink_timesync_message(0,1).pack(self.wire.MAVLink(None))):
            with self.subTest(data=data):
                guard=HeartbeatGuard(1,1,0);guard.feed(data,0)
                self.assertFalse(guard.ready(1))
                with self.assertRaisesRegex(RuntimeError,'within 10s'):guard.assert_fresh(10.001)
        guard=HeartbeatGuard(1,1,0);guard.feed(self.packet(),0)
        self.assertFalse(guard.ready(2))
        with self.assertRaisesRegex(RuntimeError,'stale'):guard.feed(self.packet(),3.001)

    def test_invalid_ids_time_and_same_chunk_repeats_cannot_establish_span(self):
        for system,component in ((0,1),(1,256),(True,1)):
            with self.assertRaises(ValueError):HeartbeatGuard(system,component,0)
        guard=HeartbeatGuard(1,1,0);guard.feed(self.packet()*3,0)
        self.assertFalse(guard.ready(2))
        for t in (float('nan'),float('inf'),-1):
            with self.subTest(time=t),self.assertRaises(RuntimeError):
                HeartbeatGuard(1,1,0).assert_fresh(t)

    def test_check_persists_raw_chunks_and_fail_closed_result_without_transmission(self):
        class Clock:
            now=0
            def monotonic(self):return self.now
        clock=Clock();packet=self.packet()
        class ReceiveOnly:
            closed=False
            def __enter__(self):return self
            def __exit__(self,*_):self.closed=True
            def settimeout(self,*_):pass
            def recv(self,*_):clock.now+=1;return b'l'+packet
            def send(self,*_):raise AssertionError('Transmitted')
            def sendall(self,*_):raise AssertionError('Transmitted')
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'check';stream=ReceiveOnly()
            with patch('linux_gpu.verified_disarmed.time.monotonic',clock.monotonic), \
                 patch('linux_gpu.verified_disarmed.socket.create_connection',return_value=stream):
                result=check(1,1,path)
            self.assertEqual(result['status'],'PASS');self.assertTrue(stream.closed)
            self.assertEqual((path/'tcp.bin').read_bytes(),(b'l'+packet)*5)
            self.assertGreater(result['bad_data'],0)
            self.assertFalse(result['forwarding_enabled'])
            with self.assertRaises(FileExistsError):check(1,1,path)
            for mode in ('eof','timeout'):
                clock.now=0
                def recv(*_):
                    clock.now+=1
                    if mode=='timeout':raise socket.timeout()
                    return b''
                with patch.object(stream,'recv',side_effect=recv), \
                     patch('linux_gpu.verified_disarmed.time.monotonic',clock.monotonic), \
                     patch('linux_gpu.verified_disarmed.socket.create_connection',return_value=stream):
                    with self.assertRaises(RuntimeError):check(1,1,Path(temp)/mode)
                self.assertEqual(json.loads((Path(temp)/mode/'result.json').read_text())['status'],'FAIL')


if __name__=='__main__':unittest.main()
