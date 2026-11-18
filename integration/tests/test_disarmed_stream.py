"""Real codec + synthetic sockets. Characterization, NOT Linux heartbeat proof.

Production disarmed.py stays unchanged/strict. Known permissive identity/version
behavior is named explicitly so a unit PASS cannot be called FCU identification.
"""
from contextlib import redirect_stdout
import importlib.util
import io
from pathlib import Path
import socket
import unittest
from unittest.mock import patch

from linux_gpu import disarmed


class Clock:
    now=0.
    def __call__(self):return self.now


class ReceiveOnly:
    def __init__(self,events,clock):
        self.events=list(events);self.clock=clock;self.closed=False;self.receives=0
    def __enter__(self):return self
    def __exit__(self,*_):self.closed=True
    def settimeout(self,value):self.timeout=value
    def recv(self,size):
        self.receives+=1;self.clock.now+=.25
        event=self.events.pop(0) if self.events else socket.timeout()
        if isinstance(event,Exception):raise event
        return event
    def send(self,*_):raise AssertionError('Passive gate transmitted data')
    def sendall(self,*_):raise AssertionError('Passive gate transmitted data')


class DisarmedStreamTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=Path(__file__).resolve().parents[1]/'mavros_live_runs/20261108/mavlink_v20.py'
        spec=importlib.util.spec_from_file_location('disarmed_fixture_wire',path)
        cls.wire=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.wire)

    def heartbeat(self,armed=False,system=1,component=1,v1=False):
        encoder=self.wire.MAVLink(None,srcSystem=system,srcComponent=component)
        return self.wire.MAVLink_heartbeat_message(2,3,128 if armed else 0,0,3,3).pack(encoder,force_mavlink1=v1)

    def check(self,events):
        clock=Clock();stream=ReceiveOnly(events,clock);console=io.StringIO()
        self.addCleanup(lambda:self.assertTrue(stream.closed))
        with patch.object(disarmed.socket,'create_connection',return_value=stream) as connect, \
             patch.object(disarmed.time,'monotonic',clock),redirect_stdout(console):
            disarmed.check()
        connect.assert_called_once_with(('127.0.0.1',5760),timeout=2)
        return console.getvalue(),stream

    def assert_decode_error(self,data,reason):
        # disarmed.check loads another instance of the actual generated module;
        # its exception class is distinct from the fixture module's MAVError.
        with self.assertRaises(Exception) as error:self.check([data])
        self.assertEqual(type(error.exception).__name__,'MAVError')
        self.assertRegex(str(error.exception),reason)

    def test_fragmented_valid_disarmed_heartbeat(self):
        packet=self.heartbeat()
        output,stream=self.check([packet[i:i+1] for i in range(len(packet))])
        self.assertIn('DISARMED heartbeat received',output)
        self.assertEqual(stream.receives,len(packet))

    def test_fragmented_armed_heartbeat_is_rejected(self):
        packet=self.heartbeat(armed=True)
        with self.assertRaisesRegex(RuntimeError,'FCU ARMED'):
            self.check([packet[:4],packet[4:11],packet[11:]])

    def test_partial_packet_then_eof_is_not_success(self):
        with self.assertRaisesRegex(RuntimeError,'FCU disconnected'):
            self.check([self.heartbeat()[:-1],b''])

    def test_silence_or_partial_packet_times_out_without_sleep(self):
        for prefix in ([],[self.heartbeat()[:8]]):
            with self.subTest(prefix=bool(prefix)):
                with self.assertRaisesRegex(RuntimeError,'No DISARMED heartbeat'):
                    self.check(prefix)

    def test_connection_failures_propagate_without_fallback(self):
        for error in (socket.timeout('connect timeout'),ConnectionRefusedError('refused')):
            with self.subTest(error=type(error).__name__), \
                 patch.object(disarmed.socket,'create_connection',side_effect=error):
                with self.assertRaises(type(error)):disarmed.check()

    def test_malformed_prefix_before_valid_packet_fails_strictly(self):
        self.assert_decode_error(b'console-prefix\n'+self.heartbeat(),'invalid MAVLink prefix')

    def test_bad_crc_before_valid_packet_fails_strictly(self):
        corrupt=bytearray(self.heartbeat());corrupt[-1]^=0xff
        self.assert_decode_error(bytes(corrupt)+self.heartbeat(),'invalid MAVLink CRC')

    def test_bad_data_is_a_codec_robust_mode_fixture_only(self):
        parser=self.wire.MAVLink(None)
        self.assertFalse(parser.robust_parsing)
        parser.robust_parsing=True  # Fixture only; never patch production gate.
        messages=parser.parse_buffer(b'junk-prefix\n'+self.heartbeat())
        self.assertIn('BAD_DATA',[m.get_type() for m in messages])
        self.assertEqual(sum(m.get_type()=='HEARTBEAT' for m in messages),1)

    def test_unexpected_source_ids_currently_accepted_not_identity_proof(self):
        for system,component in ((42,1),(1,42),(255,190)):
            with self.subTest(system=system,component=component):
                packet=self.heartbeat(system=system,component=component)
                decoded=self.wire.MAVLink(None).parse_buffer(packet)[0]
                self.assertEqual((decoded.get_srcSystem(),decoded.get_srcComponent()),(system,component))
                output,_=self.check([packet])
                self.assertIn('DISARMED heartbeat received',output)

    def test_v1_currently_accepted_not_mavlink2_proof(self):
        packet=self.heartbeat(v1=True)
        self.assertEqual(packet[0],254)
        output,_=self.check([packet])
        self.assertIn('DISARMED heartbeat received',output)

    def test_other_valid_message_is_not_a_heartbeat(self):
        packet=self.wire.MAVLink_timesync_message(0,123).pack(self.wire.MAVLink(None))
        with self.assertRaisesRegex(RuntimeError,'FCU disconnected'):
            self.check([packet,b''])
        output,_=self.check([packet,self.heartbeat()])
        self.assertIn('DISARMED heartbeat received',output)


if __name__=='__main__':unittest.main()
