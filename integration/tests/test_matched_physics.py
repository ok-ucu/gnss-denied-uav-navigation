"""Offline fixtures only: no simulator, socket, live VIO or FCU sender."""
import ctypes
import json
from pathlib import Path
import struct
import tempfile
import unittest
from analyze_navigation_clocks import analyze
from matched_physics.analyze_physics_capture import packets,expected_json,transform,validate_physics
from matched_physics.analyze_clock_evidence import progress


class CaptureTests(unittest.TestCase):
    def test_enu_ned_flu_frd_basis_and_quaternion_direction(self):
        identity=dict(position=[0,0,0],quaternion_xyzw=[0,0,0,1])
        root=dict(position=[1,2,3],quaternion_xyzw=[0,0,0,1])
        world=transform('0 0 0 3.141592653589793 0 1.5707963267948966')
        body=transform('0 0 0 3.141592653589793 0 0')
        p,r=expected_json(root,identity,identity,world,body)
        for actual,expected in zip(p,[2,1,-3]):self.assertAlmostEqual(actual,expected)
        expected=[[0,-1,0],[1,0,0],[0,0,1]]
        for actual,wanted in zip(sum(r,[]),sum(expected,[])):self.assertAlmostEqual(actual,wanted)
    def test_nan_and_nonunit_quaternion_are_rejected(self):
        good=dict(timestamp=1.,position=[0,0,0],velocity=[0,0,0],quaternion=[1,0,0,0],imu=dict(gyro=[0,0,0],accel_body=[0,0,-9.8]))
        validate_physics(good)
        for key,value in [('timestamp',float('nan')),('quaternion',[2,0,0,0]),('position',[float('inf'),0,0])]:
            with self.subTest(key=key),self.assertRaises(ValueError):validate_physics(dict(good,**{key:value}))
    def test_actual_pcap_layout_and_nanosecond_precision(self):
        payload=b'\n{"timestamp":1.25}\n'
        udp=struct.pack('!HHHH',9002,45666,8+len(payload),0)+payload
        ip=struct.pack('!BBHHHBBH4s4s',0x45,0,20+len(udp),0,0,64,17,0,b'\x7f\x00\x00\x01',b'\x7f\x00\x00\x01')+udp
        frame=b'\0'*12+b'\x08\x00'+ip
        header=b'\x4d\x3c\xb2\xa1'+struct.pack('<HHIIII',2,4,0,0,262144,1)
        data=header+struct.pack('<IIII',123,987654321,len(frame),len(frame))+frame
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fixture.pcap';path.write_bytes(data)
            row=list(packets(path))[0]
            self.assertEqual(row['receive_unix_ns'],123987654321)
            self.assertEqual(row['payload'],payload)
            self.assertEqual((row['source_port'],row['destination_port']),(9002,45666))
            path.write_bytes(data[:-1])
            with self.assertRaises(ValueError):list(packets(path))
    def test_444_numeric_boolean_parser_does_not_parse_json_true(self):
        libc=ctypes.CDLL(None);libc.strtoull.argtypes=[ctypes.c_char_p,ctypes.c_void_p,ctypes.c_int];libc.strtoull.restype=ctypes.c_ulonglong
        self.assertEqual(libc.strtoull(b'true',None,10),0)
        self.assertEqual(libc.strtoull(b'false',None,10),0)
        self.assertEqual(libc.strtoull(b'1',None,10),1)


class ClockMetadataTests(unittest.TestCase):
    def fixture(self,path):
        gz=[dict(source_ns=i*10_000_000,host_ns=20*10**9+i*10_000_000) for i in range(8)]
        fcu=[dict(kind='timesync',source_ns=10*10**9+i*10_000_000,host_ns=20*10**9+i*10_000_000,rtt_ns=1000) for i in range(1,7)]
        (path/'gazebo_clock.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in gz))
        (path/'fcu_clock.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in fcu))
    def test_default_metadata_is_unknown_and_old_evidence_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory);self.fixture(path)
            prior=path/'diagnostic_anchors.json';prior.write_text('existing evidence')
            result=analyze(path)
            self.assertIsNone(result['independent_physics'])
            self.assertIn('NOT ASSESSED',result['pause_resume'])
            self.assertIsNone(result['all_fcu_disarmed'])
            self.assertFalse(result['uncertainty_bound_verified'])
            self.assertEqual(prior.read_text(),'existing evidence')
    def test_explicit_json_topology_is_static_scope_and_remains_hold(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory);self.fixture(path)
            result=analyze(path,dict(fcu_model='JSON',shared_plugin_endpoint=True,scope='static configured path',pause_resume='archived response observed'))
            self.assertFalse(result['independent_physics'])
            self.assertEqual(result['decision'],'HOLD')
            output=path/'new.json';analyze(path,anchors_output=output)
            with self.assertRaises(FileExistsError):analyze(path,anchors_output=output)
    def test_pause_window_without_samples_cannot_prove_zero_progress(self):
        rows=[dict(source_ns=0,host_ns=0),dict(source_ns=1_000_000_000,host_ns=10_000_000_000)]
        result=progress(rows,3,5)
        self.assertEqual(result['count'],0)
        self.assertIsNone(result['progress_s'])

    def test_missing_clock_samples_return_hold(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)
            (path/'gazebo_clock.jsonl').write_text('')
            (path/'fcu_clock.jsonl').write_text('')
            result=analyze(path)
            self.assertEqual(result['decision'],'HOLD')
            self.assertFalse(result['execution_allowed'])
