"""Offline source-handoff regressions, never live hardware/GPU verification."""
import ast
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from linux_gpu import run_camera
from linux_gpu.analyze_camera import union

class CameraHandoffTests(unittest.TestCase):
    def test_unknown_or_software_context_cannot_be_hardware_candidate(self):
        for identity in ('','renderer=llvmpipe (LLVM)\nversion=4.5',
                         'renderer=softpipe\nversion=4.5', 'renderer=NVIDIA GPU'):
            self.assertTrue(run_camera.hardware_status(identity,'GL_RENDERER = NVIDIA GPU').startswith('HOLD'))
        self.assertTrue(run_camera.hardware_status('renderer=NVIDIA GPU\nversion=4.5','missing GL renderer').startswith('HOLD'))

    def test_enumerated_software_device_does_not_override_actual_hardware_context(self):
        verdict=run_camera.hardware_status('renderer=NVIDIA GPU\nversion=4.5',
            'Enumerated EGL_MESA_device_software\nGL_RENDERER = NVIDIA GPU')
        self.assertTrue(verdict.startswith('HARDWARE CANDIDATE'))
        self.assertNotIn('PASS',verdict)

    def test_prepare_preserves_inputs_and_remains_closed_without_runtime(self):
        with tempfile.TemporaryDirectory() as directory,patch.object(run_camera,'INTEGRATION',Path(directory)):
            run=run_camera.prepare('20261111-unit1','dri','camera:test','sitl:test',30)
            model=run/'models/world.sdf'
            self.assertEqual(model.read_bytes(),run_camera.verified_file('models/patched_c_world.sdf').read_bytes())
            decision=json.loads((run/'PREPARATION.json').read_text())
            self.assertFalse(decision['execution_allowed'])
            self.assertFalse(decision['forwarding_enabled'])
            self.assertEqual((run/'artifacts/camera.parm').read_text(),'SERIAL0_PROTOCOL 2\nLOG_DISARMED 1\n')
            with self.assertRaises(ValueError):run_camera.prepare('20261111-unit1','dri','camera:test','sitl:test',30)

    def test_invalid_duration_mode_and_identity_are_rejected(self):
        for duration in (float('nan'),float('inf'),0,61):
            with self.assertRaises(ValueError):run_camera.prepare('20261111-unit2','dri','camera:test','sitl:test',duration)
        with self.assertRaises(ValueError):run_camera.prepare('../../other','dri','camera:test','sitl:test',30)
        with self.assertRaises(ValueError):run_camera.prepare('20261111-unit2','unknown','camera:test','sitl:test',30)

    def test_nested_scene_spans_are_union_not_double_counted(self):
        self.assertEqual(union([(0,100),(10,40),(45,60),(110,120)]),110)
        self.assertEqual(union([]),0)

    def test_passive_heartbeat_uses_real_codec_and_never_sends(self):
        import importlib.util
        from linux_gpu import disarmed
        file=Path(__file__).resolve().parents[1]/'mavros_live_runs/20261108/mavlink_v20.py'
        spec=importlib.util.spec_from_file_location('handoff_test_wire',file)
        wire=importlib.util.module_from_spec(spec);spec.loader.exec_module(wire)
        class ReceiveOnly:
            def __init__(self,data):self.data=data
            def __enter__(self):return self
            def __exit__(self,*_):pass
            def settimeout(self,*_):pass
            def recv(self,*_):return self.data
            def send(self,*_):raise AssertionError('Passive check transmitted data')
            def sendall(self,*_):raise AssertionError('Passive check transmitted data')
        for mode in (0,128):
            payload=wire.MAVLink_heartbeat_message(2,3,mode,0,3,3).pack(wire.MAVLink(None))
            with patch.object(disarmed.socket,'create_connection',return_value=ReceiveOnly(payload)):
                if mode:
                    with self.assertRaisesRegex(RuntimeError,'ARMED'):disarmed.check()
                else:disarmed.check()

if __name__=='__main__':unittest.main()
