"""Offline geometry/patch regressions. Actual parser C++ tests run separately."""
import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
CODE=ROOT/'isolated_blockers/20261109_a/code'
spec=importlib.util.spec_from_file_location('isolated_mount_math',CODE/'mount_math.py')
mount=importlib.util.module_from_spec(spec);spec.loader.exec_module(mount)

class MountTests(unittest.TestCase):
    def test_target_sensor_basis_and_nonzero_lever(self):
        identity=dict(position=[0,0,0],quaternion_xyzw=[0,0,0,1])
        gimbal=dict(position=[.1,-.01,-.12],quaternion_xyzw=[0,0,0,1])
        target=[[0,0,1],[0,1,0],[-1,0,0]]
        result=mount.measure_mount(identity,gimbal,identity,target,[0,0,0])
        self.assertTrue(mount.mount_pass([result]))
        self.assertEqual(result['lever_body_flu_m'],[.1,-.01,-.12])
    def test_empty_wrong_and_nonfinite_mount_never_pass(self):
        for samples in ([],[dict(angles_deg=[0,0,2])],[dict(angles_deg=[0,float('nan'),0])],
                        [dict(angles_deg=[0,0])],[dict(angles_deg=[-1,0,0])]):
            self.assertFalse(mount.mount_pass(samples))
        with self.assertRaises(ValueError):mount.mount_pass([],5)
    def test_nonunit_quaternion_is_rejected(self):
        for q in ([0,0,0,0],[0,0,0,2],[float('inf'),0,0,1]):
            with self.assertRaises(ValueError):mount.quaternion_matrix(q)
    def test_q_and_minus_q_are_equivalent(self):
        q=[0,0,math.sqrt(.5),math.sqrt(.5)]
        self.assertEqual(mount.quaternion_matrix(q),mount.quaternion_matrix([-x for x in q]))
    def test_recorded_bad_mount_is_still_a_failure(self):
        import json
        recorded=json.loads((ROOT/'matched_physics/results/camera_mount_review_20261109.json').read_text())
        self.assertFalse(mount.mount_pass([dict(angles_deg=recorded['max_deviation_deg'])]))

class ExperimentalPatchTests(unittest.TestCase):
    def test_instrumentation_only_touches_experimental_copy(self):
        original=(ROOT/'matched_physics/results/SIM_JSON_review_20261109.cpp').read_bytes()
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);src=root/'libraries/SITL';src.mkdir(parents=True)
            (src/'SIM_JSON.cpp').write_bytes(original)
            (src/'SIM_JSON.h').write_bytes((ROOT/'matched_physics/results/SIM_JSON_review_20261109.h').read_bytes())
            subprocess.run([sys.executable,'-B',str(CODE/'prepare_source.py'),str(root),'instrument'],check=True)
            # fix mode's patch artifact goes into its own code folder, so only
            # apply production transformation in real build; avoid writing it here.
            self.assertIn('GNSS_EXPERIMENT_TRACE',(src/'SIM_JSON.cpp').read_text())
            self.assertEqual((ROOT/'matched_physics/results/SIM_JSON_review_20261109.cpp').read_bytes(),original)

if __name__=='__main__':unittest.main()
