"""Offline fixtures. These tests cannot authorize live forwarding or flight."""
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from physics_clock_mapping import AppliedClockMapping,PATCHED_SHA
from vio_estimation_inputs import fixed_mount,vertical_inputs
from phase1_transport import fcu_allowed,common_watermark
from launch_phase1 import validate_run_id
from analyze_phase1_run import evaluate


def prepared():
    m=AppliedClockMapping('fixture-epoch',PATCHED_SHA)
    for i in range(15):m.feed(i*20_000_000,1_000_000_000+i*19_999_000,10_000_000_000+i*60_000_000,
                            parsed_no_time_sync=1,use_time_sync=0)
    for i in range(2,12):
        a=m.anchors[i]
        assert m.crosscheck(a['target_ns'],a['host_ns'],a['host_ns']+1_000_000)
    return m

class AppliedClockTests(unittest.TestCase):
    def test_bracket_envelope_is_not_receive_regression(self):
        m=prepared();v=m.map(250_000_000,10_850_000_000,'fixture-epoch')
        self.assertLessEqual(v['lower_fcu_ns'],v['fcu_ns'])
        self.assertLessEqual(v['fcu_ns'],v['upper_fcu_ns'])
        self.assertLessEqual(v['uncertainty_ns'],20_000_000)
        self.assertEqual(v['acquisition_ns'],250_000_000)
    def test_no_proof_no_extrapolation_expiry_and_epoch(self):
        m=AppliedClockMapping('unit',PATCHED_SHA)
        with self.assertRaises(ValueError):m.map(1,1,'unit')
        m=prepared()
        for stamp,host,epoch in [(300_000_000,10_850_000_000,'fixture-epoch'),
                                  (250_000_000,12_000_000_000,'fixture-epoch'),
                                  (250_000_000,10_850_000_000,'changed')]:
            with self.assertRaises(ValueError):m.map(stamp,host,epoch)
    def test_clock_reset_and_bad_parser_latch(self):
        for wrong in ['reset','parser']:
            m=prepared()
            with self.assertRaises(ValueError):m.feed(1 if wrong=='reset' else 300_000_000,
                2_000_000_000,11_000_000_000,parsed_no_time_sync=0 if wrong=='parser' else 1,use_time_sync=0)
            self.assertTrue(m.broken)
            with self.assertRaises(ValueError):m.map(250_000_000,10_850_000_000,'fixture-epoch')
    def test_remote_domain_mismatch_latches(self):
        m=prepared()
        with self.assertRaises(ValueError):m.crosscheck(900_000_000_000,10_300_000_000,10_400_000_000)
        self.assertFalse(m.verified)
    def test_wrong_binary_is_not_provenance(self):
        with self.assertRaises(ValueError):AppliedClockMapping('fixture','baseline')

class NavigationInputTests(unittest.TestCase):
    def test_height_uses_only_z_and_imu_rotation(self):
        h,camera=vertical_inputs(.195,.001,np.eye(3),[.018,.01,.103])
        self.assertAlmostEqual(h,.194)
        self.assertAlmostEqual(camera,.091)
    def test_bad_calibration_and_nonfinite_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'cal.json'
            p.write_text(json.dumps(dict(mount_status='PASS',max_angles_deg=[0,2,0],final_lever_body_flu_m=[0,0,-.1])))
            with self.assertRaises(ValueError):fixed_mount(p)
        with self.assertRaises(ValueError):vertical_inputs(float('nan'),0,np.eye(3),[0,0,.1])

class WirePolicyTests(unittest.TestCase):
    def test_launcher_rejects_existing_service_or_path_names(self):
        self.assertEqual(validate_run_id('20261110-check1'),'20261110-check1')
        for value in ('gnss-phase1-sitl','../../other','20261110-other/name','20261110-ABC',''):
            with self.assertRaises(ValueError):validate_run_id(value)
    def test_common_watermark_advances_when_ros_lags_newest_anchor(self):
        anchors=[dict(source_ns=s,host_ns=s+100) for s in (100,120,140)]
        self.assertEqual(common_watermark(anchors,139)['source_ns'],120)
        self.assertEqual(common_watermark(anchors,140)['source_ns'],140)
        self.assertIsNone(common_watermark(anchors,99))
    def test_capture_policy_has_no_vision_or_control_route(self):
        self.assertTrue(fcu_allowed('TIMESYNC'))
        for kind in ('VISION_POSITION_ESTIMATE','VISION_SPEED_ESTIMATE','ODOMETRY','COMMAND_LONG',
                     'SET_POSITION_TARGET_LOCAL_NED','SET_MODE','PARAM_SET','HEARTBEAT'):
            self.assertFalse(fcu_allowed(kind))

class EvaluationTests(unittest.TestCase):
    def test_reset_gt_cannot_be_interpolated_as_a_trajectory(self):
        rows=[dict(t=t,state=[0]*9) for t in (.01,.02,.03)]
        poses=[dict(t=t) for t in (0,.02,.01,.04)]
        result=evaluate(rows,poses,0)
        self.assertEqual(result['status'],'NOT TESTED')
    def test_no_gt_extrapolation(self):
        rows=[dict(t=t,state=[0]*9) for t in (.01,.02,.03)]
        poses=[dict(t=t) for t in (.02,.03,.04)]
        self.assertEqual(evaluate(rows,poses,0)['status'],'NOT TESTED')

if __name__=='__main__':unittest.main()
