"""Offline diagnostic/preflight fixtures, never autopilot acceptance evidence."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.append(str(Path(__file__).resolve().parents[1]))
from analyze_ekf3_readiness import covariance_diagnostic
from preflight_ekf3 import parse_profile,check_profile,EXPECTED,PORTS,build,assess_offset


class ReadinessTests(unittest.TestCase):
    def test_profile_and_version_mismatch(self):
        self.assertEqual(check_profile(EXPECTED,PORTS),[])
        self.assertTrue(check_profile(dict(EXPECTED,GPS1_TYPE=0),PORTS))
        self.assertTrue(check_profile(dict(EXPECTED,EK3_SRC1_VELZ=6),PORTS))
    def test_duplicate_nonfinite_and_port_collision(self):
        for text in ['GPS_TYPE 0\nGPS_TYPE 1','VISO_SCALE nan','VISO_SCALE inf']:
            with self.assertRaises(ValueError):parse_profile(text)
        self.assertTrue(check_profile(EXPECTED,dict(PORTS,monitor=PORTS['vio'])))
    def test_dry_run_cannot_enable_forwarding(self):
        result=build(EXPECTED,[])
        self.assertEqual(result['decision'],'HOLD');self.assertFalse(result['forwarding_allowed'])
    def test_negative_timesync_offset_is_not_false_pass(self):
        huge=((100+100-2*(100+10**9)) & ((1<<64)-1))//2
        self.assertEqual(assess_offset([{'observed_offset_ns':huge}],10**9),'FAIL')
        self.assertEqual(assess_offset([{'observed_offset_ns':10**9}],-10**9),'PASS')
        self.assertEqual(assess_offset([],10**9),'FAIL')
    def test_covariance_conservative_diagnostic(self):
        r=covariance_diagnostic(np.zeros((5,2)),np.tile(np.eye(2),(5,1,1)))
        self.assertEqual(r['nees_mean'],0);self.assertIn('NOT VERIFIED',r['calibration'])
        for p in [np.zeros((5,2,2)),np.tile(np.diag([-1,1]),(5,1,1))]:
            with self.assertRaises(ValueError):covariance_diagnostic(np.zeros((5,2)),p)


if __name__=='__main__':unittest.main()
