"""Synthetic gate regression tests, not live EKF/ROS evidence."""
from pathlib import Path
import sys
import unittest
import os
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, os.environ.get('VIO_SOURCE_DIR', str(Path(__file__).resolve().parents[2]/'vio')))
try:
    import numpy as np
    from vio_quality_gate import QualityGate, covariance_ok, distributed_flow, fresh, ekf_ready
except ImportError:
    np = None


@unittest.skipIf(np is None, 'NumPy required')
class GateTests(unittest.TestCase):
    def setUp(self):
        self.g = QualityGate()
        self.kw = dict(imu_age=.001, frame_age=.01, clock_age=.01, max_imu_gap=.001,
                       inliers=50, contrast=40, covariance=np.eye(9)*.01,
                       state_finite=True, audit_ok=True, ekf_ok=True)
    def ready(self):
        for t in [1., 1.15, 1.3]:
            result = self.g.assess(t, t, **self.kw)
        self.assertTrue(result)
    def test_readiness_and_immediate_block(self):
        self.ready()
        self.assertFalse(self.g.block('camera loss', 1.31, 2))
        self.assertFalse(self.g.assess(1.4, 2.1, **self.kw))
        self.assertFalse(self.g.assess(1.5, 2.2, **self.kw))
        self.assertTrue(self.g.assess(1.7, 2.3, **self.kw))
    def test_all_faults_fail_closed(self):
        for key, value in [('imu_age', .1), ('frame_age', .4), ('clock_age', -.1),
                           ('max_imu_gap', .263), ('inliers', 2), ('contrast', 0),
                           ('frame_gap', .4),
                           ('state_finite', False), ('audit_ok', False), ('ekf_ok', False),
                           ('frame_age', float('nan')), ('covariance', -np.eye(9))]:
            with self.subTest(key=key):
                self.setUp()
                self.ready()
                self.assertFalse(self.g.assess(1.4, 2, **dict(self.kw, **{key: value})))
    def test_timestamp_reset_requires_readiness(self):
        self.ready()
        self.assertFalse(self.g.assess(.5, 2, **self.kw))
        self.assertFalse(self.g.assess(.6, 2.1, **self.kw))
    def test_camera_gap_nanosecond_boundary(self):
        self.ready()
        self.assertTrue(self.g.assess(1.4,2,**dict(self.kw,frame_gap=.3000000000007276)))
        self.assertFalse(self.g.assess(1.5,2.1,**dict(self.kw,frame_gap=.30000001)))
    def test_internal_velocity_reset_is_not_normal_acceptance(self):
        from vio_core import VioEKF
        e = VioEKF(max_rejects=1)
        self.assertFalse(ekf_ready(e))
        e.update_velocity_xy([0, 0], 1)
        self.assertTrue(ekf_ready(e))
        accepted, _ = e.update_velocity_xy([10, 10], 1)
        self.assertTrue(accepted)  # Source reports acceptance after forced reset.
        self.assertEqual(e.n_resets, 1)
        self.assertFalse(ekf_ready(e))
    def test_nanosecond_boundary_not_millisecond_relaxation(self):
        self.assertTrue(fresh(.2000000000007276, .2))
        self.assertFalse(fresh(.20000001, .2))
        self.assertFalse(fresh(-.000001, .2))
        self.assertFalse(fresh(float('nan'), .2))
    def test_covariance(self):
        self.assertTrue(covariance_ok(np.zeros((9, 9)), 9))
        p = np.eye(9)*.1
        p[0, 1] = .02
        self.assertFalse(covariance_ok(p, 9))
        self.assertFalse(covariance_ok(np.full((9, 9), float('inf')), 9))
        self.assertFalse(covariance_ok(np.diag([.1]*3+[-1.]*3), 6, 100.))
        self.assertTrue(covariance_ok(np.diag([.1]*3+[100.]*3), 6, 100.))
    def test_distributed_translation(self):
        import cv2
        prev = np.random.default_rng(14).integers(0, 256, (480, 640), dtype=np.uint8)
        gray = cv2.warpAffine(prev, np.float32([[1, 0, 2], [0, 1, 0]]), (640, 480))
        audit = distributed_flow(prev, gray, .1, 1, 200, np.zeros(3), np.eye(3), [0, -.1])
        self.assertTrue(audit['ok'], audit)
        audit = distributed_flow(prev, gray, .1, 1, 200, np.zeros(3), np.eye(3), [.3, -.1])
        self.assertFalse(audit['ok'])
    def test_local_patch_not_background_proof(self):
        prev = np.zeros((480, 640), np.uint8)
        prev[180:260, 250:330] = np.random.default_rng(5).integers(0, 256, (80, 80), dtype=np.uint8)
        audit = distributed_flow(prev, prev, .1, 1, 200, np.zeros(3), np.eye(3), [0, 0])
        self.assertFalse(audit['ok'])


if __name__ == '__main__':
    unittest.main()
