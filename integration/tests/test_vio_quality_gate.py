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
    def test_one_rejected_frame_exhausts_500ms_margin_at_10hz(self):
        self.ready()
        self.assertFalse(self.g.assess(1.4, 1.4, **dict(self.kw, audit_ok=False)))
        for t in (1.5, 1.6, 1.7):
            self.assertFalse(self.g.assess(t, t, **self.kw))
        self.assertTrue(self.g.assess(1.8, 1.8, **self.kw))
        self.assertEqual(round((1.8 - 1.3) * 1e9), 500_000_000)

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
    def test_audit_rejection_is_not_labelled_ekf(self):
        # run 20261117-flightfea1c93152a2, t=41.4: audit failed, so the flow was
        # never fused (ekf_ok False too) and the gate reported 'EKF not ready'.
        self.ready()
        self.assertFalse(self.g.assess(1.4, 2, **dict(self.kw, audit_ok=False, ekf_ok=False)))
        self.assertEqual(self.g.reason, 'distributed flow disagreement/unavailable')
        self.setUp(); self.ready()
        self.assertFalse(self.g.assess(1.4, 2, **dict(self.kw, ekf_ok=False)))   # audit ok, EKF rejected
        self.assertEqual(self.g.reason, 'EKF not ready')
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


@unittest.skipIf(np is None, 'NumPy required')
class GateToleranceTests(unittest.TestCase):
    """Audit C2 opt-in: one image-quality rejection does not cost readiness."""
    def setUp(self):
        self.g = QualityGate(tolerate_single_rejection=True)
        self.kw = dict(imu_age=.001, frame_age=.01, clock_age=.01, max_imu_gap=.001,
                       inliers=50, contrast=40, covariance=np.eye(9)*.01,
                       state_finite=True, audit_ok=True, ekf_ok=True, estimator_healthy=True)
        # What run_vio_stream passes when the frontend loses its features:
        # no flow, so nothing was fused and ekf_ok is False too.
        self.lost = dict(self.kw, inliers=0, ekf_ok=False)

    def ready(self):
        for t in [1., 1.15, 1.3]:
            result = self.g.assess(t, t, **self.kw)
        self.assertTrue(result)

    def test_default_is_off_and_unchanged(self):
        g = QualityGate()
        for t in [1., 1.15, 1.3]:
            g.assess(t, t, **self.kw)
        self.assertFalse(g.assess(1.4, 1.4, **self.lost))
        self.assertFalse(g.assess(1.5, 1.5, **self.kw))
        self.assertEqual(g.count, 1)

    def test_single_image_rejection_is_withheld_then_resumes(self):
        self.ready()
        self.assertFalse(self.g.assess(1.4, 1.4, **self.lost))
        self.assertFalse(self.g.ready)              # nothing may be published for it
        self.assertTrue(self.g.suspended)
        self.assertTrue(self.g.assess(1.5, 1.5, **self.kw))
        # Last publication 1.3 -> next 1.5: 200 ms instead of 500 ms.
        self.assertEqual(round((1.5 - 1.3) * 1e9), 200_000_000)
        self.assertEqual(self.g.tolerated, 1)
        self.assertTrue(self.g.transitions[-2]['tolerated'])

    def test_each_image_reason_is_tolerable(self):
        for change in (dict(inliers=0), dict(contrast=0), dict(audit_ok=False),
                       dict(consistency_ok=False)):
            with self.subTest(change):
                self.setUp(); self.ready()
                self.assertFalse(self.g.assess(1.4, 1.4, **dict(self.kw, ekf_ok=False, **change)))
                self.assertTrue(self.g.suspended)
                self.assertTrue(self.g.assess(1.5, 1.5, **self.kw))

    def test_two_consecutive_rejections_block_as_before(self):
        self.ready()
        self.assertFalse(self.g.assess(1.4, 1.4, **self.lost))
        self.assertFalse(self.g.assess(1.5, 1.5, **self.lost))
        self.assertFalse(self.g.suspended)
        self.assertEqual(self.g.count, 0)
        for t in (1.6, 1.7, 1.8):
            self.assertFalse(self.g.assess(t, t, **self.kw))
        self.assertTrue(self.g.assess(1.9, 1.9, **self.kw))

    def test_non_image_failures_are_never_tolerated(self):
        for change in (dict(imu_age=.1), dict(frame_age=.4), dict(clock_age=-.1),
                       dict(max_imu_gap=.263), dict(frame_gap=.4), dict(state_finite=False),
                       dict(covariance=-np.eye(9)), dict(ekf_ok=False),
                       dict(inliers=0, ekf_ok=False, covariance=-np.eye(9)),   # image + covariance
                       dict(inliers=0, ekf_ok=False, imu_age=.1),             # image + IMU
                       dict(inliers=0, ekf_ok=False, estimator_healthy=False),
                       dict(inliers=0, ekf_ok=False, estimator_healthy=None)):
            with self.subTest(change):
                self.setUp(); self.ready()
                self.assertFalse(self.g.assess(1.4, 1.4, **dict(self.kw, **change)))
                self.assertFalse(self.g.suspended)
                self.assertEqual(self.g.count, 0)
                self.assertFalse(self.g.assess(1.5, 1.5, **self.kw))

    def test_timestamp_reset_and_external_block_are_never_tolerated(self):
        self.ready()
        self.assertFalse(self.g.assess(1.3, 1.31, **self.lost))
        self.assertFalse(self.g.suspended)
        self.setUp(); self.ready()
        self.assertFalse(self.g.assess(1.4, 1.4, **self.lost))
        self.g.block('stale at publication', 1.45, 1.45)
        self.assertFalse(self.g.suspended)
        self.assertFalse(self.g.assess(1.5, 1.5, **self.kw))

    def test_no_tolerance_before_first_readiness(self):
        self.g.assess(1., 1., **self.kw)
        self.g.assess(1.15, 1.15, **self.kw)
        self.assertFalse(self.g.assess(1.2, 1.2, **self.lost))
        self.assertFalse(self.g.suspended)
        self.assertEqual(self.g.count, 0)

    def test_alternating_failures_block_until_rearmed(self):
        self.ready()
        self.assertFalse(self.g.assess(1.4, 1.4, **self.lost))
        self.assertTrue(self.g.assess(1.5, 1.5, **self.kw))
        # Second failure before re-arming: full block, as without the option.
        self.assertFalse(self.g.assess(1.6, 1.6, **self.lost))
        self.assertFalse(self.g.suspended)
        self.assertEqual(self.g.count, 0)

    def test_rearms_after_normal_readiness_criterion(self):
        self.ready()
        self.assertFalse(self.g.assess(1.4, 1.4, **self.lost))
        for t in (1.5, 1.6, 1.7):
            self.assertTrue(self.g.assess(t, t, **self.kw))
            self.assertTrue(self.g.recovering)      # 3 frames but only 200 ms
        self.assertTrue(self.g.assess(1.8, 1.8, **self.kw))
        self.assertFalse(self.g.recovering)
        self.assertFalse(self.g.assess(1.9, 1.9, **self.lost))
        self.assertTrue(self.g.suspended)
        self.assertTrue(self.g.assess(2.0, 2.0, **self.kw))
        self.assertEqual(self.g.tolerated, 2)


if __name__ == '__main__':
    unittest.main()
