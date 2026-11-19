"""Regression for LAND enlargement after weak/failed optical flow."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vio_frontend_recovery import prefer_redetection


class TestEarlyRedetection(unittest.TestCase):
    def test_climb_shrinkage_preserved(self):
        self.assertTrue(prefer_redetection(26, 15, .80))

    def test_land_enlargement_weak_support(self):
        self.assertTrue(prefer_redetection(26, 15, 1.3278855523169943))

    def test_land_enlargement_after_failed_frame(self):
        self.assertTrue(prefer_redetection(0, 15, 1.48355746949618))

    def test_no_extra_recovery_when_support_stable(self):
        self.assertFalse(prefer_redetection(129, 15, 1.2485997975553502))

    def test_near_unit_scale_never_early(self):
        self.assertFalse(prefer_redetection(0, 15, 1.05))
        self.assertFalse(prefer_redetection(0, 15, .95))

    def test_missing_support_or_scale(self):
        self.assertFalse(prefer_redetection(None, 15, 1.4))
        self.assertFalse(prefer_redetection(10, 15, None))

    def test_recorded_land_pair_passes_with_expanded_candidate_pool(self):
        import os
        from unittest.mock import patch

        import numpy as np

        integration = Path(__file__).resolve().parents[1]
        pair = integration / ('phase1_runs/20261119-landverify054946/evidence/flight-source/'
                              'frontend-rejection-57500000000.npz')
        if not pair.exists():
            self.skipTest('Recorded LAND pair is unavailable')
        sys.path.insert(0, os.environ.get('VIO_SOURCE_DIR', str(integration.parent/'vio')))
        from vio_frontend import FlowFrontend
        from vio_frontend_recovery import redetect
        from vio_quality_gate import judge_distributed_flow, measure_distributed_flow
        from vio_scale_compensation import compensate_previous

        with np.load(pair) as inputs:
            previous, gray = inputs['previous'], inputs['current']
            scale = float(inputs['scale'])
            gyro, rotation = inputs['gyro_frd'], inputs['body_to_ned']
            height = float(inputs['depth_m'])*max(float(rotation[2, 2]), .5)
            compensated = compensate_previous(previous, scale)
            frontend = FlowFrontend(205.4696)
            self.assertTrue(prefer_redetection(26, frontend.min_inliers, scale))
            frontend.max_corners = 80
            self.assertIsNone(redetect(frontend, compensated, gray, .1, height, gyro,
                                       rotation, scale, float('inf')))
            frontend.max_corners = 200
            result = redetect(frontend, compensated, gray, .1, height, gyro,
                              rotation, scale, float('inf'))
            self.assertIsNotNone(result)
            self.assertGreaterEqual(result.n_inliers, 29)
            audit = judge_distributed_flow(measure_distributed_flow(previous, gray, .1, scale),
                                           height, frontend.fx, gyro, rotation, result.v_xy)
            self.assertTrue(audit['ok'], audit)
            self.assertLess(audit['disagreement_px'], .05)
            with patch('vio_frontend_recovery.time.monotonic', return_value=26.):
                self.assertIsNone(redetect(frontend, compensated, gray, .1, height, gyro,
                                           rotation, scale, 25.))


if __name__ == '__main__':
    unittest.main()
