"""Replay captured CLIMB inputs without ROS, Docker or an FCU connection."""
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

from vio_frontend import FlowFrontend
from vio_frontend_recovery import redetect
from vio_quality_gate import judge_distributed_flow, measure_distributed_flow
from vio_scale_compensation import compensate_previous


class FrontendRecoveryTests(unittest.TestCase):
    def test_captured_climb_pairs_agree_with_spatial_audit(self):
        runs = Path(__file__).resolve().parents[1]/'phase1_runs'
        for run, expected in [('20261119-flightdiag01', 29), ('20261119-flightfinal022718', 33)]:
            path = runs/run/'evidence/flight-source/frontend-rejection-40300000000.npz'
            if not path.exists():
                self.skipTest('Captured runtime evidence is unavailable')
            with self.subTest(run=run), np.load(path) as data:
                rotation, gyro = data['body_to_ned'], data['gyro_frd']
                height = float(data['depth_m'])*max(float(rotation[2, 2]), .5)
                scale = float(data['scale'])
                prev, gray = data['previous'], data['current']
                warped = compensate_previous(prev, scale)
                frontend = FlowFrontend(205.4696)
                frontend.prev, frontend.prev_t = warped, float(data['previous_ns'])/1e9
                self.assertIsNone(frontend.process(gray, float(data['current_ns'])/1e9, height, gyro, rotation))
                result = redetect(frontend, warped, gray, .1, height, gyro, rotation, scale, float('inf'))
                self.assertIsNotNone(result)
                self.assertGreaterEqual(result.n_inliers, expected-2)
                audit = judge_distributed_flow(measure_distributed_flow(prev, gray, .1, scale),
                                               height, frontend.fx, gyro, rotation, result.v_xy)
                self.assertTrue(audit['ok'], audit)
                self.assertLess(audit['disagreement_px'], .1)

    def test_bad_images_geometry_and_timing_are_rejected(self):
        rng = np.random.default_rng(290)
        image = rng.integers(0, 256, (200, 300), dtype=np.uint8)
        unrelated = rng.integers(0, 256, image.shape, dtype=np.uint8)
        sparse = np.full_like(image, 127)
        sparse[50:110,90:140] = image[50:110,90:140]
        frontend = FlowFrontend(205.4696)

        def recover(a, b, dt=.1, height=.19, deadline=float('inf')):
            return redetect(frontend, a, b, dt, height, np.zeros(3), np.eye(3), 1., deadline)

        self.assertIsNotNone(recover(image, np.roll(image, 2, axis=1)))
        self.assertIsNone(recover(image, unrelated))
        self.assertIsNone(recover(image, np.full_like(image, 127)))
        self.assertIsNone(recover(image, cv2.GaussianBlur(image, (31,31), 8)))
        self.assertIsNone(recover(sparse, sparse))
        self.assertIsNone(recover(image, image, dt=0))
        self.assertIsNone(recover(image, image, height=0))
        self.assertIsNone(recover(image, image, deadline=time.monotonic()-1))
