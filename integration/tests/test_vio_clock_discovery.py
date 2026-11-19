"""Regression for the camera/clock and DDS sequence in clockdiag020331."""
import unittest
from unittest.mock import Mock, patch

import numpy as np

import run_vio_stream as vio
from vio_quality_gate import QualityGate


class ClockDiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.patch = patch.multiple(vio.time, monotonic=lambda: self.now, sleep=self.advance)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def advance(self, duration):
        self.now += duration

    def clock(self, stamp):
        return dict(t=stamp, arrival_monotonic_s=self.now)

    def assess(self, gate, stamp, contrast=46):
        return gate.assess(stamp, self.now, imu_age=0, frame_age=.01, clock_age=.001,
                           max_imu_gap=.001, inliers=154 if contrast>=8 else 0, contrast=contrast,
                           covariance=np.eye(9)*.001, state_finite=True, audit_ok=True, ekf_ok=True)

    def test_clockdiag_311_to_322_preserves_quality_and_requires_subscriber(self):
        gate = QualityGate()
        for stamp in (30.8, 30.9, 31., 31.1):
            ready = self.assess(gate, stamp)
        self.assertTrue(ready)

        for stamp in (31.2, 31.3):
            initial = self.clock(stamp-.001)
            real_sample = self.clock(stamp+.001)
            buffer = Mock()
            buffer.latest.side_effect = [initial, real_sample]
            selected = vio.await_frame_clock(buffer, stamp, initial, .04, self.now)
            self.assertIs(selected, real_sample)
            self.assertTrue(vio.clock_status(stamp, selected)['valid'])
            self.assertTrue(self.assess(gate, stamp))

        # Replay genuine DDS loss and verify the source stays publishable
        # immediately when both subscriptions actually return.
        for stamp in (31.4, 31.5, 31.6, 31.7, 31.8, 31.9, 32., 32.1):
            self.assertTrue(self.assess(gate, stamp))
            scores = (1, 0) if stamp in (31.7, 32.1) else (2, 1)
            reason = vio.publication_block(gate, vio.clock_status(stamp, self.clock(stamp)),
                                           *scores, 2, 1)
            self.assertEqual(reason, None if scores == (2, 1) else 'waiting for downstream DDS discovery')
            self.assertTrue(gate.ready)
        self.assertFalse(self.assess(gate, 32.2, contrast=.962))
        for stamp in (32.3, 32.4, 32.5):
            self.assertFalse(self.assess(gate, stamp))
        self.assertTrue(self.assess(gate, 32.6))

    def test_lag_cannot_wait_past_remaining_budget_or_replace_a_sample(self):
        initial = self.clock(31.199)
        buffer = Mock()
        buffer.latest.return_value = initial
        started = self.now
        self.assertIs(vio.await_frame_clock(buffer, 31.2, initial, .155, started), initial)
        self.assertLessEqual(self.now-started, .002001)
        self.assertFalse(vio.clock_status(31.2, initial)['valid'])
        self.assertIs(vio.await_frame_clock(buffer, 31.2, initial, .157, self.now), initial)

    def test_epoch_validation_and_publication_freshness_still_fail_closed(self):
        buffer = Mock()
        buffer.latest.side_effect = ValueError('clock epoch changed')
        with self.assertRaisesRegex(ValueError, 'epoch changed'):
            vio.await_frame_clock(buffer, 31.2, self.clock(31.199), .04, self.now)
        gate = QualityGate()
        for stamp in (30.8, 30.9, 31., 31.1):
            self.assess(gate, stamp)
        check = vio.clock_status(31.1, self.clock(31.3), limit=vio.PUBLICATION_MAX_AGE_S)
        self.assertEqual(vio.publication_block(gate, check, 1, 0, 2, 1), 'stale at publication')
        self.assertFalse(gate.ready)
