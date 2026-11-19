"""Replay the recorded LAND timing and distinguish sim readiness from host expiry."""
import unittest

import numpy as np

from run_vio_stream import clock_status, publication_block
from vio_latency_contract import SOURCE_PUBLICATION_MAX_AGE_S
from vio_quality_gate import QualityGate


class LandRecoveryTimingTests(unittest.TestCase):
    def replay(self, failed):
        gate = QualityGate()
        emitted = []
        for i in range(512, 526):
            stamp = i / 10
            loss = failed.get(i)
            ready = gate.assess(
                stamp, i * .1, imu_age=0., frame_age=.01, clock_age=.004,
                max_imu_gap=.001, inliers=0 if loss == 'tracking' else 194,
                contrast=32., covariance=np.eye(9)*.001, state_finite=True,
                audit_ok=loss != 'tracking', ekf_ok=True)
            if gate.count:
                clock = {'t': stamp + (.179 if loss == 'stale' else .050),
                         'arrival_monotonic_s': i * .1}
                check = clock_status(stamp, clock, now=i * .1,
                                     limit=SOURCE_PUBLICATION_MAX_AGE_S)
                if publication_block(gate, check, 2, 1, 2, 1) is None and ready:
                    emitted.append(i)
        return emitted

    def test_one_stale_frame_resumes_too_late_for_recorded_host_watchdog(self):
        self.assertEqual(self.replay({516: 'stale'}), [515, 520, 521, 522, 523, 524, 525])
        # landdiag061527, host-monotonic event stamps. The source sim stamps
        # 51.5 and 52.0 are intentionally not used to compute host age.
        last_callback = 39333464452205
        watchdog = 39333964916124
        source_resumed = 39333965716746
        resumed_callback = 39333966878193
        self.assertGreater(watchdog-last_callback, 500_000_000)
        self.assertLess(watchdog, source_resumed)
        self.assertLess(source_resumed, resumed_callback)

    def test_consecutive_stalls_and_tracking_loss_require_full_readiness(self):
        for first, second in [('stale', 'stale'), ('stale', 'tracking'),
                              ('tracking', 'stale'), ('tracking', 'tracking')]:
            with self.subTest(first=first, second=second):
                # A freshness miss while readiness is already rebuilding must
                # not count as one of the healthy recovery frames.
                result = self.replay({516: first, 517: second})
                self.assertEqual(result[0], 515)
                self.assertEqual(result[1], 521)
                self.assertEqual(result, sorted(set(result)))
