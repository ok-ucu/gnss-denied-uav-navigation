"""Phase A reproduction (no production change): one rejected frame trips the watchdog.

Deterministic, offline. Drives the REAL QualityGate frame by frame at 10 Hz
simulation time and maps simulation time to host time with a fixed
simulation/host ratio. The transport watchdog lives inline in
phase1_transport.main() (condition `time.monotonic_ns()-last_odom[0]>500_000_000`,
fed by on_odom's callback-entry stamp), so it is modelled here and the model is
pinned against the source text below. This test documents CURRENT behaviour;
it must keep passing unchanged while the degraded mode is off.
"""
import re
import unittest
from pathlib import Path

import numpy as np

from vio_quality_gate import QualityGate

ROOT = Path(__file__).resolve().parents[1]
WATCHDOG_NS = 500_000_000
FRAME_S = .1


def run_timeline(bad_frames, ratio, frames=range(0, 30), period=FRAME_S):
    """Return (published sim stamps, first watchdog host time or None, gate transitions).

    Host time of a publication = sim time / ratio (a constant processing
    latency cancels in every host-age difference). The watchdog is evaluated
    continuously between publications, as the transport main loop polls every
    ~2 ms (spin_once(timeout_sec=.002)).
    """
    gate = QualityGate()
    published = []
    for i in frames:
        t = round(i*period, 9)
        bad = i in bad_frames
        ready = gate.assess(t, t/ratio, imu_age=0., frame_age=.01, clock_age=.004,
                            max_imu_gap=.001, inliers=0 if bad else 120, contrast=30.,
                            covariance=np.eye(9)*.01, state_finite=True,
                            audit_ok=not bad, ekf_ok=not bad)
        if ready:
            published.append(t)
    watchdog = None
    for a, b in zip(published, published[1:]):
        host_a, host_b = a/ratio, b/ratio
        if round((host_b-host_a)*1e9) > WATCHDOG_NS:
            watchdog = host_a+WATCHDOG_NS/1e9   # fires on the first poll after this instant
            break
    return published, watchdog, gate.transitions


class SingleRejectionTimelineTests(unittest.TestCase):
    def test_watchdog_model_matches_transport_source(self):
        source = (ROOT/'phase1_transport.py').read_text()
        # Unconfigured (hold is None) the condition and reason are exactly the original ones.
        self.assertIn('elif (last_odom[0] is not None and time.monotonic_ns()-last_odom[0]>500_000_000\n'
                      '                      and (hold is None or not hold.valid(time.monotonic_ns(),clock[0]))):', source)
        self.assertIn("else 'VIO availability watchdog')", source)
        self.assertIn('block(watchdog_reason)', source)
        # Liveness is stamped at callback entry, before validation.
        self.assertRegex(source, re.compile(r"callback_entry=time\.monotonic_ns\(\)[\s\S]{0,300}last_odom\[0\]=callback_entry"))
        self.assertIn('rclpy.spin_once(node,timeout_sec=.002)', source)

    def test_gate_reopen_rule_needs_four_good_frames_at_10hz(self):
        published, _, transitions = run_timeline({11}, ratio=.6)
        # Warm-up: count 3 at 0.2 s spans only 0.2 s; count 4 at 0.3 s spans 0.3 s >= 0.25.
        self.assertEqual(published[0], .3)
        self.assertIn(1.0, published)
        self.assertNotIn(1.1, published)                      # the bad frame
        for t in (1.2, 1.3, 1.4):                             # good 1/3, 2/3, 3/3 (span 0.2 s)
            self.assertNotIn(t, published)
        self.assertIn(1.5, published)                         # count 4, span 0.3 s >= 0.25
        reasons = [(round(r['t'], 6), r['ready'], r['reason']) for r in transitions]
        self.assertIn((1.1, False, 'inliers'), reasons)
        self.assertIn((1.5, True, 'readiness passed'), reasons)

    def test_one_bad_frame_trips_watchdog_before_reopen_at_ratio_0_6(self):
        published, watchdog, _ = run_timeline({11}, ratio=.6)
        last_before, first_after = 1.0, 1.5
        self.assertEqual(published[published.index(last_before)+1], first_after)
        host_gap = (first_after-last_before)/.6
        self.assertAlmostEqual(host_gap, .8333333, places=6)
        self.assertIsNotNone(watchdog)
        # Fires 0.5 s host after the last publication: sim ~T+0.3 (good frame 2/3),
        # before frame 1.4 (good 3/3) and before the reopen publication at 1.5.
        self.assertAlmostEqual(watchdog-last_before/.6, .5, places=9)
        self.assertLess(watchdog, 1.4/.6)
        self.assertLess(watchdog, first_after/.6)

    def test_measured_ratios_all_trip_and_real_time_has_zero_margin(self):
        # Medians measured from preserved flights (analyze_degraded_drift.py):
        # 0.527 (4414), 0.629 (fbf7), 0.639 (abe2), 0.662 (6f32); p10 down to 0.478.
        for ratio in (.478, .527, .6, .629, .639, .662):
            with self.subTest(ratio=ratio):
                self.assertIsNotNone(run_timeline({11}, ratio=ratio)[1])
        # At real time the 0.5 s publication gap equals the limit exactly: the
        # strict '>' does not fire, but any processing jitter would.
        self.assertIsNone(run_timeline({11}, ratio=1.)[1])

    def test_real_time_publication_gap_depends_on_camera_rate(self):
        # Hardware runs at sim/host = 1.0. The gate needs >= 3 frames AND >= 0.25 s,
        # so the single-rejection output gap is 2/f + ceil-to-grid(max(0.25, 2/f)).
        expected = {10: .5, 15: .4, 20: .35, 30: 1/30+.3}
        for hz, gap in expected.items():
            with self.subTest(hz=hz):
                period = 1/hz
                frames = range(0, 6*hz)
                bad = 3*hz
                published, watchdog, _ = run_timeline({bad}, ratio=1., frames=frames, period=period)
                last = round((bad-1)*period, 9)
                after = published[published.index(last)+1]
                self.assertAlmostEqual(after-last, gap, places=6)
                self.assertIsNone(watchdog)          # strict '>' at exactly 0.5 s for 10 Hz
        # 10 Hz at real time: zero margin; any slower host (ratio < 1) trips.
        self.assertIsNotNone(run_timeline({11}, ratio=.99)[1])
        # 15 Hz at real time still has 0.1 s margin, lost below ratio 0.8.
        self.assertIsNone(run_timeline({45}, ratio=.81, frames=range(90), period=1/15)[1])
        self.assertIsNotNone(run_timeline({45}, ratio=.79, frames=range(90), period=1/15)[1])

    def test_without_rejection_no_watchdog(self):
        self.assertIsNone(run_timeline(set(), ratio=.478)[1])


if __name__ == '__main__':
    unittest.main()
