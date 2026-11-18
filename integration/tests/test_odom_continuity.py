"""Odometry continuity through takeoff/motion, and the 20261117 CLIMB regression.

Synthetic images are rendered textured ground, not Gazebo frames. The recorded
regression reads the preserved transport evidence when it is in the checkout.
"""
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, os.environ.get('VIO_SOURCE_DIR', str(HERE.parent/'vio')))
try:
    import cv2
    import numpy as np
except ImportError:
    np = None

import odom_continuity
from skip_log import SkipLog

RECORDED = HERE/'debug_evidence/20261117-flight6f32c08e7aaf'
FX = 205.4696
PERIOD = 100_000_000  # 10 Hz camera in the Gazebo flight world

# Ground-truth heights (m) of the frames around liftoff and touchdown in run
# 20261117-flight6f32c08e7aaf (flight-source/stream.jsonl gt_ned), where the
# frontend lost tracking. Hover/rest padding before and after.
LIFTOFF = [.195]*10+[.201, .215, .239, .272, .316, .370, .436, .499, .582, .668, .751,
                     .826, .891, .940, .988, 1.016, 1.038, 1.052, 1.060]
TOUCHDOWN = [1.06]*5+[1.032, .996, .953, .909, .860, .810, .758, .714, .661, .607, .554,
                      .502, .451, .400, .350, .309, .261, .213, .195]+[.195]*6


def flight_log(transitions):
    return [dict(event='flight_transition', state=s, host_ns=h) for s, h in transitions]


def odom_rows(stamps_ns, host_of=lambda ns: ns):
    return [dict(acquisition_ns=ns, callback_entry_host_ns=host_of(ns)) for ns in stamps_ns]


class ContinuityBoundTests(unittest.TestCase):
    def test_bound_is_derived_from_sensor_period(self):
        ten_hz = odom_rows(range(0, 2_000_000_000, PERIOD))
        report = odom_continuity.analyze(ten_hz)
        self.assertEqual(report['period_ns'], PERIOD)
        self.assertEqual(report['max_allowed_gap_ns'], 2*PERIOD)
        twenty_hz = odom_rows(range(0, 2_000_000_000, 50_000_000))
        self.assertEqual(odom_continuity.analyze(twenty_hz)['max_allowed_gap_ns'], 100_000_000)

    def test_one_superseded_frame_is_allowed_two_are_not(self):
        stamps = list(range(0, 3_000_000_000, PERIOD))
        one = [s for s in stamps if s != 1_000_000_000]
        self.assertEqual(odom_continuity.analyze(odom_rows(one))['status'], 'PASS')
        two = [s for s in stamps if s not in (1_000_000_000, 1_100_000_000)]
        report = odom_continuity.analyze(odom_rows(two))
        self.assertEqual(report['status'], 'FAIL')
        self.assertEqual(report['gaps'][0]['missing_frames'], 2)

    def test_continuous_through_takeoff_hover_and_land(self):
        # Every phase covered at the full sensor rate, including CLIMB/HOVER/LAND.
        stamps = list(range(20_000_000_000, 60_000_000_000, PERIOD))
        flight = flight_log([('GUIDED', 30e9), ('ARM', 33e9), ('CLIMB', 34e9), ('HOVER', 40e9),
                             ('LAND', 46e9), ('DONE', 55e9)])
        report = odom_continuity.analyze(odom_rows(stamps), flight)
        self.assertEqual(report['status'], 'PASS', report['gaps'])
        for phase in ('CLIMB', 'HOVER', 'LAND'):
            self.assertGreater(report['phases'][phase]['received'], 0)
            self.assertEqual(report['phases'][phase]['gaps'], 0)
        self.assertEqual(report['missing'], 0)

    def test_duplicate_stamps_fail(self):
        stamps = list(range(0, 1_000_000_000, PERIOD))
        report = odom_continuity.analyze(odom_rows(stamps+[stamps[3]]))
        self.assertEqual(report['status'], 'FAIL')
        self.assertEqual(report['duplicates'], 1)

    def test_synthetic_climb_gap_regression(self):
        # Same pattern as run 20261117-flight6f32c08e7aaf: steady 10 Hz until
        # 42.1 s, nothing until 45.4 s, CLIMB began before the last sample.
        stamps = [s for s in range(24_700_000_000, 46_000_000_000, PERIOD)
                  if not 42_100_000_000 < s < 45_400_000_000]
        flight = flight_log([('GUIDED', 38e9), ('ARM', 41e9), ('CLIMB', 41.5e9), ('EMERGENCY_LAND', 42.6e9)])
        report = odom_continuity.analyze(odom_rows(stamps), flight)
        self.assertEqual(report['status'], 'FAIL')
        gap = report['gaps'][0]
        self.assertEqual((gap['from_acquisition_ns'], gap['to_acquisition_ns'], gap['gap_ns']),
                         (42_100_000_000, 45_400_000_000, 3_300_000_000))
        self.assertEqual(gap['phase'], 'CLIMB')
        self.assertEqual(gap['missing_frames'], 32)

    def test_cli_reads_transport_log_format(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            stamps = list(range(0, 2_000_000_000, PERIOD))
            (d/'odom.jsonl').write_text(''.join(json.dumps(dict(r, position_ned=[0, 0, 0]))+'\n' for r in odom_rows(stamps)))
            (d/'flight.jsonl').write_text(json.dumps(dict(event='flight_transition', state='CLIMB', host_ns=5e8))+'\n')
            self.assertEqual(odom_continuity.main([str(d)]), 0)
            (d/'odom.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in odom_rows(stamps[:5]+stamps[9:])))
            self.assertEqual(odom_continuity.main([str(d)]), 1)


@unittest.skipUnless((RECORDED/'odom.jsonl').is_file(), 'Recorded 20261117 evidence not in this checkout')
class RecordedFlightRegression(unittest.TestCase):
    """The failing flight must keep failing this check: proves it detects the bug."""
    def test_recorded_gap_is_detected_in_climb(self):
        report = odom_continuity.analyze_directory(RECORDED)
        self.assertEqual(report['status'], 'FAIL')
        self.assertEqual(report['period_ns'], PERIOD)
        self.assertEqual((report['received'], report['expected'], report['missing']), (195, 302, 107))
        self.assertEqual((report['duplicates'], report['arrival_reordered']), (0, 0))
        first = report['gaps'][0]
        self.assertEqual((first['from_acquisition_ns'], first['to_acquisition_ns']), (42_100_000_000, 45_400_000_000))
        self.assertEqual(first['phase'], 'CLIMB')

    def test_source_did_not_publish_during_gap(self):
        # Loss is at the source, not between publisher and transport.
        stream = RECORDED/'flight-source/stream.jsonl'
        if not stream.is_file():
            self.skipTest('flight-source evidence not in this checkout')
        rows = [json.loads(line) for line in stream.read_text().splitlines()]
        published = {round(r['t']*1e9) for r in rows if r.get('published')}
        received = {r['acquisition_ns'] for r in odom_continuity.read_jsonl(RECORDED/'odom.jsonl')}
        self.assertEqual(published, received)
        self.assertFalse(any(42_100_000_000 < ns < 45_400_000_000 for ns in published))


@unittest.skipIf(np is None, 'NumPy/OpenCV required')
class ScaleCompensationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rng = np.random.default_rng(0)
        noise = cv2.GaussianBlur((rng.random((3000, 3000))*255).astype(np.float32), (0, 0), 2)
        cls.ground = cv2.normalize(noise, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    def render(self, height, x=0., texel=.002):
        s = FX*texel/height
        c = np.array([1500+x/texel, 1500.])
        m = np.array([[s, 0, 320-s*c[0]], [0, s, 240-s*c[1]]])
        return cv2.warpAffine(self.ground, m, (640, 480), flags=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)

    def pair(self, h0, h1, compensate, x1=0.):
        from vio_frontend import FlowFrontend
        from vio_quality_gate import distributed_flow
        from vio_scale_compensation import compensate_previous, predicted_scale
        a, b = self.render(h0), self.render(h1, x1)
        scale = predicted_scale(h0, h1) if compensate else None
        frontend = FlowFrontend(FX)
        frontend.process(a, 0., h0, [0, 0, 0], np.eye(3))
        frontend.prev = compensate_previous(frontend.prev, scale)
        result = frontend.process(b, .1, h1, [0, 0, 0], np.eye(3))
        audit = distributed_flow(a, b, .1, h1, FX, [0, 0, 0], np.eye(3),
                                 result.v_xy if result else np.zeros(2), scale=scale)
        return result, audit

    def test_large_scale_change_tracks_only_with_compensation(self):
        for step in (1.3, 1.4):   # image magnification per frame (descending)
            with self.subTest(step=step):
                raw, raw_audit = self.pair(.3, .3/step, False)
                self.assertTrue(raw is None or raw.n_inliers < 40 or not raw_audit['ok'])
                comp, audit = self.pair(.3, .3/step, True)
                self.assertIsNotNone(comp, 'frontend lost tracking despite compensation')
                self.assertGreaterEqual(comp.n_inliers, 150)
                self.assertTrue(audit['ok'], audit)
                self.assertLess(np.hypot(*comp.v_xy), .01)  # pure vertical motion

    def test_compensation_preserves_horizontal_velocity(self):
        dx = .02   # 0.2 m/s while climbing 15 % per frame
        result, audit = self.pair(.4, .4*1.15, True, x1=dx)
        self.assertTrue(audit['ok'], audit)
        self.assertAlmostEqual(float(np.hypot(*result.v_xy)), dx/.1, delta=.005)

    def test_identity_and_unusable_predictions(self):
        from vio_scale_compensation import compensate_previous, predicted_scale
        image = self.render(.5)
        self.assertIs(compensate_previous(image, None), image)
        self.assertIs(compensate_previous(image, 1.), image)
        for previous, now in ((None, 1.), (1., 0.), (1., float('nan')), (1., 3.), (-1., 1.)):
            self.assertIsNone(predicted_scale(previous, now))

    def test_parallel_audit_measurement_matches_inline(self):
        from vio_quality_gate import measure_distributed_flow, judge_distributed_flow, distributed_flow
        a, b = self.render(.5), self.render(.5/1.1, .01)
        inline = distributed_flow(a, b, .1, .5/1.1, FX, [0, 0, 0], np.eye(3), [0., .1], scale=1.1)
        with ThreadPoolExecutor(1) as pool:
            measured = pool.submit(measure_distributed_flow, a, b, .1, 1.1).result()
        threaded = judge_distributed_flow(measured, .5/1.1, FX, [0, 0, 0], np.eye(3), [0., .1])
        self.assertEqual(inline, threaded)

    def test_source_pipeline_stays_continuous_through_takeoff_and_touchdown(self):
        """Frontend + audit + EKF + readiness gate over the recorded height profile.

        Every frame passes through the same gate the source uses; the published
        stamps must satisfy the derived continuity bound once ready.
        """
        from vio_frontend import FlowFrontend
        from vio_core import VioEKF
        from vio_quality_gate import QualityGate, distributed_flow, ekf_ready
        from vio_scale_compensation import compensate_previous, predicted_scale
        for name, heights in (('liftoff', LIFTOFF), ('touchdown', TOUCHDOWN)):
            with self.subTest(profile=name):
                frontend, ekf, gate = FlowFrontend(FX), VioEKF(), QualityGate()
                previous, previous_h, published, reasons = None, None, [], []
                for i, h in enumerate(heights):
                    t = 40.+i*.1
                    gray = self.render(h)
                    scale = predicted_scale(previous_h, h) if previous is not None else None
                    frontend.prev = compensate_previous(frontend.prev, scale)
                    for _ in range(10):
                        ekf.predict(np.array([0, 0, -9.80665]), np.eye(3), .01)
                    ekf.update_height(-h)
                    result = frontend.process(gray, t, h, [0, 0, 0], np.eye(3))
                    audit = {'ok': False}
                    accepted = False
                    if result is not None:
                        audit = distributed_flow(previous, gray, .1, h, FX, [0, 0, 0], np.eye(3), result.v_xy, scale=scale)
                        if audit['ok']:
                            accepted, _ = ekf.update_velocity_xy(result.v_xy, result.sigma_scale)
                    ready = gate.assess(t, t, imu_age=0., frame_age=.02, clock_age=.02, max_imu_gap=.001,
                                        inliers=result.n_inliers if result else 0, contrast=float(gray.std()),
                                        covariance=ekf.P, state_finite=True, audit_ok=audit['ok'],
                                        ekf_ok=bool(accepted and ekf_ready(ekf)))
                    if ready:
                        published.append(round(t*1e9))
                    else:
                        reasons.append((t, gate.reason))
                    previous, previous_h = gray, h
                # Readiness needs 3 frames >= 250 ms after the first pair: 4 frames.
                self.assertEqual(published[0], round((40.+4*.1)*1e9), reasons)
                report = odom_continuity.analyze(odom_rows(published), period_ns=PERIOD)
                self.assertEqual(report['status'], 'PASS', (report['gaps'], reasons))
                self.assertEqual(report['missing'], 0, reasons)


@unittest.skipIf(np is None, 'NumPy required')
class GateReasonTests(unittest.TestCase):
    def test_withheld_frames_always_name_a_reason(self):
        from vio_quality_gate import QualityGate
        g = QualityGate()
        kw = dict(imu_age=.001, frame_age=.01, clock_age=.01, max_imu_gap=.001, inliers=50, contrast=40,
                  covariance=np.eye(9)*.01, state_finite=True, audit_ok=True, ekf_ok=True)
        self.assertFalse(g.assess(1., 1., **kw))
        self.assertIn('hysteresis 1/3', g.reason)
        g.assess(1.1, 1.1, **kw)
        self.assertFalse(g.assess(1.2, 1.2, **kw))
        self.assertIn('hysteresis 3/3', g.reason)
        self.assertTrue(g.assess(1.3, 1.3, **kw))
        self.assertIsNone(g.reason)
        self.assertFalse(g.assess(1.4, 1.4, **dict(kw, inliers=3)))
        self.assertEqual(g.reason, 'inliers')
        g.block('stale at publication', 1.5, 1.5)
        self.assertEqual(g.reason, 'stale at publication')


class SkipLogTests(unittest.TestCase):
    def test_rate_limited_per_reason_with_suppressed_counts(self):
        rows, now = [], [0.]
        log = SkipLog(rows.append, interval_s=1., clock=lambda: now[0])
        for i in range(30):            # 3 s outage at 10 Hz, one reason
            now[0] = i*.1
            log('stale at publication', 42_200_000_000+i*PERIOD, 'vio_source', t=42.2+i*.1)
        log('inliers', 1, 'vio_source')
        self.assertEqual([r['reason'] for r in rows].count('stale at publication'), 3)
        self.assertEqual(rows[0]['acquisition_ns'], 42_200_000_000)
        self.assertEqual(rows[0]['suppressed_since_last'], 0)
        self.assertEqual(rows[1]['suppressed_since_last'], 9)
        self.assertEqual(rows[-1]['reason'], 'inliers')
        log.flush('vio_source')
        self.assertEqual(rows[-1]['suppressed_since_last'], 9)
        self.assertTrue(rows[-1]['final_flush'])
        self.assertEqual(log.total, 31)
        self.assertTrue(all(r['event'] == 'odometry_skipped' for r in rows))

    def test_steady_state_emits_nothing(self):
        rows = []
        SkipLog(rows.append)
        self.assertEqual(rows, [])


if __name__ == '__main__':
    unittest.main()
