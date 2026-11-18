"""VIO during motion: run 20261117-flightfea1c93152a2 (watchdog 5 s into CLIMB).

Covers the three changes made for it and the evidence chain that motivated them:
  * IMU propagated in 5 ms blocks (1 kHz flight IMU -> the 200 Hz design rate)
  * a skipped frame still becomes the tracking reference (no doubled baseline)
  * fresh, gap-bounded odometry during motion in replay of recorded flight costs
"""
import json
import math
import os
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, os.environ.get('VIO_SOURCE_DIR', str(HERE.parent/'vio')))
try:
    import numpy as np
except ImportError:
    np = None

import odom_continuity
from vio_latency_contract import SOURCE_PUBLICATION_MAX_AGE_S

RUN = HERE/'phase1_runs/20261117-flightfea1c93152a2/evidence'
EVIDENCE = HERE/'debug_evidence'
PERIOD = .1
MAX_GAP = odom_continuity.GAP_FRAMES*PERIOD
WATCHDOG_S = .5   # phase1_transport VIO availability watchdog (unchanged)


def aggressive_imu(rate_hz=1000, seconds=3.):
    out = []
    for k in range(1, int(round(seconds*rate_hz))+1):
        t = round(k/rate_hz, 6)
        yaw, roll, pitch = 1.*t, .15*math.sin(2*t), .1*math.sin(3*t)
        cy, sy, cr, sr, cp, sp = (math.cos(yaw/2), math.sin(yaw/2), math.cos(roll/2), math.sin(roll/2),
                                  math.cos(pitch/2), math.sin(pitch/2))
        q = [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]
        out.append(dict(t=t, f=[1.5*math.sin(4*t), math.cos(3*t), -9.80665+.8*math.sin(5*t)],
                        gyro=[.3*math.cos(2*t), .3*math.cos(3*t), 1.], q=q))
    return out


def propagate(imu, step_s, predict_counter=None):
    from vio_core import VioEKF
    from vio_imu_preintegration import advance, new_pending
    from test_vio_backlog import rotation
    ekf = VioEKF()
    predict = ekf.predict
    if predict_counter is not None:
        def predict(*args):
            predict_counter.append(1)
            return ekf.predict(*args)
    state = dict(last_imu=imu[0], filter_t=imu[0]['t'], imu_index=1, **new_pending())
    count, bad = advance(state, imu, 0, 1e9, predict, rotation, step_s=step_s)
    gyro = sum(dt*np.asarray(g) for dt, g in state['pending_gyro'])/sum(dt for dt, _ in state['pending_gyro'])
    return ekf, state, count, bad, gyro


@unittest.skipIf(np is None, 'NumPy required')
class BlockPropagation(unittest.TestCase):
    def test_matches_per_sample_propagation_under_aggressive_motion(self):
        imu = aggressive_imu()
        fine, _, _, _, g_fine = propagate(imu, 0.)
        block, _, _, _, g_block = propagate(imu, .005)
        self.assertLess(np.linalg.norm(fine.x[3:6]-block.x[3:6]), 1e-4)    # m/s after 3 s at ~2 m/s
        self.assertLess(np.linalg.norm(fine.x[:3]-block.x[:3]), 1e-4)      # m
        self.assertLess(np.abs(fine.P-block.P).max()/np.abs(fine.P).max(), 1e-2)
        self.assertTrue(np.allclose(g_fine, g_block, atol=1e-12))          # per-frame gyro mean unchanged

    def test_five_times_fewer_predictions_at_1khz(self):
        calls_fine, calls_block = [], []
        imu = aggressive_imu(seconds=1.)
        propagate(imu, 0., calls_fine)
        propagate(imu, .005, calls_block)
        self.assertEqual(len(calls_fine), 999)
        self.assertLessEqual(len(calls_block), len(calls_fine)//5+1)

    def test_200hz_input_is_propagated_exactly_as_before(self):
        imu = aggressive_imu(rate_hz=200)
        fine, *_ = propagate(imu, 0.)
        block, *_ = propagate(imu, .005)
        self.assertTrue(np.array_equal(fine.x, block.x))
        self.assertTrue(np.array_equal(fine.P, block.P))

    def test_gap_inside_a_block_is_still_rejected(self):
        imu = [m for m in aggressive_imu(seconds=.2) if not .0525 < m['t'] < .0755]   # 23 ms raw gap
        _, state, count, bad, _ = propagate(imu, .005)
        self.assertIsNotNone(bad)
        self.assertAlmostEqual(state['last_imu']['t'], .052)
        self.assertLessEqual(state['pending_max_gap'], .02)

    def test_nonfinite_sample_inside_a_block_is_rejected(self):
        imu = aggressive_imu(seconds=.05)
        imu[22] = dict(imu[22], f=[float('nan'), 0., -9.8])
        _, state, _, bad, _ = propagate(imu, .005)
        self.assertIs(bad, imu[22])
        self.assertEqual(state['imu_index'], 22)

    def test_frame_path_split_is_preserved(self):
        from vio_core import VioEKF
        from vio_imu_preintegration import frame_step, idle_step, new_pending
        from test_vio_backlog import rotation
        imu = aggressive_imu(seconds=.5)
        ekf = VioEKF()
        state = dict(last_imu=imu[99], filter_t=imu[99]['t'], imu_index=100, frame_stamp=.1, camera_period=.1,
                     reset_after=None, **new_pending())
        for _ in range(30):
            idle_step(state, lambda i, n: imu[i:i+n], ekf.predict, rotation, step_s=.005)
        self.assertLess(state['filter_t'], .2)
        frame_step(state, imu, .2, ekf.predict, rotation, step_s=.005)
        self.assertEqual(state['filter_t'], .2)


class SkippedFrameStaysReference(unittest.TestCase):
    class Frontend:
        prev, prev_t = None, None

    def test_next_pair_is_one_period_apart(self):
        from vio_admission import keep_as_reference
        frontend = self.Frontend()
        frontend.prev, frontend.prev_t = 'A', 41.2
        state = dict(previous='A', previous_t=41.2, previous_depth=.40, pending_gyro=[(.05, 0)], pending_max_gap=.004)
        self.assertTrue(keep_as_reference(state, frontend, 'B', 41.3, .47))
        self.assertEqual((frontend.prev, frontend.prev_t), ('B', 41.3))
        self.assertEqual((state['previous'], state['previous_t'], state['previous_depth']), ('B', 41.3, .47))
        self.assertEqual(state['pending_gyro'], [])          # derotation window restarts with the flow
        self.assertEqual(state['pending_max_gap'], .004)     # IMU gap evidence is never discarded

    def test_no_reference_before_tracking_started(self):
        from vio_admission import keep_as_reference
        frontend = self.Frontend()
        state = dict(previous=None, previous_t=None, previous_depth=None, pending_gyro=[])
        self.assertFalse(keep_as_reference(state, frontend, 'B', 1., .5))
        self.assertIsNone(frontend.prev)


@unittest.skipIf(np is None, 'NumPy required')
class ReadinessAndWatchdog(unittest.TestCase):
    """Why one blocked frame trips the watchdog, and why a skip does not."""
    kw = dict(imu_age=.001, frame_age=.01, clock_age=.01, max_imu_gap=.001, inliers=50, contrast=40,
              state_finite=True, audit_ok=True, ekf_ok=True)

    def outage(self, event):
        from vio_quality_gate import QualityGate
        g, published = QualityGate(), []
        for k in range(30):
            t = round(10+k*PERIOD, 6)
            if k == 15 and event == 'skip':
                continue                                    # admission skip: the gate is not consulted
            bad = k == 15 and event == 'block'
            if g.assess(t, t, covariance=np.eye(9)*.01, **dict(self.kw, inliers=3 if bad else 50)):
                published.append(t)
        return max(b-a for a, b in zip(published, published[1:]))

    def test_one_blocked_frame_costs_at_least_the_watchdog(self):
        # At 10 Hz the unchanged 3-frame/250 ms readiness needs 4 frames after a
        # block: 0.5 s of simulation time without output, >= 500 ms of wall time
        # at real-time factor <= 1. In flight every gate block must be avoided.
        self.assertGreaterEqual(self.outage('block')+1e-9, WATCHDOG_S)

    def test_one_skipped_frame_stays_within_the_continuity_bound(self):
        self.assertLessEqual(self.outage('skip'), MAX_GAP+1e-9)


@unittest.skipUnless((RUN/'flight-source/skips.jsonl').is_file(), 'fea1c93152a2 evidence not in checkout')
class RecordedClimbFailure(unittest.TestCase):
    def test_evidence_chain(self):
        skips = [json.loads(line) for line in (RUN/'flight-source/skips.jsonl').read_text().splitlines()]
        stages = [json.loads(line) for line in (RUN/'flight-transport/stages.jsonl').read_text().splitlines()]
        watchdog = next(r for r in stages if r['stage'] == 'vio_watchdog_diagnostic')
        before = [r for r in skips if r['host_monotonic_s'] <= watchdog['host_ns']/1e9 and r.get('t', 0) > 40]
        reasons = [(round(r['t'], 1), r['reason']) for r in before]
        # 1. admission skip, 2. the next frame was paired across the skipped one
        self.assertEqual(reasons[0], (41.3, 'skipped: predicted to exceed publication age (consumer behind)'))
        failed = before[1]
        self.assertEqual((round(failed['t'], 1), failed['reason']), (41.4, 'EKF not ready'))
        self.assertLess(failed['scale_compensation'], .7)         # >40 % height change across 0.2 s
        self.assertLess(failed['inliers'], 50)
        self.assertNotEqual(failed['audit_reason'], 'agreement')
        self.assertEqual(failed['epoch'], 0)                      # no estimator reset: not an EKF fault
        # 3. readiness re-armed but needed more frames than the watchdog allows
        self.assertTrue(all('hysteresis' in reason for _, reason in reasons[2:]))
        self.assertGreaterEqual(watchdog['odom_age_ns'], WATCHDOG_S*1e9)
        self.assertFalse(watchdog['gate_blocked'])

    def test_processing_exceeded_the_frame_period_once_airborne(self):
        skips = [json.loads(line) for line in (RUN/'flight-source/skips.jsonl').read_text().splitlines()]
        airborne = [r['processing_s'] for r in skips if r.get('processing_s') and 41.3 <= r.get('t', 0) <= 49.5]
        ground = [r['processing_s'] for r in skips if r.get('processing_s') and r.get('t', 0) < 25 and r.get('inliers')]
        self.assertGreater(min(airborne), .08)
        self.assertLess(max(ground), .06)


@unittest.skipUnless((EVIDENCE/'20261117-flight4414cb818f2a/flight-source/stream.jsonl').is_file(), 'recorded costs not in checkout')
class FreshOdometryDuringMotion(unittest.TestCase):
    """Replay of recorded per-frame costs through the capture loop (not a flight)."""
    def replay(self, run, policy, window=None):
        import analyze_vio_latency as a
        rows, meta = a.replay(a.load(EVIDENCE/run), policy)
        rows = [r for r in rows if window is None or window[0] <= r['t'] <= window[1]]
        fresh = sorted(r['t'] for r in rows if r['publish_age'] <= SOURCE_PUBLICATION_MAX_AGE_S)
        return dict(over=sum(r['publish_age'] > SOURCE_PUBLICATION_MAX_AGE_S for r in rows),
                    max_gap=max(b-a for a, b in zip(fresh, fresh[1:])), skipped=meta['skipped'], n=len(rows))

    def test_climb_and_flight_stay_fresh_and_continuous(self):
        before = self.replay('20261117-flight4414cb818f2a', 'frame')
        after = self.replay('20261117-flight4414cb818f2a', 'idle+admit+5ms')
        self.assertGreater(before['over'], 10)
        self.assertEqual(after['over'], 0, after)
        self.assertLessEqual(after['max_gap'], MAX_GAP+1e-6, after)
        self.assertLessEqual(after['skipped'], 5)

    def test_real_time_airborne_segment(self):
        # Earlier run, airborne at real-time factor 0.99 (recorded costs predate the
        # parallel audit, so this is pessimistic). One recorded processing spike remains.
        if not (EVIDENCE/'20261117-flight6f32c08e7aaf/flight-source/stream.jsonl').is_file():
            self.skipTest('6f32c08e7aaf evidence not in checkout')
        before = self.replay('20261117-flight6f32c08e7aaf', 'frame', (43.5, 51))
        after = self.replay('20261117-flight6f32c08e7aaf', 'idle+admit+5ms', (43.5, 51))
        self.assertGreater(before['over'], 50)
        self.assertLessEqual(after['over'], 1, after)


if __name__ == '__main__':
    unittest.main()
