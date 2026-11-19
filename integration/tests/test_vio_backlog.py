"""VIO source backlog: IMU off the frame path, superseded-frame accounting,
freshness under load, and the 20261117-flight4414cb818f2a 37.9 s regression.

Equivalence tests run the real integration code against the frame-path IMU
loop exactly as it was before (copied from 858273e). Scheduling tests replay
costs through analyze_vio_latency's model of the capture loop; that model is
itself checked against the recorded run.
"""
import math
import os
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, os.environ.get('VIO_SOURCE_DIR', str(HERE.parent/'vio')))
try:
    import numpy as np
except ImportError:
    np = None

import odom_continuity
from skip_log import SkipLog
from vio_frame_slot import SlotAccounting

RECORDED = HERE/'debug_evidence/20261117-flight4414cb818f2a'
PERIOD = .1
MAX_GAP = odom_continuity.GAP_FRAMES*PERIOD   # derived bound: 2 x camera period


def rotation(q):
    q = np.asarray(q, float)
    if not np.isfinite(q).all() or abs(q@q-1) > 1e-4:
        raise ValueError('invalid quaternion')
    from vio_ros_out import quat_to_rot
    return quat_to_rot(q)


def reference_frame_imu(state, imu, t, ekf):
    """Frame-path IMU loop exactly as in 858273e run_vio_stream.py (block() omitted)."""
    gyro_samples = []
    max_gap = .001
    while state['imu_index'] < len(imu) and imu[state['imu_index']]['t'] <= t:
        m = imu[state['imu_index']]
        last = state['last_imu']
        gap = m['t']-last['t']
        max_gap = max(max_gap, gap)
        if not 0 < gap <= .02 or not np.isfinite(m['f']+m['gyro']).all():
            raise ValueError('IMU gap/reset/nonfinite')
        dt = m['t']-state['filter_t']
        if dt > 0:
            ekf.predict(np.asarray(last['f']), rotation(last['q']), dt)
            gyro_samples.append((dt, np.asarray(last['gyro'])))
        state.update(last_imu=m, filter_t=m['t'], imu_index=state['imu_index']+1)
    last = state['last_imu']
    imu_age = t-last['t']
    if not 0 <= imu_age <= .005:
        raise ValueError('IMU watermark/alignment')
    r = rotation(last['q'])
    dt = t-state['filter_t']
    if dt > 0:
        ekf.predict(np.asarray(last['f']), r, dt)
        gyro_samples.append((dt, np.asarray(last['gyro'])))
    state['filter_t'] = t
    return gyro_samples, max_gap


def imu_stream(duration=3., drop=()):
    out = []
    for k in range(1, int(round(duration*1000))+1):
        t = round(k*.001, 6)
        if any(lo <= t < hi for lo, hi in drop):
            continue
        yaw = .3*math.sin(t)
        out.append(dict(t=t, f=[.1*math.sin(3*t), .2*math.cos(2*t), -9.80665+.05*math.sin(5*t)],
                        gyro=[.01*math.cos(t), -.02*math.sin(t), .3*math.cos(t)],
                        q=[0., 0., math.sin(yaw/2), math.cos(yaw/2)]))
    return out


def init_state(imu, t):
    from vio_imu_preintegration import new_pending
    i = next(i for i in range(len(imu)-1, -1, -1) if imu[i]['t'] <= t)
    return dict(last_imu=imu[i], filter_t=imu[i]['t'], imu_index=i+1, frame_stamp=t, camera_period=None,
                reset_after=None, superseded_since_frame=0, **new_pending())


@unittest.skipIf(np is None, 'NumPy required')
class PreintegrationEquivalence(unittest.TestCase):
    def run_both(self, imu, frames, errors_before_imu=()):
        """frames: stamps the consumer takes; errors_before_imu: taken but fail before IMU."""
        from vio_core import VioEKF
        from vio_imu_preintegration import frame_step, idle_step, observe_frame_stamp, TICK_CHUNK
        first = frames[0]
        ref_ekf, new_ekf = VioEKF(), VioEKF()
        ref, new = init_state(imu, first), init_state(imu, first)
        results = []
        now, k = first, 1
        while k < len(frames):
            now = round(now+.002, 6)
            available = [m for m in imu if m['t'] <= now]
            tail = lambda index, limit: available[index:index+limit]
            if frames[k] <= now:
                t = frames[k]
                observe_frame_stamp(new, t)
                if t not in errors_before_imu:
                    expected = reference_frame_imu(ref, imu, t, ref_ekf)
                    got = frame_step(new, imu, t, new_ekf.predict, rotation)
                    results.append((t, expected, got))
                    self.assertTrue(np.array_equal(ref_ekf.x, new_ekf.x), t)
                    self.assertTrue(np.array_equal(ref_ekf.P, new_ekf.P), t)
                    self.assertEqual(expected[1], got[1], t)
                    self.assertEqual(len(expected[0]), len(got[0]), t)
                    for (a_dt, a_g), (b_dt, b_g) in zip(expected[0], got[0]):
                        self.assertEqual(a_dt, b_dt)
                        self.assertTrue(np.array_equal(a_g, b_g))
                k += 1
            n = idle_step(new, tail, new_ekf.predict, rotation)
            self.assertLessEqual(n, TICK_CHUNK)
            if new['camera_period'] is not None:
                self.assertLess(new['filter_t'], new['frame_stamp']+new['camera_period'])
        return results

    def test_bit_identical_to_frame_path_integration(self):
        imu = imu_stream()
        frames = [round(.1*k, 6) for k in range(1, 30)]
        results = self.run_both(imu, frames)
        self.assertEqual(len(results), 28)

    def test_superseded_frames_and_frames_failing_before_imu(self):
        imu = imu_stream()
        frames = [round(.1*k, 6) for k in range(1, 30) if k not in (5, 6, 17)]  # superseded
        results = self.run_both(imu, frames, errors_before_imu=(1.2, 1.3))
        self.assertTrue(any(abs(len(expected[0])-200) <= 1 for _, expected, _ in results))  # 2-frame span

    def test_most_work_leaves_the_frame_path(self):
        from vio_imu_preintegration import frame_step, idle_step, observe_frame_stamp
        from vio_core import VioEKF
        imu = imu_stream(1.)
        state, ekf = init_state(imu, .1), VioEKF()
        state['camera_period'] = PERIOD
        frame_step(state, imu, .1, ekf.predict, rotation)
        for _ in range(20):
            idle_step(state, lambda i, n: [m for m in imu if m['t'] <= .2][i:i+n], ekf.predict, rotation)
        observe_frame_stamp(state, .2)
        before = state['imu_index']
        frame_step(state, imu, .2, ekf.predict, rotation)
        self.assertLessEqual(state['imu_index']-before, 1)   # only the sample at exactly t
        self.assertEqual(state['pending_count'], 0)

    def test_bad_sample_is_left_for_the_frame_path(self):
        from vio_imu_preintegration import frame_step, idle_step, ImuGapError
        from vio_core import VioEKF
        imu = imu_stream(1., drop=[(.15, .18)])   # 30 ms IMU gap
        state, ekf = init_state(imu, .1), VioEKF()
        state['camera_period'] = PERIOD
        frame_step(state, imu, .1, ekf.predict, rotation)
        for _ in range(50):
            idle_step(state, lambda i, n: imu[i:i+n], ekf.predict, rotation)
        self.assertAlmostEqual(state['last_imu']['t'], .149)
        with self.assertRaises(ImuGapError):
            frame_step(state, imu, .2, ekf.predict, rotation)
        ref_state, ref_ekf = init_state(imu, .1), VioEKF()
        reference_frame_imu(ref_state, imu, .1, ref_ekf)
        with self.assertRaisesRegex(ValueError, 'IMU gap'):
            reference_frame_imu(ref_state, imu, .2, ref_ekf)

    def test_no_preintegration_before_init_or_during_reset(self):
        from vio_imu_preintegration import idle_step
        imu = imu_stream(.5)
        state = init_state(imu, .1)
        state['camera_period'] = PERIOD
        state['reset_after'] = .35
        self.assertEqual(idle_step(state, lambda i, n: imu[i:i+n], None, rotation), 0)
        state.update(reset_after=None, last_imu=None)
        self.assertEqual(idle_step(state, lambda i, n: imu[i:i+n], None, rotation), 0)

    def test_overrun_fails_closed(self):
        from vio_imu_preintegration import frame_step, PreintegrationOverrun
        imu = imu_stream(.5)
        state = init_state(imu, .1)
        state['filter_t'] = .25
        with self.assertRaises(PreintegrationOverrun):
            frame_step(state, imu, .2, None, rotation)

    def test_frame_stamp_bookkeeping(self):
        from vio_imu_preintegration import observe_frame_stamp
        state = dict(frame_stamp=None, camera_period=None, superseded_since_frame=0)
        self.assertEqual(observe_frame_stamp(state, 1.0), 0)
        self.assertEqual(observe_frame_stamp(state, 1.1), 0)
        self.assertAlmostEqual(state['camera_period'], .1)
        state['superseded_since_frame'] = 1
        self.assertEqual(observe_frame_stamp(state, 1.4), 1)   # 2 missing, 1 superseded -> 1 never received
        self.assertEqual(state['superseded_since_frame'], 0)


class SlotAccountingTests(unittest.TestCase):
    def test_reports_every_superseded_stamp(self):
        slot = SlotAccounting()
        for ns in (100, 200, 300, 400):
            slot.arrived(ns)
        self.assertEqual(slot.consumed(100), [])
        self.assertEqual(slot.consumed(400), [200, 300])
        slot.arrived(500)
        self.assertEqual(slot.consumed(500), [])
        self.assertEqual(slot.superseded, 2)

    def test_superseded_frames_are_logged_with_acquisition_ns(self):
        rows, now = [], [0.]
        log = SkipLog(rows.append, clock=lambda: now[0])
        slot = SlotAccounting()
        for i in range(30):
            slot.arrived(i*100_000_000)
        for ns in slot.consumed(29*100_000_000):
            log('superseded in latest-only camera slot (consumer busy)', ns, 'vio_source')
        self.assertEqual(rows[0]['acquisition_ns'], 0)
        self.assertEqual(log.total, 29)
        log.flush('vio_source')
        self.assertEqual(rows[-1]['suppressed_since_last'], 28)
        self.assertEqual(rows[-1]['suppressed_last_acquisition_ns'], 28*100_000_000)


class TailFromTests(unittest.TestCase):
    def test_matches_snapshot_indices(self):
        try:
            from vio_sensor_capture import BoundedRecords
        except ImportError as exc:
            self.skipTest(str(exc))
        b = BoundedRecords(1000)
        for i in range(50):
            b.append(dict(t=i))
        snap = b.snapshot()
        self.assertEqual(b.tail_from(40), snap[40:])
        self.assertEqual(b.tail_from(40, 3), snap[40:43])
        self.assertEqual(b.tail_from(60), [])


def synthetic_run(rtf, base_ms, imu_ms, seconds=6., start=20., delivery=.015, climb=None):
    """Recorded-format inputs for analyze_vio_latency.replay with chosen load.

    rtf/base_ms/imu_ms are functions of simulation time; imu_ms is the frame-path
    IMU cost for one 100-sample interval.
    """
    from analyze_vio_latency import Timebase
    pairs, host, sim = [], 1000., start-1.
    while sim < start+seconds+1.:
        pairs.append((host, sim))
        host += .001
        sim += .001*rtf(sim)
    clock = Timebase(pairs)
    stream = []
    for k in range(int(round(seconds/PERIOD))):
        t = round(start+k*PERIOD, 6)
        imu = imu_ms(t)/1e3
        stream.append(dict(t=t, wall=clock.to_host(t+delivery), processing_s=base_ms(t)/1e3+imu,
                           imu_processing_wall_s=imu, imu_batch=dict(count=100), source_age=delivery))
    return dict(stream=stream, clock=clock, pair_age={}, pair_cost=[.0078], uncertainty_s=.017,
                climb_window=climb or (start, start+seconds))


@unittest.skipIf(np is None, 'NumPy required')
class FreshnessUnderLoad(unittest.TestCase):
    def check(self, data, policy):
        import analyze_vio_latency as a
        rows, _ = a.replay(data, policy)
        limit = a.conversion_limit(data)
        steady = [r for r in rows if r['t'] >= data['stream'][0]['t']+.5]
        usable = sorted(r['t'] for r in steady if r['publish_age'] <= limit)
        gaps = [b-a for a, b in zip(usable, usable[1:])]
        return dict(max_age=max(r['publish_age'] for r in steady), over=sum(r['publish_age'] > limit for r in steady),
                    max_gap=max(gaps) if gaps else float('inf'), limit=limit)

    def test_frames_faster_than_frame_path_processing(self):
        # 10 Hz frames, 120 ms frame-path processing at real time (80 ms frame
        # work + 40 ms IMU): frames arrive faster than they are processed. With
        # IMU on the frame path outputs go stale; with idle-time IMU and
        # most-recent-frame admission they stay fresh at a reduced rate, and
        # gaps stay within the derived 2 x camera period.
        data = synthetic_run(lambda s: 1., lambda t: 80., lambda t: 40.)
        before, after = self.check(data, 'frame'), self.check(data, 'idle+admit')
        self.assertGreater(before['over'], 0, before)
        self.assertEqual(after['over'], 0, after)
        self.assertLess(after['max_age'], after['limit'])
        self.assertLessEqual(after['max_gap'], MAX_GAP+1e-6, after)

    def test_climb_like_rising_load(self):
        # Recorded pattern: takeoff lifts RTF 0.5 -> ~1, non-IMU work 60 -> 95 ms,
        # IMU 25 -> 45 ms per frame interval.
        ramp = lambda lo, hi: (lambda t: lo+(hi-lo)*min(1., max(0., (t-21.)/2.)))
        data = synthetic_run(ramp(.5, 1.), ramp(60., 95.), ramp(25., 45.), seconds=5.)
        before, after = self.check(data, 'frame'), self.check(data, 'idle+admit')
        self.assertGreater(before['over'], 0, before)
        self.assertEqual(after['over'], 0, after)
        self.assertLessEqual(after['max_gap'], MAX_GAP+1e-6, after)

    def test_idle_imu_alone_fixes_latency_when_there_is_headroom(self):
        # RTF 0.73 (the recorded CLIMB): 90 ms frame work + 40 ms IMU fits in the
        # 137 ms frame interval only if the IMU leaves the frame path.
        data = synthetic_run(lambda s: .73, lambda t: 90., lambda t: 40.)
        idle = self.check(data, 'idle')
        self.assertEqual(idle['over'], 0, idle)
        self.assertLessEqual(idle['max_gap'], PERIOD+1e-6, idle)

    def test_sustained_overload_is_reported_not_hidden(self):
        # Frame work alone exceeds the frame period (110 ms per 100 ms at RTF 1):
        # no scheduling keeps outputs fresh; the analysis must say so.
        data = synthetic_run(lambda s: 1., lambda t: 110., lambda t: 40.)
        self.assertGreater(self.check(data, 'idle+admit')['over'], 0)


class AdmissionTests(unittest.TestCase):
    def test_admits_until_it_has_history(self):
        from vio_admission import Admission
        self.assertTrue(Admission(.18).admit(10.))

    def test_skips_predicted_stale_but_never_twice_in_a_row(self):
        from vio_admission import Admission
        a = Admission(.18)
        a.processed(.09)
        self.assertTrue(a.admit(.09))        # 0.09 + 0.09 = limit: admitted
        self.assertFalse(a.admit(.10))       # predicted 0.19 > 0.18: skipped
        self.assertTrue(a.admit(.15))        # never two skips in a row
        self.assertFalse(a.admit(.15))
        self.assertTrue(a.admit(.01))

    def test_uses_pessimistic_recent_processing_time(self):
        from vio_admission import Admission
        a = Admission(.18, window=3)
        for d in (.05, .12, .06):
            a.processed(d)
        self.assertEqual(a.expected(), .12)
        a.processed(.05)
        a.processed(.05)
        self.assertEqual(a.expected(), .06)

    def test_land_stall_does_not_starve_following_fresh_frames(self):
        from vio_admission import Admission

        admission = Admission(.157)
        for duration in (.054, .062, .058, .048, .057, .163):
            admission.processed(duration)

        # Frame 51.9 was rejected by the absolute freshness gate. The cost
        # model must allow frames 52.0 onward to be evaluated normally.
        self.assertTrue(admission.admit(.080))
        for age, cost in ((.013, .045), (.006, .054), (.010, .055), (.008, .051)):
            admission.processed(cost)
            self.assertTrue(admission.admit(age))

    def test_repeated_stalls_keep_conservative_prediction(self):
        from vio_admission import Admission

        admission = Admission(.157)
        for duration in (.040, .055, .048, .162, .163):
            admission.processed(duration)
        self.assertEqual(admission.expected(), .162)
        self.assertFalse(admission.admit(.008))

    def test_limit_must_be_the_positive_publication_age(self):
        from vio_admission import Admission
        with self.assertRaises(ValueError):
            Admission(0)


@unittest.skipUnless((RECORDED/'flight-source/stream.jsonl').is_file(), 'Recorded 4414cb818f2a evidence not in this checkout')
class RecordedRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import analyze_vio_latency as a
        cls.report = a.report(RECORDED)

    def test_replay_reproduces_the_recording(self):
        v = self.report['replay_validation']
        self.assertGreater(v['frames'], 150)
        self.assertLess(v['median_abs_error_ms'], 5)
        self.assertLess(v['p95_abs_error_ms'], 15)

    def test_acquisition_37_9_expired_as_recorded_and_is_fresh_after_fix(self):
        r, limit = self.report['acquisition_37_9'], self.report['limit_conversion_age_ms']
        self.assertGreater(r['measured']['conversion_age'], limit)          # the recorded failure
        self.assertLess(abs(r['replay_frame']['conversion_age']-r['measured']['conversion_age']), 10)
        for policy in ('replay_idle', 'replay_idle+admit'):
            self.assertLess(r[policy]['conversion_age'], limit-self.report['slowest_pair_ms'], policy)

    def test_climb_window_stays_fresh_and_continuous(self):
        self.assertGreater(self.report['table']['measured']['CLIMB']['over_limit'], 0)
        for policy in ('replay_idle', 'replay_idle+admit'):
            climb = self.report['table'][policy]['CLIMB']
            self.assertEqual(climb['over_limit'], 0, policy)
            self.assertLessEqual(climb['max_gap_ms'], MAX_GAP*1e3+1e-6, policy)

    def test_whole_flight_improves_and_residual_is_visible(self):
        table = self.report['table']
        self.assertLess(table['replay_idle+admit']['flight']['over_limit'], table['replay_frame']['flight']['over_limit'])
        self.assertLessEqual(table['replay_idle+admit']['flight']['over_limit'], 1)


if __name__ == '__main__':
    unittest.main()
