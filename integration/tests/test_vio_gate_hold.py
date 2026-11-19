"""Gate-closed hold (option D) and the separate VISO_QUAL_MIN preflight check."""
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from preflight_ekf3 import AP444_COMMIT, check_profile, viso_quality_min_error, EXPECTED, PORTS
from vio_gate_hold import (ProducerHold, TransportHold, frame_cause, load_budget, nonvisual_ok)
from vio_quality_gate import QualityGate

ROOT = Path(__file__).resolve().parents[1]
SIM = dict(max_sim_ns=500_000_000, max_host_ns=1_200_000_000)
OK = dict(imu_age=0., frame_age=.01, clock_age=.004, max_imu_gap=.001, frame_gap=.1,
          covariance=np.eye(9)*.01, state_finite=True)


def fly(bad, ratio, hold_on, frames=range(30), budget=SIM):
    """Real gate at 10 Hz; host = sim/ratio; transport watchdog polled every 2 ms."""
    gate, producer, transport = QualityGate(), ProducerHold(budget, 's'), TransportHold(budget)
    published, statuses, last_odom, fired, poll = [], [], None, None, 0
    for i in frames:
        t = round(i*.1, 9)
        stamp, host = round(t*1e9), round(t/ratio*1e9)
        while fired is None and poll < host:
            if (last_odom is not None and poll-last_odom > 500_000_000
                    and not (hold_on and transport.valid(poll, round(poll*ratio)))):
                fired = ('cap' if transport.cap_exceeded(poll, round(poll*ratio)) else 'watchdog', poll)
            poll += 2_000_000
        if gate.assess(t, t/ratio, inliers=0 if i in bad else 120, contrast=30., audit_ok=i not in bad,
                       ekf_ok=i not in bad, consistency_ok=True, **{k: v for k, v in OK.items()}):
            producer.normal(stamp, host/1e9)
            transport.normal(stamp, host)
            published.append(t)
            last_odom = host
            continue
        message = producer.status(stamp, host/1e9, 0, frame_cause(gate.reason, False), True)
        if message is not None:
            statuses.append((t, message['cause']))
            if hold_on:
                transport.on_status(json.dumps(message), host, stamp+4_000_000, 's')
    return published, statuses, fired


def status(seq, frame_ms, start_ms=1100, normal_ms=1000, cause='LOW_INLIERS', session='s'):
    return json.dumps(dict(schema=1, session=session, epoch=0, seq=seq, frame_stamp_ns=frame_ms*1_000_000,
                           hold_start_ns=start_ms*1_000_000, last_normal_stamp_ns=normal_ms*1_000_000,
                           cause=cause, frames_closed=1))


class GateHoldTests(unittest.TestCase):
    def test_single_rejection_hold_keeps_route_alive_without_odometry(self):
        without = fly({11}, .6, hold_on=False)
        published, statuses, fired = fly({11}, .6, hold_on=True)
        self.assertEqual(without[2][0], 'watchdog')                 # today: latches
        self.assertIsNone(fired)                                    # with the hold: no latch
        self.assertEqual(published, without[0])                     # no odometry added during the hold
        self.assertNotIn(1.1, published)
        self.assertEqual(published[published.index(1.0)+1], 1.5)    # full 3-frame/250 ms reopen rule
        self.assertEqual(statuses, [(1.1, 'LOW_INLIERS'), (1.2, 'RECOVERING'), (1.3, 'RECOVERING'),
                                    (1.4, 'RECOVERING')])
        self.assertIsNone(fly({11}, 1., hold_on=True)[2])

    def test_caps_are_counted_from_first_closed_frame(self):
        # Sim cap: seven bad frames; the producer stops at 0.5 s and the transport's own /clock cap
        # (0.5 s + 200 ms freshness) latches before the gate reopens at 2.1 s.
        published, _, fired = fly(set(range(11, 18)), 1., hold_on=True)
        self.assertEqual(fired, ('cap', 1_800_000_000+2_000_000))
        self.assertEqual(published[published.index(1.0)+1], 2.1)
        transport = TransportHold(dict(SIM, max_sim_ns=10**12))  # producer-independent host cap
        transport.normal(1_000_000_000, 0)
        for k in range(13):                                        # fresh status every 100 ms host, 0..1.2 s
            self.assertTrue(transport.on_status(status(k, 1100+10*k), k*100_000_000,
                                                (1100+10*k)*1_000_000, 's'))
        self.assertTrue(transport.valid(1_200_000_000, 1_240_000_000))
        self.assertFalse(transport.valid(1_200_000_001, 1_240_000_000))  # statuses never move the anchor
        self.assertTrue(transport.cap_exceeded(1_300_000_000, 1_240_000_000))
        sim = TransportHold(SIM)
        sim.normal(1_000_000_000, 0)
        for k, ms in enumerate((1100, 1400, 1600)):
            sim.on_status(status(k, ms), k, ms*1_000_000, 's')
        self.assertFalse(sim.cap_exceeded(3, 1_604_000_000))             # exactly 0.5 s: allowed
        sim.on_status(status(3, 1700), 4, 1_700_000_000, 's')
        self.assertTrue(sim.cap_exceeded(5, 1_704_000_000))              # 0.6 s of closed frames
        self.assertFalse(TransportHold(SIM).cap_exceeded(0, None))     # no hold, nothing to exceed

    def test_sequence_gap_ends_hold_only_and_foreign_status_latches(self):
        transport = TransportHold(SIM)
        transport.normal(1_000_000_000, 0)
        self.assertTrue(transport.on_status(status(0, 1100), 1, 1_100_000_000, 's'))
        self.assertFalse(transport.on_status(status(2, 1200), 2, 1_200_000_000, 's'))   # gap: no raise
        self.assertIsNone(transport.hold)
        self.assertFalse(transport.on_status(status(3, 1300), 3, 1_300_000_000, 's'))   # no re-entry
        for row in (status(4, 1400, session='other'), status(4, 1400, cause='OTHER'), '{"schema":1}', 'x'):
            with self.subTest(row=row), self.assertRaises(ValueError):
                transport.on_status(row, 4, 1_400_000_000, 's')

    def test_no_hold_without_normal_sample_or_for_nonvisual_cause(self):
        self.assertEqual([frame_cause(r, False) for r in ('inliers', 'contrast', 'EKF not ready', 'IMU gap',
                          'covariance', 'stale at publication', 'readiness hysteresis 1/3 frames, 0.000/0.25 s')],
                         ['LOW_INLIERS', 'LOW_CONTRAST', None, None, None, None, 'RECOVERING'])
        self.assertTrue(nonvisual_ok(1., 1., **OK))
        for change in (dict(state_finite=False), dict(max_imu_gap=.05), dict(covariance=-np.eye(9))):
            self.assertFalse(nonvisual_ok(1., 1., **dict(OK, **change)))
        producer = ProducerHold(SIM, 's')
        self.assertIsNone(producer.status(1, 0., 0, 'LOW_INLIERS', True))           # startup
        producer.normal(1, 0.)
        producer.abort('IMU gap/reset/nonfinite', 2, 0.)
        self.assertIsNone(producer.status(3, 0., 0, 'LOW_INLIERS', True))           # after a reset
        producer.normal(4, 0.)
        self.assertIsNone(producer.status(5, 0., 0, 'LOW_INLIERS', False))          # non-visual check failed
        transport = TransportHold(SIM)
        self.assertFalse(transport.on_status(status(0, 1100, normal_ms=0), 0, 1_100_000_000, 's'))

    def test_budget_configuration_rules(self):
        self.assertIsNone(load_budget(None, None, None, simulation=True))
        evidence = ROOT/'gate_hold_budget_simulation.json'
        self.assertEqual(load_budget(.5, 1.2, evidence, simulation=True)['max_host_ns'], 1_200_000_000)
        for args, sim in (((.5, None, evidence), True), ((.5, 1.2, evidence), False), ((.4, 1.2, evidence), True)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                load_budget(*args, simulation=sim)
        with tempfile.TemporaryDirectory() as temp:
            hardware = Path(temp)/'hw.json'
            hardware.write_text(json.dumps(dict(schema=1, scope='hardware', max_sim_s=.5, max_host_s=.5)))
            with self.assertRaisesRegex(ValueError, 'hardware'):
                load_budget(.5, .5, hardware, simulation=False)

    def test_viso_quality_min_preflight(self):
        new = '92b0cd788ec29406f26c6f9c31d5ceedbd1cc538'   # Copter-4.6.3
        self.assertIsNone(viso_quality_min_error({'VISO_QUAL_MIN': 0}, new))
        self.assertIsNone(viso_quality_min_error({'VISO_QUAL_MIN': -1}, new))
        self.assertIn('> 0', viso_quality_min_error({'VISO_QUAL_MIN': 1}, new))
        self.assertIn('missing', viso_quality_min_error({}, new))
        self.assertIsNone(viso_quality_min_error({}, AP444_COMMIT))
        self.assertTrue(any('VISO_QUAL_MIN' in e for e in check_profile(dict(EXPECTED, VISO_QUAL_MIN=1), PORTS)))
        self.assertEqual(check_profile(dict(EXPECTED, VISO_QUAL_MIN=0), PORTS), [])


if __name__ == '__main__':
    unittest.main()
