"""Run 20261118-flightabe2d67e5306: watchdog at liftoff on the noise ground.

Evidence chain pinned here:
  * the source that ran was 53eeddf (flow consistency check not wired in);
  * the watchdog fired at acquisition 41.7 s, 4.4 s into CLIMB, while the drone
    lifted from 0.20 to 0.31 m and the frontend lost its features; the later
    publications (to 47.3 s) arrived after the transport had latched;
  * every 'waiting for downstream DDS discovery' after startup happens after
    the transport reached DONE and exited: a consequence, not the cause;
  * the transport received every publication; nothing was lost in transit;
  * the noise texture removed the flow aliasing of run fbf7 in real flight;
  * the drone's own hard shadow (recorded in the fbf7 probe frame, same camera
    and sun) reproduces the liftoff feature collapse on the noise ground.
"""
import json
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
    import cv2
except ImportError:
    np = None

RUN = HERE/'phase1_runs/20261118-flightabe2d67e5306/evidence'
FBF7 = HERE/'phase1_runs/20261118-flightfbf7d8854ed1/evidence'
FX = 205.4696


def jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


@unittest.skipIf(np is None or not RUN.exists(), 'run evidence required')
class RecordedLiftoffFailure(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rows = {round(r['t'], 1): r for r in jsonl(RUN/'flight-source/stream.jsonl') if r.get('t') is not None}
        cls.flight = jsonl(RUN/'flight-transport/flight.jsonl')
        cls.events = jsonl(RUN/'flight-transport/events.jsonl')
        cls.done_s = next(r['host_ns'] for r in cls.flight if r.get('state') == 'DONE')/1e9

    def test_source_that_ran_lacked_the_flow_check(self):
        config = json.loads((RUN/'flight-source/configuration.json').read_text())
        self.assertNotIn('flow_consistency', config)
        self.assertEqual(config['source_sha256']['run_vio_stream.py'][:12], '9e9e1e11bac0')   # 53eeddf

    def test_watchdog_fired_at_liftoff_not_after_47_3(self):
        watchdog = next(e for e in self.events if e.get('stage') == 'transport_watchdog')
        self.assertEqual(watchdog['acquisition_ns'], 41_700_000_000)
        self.assertEqual(watchdog['flight_state'], 'CLIMB')
        self.assertTrue(self.rows[41.7]['published'])
        withheld = {t: self.rows[t]['skip_reason'] for t in (41.8, 41.9, 42.0, 42.1, 42.2)}
        self.assertEqual(set(withheld.values()), {'distributed flow disagreement/unavailable', 'inliers'})
        for t in withheld:
            self.assertLess(-self.rows[t]['gt_ned'][2], .35)                 # liftoff band
        self.assertTrue(self.rows[42.6]['published'])                         # too late: latched at 42.289
        last = jsonl(RUN/'flight-source/publications.jsonl')[-1]
        self.assertEqual(round(last['t'], 1), 47.3)
        self.assertLess(last['publish_monotonic_s'], self.done_s)              # just before transport exit

    def test_recorded_rejection_burst_still_exceeds_watchdog_in_gate_replay(self):
        from vio_quality_gate import QualityGate

        gate = QualityGate()
        output = []
        for tick in range(412, 427):
            row = self.rows[tick / 10]
            if gate.assess(row['t'], row['wall'], imu_age=row['imu_age'],
                           frame_age=row['source_age'], clock_age=row['clock_age'],
                           max_imu_gap=.001, inliers=row.get('inliers') or 0,
                           contrast=row['contrast'], covariance=row['covariance'],
                           state_finite=bool(np.isfinite(row['state']).all()),
                           audit_ok=row['audit']['ok'], ekf_ok=row['flow_accepted']):
                output.append(row)
        before, after = output[-2:]
        self.assertEqual((before['t'], after['t']), (41.7, 42.6))
        self.assertAlmostEqual(after['t'] - before['t'], .9)
        self.assertAlmostEqual(after['wall'] - before['wall'], .900049983, places=6)
        self.assertGreater(after['wall'] - before['wall'], .5)
        # Fixed-RTF sensitivity uses a separate host timeline, never a measured interval.
        self.assertAlmostEqual((after['t'] - before['t']) / .656, 1.3719512195)

    def test_dds_discovery_skips_are_after_transport_exit(self):
        dds = [r for r in jsonl(RUN/'flight-source/skips.jsonl') if r['reason'] == 'waiting for downstream DDS discovery']
        startup = [r for r in dds if r['t'] < 25]
        after = [r for r in dds if r['t'] >= 25]
        self.assertEqual(len(startup), 1)
        self.assertGreater(len(after), 10)
        self.assertTrue(all(r['host_monotonic_s'] > self.done_s for r in after))

    def test_transport_received_every_publication(self):
        published = [round(p['t']*1e9) for p in jsonl(RUN/'flight-source/publications.jsonl')]
        received = [o['acquisition_ns'] for o in jsonl(RUN/'flight-transport/odom.jsonl')]
        self.assertEqual(published, received)
        self.assertTrue(all(b > a for a, b in zip(received, received[1:])))
        reference = next(e for e in self.events if e['event'] == 'diagnostic_reference_initialized')['initial_acquisition_ns']
        mapped = {m['acquisition_ns'] for m in jsonl(RUN/'flight-transport/mapping.jsonl')}
        unmapped = [a for a in received if a <= 41_700_000_000 and a not in mapped]
        self.assertTrue(all(a <= reference for a in unmapped))

    def test_noise_ground_removed_flow_aliasing_in_flight(self):
        def airborne_errors(stream):
            rows = [r for r in jsonl(stream) if r.get('gt_ned')]
            out = []
            for prev, row in zip(rows, rows[1:]):
                if row.get('flow_xy') and -row['gt_ned'][2] > .3:
                    dt = row['t']-prev['t']
                    true = (np.array(row['gt_ned'])-np.array(prev['gt_ned']))[:2]
                    out.append(np.linalg.norm(np.array(row['flow_xy'])*dt-true)*100)
            return np.array(out)
        noise = airborne_errors(RUN/'flight-source/stream.jsonl')
        checker = airborne_errors(FBF7/'flight-source/stream.jsonl')
        self.assertGreaterEqual(len(noise), 15)
        self.assertLess(noise.max(), .5)                  # cm per frame
        self.assertGreater(checker.max(), 10)             # run fbf7, 2 cm checker

    def test_features_collapse_only_in_the_liftoff_band(self):
        low = [self.rows[t].get('inliers') or 0 for t in (41.8, 41.9, 42.0, 42.1, 42.2)]
        high = [self.rows[t]['inliers'] for t in (42.9, 43.0, 43.1, 43.2, 43.4)]
        self.assertLess(max(low), 40)
        self.assertGreater(min(high), 150)


@unittest.skipIf(np is None or not FBF7.exists(), 'probe frame required')
class SelfShadowBench(unittest.TestCase):
    """Recorded drone shadow over the noise ground, pinned frontend + audit."""
    @classmethod
    def setUpClass(cls):
        import vio_ground_fixture as fixture
        from test_vio_ground_fixture import render
        probe = cv2.cvtColor(cv2.imread(str(FBF7/'integrated/frame_0100.ppm')), cv2.COLOR_BGR2GRAY).astype(np.float32)
        lit, dark = 64.7, 37.4                                       # recorded levels, h = 0.19 m
        cls.shade0 = np.clip((probe-dark)/(lit-dark), 0, 1)*(1-dark/lit)+dark/lit
        # world sun direction (-0.5, 0.1, -0.9): the ground shadow keeps its size,
        # so in the image it scales about this fixed point as the drone climbs
        cls.anti = (320-.5/.9*FX, 240-.1/.9*FX)
        cls.texture, cls.render = fixture.noise_texture(8, .02), staticmethod(render)

    def frame(self, h, shadow):
        image = self.render(self.texture, 8, .1, -.6, h).astype(np.float32)
        if shadow:
            s = .19/h
            m = np.float32([[s, 0, self.anti[0]*(1-s)], [0, s, self.anti[1]*(1-s)]])
            image *= cv2.warpAffine(self.shade0, m, (640, 480), flags=cv2.INTER_LINEAR, borderValue=1.)
        return np.clip(image, 0, 255).astype(np.uint8)

    def liftoff_step(self, h0, h1, shadow):
        from vio_frontend import FlowFrontend
        from vio_scale_compensation import compensate_previous, predicted_scale
        from vio_quality_gate import measure_distributed_flow, judge_distributed_flow
        a, b = self.frame(h0, shadow), self.frame(h1, shadow)
        scale = predicted_scale(h0, h1)
        frontend = FlowFrontend(FX)
        frontend.process(a, 1., h0, np.zeros(3), np.eye(3))
        frontend.prev = compensate_previous(frontend.prev, scale)
        result = frontend.process(b, 1.1, h1, np.zeros(3), np.eye(3))
        measured = measure_distributed_flow(a, b, .1, scale)
        audit = judge_distributed_flow(measured, h1, FX, np.zeros(3), np.eye(3), result.v_xy)
        return result, audit

    @unittest.skipUnless((RUN/'flight-source/stream.jsonl').is_file(), 'abe2 source evidence required')
    def test_no_shadow_clears_visual_checks_across_recorded_rejection_band(self):
        # Use the recorded body-height trajectory only as a bench input.
        records = {round(r['t'], 1): r for r in jsonl(RUN/'flight-source/stream.jsonl')
                   if 41.69 <= r.get('t', 0) <= 42.31}
        for tick in range(418, 424):
            previous, current = records[(tick-1)/10], records[tick/10]
            with self.subTest(t=current['t']):
                result, audit = self.liftoff_step(-previous['gt_ned'][2],
                                                  -current['gt_ned'][2], shadow=False)
                self.assertGreaterEqual(result.n_inliers, 150)
                self.assertTrue(audit['ok'], audit)

    def test_shadow_reproduces_liftoff_collapse(self):
        # recorded 41.7 -> 41.8 (0.19 -> 0.20 m): 35 inliers, ambiguous spatial support
        result, audit = self.liftoff_step(.19, .20, shadow=True)
        self.assertLess(result.n_inliers, 50)
        self.assertFalse(audit['ok'])

    def test_same_liftoff_without_shadow_is_clean(self):
        for h0, h1 in ((.19, .20), (.20, .21), (.23, .27), (.31, .35)):
            result, audit = self.liftoff_step(h0, h1, shadow=False)
            self.assertGreater(result.n_inliers, 150)
            self.assertTrue(audit['ok'], audit)
            self.assertLess(np.linalg.norm(result.v_xy), .01)    # no lateral motion, none measured


if __name__ == '__main__':
    unittest.main()
