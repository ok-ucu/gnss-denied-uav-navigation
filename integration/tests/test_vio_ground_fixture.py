"""Flight ground texture vs optical flow: run 20261118-flightfbf7d8854ed1.

The flight runner installed vio_ground_fixture --size-m 4 --tile-size-m .02
(2 cm checkerboard). A checkerboard repeats under a one-tile diagonal shift,
so flow is ambiguous by the lattice {(2a, 2b) cm : a+b even}. Every airborne
flow error > 1 cm in that run is such a lattice vector, 4 of them were
published, and inliers collapse with height because 2 cm tiles shrink to
~3 px at 1.3 m. The render bench below reproduces the recorded error with the
pinned frontend and shows the non-repeating `noise` pattern measures the same
motions exactly.
"""
import json
import os
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, os.environ.get('VIO_SOURCE_DIR', str(HERE.parent/'vio')))
try:
    import numpy as np
    import cv2
    import vio_ground_fixture as fixture
    import vio_flow_consistency as fc
    from vio_frontend import FlowFrontend
except ImportError:
    np = None

FX = 205.4696
W, H = 640, 480
RUN = HERE/'phase1_runs/20261118-flightfbf7d8854ed1/evidence'


def render(texture, size_m, north, east, height):
    """Nadir camera view, frontend mount: image x = east, image y = -north.

    Pre-blurs when minifying (what mipmapping does), then samples bilinearly.
    """
    ppm = texture.shape[0]/size_m
    k = (FX/height)/ppm
    image = cv2.GaussianBlur(texture, (0, 0), .5/k) if k < 1 else texture
    uc, vc = (east+size_m/2)*ppm, (north+size_m/2)*ppm
    m = np.float32([[k, 0, W/2-k*uc], [0, -k, H/2+k*vc]])
    return cv2.warpAffine(image, m, (W, H), flags=cv2.INTER_LINEAR, borderValue=0)


def flow_pair(texture, size_m, height, move, start=(.1, -.6)):
    """NED displacement (m) measured by the pinned frontend over 0.1 s."""
    a = render(texture, size_m, start[0], start[1], height)
    b = render(texture, size_m, start[0]+move[0], start[1]+move[1], height)
    frontend = FlowFrontend(FX)
    frontend.process(a, 1., height, np.zeros(3), np.eye(3))
    result = frontend.process(b, 1.1, height, np.zeros(3), np.eye(3))
    return a, result


@unittest.skipIf(np is None, 'NumPy/OpenCV required')
class FixtureTextures(unittest.TestCase):
    def test_checker_default_is_unchanged(self):
        for size, tile in ((1, .02), (4, .02), (8, .1)):
            pixels = int(round(size/tile))*20
            yy, xx = np.indices((pixels, pixels))
            legacy = np.where(((xx//20+yy//20) % 2) == 0, 210, 55).astype('uint8')
            np.testing.assert_array_equal(fixture.checker_texture(size, tile), legacy)

    def test_sdf_is_unchanged(self):
        path = Path('/run/integration/artifacts/flight-fixture/checker.png')
        legacy = f'''<sdf version="1.9"><model name="integration_vio_ground_checker"><static>true</static><pose>0 0 0.001 0 0 0</pose><link name="visual_only"><visual name="checker"><cast_shadows>false</cast_shadows><geometry><plane><normal>0 0 1</normal><size>4.0 4.0</size></plane></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>file://{path}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr></material></visual></link></model></sdf>'''
        self.assertEqual(fixture.fixture_sdf(4.0, path), legacy)
        recorded = json.loads((RUN/'flight-fixture.txt').read_text())['request']
        self.assertIn(json.dumps(legacy), recorded)          # byte-identical to what the flight used

    def test_noise_is_reproducible_and_seeded(self):
        a, b = fixture.noise_texture(2, .02, 0), fixture.noise_texture(2, .02, 0)
        np.testing.assert_array_equal(a, b)
        self.assertFalse(np.array_equal(a, fixture.noise_texture(2, .02, 1)))
        self.assertEqual(a.shape, (800, 800))
        self.assertGreater(a.std(), 40)

    def test_budget_and_scale_are_validated(self):
        with self.assertRaises(ValueError):
            fixture.noise_texture(32, .02)       # 12800 px > 6400
        with self.assertRaises(ValueError):
            fixture.noise_texture(4, .005)
        with self.assertRaises(ValueError):
            fixture.checker_texture(40, .1)


@unittest.skipIf(np is None, 'NumPy/OpenCV required')
class RenderBench(unittest.TestCase):
    """Pinned frontend on rendered views of each fixture texture."""
    @classmethod
    def setUpClass(cls):
        cls.checker = fixture.checker_texture(4, .02)
        cls.noise = fixture.noise_texture(8, .02)

    def test_checker_reproduces_recorded_error(self):
        # recorded t=47.1: true (+0.60, -2.06) cm, measured (-1.39, 0.00) cm
        _, result = flow_pair(self.checker, 4, .6, (.006, -.0206))
        np.testing.assert_allclose((result.v_xy*.1-[.006, -.0206])*100, [-2, 2], atol=.05)

    def test_checker_loses_features_with_height(self):
        _, low = flow_pair(self.checker, 4, .19, (.006, -.0206))
        _, high = flow_pair(self.checker, 4, 1.3, (.006, -.0206))
        self.assertGreater(low.n_inliers, 150)
        self.assertLess(high.n_inliers, 50)

    def test_noise_measures_motion_exactly(self):
        for height in (.19, .6, 1.3):
            for move in ((.006, -.0206), (.02, .03), (-.015, .0)):
                with self.subTest(height=height, move=move):
                    _, result = flow_pair(self.noise, 8, height, move)
                    self.assertIsNotNone(result)
                    np.testing.assert_allclose(result.v_xy*.1, move, atol=5e-4)   # < 0.5 mm, signs NED
                    self.assertGreater(result.n_inliers, 100)

    def test_climb_with_scale_compensation(self):
        # Liftoff at 0.8 m/s, as the source runs it (vio_scale_compensation).
        from vio_scale_compensation import compensate_previous, predicted_scale
        def climb(texture, size, h0, h1, move):
            a = render(texture, size, .1, -.6, h0)
            b = render(texture, size, .1+move[0], -.6+move[1], h1)
            frontend = FlowFrontend(FX)
            frontend.process(a, 1., h0, np.zeros(3), np.eye(3))
            frontend.prev = compensate_previous(frontend.prev, predicted_scale(h0, h1))
            return frontend.process(b, 1.1, h1, np.zeros(3), np.eye(3))
        for h0, move in ((.3, (0., 0.)), (.5, (.005, .003)), (.9, (0., 0.))):
            result = climb(self.noise, 8, h0, h0+.08, move)
            np.testing.assert_allclose(result.v_xy*.1, move, atol=5e-4)
            self.assertGreater(result.n_inliers, 150)
        # recorded liftoff: inliers 49 -> 39 -> 15 on the checker
        self.assertLess(climb(self.checker, 4, .3, .38, (0., 0.)).n_inliers, 40)

    def test_texture_periods_and_flow_check(self):
        for height in (.19, .6, 1.3):
            self.assertTrue(fc.texture_periods(render(self.checker, 4, .1, -.6, height)))
            self.assertEqual(fc.texture_periods(render(self.noise, 8, .1, -.6, height)), [])
        image, result = flow_pair(self.noise, 8, 1.3, (.006, -.0206))
        truth = np.array([.06, -.206])
        out = fc.check(result.v_xy, result.sigma_v, truth, np.eye(2)*.0044, np.zeros(3), np.eye(3),
                       1.3, .1, FX, fc.texture_periods(image))
        self.assertTrue(out['ok'], out)


@unittest.skipIf(np is None or not RUN.exists(), 'run evidence required')
class RecordedFlight(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rows = [json.loads(line) for line in (RUN/'flight-source/stream.jsonl').read_text().splitlines()]
        rows = [r for r in rows if r.get('gt_ned')]
        cls.pairs = []
        for prev, row in zip(rows, rows[1:]):
            if row.get('flow_xy') and -row['gt_ned'][2] > .3:
                dt = row['t']-prev['t']
                true = (np.array(row['gt_ned'])-np.array(prev['gt_ned']))[:2]
                vz = -(row['gt_ned'][2]-prev['gt_ned'][2])/dt
                cls.pairs.append((prev, row, dt, true, np.array(row['flow_xy'])*dt, vz))

    def test_flight_used_two_centimetre_checker(self):
        host = (RUN/'flight-host.jsonl').read_text()
        self.assertIn('vio_ground_fixture.py create --directory /run/integration/artifacts/flight-fixture '
                      '--size-m 4 --tile-size-m .02', host)

    def test_errors_are_checker_lattice_vectors(self):
        wrong = [(row, (m-d)*100, vz) for _, row, _, d, m, vz in self.pairs if np.linalg.norm(m-d) > .01]
        self.assertGreaterEqual(len(wrong), 30)
        fused_level = 0
        for row, error, vz in wrong:
            steps = np.round(error/2)
            if row['inliers'] >= 19:
                # Diagonal checker lattice {(2a, 2b) cm : a+b even}. Frames with
                # 15-18 inliers (46.4, 46.8, 48.8; all rejected by the audit)
                # mix partial aliases and are not single lattice steps.
                self.assertEqual(int(steps.sum()) % 2, 0, row['t'])
            if row['flow_accepted'] and abs(vz) < .1:
                # Level flight and fused into the EKF: exactly one lattice vector.
                np.testing.assert_allclose(error, steps*2, atol=.3, err_msg=str(row['t']))
                fused_level += 1
        self.assertGreaterEqual(fused_level, 5)

    def test_wrong_velocities_were_published(self):
        published = [row['t'] for _, row, _, d, m, _ in self.pairs if row.get('published') and np.linalg.norm(m-d) > .01]
        self.assertEqual([round(t, 1) for t in published], [42.2, 43.2, 50.5, 50.7])

    def test_flow_check_rejects_every_wrong_flow(self):
        r = np.eye(3)
        for prev, row, dt, true, measured, _ in self.pairs:
            if not row.get('flow_accepted'):
                continue
            depth = -row['gt_ned'][2]
            jac = fc.velocity_to_flow_jacobian(r, depth, FX)
            periods = []
            for v in ((.02, .02), (.02, -.02)):
                dx, dy = (int(x) for x in np.round(jac@np.array(v)))
                periods.append((dx, dy, .9) if dy > 0 or (dy == 0 and dx > 0) else (-dx, -dy, .9))
            prior_cov = np.array(prev['covariance'])[3:5, 3:5]+np.eye(2)*(.3*dt)**2
            out = fc.check(row['flow_xy'], row['sigma_v'], prev['state'][3:5], prior_cov,
                           np.zeros(3), r, depth, dt, FX, periods)
            if np.linalg.norm(measured-true) > .01:
                self.assertFalse(out['ok'], row['t'])

    def test_transport_received_every_publication(self):
        published = [json.loads(line)['t'] for line in (RUN/'flight-source/publications.jsonl').read_text().splitlines()]
        odom = [json.loads(line) for line in (RUN/'flight-transport/odom.jsonl').read_text().splitlines()]
        acquisition = [o['acquisition_ns'] for o in odom]
        self.assertEqual([round(t*1e9) for t in published], acquisition)
        self.assertTrue(all(b > a for a, b in zip(acquisition, acquisition[1:])))
        for o in odom:
            for key in ('pose_covariance', 'twist_covariance'):
                c = np.array(o[key]).reshape(6, 6)[:3, :3]
                self.assertTrue(np.isfinite(c).all() and np.allclose(c, c.T) and np.linalg.eigvalsh(c).min() >= -1e-12)

    def test_watchdog_gap(self):
        events = [json.loads(line) for line in (RUN/'flight-transport/events.jsonl').read_text().splitlines()]
        watchdog = next(e for e in events if e.get('stage') == 'transport_watchdog')
        self.assertEqual(watchdog['acquisition_ns'], 42300000000)    # last delivered sample
        self.assertEqual(watchdog['flight_state'], 'CLIMB')
        rows = {round(json.loads(line)['t'], 1): json.loads(line)
                for line in (RUN/'flight-source/stream.jsonl').read_text().splitlines()}
        self.assertEqual(rows[42.4]['skip_reason'], 'distributed flow disagreement/unavailable')
        self.assertEqual(rows[42.6]['skip_reason'], 'inliers')
        self.assertTrue(rows[43.0]['published'])


if __name__ == '__main__':
    unittest.main()
