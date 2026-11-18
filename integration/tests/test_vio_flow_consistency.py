"""Flow velocity vs IMU prior and periodic-texture aliasing (vio_flow_consistency).

Evidence: run 20261117-flight6f32c08e7aaf, airborne fused flow velocities were
off by one fixed ground displacement (-2.0, +2.0) cm whenever the true
displacement between the image pair exceeded ~1.45 cm (ground texture period
~2 cm). The pinned frontend is exercised on synthetic textures to show the
same mechanism and that the check rejects it while passing a random texture.
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
    import vio_flow_consistency as fc
    from vio_quality_gate import QualityGate
except ImportError:
    np = None

FX = 205.4696
RUN = HERE/'debug_evidence/20261117-flight6f32c08e7aaf/flight-source/stream.jsonl'


def rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz@ry@rx


def frontend_velocity(flow, gyro, r, depth):
    """The pinned frontend's formula (vio/vio_frontend.py), flow in px/s."""
    flow_rot = FX*np.array([gyro[0], gyro[1]])
    v_cam = -(depth/FX)*(np.asarray(flow)-flow_rot)
    return (r@np.array([-v_cam[1], v_cam[0], 0.]))[:2]


def periodic(shape, period, seed=0):
    yy, xx = np.mgrid[:shape[0], :shape[1]].astype(np.float32)
    rng = np.random.default_rng(seed)
    g = 128+50*np.sin(2*np.pi*xx/period)+50*np.sin(2*np.pi*yy/period)+rng.normal(0, 2, shape)
    return np.clip(g, 0, 255).astype(np.uint8)


def tiled(shape, period, seed=0):
    """A random tile repeated, as a renderer repeats a ground texture image."""
    tile = np.random.default_rng(seed).uniform(0, 255, (period, period)).astype(np.float32)
    return np.tile(tile, (shape[0]//period+2, shape[1]//period+2))[:shape[0], :shape[1]].astype(np.uint8)


def random_texture(shape, seed=0):
    rng = np.random.default_rng(seed)
    g = rng.uniform(0, 255, (shape[0]//4, shape[1]//4)).astype(np.float32)
    return np.clip(cv2.resize(g, (shape[1], shape[0]), interpolation=cv2.INTER_CUBIC), 0, 255).astype(np.uint8)


def shifted(img, dx, dy):
    m = np.float32([[1, 0, dx], [0, 1, dy]])
    return cv2.warpAffine(img, m, (img.shape[1], img.shape[0]), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REFLECT)


@unittest.skipIf(np is None, 'NumPy/OpenCV required')
class FrontendModel(unittest.TestCase):
    def test_flow_px_inverts_the_frontend(self):
        rng = np.random.default_rng(3)
        for _ in range(50):
            r = rot(*rng.uniform(-.3, .3, 2), rng.uniform(-np.pi, np.pi))
            gyro, depth, flow = rng.normal(0, .3, 3), rng.uniform(.3, 20), rng.normal(0, 40, 2)
            v = frontend_velocity(flow, gyro, r, depth)
            np.testing.assert_allclose(fc.flow_px(v, gyro, r, depth, FX), flow, atol=1e-9)


@unittest.skipIf(np is None, 'NumPy/OpenCV required')
class TexturePeriods(unittest.TestCase):
    def test_periodic_texture_found(self):
        for image in (periodic((240, 320), 6), tiled((240, 320), 6)):
            vectors = {(dx, dy) for dx, dy, _ in fc.texture_periods(image)}
            self.assertIn((6, 0), vectors)
            self.assertIn((0, 6), vectors)

    def test_random_and_blank_have_none(self):
        self.assertEqual(fc.texture_periods(random_texture((240, 320))), [])
        self.assertEqual(fc.texture_periods(np.random.default_rng(1).integers(0, 255, (240, 320)).astype(np.uint8)), [])
        self.assertEqual(fc.texture_periods(np.zeros((240, 320), np.uint8)), [])


@unittest.skipIf(np is None, 'NumPy/OpenCV required')
class AliasingWithPinnedFrontend(unittest.TestCase):
    """Same mechanism as the recorded flight, reproduced with vio/vio_frontend.py."""
    def run_pair(self, image, shift_px):
        from vio_frontend import FlowFrontend
        depth, dt, r, gyro = 1.1, .1, np.eye(3), np.zeros(3)
        frontend = FlowFrontend(FX)
        frontend.process(image, 1., depth, gyro, r)
        result = frontend.process(shifted(image, *shift_px), 1.+dt, depth, gyro, r)
        self.assertIsNotNone(result)
        truth = frontend_velocity(np.array(shift_px)/dt, gyro, r, depth)
        prior_cov = np.eye(2)*.0044          # recorded EKF velocity variance in flight
        periods = fc.texture_periods(image)
        return result, truth, fc.check(result.v_xy, result.sigma_v, truth, prior_cov, gyro, r, depth, dt, FX, periods)

    def test_periodic_texture_aliases_and_is_rejected(self):
        result, truth, out = self.run_pair(tiled((240, 320), 6), (3.8, 0.))      # > half period
        step = 6*1.1/FX/.1                                                       # one period, m/s
        self.assertAlmostEqual(np.linalg.norm(result.v_xy-truth), step, delta=.02)   # fooled by one step
        self.assertGreater(result.n_inliers, 100)                                # ...confidently
        self.assertFalse(out['ok'])   # 0.32 m/s step: already beyond the kinematic check

    def test_small_alias_only_the_texture_check_catches(self):
        # 4 px period: 0.21 m/s step, inside the kinematic tolerance (~0.25 m/s),
        # like the recorded 0.14 m/s errors.
        result, truth, out = self.run_pair(tiled((240, 320), 4), (2.6, 0.))
        self.assertAlmostEqual(np.linalg.norm(result.v_xy-truth), 4*1.1/FX/.1, delta=.02)
        self.assertFalse(out['ok'])
        self.assertIn('periodic texture', out['reason'])
        no_texture = fc.check(result.v_xy, result.sigma_v, truth, np.eye(2)*.0044, np.zeros(3), np.eye(3),
                              1.1, .1, FX, [])
        self.assertTrue(no_texture['ok'])   # why the IMU comparison alone is not enough

    def test_random_texture_same_motion_is_accepted(self):
        result, truth, out = self.run_pair(random_texture((240, 320)), (3.8, 0.))
        self.assertLess(np.linalg.norm(result.v_xy-truth), .02)
        self.assertTrue(out['ok'], out)

    def test_kinematic_inconsistency_rejected_without_texture(self):
        out = fc.check([.5, 0], .002, [0, 0], np.eye(2)*.0044, np.zeros(3), np.eye(3), 1.1, .1, FX, [])
        self.assertEqual(out['reason'], 'flow inconsistent with IMU prediction')
        self.assertTrue(fc.check([.05, 0], .002, [0, 0], np.eye(2)*.0044, np.zeros(3), np.eye(3), 1.1, .1, FX, [])['ok'])

    def test_nonfinite_inputs_fail_closed(self):
        out = fc.check([np.nan, 0], .002, [0, 0], np.eye(2), np.zeros(3), np.eye(3), 1.1, .1, FX, [])
        self.assertFalse(out['ok'])


@unittest.skipIf(np is None, 'NumPy/OpenCV required')
class GateLabel(unittest.TestCase):
    def test_consistency_failure_blocks_with_its_own_label(self):
        g = QualityGate()
        kw = dict(imu_age=.001, frame_age=.01, clock_age=.01, max_imu_gap=.001, inliers=50, contrast=40,
                  covariance=np.eye(9)*.01, state_finite=True, audit_ok=True, ekf_ok=False)
        self.assertFalse(g.assess(1., 1., consistency_ok=False, **kw))
        self.assertEqual(g.reason, 'flow inconsistent with IMU prediction / aliased')


@unittest.skipIf(np is None or not RUN.exists(), 'recorded flight evidence required')
class RecordedAliasing(unittest.TestCase):
    """Run 6f32c08e7aaf: the error is one ground lattice step, not noise."""
    def setUp(self):
        rows = [json.loads(line) for line in RUN.read_text().splitlines()]
        rows = [r for r in rows if r.get('gt_ned')]
        self.pairs = []
        for prev, row in zip(rows, rows[1:]):
            if row.get('flow_accepted') and -row['gt_ned'][2] > .9:
                dt = row['t']-prev['t']
                true = (np.array(row['gt_ned'])-np.array(prev['gt_ned']))[:2]
                self.pairs.append((row, prev, dt, true, np.array(row['flow_xy'])*dt))

    def test_errors_are_lattice_steps(self):
        bad = [(m-d, d) for _, _, _, d, m in self.pairs if np.linalg.norm(m-d) > .01]
        self.assertGreaterEqual(len(bad), 14)
        for error, true in bad:
            steps = np.round(error/.02)
            self.assertTrue(np.any(steps != 0))
            np.testing.assert_allclose(error, steps*.02, atol=.0045)   # whole 2 cm steps
            self.assertGreater(np.abs(true).max(), .0095)              # only past half a period

    def test_small_displacements_are_correct(self):
        small = [np.linalg.norm(m-d) for _, _, _, d, m in self.pairs if np.abs(d).max() < .009]
        self.assertGreater(len(small), 15)
        self.assertLess(max(small), .006)

    def test_check_rejects_every_aliased_frame(self):
        r = np.eye(3)
        for row, prev, dt, true, measured in self.pairs:
            depth = -row['gt_ned'][2]
            jac = fc.velocity_to_flow_jacobian(r, depth, FX)
            periods = []
            for v in ((.02, 0.), (0., .02)):
                dx, dy = (int(x) for x in np.round(jac@np.array(v)))
                periods.append((dx, dy, .9) if dy > 0 or (dy == 0 and dx > 0) else (-dx, -dy, .9))
            prior_cov = np.array(prev['covariance'])[3:5, 3:5]+np.eye(2)*(.3*dt)**2
            out = fc.check(row['flow_xy'], row['sigma_v'], prev['state'][3:5], prior_cov,
                           np.zeros(3), r, depth, dt, FX, periods)
            if np.linalg.norm(measured-true) > .01:
                self.assertFalse(out['ok'], row['t'])


if __name__ == '__main__':
    unittest.main()
