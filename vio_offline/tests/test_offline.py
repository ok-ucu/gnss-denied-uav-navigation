"""Offline tests for vio_offline (no ROS, no Docker, no downloaded dataset).

    cd vio_offline && python3 -m unittest discover -s tests -v

Synthetic recordings are rendered here with a known motion, so the tests check
signs and axes end to end: frames -> team's FlowFrontend/VioEKF -> trajectory.
"""
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import ahrs                       # noqa: E402
import allan_variance             # noqa: E402
import calib_check                # noqa: E402
import evaluate                   # noqa: E402
import recording as rec_io        # noqa: E402
import replay_vio                 # noqa: E402
import vio_source                 # noqa: E402
from rotations import (NED_ENU, R_to_euler, R_to_quat, euler_to_R, ned_frd_to_enu_flu, quat_to_R, rot_z)  # noqa: E402
from trajectory import Trajectory, read_csv, read_tum   # noqa: E402

G = 9.80665
FX, W, H = 300.0, 320, 240
STANDARD_MOUNT = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])    # cam x = right, cam y = back
ROTATED_MOUNT = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])    # cam x = back, cam y = left


def texture(size=1400, seed=3):
    rng = np.random.default_rng(seed)
    img = rng.integers(0, 255, (size // 8, size // 8)).astype(np.uint8)
    img = cv2.resize(img, (size, size), interpolation=cv2.INTER_CUBIC)
    return cv2.GaussianBlur(img, (0, 0), 1.2)


def render(tex, shift_x, shift_y):
    """Standard-mount frame in which ground features have moved by (+shift_x, +shift_y) pixels."""
    x0, y0 = tex.shape[1] / 2 - W / 2 - shift_x, tex.shape[0] / 2 - H / 2 - shift_y
    M = np.array([[1.0, 0.0, -x0], [0.0, 1.0, -y0]])
    return cv2.warpAffine(tex, M, (W, H), flags=cv2.INTER_LINEAR)


def write_recording(root, frames, frame_t, imu_t, gyro, acc, att_t=None, att=None, height=10.0, R_cam_imu=STANDARD_MOUNT,
                    velocity_ned=(0.0, 0.0, 0.0), timeshift=0.0):
    """Minimal recording in the team's format."""
    ns = lambda t: int(round(t * 1e9)) + 1_000_000_000_000
    (root / "mav0" / "cam0" / "data").mkdir(parents=True)
    (root / "mav0" / "imu0").mkdir(parents=True)
    with open(root / "mav0" / "cam0" / "data.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["#timestamp [ns]", "filename"])
        for t, img in zip(frame_t, frames):
            cv2.imwrite(str(root / "mav0" / "cam0" / "data" / f"{ns(t)}.png"), img)
            w.writerow([ns(t), f"{ns(t)}.png"])
    with open(root / "mav0" / "imu0" / "data.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["#timestamp [ns]", "gx", "gy", "gz", "ax", "ay", "az"])
        for t, g, a in zip(imu_t, gyro, acc):
            w.writerow([ns(t), *g, *a])
    if att_t is not None:
        with open(root / "attitude.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_ns", "roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"])
            for t, a in zip(att_t, att):
                w.writerow([ns(t), *a, 0, 0, 0])
    with open(root / "local_position.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_rx_ns", "x", "y", "z", "vx", "vy", "vz"])
        for t in imu_t[::5]:
            w.writerow([ns(t), velocity_ned[0] * t, velocity_ned[1] * t, -height, *velocity_ned])
    with open(root / "groundtruth.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ns", "north_m", "east_m", "down_m", "qw", "qx", "qy", "qz"])
        for t in imu_t[::5]:
            w.writerow([ns(t), velocity_ned[0] * t, velocity_ned[1] * t, -height, 1, 0, 0, 0])
    T = np.eye(4)
    T[:3, :3] = R_cam_imu
    (root / "camera_params.json").write_text(json.dumps(dict(
        camera_model="pinhole", distortion_model="radtan", fx=FX, fy=FX, cx=W / 2 - 0.5, cy=H / 2 - 0.5,
        dist=[0, 0, 0, 0], width=W, height=H, T_cam_imu=T.tolist(), timeshift_cam_imu_s=timeshift)))


def straight_flight(root, v_forward=2.0, v_right=0.0, height=10.0, seconds=3.0, fps=20, mount="standard"):
    """Level flight at constant velocity, heading north. Forward motion moves ground features towards
    image +y (back of the drone) for the standard mounting."""
    tex = texture()
    frame_t = np.arange(0, seconds, 1.0 / fps)
    frames = []
    for t in frame_t:
        # ground moves opposite to the drone: backwards (+y) for forward flight, to the left (-x) for flight to the right
        img = render(tex, -FX * v_right * t / height, FX * v_forward * t / height)
        frames.append(np.rot90(img, 1).copy() if mount == "rotated" else img)
    imu_t = np.arange(-0.2, seconds + 0.1, 0.005)
    n = len(imu_t)
    R = ROTATED_MOUNT if mount == "rotated" else STANDARD_MOUNT
    write_recording(root, frames, frame_t, imu_t, np.zeros((n, 3)), np.tile([0.0, 0.0, -G], (n, 1)),
                    att_t=imu_t[::4], att=np.zeros((len(imu_t[::4]), 3)), height=height, R_cam_imu=R,
                    velocity_ned=(v_forward, v_right, 0.0))
    if mount == "rotated":      # the rotated camera delivers a portrait frame
        p = json.loads((root / "camera_params.json").read_text())
        p.update(width=H, height=W, cx=H / 2 - 0.5, cy=W / 2 - 0.5)
        (root / "camera_params.json").write_text(json.dumps(p))


class Args:
    """Defaults of replay_vio.py's command line."""
    def __init__(self, recording, out, **kw):
        self.recording, self.out = str(recording), str(out)
        self.camera_params = None
        self.attitude, self.yaw0_deg, self.vertical = "auto", 0.0, "auto"
        self.gyro, self.timeshift, self.scale, self.rate = "sample", None, 1.0, 30.0
        self.start, self.end, self.only_sent, self.vio_dir = 0.0, None, False, None
        self.ekf, self.frontend = [], []
        self.__dict__.update(kw)


class RotationTests(unittest.TestCase):
    def test_euler_round_trip(self):
        for rpy in ((0.1, -0.2, 0.3), (-0.4, 0.5, -2.9), (0.0, 0.0, 3.0)):
            np.testing.assert_allclose(R_to_euler(euler_to_R(*rpy)), rpy, atol=1e-12)

    def test_quaternion_round_trip(self):
        R = euler_to_R(0.3, -0.7, 2.1)
        np.testing.assert_allclose(quat_to_R(R_to_quat(R)), R, atol=1e-12)

    def test_ned_enu_is_a_proper_rotation(self):
        self.assertAlmostEqual(np.linalg.det(NED_ENU), 1.0)
        np.testing.assert_allclose(NED_ENU @ [1.0, 2.0, 3.0], [2.0, 1.0, -3.0])

    def test_level_heading_north_in_enu(self):
        """FRD body level and pointing north = FLU body whose x axis points along ENU north (+y)."""
        R = ned_frd_to_enu_flu(euler_to_R(0, 0, 0))
        np.testing.assert_allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        np.testing.assert_allclose(R @ [0, 0, 1], [0, 0, 1], atol=1e-12)


class TrajectoryTests(unittest.TestCase):
    def make(self):
        t = np.linspace(0, 10, 51)
        p_ned = np.column_stack([2 * t, 0.5 * t, -10 - 0.1 * t])
        rpy = np.column_stack([0.01 * t, -0.02 * t, 0.1 * t])
        cov = np.tile(np.diag([1.0, 4.0, 9.0]), (len(t), 1, 1))
        return Trajectory.from_ned(t, p_ned, rpy=rpy, v_ned=np.tile([2, 0.5, -0.1], (len(t), 1)), cov_p_ned=cov,
                                   t0_ns=123456789, name="x")

    def test_ned_to_enu(self):
        traj = self.make()
        np.testing.assert_allclose(traj.p_enu[-1], [5.0, 20.0, 11.0])
        np.testing.assert_allclose(traj.p_ned[-1], [20.0, 5.0, -11.0])
        np.testing.assert_allclose(np.diag(traj.cov_p_enu[0]), [4.0, 1.0, 9.0])     # east var = old east var

    def test_csv_and_tum_round_trip(self):
        traj = self.make()
        with tempfile.TemporaryDirectory() as d:
            info = traj.write_all(d, "x")
            self.assertEqual(info["rows"], 51)
            back = read_csv(Path(d) / "x.csv")
            np.testing.assert_allclose(back.p_enu, traj.p_enu, atol=1e-5)
            np.testing.assert_allclose(back.cov_p_enu, traj.cov_p_enu, atol=1e-5)
            self.assertEqual(back.t0_ns, 123456789)
            tum = read_tum(Path(d) / "x.tum")
            np.testing.assert_allclose(tum.q, traj.q, atol=1e-6)
            first = (Path(d) / "x_euroc.csv").read_text().splitlines()[1].split(",")
            self.assertEqual(int(first[0]), 123456789)
            self.assertEqual(len(first), 11)


class EvaluateTests(unittest.TestCase):
    def reference(self):
        t = np.linspace(0, 60, 601)
        p = np.column_stack([30 * np.sin(t / 10), 20 * np.cos(t / 7), 0.2 * t])
        return Trajectory(t, p, np.tile([0, 0, 0, 1.0], (len(t), 1)))

    def test_yaw_alignment_recovers_a_rotated_copy(self):
        ref = self.reference()
        R = rot_z(np.radians(-75.0))
        est = Trajectory(ref.t[::3] + 0.0, ref.p_enu[::3] @ R.T + [5, -3, 2], ref.q[::3])
        m, _ = evaluate.evaluate_one(est, ref, "yaw")
        self.assertLess(m["ate_horizontal_m"]["rmse"], 1e-6)
        self.assertAlmostEqual(m["alignment"]["yaw_deg"], 75.0, places=2)
        self.assertAlmostEqual(m["scale_from_sim3"], 1.0, places=3)

    def test_scale_error_is_reported(self):
        ref = self.reference()
        est = Trajectory(ref.t, ref.p_enu * 1.25, ref.q)
        m, _ = evaluate.evaluate_one(est, ref, "yaw")
        self.assertAlmostEqual(m["scale_from_sim3"], 0.8, places=3)
        self.assertGreater(m["ate_horizontal_m"]["rmse"], 1.0)

    def test_drift_with_alignment_on_first_seconds(self):
        ref = self.reference()
        drift = np.column_stack([0.05 * ref.t, np.zeros(len(ref.t)), np.zeros(len(ref.t))])
        est = Trajectory(ref.t, ref.p_enu + drift, ref.q)
        m, _ = evaluate.evaluate_one(est, ref, "origin")
        self.assertAlmostEqual(m["final_error_horizontal_m"], 3.0, places=2)
        self.assertAlmostEqual(m["drift_final_percent"], 3.0 / m["reference_path_m"] * 100, places=1)

    def test_samples_outside_reference_are_dropped(self):
        ref = self.reference()
        est = Trajectory(np.linspace(-10, 70, 81), np.zeros((81, 3)), np.tile([0, 0, 0, 1.0], (81, 1)))
        mask, _ = evaluate.associate(est, ref)
        self.assertEqual(int(mask.sum()), 60)        # 1..60 s: nothing before or after the reference span


class AhrsTests(unittest.TestCase):
    def test_static_tilt(self):
        R = euler_to_R(0.2, -0.1, 0.0)
        f = R.T @ np.array([0.0, 0.0, -G])
        t = np.arange(0, 2, 0.005)
        rpy = ahrs.mahony(t, np.zeros((len(t), 3)), np.tile(f, (len(t), 1)))
        np.testing.assert_allclose(rpy[-1, :2], [0.2, -0.1], atol=1e-3)

    def test_yaw_integrates_gyro(self):
        t = np.arange(0, 4, 0.005)
        gyro = np.tile([0.0, 0.0, 0.25], (len(t), 1))
        rpy = ahrs.mahony(t, gyro, np.tile([0.0, 0.0, -G], (len(t), 1)), yaw0=0.5)
        self.assertAlmostEqual(rpy[-1, 2], 0.5 + 0.25 * t[-1], places=2)


class AllanTests(unittest.TestCase):
    def test_white_noise_density_is_recovered(self):
        rate, density = 200.0, 2.0e-3
        rng = np.random.default_rng(1)
        x = rng.normal(0, density * np.sqrt(rate), 200 * 600)
        tau, adev = allan_variance.allan_deviation(x, rate)
        n, _ = allan_variance.noise_parameters(tau, adev)
        self.assertAlmostEqual(n / density, 1.0, delta=0.1)

    def test_random_walk_is_recovered(self):
        rate, density, walk = 100.0, 1.0e-3, 5.0e-4
        rng = np.random.default_rng(2)
        n = int(rate * 3600)
        x = rng.normal(0, density * np.sqrt(rate), n) + np.cumsum(rng.normal(0, walk / np.sqrt(rate), n))
        tau, adev = allan_variance.allan_deviation(x, rate)
        _, k = allan_variance.noise_parameters(tau, adev)
        self.assertAlmostEqual(k / walk, 1.0, delta=0.5)


class RecordingTests(unittest.TestCase):
    SAMPLE = HERE.parents[1] / "recordings" / "sample_flight"
    PARAMS = HERE.parents[1] / "calibration" / "camera_params.json"

    @unittest.skipUnless(SAMPLE.exists() and PARAMS.exists(), "team sample recording is not checked out")
    def test_team_sample_recording_loads(self):
        rec = rec_io.load(self.SAMPLE, self.PARAMS)
        self.assertEqual(len(rec.frame_t), 150)
        self.assertEqual(len(rec.imu_t), 500)
        self.assertEqual(len(rec.att_t), 150)
        self.assertAlmostEqual(rec.frame_t[0], rec.calib.timeshift_cam_imu_s, places=9)
        self.assertLess(rec.calib.mount_deviation_deg(), 0.01)       # the mounting vio_frontend.py assumes
        self.assertEqual(rec.gt_source.split(" ")[0], "mav0/gps0")
        # GPS track of the sample: 2 m/s to the north
        north = rec.gt.p_enu[-1, 1] - rec.gt.p_enu[0, 1]
        self.assertAlmostEqual(north / (rec.gt.t[-1] - rec.gt.t[0]), 2.0, delta=0.05)

    def test_calibration_adapter(self):
        T = np.eye(4)
        T[:3, :3] = ROTATED_MOUNT
        calib = rec_io.CameraCalib(FX, FX, 0, 0, W, H, T_cam_imu=T)
        self.assertAlmostEqual(calib.mount_deviation_deg(), 90.0, places=6)
        self.assertAlmostEqual(calib.optical_axis_tilt_deg(), 0.0, places=6)
        # the IMU's forward axis must map to "left" of the camera-aligned body (camera is yawed by 90 deg)
        np.testing.assert_allclose(calib.R_viobody_imu @ [1, 0, 0], [0, -1, 0], atol=1e-12)


class ReplayTests(unittest.TestCase):
    """End to end on rendered frames: sign, axes and scale of the estimated velocity."""

    def run_flight(self, **kw):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name) / "rec"
        root.mkdir()
        straight_flight(root, **kw)
        summary = replay_vio.run(Args(root, Path(tmp.name) / "out"))
        return summary, read_csv(Path(tmp.name) / "out" / "vio.csv"), Path(tmp.name) / "out"

    def test_uses_unmodified_team_estimator(self):
        _, info = vio_source.locate()
        self.assertTrue(info["matches_pin"], "vio_core.py / vio_frontend.py differ from the pinned vio-setup commit")

    def test_forward_flight_is_north(self):
        summary, traj, _ = self.run_flight(v_forward=2.0)
        v_ned = traj.v_enu[-1] @ NED_ENU.T
        np.testing.assert_allclose(v_ned[:2], [2.0, 0.0], atol=0.1)
        self.assertGreater(summary["accepted"], 0.9 * summary["counters"]["frames"] - 2)
        self.assertAlmostEqual(traj.p_ned[-1, 0] - traj.p_ned[0, 0], 2.0 * (traj.t[-1] - traj.t[0]), delta=0.3)
        self.assertAlmostEqual(traj.p_ned[-1, 2], -10.0, delta=0.2)

    def test_flight_to_the_right_is_east(self):
        _, traj, _ = self.run_flight(v_forward=0.0, v_right=1.5)
        np.testing.assert_allclose((traj.v_enu[-1] @ NED_ENU.T)[:2], [0.0, 1.5], atol=0.1)

    def test_rotated_camera_gives_the_same_answer(self):
        """A camera yawed by 90 deg, described by T_cam_imu, must not change the result."""
        summary, traj, _ = self.run_flight(v_forward=2.0, v_right=0.5, mount="rotated")
        self.assertAlmostEqual(summary["settings"]["mount_deviation_deg"], 90.0, places=1)
        np.testing.assert_allclose((traj.v_enu[-1] @ NED_ENU.T)[:2], [2.0, 0.5], atol=0.1)

    def test_outputs_and_ground_truth_comparison(self):
        _, traj, out = self.run_flight(v_forward=2.0)
        for name in ("vio.csv", "vio.tum", "vio_euroc.csv", "states.csv", "frames.csv", "run.json", "run.log"):
            self.assertTrue((out / name).exists(), name)
        rec = rec_io.load(out.parent / "rec")
        m, _ = evaluate.evaluate_one(traj, rec.gt, "origin")
        self.assertLess(m["ate_horizontal_m"]["max"], 0.3)


class CalibCheckTests(unittest.TestCase):
    def test_time_offset_and_axes_from_roll_motion(self):
        """Roll oscillation: image moves along x by fx * angle. The camera stamps are 30 ms late."""
        late = 0.030
        fps, seconds = 30, 8.0
        angle = lambda t: 0.03 * np.sin(2 * np.pi * 1.3 * t) + 0.015 * np.sin(2 * np.pi * 2.9 * t + 1.0)
        rate = lambda t: (angle(t + 1e-4) - angle(t - 1e-4)) / 2e-4
        tex = texture()
        frame_t = np.arange(0, seconds, 1.0 / fps)
        # positive roll (rotation about body x) moves ground features towards image +x by fx * angle
        frames = [render(tex, FX * angle(t - late), 0.0) for t in frame_t]
        imu_t = np.arange(-0.3, seconds + 0.3, 0.004)
        gyro = np.column_stack([rate(imu_t), np.zeros(len(imu_t)), np.zeros(len(imu_t))])
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            write_recording(root, frames, frame_t, imu_t, gyro, np.tile([0.0, 0.0, -G], (len(imu_t), 1)))
            vio_source.locate()
            rec = rec_io.load(root)
            t_prev, t, flow = calib_check.measure_flow(rec, 1.0, 0.0, None, log=lambda *_: None)
            r = calib_check.analyse(rec, t_prev, t, flow)
        self.assertAlmostEqual(r["time_offset_s"], -late, delta=0.006)
        self.assertGreater(r["axis_correlation"]["xx"], 0.9)
        self.assertAlmostEqual(r["slope_flow_over_gyro"][0], 1.0, delta=0.12)


if __name__ == "__main__":
    unittest.main()
