"""Load a recording for offline replay.

One loader for:
  * the team's recorder format (dataset/recorder.py, see dataset/README.md);
  * a plain EuRoC/ASL folder (mav0/cam0, mav0/imu0, state_groundtruth_estimate0);
  * folders produced by convert_insane.py (team format + groundtruth.csv).

Time: every stream is converted to seconds from the FIRST CAMERA FRAME
(`t0_ns`). Camera stamps get the Kalibr time shift applied
(t_imu = t_cam + timeshift_cam_imu_s), so all streams are on the IMU clock.

IMU: [gx gy gz] rad/s and [ax ay az] m/s^2 (specific force) in the autopilot
body frame FRD, exactly as dataset/recorder.py writes them.
"""
import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from rotations import FRD_FLU, NED_ENU, quat_to_R, R_to_quat
from trajectory import Trajectory

# Camera axes assumed by vio/vio_frontend.py: image x = body right (+Y),
# image y = body back (-X), optical axis = body down (+Z).
R_CAM_FROM_VIO_BODY = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

WGS84_A = 6378137.0
WGS84_E2 = (1.0 / 298.257223563) * (2.0 - 1.0 / 298.257223563)


def _rows(path):
    """Numeric CSV rows; header and comment lines are skipped."""
    path = Path(path)
    if not path.exists():
        return []
    out = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.reader(f):
            if row and row[0].strip() and (row[0].strip()[0].isdigit() or row[0].strip()[0] == "-"):
                out.append([c.strip() for c in row])
    return out


def _table(path, ncols):
    rows = [r[:ncols] for r in _rows(path) if len(r) >= ncols]
    if not rows:
        return np.zeros(0, dtype=np.int64), np.zeros((0, ncols - 1))
    t_ns = np.array([int(r[0]) for r in rows], dtype=np.int64)
    return t_ns, np.array([[float(x) for x in r[1:]] for r in rows], dtype=float)


def ecef(lat_deg, lon_deg, alt_m):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    return np.array([(n + alt_m) * math.cos(lat) * math.cos(lon),
                     (n + alt_m) * math.cos(lat) * math.sin(lon),
                     (n * (1.0 - WGS84_E2) + alt_m) * math.sin(lat)])


def lla_to_enu(lla, origin):
    """WGS84 lat/lon/alt rows -> local ENU metres (same method as Person 1's gps_trajectory_export.py)."""
    lat0, lon0 = math.radians(origin[0]), math.radians(origin[1])
    R = np.array([[-math.sin(lon0), math.cos(lon0), 0.0],
                  [-math.sin(lat0) * math.cos(lon0), -math.sin(lat0) * math.sin(lon0), math.cos(lat0)],
                  [math.cos(lat0) * math.cos(lon0), math.cos(lat0) * math.sin(lon0), math.sin(lat0)]])
    e0 = ecef(*origin)
    return np.array([R @ (ecef(*row) - e0) for row in lla])


@dataclass
class CameraCalib:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    dist: list = field(default_factory=list)
    distortion_model: str = "radtan"
    T_cam_imu: np.ndarray = field(default_factory=lambda: np.eye(4))   # Kalibr: p_cam = T_cam_imu * p_imu
    timeshift_cam_imu_s: float = 0.0                                    # Kalibr: t_imu = t_cam + shift
    source: str = "default"

    @classmethod
    def from_json(cls, path):
        p = json.loads(Path(path).read_text(encoding="utf-8"))
        T = np.array(p.get("T_cam_imu", np.eye(4)), dtype=float)
        if "T_cam_imu" not in p:      # the team's assumed nadir mount, IMU = FRD body
            T[:3, :3] = R_CAM_FROM_VIO_BODY
        return cls(p["fx"], p["fy"], p["cx"], p["cy"], int(p["width"]), int(p["height"]),
                   list(p.get("dist", [])), p.get("distortion_model", "radtan"), T,
                   float(p.get("timeshift_cam_imu_s", 0.0)), source=str(path))

    @property
    def K(self):
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    @property
    def R_cam_imu(self):
        return self.T_cam_imu[:3, :3]

    @property
    def R_viobody_imu(self):
        """IMU axes -> the camera-aligned FRD body the VIO frontend assumes.

        Identity when the camera is mounted the way vio_frontend.py expects.
        Rotating IMU and attitude through it lets the unmodified frontend run
        with any calibrated camera mounting."""
        return R_CAM_FROM_VIO_BODY.T @ self.R_cam_imu

    def mount_deviation_deg(self):
        """Angle between the calibrated mounting and the one vio_frontend.py assumes."""
        c = (np.trace(self.R_viobody_imu) - 1.0) / 2.0
        return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))

    def optical_axis_tilt_deg(self):
        """Angle between the optical axis and the body 'down' axis."""
        axis_in_imu = self.R_cam_imu.T @ np.array([0.0, 0.0, 1.0])
        return float(np.degrees(np.arccos(np.clip(axis_in_imu[2], -1.0, 1.0))))


@dataclass
class Recording:
    root: Path
    name: str
    t0_ns: int
    frame_t: np.ndarray
    frame_files: list
    imu_t: np.ndarray
    gyro: np.ndarray
    acc: np.ndarray
    att_t: np.ndarray
    att: np.ndarray                 # roll pitch yaw [rad], autopilot ATTITUDE
    range_t: np.ndarray
    range_m: np.ndarray             # distance, min, max
    local_t: np.ndarray
    local: np.ndarray               # LOCAL_POSITION_NED x y z vx vy vz
    gt: Trajectory
    gt_source: str
    calib: CameraCalib
    meta: dict

    def sec(self, t_ns):
        return (np.asarray(t_ns, dtype=np.int64) - self.t0_ns) * 1e-9

    @property
    def duration(self):
        return float(self.frame_t[-1] - self.frame_t[0]) if len(self.frame_t) > 1 else 0.0


def _load_gt(root, t0_ns, gps_min_fix=3):
    """Ground truth, first source that exists. Returns (Trajectory | None, source)."""
    sec = lambda t: (t - t0_ns) * 1e-9
    # 1) groundtruth.csv: t_ns, north, east, down, qw, qx, qy, qz   (body FRD -> NED)
    t_ns, a = _table(root / "groundtruth.csv", 8)
    if len(t_ns):
        q = np.array([R_to_quat(NED_ENU @ quat_to_R([r[4], r[5], r[6], r[3]]) @ FRD_FLU) for r in a])
        return Trajectory(sec(t_ns), a[:, 0:3] @ NED_ENU.T, q, t0_ns=t0_ns, name="groundtruth"), "groundtruth.csv"
    # 2) EuRoC: t_ns, p xyz, q wxyz, v xyz, ...  (world: gravity-aligned, z up)
    t_ns, a = _table(root / "mav0" / "state_groundtruth_estimate0" / "data.csv", 11)
    if len(t_ns):
        q = np.column_stack([a[:, 4], a[:, 5], a[:, 6], a[:, 3]])
        return (Trajectory(sec(t_ns), a[:, 0:3], q, a[:, 7:10], t0_ns=t0_ns, name="groundtruth"),
                "mav0/state_groundtruth_estimate0")
    # 3) recorder GPS: t_rx_ns, lat, lon, alt, eph, epv, vel, fix, sats
    t_ns, a = _table(root / "mav0" / "gps0" / "data.csv", 9)
    if len(t_ns):
        ok = a[:, 6] >= gps_min_fix
        if ok.sum() >= 2:
            enu = lla_to_enu(a[ok, 0:3], a[ok][0, 0:3])
            q = np.tile([0.0, 0.0, 0.0, 1.0], (len(enu), 1))
            return (Trajectory(sec(t_ns[ok]), enu, q, t0_ns=t0_ns, name="gps", has_orientation=False),
                    "mav0/gps0 (GPS_RAW_INT, fix>=%d)" % gps_min_fix)
    return None, "none"


def _euroc_calib(root):
    """Calibration from a EuRoC cam0/sensor.yaml (tiny hand parser, no YAML dependency)."""
    path = root / "mav0" / "cam0" / "sensor.yaml"
    if not path.exists():
        return None
    text = path.read_text(encoding="utf-8")

    def numbers(key):
        start = text.index(key)
        chunk = text[text.index("[", start) + 1:text.index("]", start)]
        return [float(x) for x in chunk.replace("\n", " ").split(",") if x.strip()]

    T_bs = np.array(numbers("data:")).reshape(4, 4)          # camera -> body(IMU)
    fu, fv, cu, cv = numbers("intrinsics:")
    w, h = numbers("resolution:")
    return CameraCalib(fu, fv, cu, cv, int(w), int(h), numbers("distortion_coefficients:"), "radtan",
                       np.linalg.inv(T_bs), 0.0, source=str(path))


def ground_truth(root):
    """Ground truth of a recording without needing its calibration. Returns (Trajectory | None, source)."""
    root = Path(root)
    cam_rows = [r for r in _rows(root / "mav0" / "cam0" / "data.csv") if len(r) >= 2]
    if not cam_rows:
        raise SystemExit(f"No frames listed in {root}/mav0/cam0/data.csv")
    return _load_gt(root, int(cam_rows[0][0]))


def load(root, camera_params=None, apply_timeshift=True, timeshift=None):
    """timeshift: override the calibration file's timeshift_cam_imu_s (seconds)."""
    root = Path(root)
    meta = {}
    if (root / "meta.json").exists():
        meta = json.loads((root / "meta.json").read_text(encoding="utf-8"))

    if camera_params is None and (root / "camera_params.json").exists():
        camera_params = root / "camera_params.json"
    calib = CameraCalib.from_json(camera_params) if camera_params else _euroc_calib(root)
    if calib is None:
        raise SystemExit(f"No camera calibration: pass --camera-params (see calibration/camera_params.json) for {root}")
    if timeshift is not None:
        calib.timeshift_cam_imu_s = float(timeshift)

    cam_rows = [r for r in _rows(root / "mav0" / "cam0" / "data.csv") if len(r) >= 2]
    if not cam_rows:
        raise SystemExit(f"No frames listed in {root}/mav0/cam0/data.csv")
    shift_ns = int(round(calib.timeshift_cam_imu_s * 1e9)) if apply_timeshift else 0
    frame_ns = np.array([int(r[0]) for r in cam_rows], dtype=np.int64)
    t0_ns = int(frame_ns[0])
    sec = lambda t: (np.asarray(t, dtype=np.int64) - t0_ns) * 1e-9
    frame_files = [root / "mav0" / "cam0" / "data" / r[1] for r in cam_rows]

    imu_ns, imu = _table(root / "mav0" / "imu0" / "data.csv", 7)
    att_ns, att = _table(root / "attitude.csv", 4)
    rng_ns, rng = _table(root / "distance.csv", 4)
    loc_ns, loc = _table(root / "local_position.csv", 7)
    gt, gt_source = _load_gt(root, t0_ns)

    return Recording(
        root=root, name=root.name, t0_ns=t0_ns,
        frame_t=sec(frame_ns + shift_ns), frame_files=frame_files,
        imu_t=sec(imu_ns), gyro=imu[:, 0:3], acc=imu[:, 3:6],
        att_t=sec(att_ns), att=att[:, 0:3],
        range_t=sec(rng_ns), range_m=rng[:, 0:3],
        local_t=sec(loc_ns), local=loc[:, 0:6],
        gt=gt, gt_source=gt_source, calib=calib, meta=meta)
