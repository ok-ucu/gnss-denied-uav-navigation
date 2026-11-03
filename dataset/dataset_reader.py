#!/usr/bin/env python3
"""Читання записаного датасету для replay та оцінювання.

    from dataset_reader import Dataset
    ds = Dataset("recordings/run_X", camera_params="../calibration/camera_params.json")
    for t, kind, data in ds.events():        # t - секунди від початку запису
        if kind == "imu":    gyro, acc = data[:3], data[3:]
        elif kind == "att":  roll, pitch, yaw = data[:3]
        elif kind == "frame": gray = data     # uint8, за потреби вже без дисторсії

Для vio_ekf.py (replay): беріть t із events() замість time.time().
Командний рядок: python3 dataset_reader.py RUN   - зведення й перевірки якості.
"""
import argparse
import csv
import heapq
import json
import math
import os

import cv2
import numpy as np


def meters_per_degree(lat_deg):
    """Та сама формула, що у vio/vio_ekf.py."""
    phi = math.radians(lat_deg)
    m_lat = 111132.954 - 559.822 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)
    m_lon = 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi) + 0.118 * math.cos(5 * phi)
    return m_lat, m_lon


def _csv(path, skip_header=True):
    if not os.path.exists(path):
        return None
    with open(path, newline="") as f:
        r = csv.reader(f)
        if skip_header:
            next(r, None)
        return [row for row in r if row]


class Dataset:
    def __init__(self, root, camera_params=None, apply_timeshift=True, undistort=False):
        self.root = root
        with open(os.path.join(root, "meta.json"), encoding="utf-8") as f:
            self.meta = json.load(f)
        self.params = None
        if camera_params:
            with open(camera_params, encoding="utf-8") as f:
                self.params = json.load(f)
        shift_ns = 0
        if self.params and apply_timeshift:
            # Kalibr: t_imu = t_cam + timeshift
            shift_ns = int(round(self.params.get("timeshift_cam_imu_s", 0.0) * 1e9))

        rows = _csv(os.path.join(root, "mav0", "cam0", "data.csv")) or []
        self._frame_t = np.array([int(r[0]) + shift_ns for r in rows], dtype=np.int64)
        self._frame_f = [os.path.join(root, "mav0", "cam0", "data", r[1]) for r in rows]

        rows = _csv(os.path.join(root, "mav0", "imu0", "data.csv")) or []
        a = np.array(rows, dtype=np.float64).reshape(-1, 7)
        self._imu_t = a[:, 0].astype(np.int64) if len(a) else np.zeros(0, np.int64)
        self._imu = a[:, 1:7]                      # gx gy gz ax ay az

        rows = _csv(os.path.join(root, "attitude.csv")) or []
        a = np.array(rows, dtype=np.float64).reshape(-1, 7)
        self._att_t = a[:, 0].astype(np.int64) if len(a) else np.zeros(0, np.int64)
        self._att = a[:, 1:7]

        rows = _csv(os.path.join(root, "mav0", "gps0", "data.csv")) or []
        a = np.array(rows, dtype=np.float64).reshape(-1, 9)
        self._gps_t = a[:, 0].astype(np.int64) if len(a) else np.zeros(0, np.int64)
        self._gps = a[:, 1:9]                      # lat lon alt eph epv vel fix sats

        rows = _csv(os.path.join(root, "local_position.csv")) or []
        a = np.array(rows, dtype=np.float64).reshape(-1, 7)
        self._loc_t = a[:, 0].astype(np.int64) if len(a) else np.zeros(0, np.int64)
        self._loc = a[:, 1:7]

        firsts = [t[0] for t in (self._frame_t, self._imu_t, self._att_t) if len(t)]
        self.t0_ns = int(min(firsts)) if firsts else 0
        self._undist = self._make_undistorter() if (undistort and self.params) else None

    # ---------- час ----------
    def sec(self, t_ns):
        return (np.asarray(t_ns) - self.t0_ns) * 1e-9

    # ---------- масиви (секунди від початку) ----------
    def imu(self):
        return np.column_stack([self.sec(self._imu_t), self._imu])

    def attitude(self):
        return np.column_stack([self.sec(self._att_t), self._att[:, :3]])

    def frame_times(self):
        return self.sec(self._frame_t)

    def gps_local_ned(self, min_fix=3):
        """Трек GPS у метрах (north, east, down) від першої придатної точки.
        Повертає масив [t, n, e, d]; це ground truth для оцінювання."""
        ok = self._gps[:, 6] >= min_fix
        if not ok.any():
            return np.zeros((0, 4))
        g, t = self._gps[ok], self._gps_t[ok]
        m_lat, m_lon = meters_per_degree(g[0, 0])
        n = (g[:, 0] - g[0, 0]) * m_lat
        e = (g[:, 1] - g[0, 1]) * m_lon
        d = -(g[:, 2] - g[0, 2])
        return np.column_stack([self.sec(t), n, e, d])

    def export_tum_trajectory(self, path, source="gps"):
        """Експорт траєкторії у форматі TUM: timestamp tx ty tz qx qy qz qw.
        Придатний для інструменту evo (Людина 4: оцінювання RMSE, ATE, RPE)."""
        lines = ["# timestamp tx ty tz qx qy qz qw\n"]
        if source == "gps":
            ned = self.gps_local_ned()
            for row in ned:
                t, n, e, d = row
                lines.append(f"{t:.6f} {n:.4f} {e:.4f} {-d:.4f} 0.0 0.0 0.0 1.0\n")
        elif source == "local":
            for t_ns, row in zip(self._loc_t, self._loc):
                t = float(self.sec(t_ns))
                x, y, z = row[:3]
                lines.append(f"{t:.6f} {x:.4f} {y:.4f} {-z:.4f} 0.0 0.0 0.0 1.0\n")
        parent = os.path.dirname(os.path.abspath(path))
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.writelines(lines)

    # ---------- кадри ----------
    def _make_undistorter(self):
        p = self.params
        K = np.array([[p["fx"], 0, p["cx"]], [0, p["fy"], p["cy"]], [0, 0, 1.0]])
        D = np.array(p["dist"], dtype=np.float64)
        size = (int(p["width"]), int(p["height"]))
        if p.get("camera_model", "pinhole") == "pinhole" and p.get("distortion_model") in ("radtan", "plumb_bob"):
            m1, m2 = cv2.initUndistortRectifyMap(K, D, None, K, size, cv2.CV_16SC2)
        elif p.get("distortion_model") == "equidistant":
            m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, size, cv2.CV_16SC2)
        else:
            raise ValueError(f"модель дисторсії не підтримана: {p.get('distortion_model')}")
        return lambda img: cv2.remap(img, m1, m2, cv2.INTER_LINEAR)

    def image(self, i):
        img = cv2.imread(self._frame_f[i], cv2.IMREAD_GRAYSCALE)
        return self._undist(img) if self._undist else img

    # ---------- потік подій для replay ----------
    def events(self, start_s=0.0, end_s=None, kinds=("imu", "att", "frame", "local")):
        """Усі вибрані події, відсортовані за часом: (t_с, тип, дані).
        imu: [gx gy gz ax ay az]; att: [roll pitch yaw rr pr yr];
        frame: gray; local: [x y z vx vy vz] (LOCAL_POSITION_NED автопілота)."""
        def gen(kind, ts, get):
            for i, t in enumerate(ts):
                yield (float(self.sec(t)), kind, i, get)
        streams = []
        if "imu" in kinds:
            streams.append(gen("imu", self._imu_t, lambda i: self._imu[i]))
        if "att" in kinds:
            streams.append(gen("att", self._att_t, lambda i: self._att[i]))
        if "frame" in kinds:
            streams.append(gen("frame", self._frame_t, self.image))
        if "local" in kinds:
            streams.append(gen("local", self._loc_t, lambda i: self._loc[i]))
        for t, kind, i, get in heapq.merge(*streams, key=lambda e: e[0]):
            if t < start_s:
                continue
            if end_s is not None and t > end_s:
                break
            yield t, kind, get(i)

    # ---------- перевірки якості ----------
    def summary(self):
        lines = []

        def rate(name, t_ns, expect=None):
            if len(t_ns) < 2:
                lines.append(f"{name}: немає даних")
                return
            dt = np.diff(t_ns) * 1e-9
            dur = (t_ns[-1] - t_ns[0]) * 1e-9
            s = f"{name}: {len(t_ns)} шт, {len(t_ns) / dur:.1f} Гц, пропуск макс {dt.max() * 1e3:.0f} мс"
            if dt.min() <= 0:
                s += "  !!! час не зростає"
            if expect and (dt > 3.0 / expect).any():
                s += f"  !!! {(dt > 3.0 / expect).sum()} проміжків > 3 періодів"
            lines.append(s)

        rate("кадри", self._frame_t, self.meta.get("camera_config", {}) and
             (self.meta["camera_config"] or {}).get("fps"))
        rate("IMU", self._imu_t, 50)
        rate("атитюд", self._att_t)
        rate("GPS", self._gps_t)
        if len(self._frame_t) and len(self._imu_t):
            lo = max(self._frame_t[0], self._imu_t[0])
            hi = min(self._frame_t[-1], self._imu_t[-1])
            lines.append(f"перетин кадрів та IMU у часі: {(hi - lo) * 1e-9:.1f} с")
        if self.meta.get("frames_dropped"):
            lines.append(f"!!! втрачено кадрів при записі: {self.meta['frames_dropped']}")
        if len(self._gps_t):
            fix = self._gps[:, 6]
            lines.append(f"GPS fix>=3: {(fix >= 3).mean() * 100:.0f}% точок, супутників "
                         f"{self._gps[:, 7].min():.0f}-{self._gps[:, 7].max():.0f}")
        ts = self.meta.get("timesync", {}).get("imu")
        if ts:
            lines.append(f"синхронізація IMU: дрейф {ts['drift_ppm']:.1f} ppm, "
                         f"затримка лінії p95 {ts['latency_p95_ms']:.1f} мс")
        return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    args = ap.parse_args()
    print(Dataset(args.run).summary())


if __name__ == "__main__":
    main()
