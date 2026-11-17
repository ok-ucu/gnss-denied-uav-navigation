#!/usr/bin/env python3
"""Генератор демонстраційного зразка датасету для тестування без заліза.

Створює реалістичний датасет польоту дрона:
- рух на висоті 10 м зі швидкістю 2 м/с на північ;
- камера дивиться вниз на текстуровану поверхню (30 fps);
- IMU 100 Гц у системі FRD;
- GPS 5 Гц (ground truth);
- формат EuRoC-подібний, готовий для dataset_reader.py, vio_ekf.py та оцінювання.

Використання:
    python3 generate_sample_dataset.py --out ../recordings/sample_flight --seconds 5.0
"""
import argparse
import csv
import json
import os
import time
from datetime import datetime

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.80665


def generate(out_dir, seconds=5.0, fps=30, imu_hz=100, gps_hz=5):
    os.makedirs(os.path.join(out_dir, "mav0", "cam0", "data"), exist_ok=True)
    os.makedirs(os.path.join(out_dir, "mav0", "imu0"), exist_ok=True)
    os.makedirs(os.path.join(out_dir, "mav0", "gps0"), exist_ok=True)

    rng = np.random.default_rng(42)
    # Текстура землі
    tex_w, tex_h = 1600, 1600
    base = rng.integers(30, 220, (tex_h, tex_w), dtype=np.uint8)
    for _ in range(120):
        cx, cy = rng.integers(50, tex_w - 50), rng.integers(50, tex_h - 50)
        rad = rng.integers(15, 60)
        col = int(rng.integers(0, 255))
        cv2.circle(base, (cx, cy), rad, col, -1)
    texture = cv2.GaussianBlur(base, (0, 0), 2.0)

    t0_ns = 1_700_000_000_000_000_000  # опорний час бортового комп'ютера (нс)
    w, h = 640, 480
    fx = 461.629
    height = 10.0
    vx = 2.0  # швидкість польоту вперед (м/с)

    # 1. Кадри камери
    dt_cam = 1.0 / fps
    n_frames = int(seconds * fps)
    cam_rows = []
    for i in range(n_frames):
        t_s = i * dt_cam
        t_ns = t0_ns + int(t_s * 1e9)
        # Зсув у текстурі по осі Y кадру (рух дрона вперед => текстура рухається назад/вниз по кадру)
        shift_y = int((vx * t_s * fx) / height)
        y0 = 200 + (shift_y % (tex_h - h - 400))
        x0 = 200
        frame = texture[y0:y0 + h, x0:x0 + w]
        fname = f"{t_ns}.png"
        cv2.imwrite(os.path.join(out_dir, "mav0", "cam0", "data", fname), frame)
        cam_rows.append([t_ns, fname])

    with open(os.path.join(out_dir, "mav0", "cam0", "data.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["#timestamp [ns]", "filename"])
        wr.writerows(cam_rows)

    # 2. IMU (100 Гц)
    dt_imu = 1.0 / imu_hz
    n_imu = int(seconds * imu_hz)
    imu_rows = []
    for i in range(n_imu):
        t_s = i * dt_imu
        t_ns = t0_ns + int(t_s * 1e9)
        # Малий шум гіроскопа та акселерометра
        gx = float(rng.normal(0, 0.002))
        gy = float(rng.normal(0, 0.002))
        gz = float(rng.normal(0, 0.001))
        ax = float(rng.normal(0, 0.05))
        ay = float(rng.normal(0, 0.05))
        az = float(-G + rng.normal(0, 0.05))  # гравітація по осі Z (вниз)
        imu_rows.append([t_ns, f"{gx:.6f}", f"{gy:.6f}", f"{gz:.6f}",
                         f"{ax:.6f}", f"{ay:.6f}", f"{az:.6f}"])

    with open(os.path.join(out_dir, "mav0", "imu0", "data.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["#timestamp [ns]", "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]",
                     "w_RS_S_z [rad s^-1]", "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]",
                     "a_RS_S_z [m s^-2]"])
        wr.writerows(imu_rows)

    # 3. GPS (5 Гц)
    dt_gps = 1.0 / gps_hz
    n_gps = int(seconds * gps_hz)
    gps_rows = []
    base_lat, base_lon, base_alt = 50.450100, 30.523400, 150.0
    m_per_deg_lat = 111132.0
    for i in range(n_gps):
        t_s = i * dt_gps
        t_ns = t0_ns + int(t_s * 1e9)
        north = vx * t_s
        dlat = north / m_per_deg_lat
        lat = base_lat + dlat
        lon = base_lon
        alt = base_alt
        gps_rows.append([t_ns, f"{lat:.7f}", f"{lon:.7f}", f"{alt:.2f}",
                         1.0, 1.5, f"{vx:.2f}", 3, 14])

    with open(os.path.join(out_dir, "mav0", "gps0", "data.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["#t_rx_ns", "lat_deg", "lon_deg", "alt_m", "eph", "epv",
                     "vel_ms", "fix_type", "satellites"])
        wr.writerows(gps_rows)

    # 4. Attitude (30 Гц)
    att_rows = []
    for i in range(n_frames):
        t_s = i * dt_cam
        t_ns = t0_ns + int(t_s * 1e9)
        att_rows.append([t_ns, "0.0", "0.0", "0.0", "0.0", "0.0", "0.0"])

    with open(os.path.join(out_dir, "attitude.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t_ns", "roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"])
        wr.writerows(att_rows)

    # 5. Local position NED (10 Гц)
    loc_rows = []
    for i in range(int(seconds * 10)):
        t_s = i * 0.1
        t_ns = t0_ns + int(t_s * 1e9)
        x = vx * t_s
        loc_rows.append([t_ns, f"{x:.3f}", "0.0", f"{-height:.3f}", f"{vx:.2f}", "0.0", "0.0"])

    with open(os.path.join(out_dir, "local_position.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t_rx_ns", "x", "y", "z", "vx", "vy", "vz"])
        wr.writerows(loc_rows)

    # 6. Events log
    with open(os.path.join(out_dir, "events.log"), "w", encoding="utf-8") as f:
        f.write(f"{t0_ns} EKF3 IMU0 is using external nav data\n")
        f.write(f"{t0_ns + 500_000_000} Arming motors\n")

    # 7. Metadata
    meta = {
        "format": "euroc-like, див. dataset/README.md",
        "start_wall": datetime.now().astimezone().isoformat(),
        "duration_s": seconds,
        "note": "Синтетичний тестовий польот: прямий рух на 10 м висоті зі швидкістю 2 м/с",
        "clock": f"CLOCK_MONOTONIC (time.monotonic_ns), нс; час старту запису: {t0_ns}",
        "host": "companion-computer",
        "machine": "arm64",
        "camera_config": {"backend": "synthetic", "width": w, "height": h, "fps": fps},
        "frames_written": n_frames,
        "frames_dropped": 0,
        "frame_shape": [h, w],
        "timesync": {
            "imu": {"n": n_imu, "drift_ppm": 0.0, "latency_median_ms": 1.2, "latency_p95_ms": 2.5, "rate_hz": float(imu_hz)},
            "attitude": {"n": n_frames, "drift_ppm": 0.0, "latency_median_ms": 1.5, "latency_p95_ms": 3.0}
        }
    }
    with open(os.path.join(out_dir, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    timesync_data = {
        "imu": {
            "model": {"a_ns": 0.0, "b_ns_per_s": 0.0, "t0_ap_ns": float(t0_ns)},
            "stats": {"n": n_imu, "drift_ppm": 0.0, "latency_median_ms": 1.2, "latency_p95_ms": 2.5, "rate_hz": float(imu_hz)}
        }
    }
    with open(os.path.join(out_dir, "timesync.json"), "w", encoding="utf-8") as f:
        json.dump(timesync_data, f, indent=2)

    print(f"Згенеровано датасет у {out_dir}: {n_frames} кадрів, {n_imu} IMU, {n_gps} GPS.")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=os.path.join(HERE, "..", "recordings", "sample_flight"))
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--imu-rate", type=int, default=100)
    ap.add_argument("--gps-rate", type=int, default=5)
    args = ap.parse_args()
    generate(args.out, args.seconds, args.fps, args.imu_rate, args.gps_rate)


if __name__ == "__main__":
    main()
