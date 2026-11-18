#!/usr/bin/env python3
"""Convert one INSANE flight (University of Klagenfurt) into the team's recording format.

INSANE is an open, real UAV dataset with a DOWNWARD navigation camera, a PX4
IMU, a laser range finder and RTK-GNSS ground truth, which is the sensor set
the team's VIO expects. Source: https://sst.aau.at/cns/datasets/insane-dataset/
(BSD-2-Clause, non-commercial; cite Brommer et al., IJRR 2024).

Download (mars_4 = 173 s, 155 m traverse, up to 25 m above ground, 1.75 GB):
    BASE=https://cns-data.aau.at/insane-dataset
    curl -LO $BASE/mars_4_nav_cam.zip -LO $BASE/mars_4_sensors.zip -LO $BASE/insane_sensor_calib_preprocessed.zip
    unzip each archive into one folder RAW/

    python3 convert_insane.py RAW --flight mars_4 --calib mars --out OUT [--scale 0.5]

Output (see dataset/README.md for the format):
    OUT/mav0/cam0/data.csv + data/<t_ns>.png   frames, downscaled by --scale
    OUT/mav0/imu0/data.csv                     PX4 IMU converted FLU -> FRD
    OUT/distance.csv                           laser range finder
    OUT/groundtruth.csv                        t_ns, north, east, down, qw, qx, qy, qz (FRD -> NED)
    OUT/camera_params.json                     intrinsics (scaled), T_cam_imu for the FRD IMU
    OUT/meta.json                              provenance and counts
There is no attitude.csv: INSANE has no autopilot attitude log (replay_vio.py
then uses its own AHRS, or the ground-truth attitude when asked).
"""
import argparse
import csv
import json
import re
from pathlib import Path

import cv2
import numpy as np

from rotations import FRD_FLU, NED_ENU, R_to_quat, quat_to_R

LRF_MIN_M, LRF_MAX_M = 0.15, 40.0      # not in the dataset docs; widest range seen is 25.8 m


def read_numeric(path, ncols):
    rows = []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if len(row) >= ncols and re.match(r"^\s*-?\d", row[0]):
                rows.append([float(x) for x in row[:ncols]])
    return np.array(rows)


def kalibr_block(text, name):
    """Parse one camera block of INSANE's camera_calibration_*.yaml (Kalibr-style)."""
    block = text[text.index(name + ":"):]
    numbers = lambda key: [float(x) for x in re.search(key + r":\s*\[([^\]]*)\]", block, re.S).group(1).split(",")]
    return dict(intrinsics=numbers("intrinsics"), dist=numbers("distortion_coeffs"),
                resolution=[int(x) for x in numbers("resolution")],
                model=re.search(r"distortion_model:\s*(\w+)", block).group(1))


def kalibr_matrix(text, name):
    block = text[text.index(name + ":"):]
    rows = re.findall(r"-\s*\[([^\]]*)\]", block)[:4]
    return np.array([[float(x) for x in r.split(",")] for r in rows])


def to_ns(t):
    return np.round(np.asarray(t) * 1e9).astype(np.int64)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("raw", type=Path, help="folder holding <flight>_nav_cam/, <flight>_sensors/, insane_sensor_calib_preprocessed/")
    ap.add_argument("--flight", default="mars_4")
    ap.add_argument("--calib", default="mars", choices=["mars", "klu1", "klu2"])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--scale", type=float, default=0.5, help="image downscale factor (camera is 2056x1542)")
    args = ap.parse_args()

    cam_dir = args.raw / f"{args.flight}_nav_cam"
    sens = args.raw / f"{args.flight}_sensors"
    calib_text = (args.raw / "insane_sensor_calib_preprocessed" / f"camera_calibration_{args.calib}.yaml").read_text()
    if args.out.exists():
        raise SystemExit(f"{args.out} already exists")
    (args.out / "mav0" / "cam0" / "data").mkdir(parents=True)
    (args.out / "mav0" / "imu0").mkdir(parents=True)

    # ---- IMU: INSANE gives specific force / rates in a z-up body (FLU); the team format is FRD ----
    imu = read_numeric(sens / "px4_imu.csv", 7)                 # t, a xyz, w xyz
    acc_frd, gyro_frd = imu[:, 1:4] @ FRD_FLU.T, imu[:, 4:7] @ FRD_FLU.T
    with open(args.out / "mav0" / "imu0" / "data.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["#timestamp [ns]", "gx [rad/s]", "gy", "gz", "ax [m/s^2]", "ay", "az"])
        for t, g, a in zip(to_ns(imu[:, 0]), gyro_frd, acc_frd):
            w.writerow([t, *(f"{x:.9g}" for x in g), *(f"{x:.9g}" for x in a)])

    # ---- laser range finder ----
    lrf = read_numeric(sens / "lrf_range.csv", 2)
    with open(args.out / "distance.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_rx_ns", "distance_m", "min_m", "max_m"])
        for t, d in zip(to_ns(lrf[:, 0]), lrf[:, 1]):
            w.writerow([t, f"{d:.4f}", LRF_MIN_M, LRF_MAX_M])

    # ---- ground truth: ENU position, quaternion body(FLU) -> ENU, already on the PX4 IMU clock ----
    gt = read_numeric(sens / "ground_truth" / "ground_truth_80hz.csv", 8)   # t, p xyz, q wxyz
    p_ned = (gt[:, 1:4] - gt[0, 1:4]) @ NED_ENU.T
    q_ned_frd = np.array([R_to_quat(NED_ENU @ quat_to_R([r[5], r[6], r[7], r[4]]) @ FRD_FLU) for r in gt])
    with open(args.out / "groundtruth.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ns", "north_m", "east_m", "down_m", "qw", "qx", "qy", "qz"])
        for t, p, q in zip(to_ns(gt[:, 0]), p_ned, q_ned_frd):
            w.writerow([t, *(f"{x:.5f}" for x in p), f"{q[3]:.7f}", f"{q[0]:.7f}", f"{q[1]:.7f}", f"{q[2]:.7f}"])

    # Convention check: rotating the measured specific force into ENU with the ground-truth attitude
    # must give roughly +g on "up". Only the last third of the flight is used: in mars_4 the
    # ground-truth ATTITUDE is not converged for about the first 75 s (it shows a 58 deg tilt while
    # the drone stands on the ground). The ground-truth POSITION (RTK) is not affected.
    tail = gt[2 * len(gt) // 3::50]
    k = np.searchsorted(imu[:, 0], tail[:, 0]).clip(0, len(imu) - 1)
    f_enu = np.array([quat_to_R([r[5], r[6], r[7], r[4]]) @ imu[i, 1:4] for r, i in zip(tail, k)])
    gravity_check = np.mean(f_enu, axis=0)

    # ---- camera: radtan model (the 'fov' model is not supported by OpenCV/OpenVINS) ----
    cam = kalibr_block(calib_text, "nav_cam_radtan")
    T_cam_flu = kalibr_matrix(calib_text, "T_cam_px4imu")
    T_cam_frd = T_cam_flu.copy()
    T_cam_frd[:3, :3] = T_cam_flu[:3, :3] @ FRD_FLU            # p_cam = R * (FRD_FLU * p_frd) + t
    s = args.scale
    fx, fy, cx, cy = cam["intrinsics"]
    width, height = (int(round(v * s)) for v in cam["resolution"])
    params = dict(
        camera_model="pinhole", distortion_model=cam["model"],
        fx=fx * s, fy=fy * s, cx=(cx + 0.5) * s - 0.5, cy=(cy + 0.5) * s - 0.5,
        dist=cam["dist"], width=width, height=height,
        T_cam_imu=T_cam_frd.tolist(), timeshift_cam_imu_s=0.0,
        note="INSANE nav_cam_radtan scaled by %.3g; IMU axes converted FLU->FRD; the dataset gives no "
             "camera-IMU time offset (0 assumed, see calib_check.py)" % s)
    (args.out / "camera_params.json").write_text(json.dumps(params, indent=2) + "\n")

    # ---- frames ----
    stamps = []
    with open(cam_dir / "nav_cam_timestamps.csv", newline="") as f:
        for row in csv.reader(f):
            if len(row) >= 3 and re.match(r"^\s*\d", row[0]):
                stamps.append((float(row[1]), row[2].strip()))
    written = missing = 0
    with open(args.out / "mav0" / "cam0" / "data.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["#timestamp [ns]", "filename"])
        for t, name in stamps:
            src = cam_dir / "img" / f"{name}.png"
            img = cv2.imread(str(src), cv2.IMREAD_GRAYSCALE) if src.exists() else None
            if img is None:
                missing += 1
                continue
            if s != 1.0:
                img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
            t_ns = int(round(t * 1e9))
            cv2.imwrite(str(args.out / "mav0" / "cam0" / "data" / f"{t_ns}.png"), img, [cv2.IMWRITE_PNG_COMPRESSION, 3])
            w.writerow([t_ns, f"{t_ns}.png"])
            written += 1

    meta = dict(
        format="euroc-like, see dataset/README.md",
        source="INSANE dataset, flight %s (https://sst.aau.at/cns/datasets/insane-dataset/)" % args.flight,
        license="BSD-2-Clause, non-commercial (see the dataset LICENSE)",
        clock="UNIX time of the recording computer, ns; ground truth already shifted onto the PX4 IMU clock by the authors",
        imu="px4_imu.csv, FLU -> FRD", imu_rows=int(len(imu)),
        range_sensor="lrf_range.csv, assumed to point along body down", range_rows=int(len(lrf)),
        groundtruth="ground_truth_80hz.csv (RTK GNSS + IMU fusion by the dataset authors), NED from the first sample",
        groundtruth_rows=int(len(gt)),
        frames_listed=len(stamps), frames_written=written, frames_missing_in_archive=missing,
        frame_shape=[height, width], image_scale=s,
        gravity_check_enu_last_third=[round(float(x), 3) for x in gravity_check],
        groundtruth_attitude_warning="attitude part of the ground truth is unreliable early in the flight; "
                                     "use the position only",
        camera_config=dict(fps=round(written / (stamps[-1][0] - stamps[0][0]), 2), width=width, height=height))
    (args.out / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(json.dumps(meta, indent=2))


if __name__ == "__main__":
    main()
