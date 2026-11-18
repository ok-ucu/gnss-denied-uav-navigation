#!/usr/bin/env python3
"""Перевірка узгодження осей координат та часових міток (Розділ 4 вимог рев'ю).

Вимога рев'ю:
Узгодити напрямки осей із Людиною 3 (FRD vs Camera optical frame),
переконатися, що timestamps камери й IMU придатні для синхронізації.

Математичний контракт з Людиною 3 (vio/vio_frontend.py):
- Корпус дрона: система FRD (Forward = +X, Right = +Y, Down = +Z)
- Спадна камера: оптична вісь Z спрямована вниз, X — праворуч (+Y дрона), Y — назад (-X дрона)
- Матриця орієнтації R_cam_body:
    [ 0,  1,  0 ]
    [-1,  0,  0 ]
    [ 0,  0,  1 ]
- Перерахунок швидкості:
    flow_rot = fx * [gx, gy]
    v_cam = -(height / fx) * (flow - flow_rot)
    v_body = [-v_cam_y, v_cam_x, 0]

Використання:
    python3 verify_sync_axes.py --dataset ../recordings/sample_flight
"""
import argparse
from bisect import bisect_left
import csv
from decimal import Decimal, InvalidOperation
from pathlib import Path
import numpy as np

def test_axis_transformation():
    """Тест математичного перетворення оптичного потоку у швидкість дрона."""
    fx = 461.629
    height = 10.0
    # Дрон летить уперед зі швидкістю 2.0 м/с (Vx = +2.0 у системі FRD)
    true_v_body = np.array([2.0, 0.0, 0.0])

    # Forward body motion yields positive image flow along camera Y.
    flow = np.array([0.0, (true_v_body[0] * fx) / height])  # пікселі рухаються "вниз" по кадру
    gyro = np.array([0.0, 0.0, 0.0])

    # Алгоритм Людини 3 (vio_frontend.py):
    flow_rot = fx * np.array([gyro[0], gyro[1]])
    v_cam = -(height / fx) * (flow - flow_rot)
    v_body_est = np.array([-v_cam[1], v_cam[0], 0.0])

    err = np.linalg.norm(true_v_body - v_body_est)
    return err < 1e-6, true_v_body, v_body_est


def read_timestamps(path):
    """Read strictly increasing integer nanoseconds from an EuRoC sensor CSV."""
    with path.open(newline="", encoding="utf-8-sig") as stream:
        rows = csv.reader(stream, strict=True)
        header = next(rows, [])
        if not header or header[0].strip() != "#timestamp [ns]":
            raise ValueError(f"{path}: expected #timestamp [ns] as the first column")
        stamps = []
        for row in rows:
            if len(row) != len(header) or not row[0].strip().isascii() or not row[0].strip().isdecimal():
                raise ValueError(f"{path}:{rows.line_num}: invalid timestamp row")
            stamp = int(row[0])
            if stamps and stamp <= stamps[-1]:
                raise ValueError(f"{path}:{rows.line_num}: timestamps must increase")
            stamps.append(stamp)
        if len(stamps) < 2:
            raise ValueError(f"{path}: at least two timestamps required")
        return stamps


def maximum_nearest_gap_ns(dataset):
    sensors = dataset / "mav0"
    camera = read_timestamps(sensors / "cam0" / "data.csv")
    imu = read_timestamps(sensors / "imu0" / "data.csv")
    if camera[0] < imu[0] or camera[-1] > imu[-1]:
        raise ValueError("IMU timestamps must cover the full camera interval")
    maximum = 0
    for stamp in camera:
        i = bisect_left(imu, stamp)
        gap = imu[i] - stamp
        if i:
            gap = min(gap, stamp - imu[i - 1])
        maximum = max(maximum, gap)
    return maximum


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, epilog=(
        "Reads mav0/{cam0,imu0}/data.csv with integer nanosecond timestamps. "
        "The dataset must declare a shared clock. Nearest-sample distance "
        "does not establish clock offset or sensor latency."))
    parser.add_argument("--dataset", type=Path, default=(
        Path(__file__).resolve().parents[1] / "recordings" / "sample_flight"))
    parser.add_argument("--max-gap-ms", type=Decimal, default=Decimal("5"))
    args = parser.parse_args(argv)
    try:
        if not args.max_gap_ms.is_finite() or args.max_gap_ms <= 0:
            raise ValueError("--max-gap-ms must be finite and positive")
        axis_ok, v_true, v_est = test_axis_transformation()
        print(f"Offline axis calculation {'PASS' if axis_ok else 'FAIL'}: {v_true} -> {v_est}")
        gap_ms = Decimal(maximum_nearest_gap_ns(args.dataset)) / 1_000_000
        time_ok = gap_ms <= args.max_gap_ms
        print(f"Dataset {args.dataset}: timestamp alignment {'PASS' if time_ok else 'FAIL'}, "
              f"maximum nearest-IMU gap {gap_ms} ms (limit {args.max_gap_ms} ms)")
        return 0 if axis_ok and time_ok else 1
    except (OSError, ValueError, csv.Error, InvalidOperation) as exc:
        print(f"Timestamp alignment FAIL: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
