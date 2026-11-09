#!/usr/bin/env python3
"""Датасет (EuRoC-подібний) -> ROS1 bag для Kalibr. ROS на цій машині НЕ потрібен
(використовується чиста Python-бібліотека rosbags: pip install rosbags).

  python3 to_rosbag.py ../dataset/recordings/calib_cam   --out cam.bag --cam-hz 4
  python3 to_rosbag.py ../dataset/recordings/calib_imucam --out imucam.bag --cam-hz 20

Теми: /cam0/image_raw (mono8), /imu0 (sensor_msgs/Imu, осі FRD автопілота).
Мітки часу - з годинника бортового комп'ютера (IMU вже переведено timesync-ом).
"""
import argparse
import csv
import os

import cv2
import numpy as np

try:
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    Writer = Stores = get_typestore = None


def main():
    if Writer is None:
        print("ПОМИЛКА: модуль 'rosbags' не знайдено.\n"
              "Встановіть залежність командою:\n"
              "    pip install rosbags\n"
              "(ROS на цій машині встановлювати НЕ потрібно).")
        raise SystemExit(1)

    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--out", required=True)
    ap.add_argument("--cam-hz", type=float, default=0.0,
                    help="проріджувати кадри до цієї частоти (0 = усі). Kalibr: ~4 Гц для "
                         "камерного калібрування, ~20 Гц для камера+IMU")
    ap.add_argument("--no-imu", action="store_true")
    ap.add_argument("--from-to", type=float, nargs=2, default=None, metavar=("S0", "S1"),
                    help="взяти лише проміжок запису, секунди від першого кадру")
    args = ap.parse_args()

    ts = get_typestore(Stores.ROS1_NOETIC)
    Image = ts.types["sensor_msgs/msg/Image"]
    Imu = ts.types["sensor_msgs/msg/Imu"]
    Header = ts.types["std_msgs/msg/Header"]
    Time = ts.types["builtin_interfaces/msg/Time"]
    Vec3 = ts.types["geometry_msgs/msg/Vector3"]
    Quat = ts.types["geometry_msgs/msg/Quaternion"]

    def stamp(t_ns, seq):
        return Header(seq=seq, stamp=Time(sec=int(t_ns // 10**9), nanosec=int(t_ns % 10**9)),
                      frame_id="")

    with open(os.path.join(args.run, "mav0", "cam0", "data.csv")) as f:
        r = csv.reader(f)
        next(r)
        frames = [(int(a), b) for a, b in (row for row in r if row)]
    t_first = frames[0][0]
    if args.from_to:
        lo, hi = (t_first + int(s * 1e9) for s in args.from_to)
        frames = [fr for fr in frames if lo <= fr[0] <= hi]
    if args.cam_hz > 0:
        step_ns, nxt, keep = 1e9 / args.cam_hz, 0, []
        for t, name in frames:
            if t >= nxt:
                keep.append((t, name))
                nxt = t + step_ns * 0.9      # 0.9: не перескочити кадр через джиттер
        frames = keep
    t_lo, t_hi = frames[0][0], frames[-1][0]

    imu = []
    if not args.no_imu:
        with open(os.path.join(args.run, "mav0", "imu0", "data.csv")) as f:
            r = csv.reader(f)
            next(r)
            for row in r:
                if row:
                    t = int(row[0])
                    if t_lo - 10**9 <= t <= t_hi + 10**9:
                        imu.append((t, [float(v) for v in row[1:7]]))

    out = args.out if args.out.endswith(".bag") else args.out + ".bag"
    if os.path.exists(out):
        os.remove(out)
    cov = np.zeros(9)
    with Writer(out) as w:
        c_img = w.add_connection("/cam0/image_raw", Image.__msgtype__, typestore=ts)
        c_imu = w.add_connection("/imu0", Imu.__msgtype__, typestore=ts)
        # записуємо в порядку часу
        events = [(t, 0, name) for t, name in frames] + [(t, 1, v) for t, v in imu]
        events.sort(key=lambda e: e[0])
        si = sm = 0
        for t, kind, payload in events:
            if kind == 0:
                img = cv2.imread(os.path.join(args.run, "mav0", "cam0", "data", payload),
                                 cv2.IMREAD_GRAYSCALE)
                msg = Image(header=stamp(t, si), height=img.shape[0], width=img.shape[1],
                            encoding="mono8", is_bigendian=0, step=img.shape[1],
                            data=img.reshape(-1))
                w.write(c_img, t, ts.serialize_ros1(msg, Image.__msgtype__))
                si += 1
            else:
                gx, gy, gz, ax, ay, az = payload
                msg = Imu(header=stamp(t, sm),
                          orientation=Quat(x=0.0, y=0.0, z=0.0, w=1.0),
                          orientation_covariance=np.array([-1.0] + [0.0] * 8),
                          angular_velocity=Vec3(x=gx, y=gy, z=gz),
                          angular_velocity_covariance=cov,
                          linear_acceleration=Vec3(x=ax, y=ay, z=az),
                          linear_acceleration_covariance=cov)
                w.write(c_imu, t, ts.serialize_ros1(msg, Imu.__msgtype__))
                sm += 1
    span = (t_hi - t_lo) * 1e-9
    print(f"{out}: {si} кадрів, {sm} IMU, {span:.1f} с"
          + (f", IMU {sm / span:.0f} Гц" if sm and span > 0 else ""))
    if sm and span > 0 and sm / span < 150:
        print("УВАГА: IMU < 150 Гц. Kalibr дає гірший результат; див. calibration/README.md")


if __name__ == "__main__":
    main()
