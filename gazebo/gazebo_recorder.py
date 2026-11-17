#!/usr/bin/env python3
"""Записувач датасету з Gazebo Harmonic (ROS 2 топіки -> EuRoC-формат).

Підписується на:
- /camera/image_raw (sensor_msgs/msg/Image)
- /imu (sensor_msgs/msg/Imu)
Зберігає у форматі, ідентичному реальному recorder.py:
mav0/cam0/data/<timestamp_ns>.png
mav0/cam0/data.csv
mav0/imu0/data.csv
meta.json

Використання всередині контейнера Gazebo або хоста з ROS 2:
    python3 gazebo_recorder.py --out ../recordings/gazebo_run1 --seconds 20
"""
import argparse
import csv
import json
import os
import time
from datetime import datetime

import cv2
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSReliabilityPolicy, QoSProfile
    from sensor_msgs.msg import Image, Imu
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


if HAS_ROS2:
    class GazeboRecorderNode(Node):
        def __init__(self, out_dir, target_seconds=0.0):
            super().__init__("gazebo_recorder")
            self.out_dir = out_dir
            self.target_seconds = target_seconds
            self.t_start = time.monotonic()

            self.cam_dir = os.path.join(out_dir, "mav0", "cam0", "data")
            self.imu_dir = os.path.join(out_dir, "mav0", "imu0")
            os.makedirs(self.cam_dir, exist_ok=True)
            os.makedirs(self.imu_dir, exist_ok=True)

            self.cam_csv = open(os.path.join(out_dir, "mav0", "cam0", "data.csv"), "w", newline="")
            self.cam_writer = csv.writer(self.cam_csv)
            self.cam_writer.writerow(["#timestamp [ns]", "filename"])

            self.imu_csv = open(os.path.join(out_dir, "mav0", "imu0", "data.csv"), "w", newline="")
            self.imu_writer = csv.writer(self.imu_csv)
            self.imu_writer.writerow(["#timestamp [ns]", "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]",
                                      "w_RS_S_z [rad s^-1]", "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]",
                                      "a_RS_S_z [m s^-2]"])

            qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.sub_img = self.create_subscription(Image, "/camera/image_raw", self.image_cb, qos)
            self.sub_imu = self.create_subscription(Imu, "/imu", self.imu_cb, qos)

            self.frames_count = 0
            self.imu_count = 0
            self.get_logger().info(f"Запис Gazebo у {out_dir} розпочато...")

        def stamp_to_ns(self, header):
            return int(header.stamp.sec * 10**9 + header.stamp.nanosec)

        def image_cb(self, msg: Image):
            t_ns = self.stamp_to_ns(msg.header)
            # Конвертація Image raw байти в OpenCV gray
            if msg.encoding in ("rgb8", "bgr8"):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY if msg.encoding == "bgr8" else cv2.COLOR_RGB2GRAY)
            elif msg.encoding == "mono8":
                gray = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            else:
                gray = np.frombuffer(msg.data, dtype=np.uint8)[:msg.height * msg.width].reshape((msg.height, msg.width))

            fname = f"{t_ns}.png"
            cv2.imwrite(os.path.join(self.cam_dir, fname), gray)
            self.cam_writer.writerow([t_ns, fname])
            self.frames_count += 1

            if self.target_seconds > 0 and (time.monotonic() - self.t_start) >= self.target_seconds:
                self.close_and_exit()

        def imu_cb(self, msg: Imu):
            t_ns = self.stamp_to_ns(msg.header)
            self.imu_writer.writerow([
                t_ns,
                f"{msg.angular_velocity.x:.6f}", f"{msg.angular_velocity.y:.6f}", f"{msg.angular_velocity.z:.6f}",
                f"{msg.linear_acceleration.x:.6f}", f"{msg.linear_acceleration.y:.6f}", f"{msg.linear_acceleration.z:.6f}"
            ])
            self.imu_count += 1

        def close_and_exit(self):
            self.cam_csv.close()
            self.imu_csv.close()
            meta = {
                "format": "euroc-like",
                "source": "Gazebo Harmonic simulation",
                "frames_written": self.frames_count,
                "imu_records": self.imu_count,
                "created_at": datetime.now().isoformat()
            }
            with open(os.path.join(self.out_dir, "meta.json"), "w") as f:
                json.dump(meta, f, indent=2)
            self.get_logger().info(f"Запис завершено: {self.frames_count} кадрів, {self.imu_count} IMU.")
            raise SystemExit(0)


def main():
    if not HAS_ROS2:
        print("ПОМИЛКА: ROS 2 (rclpy) не встановлено у поточному середовищі.\n"
              "Цей скрипт запускається всередині контейнера Gazebo (див. gazebo/Dockerfile)\n"
              "або на системі з ROS 2 Jazzy.")
        return 1

    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="папка збереження датасету")
    ap.add_argument("--seconds", type=float, default=0.0, help="тривалість запису в секундах (0 = до Ctrl+C)")
    args = ap.parse_args()

    rclpy.init()
    node = GazeboRecorderNode(args.out, args.seconds)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.close_and_exit()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
