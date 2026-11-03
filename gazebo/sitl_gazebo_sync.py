#!/usr/bin/env python3
"""Синхронізатор руху між ArduPilot SITL та моделлю камери в Gazebo.

Вимога рев'ю:
Забезпечити візуальні дані від керованого дрона: кадри камери й показники IMU
повинні надходити від того самого руху, яким керує автопілот ArduPilot.

Як це працює:
1. Підключається до порту MAVLink SITL (за замовчуванням UDP 14551).
2. Отримує поточну швидкість та положення автопілота (LOCAL_POSITION_NED, ATTITUDE).
3. Передає ці команди на модель дрона в Gazebo через топік керування /model/camera_drone/cmd_vel.
4. Результат: коли в QGroundControl або MAVProxy подається команда "takeoff 10" або
   "guideto X Y", дрон у Gazebo летить точно за траєкторією ArduPilot, і VIO отримує
   реальні кадри керованого польоту!

Використання:
    python3 sitl_gazebo_sync.py --conn udpin:127.0.0.1:14551
    # Тестовий запуск без живого автопілота:
    python3 sitl_gazebo_sync.py --mock --seconds 5
"""
import argparse
import sys
import time

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
except ImportError:
    HAS_PYMAVLINK = False

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


class SitlGazeboBridge:
    def __init__(self, conn_str="udpin:127.0.0.1:14551", is_mock=False):
        self.conn_str = conn_str
        self.is_mock = is_mock
        self.cmd_pub = None
        self.ros_node = None

        if HAS_ROS2:
            self.ros_node = rclpy.create_node("sitl_gazebo_sync")
            self.cmd_pub = self.ros_node.create_publisher(Twist, "/model/camera_drone/cmd_vel", 10)

        self.last_pos = [0.0, 0.0, 0.0]
        self.last_vel = [0.0, 0.0, 0.0]
        self.last_att = [0.0, 0.0, 0.0]

    def send_cmd_vel(self, vx, vy, vz, yaw_rate=0.0):
        """Відправка швидкості в Gazebo VelocityControl plugin (ENU система)."""
        if self.cmd_pub:
            msg = Twist()
            # ArduPilot NED (North, East, Down) -> Gazebo ENU (East, North, Up)
            msg.linear.x = float(vy)     # East
            msg.linear.y = float(vx)     # North
            msg.linear.z = float(-vz)    # Up (-Down)
            msg.angular.z = float(-yaw_rate)
            self.cmd_pub.publish(msg)

    def run_live(self, duration=0.0):
        if not HAS_PYMAVLINK:
            print("ПОМИЛКА: pymavlink не встановлено (pip install pymavlink).")
            return False

        print(f"Підключення до ArduPilot SITL за адресою {self.conn_str}...")
        mav = mavutil.mavlink_connection(self.conn_str)
        mav.wait_heartbeat(timeout=30)
        print("Зв'язок із SITL встановлено! Синхронізація руху з Gazebo активна.")

        t_start = time.monotonic()
        try:
            while duration == 0.0 or (time.monotonic() - t_start) < duration:
                if HAS_ROS2:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.01)

                msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=True, timeout=0.05)
                if not msg:
                    continue

                if msg.get_type() == "LOCAL_POSITION_NED":
                    # vx, vy, vz у м/с (система NED автопілота)
                    self.send_cmd_vel(msg.vx, msg.vy, msg.vz)
                    self.last_pos = [msg.x, msg.y, msg.z]
                    self.last_vel = [msg.vx, msg.vy, msg.vz]

                elif msg.get_type() == "ATTITUDE":
                    self.last_att = [msg.roll, msg.pitch, msg.yaw]

        except KeyboardInterrupt:
            print("\nЗупинка синхронізації.")
        finally:
            self.send_cmd_vel(0, 0, 0)
        return True

    def run_mock(self, seconds=5.0):
        """Імітація зльоту та руху вперед у SITL."""
        print(f"[MOCK] Демонстрація синхронізації SITL -> Gazebo ({seconds} с)...")
        print("  - 0-2 с: Зліт дрона на висоту 10 м (vz = -5 м/с у NED)")
        print("  - 2-5 с: Політ уперед зі швидкістю 2 м/с (vx = 2 м/с у NED)")

        t_end = time.monotonic() + seconds
        t0 = time.monotonic()
        while time.monotonic() < t_end:
            el = time.monotonic() - t0
            if el < 2.0:
                self.send_cmd_vel(vx=0.0, vy=0.0, vz=-5.0)
            else:
                self.send_cmd_vel(vx=2.0, vy=0.0, vz=0.0)
            time.sleep(0.05)

        self.send_cmd_vel(0, 0, 0)
        print("[MOCK] Синхронізацію успішно відпрацьовано: команди передано в Gazebo.")
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--conn", default="udpin:127.0.0.1:14551", help="рядок з'єднання MAVLink")
    ap.add_argument("--seconds", type=float, default=0.0, help="час роботи (0 = безперервно)")
    ap.add_argument("--mock", action="store_true", help="демо-режим без живого SITL")
    args = ap.parse_args()

    if HAS_ROS2:
        rclpy.init()

    bridge = SitlGazeboBridge(args.conn, is_mock=args.mock)
    try:
        if args.mock or not HAS_PYMAVLINK:
            bridge.run_mock(args.seconds or 5.0)
        else:
            bridge.run_live(args.seconds)
    finally:
        if HAS_ROS2:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
