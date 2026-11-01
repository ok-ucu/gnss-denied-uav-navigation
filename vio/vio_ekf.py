#!/usr/bin/env python3
import argparse
import collections
import csv
import math
import time

import numpy as np
import cv2
from pymavlink import mavutil

from vio_core import G, QualityMonitor, VioEKF, rot_body_to_ned
from vio_frontend import FlowFrontend

ORIGIN = (-35.363261, 149.165230, 584.0)
MIN_SEND_Q = 0.1      # нижче цієї якості в ArduPilot нічого не шлемо
DEGRADED_Q = 0.5


def meters_per_degree(lat_deg):
    """Скільки метрів у градусі широти та довготи на заданій широті."""
    phi = math.radians(lat_deg)
    m_lat = 111132.954 - 559.822 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)
    m_lon = 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi) + 0.118 * math.cos(5 * phi)
    return m_lat, m_lon


class SimVisualSensor:
    """Замінник камери: бере справжню швидкість із симулятора і псує її."""

    WINDOW_S = 0.15     # вікно для швидкості; менше вікно = менша затримка

    def __init__(self, dropout=False, seed=0):
        self.rng = np.random.default_rng(seed)
        self.dropout = dropout
        self.scale_err = 0.02
        self.bias = np.array([0.02, -0.01, 0.0])
        self.noise = 0.05
        self.hist = collections.deque()
        self.lat0 = self.lon0 = None
        self.m_lat = self.m_lon = None
        self.n = self.e = 0.0
        self.vz = 0.0
        self.v_local = None
        self.have_simstate = False
        self.t_start = time.time()

    def feed_simstate(self, t, lat_deg, lon_deg):
        if self.lat0 is None:
            self.lat0, self.lon0 = lat_deg, lon_deg
            self.m_lat, self.m_lon = meters_per_degree(lat_deg)
        self.have_simstate = True
        self.n = (lat_deg - self.lat0) * self.m_lat
        self.e = (lon_deg - self.lon0) * self.m_lon
        self.hist.append((t, self.n, self.e))
        while len(self.hist) > 2 and t - self.hist[0][0] > self.WINDOW_S:
            self.hist.popleft()

    def feed_local(self, vx, vy, vz):
        self.vz = vz
        self.v_local = np.array([vx, vy, vz])

    def _true_velocity(self):
        if self.have_simstate and len(self.hist) >= 2:
            t1, n1, e1 = self.hist[-1]
            t0, n0, e0 = self.hist[0]
            if t1 - t0 > 0.08:
                return np.array([(n1 - n0) / (t1 - t0), (e1 - e0) / (t1 - t0), self.vz])
            return None
        return self.v_local

    def blackout(self, now):
        """Провал "камери": 4 с кожні 40 с (однотонне поле, засвітка...)."""
        if not self.dropout:
            return False
        return 20.0 <= (now - self.t_start) % 40.0 < 24.0

    def measure(self, now):
        if self.blackout(now):
            return None
        v = self._true_velocity()
        if v is None:
            return None
        return (1 + self.scale_err) * v + self.bias + self.rng.normal(0, self.noise, 3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="udpin:127.0.0.1:14551")
    ap.add_argument("--rate", type=float, default=30.0, help="Гц виходу в ArduPilot")
    ap.add_argument("--vis-rate", type=float, default=20.0, help="Гц 'камери'")
    ap.add_argument("--dropout", action="store_true", help="вмикати провали камери")
    ap.add_argument("--log", default="vio_log.csv")
    ap.add_argument("--video", default=None, help="відео/номер камери; без нього працює SimVisualSensor")
    ap.add_argument("--fx", type=float, default=500.0, help="фокусна відстань камери, пікселі")
    args = ap.parse_args()

    master = mavutil.mavlink_connection(args.conn)
    print("Чекаю heartbeat...")
    master.wait_heartbeat()
    sysid, compid = master.target_system, master.target_component
    print(f"Підключено: system {sysid}")

    master.mav.set_gps_global_origin_send(
        sysid, int(ORIGIN[0] * 1e7), int(ORIGIN[1] * 1e7),
        int(ORIGIN[2] * 1000), int(time.time() * 1e6))

    # просимо потрібні повідомлення: RAW_IMU, ATTITUDE, LOCAL_POSITION_NED,
    # SIMSTATE, DISTANCE_SENSOR (далекомір, якщо є)
    for msg_id, hz in ((27, 50), (30, 30), (32, 30), (164, 30), (132, 10)):
        master.mav.command_long_send(
            sysid, compid, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, 1e6 / hz, 0, 0, 0, 0, 0)

    ekf = VioEKF()
    monitor = QualityMonitor()
    sensor = SimVisualSensor(dropout=args.dropout)
    cap = None
    frontend = None
    is_file = False
    video_fps = 30.0
    frame_idx = 0
    eof_reported = False
    if args.video is not None:
        is_file = not args.video.isdigit()
        cap = cv2.VideoCapture(args.video if is_file else int(args.video))
        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps and fps > 1.0:
            video_fps = fps
        frontend = FlowFrontend(fx=args.fx)
    gyro = np.zeros(3)
    h_local = 1.0
    h_range, t_range = None, 0.0

    roll = pitch = yaw = 0.0
    have_att = False
    last_imu_t = None            # час останнього RAW_IMU (з самого повідомлення)
    last_vis = last_out = 0.0
    last_vert = 0.0
    z_init = False
    accepted = rejected = 0
    degraded = False
    paused = False
    t0 = time.time()

    n_imu = n_pred = n_skip = 0
    t_rep = time.time()

    logf = open(args.log, "w", newline="")
    log = csv.writer(logf)
    log.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "true_n", "true_e",
                  "err_xy", "quality", "accepted", "rejected"])
    print("VIO працює. Зупинка: Ctrl+C")

    try:
        while True:
            msg = master.recv_match(
                type=["RAW_IMU", "ATTITUDE", "SIMSTATE", "LOCAL_POSITION_NED",
                      "DISTANCE_SENSOR"],
                blocking=True, timeout=0.01)
            now = time.time()
            if msg is not None:
                ty = msg.get_type()
                if ty == "ATTITUDE":
                    roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw
                    have_att = True
                elif ty == "RAW_IMU":
                    # RAW_IMU: мілі-g -> м/с^2; dt беремо з часових міток автопілота,
                    # а не з моменту прибуття UDP-пакета
                    n_imu += 1
                    t_imu = msg.time_usec * 1e-6 if msg.time_usec else now
                    f = np.array([msg.xacc, msg.yacc, msg.zacc]) / 1000.0 * G
                    if have_att and last_imu_t is not None:
                        dt = t_imu - last_imu_t
                        if 0.0 < dt < 0.2:
                            ekf.predict(f, rot_body_to_ned(roll, pitch, yaw), dt)
                            n_pred += 1
                        else:
                            n_skip += 1
                    gyro = np.array([msg.xgyro, msg.ygyro, msg.zgyro]) / 1000.0
                    last_imu_t = t_imu
                elif ty == "SIMSTATE":
                    # у SIMSTATE поля називаються lat/lng (degE7)
                    lat = getattr(msg, "lat", None)
                    lng = getattr(msg, "lng", None)
                    if lat is None:
                        lat, lng = msg.lat_int, msg.lng_int
                    sensor.feed_simstate(now, lat * 1e-7, lng * 1e-7)
                elif ty == "LOCAL_POSITION_NED":
                    sensor.feed_local(msg.vx, msg.vy, msg.vz)
                    h_local = max(-msg.z, 0.5)
                    # вертикаль камерою не міряється - підтримуємо її з автопілота
                    if not z_init:
                        ekf.x[2], ekf.x[5] = msg.z, msg.vz
                        z_init = True
                    elif now - last_vert >= 0.1:
                        ekf.update_vertical(msg.z, msg.vz)
                        last_vert = now
                elif ty == "DISTANCE_SENSOR":
                    d = msg.current_distance / 100.0
                    if msg.min_distance / 100.0 <= d <= msg.max_distance / 100.0:
                        h_range, t_range = d, now

            if have_att and now - last_vis >= 1.0 / args.vis_rate:
                last_vis = now
                if cap is not None:
                    got, frame = cap.read()
                    if got:
                        frame_idx += 1
                        # час кадру: для файлу - з самого відео, для камери - момент зчитування
                        t_frame = frame_idx / video_fps if is_file else now
                        # висота: далекомір, якщо свіжий, інакше - з LOCAL_POSITION_NED
                        h = h_range if (h_range is not None and now - t_range < 0.5) else h_local
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        res = frontend.process(gray, t_frame, h, gyro,
                                               rot_body_to_ned(roll, pitch, yaw))
                        if res is None:          # кадр непридатний
                            monitor.push(False, now)
                            rejected += 1
                        else:
                            ok, _ = ekf.update_velocity_xy(res.v_xy, res.sigma_scale)
                            monitor.push(ok, now)
                            accepted += ok
                            rejected += (not ok)
                    else:
                        # відео скінчилось або камера не віддала кадр
                        monitor.push(False, now)
                        rejected += 1
                        if not eof_reported:
                            print("Кадри закінчились (або камера не відповідає)")
                            eof_reported = True
                else:
                    # імітована камера
                    v = sensor.measure(now)
                    if v is not None:
                        ok, _ = ekf.update_velocity(v)
                        monitor.push(ok, now)
                        accepted += ok
                        rejected += (not ok)
            if have_att and now - last_out >= 1.0 / args.rate:
                last_out = now
                q = monitor.quality(now)
                infl = monitor.inflate(q)
                usec = int((now - t0) * 1e6)
                x, y, z = ekf.x[0:3]
                vx, vy, vz = ekf.x[3:6]

                if q >= MIN_SEND_Q:
                    if paused:
                        paused = False
                        print("VIO знову відправляє дані")
                    master.mav.vision_position_estimate_send(
                        usec, x, y, z, roll, pitch, yaw, ekf.pose_cov(infl))
                    master.mav.vision_speed_estimate_send(
                        usec, vx, vy, vz, ekf.speed_cov(infl))
                elif not paused:
                    # даних майже немає - краще мовчати, ніж слати дрейф від IMU
                    paused = True
                    print("!!! VIO: даних немає, відправку зупинено")

                if q < DEGRADED_Q and not degraded:
                    degraded = True
                    master.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_WARNING, b"VIO: DEGRADED")
                    print("!!! VIO деградував, коваріацію збільшено")
                elif q >= DEGRADED_Q and degraded:
                    degraded = False
                    master.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_INFO, b"VIO: recovered")
                    print("VIO відновився")

                err = math.hypot(x - sensor.n, y - sensor.e) if sensor.have_simstate else float("nan")
                log.writerow([f"{now - t0:.3f}", x, y, z, vx, vy, vz,
                              sensor.n, sensor.e, err, q, accepted, rejected])
            if now - t_rep >= 1.0:
                print(f"RAW_IMU {n_imu}/с, predict {n_pred}, пропущено {n_skip}, "
                      f"перезапусків швидкості {ekf.n_resets}")
                n_imu = n_pred = n_skip = 0
                t_rep = now
    except KeyboardInterrupt:
        print("\nЗупинено.")
    finally:
        logf.close()


if __name__ == "__main__":
    main()
