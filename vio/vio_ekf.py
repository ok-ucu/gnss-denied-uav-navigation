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
    ap.add_argument("--rate", type=float, default=30.0, help="Гц виходу в ArduPilot (мінімум)")
    ap.add_argument("--vis-rate", type=float, default=20.0, help="Гц 'камери'")
    ap.add_argument("--dropout", action="store_true", help="вмикати провали камери")
    ap.add_argument("--log", default="vio_log.csv")
    ap.add_argument("--video", default=None, help="відео/номер камери; без нього працює SimVisualSensor")
    ap.add_argument("--ros-image", default=None, metavar="TOPIC",
                    help="топік sensor_msgs/Image із Gazebo (ROS 2), напр. /camera/image_raw")
    ap.add_argument("--ros-sim-time", action="store_true",
                    help="годинник ROS-вузла = /clock Gazebo: тоді latency_ms включає "
                         "вік кадра (stamp -> прийом); потрібен міст /clock")
    ap.add_argument("--fx", type=float, default=500.0, help="фокусна відстань камери, пікселі")
    ap.add_argument("--vertical", choices=["range", "ap", "auto"], default=None,
                    help="звідки брати висоту/вертикаль: range - лише далекомір "
                         "(незалежно від EKF3), ap - з LOCAL_POSITION_NED автопілота, "
                         "auto - далекомір, поки він живий і дивиться вниз, інакше висота "
                         "автопілота (з попередженням). За замовчуванням: auto для камери, "
                         "ap для SimVisualSensor")
    ap.add_argument("--mute-at", type=float, default=None, metavar="S",
                    help="тест EKF3: через S с після старту на --mute-for с припинити відправку "
                         "в ArduPilot (STATUSTEXT 'VIO: MUTE START/END'); див. check_ekf.py")
    ap.add_argument("--mute-for", type=float, default=10.0, metavar="S")
    ap.add_argument("--max-latency-ms", type=float, default=50.0,
                    help="поріг попередження про затримку кадр -> MAVLink")
    ap.add_argument("--ros-odom", default=None, metavar="TOPIC",
                    help="топік nav_msgs/Odometry для виходу VIO (за замовчуванням /vio/odom, "
                         "коли є --ros-image); поряд іде /vio/status")
    args = ap.parse_args()

    use_camera = args.video is not None or args.ros_image is not None
    vertical = args.vertical or ("auto" if use_camera else "ap")

    master = mavutil.mavlink_connection(args.conn)
    print("Чекаю heartbeat...")
    master.wait_heartbeat()
    sysid, compid = master.target_system, master.target_component
    print(f"Підключено: system {sysid}; вертикаль: {vertical}")

    master.mav.set_gps_global_origin_send(
        sysid, int(ORIGIN[0] * 1e7), int(ORIGIN[1] * 1e7),
        int(ORIGIN[2] * 1000), int(time.time() * 1e6))

    # просимо потрібні повідомлення: RAW_IMU, ATTITUDE, LOCAL_POSITION_NED,
    # SIMSTATE, DISTANCE_SENSOR (далекомір, якщо є)
    def request_streams():
        for msg_id, hz in ((27, 50), (30, 30), (32, 30), (164, 30), (132, 10)):
            master.mav.command_long_send(
                sysid, compid, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, 1e6 / hz, 0, 0, 0, 0, 0)

    request_streams()
    last_req = time.time()

    ekf = VioEKF()
    monitor = QualityMonitor()
    sensor = SimVisualSensor(dropout=args.dropout)
    cap = None
    ros = None
    frontend = None
    is_file = False
    video_fps = 30.0
    frame_idx = 0
    t_video0 = None
    eof_reported = False
    if args.ros_image is not None:
        from vio_ros_source import RosFrameSource
        ros = RosFrameSource(args.ros_image, use_sim_time=args.ros_sim_time)
        frontend = FlowFrontend(fx=args.fx)
        print(f"Кадри з ROS 2: {args.ros_image}")
        if not args.ros_sim_time:
            print("УВАГА: без --ros-sim-time затримка рахується лише від прийому кадра вузлом")
    elif args.video is not None:
        v = args.video
        is_pipeline = "!" in v                       # GStreamer-пайплайн
        is_stream = is_pipeline or "://" in v        # udp://, rtsp://...
        is_file = not (v.isdigit() or is_stream)     # лише справжній файл програється "за годинником"
        if v.isdigit():
            cap = cv2.VideoCapture(int(v))
        elif is_pipeline:
            cap = cv2.VideoCapture(v, cv2.CAP_GSTREAMER)
        else:
            cap = cv2.VideoCapture(v)
        if not cap.isOpened():
            raise SystemExit(f"Не вдалося відкрити джерело відео: {v}\n"
                             "(для пайплайна потрібен OpenCV зі збіркою GStreamer: "
                             "перевір cv2.getBuildInformation())")
        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps and fps > 1.0:
            video_fps = fps
        frontend = FlowFrontend(fx=args.fx)
    odom = None
    odom_topic = args.ros_odom or ("/vio/odom" if ros is not None else None)
    if odom_topic:
        from vio_ros_out import RosOdomPublisher
        odom = RosOdomPublisher(node=ros.node if ros is not None else None,
                                topic=odom_topic, use_sim_time=args.ros_sim_time)
        print(f"Публікую Odometry: {odom_topic} (odom_ned/base_link_frd), статус: /vio/status")
    source_name = ("optical_flow:ros" if ros is not None else
                   "optical_flow:video" if cap is not None else "SIMULATED")
    gyro = np.zeros(3)
    h_local = 1.0
    h_range, t_range = None, 0.0
    t_range_upd = 0.0
    warned_range = False
    range_buf = collections.deque(maxlen=3)      # медіана з 3: прибирає поодинокі викиди
    n_rng = n_rng_bad = 0
    t_loc = 0.0
    last_h, h_src = float("nan"), "none"
    ratio_bad, warned_ratio = 0, False
    was_muted = False

    roll = pitch = yaw = 0.0
    have_att = False
    last_imu_t = None            # час останнього RAW_IMU (з самого повідомлення)
    last_vis = last_out = 0.0
    last_vert = 0.0
    z_init = False
    accepted = rejected = 0
    degraded = False
    paused = False
    force_out = False            # щойно була корекція від камери - шлемо одразу
    pending_wall = None          # момент отримання кадра, що ще не пішов у MAVLink
    pending_age = 0.0            # вік кадра на момент отримання (stamp -> прийом), с
    last_nis = float("nan")
    last_inl, last_sig = "", ""
    lat_max = lat_sum = 0.0
    lat_n = n_out = 0
    lat_warned = False
    t0 = time.time()

    n_imu = n_pred = n_skip = n_kin = 0
    n_att = n_sim = n_loc = 0
    n_cam = 0                    # кадри, отримані за останню секунду
    stamp_first = stamp_last = None
    dt_max = 0.0
    t_rep = time.time()

    logf = open(args.log, "w", newline="")
    log = csv.writer(logf)
    log.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "true_n", "true_e",
                  "err_xy", "quality", "accepted", "rejected", "nis", "n_inliers",
                  "sigma_v", "latency_ms", "ap_vn", "ap_ve", "pos_var", "speed_var", "h_used"])
    if use_camera:
        print("Джерело швидкості: ОПТИЧНИЙ ПОТІК. SIMSTATE/LOCAL_POSITION_NED у логі "
              "лише як істина для порівняння (колонки true_*, ap_*), у фільтр не йдуть.")
    print("VIO працює. Зупинка: Ctrl+C")

    try:
        while True:
            msg = master.recv_match(
                type=["RAW_IMU", "ATTITUDE", "SIMSTATE", "LOCAL_POSITION_NED",
                      "DISTANCE_SENSOR"],
                blocking=True, timeout=0.01)
            now = time.time()
            # інша програма (напр. MAVProxy зі streamrate=4) може збити частоти
            # повідомлень, тому періодично просимо їх знову
            if now - last_req >= 3.0:
                request_streams()
                last_req = now
            if msg is not None:
                ty = msg.get_type()
                if ty == "ATTITUDE":
                    roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw
                    have_att = True
                    n_att += 1
                elif ty == "RAW_IMU":
                    # RAW_IMU: мілі-g -> м/с^2; dt беремо з часових міток автопілота,
                    # а не з моменту прибуття UDP-пакета
                    n_imu += 1
                    t_imu = msg.time_usec * 1e-6 if msg.time_usec else now
                    f = np.array([msg.xacc, msg.yacc, msg.zacc]) / 1000.0 * G
                    if have_att and last_imu_t is not None:
                        dt = t_imu - last_imu_t
                        dt_max = max(dt_max, dt)
                        if 0.0 < dt < 0.2:
                            ekf.predict(f, rot_body_to_ned(roll, pitch, yaw), dt)
                            n_pred += 1
                        elif 0.2 <= dt < 3.0:
                            # IMU надто рідкий: не зупиняємо інтегрування позиції
                            ekf.predict_kinematic(dt)
                            n_kin += 1
                        else:
                            n_skip += 1       # dt <= 0 або величезний стрибок
                    gyro = np.array([msg.xgyro, msg.ygyro, msg.zgyro]) / 1000.0
                    last_imu_t = t_imu
                elif ty == "SIMSTATE":
                    # у SIMSTATE поля називаються lat/lng (degE7)
                    lat = getattr(msg, "lat", None)
                    lng = getattr(msg, "lng", None)
                    if lat is None:
                        lat, lng = msg.lat_int, msg.lng_int
                    sensor.feed_simstate(now, lat * 1e-7, lng * 1e-7)
                    n_sim += 1
                elif ty == "LOCAL_POSITION_NED":
                    sensor.feed_local(msg.vx, msg.vy, msg.vz)
                    n_loc += 1
                    h_local = max(-msg.z, 0.5)
                    t_loc = now
                    # у режимі "ap" вертикаль підтримуємо з автопілота; у режимі
                    # "range" LOCAL_POSITION_NED тут НЕ використовується взагалі
                    if vertical == "ap" or (vertical == "auto" and now - t_range >= 0.5):
                        if not z_init:
                            ekf.x[2], ekf.x[5] = msg.z, msg.vz
                            z_init = True
                        elif now - last_vert >= 0.1:
                            ekf.update_vertical(msg.z, msg.vz)
                            last_vert = now
                elif ty == "DISTANCE_SENSOR":
                    n_rng += 1
                    d = msg.current_distance / 100.0
                    lo, hi = msg.min_distance / 100.0, msg.max_distance / 100.0
                    if getattr(msg, "orientation", 25) != 25:
                        # 25 = вниз (PITCH_270). Далекомір в іншу сторону зіпсував би висоту
                        n_rng_bad += 1
                    elif lo <= d < hi - 0.05:        # d біля max = "нічого не видно", не висота
                        range_buf.append(d)
                        d_med = float(np.median(range_buf))
                        # далекомір міряє вздовж осі дрона: у висоту по вертикалі
                        # переводимо через нахил
                        cos_tilt = max(float(rot_body_to_ned(roll, pitch, yaw)[2, 2]), 0.5)
                        h_range, t_range = d_med * cos_tilt, now
                        if vertical in ("range", "auto"):
                            if not z_init:
                                ekf.x[2] = -h_range
                                z_init = True
                            elif now - t_range_upd >= 0.1:
                                ekf.update_height(-h_range)
                                t_range_upd = now

            if vertical in ("range", "auto") and use_camera and not warned_range \
                    and now - t0 > 5.0 and h_range is None:
                if vertical == "range":
                    print("!!! Немає DISTANCE_SENSOR: без висоти швидкість не порахувати. "
                          "Підключи далекомір або запусти з --vertical auto/ap")
                else:
                    print("!!! Немає придатного DISTANCE_SENSOR (немає повідомлень, не вниз або "
                          "поза діапазоном): масштаб потоку береться з висоти EKF3, тобто НЕ "
                          "незалежний. Діагностика: check_range.py")
                warned_range = True

            # ---------- вимірювання від камери ----------
            frame = None                       # (gray, t_frame, t_wall, age_s)
            if have_att and ros is not None:
                frame = ros.poll()
            elif have_att and cap is not None and now - last_vis >= 1.0 / args.vis_rate:
                last_vis = now
                if is_file:
                    # відео "летить" у реальному часі: пропускаємо кадри, що
                    # не встигли, інакше швидкість і позиція масштабуються хибно
                    if t_video0 is None:
                        t_video0 = now
                    target = int((now - t_video0) * video_fps) + 1
                    if target > frame_idx:
                        for _ in range(target - frame_idx - 1):
                            cap.grab()
                        got, bgr = cap.read()
                        if got:
                            frame_idx = target
                            frame = (cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY),
                                     frame_idx / video_fps, time.time(), 0.0)
                        else:
                            monitor.push(False, now)
                            rejected += 1
                            if not eof_reported:
                                print("Кадри закінчились")
                                eof_reported = True
                else:
                    got, bgr = cap.read()
                    if got:
                        frame = (cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), time.time(), time.time(), 0.0)
                    else:
                        monitor.push(False, now)
                        rejected += 1
                        if not eof_reported:
                            print("Камера не віддає кадри")
                            eof_reported = True

            if frame is not None:
                gray, t_frame, t_wall, t_age = frame
                n_cam += 1
                stamp_first = t_frame if stamp_first is None else stamp_first
                stamp_last = t_frame
                # висота: свіжий далекомір, інакше (лише в режимі ap) LOCAL_POSITION_NED;
                # у режимі range без далекоміра висоти немає => кадр відкидається
                if h_range is not None and now - t_range < 0.5:
                    h, h_src = h_range, "range"
                elif vertical in ("ap", "auto"):
                    h, h_src = h_local, "ap"
                else:
                    h, h_src = 0.0, "none"
                last_h = h
                res = frontend.process(gray, t_frame, h, gyro,
                                       rot_body_to_ned(roll, pitch, yaw))
                if res is None:                  # кадр непридатний
                    monitor.push(False, now)
                    rejected += 1
                else:
                    sigma_scale = res.sigma_scale
                    ok, last_nis = ekf.update_velocity_xy(res.v_xy, sigma_scale)
                    monitor.push(ok, now)
                    accepted += ok
                    rejected += (not ok)
                    last_inl, last_sig = res.n_inliers, res.sigma_v
                    if ok:
                        force_out = True
                        pending_wall, pending_age = t_wall, (t_age or 0.0)
            elif have_att and cap is None and ros is None and now - last_vis >= 1.0 / args.vis_rate:
                # імітована камера
                last_vis = now
                v = sensor.measure(now)
                if v is not None:
                    ok, last_nis = ekf.update_velocity(v)
                    monitor.push(ok, now)
                    accepted += ok
                    rejected += (not ok)
                    if ok:
                        force_out = True
                        pending_wall, pending_age = now, 0.0

            # ---------- вихід в ArduPilot ----------
            if have_att and (force_out or now - last_out >= 1.0 / args.rate):
                force_out = False
                last_out = now
                q = monitor.quality(now)
                infl = monitor.inflate(q)
                usec = int((now - t0) * 1e6)
                x, y, z = ekf.x[0:3]
                vx, vy, vz = ekf.x[3:6]
                lat_ms = float("nan")

                muted = args.mute_at is not None and \
                    args.mute_at <= now - t0 < args.mute_at + args.mute_for
                if muted != was_muted:
                    was_muted = muted
                    master.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_NOTICE,
                        b"VIO: MUTE START" if muted else b"VIO: MUTE END")
                    print("VIO: відправку в ArduPilot " + ("ПРИПИНЕНО (тест)" if muted else "відновлено"))

                if q >= MIN_SEND_Q and not muted:
                    if paused:
                        paused = False
                        print("VIO знову відправляє дані")
                    master.mav.vision_position_estimate_send(
                        usec, x, y, z, roll, pitch, yaw, ekf.pose_cov(infl))
                    master.mav.vision_speed_estimate_send(
                        usec, vx, vy, vz, ekf.speed_cov(infl))
                    if odom is not None:
                        odom.publish(rot_body_to_ned(roll, pitch, yaw), ekf.x[0:3], ekf.x[3:6],
                                     ekf.P[0:3, 0:3] * infl, ekf.P[3:6, 3:6] * infl,
                                     3e-4 * infl)
                    n_out += 1
                    if pending_wall is not None:
                        # повна затримка: вік кадра при отриманні (stamp -> вузол,
                        # якщо відомий) + обробка до відправки пакета
                        lat_ms = (pending_age + time.time() - pending_wall) * 1000.0
                        pending_wall = None
                        lat_max = max(lat_max, lat_ms)
                        lat_sum += lat_ms
                        lat_n += 1
                        if lat_ms > args.max_latency_ms and not lat_warned:
                            print(f"УВАГА: затримка кадр->MAVLink {lat_ms:.0f} мс "
                                  f"> {args.max_latency_ms:.0f} мс")
                            lat_warned = True
                elif q < MIN_SEND_Q and not paused:
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
                              sensor.n, sensor.e, err, q, accepted, rejected,
                              last_nis, last_inl if frontend else "",
                              last_sig if frontend else "", lat_ms,
                              sensor.v_local[0] if sensor.v_local is not None else "",
                              sensor.v_local[1] if sensor.v_local is not None else "",
                              ekf.P[0, 0] * infl, ekf.P[3, 3] * infl, last_h])
                last_nis = float("nan")

            if now - t_rep >= 1.0:
                lat_txt = f"{lat_sum / lat_n:.1f}/{lat_max:.1f} мс" if lat_n else "-"
                print(f"RAW_IMU {n_imu}/с (макс. dt {dt_max:.2f} с), predict {n_pred}, "
                      f"без IMU-прискорень {n_kin}, пропущено {n_skip} | "
                      f"ATT {n_att}, SIMSTATE {n_sim}, LOCAL {n_loc} | "
                      f"вихід {n_out} Гц, затримка сер/макс {lat_txt} | "
                      f"перезапусків швидкості {ekf.n_resets}")
                if use_camera:
                    win = now - t_rep
                    rtf = ((stamp_last - stamp_first) / win) if n_cam > 1 and win > 0 else float("nan")
                    extra = ""
                    if ros is not None:
                        extra = (f", усього {ros.n_frames}, непідтримане кодування {ros.n_unsupported}, "
                                 f"поза межами віку {ros.n_bad_age}")
                    print(f"КАМЕРА {n_cam} кадр/с (stamp-час/реальний = {rtf:.2f}){extra}")
                    if n_cam == 0 and now - t0 > 5.0:
                        print("!!! Кадрів немає: перевір топік (ros2 topic hz), міст Gazebo->ROS, "
                              "QoS та чи не на паузі симуляція")
                if use_camera and vertical in ("range", "auto", "ap"):
                    ratio = float("nan")
                    if h_range is not None and now - t_range < 1.0 and h_local >= 1.5 and now - t_loc < 1.0:
                        ratio = h_range / h_local
                    print(f"ВИСОТА: джерело {h_src}, h={last_h:.2f} м | DISTANCE_SENSOR {n_rng}/с "
                          f"(не вниз: {n_rng_bad}) | range/EKF3 = {ratio:.2f}")
                    ratio_bad = ratio_bad + 1 if (np.isfinite(ratio) and not 0.7 <= ratio <= 1.4) else 0
                    if ratio_bad >= 3 and not warned_ratio:
                        print("!!! Далекомір і висота EKF3 розходяться більш ніж у 1.4 раза: масштаб "
                              "потоку хибний. Перевір RNGFND1_SCALING/ORIENT/MIN/MAX і чи земля на "
                              "висоті home. Діагностика: check_range.py")
                        warned_ratio = True
                if odom is not None:
                    odom.publish_status(
                        source=source_name, quality=monitor.quality(now),
                        frames_in=(ros.n_frames if ros is not None else None),
                        frames_per_s=n_cam, accepted=accepted, rejected=rejected,
                        odom_published=odom.n_pub, speed_resets=ekf.n_resets,
                        last_frame_stamp=stamp_last, paused=paused,
                        height_source=h_src, h_used=last_h)
                if n_imu < 30:
                    print("УВАГА: RAW_IMU приходить лише "
                          f"{n_imu} Гц - частоти повідомлень збиті (GCS/MAVProxy?)")
                n_imu = n_pred = n_skip = n_kin = 0
                n_att = n_sim = n_loc = n_out = 0
                n_cam = 0
                n_rng = n_rng_bad = 0
                stamp_first = stamp_last = None
                lat_max = lat_sum = 0.0
                lat_n = 0
                dt_max = 0.0
                t_rep = now
    except KeyboardInterrupt:
        print("\nЗупинено.")
    finally:
        logf.close()
        if odom is not None:
            odom.close()
        if ros is not None:
            ros.close()


if __name__ == "__main__":
    main()
