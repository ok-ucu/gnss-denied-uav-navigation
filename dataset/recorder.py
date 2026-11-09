#!/usr/bin/env python3
"""Записувач датасету: кадри камери + IMU + атитюд + GPS (ground truth) для replay.

Усе пишеться на одному годиннику бортового комп'ютера (CLOCK_MONOTONIC, нс).
Формат - EuRoC-подібний (його читають Kalibr-конвертер, OpenVINS, VINS-Fusion),
опис у dataset/README.md.

Приклади:
  # SITL-стенд (камера синтетична, без заліза):
  python3 recorder.py --conn udpin:127.0.0.1:14551 --camera-config ../camera/camera_config.yaml --synthetic
  # Реальний дрон, UART до польотного контролера:
  python3 recorder.py --conn /dev/ttyTHS1 --baud 921600 --camera-config ../camera/camera_config.yaml
"""
import argparse
import csv
import json
import os
import platform
import queue
import subprocess
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
except ImportError:
    mavutil = None
    HAS_PYMAVLINK = False

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "camera"))
sys.path.insert(0, HERE)
from capture import Camera, CaptureThread, load_config   # noqa: E402
import timesync                                           # noqa: E402

G = 9.80665
# id повідомлення -> (назва, ціль у Гц за замовчуванням)
MSG_IDS = {"RAW_IMU": 27, "ATTITUDE": 30, "LOCAL_POSITION_NED": 32,
           "GPS_RAW_INT": 24, "GLOBAL_POSITION_INT": 33, "DISTANCE_SENSOR": 132}


class MockMavMsg:
    def __init__(self, msg_type, **kwargs):
        self._type = msg_type
        self.__dict__.update(kwargs)

    def get_type(self):
        return self._type

    def get_srcSystem(self):
        return 1


class MockAutopilotConnection:
    """Генератор синтетичних повідомлень MAVLink для тестів без реального автопілота."""

    def __init__(self, rates, clock_offset_s=1234.5, drift_ppm=30.0):
        self.target_system = 1
        self.target_component = 1
        self.rates = rates
        self.clock_offset_s = clock_offset_s
        self.drift = drift_ppm * 1e-6
        self.t0 = time.monotonic()
        self.rng = np.random.default_rng(42)
        self.last_sent = {k: 0.0 for k in rates}
        self.n = 0
        self.mav = self

    def wait_heartbeat(self, timeout=30):
        return True

    def command_long_send(self, *args, **kwargs):
        pass

    def recv_match(self, type=None, blocking=True, timeout=0.05):
        now = time.monotonic()
        t_ap = (now - self.t0) * (1.0 + self.drift) + self.clock_offset_s
        # Порядок перевірки з урахуванням частоти
        for name in ("RAW_IMU", "ATTITUDE", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "DISTANCE_SENSOR"):
            hz = self.rates.get(name, 10.0)
            interval = 1.0 / max(hz, 0.1)
            if now - self.last_sent[name] >= interval:
                self.last_sent[name] = now
                if name == "RAW_IMU":
                    self.n += 1
                    ph = self.n * 0.02
                    jitter = self.rng.uniform(0.001, 0.004)
                    return MockMavMsg("RAW_IMU",
                                      time_usec=int(t_ap * 1e6),
                                      xgyro=int(np.sin(ph) * 80),
                                      ygyro=int(np.cos(ph) * 40),
                                      zgyro=int(np.sin(ph * 0.5) * 20),
                                      xacc=int(np.sin(ph) * 30),
                                      yacc=int(np.cos(ph) * 30),
                                      zacc=-1000)
                elif name == "ATTITUDE":
                    return MockMavMsg("ATTITUDE",
                                      time_boot_ms=int(t_ap * 1e3) & 0xFFFFFFFF,
                                      roll=0.01, pitch=0.02, yaw=0.05,
                                      rollspeed=0.001, pitchspeed=0.002, yawspeed=0.0)
                elif name == "GPS_RAW_INT":
                    return MockMavMsg("GPS_RAW_INT",
                                      time_usec=int(t_ap * 1e6),
                                      fix_type=3,
                                      lat=int((50.4501 + self.n * 1e-7) * 1e7),
                                      lon=int((30.5234 + self.n * 1e-7) * 1e7),
                                      alt=150000,
                                      eph=100, epv=150,
                                      vel=150,
                                      satellites_visible=12)
                elif name == "LOCAL_POSITION_NED":
                    return MockMavMsg("LOCAL_POSITION_NED",
                                      time_boot_ms=int(t_ap * 1e3) & 0xFFFFFFFF,
                                      x=float(self.n * 0.02), y=float(self.n * 0.01), z=-10.0,
                                      vx=0.5, vy=0.2, vz=0.0)
                elif name == "GLOBAL_POSITION_INT":
                    return MockMavMsg("GLOBAL_POSITION_INT",
                                      time_boot_ms=int(t_ap * 1e3) & 0xFFFFFFFF,
                                      lat=int((50.4501 + self.n * 1e-7) * 1e7),
                                      lon=int((30.5234 + self.n * 1e-7) * 1e7),
                                      alt=150000, relative_alt=10000,
                                      vx=50, vy=20, vz=0, hdg=9000)
                elif name == "DISTANCE_SENSOR":
                    return MockMavMsg("DISTANCE_SENSOR",
                                      time_boot_ms=int(t_ap * 1e3) & 0xFFFFFFFF,
                                      current_distance=1000,
                                      min_distance=20, max_distance=4000)
        time.sleep(min(timeout, 0.002))
        return None


class Csv:
    """CSV з періодичним flush, щоб при вимкненні живлення втратити мало."""

    def __init__(self, path, header):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(header)
        self.n = 0
        self._t = time.monotonic()

    def row(self, *vals):
        self.w.writerow(vals)
        self.n += 1
        if time.monotonic() - self._t > 1.0:
            self.f.flush()
            self._t = time.monotonic()

    def close(self):
        self.f.close()


class FrameWriter(threading.Thread):
    def __init__(self, run, png_level, qsize):
        super().__init__(daemon=True)
        self.dir = os.path.join(run, "mav0", "cam0", "data")
        os.makedirs(self.dir, exist_ok=True)
        self.csv = Csv(os.path.join(run, "mav0", "cam0", "data.csv"),
                       ["#timestamp [ns]", "filename"])
        self.q = queue.Queue(maxsize=qsize)
        self.params = [cv2.IMWRITE_PNG_COMPRESSION, png_level]
        self.n_dropped = 0
        self.n_written = 0
        self.shape = None
        self.stop_flag = threading.Event()

    def push(self, t_ns, img):
        try:
            self.q.put_nowait((t_ns, img))
        except queue.Full:
            self.n_dropped += 1        # диск не встигає: краще пропуск, ніж зсув часу

    def run(self):
        while not (self.stop_flag.is_set() and self.q.empty()):
            try:
                t_ns, img = self.q.get(timeout=0.1)
            except queue.Empty:
                continue
            name = f"{t_ns}.png"
            cv2.imwrite(os.path.join(self.dir, name), img, self.params)
            self.csv.row(t_ns, name)
            self.shape = img.shape
            self.n_written += 1

    def close(self):
        self.stop_flag.set()
        self.join(timeout=30)
        self.csv.close()


def git_commit():
    try:
        return subprocess.check_output(["git", "-C", HERE, "rev-parse", "--short", "HEAD"],
                                       stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conn", default="udpin:127.0.0.1:14551")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--out", default=None, help="папка запису (за замовчуванням recordings/run_<час>)")
    ap.add_argument("--camera-config", default=os.path.join(HERE, "..", "camera", "camera_config.yaml"))
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--synthetic", action="store_true", help="синтетична камера (тест без заліза)")
    ap.add_argument("--mock-autopilot", action="store_true", help="імітувати автопілот без мережі (тест без заліза)")
    ap.add_argument("--imu-rate", type=float, default=100.0)
    ap.add_argument("--att-rate", type=float, default=30.0)
    ap.add_argument("--gps-rate", type=float, default=5.0)
    ap.add_argument("--png-level", type=int, default=1, help="0-9; менше = швидше, більше файлів")
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = до Ctrl+C")
    ap.add_argument("--note", default="", help="коментар до запису (сцена, висота, погода)")
    args = ap.parse_args()

    run = args.out or os.path.join("recordings", datetime.now().strftime("run_%Y%m%d_%H%M%S"))
    os.makedirs(os.path.join(run, "raw"), exist_ok=True)

    rates = {"RAW_IMU": args.imu_rate, "ATTITUDE": args.att_rate,
             "LOCAL_POSITION_NED": 10, "GPS_RAW_INT": args.gps_rate,
             "GLOBAL_POSITION_INT": 10, "DISTANCE_SENSOR": 10}

    # ---- MAVLink ----
    is_mock = args.mock_autopilot or args.conn == "mock"
    if is_mock:
        print("Використовується імітатор автопілота (MockAutopilotConnection)...")
        master = MockAutopilotConnection(rates)
        sysid, compid = 1, 1
    else:
        if not HAS_PYMAVLINK:
            print("ПОМИЛКА: модуль 'pymavlink' не встановлено.\n"
                  "Встановіть залежності командою:\n"
                  "    pip install pymavlink opencv-python pyyaml\n"
                  "Або запустіть із прапорцем --mock-autopilot для тесту без заліза.")
            sys.exit(1)
        master = mavutil.mavlink_connection(args.conn, baud=args.baud)
        print("Чекаю heartbeat...")
        master.wait_heartbeat(timeout=30)
        sysid, compid = master.target_system, master.target_component
        print(f"Підключено: system {sysid}")

    def request_streams():
        if is_mock or not HAS_PYMAVLINK:
            return
        for name, hz in rates.items():
            master.mav.command_long_send(
                sysid, compid, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                MSG_IDS[name], 1e6 / hz, 0, 0, 0, 0, 0)

    request_streams()
    last_req = time.monotonic()

    # ---- файли ----
    imu = Csv(os.path.join(run, "raw", "imu_raw.csv"),
              ["t_ap_ns", "t_rx_ns", "gx", "gy", "gz", "ax", "ay", "az"])
    att = Csv(os.path.join(run, "raw", "attitude_raw.csv"),
              ["t_ap_ns", "t_rx_ns", "roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"])
    gps = Csv(os.path.join(run, "mav0", "gps0", "data.csv"),
              ["#t_rx_ns", "lat_deg", "lon_deg", "alt_m", "eph", "epv", "vel_ms",
               "fix_type", "satellites"])
    gpos = Csv(os.path.join(run, "global_position.csv"),
               ["t_rx_ns", "lat_deg", "lon_deg", "alt_m", "rel_alt_m", "vx", "vy", "vz", "hdg_deg"])
    loc = Csv(os.path.join(run, "local_position.csv"),
              ["t_rx_ns", "x", "y", "z", "vx", "vy", "vz"])
    dist = Csv(os.path.join(run, "distance.csv"),
               ["t_rx_ns", "distance_m", "min_m", "max_m"])
    events = open(os.path.join(run, "events.log"), "w", encoding="utf-8")

    # ---- камера ----
    writer = cam_thread = None
    cam_cfg = None
    if not args.no_camera:
        cam_cfg = load_config(args.camera_config)
        if args.synthetic:
            cam_cfg["backend"] = "synthetic"
        cam = Camera(cam_cfg)
        writer = FrameWriter(run, args.png_level, qsize=int(cam_cfg.get("fps", 30) * 2))
        writer.start()
        cam_thread = CaptureThread(cam, writer.push)
        cam_thread.start()

    t_start_ns = time.monotonic_ns()
    wall_start = datetime.now().astimezone().isoformat()
    cnt = dict.fromkeys(MSG_IDS, 0)
    tot = dict.fromkeys(MSG_IDS, 0)
    last_frames = 0
    t_rep = time.monotonic()
    t_end = time.monotonic() + args.seconds if args.seconds > 0 else None
    print(f"Запис у {run}. Зупинка: Ctrl+C")

    try:
        while t_end is None or time.monotonic() < t_end:
            msg = master.recv_match(type=list(MSG_IDS) + ["STATUSTEXT"], blocking=True, timeout=0.05)
            t_rx = time.monotonic_ns()
            now = time.monotonic()
            if now - last_req >= 3.0:
                request_streams()       # MAVProxy/GCS можуть збити частоти
                last_req = now
            if msg is not None and msg.get_srcSystem() == sysid:
                ty = msg.get_type()
                if ty in ("RAW_IMU", "SCALED_IMU"):
                    # як у vio_ekf.py: acc мілі-g, gyro мілі-рад/с
                    imu.row(msg.time_usec * 1000, t_rx,
                            msg.xgyro / 1000.0, msg.ygyro / 1000.0, msg.zgyro / 1000.0,
                            msg.xacc / 1000.0 * G, msg.yacc / 1000.0 * G, msg.zacc / 1000.0 * G)
                elif ty == "ATTITUDE":
                    att.row(msg.time_boot_ms * 1_000_000, t_rx, msg.roll, msg.pitch, msg.yaw,
                            msg.rollspeed, msg.pitchspeed, msg.yawspeed)
                elif ty == "GPS_RAW_INT":
                    gps.row(t_rx, msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3,
                            msg.eph, msg.epv, msg.vel * 1e-2, msg.fix_type, msg.satellites_visible)
                elif ty == "GLOBAL_POSITION_INT":
                    gpos.row(t_rx, msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3,
                             msg.relative_alt * 1e-3, msg.vx * 1e-2, msg.vy * 1e-2,
                             msg.vz * 1e-2, msg.hdg * 1e-2)
                elif ty == "LOCAL_POSITION_NED":
                    loc.row(t_rx, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
                elif ty == "DISTANCE_SENSOR":
                    dist.row(t_rx, msg.current_distance / 100.0,
                             msg.min_distance / 100.0, msg.max_distance / 100.0)
                elif ty == "STATUSTEXT":
                    text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
                    events.write(f"{t_rx} {text}\n")
                    events.flush()
                    print("  [autopilot]", text)
                if ty in cnt:
                    cnt[ty] += 1
                    tot[ty] += 1
            if now - t_rep >= 1.0:
                fw = writer.n_written if writer else 0
                print(f"IMU {cnt['RAW_IMU']}/с  ATT {cnt['ATTITUDE']}/с  "
                      f"GPS {cnt['GPS_RAW_INT']}/с  кадрів {fw - last_frames}/с "
                      f"(втрачено {writer.n_dropped if writer else 0})")
                if cnt["RAW_IMU"] < 0.5 * args.imu_rate:
                    print(f"УВАГА: IMU лише {cnt['RAW_IMU']} Гц при запиті {args.imu_rate:.0f}. "
                          "Для Kalibr бажано >=200 Гц; див. dataset/README.md")
                last_frames = fw
                cnt = dict.fromkeys(MSG_IDS, 0)
                t_rep = now
    except KeyboardInterrupt:
        print("\nЗупиняю...")
    finally:
        if cam_thread:
            cam_thread.stop()
        if writer:
            writer.close()
        for c in (imu, att, gps, gpos, loc, dist):
            c.close()
        events.close()

    duration = (time.monotonic_ns() - t_start_ns) * 1e-9
    meta = {
        "format": "euroc-like, див. dataset/README.md",
        "start_wall": wall_start, "duration_s": duration, "note": args.note,
        "clock": "CLOCK_MONOTONIC (time.monotonic_ns), нс; час старту запису: %d" % t_start_ns,
        "host": platform.node(), "machine": platform.machine(), "git_commit": git_commit(),
        "mavlink_conn": args.conn, "requested_rates_hz": rates, "message_totals": tot,
        "camera_config": cam_cfg,
        "frames_written": writer.n_written if writer else 0,
        "frames_dropped": writer.n_dropped if writer else 0,
        "frame_shape": list(writer.shape) if writer and writer.shape else None,
    }
    try:
        res = timesync.process_run(run)
        meta["timesync"] = {k: v["stats"] for k, v in res.items()}
        for k, v in res.items():
            s = v["stats"]
            print(f"синхронізація {k}: {s['n']} повідомлень, дрейф {s['drift_ppm']:.1f} ppm, "
                  f"затримка лінії p95 {s['latency_p95_ms']:.1f} мс")
    except Exception as e:                       # запис не втрачаємо через пост-обробку
        print(f"УВАГА: синхронізація не вдалась ({e}); raw/ збережено, повтори: python3 timesync.py {run}")
        meta["timesync_error"] = str(e)
    with open(os.path.join(run, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False, default=str)
    print(f"Готово: {run}  (кадрів {meta['frames_written']}, IMU {tot['RAW_IMU']})")


if __name__ == "__main__":
    main()
