#!/usr/bin/env python3
"""Перевірка живого потоку Gazebo (ROS 2: /camera/image_raw, /imu, /clock, odometry).

Що нового порівняно зі старою версією:
  * БЕЗ mock-fallback: якщо немає rclpy, скрипт завершується з кодом 2 (а не друкує вигадані числа).
    Mock працює ТІЛЬКИ з явним --mock і чесно підписаний як MOCK.
  * Частоти і за sim-часом (header.stamp), і за wall-часом.
  * Максимальні паузи між повідомленнями (wall і sim), перцентиль p99, кількість пауз > 200/500 мс.
  * Real-time factor з /clock (sim-секунди / wall-секунди).
  * Перевірка /model/camera_drone/odometry (критерій Фази 1: пауза <= 500 мс).
  * --json для збереження доказів; --analyze-recording для перевірки запису gazebo_recorder.py.

Використання (всередині контейнера з ROS 2, при запущених gz sim + ros_gz_bridge):
    python3 gazebo/check_gazebo_stream.py --duration 60 --json out/stream.json
    python3 gazebo/check_gazebo_stream.py --analyze-recording recordings/run1 --json out/rec.json
    python3 gazebo/check_gazebo_stream.py --mock --duration 2     # лише для CI без Gazebo

Коди виходу: 0 = PASS (або MOCK), 1 = FAIL, 2 = не вдалося виконати живу перевірку.
"""
import argparse
import csv
import json
import os
import sys
import time

import numpy as np

try:
    import rclpy
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import Image, Imu
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

CAM_TOPIC = "/camera/image_raw"
IMU_TOPIC = "/imu"
CLOCK_TOPIC = "/clock"
ODOM_TOPIC_DEFAULT = "/model/camera_drone/odometry"

# пороги: (мін Гц, макс Гц, макс пауза між повідомленнями, с)
LIMITS = {
    "camera": (25.0, 35.0, 0.2),
    "imu": (175.0, 225.0, 0.1),
    "odom": (0.0, 1e9, 0.5),      # критерій Фази 1: пауза Odometry <= 500 мс
    "clock": (0.0, 1e9, 0.5),
}
MIN_RTF = 0.9


# ───────────────────────────── аналіз ─────────────────────────────
def analyze_stream(wall, sim):
    """wall, sim — списки часів (с). Повертає словник статистик (без прив'язки до ROS)."""
    wall = np.asarray(wall, dtype=float)
    sim = np.asarray(sim, dtype=float)
    n = len(wall)
    res = {"count": n}
    if n < 2:
        return res
    gw = np.diff(wall)
    gs = np.diff(sim)
    wspan = wall[-1] - wall[0]
    sspan = sim[-1] - sim[0]
    res.update({
        "wall_hz": (n - 1) / wspan if wspan > 0 else 0.0,
        "sim_hz": (n - 1) / sspan if sspan > 0 else 0.0,
        "max_gap_wall_s": float(gw.max()),
        "p99_gap_wall_s": float(np.percentile(gw, 99)),
        "max_gap_sim_s": float(gs.max()),
        "gaps_wall_gt_200ms": int((gw > 0.2).sum()),
        "gaps_wall_gt_500ms": int((gw > 0.5).sum()),
        "jitter_wall_ms": float(gw.std() * 1e3),
        "stamp_backwards": int((gs < 0).sum()),
        "stamp_duplicates": int((gs == 0).sum()),
        "sim_span_s": float(sspan),
        "wall_span_s": float(wspan),
    })
    return res


def evaluate(stats, meta):
    """Повертає список (назва, ok, деталі)."""
    checks = []

    def add(name, ok, detail):
        checks.append((name, bool(ok), detail))

    for key, label in (("camera", "Камера"), ("imu", "IMU")):
        s = stats.get(key, {})
        lo, hi, max_gap = LIMITS[key]
        if s.get("count", 0) < 2:
            add(f"{label}: дані", False, f"отримано {s.get('count', 0)} повідомлень")
            continue
        add(f"{label}: частота wall", lo <= s["wall_hz"] <= hi, f"{s['wall_hz']:.1f} Гц (норма {lo:.0f}-{hi:.0f})")
        add(f"{label}: частота sim", lo <= s["sim_hz"] <= hi, f"{s['sim_hz']:.1f} Гц (норма {lo:.0f}-{hi:.0f})")
        add(f"{label}: макс. пауза wall", s["max_gap_wall_s"] <= max_gap,
            f"{s['max_gap_wall_s']*1e3:.0f} мс (ліміт {max_gap*1e3:.0f} мс)")
        add(f"{label}: штампи монотонні", s["stamp_backwards"] == 0,
            f"назад: {s['stamp_backwards']}, дублікатів: {s['stamp_duplicates']}")
    add("Камера: роздільність", meta.get("cam_res") == "640x480", f"{meta.get('cam_res')} ({meta.get('cam_enc')})")

    o = stats.get("odom", {})
    if o.get("count", 0) < 2:
        add("Odometry: дані", False, f"отримано {o.get('count', 0)} повідомлень (чи є в bridge odometry?)")
    else:
        lim = LIMITS["odom"][2]
        add("Odometry: макс. пауза wall", o["max_gap_wall_s"] <= lim,
            f"{o['max_gap_wall_s']*1e3:.0f} мс (ліміт {lim*1e3:.0f} мс); sim: {o['max_gap_sim_s']*1e3:.0f} мс; "
            f"частота wall {o['wall_hz']:.1f} Гц")

    c = stats.get("clock", {})
    if c.get("count", 0) < 2:
        add("/clock", False, "не отримано (немає /clock у bridge?)")
    else:
        rtf = c["sim_span_s"] / c["wall_span_s"] if c["wall_span_s"] > 0 else 0.0
        meta["rtf"] = rtf
        add("/clock: активний, штампи монотонні", c["stamp_backwards"] == 0, f"{c['count']} повідомлень, назад: {c['stamp_backwards']}")
        add("Real-time factor", rtf >= MIN_RTF, f"{rtf:.3f} (норма >= {MIN_RTF})")
    return checks


def print_report(title, checks, verdict_label):
    print("\n" + "=" * 78)
    print(f"  {title}")
    print("=" * 78)
    for name, ok, detail in checks:
        print(f"{'[PASS]' if ok else '[FAIL]'} {name:<36} {detail}")
    print("=" * 78)
    print(f">>> ВЕРДИКТ: {verdict_label}\n")


# ───────────────────────────── ROS 2 збір ─────────────────────────────
def run_live(duration, warmup, odom_topic, startup_timeout=45.0):
    rclpy.init()
    node = rclpy.create_node("gazebo_stream_checker")
    qos = QoSProfile(depth=2000, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    data = {k: {"wall": [], "sim": []} for k in ("camera", "imu", "odom", "clock")}
    meta = {"cam_res": "N/A", "cam_enc": "N/A"}

    def stamp(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    def on_image(m):
        data["camera"]["wall"].append(time.monotonic()); data["camera"]["sim"].append(stamp(m.header))
        meta["cam_res"] = f"{m.width}x{m.height}"; meta["cam_enc"] = m.encoding

    def on_imu(m):
        data["imu"]["wall"].append(time.monotonic()); data["imu"]["sim"].append(stamp(m.header))

    def on_odom(m):
        data["odom"]["wall"].append(time.monotonic()); data["odom"]["sim"].append(stamp(m.header))

    def on_clock(m):
        data["clock"]["wall"].append(time.monotonic()); data["clock"]["sim"].append(m.clock.sec + m.clock.nanosec * 1e-9)

    node.create_subscription(Image, CAM_TOPIC, on_image, qos)
    node.create_subscription(Imu, IMU_TOPIC, on_imu, qos)
    node.create_subscription(Odometry, odom_topic, on_odom, qos)
    node.create_subscription(Clock, CLOCK_TOPIC, on_clock, qos)

    print(f"Очікування перших повідомлень (до {startup_timeout:.0f} с)...")
    t0 = time.monotonic()
    t_first = None
    while time.monotonic() - t0 < startup_timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        if data["camera"]["wall"] and data["imu"]["wall"]:
            t_first = time.monotonic()
            break
    if t_first is None:
        node.destroy_node(); rclpy.shutdown()
        return None, meta
    print(f"Прогрів {warmup:.0f} с, далі вимірювання {duration:.0f} с...")
    t_meas_start = t_first + warmup
    t_end = t_meas_start + duration
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node(); rclpy.shutdown()

    stats = {}
    for k, v in data.items():
        idx = [i for i, w in enumerate(v["wall"]) if w >= t_meas_start]
        stats[k] = analyze_stream([v["wall"][i] for i in idx], [v["sim"][i] for i in idx])
    return stats, meta


def run_mock(duration):
    print(f"[MOCK MODE] Це НЕ жива перевірка. Дані вигадані ({duration} с).")
    stats = {
        "camera": {"count": int(duration * 30), "wall_hz": 30.0, "sim_hz": 30.0, "max_gap_wall_s": 0.04, "max_gap_sim_s": 0.034,
                   "p99_gap_wall_s": 0.036, "gaps_wall_gt_200ms": 0, "gaps_wall_gt_500ms": 0, "jitter_wall_ms": 1.2,
                   "stamp_backwards": 0, "stamp_duplicates": 0, "sim_span_s": duration, "wall_span_s": duration},
        "imu": {"count": int(duration * 200), "wall_hz": 200.0, "sim_hz": 200.0, "max_gap_wall_s": 0.01, "max_gap_sim_s": 0.005,
                "p99_gap_wall_s": 0.006, "gaps_wall_gt_200ms": 0, "gaps_wall_gt_500ms": 0, "jitter_wall_ms": 0.4,
                "stamp_backwards": 0, "stamp_duplicates": 0, "sim_span_s": duration, "wall_span_s": duration},
        "odom": {"count": int(duration * 50), "wall_hz": 50.0, "sim_hz": 50.0, "max_gap_wall_s": 0.03, "max_gap_sim_s": 0.02,
                 "p99_gap_wall_s": 0.02, "gaps_wall_gt_200ms": 0, "gaps_wall_gt_500ms": 0, "jitter_wall_ms": 1.0,
                 "stamp_backwards": 0, "stamp_duplicates": 0, "sim_span_s": duration, "wall_span_s": duration},
        "clock": {"count": int(duration * 100), "wall_hz": 100.0, "sim_hz": 100.0, "max_gap_wall_s": 0.02, "max_gap_sim_s": 0.01,
                  "p99_gap_wall_s": 0.01, "gaps_wall_gt_200ms": 0, "gaps_wall_gt_500ms": 0, "jitter_wall_ms": 1.0,
                  "stamp_backwards": 0, "stamp_duplicates": 0, "sim_span_s": duration, "wall_span_s": duration},
    }
    return stats, {"cam_res": "640x480", "cam_enc": "rgb8"}


# ───────────────────────────── аналіз запису ─────────────────────────────
def _read_ts(csv_path):
    ts = []
    with open(csv_path, newline="") as f:
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            ts.append(int(row[0]) * 1e-9)
    return np.asarray(ts)


def analyze_recording(run_dir):
    """Перевірка датасету від gazebo_recorder.py: повнота, паузи, відповідність PNG ↔ CSV (sim-час)."""
    cam_csv = os.path.join(run_dir, "mav0", "cam0", "data.csv")
    imu_csv = os.path.join(run_dir, "mav0", "imu0", "data.csv")
    cam_dir = os.path.join(run_dir, "mav0", "cam0", "data")
    for p in (cam_csv, imu_csv):
        if not os.path.isfile(p):
            return [("Файл запису", False, f"немає {p}")], {}
    cam, imu = _read_ts(cam_csv), _read_ts(imu_csv)
    checks, summary = [], {}
    for name, ts, hz, max_gap in (("Камера", cam, 30.0, 0.2), ("IMU", imu, 200.0, 0.1)):
        if len(ts) < 2:
            checks.append((f"{name}: записи", False, f"лише {len(ts)} записів")); continue
        span = ts[-1] - ts[0]
        expected = span * hz + 1
        ratio = len(ts) / expected
        gap = float(np.diff(ts).max())
        back = int((np.diff(ts) <= 0).sum())
        summary[name] = {"records": len(ts), "sim_span_s": float(span), "expected": float(expected), "ratio": float(ratio),
                         "max_gap_sim_s": gap, "non_increasing": back}
        checks.append((f"{name}: повнота", ratio >= 0.95, f"{len(ts)} з ~{expected:.0f} очікуваних ({ratio*100:.1f}%) за {span:.1f} sim-с"))
        checks.append((f"{name}: макс. пауза sim", gap <= max_gap, f"{gap*1e3:.0f} мс (ліміт {max_gap*1e3:.0f} мс)"))
        checks.append((f"{name}: штампи зростають", back == 0, f"порушень: {back}"))
    if os.path.isdir(cam_dir):
        pngs = len([f for f in os.listdir(cam_dir) if f.endswith(".png")])
        checks.append(("PNG відповідають CSV", pngs == len(cam), f"png: {pngs}, рядків CSV: {len(cam)}"))
    checks = [(n, bool(o), d) for n, o, d in checks]
    return checks, summary


# ───────────────────────────── main ─────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=5.0, help="тривалість вимірювання після прогріву (с)")
    ap.add_argument("--warmup", type=float, default=3.0, help="прогрів, дані не рахуються (с)")
    ap.add_argument("--odom-topic", default=ODOM_TOPIC_DEFAULT)
    ap.add_argument("--mock", action="store_true", help="ЯВНИЙ автономний режим без Gazebo (не доказ!)")
    ap.add_argument("--analyze-recording", metavar="DIR", help="перевірити датасет, записаний gazebo_recorder.py")
    ap.add_argument("--json", metavar="FILE", help="зберегти результат у JSON")
    args = ap.parse_args()

    if args.analyze_recording:
        checks, summary = analyze_recording(args.analyze_recording)
        ok = bool(checks) and all(c[1] for c in checks)
        print_report(f"ПЕРЕВІРКА ЗАПИСУ: {args.analyze_recording}", checks, "PASS" if ok else "FAIL")
        if args.json:
            with open(args.json, "w") as f:
                json.dump({"verdict": "PASS" if ok else "FAIL", "summary": summary,
                           "checks": [{"name": n, "ok": o, "detail": d} for n, o, d in checks]}, f, indent=2, ensure_ascii=False)
        return 0 if ok else 1

    if args.mock:
        stats, meta = run_mock(args.duration)
        mode = "MOCK"
    else:
        if not HAS_ROS2:
            print("ПОМИЛКА: rclpy/nav_msgs недоступні. Це жива перевірка — запускайте в контейнері ROS 2 "
                  "(source /opt/ros/jazzy/setup.bash). Для автономного прогону використовуйте явний --mock.")
            return 2
        stats, meta = run_live(args.duration, args.warmup, args.odom_topic)
        mode = "LIVE"
        if stats is None:
            print("[FAIL] За 45 с не надійшло жодного повідомлення камери та IMU. Gazebo/bridge запущено?")
            return 2

    checks = evaluate(stats, meta)
    ok = all(c[1] for c in checks)
    verdict = ("MOCK (НЕ жива перевірка)" if mode == "MOCK" else ("PASS" if ok else "FAIL"))
    print_report(f"ПЕРЕВІРКА ПОТОКУ GAZEBO [{mode}]", checks, verdict)
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"mode": mode, "verdict": verdict, "rtf": meta.get("rtf"), "meta": meta, "stats": stats,
                       "checks": [{"name": n, "ok": o, "detail": d} for n, o, d in checks]}, f, indent=2, ensure_ascii=False)
    return 0 if (ok or mode == "MOCK") else 1


if __name__ == "__main__":
    sys.exit(main())
