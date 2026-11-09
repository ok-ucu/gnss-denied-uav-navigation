"""Тести для модуля dataset: записувач, читач, синхронізація часу."""
import os
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from dataset_reader import Dataset
import timesync

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
except ImportError:
    mavutil = None
    HAS_PYMAVLINK = False

PORT = 14991
CLOCK_OFFSET_S = 1234.5      # автопілот: свій годинник зі зсувом
DRIFT = 30e-6                # і дрейфом 30 ppm


def test_timesync_fit_clock():
    """Тест алгоритму оцінки дрейфу та зсуву часу методом нижньої огинаючої."""
    n_pts = 1000
    t_ap = np.linspace(1e9, 10e9, n_pts, dtype=np.int64)
    drift = 25e-6
    offset = 500e9
    rng = np.random.default_rng(123)
    jitter = rng.uniform(1e6, 5e6, size=n_pts)
    t_rx = (t_ap * (1.0 + drift) + offset + jitter).astype(np.int64)

    model, stats = timesync.fit_clock(t_ap, t_rx, win_s=2.0)
    assert abs(stats["drift_ppm"] - 25.0) < 5.0
    assert stats["latency_median_ms"] > 0.0

    t_est = timesync.apply_clock(t_ap, model)
    assert len(t_est) == n_pts
    assert np.all(np.diff(t_est) > 0)


def test_recorder_mock_autopilot(tmp_path):
    """Тест запису через вбудований mock-генератор повідомлень (без реального MAVLink)."""
    out = tmp_path / "run_mock"
    cmd = [
        sys.executable,
        os.path.join(HERE, "recorder.py"),
        "--mock-autopilot",
        "--synthetic",
        "--seconds", "4",
        "--imu-rate", "100",
        "--out", str(out),
    ]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stdout + r.stderr

    ds = Dataset(str(out))
    imu = ds.imu()
    assert len(imu) > 100
    assert np.all(np.diff(imu[:, 0]) > 0)

    ft = ds.frame_times()
    assert len(ft) > 60

    gps_ned = ds.gps_local_ned()
    assert len(gps_ned) > 5
    assert np.all(gps_ned[:, 0] >= 0.0)

    # Перевірка експорту в TUM-формат (для Людини 4)
    tum_path = str(tmp_path / "gps_tum.txt")
    ds.export_tum_trajectory(tum_path, source="gps")
    assert os.path.exists(tum_path)
    with open(tum_path) as f:
        lines = f.readlines()
        assert len(lines) > 5
        assert lines[0].startswith("# timestamp")

    # Перевірка переліку подій
    kinds = set()
    for t, kind, _ in ds.events():
        kinds.add(kind)
    assert "imu" in kinds
    assert "frame" in kinds


@pytest.mark.skipif(not HAS_PYMAVLINK, reason="потрібен встановлений pymavlink для тесту по UDP")
def test_record_and_read_udp(tmp_path):
    """Тест з фейковим автопілотом по UDP сокету."""
    stop = threading.Event()

    def fake_autopilot():
        m = mavutil.mavlink_connection(f"udpout:127.0.0.1:{PORT}", source_system=1)
        t0 = time.monotonic()
        rng = np.random.default_rng(1)
        last_hb = last_att = last_gps = 0.0
        n = 0
        while not stop.is_set():
            now = time.monotonic()
            t_ap = (now - t0) * (1 + DRIFT) + CLOCK_OFFSET_S
            if now - last_hb > 0.5:
                m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                     mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
                last_hb = now
            time.sleep(0.01)
            jitter = rng.uniform(0, 0.005)
            time.sleep(jitter)
            ph = n * 0.01
            m.mav.raw_imu_send(int(t_ap * 1e6), int(np.sin(ph) * 100), 0, -1000,
                               int(np.cos(ph) * 50), 0, 0, 0, 0, 0)
            n += 1
            if now - last_att > 0.033:
                m.mav.attitude_send(int(t_ap * 1e3) & 0xFFFFFFFF, 0.01, 0.02, ph, 0, 0, 0)
                last_att = now
            if now - last_gps > 0.2:
                m.mav.gps_raw_int_send(int(t_ap * 1e6), 3, int((49.0 + n * 1e-7) * 1e7),
                                       int(24.0 * 1e7), 300000, 100, 100, 0, 0, 10)
                last_gps = now

    th = threading.Thread(target=fake_autopilot, daemon=True)
    th.start()
    out = tmp_path / "run_udp"
    cmd = [
        sys.executable,
        os.path.join(HERE, "recorder.py"),
        "--conn", f"udpin:127.0.0.1:{PORT}",
        "--out", str(out),
        "--synthetic",
        "--seconds", "6",
        "--imu-rate", "100"
    ]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    stop.set()
    assert r.returncode == 0, r.stdout + r.stderr

    ds = Dataset(str(out))
    assert len(ds.imu()) > 150
    assert len(ds.frame_times()) > 50
    assert "кадри" in ds.summary()
