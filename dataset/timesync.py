#!/usr/bin/env python3
"""Синхронізація часу: годинник автопілота -> годинник бортового комп'ютера.

Проблема. RAW_IMU і ATTITUDE мають мітку часу АВТОПІЛОТА (час від увімкнення),
кадри камери - мітку БОРТОВОГО комп'ютера (monotonic). Це два різні годинники:
вони зсунуті на невідому константу і трохи розходяться (дрейф, десятки ppm).

Метод (програмна синхронізація, без апаратних тригерів). Для кожного
повідомлення знаємо t_ap (мітка автопілота) і t_rx (момент прийому на
комп'ютері). Тоді t_rx - t_ap = зсув + затримка_лінії, а затримка >= 0.
Беремо МІНІМУМ у кожному вікні (нижня огинаюча) - там затримка найменша -
і проводимо крізь ці мінімуми пряму: зсув + дрейф. Похибка методу порядку
мінімальної затримки лінії (UART/UDP: 1-3 мс); постійна частина цієї
похибки далі входить у timeshift_cam_imu, який знаходить Kalibr.

Запуск повторно над записом: python3 timesync.py recordings/run_XXX
"""
import argparse
import csv
import json
import os

import numpy as np

IMU_HEADER = ("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
              "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
              "a_RS_S_z [m s^-2]")


def fit_clock(t_ap_ns, t_rx_ns, win_s=5.0):
    """Повертає модель: t_host = t_ap + a + b * (t_ap - t0) (усе в нс) і статистику."""
    t_ap = np.asarray(t_ap_ns, dtype=np.float64)
    d = np.asarray(t_rx_ns, dtype=np.float64) - t_ap
    t0 = float(t_ap[0])
    if len(t_ap) < 10:
        raise ValueError("замало повідомлень для синхронізації")
    sec = (t_ap - t0) * 1e-9
    win = np.floor(sec / win_s).astype(int)
    pts = []
    for w in np.unique(win):
        idx = np.where(win == w)[0]
        j = idx[np.argmin(d[idx])]
        pts.append((sec[j], d[j]))
    pts = np.array(pts)
    if len(pts) < 3:
        a, b = float(d.min()), 0.0                    # коротка сесія: лише константа
    else:
        x, y = pts[:, 0], pts[:, 1]
        for _ in range(2):                            # відкидаємо викиди мінімумів
            k, c = np.polyfit(x, y, 1)
            res = y - (k * x + c)
            keep = np.abs(res) < max(3.0 * 1.4826 * np.median(np.abs(res)), 2e5)
            if keep.all() or keep.sum() < 3:
                break
            x, y = x[keep], y[keep]
        k, c = np.polyfit(x, y, 1)
        a, b = float(c), float(k)                     # k: нс зсуву на 1 с => b у нс/с
    line = a + b * sec
    lat = d - line                                    # затримка відносно огинаючої
    lat -= min(lat.min(), 0.0)
    model = {"a_ns": a, "b_ns_per_s": b, "t0_ap_ns": t0}
    stats = {
        "n": int(len(t_ap)), "n_windows": int(len(pts)),
        "drift_ppm": b * 1e-3,
        "latency_median_ms": float(np.median(lat) * 1e-6),
        "latency_p95_ms": float(np.percentile(lat, 95) * 1e-6),
        "latency_max_ms": float(lat.max() * 1e-6),
    }
    return model, stats


def apply_clock(t_ap_ns, model):
    t_ap = np.asarray(t_ap_ns, dtype=np.float64)
    sec = (t_ap - model["t0_ap_ns"]) * 1e-9
    return np.round(t_ap + model["a_ns"] + model["b_ns_per_s"] * sec).astype(np.int64)


def _read_csv(path):
    with open(path, newline="") as f:
        r = csv.reader(f)
        next(r)
        rows = [[float(v) if "." in v or "e" in v.lower() else int(v) for v in row]
                for row in r if row]
    return np.array(rows, dtype=np.float64) if rows else np.zeros((0, 0))


def _clean(t_ap, arr):
    """Сортуємо за t_ap, прибираємо дублікати/нульові мітки."""
    order = np.argsort(t_ap, kind="stable")
    t_ap, arr = t_ap[order], arr[order]
    keep = np.concatenate([[True], np.diff(t_ap) > 0]) & (t_ap > 0)
    return t_ap[keep], arr[keep]


def process_run(run):
    """raw/imu_raw.csv + raw/attitude_raw.csv -> mav0/imu0/data.csv + attitude.csv."""
    out = {}
    raw_imu = os.path.join(run, "raw", "imu_raw.csv")
    if not os.path.exists(raw_imu):
        raise FileNotFoundError(raw_imu)
    m = _read_csv(raw_imu)           # t_ap_ns, t_rx_ns, gx gy gz ax ay az
    t_ap_i, m = _clean(m[:, 0].astype(np.int64).astype(np.float64), m)
    model, stats = fit_clock(t_ap_i, m[:, 1])
    t = apply_clock(t_ap_i, model)
    t = np.maximum.accumulate(t)
    keep = np.concatenate([[True], np.diff(t) > 0])
    os.makedirs(os.path.join(run, "mav0", "imu0"), exist_ok=True)
    with open(os.path.join(run, "mav0", "imu0", "data.csv"), "w") as f:
        f.write(IMU_HEADER + "\n")
        for ti, row in zip(t[keep], m[keep]):
            f.write(f"{int(ti)}," + ",".join(f"{v:.9g}" for v in row[2:8]) + "\n")
    stats["rate_hz"] = float((keep.sum() - 1) / ((t[keep][-1] - t[keep][0]) * 1e-9))
    out["imu"] = {"model": model, "stats": stats}

    raw_att = os.path.join(run, "raw", "attitude_raw.csv")
    if os.path.exists(raw_att):
        a = _read_csv(raw_att)       # t_ap_ns, t_rx_ns, roll pitch yaw rr pr yr
        if len(a) >= 10:
            t_ap_a, a = _clean(a[:, 0], a)
            m_a, s_a = fit_clock(t_ap_a, a[:, 1])
            t_a = np.maximum.accumulate(apply_clock(t_ap_a, m_a))
            with open(os.path.join(run, "attitude.csv"), "w") as f:
                f.write("t_ns,roll,pitch,yaw,rollspeed,pitchspeed,yawspeed\n")
                for ti, row in zip(t_a, a):
                    f.write(f"{int(ti)}," + ",".join(f"{v:.9g}" for v in row[2:8]) + "\n")
            out["attitude"] = {"model": m_a, "stats": s_a}
    with open(os.path.join(run, "timesync.json"), "w") as f:
        json.dump(out, f, indent=2)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    args = ap.parse_args()
    res = process_run(args.run)
    for name, r in res.items():
        s = r["stats"]
        print(f"{name}: {s['n']} повід., дрейф {s['drift_ppm']:.1f} ppm, затримка лінії "
              f"медіана {s['latency_median_ms']:.1f} мс, p95 {s['latency_p95_ms']:.1f} мс, "
              f"макс {s['latency_max_ms']:.1f} мс")


if __name__ == "__main__":
    main()
