#!/usr/bin/env python3
"""Результати Kalibr -> camera_params.json (для replay/VIO) + calibration_report.md.

  python3 make_report.py --camchain imucam-camchain-imucam.yaml \
      --results imucam-results-imucam.txt --cam-results cam-results-cam.txt \
      --run ../dataset/recordings/calib_imucam --camera-name "<модель камери>"

Звіт додається до датасету (Deliverable за ТЗ). Пороги за замовчуванням - типові для
Kalibr; заміни своїми з розділу 5 ТЗ через --max-reproj-px.
"""
import argparse
import json
import math
import os
import re
from datetime import date

import numpy as np
import yaml

# Очікувана орієнтація для камери, що дивиться вниз, з монтажем із vio/vio_frontend.py:
# x кадру = права сторона дрона (body Y), y кадру = назад (-body X), z (оптична вісь) = вниз.
# v_cam = R_exp @ v_body (FRD).
R_EXPECTED = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
NUM = r"([-+]?\d+\.?\d*(?:[eE][-+]?\d+)?)"


def parse_camchain(path, cam="cam0"):
    with open(path, encoding="utf-8") as f:
        c = yaml.safe_load(f)[cam]
    fx, fy, cx, cy = c["intrinsics"]
    return {
        "camera_model": c["camera_model"], "distortion_model": c["distortion_model"],
        "fx": fx, "fy": fy, "cx": cx, "cy": cy, "dist": list(c["distortion_coeffs"]),
        "width": int(c["resolution"][0]), "height": int(c["resolution"][1]),
        "T_cam_imu": c["T_cam_imu"] if "T_cam_imu" in c else None,
        "timeshift_cam_imu_s": float(c.get("timeshift_cam_imu", 0.0)),
        "rostopic": c.get("rostopic"),
    }


def parse_results(path):
    """Достає похибки з results-*.txt. Формат різниться між версіями Kalibr, тому
    шукаємо за шаблонами; чого не знайшли - повертаємо як None."""
    out = {}
    if not path or not os.path.exists(path):
        return out
    txt = open(path, encoding="utf-8", errors="ignore").read()
    m = re.search(r"Reprojection error \(cam0\)[^:]*:\s*mean\s+" + NUM + r",\s*median\s+" + NUM
                  + r",\s*std:\s*" + NUM, txt)
    if m:
        out["reproj_mean_px"], out["reproj_median_px"], out["reproj_std_px"] = map(float, m.groups())
    else:   # результати лише камери: "reprojection error: [x, y] +- [sx, sy]"
        m = re.search(r"reprojection error:\s*\[" + NUM + r",\s*" + NUM + r"\]\s*\+-\s*\["
                      + NUM + r",\s*" + NUM + r"\]", txt)
        if m:
            ex, ey, sx, sy = map(float, m.groups())
            out["reproj_mean_px"] = math.hypot(ex, ey)
            out["reproj_std_px"] = math.hypot(sx, sy)
    for key, name in (("gyro", "Gyroscope"), ("acc", "Accelerometer")):
        m = re.search(name + r" error \(imu0\)[^:]*:\s*mean\s+" + NUM + r",\s*median\s+" + NUM
                      + r",\s*std:\s*" + NUM, txt)
        if m:
            out[f"{key}_err_mean"], out[f"{key}_err_median"], out[f"{key}_err_std"] = map(float, m.groups())
    return out


def mount_check(T):
    if T is None:
        return None
    R = np.array(T)[:3, :3]
    c = (np.trace(R_EXPECTED.T @ R) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camchain", required=True, help="*camchain-imucam.yaml (або camchain.yaml без IMU)")
    ap.add_argument("--results", help="*results-imucam.txt")
    ap.add_argument("--cam-results", help="*results-cam.txt (калібрування лише камери)")
    ap.add_argument("--run", help="папка запису калібрування (для meta.json)")
    ap.add_argument("--camera-name", default="<модель камери>")
    ap.add_argument("--target", default="aprilgrid.yaml")
    ap.add_argument("--max-reproj-px", type=float, default=1.0)
    ap.add_argument("--max-mount-deg", type=float, default=10.0)
    ap.add_argument("--out-dir", default=".")
    args = ap.parse_args()

    p = parse_camchain(args.camchain)
    res = parse_results(args.results)
    cam_res = parse_results(args.cam_results)
    ang = mount_check(p["T_cam_imu"])
    hfov = math.degrees(2 * math.atan(p["width"] / (2 * p["fx"])))
    vfov = math.degrees(2 * math.atan(p["height"] / (2 * p["fy"])))
    params = dict(p, **{"hfov_deg": hfov, "vfov_deg": vfov, "residuals": res,
                        "residuals_cam_only": cam_res, "mount_deviation_deg": ang})
    os.makedirs(args.out_dir, exist_ok=True)
    with open(os.path.join(args.out_dir, "camera_params.json"), "w", encoding="utf-8") as f:
        json.dump(params, f, indent=2)

    meta = {}
    if args.run and os.path.exists(os.path.join(args.run, "meta.json")):
        meta = json.load(open(os.path.join(args.run, "meta.json"), encoding="utf-8"))
    ts = (meta.get("timesync") or {}).get("imu")

    def fmt(v, f="{:.3f}"):
        return f.format(v) if v is not None else "не знайдено у results.txt"

    t = p["T_cam_imu"]
    tr = np.array(t)[:3, 3] * 1000.0 if t else None
    reproj = res.get("reproj_mean_px", cam_res.get("reproj_mean_px"))
    checks = [
        (f"Середня похибка репроєкції <= {args.max_reproj_px} px",
         None if reproj is None else reproj <= args.max_reproj_px),
        (f"Монтаж збігається з припущенням vio_frontend.py (відхилення <= {args.max_mount_deg} град)",
         None if ang is None else ang <= args.max_mount_deg),
        ("Частота IMU >= 150 Гц", None if not ts else ts.get("rate_hz", 0) >= 150),
        ("Втрачених кадрів при записі немає", None if not meta else meta.get("frames_dropped", 0) == 0),
    ]
    mark = {True: "ТАК", False: "НІ", None: "не перевірено"}
    L = []
    L.append(f"# Звіт про калібрування камери та камера-IMU\n")
    L.append(f"Дата: {date.today().isoformat()}  \nКамера: {args.camera_name}  \n"
             f"Мішень: {args.target}  \nКомпʼютер запису: {meta.get('host', '—')} ({meta.get('machine', '—')}), "
             f"коміт: {meta.get('git_commit') or '—'}\n")
    L.append("## 1. Внутрішні параметри\n")
    L.append(f"| | |\n|---|---|\n| Модель | {p['camera_model']} + {p['distortion_model']} |\n"
             f"| Роздільність | {p['width']}x{p['height']} px |\n"
             f"| fx, fy | {p['fx']:.2f}, {p['fy']:.2f} px |\n| cx, cy | {p['cx']:.2f}, {p['cy']:.2f} px |\n"
             f"| Дисторсія | {', '.join(f'{d:.6f}' for d in p['dist'])} |\n"
             f"| Поле зору (гор. x верт.) | {hfov:.1f} x {vfov:.1f} град |\n")
    L.append(f"Для `vio_ekf.py`: `--fx {p['fx']:.1f}` (дійсно лише для кадрів {p['width']}x{p['height']} "
             f"і без дисторсії - див. `Dataset(undistort=True)` у `dataset_reader.py`).\n")
    L.append("## 2. Залишкові похибки\n")
    L.append("| Величина | Середнє | Медіана | СКВ (std) |\n|---|---|---|---|")
    for lab, d, k, u in (("Репроєкція (камера+IMU), px", res, "reproj", ""),
                         ("Репроєкція (лише камера), px", cam_res, "reproj", ""),
                         ("Гіроскоп, рад/с", res, "gyro_err", ""), ("Акселерометр, м/с²", res, "acc_err", "")):
        if k == "reproj":
            if "reproj_mean_px" in d:
                L.append(f"| {lab} | {fmt(d.get('reproj_mean_px'))} | {fmt(d.get('reproj_median_px'))} | "
                         f"{fmt(d.get('reproj_std_px'))} |")
        elif f"{k}_mean" in d:
            L.append(f"| {lab} | {fmt(d.get(k + '_mean'))} | {fmt(d.get(k + '_median'))} | {fmt(d.get(k + '_std'))} |")
    L.append("\nГрафіки залишків — у `*report*.pdf`, що згенерував Kalibr (додати до датасету).\n")
    L.append("## 3. Взаємне розташування камера–IMU\n")
    if t:
        L.append("`T_cam_imu` (IMU → камера, осі IMU = FRD автопілота):\n\n```")
        L += ["  ".join(f"{v: .6f}" for v in row) for row in t]
        L.append("```\n")
        L.append(f"Зсув центру камери відносно IMU: x={tr[0]:.1f}, y={tr[1]:.1f}, z={tr[2]:.1f} мм. "
                 f"Відхилення від очікуваного монтажу (кадр x = праворуч, y = назад, z = вниз): "
                 f"{ang:.1f} град.\n")
    else:
        L.append("Камера–IMU калібрування не виконувалось (немає `T_cam_imu`).\n")
    L.append("## 4. Синхронізація часу\n")
    L.append(f"- Зсув камера–IMU (Kalibr): **{p['timeshift_cam_imu_s'] * 1000:.2f} мс** "
             f"(t_imu = t_cam + зсув; застосовується в `Dataset(..., camera_params=...)`).")
    if ts:
        L.append(f"- Метод: програмний, нижня огинаюча затримки між годинниками автопілота й компʼютера "
                 f"(`dataset/timesync.py`). Дрейф {ts['drift_ppm']:.1f} ppm, затримка лінії: медіана "
                 f"{ts['latency_median_ms']:.1f} мс, p95 {ts['latency_p95_ms']:.1f} мс.")
        L.append(f"- Фактична частота IMU у записі: {ts.get('rate_hz', 0):.0f} Гц.")
    else:
        L.append("- Статистика синхронізації: передай `--run` з папкою запису.")
    L.append("\n## 5. Автоматична перевірка\n")
    L.append("| Критерій | Виконано |\n|---|---|")
    for lab, ok in checks:
        L.append(f"| {lab} | {mark[ok]} |")
    L.append("\n## 6. Умови й обмеження (заповнити вручну)\n")
    L.append("- Освітлення, експозиція, відстань до мішені:\n- Кількість кадрів / тривалість запису:\n"
             "- Чи збудили всі осі IMU (рух по всіх 6 ступенях свободи):\n- Температура IMU/компʼютера:\n"
             "- Відомі проблеми:\n")
    with open(os.path.join(args.out_dir, "calibration_report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    print("Записано camera_params.json і calibration_report.md у", os.path.abspath(args.out_dir))
    for lab, ok in checks:
        print(f"  [{mark[ok]:>13}] {lab}")


if __name__ == "__main__":
    main()
