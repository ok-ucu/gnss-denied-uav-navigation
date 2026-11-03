#!/usr/bin/env python3
"""Перевірка вимог ТЗ за логом vio_log.csv: частота, затримка, узгодженість covariance (NIS).

    python analyze_log.py vio_log.csv [--dof 2]

NIS (normalized innovation squared) для чесної моделі має бути ~ chi^2 з dof ступенями
свободи: середнє ~ dof, а ~95% значень нижче 95-го перцентиля chi^2."""
import argparse
import csv
import math

import numpy as np

CHI2_95 = {1: 3.84, 2: 5.99, 3: 7.81}


def col(rows, name):
    out = []
    for r in rows:
        try:
            v = float(r[name])
        except (KeyError, ValueError):
            continue
        if not math.isnan(v):
            out.append(v)
    return np.array(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--dof", type=int, default=2, help="2 для камери-потоку (vx,vy), 3 для SimVisualSensor")
    ap.add_argument("--max-latency-ms", type=float, default=50.0)
    ap.add_argument("--min-rate", type=float, default=10.0)
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.log)))
    ok_all = True

    t = col(rows, "t")
    rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 2 else 0.0
    good = rate >= a.min_rate
    ok_all &= good
    print(f"[{'OK' if good else 'FAIL'}] частота виходу {rate:.1f} Гц (треба >= {a.min_rate:.0f})")

    lat = col(rows, "latency_ms")
    if len(lat):
        p95 = float(np.percentile(lat, 95))
        good = p95 <= a.max_latency_ms
        ok_all &= good
        print(f"[{'OK' if good else 'FAIL'}] затримка кадр->MAVLink: сер {lat.mean():.1f}, "
              f"p95 {p95:.1f}, макс {lat.max():.1f} мс (треба <= {a.max_latency_ms:.0f})")
    else:
        print("[--] немає вимірів затримки")

    nis = col(rows, "nis")
    nis = nis[nis > 0]
    if len(nis) >= 30:
        thr = CHI2_95[a.dof]
        frac = float(np.mean(nis < thr))
        mean = float(nis.mean())
        overconfident = mean > 2.0 * a.dof or frac < 0.9      # небезпечно: фільтр надто впевнений
        conservative = mean < 0.5 * a.dof                      # безпечно, але неточно
        ok_all &= not overconfident
        tag = "FAIL" if overconfident else ("WARN" if conservative else "OK")
        verdict = ("covariance ЗАНИЖЕНА (надто впевнений) - збільш sigma_vis/sigma_acc" if overconfident
                   else "covariance завищена (обережно; на ідеальних/безшумних даних це нормально)"
                   if conservative else "узгоджена")
        print(f"[{tag}] NIS: сер {mean:.2f} (очікується ~{a.dof}), "
              f"{100 * frac:.0f}% < {thr} (очікується ~95%) -> {verdict}")
    else:
        print("[--] замало NIS-вимірів (<30)")

    q = col(rows, "quality")
    sig = col(rows, "sigma_v")
    inl = col(rows, "n_inliers")
    if len(sig) > 10 and len(inl) == len(sig) and np.std(inl) > 5 and np.std(sig) > 0:
        c = float(np.corrcoef(inl, sig)[0, 1])
        print(f"[info] кореляція (кількість точок, sigma_v) = {c:.2f} (очікується від'ємна)")
    if len(q):
        print(f"[info] якість: мін {q.min():.2f}, сер {q.mean():.2f}")
    err = col(rows, "err_xy")
    if len(err):
        print(f"[info] помилка позиції відносно симулятора: сер {err.mean():.2f}, макс {err.max():.2f} м")
    raise SystemExit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
