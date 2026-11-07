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

    # --- знаки та осі: швидкість VIO проти швидкості автопілота (LOCAL_POSITION_NED) ---
    def pairs(a_name, b_name):
        xs, ys = [], []
        for r in rows:
            try:
                x, y = float(r[a_name]), float(r[b_name])
            except (KeyError, ValueError):
                continue
            if not (math.isnan(x) or math.isnan(y)):
                xs.append(x); ys.append(y)
        return np.array(xs), np.array(ys)

    for axis, vio, ap_ in (("північ (вперед при курсі 0)", "vx", "ap_vn"), ("схід (вбік вправо при курсі 0)", "vy", "ap_ve")):
        v, ref = pairs(vio, ap_)
        if len(v) < 30 or np.max(np.abs(ref)) < 0.5:
            print(f"[--] вісь {axis}: рух по ній не збуджено (|v| < 0.5 м/с) - політай уздовж цієї осі")
            continue
        slope = float(np.dot(v, ref) / np.dot(ref, ref))
        if np.std(ref) > 0.1 and np.std(v) > 0:
            corr = float(np.corrcoef(v, ref)[0, 1])      # швидкість змінювалась - дивимось кореляцію
        else:
            corr = 1.0 if slope > 0 else -1.0            # стала швидкість: кореляція не визначена
        good = corr > 0.5 and 0.7 < slope < 1.3
        ok_all &= good
        hint = ""
        if not good:
            hint = " -> ЗНАК ІНВЕРТОВАНИЙ" if slope < -0.3 else " -> перевір осі/масштаб (fx, висота)"
            other = pairs(vio, "ap_ve" if ap_ == "ap_vn" else "ap_vn")
            if len(other[0]) > 30 and np.std(other[0]) > 0 and np.std(other[1]) > 0 \
                    and abs(np.corrcoef(*other)[0, 1]) > abs(corr) and abs(corr) < 0.5:
                hint += "; схоже, осі x/y переплутані"
        print(f"[{'OK' if good else 'FAIL'}] вісь {axis}: нахил VIO/автопілот {slope:.2f} "
              f"(очікується ~1), кореляція {corr:.2f}{hint}")

    # --- прийняті/відхилені вимірювання ---
    if rows:
        try:
            acc, rej = int(float(rows[-1]["accepted"])), int(float(rows[-1]["rejected"]))
            tot = max(acc + rej, 1)
            print(f"[info] вимірювань камери: прийнято {acc}, відхилено {rej} ({100 * rej / tot:.0f}% відмов)")
        except (KeyError, ValueError):
            pass

    # --- covariance має рости при погіршенні якості ---
    qv, pv = [], []
    for r in rows:
        try:
            qv.append(float(r["quality"])); pv.append(float(r["pos_var"]))
        except (KeyError, ValueError):
            pass
    qv, pv = np.array(qv), np.array(pv)
    if len(qv):
        hi, lo = pv[qv > 0.9], pv[qv < 0.5]
        if len(hi) >= 10 and len(lo) >= 10:
            r_ = float(np.median(lo) / np.median(hi))
            good = r_ > 2.0
            ok_all &= good
            print(f"[{'OK' if good else 'FAIL'}] covariance при поганій якості в {r_:.1f}x більша, "
                  f"ніж при доброї (треба > 2x)")
        else:
            print("[--] деградації якості в логі немає - зроби тест: закрий/розмий камеру на кілька секунд")

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
