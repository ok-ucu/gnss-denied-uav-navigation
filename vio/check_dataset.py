"""check_dataset.py - перевірка датасету sample_flight на твоєму VIO-коді.

Запуск з папки, де лежать vio_core.py і vio_frontend.py:
    python check_dataset.py recordings/sample_flight

Що робить:
  1. читає кадри, IMU, attitude.csv і local_position.csv;
  2. для кожного кадра бере висоту (-z), орієнтацію і гіроскоп на момент кадра;
  3. проганяє кадри через FlowFrontend (оптичний потік -> швидкість);
  4. проганяє VioEKF (IMU + швидкість з потоку);
  5. порівнює з vx, vy з local_position.csv і друкує підсумок.
"""
import sys
from pathlib import Path

import cv2
import numpy as np

from vio_core import VioEKF, rot_body_to_ned
from vio_frontend import FlowFrontend

FX = 461.629  # з meta.json


def load_table(path):
    """Читає CSV/TSV: перша колонка - час (нс, ціле), решта - числа.
    Рядки-заголовки (з '#' або з текстом) пропускає."""
    ts, rows = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace(",", " ").split()
            try:
                t = int(parts[0])
                vals = [float(x) for x in parts[1:]]
            except ValueError:
                continue
            ts.append(t)
            rows.append(vals)
    return np.array(ts, dtype=np.int64), np.array(rows, dtype=float)


def load_cam_list(path):
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            a = line.replace(",", " ").split()
            out.append((int(a[0]), a[1]))
    return out


def main(root):
    root = Path(root)
    cam = load_cam_list(root / "mav0/cam0/data.csv")
    t_imu_ns, imu = load_table(root / "mav0/imu0/data.csv")
    t_att_ns, att = load_table(root / "attitude.csv")
    t_pos_ns, pos = load_table(root / "local_position.csv")

    T0 = cam[0][0]

    def sec(t_ns):
        return (np.asarray(t_ns, dtype=np.int64) - T0) * 1e-9

    t_imu, t_att, t_pos = sec(t_imu_ns), sec(t_att_ns), sec(t_pos_ns)

    # --- attitude: радіани чи градуси? ---
    ang = att[:, 0:3].copy()
    if np.abs(ang).max() > 2 * np.pi:
        print("УВАГА: attitude схоже в ГРАДУСАХ, перетворюю в радіани")
        ang = np.deg2rad(ang)
    ang[:, 2] = np.unwrap(ang[:, 2])

    def R_at(t):
        r, p, y = (float(np.interp(t, t_att, ang[:, k])) for k in range(3))
        return rot_body_to_ned(r, p, y)

    def pos_at(t, col):
        return float(np.interp(t, t_pos, pos[:, col]))

    print(f"кадрів: {len(cam)}, IMU: {len(t_imu)}, attitude: {len(t_att)}, position: {len(t_pos)}")
    print(f"середнє акселерометра (має бути ~ [0, 0, -9.8]): {imu[:, 3:6].mean(axis=0).round(3)}")
    print(f"висота (-z) від {-pos[:, 2].max():.2f} до {-pos[:, 2].min():.2f} м")

    fe = FlowFrontend(FX)
    ekf = VioEKF()

    imu_i, prev_t = 0, None
    rows, n_none = [], 0
    for tn, name in cam:
        t = float(sec(tn))

        # EKF: прогноз по всіх відліках IMU до моменту кадра
        while imu_i < len(t_imu) and t_imu[imu_i] <= t:
            ti = float(t_imu[imu_i])
            if prev_t is not None:
                dt = ti - prev_t
                if 0.0 < dt < 0.1:
                    ekf.predict(imu[imu_i, 3:6], R_at(ti), dt)
            prev_t = ti
            imu_i += 1

        gray = cv2.imread(str(root / "mav0/cam0/data" / name), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print("не вдалося прочитати кадр", name)
            continue

        height = -pos_at(t, 2)
        gyro = [float(np.interp(t, t_imu, imu[:, k])) for k in range(3)]
        res = fe.process(gray, t, height, gyro, R_at(t))
        if res is None:
            n_none += 1
            continue

        ekf.update_velocity_xy(res.v_xy, res.sigma_scale)
        ekf.update_height(pos_at(t, 2))   # замість далекоміра: z з local_position

        rows.append([t, res.v_xy[0], res.v_xy[1], pos_at(t, 3), pos_at(t, 4),
                     ekf.x[3], ekf.x[4], res.n_inliers, res.sigma_v])

    print(f"\nкадрів відкинуто фронтендом: {n_none} з {len(cam)} (перший кадр завжди відкидається)")
    if not rows:
        print("Жодного вимірювання потоку - дивись, чому кадри відкидаються (контраст? мало точок?)")
        return

    r = np.array(rows)
    print("\n                 vx (північ)   vy (схід)")
    print(f"істина (local_position):  {np.median(r[:, 3]):8.3f}   {np.median(r[:, 4]):8.3f}")
    print(f"оптичний потік (медіана): {np.median(r[:, 1]):8.3f}   {np.median(r[:, 2]):8.3f}")
    print(f"EKF (в кінці):            {r[-1, 5]:8.3f}   {r[-1, 6]:8.3f}")
    print(f"\nсередня похибка потоку: vx {np.mean(np.abs(r[:, 1] - r[:, 3])):.3f} м/с, "
          f"vy {np.mean(np.abs(r[:, 2] - r[:, 4])):.3f} м/с")
    print(f"середня кількість інлаєрів: {r[:, 7].mean():.0f}, середня sigma_v: {r[:, 8].mean():.3f} м/с")

    t_end = r[-1, 0]
    print(f"\nпозиція в кінці ({t_end:.2f} с):")
    print(f"  істина x,y: {pos_at(t_end, 0):.2f}, {pos_at(t_end, 1):.2f} м")
    print(f"  EKF    x,y: {ekf.x[0]:.2f}, {ekf.x[1]:.2f} м")
    print(f"  EKF відхилень швидкості (reset): {ekf.n_resets}")

    print("\nЯк читати:")
    print("  потік vx ~ +2 і vy ~ 0   -> все гаразд;")
    print("  vx ~ -2                  -> знак осі перевернуто (правити v_body у vio_frontend.py);")
    print("  vy ~ +-2, vx ~ 0         -> осі поміняно місцями;")
    print("  швидкість в разів більша/менша -> перевір fx і висоту.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Використання: python check_dataset.py recordings/sample_flight")
        sys.exit(1)
    main(sys.argv[1])
