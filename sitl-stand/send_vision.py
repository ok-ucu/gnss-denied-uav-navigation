#!/usr/bin/env python3
"""Імітація зовнішньої навігації: бере LOCAL_POSITION_NED від SITL
і шле її назад як VISION_POSITION_ESTIMATE / VISION_SPEED_ESTIMATE.

Правила відправки (порт поведінки ov_repub/republisher.cpp з robocin/open_vins_ardupilot,
див. tests/ref_port/test_ref_sitl_link.py):
- до першого ATTITUDE не шлемо нічого (орієнтацію не вигадуємо);
- до першого LOCAL_POSITION_NED шлемо ЯВНИЙ старт "у початку координат" зі збільшеною
  коваріацією: без зовнішньої навігації EKF3 сам LOCAL_POSITION_NED не публікує,
  тож без цього кроку петля ніколи не запуститься (у SITL ще не перевірено);
- після цього шлемо лише свіжі дані (не старші за STALE_S); якщо SITL перестав їх
  давати, відправку зупиняємо, а не повторюємо застиглу позицію з новою міткою часу,
  щоб автопілот побачив втрату (FS_EKF);
- коваріацію задаємо явно (нулі в pymavlink означали б "ідеально точно")."""
import time
from pymavlink import mavutil

CONN = "udpin:127.0.0.1:14551"
RATE_HZ = 30
ORIGIN = (-35.363261, 149.165230, 584.0)
STALE_S = 0.5          # LOCAL_POSITION_NED старше за це не пересилаємо


def pose_cov(pos_var, att_var=1e-3):
    """21 елемент: верхній трикутник 6x6 по рядках (x y z roll pitch yaw)."""
    diag = {0: pos_var, 6: pos_var, 11: pos_var, 15: att_var, 18: att_var, 20: att_var}
    return [diag.get(i, 0.0) for i in range(21)]


def speed_cov(vel_var):
    return [vel_var, 0.0, 0.0, 0.0, vel_var, 0.0, 0.0, 0.0, vel_var]


POS_COV, VEL_COV = pose_cov(0.01), speed_cov(0.01)            # 0.1 м, 0.1 м/с
BOOT_POS_COV, BOOT_VEL_COV = pose_cov(0.25), speed_cov(0.25)  # 0.5 м, 0.5 м/с

master = mavutil.mavlink_connection(CONN)
print("Чекаю heartbeat...")
master.wait_heartbeat()
print(f"Підключено: system {master.target_system}")

master.mav.set_gps_global_origin_send(
    master.target_system,
    int(ORIGIN[0] * 1e7), int(ORIGIN[1] * 1e7), int(ORIGIN[2] * 1000),
    int(time.time() * 1e6),
)
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 30, 1)
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 30, 1)

t0 = time.time()
x = y = z = vx = vy = vz = 0.0
roll = pitch = yaw = 0.0
have_att = False
t_pos = None          # коли прийшов останній LOCAL_POSITION_NED (None - ще жодного)
stale_reported = False
last = 0.0
print("Шлю позицію з SITL. Зупинка: Ctrl+C")
while True:
    msg = master.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=False)
    while msg:
        if msg.get_type() == "LOCAL_POSITION_NED":
            x, y, z, vx, vy, vz = msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz
            if t_pos is None:
                print("Перший LOCAL_POSITION_NED: старт завершено")
            t_pos = time.time()
            if stale_reported:
                print("LOCAL_POSITION_NED знову надходить, відправку відновлено")
                stale_reported = False
        else:
            roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw
            have_att = True
        msg = master.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=False)
    now = time.time()
    if now - last >= 1.0 / RATE_HZ and have_att:
        if t_pos is None:
            pos_cov, vel_cov = BOOT_POS_COV, BOOT_VEL_COV      # старт у (0, 0, 0), швидкість 0
        elif now - t_pos <= STALE_S:
            pos_cov, vel_cov = POS_COV, VEL_COV
        else:
            pos_cov = None
            if not stale_reported:
                print(f"!!! LOCAL_POSITION_NED не оновлювався {now - t_pos:.1f} с: відправку зупинено")
                stale_reported = True
        if pos_cov is not None:
            usec = int((now - t0) * 1e6)
            master.mav.vision_position_estimate_send(usec, x, y, z, roll, pitch, yaw, pos_cov)
            master.mav.vision_speed_estimate_send(usec, vx, vy, vz, vel_cov)
            last = now
    time.sleep(0.002)
