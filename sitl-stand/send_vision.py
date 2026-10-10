#!/usr/bin/env python3
"""Імітація зовнішньої навігації: бере LOCAL_POSITION_NED від SITL
і шле її назад як VISION_POSITION_ESTIMATE / VISION_SPEED_ESTIMATE."""
import time
from pymavlink import mavutil

CONN = "udpin:127.0.0.1:14551"
RATE_HZ = 30
ORIGIN = (-35.363261, 149.165230, 584.0)

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
last = 0.0
print("Шлю позицію з SITL. Зупинка: Ctrl+C")
while True:
    msg = master.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=False)
    while msg:
        if msg.get_type() == "LOCAL_POSITION_NED":
            x, y, z, vx, vy, vz = msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz
        else:
            roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw
        msg = master.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=False)
    now = time.time()
    if now - last >= 1.0 / RATE_HZ:
        usec = int((now - t0) * 1e6)
        master.mav.vision_position_estimate_send(usec, x, y, z, roll, pitch, yaw)
        master.mav.vision_speed_estimate_send(usec, vx, vy, vz)
        last = now
    time.sleep(0.002)
