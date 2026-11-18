#!/usr/bin/env python3
"""Витягує GPS-трек (ground truth) із логу ArduPilot (.BIN) у CSV.
Використання: python3 log_to_gps_csv.py flight.BIN out.csv
Колонки:
  t_s         секунди від першої GPS-точки
  gps_time_s  абсолютний GPS-час (тиждень*604800 + мс тижня/1000):
              для вирівнювання з мітками часу камери й VIO
  lat, lon, alt_m
  north_m, east_m, up_m   метри відносно першої точки"""
import csv
import math
import sys
from pymavlink import mavutil

src, dst = sys.argv[1], sys.argv[2]
log = mavutil.mavlink_connection(src)
M_PER_DEG = 111319.5
rows = []
ref = None
while True:
    m = log.recv_match(type="GPS", blocking=False)
    if m is None:
        break
    if getattr(m, "Status", 3) < 3:        # без 3D-фіксації точки не беремо
        continue
    lat, lon, alt = m.Lat, m.Lng, m.Alt
    if abs(lat) > 90:                      # на випадок градусів * 1e7
        lat, lon = lat * 1e-7, lon * 1e-7
    t = m.TimeUS / 1e6
    gps_t = m.GWk * 604800 + m.GMS / 1000.0
    if ref is None:
        ref = (t, lat, lon, alt)
    north = (lat - ref[1]) * M_PER_DEG
    east = (lon - ref[2]) * M_PER_DEG * math.cos(math.radians(ref[1]))
    rows.append([round(t - ref[0], 3), round(gps_t, 3), lat, lon, alt,
                 round(north, 3), round(east, 3), round(alt - ref[3], 3)])

with open(dst, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_s", "gps_time_s", "lat", "lon", "alt_m",
                "north_m", "east_m", "up_m"])
    w.writerows(rows)
print(f"Записано {len(rows)} точок у {dst}")
