#!/usr/bin/env python3
"""Offline ArduPilot DataFlash BIN -> GPS/IMU/attitude CSV.

Never connects to an FCU. TimeUS is FCU boot-relative microseconds,
not UNIX time and not automatically synchronized to camera timestamps.
"""
import argparse
import csv
import json
import math
from pathlib import Path


def finite(value):
    try:
        x = float(value)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def field(msg, *names):
    for name in names:
        value = getattr(msg, name, None)
        if value is not None:
            return value
    return None


def timestamp_us(msg):
    value = field(msg, "TimeUS")
    try:
        t = int(value)
        return t if t >= 0 else None
    except (TypeError, ValueError, OverflowError):
        return None


SCHEMAS = {
    "gps": [
        "time_boot_us", "instance", "lat_deg", "lon_deg",
        "alt_m", "fix_type", "satellites", "hdop",
    ],
    "imu": [
        "time_boot_us", "instance",
        "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s",
        "acc_x_m_s2", "acc_y_m_s2", "acc_z_m_s2",
    ],
    "attitude": [
        "time_boot_us", "roll_rad", "pitch_rad", "yaw_rad",
    ],
}


def parse_message(msg):
    kind = msg.get_type()
    t = timestamp_us(msg)
    if t is None:
        return None

    if kind in ("GPS", "GPS2"):
        lat = finite(field(msg, "Lat"))
        lon = finite(field(msg, "Lng", "Lon"))
        alt = finite(field(msg, "Alt"))
        if lat is None or lon is None or alt is None:
            return None
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return None

        return "gps", {
            "time_boot_us": t,
            "instance": field(msg, "I") if kind == "GPS" else 1,
            "lat_deg": lat,
            "lon_deg": lon,
            "alt_m": alt,
            "fix_type": field(msg, "Status"),
            "satellites": field(msg, "NSats"),
            "hdop": field(msg, "HDop"),
        }

    if kind in ("IMU", "IMU2", "IMU3"):
        gyro = [finite(field(msg, k)) for k in ("GyrX", "GyrY", "GyrZ")]
        acc = [finite(field(msg, k)) for k in ("AccX", "AccY", "AccZ")]
        if any(v is None for v in gyro + acc):
            return None
        instance = field(msg, "I")
        if instance is None:
            instance = {"IMU": 0, "IMU2": 1, "IMU3": 2}[kind]
        return "imu", {
            "time_boot_us": t,
            "instance": instance,
            **dict(zip(
                SCHEMAS["imu"][2:],
                gyro + acc,
            )),
        }

    if kind == "ATT":
        angles = [finite(field(msg, k)) for k in ("Roll", "Pitch", "Yaw")]
        if any(v is None for v in angles):
            return None

        # ArduPilot DataFlash ATT angles are in degrees.
        return "attitude", {
            "time_boot_us": t,
            "roll_rad": math.radians(angles[0]),
            "pitch_rad": math.radians(angles[1]),
            "yaw_rad": math.radians(angles[2]),
        }

    return None


def export_log(source, output):
    from pymavlink import mavutil

    source = Path(source)
    output = Path(output)

    if not source.is_file():
        raise FileNotFoundError(source)

    output.mkdir(parents=True, exist_ok=True)

    files = {}
    writers = {}
    counts = {key: 0 for key in SCHEMAS}

    try:
        for name, columns in SCHEMAS.items():
            handle = (output / f"{name}.csv").open(
                "w", newline="", encoding="utf-8"
            )
            files[name] = handle
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            writers[name] = writer

        connection = mavutil.mavlink_connection(
            str(source),
            dialect="ardupilotmega",
            notimestamps=True,
        )

        try:
            while True:
                msg = connection.recv_match(blocking=False)
                if msg is None:
                    break

                result = parse_message(msg)
                if result is None:
                    continue

                name, row = result
                writers[name].writerow(row)
                counts[name] += 1
        finally:
            connection.close()
    finally:
        for handle in files.values():
            handle.close()

    metadata = {
        "source": str(source),
        "format": "ArduPilot DataFlash",
        "clock_domain": "FCU boot-relative TimeUS",
        "synchronized_to_camera": False,
        "gps_altitude_datum": "verify against source log metadata",
        "gps_ground_truth_accuracy": "NOT VERIFIED",
        "counts": counts,
    }

    (output / "metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n",
        encoding="utf-8",
    )

    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    result = export_log(args.input, args.output)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
