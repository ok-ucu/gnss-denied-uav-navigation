#!/usr/bin/env python3
"""Read-only ArduPilot DataFlash GPS/IMU/ATT exporter."""

import argparse
import csv
import json
import tempfile
from collections import Counter
from pathlib import Path

from pymavlink import mavutil


FIELDS = {
    "gps": [
        "time_us", "timestamp_ap_ns", "instance",
        "lat_deg", "lon_deg", "alt_m",
        "hdop", "hacc_m", "vacc_m",
        "vel_ms", "fix_type", "satellites", "valid_fix",
    ],
    "imu": [
        "time_us", "timestamp_ap_ns", "instance",
        "gx_rad_s", "gy_rad_s", "gz_rad_s",
        "ax_m_s2", "ay_m_s2", "az_m_s2",
    ],
    "attitude": [
        "time_us", "timestamp_ap_ns",
        "roll_deg", "pitch_deg", "yaw_deg",
    ],
}


def timestamp(msg):
    us = int(msg.TimeUS)
    return us, us * 1000


def _export_to_directory(source: Path, destination: Path):
    if not source.is_file():
        raise FileNotFoundError(source)

    if destination.exists():
        raise FileExistsError(
            f"Output directory already exists: {destination}"
        )

    destination.mkdir(parents=True)

    files = {}
    writers = {}

    try:
        for kind, fields in FIELDS.items():
            handle = (destination / f"{kind}.csv").open(
                "w", newline="", encoding="utf-8"
            )
            files[kind] = handle
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writers[kind] = writer

        log = mavutil.mavlink_connection(str(source))
        counts = Counter()
        gps_quality = Counter()

        # GPA may be logged before or after GPS at the same TimeUS.
        # Collect accuracy separately and match by (instance, TimeUS).
        gps_records = []
        gpa_records = {}

        while True:
            msg = log.recv_match()
            if msg is None:
                break

            kind = msg.get_type()

            if kind == "GPA":
                key = (int(msg.I), int(msg.TimeUS))
                gpa_records[key] = {
                    "hacc_m": float(msg.HAcc),
                    "vacc_m": float(msg.VAcc),
                }

            elif kind == "GPS":
                gps_records.append(msg.to_dict())

            elif kind == "IMU":
                us, ns = timestamp(msg)
                writers["imu"].writerow({
                    "time_us": us,
                    "timestamp_ap_ns": ns,
                    "instance": int(msg.I),
                    "gx_rad_s": msg.GyrX,
                    "gy_rad_s": msg.GyrY,
                    "gz_rad_s": msg.GyrZ,
                    "ax_m_s2": msg.AccX,
                    "ay_m_s2": msg.AccY,
                    "az_m_s2": msg.AccZ,
                })
                counts["imu"] += 1

            elif kind == "ATT":
                us, ns = timestamp(msg)
                writers["attitude"].writerow({
                    "time_us": us,
                    "timestamp_ap_ns": ns,
                    "roll_deg": msg.Roll,
                    "pitch_deg": msg.Pitch,
                    "yaw_deg": msg.Yaw,
                })
                counts["attitude"] += 1

        for msg in gps_records:
            us = int(msg["TimeUS"])
            instance = int(msg["I"])
            fix = int(msg["Status"])
            accuracy = gpa_records.get((instance, us), {})

            writers["gps"].writerow({
                "time_us": us,
                "timestamp_ap_ns": us * 1000,
                "instance": instance,
                "lat_deg": msg["Lat"],
                "lon_deg": msg["Lng"],
                "alt_m": msg["Alt"],
                "hdop": msg["HDop"],
                "hacc_m": accuracy.get("hacc_m", ""),
                "vacc_m": accuracy.get("vacc_m", ""),
                "vel_ms": msg["Spd"],
                "fix_type": fix,
                "satellites": msg["NSats"],
                "valid_fix": int(fix >= 3),
            })

            counts["gps"] += 1
            gps_quality[str(fix)] += 1

        summary = {
            "source": str(source.resolve()),
            "time_domain": "ArduPilot DataFlash TimeUS",
            "camera_synchronized": False,
            "gps_samples": counts["gps"],
            "imu_samples": counts["imu"],
            "attitude_samples": counts["attitude"],
            "gps_fix_statuses": dict(gps_quality),
            "notes": [
                "SITL GPS is not physical ground truth.",
                "DataFlash timestamps are not synchronized to camera timestamps.",
                "GPS fix >= 3 is a basic status filter, not an accuracy guarantee.",
                "IMU and attitude use native DataFlash units.",
            ],
        }

        with (destination / "summary.json").open(
            "w", encoding="utf-8"
        ) as handle:
            json.dump(summary, handle, indent=2)

    finally:
        for handle in files.values():
            handle.close()

    return summary



def export(source: Path, destination: Path):
    """Export atomically without leaving partial output directories."""
    source = Path(source)
    destination = Path(destination)

    if not source.is_file():
        raise FileNotFoundError(source)

    if destination.exists():
        raise FileExistsError(destination)

    if not destination.parent.is_dir():
        raise FileNotFoundError(destination.parent)

    with tempfile.TemporaryDirectory(
        prefix=".dataflash_export_",
        dir=destination.parent,
    ) as temporary:
        staging = Path(temporary) / "result"
        summary = _export_to_directory(source, staging)

        if destination.exists():
            raise FileExistsError(destination)

        staging.rename(destination)

    return summary

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()

    summary = export(args.source, args.destination)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
