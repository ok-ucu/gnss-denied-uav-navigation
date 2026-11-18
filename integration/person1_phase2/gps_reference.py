#!/usr/bin/env python3
"""Convert exported ArduPilot GPS coordinates to local ENU coordinates."""

import argparse
import csv
import math
from pathlib import Path

from pyproj import Transformer


OUTPUT_FIELDS = [
    "time_us",
    "timestamp_ap_ns",
    "instance",
    "x_m",
    "y_m",
    "z_m",
    "lat_deg",
    "lon_deg",
    "alt_m",
    "hacc_m",
    "vacc_m",
    "fix_type",
    "satellites",
]


def convert(source: Path, destination: Path):
    source = Path(source)
    destination = Path(destination)

    if not source.is_file():
        raise FileNotFoundError(source)

    if destination.exists():
        raise FileExistsError(destination)

    with source.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        required = {
            "time_us", "timestamp_ap_ns", "instance",
            "lat_deg", "lon_deg", "alt_m", "valid_fix",
        }
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("GPS CSV is missing required columns")

        rows = [
            row for row in reader
            if row["valid_fix"] == "1"
        ]

    if not rows:
        raise ValueError("No valid GPS fixes available")

    # WGS84 geodetic coordinates -> ECEF meters.
    to_ecef = Transformer.from_crs(
        "EPSG:4979", "EPSG:4978", always_xy=True
    )

    origin = rows[0]
    lat0 = float(origin["lat_deg"])
    lon0 = float(origin["lon_deg"])
    alt0 = float(origin["alt_m"])

    x0, y0, z0 = to_ecef.transform(lon0, lat0, alt0)

    lat_rad = math.radians(lat0)
    lon_rad = math.radians(lon0)

    sin_lat, cos_lat = math.sin(lat_rad), math.cos(lat_rad)
    sin_lon, cos_lon = math.sin(lon_rad), math.cos(lon_rad)

    destination.parent.mkdir(parents=True, exist_ok=True)

    with destination.open("x", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()

        for row in rows:
            lat = float(row["lat_deg"])
            lon = float(row["lon_deg"])
            alt = float(row["alt_m"])

            if not (
                -90 <= lat <= 90
                and -180 <= lon <= 180
                and all(map(math.isfinite, (lat, lon, alt)))
            ):
                raise ValueError("Invalid GPS coordinates")

            x, y, z = to_ecef.transform(lon, lat, alt)
            dx, dy, dz = x - x0, y - y0, z - z0

            east = -sin_lon * dx + cos_lon * dy
            north = (
                -sin_lat * cos_lon * dx
                - sin_lat * sin_lon * dy
                + cos_lat * dz
            )
            up = (
                cos_lat * cos_lon * dx
                + cos_lat * sin_lon * dy
                + sin_lat * dz
            )

            writer.writerow({
                "time_us": row["time_us"],
                "timestamp_ap_ns": row["timestamp_ap_ns"],
                "instance": row["instance"],
                "x_m": east,
                "y_m": north,
                "z_m": up,
                "lat_deg": row["lat_deg"],
                "lon_deg": row["lon_deg"],
                "alt_m": row["alt_m"],
                "hacc_m": row.get("hacc_m", ""),
                "vacc_m": row.get("vacc_m", ""),
                "fix_type": row.get("fix_type", ""),
                "satellites": row.get("satellites", ""),
            })

    return len(rows)


def main():
    parser = argparse.ArgumentParser(
        description="Convert GPS CSV to local ENU reference trajectory"
    )
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()

    count = convert(args.source, args.destination)
    print(f"Exported {count} valid GPS points to {args.destination}")


if __name__ == "__main__":
    main()
