#!/usr/bin/env python3
"""Convert filtered GPS CSV to local NED/ENU trajectories.

Coordinates use WGS84. Input altitude must be ellipsoidal
for geometrically correct WGS84 ECEF conversion.
FCU TimeUS remains boot-relative, not UNIX/ROS time.
"""
import argparse
import csv
import json
import math
from pathlib import Path

A = 6378137.0
F = 1.0 / 298.257223563
E2 = F * (2.0 - F)


def ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n = A / math.sqrt(1.0 - E2 * sin_lat**2)
    return (
        (n + alt_m) * cos_lat * math.cos(lon),
        (n + alt_m) * cos_lat * math.sin(lon),
        (n * (1.0 - E2) + alt_m) * sin_lat,
    )


def local_enu(point, origin):
    x, y, z = ecef(*point)
    x0, y0, z0 = ecef(*origin)
    dx, dy, dz = x - x0, y - y0, z - z0

    lat0 = math.radians(origin[0])
    lon0 = math.radians(origin[1])

    east = -math.sin(lon0) * dx + math.cos(lon0) * dy
    north = (
        -math.sin(lat0) * math.cos(lon0) * dx
        -math.sin(lat0) * math.sin(lon0) * dy
        + math.cos(lat0) * dz
    )
    up = (
        math.cos(lat0) * math.cos(lon0) * dx
        + math.cos(lat0) * math.sin(lon0) * dy
        + math.sin(lat0) * dz
    )
    return east, north, up


def export(source, output, instance=0):
    with Path(source).open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    points = []
    for row in rows:
        if int(row["instance"]) != instance:
            continue
        t = int(row["time_boot_us"])
        coords = (
            float(row["lat_deg"]),
            float(row["lon_deg"]),
            float(row["alt_m"]),
        )
        if t < 0 or not all(math.isfinite(v) for v in coords):
            raise ValueError("Invalid GPS timestamp or coordinates")
        if not (-90 <= coords[0] <= 90 and -180 <= coords[1] <= 180):
            raise ValueError("Invalid latitude/longitude")
        points.append((t, coords))

    if not points:
        raise ValueError("No GPS points for selected instance")

    points.sort(key=lambda p: p[0])
    if any(b[0] <= a[0] for a, b in zip(points, points[1:])):
        raise ValueError("Non-increasing GPS timestamps")

    origin = points[0][1]
    t0 = points[0][0]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)

    with (
        (output / "gps_enu.csv").open("w", newline="") as enu_file,
        (output / "gps_ned.csv").open("w", newline="") as ned_file,
        (output / "gps_enu.tum").open("w") as tum_file,
    ):
        enu = csv.writer(enu_file)
        ned = csv.writer(ned_file)

        enu.writerow(["time_boot_us", "east_m", "north_m", "up_m"])
        ned.writerow(["time_boot_us", "north_m", "east_m", "down_m"])

        for t, coords in points:
            e, n, u = local_enu(coords, origin)
            enu.writerow([t, e, n, u])
            ned.writerow([t, n, e, -u])

            # TUM: timestamp tx ty tz qx qy qz qw.
            # Identity quaternion is a placeholder, NOT GPS orientation.
            ts = (t - t0) / 1e6
            tum_file.write(
                f"{ts:.6f} {e:.6f} {n:.6f} {u:.6f} "
                "0 0 0 1\n"
            )

    metadata = {
        "source": str(source),
        "instance": instance,
        "points": len(points),
        "origin_lat_lon_alt": list(origin),
        "coordinate_frame": "ENU",
        "alternate_frame": "NED",
        "coordinate_reference": "WGS84",
        "altitude_datum_verified": False,
        "time_domain": "FCU boot-relative microseconds",
        "tum_time_origin": "first exported GPS point",
        "tum_orientation": "placeholder identity; not measured",
        "camera_imu_synchronized": False,
        "ground_truth_accuracy_verified": False,
    }
    (output / "trajectory_metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n"
    )
    return metadata


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("gps_csv")
    ap.add_argument("--output", required=True)
    ap.add_argument("--instance", type=int, default=0)
    args = ap.parse_args()
    print(json.dumps(
        export(args.gps_csv, args.output, args.instance),
        indent=2
    ))


if __name__ == "__main__":
    main()
