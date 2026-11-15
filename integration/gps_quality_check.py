#!/usr/bin/env python3
"""Offline GPS trajectory quality checks for ArduPilot DataFlash CSV."""

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path


def distance_m(a, b):
    radius = 6371000.0
    lat1, lon1 = map(math.radians, (a["lat"], a["lon"]))
    lat2, lon2 = map(math.radians, (b["lat"], b["lon"]))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    h = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2)
        * math.sin(dlon / 2) ** 2
    )
    horizontal = 2 * radius * math.asin(min(1.0, math.sqrt(h)))
    return math.hypot(horizontal, b["alt"] - a["alt"])


def analyze(path, min_fix=3, min_sats=6, max_speed=50.0):
    with Path(path).open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    points = []
    for row in rows:
        try:
            point = {
                "time_us": int(row["time_boot_us"]),
                "instance": int(row["instance"]),
                "lat": float(row["lat_deg"]),
                "lon": float(row["lon_deg"]),
                "alt": float(row["alt_m"]),
                "fix": int(row["fix_type"]),
                "sats": int(row["satellites"]),
            }
        except (ValueError, TypeError, KeyError):
            continue

        if not all(math.isfinite(point[k]) for k in ("lat", "lon", "alt")):
            continue
        if not (-90 <= point["lat"] <= 90 and -180 <= point["lon"] <= 180):
            continue
        points.append(point)

    points.sort(key=lambda p: (p["instance"], p["time_us"]))

    accepted = []
    rejected = Counter()
    last_by_instance = {}

    for point in points:
        if point["fix"] < min_fix:
            rejected["insufficient_fix"] += 1
            continue
        if point["sats"] < min_sats:
            rejected["insufficient_satellites"] += 1
            continue

        previous = last_by_instance.get(point["instance"])
        if previous is not None:
            dt = (point["time_us"] - previous["time_us"]) / 1e6
            if dt <= 0:
                rejected["non_increasing_timestamp"] += 1
                continue
            speed = distance_m(previous, point) / dt
            if speed > max_speed:
                rejected["implausible_speed"] += 1
                continue

        accepted.append(point)
        last_by_instance[point["instance"]] = point

    return {
        "input_rows": len(rows),
        "parsed_points": len(points),
        "accepted_points": len(accepted),
        "rejected": dict(rejected),
        "accepted_by_instance": dict(
            Counter(str(p["instance"]) for p in accepted)
        ),
        "criteria": {
            "min_fix": min_fix,
            "min_satellites": min_sats,
            "max_speed_m_s": max_speed,
        },
        "accuracy_verified": False,
    }, accepted


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("gps_csv", type=Path)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--min-fix", type=int, default=3)
    ap.add_argument("--min-sats", type=int, default=6)
    ap.add_argument("--max-speed", type=float, default=50.0)
    args = ap.parse_args()

    report, accepted = analyze(
        args.gps_csv,
        args.min_fix,
        args.min_sats,
        args.max_speed,
    )

    args.output.mkdir(parents=True, exist_ok=True)

    (args.output / "gps_quality.json").write_text(
        json.dumps(report, indent=2) + "\n",
        encoding="utf-8",
    )

    with (args.output / "gps_filtered.csv").open(
        "w", newline="", encoding="utf-8"
    ) as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "time_boot_us", "instance", "lat_deg",
                "lon_deg", "alt_m", "fix_type", "satellites"
            ],
        )
        writer.writeheader()
        for p in accepted:
            writer.writerow({
                "time_boot_us": p["time_us"],
                "instance": p["instance"],
                "lat_deg": p["lat"],
                "lon_deg": p["lon"],
                "alt_m": p["alt"],
                "fix_type": p["fix"],
                "satellites": p["sats"],
            })

    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
