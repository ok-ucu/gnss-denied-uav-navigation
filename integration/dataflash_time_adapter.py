#!/usr/bin/env python3
"""Apply an explicitly verified FCU->host clock model to DataFlash CSV.

The model must be calibrated for the SAME FCU boot/session.
No automatic synchronization or clock fitting is performed here.
"""
import argparse
import csv
import json
import math
from pathlib import Path


def validate_model(model):
    if model.get("verified_same_session") is not True:
        raise ValueError(
            "Clock model is not verified for this DataFlash session"
        )

    if model.get("source_clock") != "FCU_BOOT_TIME_US":
        raise ValueError("Unexpected source clock")

    if model.get("target_clock") != "HOST_MONOTONIC_NS":
        raise ValueError("Unexpected target clock")

    for key in ("a_ns", "b_ns_per_s", "t0_ap_ns"):
        if key not in model:
            raise ValueError(f"Missing clock model field: {key}")
        if not isinstance(model[key], (int, float)):
            raise ValueError(f"Invalid clock model field: {key}")
        if not math.isfinite(model[key]):
            raise ValueError(f"Non-finite clock model field: {key}")


def convert_time(time_boot_us, model):
    t_ap_ns = int(time_boot_us) * 1000
    dt_s = (t_ap_ns - model["t0_ap_ns"]) / 1e9
    return round(
        t_ap_ns
        + model["a_ns"]
        + model["b_ns_per_s"] * dt_s
    )


def convert_csv(source, output, model):
    validate_model(model)

    with Path(source).open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames or "time_boot_us" not in reader.fieldnames:
            raise ValueError("Input CSV lacks time_boot_us")
        rows = list(reader)
        fields = ["time_host_ns"] + [
            x for x in reader.fieldnames if x != "time_host_ns"
        ]

    converted = []
    for row in rows:
        row = dict(row)
        row["time_host_ns"] = convert_time(row["time_boot_us"], model)
        converted.append(row)

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)

    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(converted)

    return len(converted)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("source")
    ap.add_argument("--model", required=True)
    ap.add_argument("--output", required=True)
    args = ap.parse_args()

    model = json.loads(Path(args.model).read_text())
    count = convert_csv(args.source, args.output, model)
    print(f"Converted {count} rows to host monotonic timestamps")


if __name__ == "__main__":
    main()
