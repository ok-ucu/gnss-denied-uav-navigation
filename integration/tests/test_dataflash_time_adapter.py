import csv
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "dataflash_time_adapter",
    ROOT / "dataflash_time_adapter.py"
)
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


def model():
    return {
        "verified_same_session": True,
        "source_clock": "FCU_BOOT_TIME_US",
        "target_clock": "HOST_MONOTONIC_NS",
        "a_ns": 1000000000,
        "b_ns_per_s": 1000,
        "t0_ap_ns": 1000000000,
    }


def test_reject_unverified_model():
    m = model()
    m["verified_same_session"] = False

    with pytest.raises(ValueError):
        adapter.validate_model(m)


def test_reject_wrong_clock_domain():
    m = model()
    m["target_clock"] = "UNIX_NS"

    with pytest.raises(ValueError):
        adapter.validate_model(m)


def test_convert_timestamp():
    m = model()
    assert adapter.convert_time(1000000, m) == 2000000000
    assert adapter.convert_time(2000000, m) == 3000001000


def test_convert_csv(tmp_path):
    source = tmp_path / "gps.csv"
    output = tmp_path / "gps_synced.csv"

    with source.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_boot_us", "lat_deg"])
        writer.writerow([1000000, 50.0])
        writer.writerow([2000000, 50.1])

    count = adapter.convert_csv(source, output, model())
    assert count == 2

    with output.open() as f:
        rows = list(csv.DictReader(f))

    assert int(rows[0]["time_host_ns"]) == 2000000000
    assert int(rows[1]["time_host_ns"]) == 3000001000


def test_missing_timestamp_column(tmp_path):
    source = tmp_path / "gps.csv"
    source.write_text("lat_deg\n50.0\n")

    with pytest.raises(ValueError):
        adapter.convert_csv(source, tmp_path / "out.csv", model())
