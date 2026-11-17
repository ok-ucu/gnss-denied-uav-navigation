import csv
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "gps_quality_check", ROOT / "gps_quality_check.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def make_csv(tmp_path, points):
    path = tmp_path / "gps.csv"
    fields = [
        "time_boot_us", "instance", "lat_deg", "lon_deg",
        "alt_m", "fix_type", "satellites"
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(points)
    return path


def point(t, lat=50.0, lon=30.0, fix=3, sats=10):
    return {
        "time_boot_us": t,
        "instance": 0,
        "lat_deg": lat,
        "lon_deg": lon,
        "alt_m": 100,
        "fix_type": fix,
        "satellites": sats,
    }


def test_good_gps_points(tmp_path):
    path = make_csv(tmp_path, [
        point(1000000),
        point(2000000, lat=50.00001),
    ])
    report, accepted = module.analyze(path)
    assert report["accepted_points"] == 2
    assert len(accepted) == 2


def test_reject_bad_fix(tmp_path):
    path = make_csv(tmp_path, [
        point(1000000, fix=1),
        point(2000000, fix=3),
    ])
    report, _ = module.analyze(path)
    assert report["accepted_points"] == 1
    assert report["rejected"]["insufficient_fix"] == 1


def test_reject_insufficient_satellites(tmp_path):
    path = make_csv(tmp_path, [
        point(1000000, sats=3),
    ])
    report, _ = module.analyze(path)
    assert report["accepted_points"] == 0
    assert report["rejected"]["insufficient_satellites"] == 1


def test_reject_gps_jump(tmp_path):
    path = make_csv(tmp_path, [
        point(1000000),
        point(2000000, lat=51.0),
    ])
    report, _ = module.analyze(path)
    assert report["accepted_points"] == 1
    assert report["rejected"]["implausible_speed"] == 1


def test_reject_duplicate_timestamp(tmp_path):
    path = make_csv(tmp_path, [
        point(1000000),
        point(1000000, lat=50.00001),
    ])
    report, _ = module.analyze(path)
    assert report["accepted_points"] == 1
    assert report["rejected"]["non_increasing_timestamp"] == 1


def test_real_sitl_csv_if_available():
    import pytest

    path = Path("/tmp/person1_dataflash_export/gps.csv")
    if not path.is_file():
        pytest.skip("Exported SITL GPS CSV unavailable")

    report, accepted = module.analyze(path)

    assert report["input_rows"] == 129
    assert report["accepted_points"] == 128
    assert len(accepted) == 128
    assert report["accuracy_verified"] is False
