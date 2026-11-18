import csv
import importlib.util
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "gps_trajectory_export",
    ROOT / "gps_trajectory_export.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_origin_is_zero():
    origin = (50.0, 30.0, 100.0)
    e, n, u = module.local_enu(origin, origin)
    assert abs(e) < 1e-6
    assert abs(n) < 1e-6
    assert abs(u) < 1e-6


def test_northward_motion():
    origin = (50.0, 30.0, 100.0)
    point = (50.001, 30.0, 100.0)
    e, n, u = module.local_enu(point, origin)

    assert abs(e) < 0.1
    assert 110 < n < 112
    assert abs(u) < 0.1


def test_eastward_motion():
    origin = (50.0, 30.0, 100.0)
    point = (50.0, 30.001, 100.0)
    e, n, u = module.local_enu(point, origin)

    assert 70 < e < 73
    assert abs(n) < 0.1
    assert abs(u) < 0.1


def test_upward_motion():
    origin = (50.0, 30.0, 100.0)
    point = (50.0, 30.0, 110.0)
    e, n, u = module.local_enu(point, origin)

    assert abs(e) < 1e-6
    assert abs(n) < 1e-6
    assert math.isclose(u, 10.0, abs_tol=1e-6)


def test_export_enu_ned(tmp_path):
    source = tmp_path / "gps.csv"
    output = tmp_path / "out"

    with source.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time_boot_us", "instance", "lat_deg",
            "lon_deg", "alt_m"
        ])
        writer.writerow([1000000, 0, 50.0, 30.0, 100.0])
        writer.writerow([2000000, 0, 50.001, 30.001, 110.0])

    metadata = module.export(source, output)

    with (output / "gps_enu.csv").open() as f:
        enu = list(csv.DictReader(f))
    with (output / "gps_ned.csv").open() as f:
        ned = list(csv.DictReader(f))

    assert len(enu) == 2
    assert len(ned) == 2

    e = float(enu[1]["east_m"])
    n = float(enu[1]["north_m"])
    u = float(enu[1]["up_m"])

    assert math.isclose(float(ned[1]["north_m"]), n)
    assert math.isclose(float(ned[1]["east_m"]), e)
    assert math.isclose(float(ned[1]["down_m"]), -u)

    assert metadata["camera_imu_synchronized"] is False
    assert metadata["ground_truth_accuracy_verified"] is False


def test_reject_non_increasing_timestamps(tmp_path):
    source = tmp_path / "gps.csv"

    with source.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time_boot_us", "instance", "lat_deg",
            "lon_deg", "alt_m"
        ])
        writer.writerow([1000000, 0, 50, 30, 100])
        writer.writerow([1000000, 0, 50.001, 30, 100])

    import pytest
    with pytest.raises(ValueError):
        module.export(source, tmp_path / "out")
