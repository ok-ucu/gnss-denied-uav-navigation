import csv
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "ardupilot_log_parser.py"

spec = importlib.util.spec_from_file_location(
    "ardupilot_log_parser", MODULE_PATH
)
parser = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parser)


class FakeMessage(SimpleNamespace):
    def get_type(self):
        return self.kind


def test_gps_conversion():
    msg = FakeMessage(
        kind="GPS",
        TimeUS=1000000,
        I=0,
        Lat=50.45,
        Lng=30.52,
        Alt=150.0,
        Status=3,
        NSats=12,
        HDop=1.2,
    )

    kind, row = parser.parse_message(msg)

    assert kind == "gps"
    assert row["lat_deg"] == 50.45
    assert row["lon_deg"] == 30.52
    assert row["time_boot_us"] == 1000000
    assert row["fix_type"] == 3


def test_imu_instances():
    for instance in (0, 1):
        msg = FakeMessage(
            kind="IMU",
            TimeUS=2000000,
            I=instance,
            GyrX=0.1,
            GyrY=0.2,
            GyrZ=0.3,
            AccX=1.0,
            AccY=2.0,
            AccZ=-9.81,
        )

        kind, row = parser.parse_message(msg)

        assert kind == "imu"
        assert row["instance"] == instance
        assert row["acc_z_m_s2"] == -9.81


def test_attitude_degrees_to_radians():
    msg = FakeMessage(
        kind="ATT",
        TimeUS=3000000,
        Roll=90,
        Pitch=0,
        Yaw=180,
    )

    kind, row = parser.parse_message(msg)

    assert kind == "attitude"
    assert math.isclose(row["roll_rad"], math.pi / 2)
    assert math.isclose(row["yaw_rad"], math.pi)


def test_missing_timestamp_rejected():
    msg = FakeMessage(
        kind="GPS",
        Lat=50,
        Lng=30,
        Alt=100,
    )

    assert parser.parse_message(msg) is None


def test_invalid_gps_rejected():
    msg = FakeMessage(
        kind="GPS",
        TimeUS=100,
        Lat=150,
        Lng=30,
        Alt=100,
    )

    assert parser.parse_message(msg) is None


def test_real_sitl_log_if_available(tmp_path):
    import pytest

    source = (
        ROOT
        / "linux_gpu_runs/20261115-eglcheck1"
        / "state/fcu/logs/00000001.BIN"
    )

    if not source.is_file():
        pytest.skip("SITL DataFlash fixture unavailable")

    pytest.importorskip("pymavlink")

    result = parser.export_log(source, tmp_path)

    assert result["counts"]["gps"] > 0
    assert result["counts"]["imu"] > 0
    assert result["counts"]["attitude"] > 0

    with (tmp_path / "imu.csv").open() as f:
        rows = list(csv.DictReader(f))

    instances = {r["instance"] for r in rows}
    assert {"0", "1"}.issubset(instances)

    for instance in instances:
        stamps = [
            int(r["time_boot_us"])
            for r in rows if r["instance"] == instance
        ]
        assert len(stamps) == len(set(stamps))

    metadata = json.loads(
        (tmp_path / "metadata.json").read_text()
    )
    assert metadata["synchronized_to_camera"] is False
    assert metadata["gps_ground_truth_accuracy"] == "NOT VERIFIED"
