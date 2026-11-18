from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from verify_sync_axes import main


def write_sensor(dataset, sensor, stamps):
    path = dataset / "mav0" / sensor / "data.csv"
    path.parent.mkdir(parents=True)
    path.write_text("#timestamp [ns],sample\n" + "".join(f"{stamp},0\n" for stamp in stamps))


def test_measured_alignment_uses_integer_nanoseconds(tmp_path):
    epoch = 1_700_000_000_000_000_000
    write_sensor(tmp_path, "imu0", [epoch, epoch + 40_000_000, epoch + 80_000_000])
    write_sensor(tmp_path, "cam0", [epoch + 1_000_001, epoch + 40_000_000])
    assert main(["--dataset", str(tmp_path), "--max-gap-ms", "1.000001"]) == 0
    assert main(["--dataset", str(tmp_path), "--max-gap-ms", "1"]) == 1


def test_deliberately_offset_frames_fail(tmp_path):
    write_sensor(tmp_path, "imu0", [0, 40_000_000, 80_000_000])
    write_sensor(tmp_path, "cam0", [10_000_000, 50_000_000])
    assert main(["--dataset", str(tmp_path)]) == 1


def test_missing_and_malformed_evidence_fail(tmp_path):
    assert main(["--dataset", str(tmp_path)]) == 1
    write_sensor(tmp_path, "imu0", [0, 40_000_000])
    write_sensor(tmp_path, "cam0", [0, 0])
    assert main(["--dataset", str(tmp_path)]) == 1
