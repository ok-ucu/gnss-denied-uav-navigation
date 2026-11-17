"""Тести парсера Kalibr і конвертера в bag. Запуск: cd calibration && pytest -q"""
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_report as mr   # noqa: E402

CHAIN = """cam0:
  camera_model: pinhole
  intrinsics: [461.629, 460.152, 362.680, 246.049]
  distortion_model: radtan
  distortion_coeffs: [-0.27695497, 0.06712482, 0.00087538, 0.00011556]
  T_cam_imu:
  - [0.0, 1.0, 0.0, 0.07]
  - [-1.0, 0.0, 0.0, -0.01]
  - [0.0, 0.0, 1.0, 0.004]
  - [0.0, 0.0, 0.0, 1.0]
  timeshift_cam_imu: -8.121e-05
  rostopic: /cam0/image_raw
  resolution: [752, 480]
"""


def test_parse_results_both_formats(tmp_path):
    a = tmp_path / "a.txt"
    a.write_text("Reprojection error (cam0) [px]:     mean 0.39, median 0.31, std: 0.30\n"
                 "Gyroscope error (imu0):        mean 32.6, median 23.0, std: 31.6\n"
                 "Accelerometer error (imu0):    mean 0.80, median 0.71, std: 0.47\n")
    r = mr.parse_results(str(a))
    assert r["reproj_mean_px"] == 0.39 and r["gyro_err_mean"] == 32.6 and r["acc_err_std"] == 0.47
    b = tmp_path / "b.txt"
    b.write_text("cam0 (/cam0/image_raw):\n  reprojection error: [0.000001, -0.000002] +- [0.3, 0.4]\n")
    r = mr.parse_results(str(b))
    assert abs(r["reproj_std_px"] - 0.5) < 1e-9


def test_mount_check_expected_is_zero(tmp_path):
    c = tmp_path / "c.yaml"
    c.write_text(CHAIN)
    p = mr.parse_camchain(str(c))
    assert mr.mount_check(p["T_cam_imu"]) < 1e-6
    assert p["fx"] == 461.629 and p["width"] == 752


def test_report_end_to_end(tmp_path):
    c = tmp_path / "c.yaml"
    c.write_text(CHAIN)
    out = tmp_path / "out"
    subprocess.run([sys.executable, os.path.join(HERE, "make_report.py"), "--camchain", str(c),
                    "--out-dir", str(out)], check=True, capture_output=True)
    prm = json.load(open(out / "camera_params.json"))
    assert abs(prm["timeshift_cam_imu_s"] + 8.121e-05) < 1e-12
    assert (out / "calibration_report.md").exists()
