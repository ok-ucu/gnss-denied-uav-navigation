"""Pytest інтеграція для регресійного тесту симуляції Gazebo."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from regression_test import run_regression_test


def test_simulation_regression():
    dataset_path = os.path.join(HERE, "..", "recordings", "sample_flight")
    cam_params = os.path.join(HERE, "..", "calibration", "camera_params.json")
    passed, stats = run_regression_test(dataset_path, cam_params)
    assert passed, f"Регресійний тест не пройшов: {stats}"
    assert stats["tracking_ratio"] >= 85.0
    assert stats["mean_inliers"] >= 30.0
