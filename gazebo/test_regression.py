"""Pytest інтеграція для регресійного тесту симуляції Gazebo."""
import os
import sys

import pytest

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
    # sample_flight: 2 m/s NORTH. Checks the direction too (a reversed camera axis gives -2).
    assert stats["v_mean_x"] == pytest.approx(2.0, abs=0.3), f"forward speed {stats['v_mean_x']:.2f} m/s, expected +2.0"
    assert abs(stats["v_mean_y"]) <= 0.3, f"lateral speed {stats['v_mean_y']:.2f} m/s, expected 0"


def test_frontend_selection_is_explicit():
    from regression_test import FallbackFlowFrontend, make_frontend

    production = make_frontend(461.629)
    assert type(production).__module__ == "vio_frontend"
    assert type(production).__name__ == "FlowFrontend"
    assert isinstance(make_frontend(461.629, synthetic_fallback=True), FallbackFlowFrontend)


def test_missing_production_frontend_does_not_fall_back(monkeypatch):
    from regression_test import make_frontend

    monkeypatch.setitem(sys.modules, "vio_frontend", None)
    with pytest.raises(ModuleNotFoundError, match="vio_frontend"):
        make_frontend(461.629)
