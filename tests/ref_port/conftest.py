"""Shared setup for tests ported from robocin/open_vins_ardupilot.

Reference: https://github.com/robocin/open_vins_ardupilot (GPL-3.0), commit
5f7318a1732212ebf64e1b6ddb8e72fc04f13e0a. No reference code is copied; each test
reimplements the *behaviour* the reference checks (or enforces at runtime) for
our Python pipeline and links the source it was derived from.

Run:  python3 -m pytest tests/ref_port -v -rA
Live SITL tests are marked @pytest.mark.sitl and skipped unless REF_PORT_SITL=1.
"""
import os
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
REF = "https://github.com/robocin/open_vins_ardupilot/blob/5f7318a1732212ebf64e1b6ddb8e72fc04f13e0a/"
SAMPLE_FLIGHT = REPO / "recordings" / "sample_flight"
ABE2_RUN = REPO / "integration" / "phase1_runs" / "20261118-flightabe2d67e5306" / "evidence"

# Same import roots the existing suites use (vio/, integration/, dataset/).
for sub in ("vio", "integration", "dataset", "sitl-stand", "vio_offline"):
    p = str(REPO / sub)
    if p not in sys.path:
        sys.path.insert(0, p)

# Values our deployment uses (sources cited where each test uses them).
SAMPLE_FX = 461.629                 # calibration/camera_params.json, dataset generator
WATCHDOG_S = 0.5                    # integration/phase1_transport.py:521 (500_000_000 ns)
CAMERA_PERIOD_S = 0.1               # Gazebo camera 10 Hz (run_vio_stream.py frame grid)


def pytest_configure(config):
    config.addinivalue_line("markers", "sitl: needs a live ArduPilot SITL on udpin:127.0.0.1:14551 (REF_PORT_SITL=1)")
    config.addinivalue_line("markers", "edge: edge case ported from a reference runtime guard")


def pytest_collection_modifyitems(config, items):
    if os.environ.get("REF_PORT_SITL") == "1":
        return
    skip = pytest.mark.skip(reason="live SITL test; set REF_PORT_SITL=1 with SITL running on udpin:127.0.0.1:14551")
    for item in items:
        if "sitl" in item.keywords:
            item.add_marker(skip)


def ground_texture(h=480, w=640, seed=7, margin=200):
    """Random blob texture like dataset/generate_sample_dataset.py (rich corners)."""
    import cv2
    import numpy as np
    rng = np.random.default_rng(seed)
    big = rng.integers(30, 220, (h + 2 * margin, w + 2 * margin), dtype=np.uint8)
    for _ in range(150):
        cx, cy = rng.integers(20, w + 2 * margin - 20), rng.integers(20, h + 2 * margin - 20)
        cv2.circle(big, (int(cx), int(cy)), int(rng.integers(8, 40)), int(rng.integers(0, 255)), -1)
    return cv2.GaussianBlur(big, (0, 0), 2.0)


def crop(texture, dx=0.0, dy=0.0, h=480, w=640, margin=200):
    """Window of the texture shifted so that ground content moves by (+dx, +dy) px in the frame."""
    import cv2
    import numpy as np
    m = np.float32([[1, 0, dx - margin], [0, 1, dy - margin]])
    return cv2.warpAffine(texture, m, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)


@pytest.fixture(scope="session")
def texture():
    return ground_texture()
