"""Unit tests for camera capture and configuration."""
import os
import sys
import time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from capture import Camera, CaptureThread, load_config


def test_load_config():
    cfg_path = os.path.join(HERE, "camera_config.yaml")
    cfg = load_config(cfg_path)
    assert "width" in cfg
    assert "height" in cfg
    assert "fps" in cfg
    assert int(cfg["width"]) == 640
    assert int(cfg["height"]) == 480


def test_synthetic_camera_read():
    cfg = {
        "backend": "synthetic",
        "width": 320,
        "height": 240,
        "fps": 60,
    }
    cam = Camera(cfg)
    ts = []
    frames = []
    for _ in range(5):
        t_ns, frame = cam.read()
        assert t_ns is not None
        assert frame is not None
        assert frame.shape == (240, 320)
        assert frame.dtype == np.uint8
        ts.append(t_ns)
        frames.append(frame)

    cam.release()
    assert np.all(np.diff(ts) > 0)


def test_capture_thread():
    cfg = {
        "backend": "synthetic",
        "width": 320,
        "height": 240,
        "fps": 100,
    }
    cam = Camera(cfg)
    received = []

    def on_frame(t_ns, img):
        received.append((t_ns, img))

    th = CaptureThread(cam, on_frame)
    th.start()
    time.sleep(0.15)
    th.stop()

    assert len(received) >= 3
    for t_ns, img in received:
        assert isinstance(t_ns, int)
        assert img.shape == (240, 320)
