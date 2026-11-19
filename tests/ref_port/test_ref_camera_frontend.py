"""Camera / optical-flow frontend edge cases.

Ported from OpenVINS runtime guards (reimplemented, not copied):
  - VioManager::feed_measurement_camera 'image received out of order' -> drop
      ov_msckf/src/core/VioManager.cpp#L330-L334
  - ROS2Visualizer::callback_monocular: drop images closer than 1/track_frequency,
      cv_bridge failure -> drop frame (no crash)
      ov_msckf/src/ros/ROS2Visualizer.cpp#L498-L535
  - ov_core/src/test_tracking.cpp / TrackKLT (KLT tracking on real image streams)
Our component: vio/vio_frontend.py FlowFrontend (+ integration/vio_scale_compensation.py).
"""
import numpy as np
import pytest

from conftest import REF, SAMPLE_FX, crop
from vio_frontend import FlowFrontend

SRC_MGR = REF + "ov_msckf/src/core/VioManager.cpp"
SRC_VIS = REF + "ov_msckf/src/ros/ROS2Visualizer.cpp"
DT = 1 / 30
H = 10.0                                  # m, sample_flight height
R = np.eye(3)
GYRO = np.zeros(3)


def v_expected(dy_px_per_s, depth=H, fx=SAMPLE_FX):
    # Frontend mount (vio_frontend.py:18-20): frame +y = body backward, so ground
    # content moving +y means the vehicle flies forward (north for R = I).
    return np.array([depth / fx * dy_px_per_s, 0.0])


# ----------------------------------------------------------------- edge cases
@pytest.mark.edge
def test_out_of_order_frame_rejected(texture):
    fe = FlowFrontend(fx=SAMPLE_FX)
    fe.process(crop(texture, 0, 0), 0.0, H, GYRO, R)
    assert fe.process(crop(texture, 0, 3), DT, H, GYRO, R) is not None
    res = fe.process(crop(texture, 0, 1), DT / 2, H, GYRO, R)
    assert res is None, f"[{SRC_MGR}#L330] frame with t={DT/2:.4f} < previous {DT:.4f} must be dropped; got {res}"


@pytest.mark.edge
def test_duplicate_timestamp_frame_rejected(texture):
    fe = FlowFrontend(fx=SAMPLE_FX)
    fe.process(crop(texture, 0, 0), 0.0, H, GYRO, R)
    res = fe.process(crop(texture, 0, 3), 0.0, H, GYRO, R)
    assert res is None, f"[{SRC_MGR}#L330] duplicate stamp t=0.0 must be dropped (dt=0); got {res}"


@pytest.mark.edge
def test_out_of_order_frame_does_not_corrupt_next_estimate(texture):
    fe = FlowFrontend(fx=SAMPLE_FX)
    fe.process(crop(texture, 0, 0), 0.0, H, GYRO, R)
    fe.process(crop(texture, 0, 3), DT, H, GYRO, R)
    fe.process(crop(texture, 0, 0), 0.0, H, GYRO, R)          # stale replay of frame 0
    res = fe.process(crop(texture, 0, 6), 2 * DT, H, GYRO, R)
    assert res is not None, "valid frame after an out-of-order one was rejected"
    exp = v_expected(90.0)
    assert np.allclose(res.v_xy, exp, atol=0.1), (
        f"[{SRC_MGR}#L330] after a dropped out-of-order frame the next velocity must be unaffected: "
        f"expected {exp.round(3)}, got {np.round(res.v_xy, 3)}")


@pytest.mark.edge
@pytest.mark.xfail(strict=True, reason="vio/vio_frontend.py is hash-pinned; frames closer than MIN_FRAME_DT_S are dropped before it by vio_guards.FrameAdmission - see tests below")
def test_tiny_dt_frame_pair_not_amplified(texture):
    """Ref drops frames closer than 1/track_frequency (ROS2Visualizer.cpp#L501-L506).
    A 1 ms pair with 0.5 px jitter must not become a large velocity measurement."""
    fe = FlowFrontend(fx=SAMPLE_FX)
    fe.process(crop(texture, 0, 0), 0.0, H, GYRO, R)
    res = fe.process(crop(texture, 0, 0.5), 0.001, H, GYRO, R)
    if res is None:
        return
    speed = float(np.linalg.norm(res.v_xy))
    assert speed < 5.0, (
        f"[{SRC_VIS}#L501-L506] frame pair 1 ms apart with 0.5 px jitter produced |v|={speed:.1f} m/s "
        f"(sigma_scale={res.sigma_scale:.1f} -> reported sigma {0.08 * res.sigma_scale:.2f} m/s); "
        f"vio/vio_frontend.py:41 only rejects dt <= 0, there is no minimum frame interval")


@pytest.mark.edge
def test_empty_image_dropped_without_exception():
    """Ref: cv_bridge conversion failure -> PRINT_ERROR and drop the frame (ROS2Visualizer.cpp#L509-L515)."""
    fe = FlowFrontend(fx=SAMPLE_FX)
    empty = np.zeros((0, 0), np.uint8)
    try:
        first = fe.process(empty, 0.0, H, GYRO, R)
        second = fe.process(empty, DT, H, GYRO, R)
    except Exception as exc:  # noqa: BLE001 - the point is that nothing escapes
        pytest.fail(f"[{SRC_VIS}#L509-L515] empty image must be dropped (None); FlowFrontend.process raised "
                    f"{type(exc).__name__}: {str(exc).splitlines()[0][:160]}")
    assert first is None and second is None, f"empty image produced a measurement: {first}, {second}"


@pytest.mark.edge
def test_flat_image_rejected_low_contrast():
    fe = FlowFrontend(fx=SAMPLE_FX)
    flat = np.full((480, 640), 120, np.uint8)
    fe.process(flat, 0.0, H, GYRO, R)
    assert fe.process(flat, DT, H, GYRO, R) is None, "featureless frame must give no measurement"


@pytest.mark.edge
def test_nonpositive_height_rejected(texture):
    fe = FlowFrontend(fx=SAMPLE_FX)
    fe.process(crop(texture, 0, 0), 0.0, 0.0, GYRO, R)
    assert fe.process(crop(texture, 0, 3), DT, 0.0, GYRO, R) is None, "height 0 must give no measurement"


@pytest.mark.edge
def test_liftoff_scale_change_tracked_with_compensation(texture):
    """Liftoff: camera 0.30 -> 0.35 m in one 10 Hz frame (scale 0.857). Ref TrackKLT has the
    same translation-only KLT model; ours compensates the predicted scale first."""
    import cv2
    from vio_scale_compensation import compensate_previous, predicted_scale
    fx, d0, d1, dt, dy = 205.4696, 0.30, 0.35, 0.1, 2.0
    prev = crop(texture, 0, 0)
    s = predicted_scale(d0, d1)
    cx, cy = 320.0, 240.0
    m = np.float32([[s, 0, cx * (1 - s)], [0, s, cy * (1 - s) + dy]])
    cur = cv2.warpAffine(prev, m, (640, 480), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    fe = FlowFrontend(fx=fx)
    fe.process(prev, 0.0, d0, GYRO, R)
    fe.prev = compensate_previous(fe.prev, s)
    res = fe.process(cur, dt, d1, GYRO, R)
    assert res is not None, f"scale {s:.3f} with compensation: frontend lost tracking"
    exp = v_expected(dy / dt, depth=d1, fx=fx)
    assert np.allclose(res.v_xy, exp, atol=0.01), f"expected v={exp.round(4)}, got {np.round(res.v_xy, 4)}"


@pytest.mark.edge
def test_frame_admission_drops_tiny_dt_pair_and_keeps_reference(texture):
    """Ref ROS2Visualizer.cpp#L501-L506: the too-close frame is dropped BEFORE tracking, so the
    frontend's reference stays the earlier frame and the next valid pair is measured correctly."""
    from vio_guards import FrameAdmission
    fe, adm = FlowFrontend(fx=SAMPLE_FX), FrameAdmission()
    results = []
    for t, dy in ((0.0, 0.0), (0.001, 0.5), (DT, 3.0)):
        img = crop(texture, 0, dy)
        results.append(fe.process(img, t, H, GYRO, R) if adm.admit(img, t) else "dropped")
    assert results[1] == "dropped", f"[{SRC_VIS}#L501-L506] 1 ms pair reached the frontend: {results[1]}"
    exp = v_expected(90.0)
    assert results[2] is not None and np.allclose(results[2].v_xy, exp, atol=0.05), (
        f"next frame after the dropped one: expected {exp.round(3)}, got "
        f"{None if results[2] is None else np.round(results[2].v_xy, 3)}")


@pytest.mark.edge
@pytest.mark.parametrize("case", ["out_of_order", "duplicate", "empty", "color", "nan_time"])
def test_frame_admission_rejects(case, texture):
    from vio_guards import FrameAdmission
    adm = FrameAdmission()
    assert adm.admit(crop(texture), 1.0)
    img, t = crop(texture), 1.1
    if case == "out_of_order":
        t = 0.9
    elif case == "duplicate":
        t = 1.0
    elif case == "empty":
        img = np.zeros((0, 0), np.uint8)
    elif case == "color":
        img = np.zeros((480, 640, 3), np.uint8)
    else:
        t = float("nan")
    assert adm.admit(img, t) is False and adm.reason, f"{case} frame admitted"
    assert adm.last_t == 1.0, f"a rejected frame moved the reference time to {adm.last_t}"


@pytest.mark.parametrize("fps", [10, 20, 30, 60])
def test_frame_admission_keeps_normal_rates(fps):
    """frame_idx/fps stamps (vio_ekf.py video playback) at normal rates are never dropped."""
    from vio_guards import FrameAdmission
    adm = FrameAdmission()
    dropped = [k for k in range(300) if not adm.admit(np.ones((4, 4), np.uint8), k / fps)]
    assert not dropped, f"{fps} fps stream: frames {dropped[:5]} dropped"


@pytest.mark.edge
def test_vio_ekf_admits_frames_before_the_frontend():
    import ast
    from conftest import REPO
    src = (REPO / "vio" / "vio_ekf.py").read_text(encoding="utf-8")
    tree = ast.parse(src)
    calls = {n.func.attr: n.lineno for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr in ("admit", "process")}
    assert "admit" in calls and "process" in calls and calls["admit"] < calls["process"], (
        f"vio_ekf.py must call FrameAdmission.admit() before frontend.process(): {calls}")


# ---------------------------------------------------------------- normal path
def test_forward_flight_sign_and_scale(texture):
    """Ref: test_tracking.cpp tracks a real image stream; here a known translation."""
    fe = FlowFrontend(fx=SAMPLE_FX)
    fe.process(crop(texture, 0, 0), 0.0, H, GYRO, R)
    res = fe.process(crop(texture, 0, 3), DT, H, GYRO, R)
    exp = v_expected(90.0)
    assert res is not None and res.n_inliers >= 15, f"tracking failed on a rich texture: {res}"
    assert np.allclose(res.v_xy, exp, atol=0.05), f"expected v_ned_xy={exp.round(3)}, got {np.round(res.v_xy, 3)}"


def test_yawed_vehicle_velocity_rotated_to_ned(texture):
    """Frame conventions (FRD body -> NED): flying forward while yawed 90 deg (east) gives +east."""
    from vio_core import rot_body_to_ned
    fe = FlowFrontend(fx=SAMPLE_FX)
    r = rot_body_to_ned(0.0, 0.0, np.pi / 2)
    fe.process(crop(texture, 0, 0), 0.0, H, GYRO, r)
    res = fe.process(crop(texture, 0, 3), DT, H, GYRO, r)
    exp = np.array([0.0, H / SAMPLE_FX * 90.0])
    assert res is not None and np.allclose(res.v_xy, exp, atol=0.05), (
        f"yaw=90deg forward flight must map to +east in NED: expected {exp.round(3)}, got "
        f"{None if res is None else np.round(res.v_xy, 3)}")
