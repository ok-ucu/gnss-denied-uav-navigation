"""IMU propagation edge cases.

Ported from the runtime guards of OpenVINS' propagator (reimplemented, not copied):
  - Propagator::propagate_and_clone   (same timestamp / backwards in time)
      ov_msckf/src/state/Propagator.cpp#L33-L50
  - Propagator::propagate_and_clone   assert |(time1-time0) - sum(dt)| < 1e-4
      ov_msckf/src/state/Propagator.cpp#L78-L101
  - Propagator::select_imu_readings   (empty buffer, missing IMU, zero-dt removal)
      ov_msckf/src/state/Propagator.cpp#L268-L392
Our components: vio/vio_core.py VioEKF.predict, integration/vio_imu_preintegration.py.
"""
import math

import numpy as np
import pytest

from conftest import REF
from vio_core import G, VioEKF
import vio_imu_preintegration as pre

SRC_PROP = REF + "ov_msckf/src/state/Propagator.cpp"
LEVEL = [0.0, 0.0, -G]          # FRD specific force at rest


def imu_stream(t0, t1, rate=1000.0, f=LEVEL, gyro=(0.0, 0.0, 0.0)):
    n = int(round((t1 - t0) * rate))
    return [dict(t=round(t0 + i / rate, 9), f=list(f), gyro=list(gyro), q=[0, 0, 0, 1]) for i in range(n + 1)]


def start_state(imu, t0):
    """Initialise exactly like run_vio_stream.py:410-417 (first causal IMU)."""
    state = dict(last_imu=None, filter_t=None, imu_index=0, frame_stamp=None, camera_period=None,
                 reset_after=None, **pre.new_pending())
    idx = max(i for i, m in enumerate(imu) if m["t"] <= t0)
    state.update(last_imu=imu[idx], filter_t=imu[idx]["t"], imu_index=idx + 1)
    return state


class Recorder:
    def __init__(self):
        self.ekf = VioEKF()
        self.dts = []

    def predict(self, f, r, dt):
        self.dts.append(dt)
        self.ekf.predict(f, r, dt)



# ----------------------------------------------------------------- edge cases
@pytest.mark.edge
def test_predict_zero_dt_is_noop():
    """Ref: Propagator::propagate_and_clone 'called again at same timestep' -> no propagation."""
    ekf = VioEKF()
    ekf.x[3:6] = [1.0, -2.0, 0.5]
    x0, p0 = ekf.x.copy(), ekf.P.copy()
    ekf.predict(np.array([1.0, 2.0, -G]), np.eye(3), 0.0)
    assert np.array_equal(ekf.x, x0) and np.allclose(ekf.P, p0), (
        f"[{SRC_PROP}#L37] dt=0 must not change the state; got dx={ekf.x - x0}, dP_max={np.abs(ekf.P - p0).max()}")


@pytest.mark.edge
@pytest.mark.xfail(strict=True, reason="vio/vio_core.py is hash-pinned (d33f660, vio_offline/vio_source.py); the guard is applied by callers instead - see the safe_predict tests below")
def test_predict_refuses_backwards_dt():
    """Ref: Propagator::propagate_and_clone 'trying to propagate backwards in time' -> refuse."""
    ekf = VioEKF()
    ekf.x[3:6] = [2.0, 0.0, 0.0]
    x0, p0 = ekf.x.copy(), ekf.P.copy()
    try:
        ekf.predict(np.array([0.0, 0.0, -G]), np.eye(3), -0.05)
    except ValueError:
        return  # refusing loudly is acceptable
    assert np.array_equal(ekf.x, x0) and np.allclose(ekf.P, p0), (
        f"[{SRC_PROP}#L43] VioEKF.predict(dt=-0.05) must be refused (no state change or ValueError); "
        f"it integrated backwards: position {x0[:3]} -> {ekf.x[:3]}, "
        f"trace(P) {np.trace(p0):.4f} -> {np.trace(ekf.P):.4f}")


@pytest.mark.edge
@pytest.mark.xfail(strict=True, reason="vio/vio_core.py is hash-pinned (d33f660, vio_offline/vio_source.py); the guard is applied by callers instead - see the safe_predict tests below")
def test_predict_nonfinite_imu_does_not_poison_state():
    """Ref: OpenVINS never propagates a non-finite reading (test_profile asserts !isnan);
    one NaN must not make the whole estimator NaN forever."""
    ekf = VioEKF()
    try:
        ekf.predict(np.array([np.nan, 0.0, -G]), np.eye(3), 0.01)
    except ValueError:
        return
    bad = [i for i, v in enumerate(ekf.x) if not math.isfinite(v)]
    assert not bad and np.isfinite(ekf.P).all(), (
        f"VioEKF.predict accepted a NaN specific force: state indices {bad} became non-finite "
        f"(x={ekf.x.tolist()}); expected the sample to be rejected (ValueError) or ignored")


@pytest.mark.edge
@pytest.mark.parametrize("dt", [0.0, -0.05, float("nan"), float("inf")])
def test_safe_predict_refuses_bad_dt(dt):
    """Ref Propagator.cpp#L37-L50, applied by the caller: vio/vio_guards.safe_predict."""
    from vio_guards import safe_predict
    ekf = VioEKF()
    ekf.x[3:6] = [2.0, 0.0, 0.0]
    x0, p0 = ekf.x.copy(), ekf.P.copy()
    for kinematic in (False, True):
        assert safe_predict(ekf, [0.0, 0.0, -G], np.eye(3), dt, kinematic=kinematic) is False
    assert np.array_equal(ekf.x, x0) and np.array_equal(ekf.P, p0), (
        f"[{SRC_PROP}#L43] safe_predict(dt={dt}) changed the state: {x0[:3]} -> {ekf.x[:3]}")


@pytest.mark.edge
@pytest.mark.parametrize("bad", ["f_nan", "R_nan", "R_inf"])
def test_safe_predict_refuses_nonfinite_inputs(bad):
    from vio_guards import safe_predict
    ekf = VioEKF()
    f, r = np.array([0.0, 0.0, -G]), np.eye(3)
    if bad == "f_nan":
        f[0] = np.nan
    else:
        r = r.copy()
        r[0, 1] = np.nan if bad == "R_nan" else np.inf     # e.g. NaN ATTITUDE -> rot_body_to_ned
    assert safe_predict(ekf, f, r, 0.01) is False
    assert np.isfinite(ekf.x).all() and np.isfinite(ekf.P).all(), f"state poisoned by {bad}: {ekf.x}"


@pytest.mark.edge
def test_vio_ekf_routes_every_prediction_through_safe_predict():
    """vio/vio_ekf.py (sitl-stand MAVLink VIO) must not call VioEKF.predict*/update_velocity* directly."""
    import ast
    from conftest import REPO
    tree = ast.parse((REPO / "vio" / "vio_ekf.py").read_text(encoding="utf-8"))
    direct = sorted({(n.func.attr, n.lineno) for n in ast.walk(tree)
                     if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                     and isinstance(n.func.value, ast.Name) and n.func.value.id == "ekf"
                     and n.func.attr in ("predict", "predict_kinematic", "update_velocity", "update_velocity_xy")})
    assert not direct, f"vio_ekf.py calls the pinned EKF directly, bypassing vio_guards: {direct}"


@pytest.mark.edge
def test_duplicate_imu_timestamp_dropped_not_fatal():
    """Ref: Propagator::select_imu_readings removes readings with |dt| < 1e-12 and keeps
    propagating (L368-L381). A duplicated IMU stamp must not abort the frame."""
    imu = imu_stream(0.0, 0.1)
    dup = dict(imu[50])
    imu.insert(51, dup)                      # same stamp twice (t=0.050)
    state, rec = start_state(imu, 0.0), Recorder()
    try:
        pre.frame_step(state, imu, 0.1, rec.predict, lambda q: np.eye(3), step_s=0.005)
    except pre.ImuGapError as exc:
        pytest.fail(f"[{SRC_PROP}#L368-L381] duplicate IMU stamp t=0.050 s should be dropped and propagation "
                    f"continue to t=0.1; integration/vio_imu_preintegration.py:86 raised {type(exc).__name__}"
                    f"('{exc}') which run_vio_stream.py:428-430 turns into a NEW ESTIMATOR EPOCH "
                    f"(reset + >=0.25 s recovery + readiness)")
    assert abs(sum(rec.dts) - 0.1) < 1e-6, f"propagated {sum(rec.dts):.6f} s instead of 0.100 s"


@pytest.mark.edge
def test_out_of_order_imu_sample_not_consumed():
    """Ref: OpenVINS never integrates a reading older than the state (propagate backwards guard).
    Ours must stop BEFORE the bad sample and report it (fail closed)."""
    imu = imu_stream(0.0, 0.05)
    imu.insert(30, dict(imu[10]))            # stale sample from t=0.010 appears at position 30
    state, rec = start_state(imu, 0.0), Recorder()
    count, bad = pre.advance(state, imu, 0, 0.05, rec.predict, lambda q: np.eye(3))
    assert bad is not None and bad["t"] == pytest.approx(0.010), (
        f"advance() should stop at the out-of-order sample t=0.010 and return it; got bad={bad}")
    assert state["filter_t"] == pytest.approx(0.029) and count == 29, (
        f"expected filter_t=0.029 after 29 samples (stopped before the bad one); got "
        f"filter_t={state['filter_t']}, count={count}")


@pytest.mark.edge
def test_empty_imu_buffer_fails_closed_without_propagating():
    """Ref: select_imu_readings 'No IMU measurements ... IMU-CAMERA are likely messed up' -> no propagation."""
    imu = imu_stream(0.0, 0.0)               # only the initial sample
    state, rec = start_state(imu, 0.0), Recorder()
    x0 = rec.ekf.x.copy()
    with pytest.raises(ValueError, match="IMU watermark/alignment"):
        pre.frame_step(state, imu, 0.1, rec.predict, lambda q: np.eye(3))
    assert rec.dts == [] and np.array_equal(rec.ekf.x, x0), (
        f"no IMU after t=0 -> nothing may be propagated; predict was called with dts={rec.dts}")


@pytest.mark.edge
def test_imu_rate_drop_gap_over_20ms_fails_closed():
    """Ref: select_imu_readings interpolates over missing readings; ours deliberately treats a
    >20 ms IMU gap as a new epoch. This pins that behaviour (it is expected to raise)."""
    imu = [m for m in imu_stream(0.0, 0.1) if not 0.040 < m["t"] < 0.070]   # 30 ms hole
    state, rec = start_state(imu, 0.0), Recorder()
    with pytest.raises(pre.ImuGapError):
        pre.frame_step(state, imu, 0.1, rec.predict, lambda q: np.eye(3), step_s=0.005)


# ---------------------------------------------------------------- normal path
def test_propagated_dt_sums_to_frame_interval():
    """Ref: Propagator.cpp#L101 assert(|(time1-time0) - dt_summed| < 1e-4)."""
    imu = imu_stream(0.0, 0.3)
    state, rec = start_state(imu, 0.0), Recorder()
    for t in (0.1, 0.2, 0.3):
        pre.frame_step(state, imu, t, rec.predict, lambda q: np.eye(3), step_s=0.005)
    assert abs(sum(rec.dts) - 0.3) < 1e-4, (
        f"[{SRC_PROP}#L101] sum of propagated dt = {sum(rec.dts):.9f}, expected 0.3 (tol 1e-4)")
    assert all(dt > 0 for dt in rec.dts), f"non-positive dt passed to predict: {[d for d in rec.dts if d <= 0]}"


def test_idle_preintegration_never_passes_next_frame():
    """Ref: OpenVINS only propagates up to the camera time being processed
    (ROS2Visualizer::callback_inertial waits for IMU newer than the image)."""
    imu = imu_stream(0.0, 0.25)
    state, rec = start_state(imu, 0.0), Recorder()
    state.update(frame_stamp=0.0, camera_period=0.1)
    for _ in range(100):
        pre.idle_step(state, lambda i, n: imu[i:i + n], rec.predict, lambda q: np.eye(3), step_s=0.005)
    assert state["filter_t"] < 0.1, f"idle pre-integration reached t={state['filter_t']} >= next frame 0.1"
    pre.frame_step(state, imu, 0.1, rec.predict, lambda q: np.eye(3), step_s=0.005)
    assert state["filter_t"] == pytest.approx(0.1)


def test_level_hover_imu_keeps_velocity_near_zero():
    """Ref: test_sim_meas / test_simulation sanity of simulated IMU at rest (|a| = g, no drift)."""
    imu = imu_stream(0.0, 1.0)
    state, rec = start_state(imu, 0.0), Recorder()
    for k in range(1, 11):
        pre.frame_step(state, imu, k / 10, rec.predict, lambda q: np.eye(3), step_s=0.005)
    v = rec.ekf.x[3:6]
    assert np.linalg.norm(v) < 1e-9, f"level hover IMU (f=[0,0,-g] FRD) produced velocity {v} after 1 s"
