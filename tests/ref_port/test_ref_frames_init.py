"""Frame conventions and initialisation / re-initialisation.

Ported from (reimplemented, not copied):
  - ov_repub/src/republisher.cpp: the odometry orientation is rotated and re-normalised
    (q.normalize()) and position is transformed with the SAME rotation.
  - ROS2Visualizer::visualize_odometry: pose in world frame, twist in body frame,
    covariance ordered position-then-orientation.   ov_msckf/src/ros/ROS2Visualizer.cpp#L283-L324
  - StaticInitializer::initialize: initialisation needs a consistent IMU window and
    is refused otherwise.          ov_init/src/static/StaticInitializer.cpp#L37-L115
  - VioManager: a failed/rejected update never forces a bad state; re-init is explicit.
Our components: vio/vio_core.py, vio/vio_ros_out.py, integration/vio_geometry.py,
integration/vio_quality_gate.py ekf_ready, integration/vio_sensor_capture.py recovery_ready.
"""
import itertools
import math

import numpy as np
import pytest

from conftest import REF
from vio_core import VioEKF, rot_body_to_ned
from vio_ros_out import build_covariances, quat_to_rot, rot_to_quat

SRC_REPUB = REF + "ov_repub/src/republisher.cpp"
SRC_STATIC = REF + "ov_init/src/static/StaticInitializer.cpp#L37-L115"

ANGLES = [-math.pi + 1e-3, -2.0, -math.pi / 2 + 1e-6, -0.3, 0.0, 0.3, math.pi / 2 - 1e-6, 2.5, math.pi - 1e-3]


# ----------------------------------------------------------------- edge cases
@pytest.mark.edge
def test_quaternion_roundtrip_large_attitudes():
    """Large attitude incl. near gimbal lock and +-180 deg: R -> q -> R, |q|=1, w>=0."""
    worst = (0.0, None)
    for roll, pitch, yaw in itertools.product(ANGLES, ANGLES[2:7], ANGLES):
        r = rot_body_to_ned(roll, pitch, yaw)
        q = rot_to_quat(r)
        err = float(np.abs(quat_to_rot(q) - r).max())
        assert abs(np.linalg.norm(q) - 1) < 1e-12 and q[3] >= 0, f"[{SRC_REPUB}] q={q} not unit / w<0"
        worst = max(worst, (err, (roll, pitch, yaw)))
    assert worst[0] < 1e-9, f"quaternion round-trip error {worst[0]:.2e} at rpy={worst[1]}"


@pytest.mark.edge
def test_frd_ned_euler_convention():
    """FRD body -> NED: +pitch raises the nose (body x gets -z NED), +roll drops the right wing."""
    r = rot_body_to_ned(0.0, 0.5, 0.0)
    assert r[2, 0] == pytest.approx(-math.sin(0.5)), f"pitch up: body x z-component {r[2, 0]}"
    r = rot_body_to_ned(0.5, 0.0, 0.0)
    assert r[2, 1] == pytest.approx(math.sin(0.5)), f"roll right: body y z-component {r[2, 1]}"
    r = rot_body_to_ned(0.0, 0.0, math.pi / 2)
    assert np.allclose(r @ [1, 0, 0], [0, 1, 0]), "yaw 90 deg: body forward must point east"


@pytest.mark.edge
def test_twist_covariance_in_body_frame_and_unknown_angular():
    """Ref publishes twist in the body frame; ours: twist linear block = R^T P_v R, angular unknown."""
    r = rot_body_to_ned(0.2, -0.4, 2.0)
    p_vel = np.diag([0.01, 0.04, 0.09])
    pose, twist = build_covariances(r, np.eye(3) * 0.5, p_vel, 3e-4)
    t = np.asarray(twist).reshape(6, 6)
    assert np.allclose(t[:3, :3], r.T @ p_vel @ r), "twist linear covariance not rotated into body frame"
    assert np.allclose(np.diag(t)[3:], -1), "angular velocity must be flagged unknown (-1)"
    p = np.asarray(pose).reshape(6, 6)
    assert np.allclose(np.diag(p), [0.5] * 3 + [3e-4] * 3), "pose covariance order must be x,y,z,roll,pitch,yaw"


@pytest.mark.edge
@pytest.mark.parametrize("bad", ["nan_velocity", "reflection", "wrong_shape"])
def test_lever_arm_transform_rejects_bad_inputs(bad):
    from vio_geometry import camera_to_body_xy
    v, r, w, lever = np.array([1.0, 0.0]), np.eye(3), np.zeros(3), np.array([0.1, 0.0, 0.05])
    if bad == "nan_velocity":
        v = np.array([np.nan, 0.0])
    elif bad == "reflection":
        r = np.diag([1.0, 1.0, -1.0])
    else:
        v = np.zeros(3)
    with pytest.raises(ValueError):
        camera_to_body_xy(v, r, w, lever)


@pytest.mark.edge
@pytest.mark.xfail(strict=True, reason="vio/vio_core.py is hash-pinned; vio_ekf.py feeds it through vio_guards.VelocityInitGuard - see test_velocity_init_guard_*")
def test_first_velocity_outlier_not_taken_as_truth():
    """Ref StaticInitializer refuses to initialise on an inconsistent window. Ours (VioEKF,
    used directly by vio/vio_ekf.py) takes the FIRST flow velocity as truth (vel_init)."""
    ekf = VioEKF()
    ekf.update_velocity_xy([25.0, -18.0])                # e.g. tiny-dt / shadow outlier
    truths = []
    for _ in range(5):                                   # 0.25 s of correct hover measurements
        ekf.predict(np.array([0, 0, -9.80665]), np.eye(3), 0.05)
        ekf.update_velocity_xy([0.0, 0.0])
        truths.append(np.linalg.norm(ekf.x[3:5]))
    assert truths[-1] < 1.0, (
        f"[{SRC_STATIC}] first velocity measurement (25,-18) m/s was accepted unconditionally "
        f"(vio_core.py:125-128) and 5 correct zero-velocity measurements were then NIS-rejected: "
        f"|v| stays {truths[-1]:.1f} m/s; recovery only after 15 rejections (max_rejects) via forced reset")


@pytest.mark.edge
@pytest.mark.parametrize("first", [[25.0, -18.0], [np.nan, 0.0], [3.0, 0.0]], ids=["implausible", "nan", "unconfirmed"])
def test_velocity_init_guard_rejects_unconfirmed_first_velocity(first):
    """Ref StaticInitializer refuses an inconsistent initialisation (#L37-L115). vio_ekf.py feeds the
    pinned VioEKF through VelocityInitGuard: an outlier first measurement never initialises it."""
    from vio_guards import VelocityInitGuard
    ekf, guard = VioEKF(), VelocityInitGuard()
    ok, _ = guard.fuse_xy(ekf, first)
    assert not ok and not ekf.vel_init, f"[{SRC_STATIC}] first measurement {first} initialised the EKF"
    speeds = []
    for _ in range(5):
        ekf.predict(np.array([0, 0, -9.80665]), np.eye(3), 0.05)
        guard.fuse_xy(ekf, [0.0, 0.0])
        speeds.append(float(np.linalg.norm(ekf.x[3:5])))
    assert ekf.vel_init and speeds[-1] < 0.05 and ekf.n_resets == 0, (
        f"after the outlier, correct zero-velocity measurements should initialise and hold |v|~0; "
        f"got |v| sequence {np.round(speeds, 3)}, vel_init={ekf.vel_init}, n_resets={ekf.n_resets}")


def test_velocity_init_guard_moving_start_and_passthrough():
    """A vehicle already moving (accelerating 5 m/s^2 at 20 Hz) initialises on the 2nd frame; after
    initialisation every measurement reaches the EKF's own gate unchanged."""
    from vio_guards import VelocityInitGuard
    ekf, guard = VioEKF(), VelocityInitGuard()
    assert guard.fuse_xy(ekf, [3.00, 0.0])[0] is False
    assert guard.fuse_xy(ekf, [3.25, 0.0])[0] is True and np.allclose(ekf.x[3:5], [3.25, 0.0])
    assert guard.fuse_xy(ekf, [3.30, 0.0])[0] is True
    assert guard.fuse(VioEKF(), [1.0, 0.0, 0.0])[0] is False      # 3-D variant guarded too


@pytest.mark.edge
def test_forced_velocity_reset_starts_new_epoch():
    """Ref: after a failure the system re-initialises and continues. ekf_ready() latches False for
    the estimator object that did a forced reset (by design); run_vio_stream.py must therefore open
    a new epoch (block(..., reset=True) -> continuous-IMU recovery -> fresh VioEKF)."""
    import ast
    from conftest import REPO
    from vio_quality_gate import ekf_ready, needs_new_epoch
    ekf = VioEKF()
    ekf.update_velocity_xy([0.0, 0.0])
    for _ in range(15):
        ekf.update_velocity_xy([30.0, 0.0])              # forces one reset
    assert ekf.n_resets == 1 and not ekf_ready(ekf) and needs_new_epoch(ekf)
    fresh = VioEKF()                                     # what the new epoch builds (run_vio_stream.py recovery)
    fresh.update_velocity_xy([30.0, 0.0])
    assert ekf_ready(fresh) and not needs_new_epoch(fresh)

    tree = ast.parse((REPO / "integration" / "run_vio_stream.py").read_text(encoding="utf-8"))
    wired = False
    for node in ast.walk(tree):
        if isinstance(node, ast.If) and isinstance(node.test, ast.Call) and getattr(node.test.func, "id", None) == "needs_new_epoch":
            for call in ast.walk(node):
                if (isinstance(call, ast.Call) and getattr(call.func, "id", None) == "block"
                        and any(k.arg == "reset" and getattr(k.value, "value", None) is True for k in call.keywords)):
                    wired = True
    assert wired, ("run_vio_stream.py: no `if needs_new_epoch(ekf): block(..., reset=True)` after fusion - "
                   "a forced velocity reset would keep the gate 'EKF not ready' until restart")


@pytest.mark.edge
def test_recovery_requires_continuous_imu_window():
    """Ref StaticInitializer needs a full IMU window; ours: 0.25 s with gaps <= 20 ms."""
    from vio_sensor_capture import recovery_ready
    good = [dict(t=round(i * 0.005, 6)) for i in range(0, 201)]
    assert recovery_ready(good, 1.0, 0.5) is True
    holey = [m for m in good if not 0.85 < m["t"] < 0.88]
    assert recovery_ready(holey, 1.0, 0.5) is False, "25 ms IMU hole inside the window must block recovery"
    assert recovery_ready(good, 0.45, 0.5) is False, "recovery before the deadline must be refused"


# ---------------------------------------------------------------- normal path
def test_velocity_fusion_converges_from_rest():
    ekf = VioEKF()
    rng = np.random.default_rng(0)
    for _ in range(100):
        ekf.predict(np.array([0, 0, -9.80665]), np.eye(3), 0.05)
        ekf.update_velocity_xy(np.array([2.0, 0.0]) + rng.normal(0, 0.05, 2))
    assert np.allclose(ekf.x[3:5], [2.0, 0.0], atol=0.1) and ekf.n_resets == 0
