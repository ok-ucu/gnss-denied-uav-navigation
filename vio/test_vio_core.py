import numpy as np

from vio_core import G, QualityMonitor, VioEKF, rot_body_to_ned


def run_straight(seconds=20.0, vis=True, seed=1):
    """Дрон летить на північ 2 м/с, літак рівний. Акселерометр з шумом."""
    rng = np.random.default_rng(seed)
    ekf = VioEKF()
    R = rot_body_to_ned(0, 0, 0)
    dt = 0.02
    for i in range(int(seconds / dt)):
        f = np.array([0.0, 0.0, -G]) + rng.normal(0, 0.1, 3)
        ekf.predict(f, R, dt)
        if vis and i % 2 == 0:
            ekf.update_velocity(np.array([2.0, 0, 0]) + rng.normal(0, 0.05, 3))
    return ekf


def test_rotation_is_orthonormal():
    R = rot_body_to_ned(0.3, -0.2, 1.0)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)


def test_velocity_converges_to_visual():
    ekf = run_straight()
    assert abs(ekf.x[3] - 2.0) < 0.1
    assert abs(ekf.x[0] - 40.0) < 2.0  # ~2 м/с * 20 с


def test_outlier_is_rejected():
    ekf = run_straight()
    ok, nis = ekf.update_velocity(np.array([30.0, 0, 0]))
    assert not ok and nis > ekf.gate_nis


def test_covariance_grows_without_vision():
    with_vis = run_straight(vis=True)
    without = run_straight(vis=False)
    assert np.trace(without.P[0:3, 0:3]) > 10 * np.trace(with_vis.P[0:3, 0:3])


def test_quality_drops_when_stale_and_recovers():
    m = QualityMonitor()
    for k in range(20):
        m.push(True, k * 0.05)
    assert m.quality(1.0) > 0.9
    assert m.quality(3.0) == 0.0
    m.push(True, 3.0)
    assert m.quality(3.05) > 0.0
    assert QualityMonitor.inflate(0.0) == 10.0
