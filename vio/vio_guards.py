"""Caller-side guards around the hash-pinned VioEKF and FlowFrontend.

vio_core.py and vio_frontend.py are pinned (commit d33f660, see vio_offline/vio_source.py
and integration/linux_gpu/prepare_vio.py) and must not change, so the checks that OpenVINS
performs inside its estimator are applied here, by the code that feeds them (vio_ekf.py).
Reference behaviour (robocin/open_vins_ardupilot, ported in tests/ref_port/):

* Propagator::propagate_and_clone refuses to propagate at the same time or backwards, and
  no non-finite reading is propagated                                      -> safe_predict()
* ROS2Visualizer::callback_monocular drops an image closer than 1/track_frequency to the
  previous one, and drops images it cannot convert; VioManager drops out-of-order images
                                                                           -> FrameAdmission
* StaticInitializer refuses to initialise from an inconsistent window      -> VelocityInitGuard

The phase-1 flight source (integration/run_vio_stream.py) has its own equivalents:
vio_imu_preintegration.advance() (dt > 0, finite IMU), the 10 Hz camera grid plus the gate's
frame-gap check, and vio_flow_consistency (first flow checked against the IMU prior).
"""
import math

import numpy as np

# Frames closer than this are dropped BEFORE the frontend, so they never become its reference
# image. 12.5 ms = 75 % of a 60 fps period, so 60/30/20/10 Hz streams (incl. float jitter of
# frame_idx/fps stamps) are never affected, while near-duplicate pairs (~1 ms) are dropped.
MIN_FRAME_DT_S = 0.0125
# Velocity initialisation: a plausible speed, confirmed by a second, consistent measurement.
MAX_INIT_SPEED_MS = 15.0
INIT_AGREEMENT_MS = 0.5


def _finite(a):
    return bool(np.all(np.isfinite(np.asarray(a, dtype=float))))


def safe_predict(ekf, f_body, rotation, dt, kinematic=False):
    """Propagate only for a finite dt > 0 and finite inputs. Returns True if it propagated;
    otherwise the state is left untouched (OpenVINS refuses instead of integrating backwards)."""
    try:
        dt = float(dt)
    except (TypeError, ValueError):
        return False
    if not (math.isfinite(dt) and dt > 0.0):
        return False
    if kinematic:
        ekf.predict_kinematic(dt)
        return True
    if f_body is None or rotation is None or not (_finite(f_body) and _finite(rotation)):
        return False
    ekf.predict(np.asarray(f_body, dtype=float), np.asarray(rotation, dtype=float), dt)
    return True


class FrameAdmission:
    """Decides whether a frame may reach the frontend at all.

    A refused frame is not a tracking failure: it is dropped before FlowFrontend.process(),
    so the frontend keeps its previous reference image and time."""

    def __init__(self, min_dt=MIN_FRAME_DT_S):
        self.min_dt = min_dt
        self.last_t = None
        self.reason = None
        self.dropped = 0

    def admit(self, gray, t):
        reason = None
        if gray is None or getattr(gray, "size", 0) == 0 or getattr(gray, "ndim", 0) != 2:
            reason = "empty or non-grayscale image"
        elif t is None or not math.isfinite(t):
            reason = "non-finite frame time"
        elif self.last_t is not None and t - self.last_t < self.min_dt:
            reason = "out of order" if t <= self.last_t else "closer than minimum frame interval"
        self.reason = reason
        if reason is not None:
            self.dropped += 1
            return False
        self.last_t = t
        return True


class VelocityInitGuard:
    """Feeds velocity measurements to VioEKF without letting one outlier initialise it.

    VioEKF takes its FIRST velocity measurement as truth (vel_init). Until then a measurement
    must be finite, plausible (<= MAX_INIT_SPEED_MS) and confirmed by the next one within
    INIT_AGREEMENT_MS; only the confirming measurement reaches the EKF. Afterwards every
    finite measurement goes to the EKF's own NIS gate unchanged."""

    def __init__(self, max_speed=MAX_INIT_SPEED_MS, agreement=INIT_AGREEMENT_MS):
        self.max_speed = max_speed
        self.agreement = agreement
        self.candidate = None

    def _fuse(self, ekf, v, update, sigma_scale):
        v = np.asarray(v, dtype=float)
        if not _finite(v) or not math.isfinite(sigma_scale):
            return False, float("nan")
        if ekf.vel_init:
            return update(v, sigma_scale)
        if np.linalg.norm(v) > self.max_speed:
            self.candidate = None
            return False, float("nan")
        if self.candidate is None or self.candidate.shape != v.shape \
                or np.linalg.norm(v - self.candidate) > self.agreement:
            self.candidate = v
            return False, float("nan")
        self.candidate = None
        return update(v, sigma_scale)

    def fuse_xy(self, ekf, v_xy, sigma_scale=1.0):
        return self._fuse(ekf, v_xy, ekf.update_velocity_xy, sigma_scale)

    def fuse(self, ekf, v, sigma_scale=1.0):
        return self._fuse(ekf, v, ekf.update_velocity, sigma_scale)
