"""Fail-closed availability gate. No ROS, GT or estimator implementation.

Thresholds are deployment checks, not guarantees of scene rigidity. The image
audit checks spatially distributed track agreement, not semantic shadow identity.
"""
import math
from stage_timing import critical_stage


def fresh(age, limit):
    # ROS stamps have nanosecond resolution. Float subtraction of large sim
    # seconds may differ by <1ns at an EXACT boundary; no millisecond slack.
    return math.isfinite(age) and -1e-9 <= age <= limit+1e-9


def ekf_ready(ekf):
    # Teammate _after_gate() can return accepted=True after forcibly resetting
    # velocity. That is not a normal accepted correction. Latch closed until
    # an explicit new estimator epoch (restart or continuous-IMU recovery).
    return bool(ekf.vel_init and ekf.n_resets == 0)


def needs_new_epoch(ekf):
    """True after a forced velocity reset. ekf_ready() stays False for this estimator
    object, so the caller must start a new estimator epoch (as for IMU gaps) instead of
    leaving the gate latched closed until a restart. OpenVINS likewise re-initialises
    after a failure rather than stopping for good (tests/ref_port, F13)."""
    return bool(ekf.n_resets)


def covariance_ok(matrix, size, max_variance=1.):
    import numpy as np
    p = np.asarray(matrix, dtype=float)
    return (p.shape == (size, size) and np.isfinite(p).all()
            and np.allclose(p, p.T, rtol=0, atol=1e-10)
            and np.linalg.eigvalsh(p).min() >= -1e-10
            and np.diag(p).max() <= max_variance)


# Failures caused by one bad image pair, after which the estimator state is
# untouched (the flow is never fused). Only these may be tolerated (opt-in).
TRANSIENT_IMAGE_REASONS = frozenset({
    'inliers', 'contrast',
    'distributed flow disagreement/unavailable',
    'flow inconsistent with IMU prediction / aliased',
})


class QualityGate:
    """Three consecutive ready frames spanning >=250ms before publication.

    Image/clock age <=200ms simulation, aligned IMU <=5ms, actual gaps <=20ms.
    Wall watchdog 500ms stops availability even if simulation clock stalls.
    A failure clears readiness immediately; no cached odometry is emitted.

    tolerate_single_rejection (audit C2, OPT-IN, default False): at 10 Hz one
    rejected frame costs 4 more frames (500 ms) of readiness, which is the
    whole transport availability watchdog. When enabled, a READY gate may
    survive exactly one rejected frame if that frame failed ONLY image-quality
    checks (TRANSIENT_IMAGE_REASONS) and the caller states the estimator is
    healthy (no forced velocity reset, no recovery in progress). The rejected
    frame itself is still withheld and readiness is False while suspended, so
    nothing is published for it. The next passing frame resumes publication
    immediately; a second consecutive failure of any kind blocks as before.
    The tolerance re-arms only after the normal readiness criterion holds
    again (3 clean frames spanning >=250 ms), so a camera failing every other
    frame still blocks on its second failure.
    Timing, clock, IMU, covariance, state and EKF failures are never tolerated.
    This loosens the readiness gate; it requires the safety owner's approval.
    """
    def __init__(self, tolerate_single_rejection=False):
        self.ready = False
        self.since = None
        self.count = 0
        self.last_t = None
        self.transitions = []
        # Why the most recent frame was withheld (None when it was ready).
        # Diagnostics only: never consulted by any decision.
        self.reason = 'not yet assessed'
        self.tolerate_single_rejection = tolerate_single_rejection is True
        self.suspended = False      # one tolerated rejection is outstanding
        self.recovering = False     # tolerance spent; waiting to re-arm
        self.rearm_since = None
        self.rearm_count = 0
        self.tolerated = 0          # diagnostics: tolerated rejections so far

    def block(self, reason, t, wall):
        previous = self.ready or self.suspended
        self.ready = False
        self.suspended = False
        self.recovering = False     # a full block restarts readiness from zero anyway
        self.since = None
        self.count = 0
        self.reason = reason
        if previous or not self.transitions or self.transitions[-1]['reason'] != reason:
            self.transitions.append(dict(t=t, wall=wall, ready=False, reason=reason))
        return False

    def assess(self, t, wall, *, imu_age, frame_age, clock_age, max_imu_gap,
               inliers, contrast, covariance, state_finite, audit_ok, ekf_ok, frame_gap=.1,
               consistency_ok=True, estimator_healthy=None):
        values = [t, wall, imu_age, frame_age, clock_age, max_imu_gap, inliers, contrast, frame_gap]
        if not all(math.isfinite(v) for v in values):
            return self.block('nonfinite timing/quality', t, wall)
        if self.last_t is not None and t <= self.last_t:
            self.last_t = t
            return self.block('timestamp reset/duplicate', t, wall)
        self.last_t = t
        checks = [(fresh(imu_age, .005), 'IMU alignment'),
                  (fresh(frame_age, .2), 'stale camera'),
                  (fresh(clock_age, .2), 'clock synchronization'),
                  (0 < max_imu_gap <= .02, 'IMU gap'),
                  (frame_gap > 0 and fresh(frame_gap, .3), 'camera gap'),
                  (inliers >= 15, 'inliers'), (contrast >= 8, 'contrast'),
                  (state_finite, 'nonfinite state'),
                  # Audit before EKF: the caller fuses flow only after the audit
                  # passes, so ekf_ok is False whenever the audit fails. Checking
                  # EKF first labelled every audit rejection 'EKF not ready'.
                  # Order changes only the reason; every check still blocks.
                  (audit_ok, 'distributed flow disagreement/unavailable'),
                  # vio_flow_consistency: flow vs IMU prior and texture aliasing.
                  # Also before EKF, which never sees a rejected flow.
                  (consistency_ok, 'flow inconsistent with IMU prediction / aliased'),
                  (ekf_ok, 'EKF not ready'),
                  (covariance_ok(covariance, 9), 'covariance')]
        failures = [reason for ok, reason in checks if not ok]
        if failures:
            # ekf_ok is necessarily False for an image failure (the flow was not
            # fused); estimator_healthy separately vouches for the EKF itself.
            if (self.tolerate_single_rejection and self.ready and not self.suspended
                    and not self.recovering and estimator_healthy is True and failures[0] in TRANSIENT_IMAGE_REASONS
                    and all(r in TRANSIENT_IMAGE_REASONS or r == 'EKF not ready' for r in failures)):
                self.ready = False
                self.suspended = True
                self.recovering = True
                self.rearm_since = None
                self.rearm_count = 0
                self.tolerated += 1
                self.reason = 'tolerated single rejection: ' + failures[0]
                self.transitions.append(dict(t=t, wall=wall, ready=False, reason=self.reason, tolerated=True))
                return False
            return self.block(failures[0], t, wall)
        if self.suspended:
            # The frame after a single tolerated rejection passed every check.
            self.suspended = False
            self.ready = True
            self.transitions.append(dict(t=t, wall=wall, ready=True, reason='resumed after tolerated rejection'))
        if self.recovering:
            if self.rearm_since is None:
                self.rearm_since = t
            self.rearm_count += 1
            if self.rearm_count >= 3 and t-self.rearm_since >= .25-1e-9:
                self.recovering = False
        if self.since is None:
            self.since = t
        self.count += 1
        if not self.ready and self.count >= 3 and t-self.since >= .25-1e-9:
            self.ready = True
            self.transitions.append(dict(t=t, wall=wall, ready=True, reason='readiness passed'))
        self.reason = None if self.ready else (
            'readiness hysteresis %d/3 frames, %.3f/0.25 s' % (self.count, t-self.since))
        return self.ready


def distributed_flow(previous, gray, dt, height, fx, gyro, rotation, frontend_velocity, scale=None):
    """Independent spatial-support audit; no GT, no knowledge of shadow position.

    Equal feature budget in 12 image cells prevents one textured moving patch
    monopolizing detection. Require affine RANSAC inliers in >=6 cells and a
    span >=40% in BOTH image axes; centre-flow agreement <=1.5 pixels (the
    frontend's own RANSAC tolerance). This cannot detect a whole-image mover.

    scale: optional height-predicted magnification (vio_scale_compensation);
    the frontend must be given the same compensation so both see one pair.
    """
    return judge_distributed_flow(measure_distributed_flow(previous, gray, dt, scale),
                                  height, fx, gyro, rotation, frontend_velocity)


def measure_distributed_flow(previous, gray, dt, scale=None):
    """Image-only half of the audit (detection, FB-LK, affine).

    Needs no estimator state, so the source runs it on a worker thread while
    the frontend runs (OpenCV releases the GIL). Thresholds are unchanged.
    """
    import cv2
    import numpy as np
    from vio_scale_compensation import compensate_previous, to_previous_coordinates
    if previous is None or not math.isfinite(dt) or dt <= 0:
        return {'ok': False, 'reason': 'no previous comparable frame'}
    original_shape = previous.shape
    previous = compensate_previous(previous, scale)
    h, w = gray.shape
    points = []
    with critical_stage('audit_detection'):
        for y in range(3):
            for x in range(4):
                x0, x1 = x*w//4, (x+1)*w//4
                y0, y1 = y*h//3, (y+1)*h//3
                p = cv2.goodFeaturesToTrack(previous[y0:y1, x0:x1], 20, .01, 8)
                if p is not None:
                    p += np.float32([x0, y0])
                    points.extend(p)
    if len(points) < 15:
        return {'ok': False, 'reason': 'insufficient distributed features'}
    p0 = np.asarray(points, np.float32).reshape(-1, 1, 2)
    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
    with critical_stage('audit_forward_lk'):
        p1, status, _ = cv2.calcOpticalFlowPyrLK(previous, gray, p0, None, **lk)
    if p1 is None:
        return {'ok': False, 'reason': 'LK unavailable'}
    with critical_stage('audit_backward_lk'):
        back, status_back, _ = cv2.calcOpticalFlowPyrLK(gray, previous, p1, None, **lk)
    if back is None:
        return {'ok': False, 'reason': 'backward LK unavailable'}
    good = status.ravel().astype(bool) & status_back.ravel().astype(bool)
    good &= np.linalg.norm(back-p0, axis=2).ravel() < 1
    a, b = p0[good, 0], p1[good, 0]
    if len(a) < 15:
        return {'ok': False, 'reason': 'insufficient FB tracks'}
    with critical_stage('audit_affine'):
        m, inliers = cv2.estimateAffinePartial2D(a, b, method=cv2.RANSAC,
                                              ransacReprojThreshold=1.5)
    if m is None or inliers is None:
        return {'ok': False, 'reason': 'no distributed affine'}
    # Spatial support is judged where the features are in the REAL previous image.
    support = to_previous_coordinates(a[inliers.ravel().astype(bool)], original_shape, scale)
    oh, ow = original_shape
    cells = {(min(3, max(0, int(x*4/ow))), min(2, max(0, int(y*3/oh)))) for x, y in support}
    span = np.ptp(support, axis=0)/[ow, oh]
    # Centre flow is invariant to the centre-fixed compensation (see module doc).
    flow = (m[:, :2]@np.array([w/2, h/2])+m[:, 2]-[w/2, h/2])/dt
    return dict(measured=True, cells=len(cells), span=span.tolist(), flow=flow, dt=dt,
                scale_compensation=None if scale is None else float(scale))


def judge_distributed_flow(measurement, height, fx, gyro, rotation, frontend_velocity):
    """Estimator-dependent half: compare audit centre flow with the frontend."""
    import numpy as np
    if not measurement.get('measured'):
        return measurement
    dt, flow, cells, span = measurement['dt'], measurement['flow'], measurement['cells'], measurement['span']
    depth = height/max(rotation[2, 2], .5)
    camera_v = -(depth/fx)*(flow-fx*np.asarray(gyro)[:2])
    velocity = (rotation@[-camera_v[1], camera_v[0], 0])[:2]
    disagreement = float(np.linalg.norm(velocity-frontend_velocity)*fx*dt/depth)
    ok = cells >= 6 and min(span) >= .4 and disagreement <= 1.5
    return dict(ok=bool(ok), cells=cells, span=span,
                disagreement_px=disagreement, velocity_xy=velocity.tolist(),
                scale_compensation=measurement['scale_compensation'],
                reason='agreement' if ok else 'ambiguous spatial support/flow')
