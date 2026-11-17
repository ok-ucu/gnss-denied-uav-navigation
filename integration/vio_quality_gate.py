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


def covariance_ok(matrix, size, max_variance=1.):
    import numpy as np
    p = np.asarray(matrix, dtype=float)
    return (p.shape == (size, size) and np.isfinite(p).all()
            and np.allclose(p, p.T, rtol=0, atol=1e-10)
            and np.linalg.eigvalsh(p).min() >= -1e-10
            and np.diag(p).max() <= max_variance)


class QualityGate:
    """Three consecutive ready frames spanning >=250ms before publication.

    Image/clock age <=200ms simulation, aligned IMU <=5ms, actual gaps <=20ms.
    Wall watchdog 500ms stops availability even if simulation clock stalls.
    A failure clears readiness immediately; no cached odometry is emitted.
    """
    def __init__(self):
        self.ready = False
        self.since = None
        self.count = 0
        self.last_t = None
        self.transitions = []

    def block(self, reason, t, wall):
        previous = self.ready
        self.ready = False
        self.since = None
        self.count = 0
        if previous or not self.transitions or self.transitions[-1]['reason'] != reason:
            self.transitions.append(dict(t=t, wall=wall, ready=False, reason=reason))
        return False

    def assess(self, t, wall, *, imu_age, frame_age, clock_age, max_imu_gap,
               inliers, contrast, covariance, state_finite, audit_ok, ekf_ok, frame_gap=.1):
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
                  (state_finite, 'nonfinite state'), (ekf_ok, 'EKF not ready'),
                  (covariance_ok(covariance, 9), 'covariance'),
                  (audit_ok, 'distributed flow disagreement/unavailable')]
        for ok, reason in checks:
            if not ok:
                return self.block(reason, t, wall)
        if self.since is None:
            self.since = t
        self.count += 1
        if not self.ready and self.count >= 3 and t-self.since >= .25-1e-9:
            self.ready = True
            self.transitions.append(dict(t=t, wall=wall, ready=True, reason='readiness passed'))
        return self.ready


def distributed_flow(previous, gray, dt, height, fx, gyro, rotation, frontend_velocity):
    """Independent spatial-support audit; no GT, no knowledge of shadow position.

    Equal feature budget in 12 image cells prevents one textured moving patch
    monopolizing detection. Require affine RANSAC inliers in >=6 cells and a
    span >=40% in BOTH image axes; centre-flow agreement <=1.5 pixels (the
    frontend's own RANSAC tolerance). This cannot detect a whole-image mover.
    """
    import cv2
    import numpy as np
    if previous is None or not math.isfinite(dt) or dt <= 0:
        return {'ok': False, 'reason': 'no previous comparable frame'}
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
    support = a[inliers.ravel().astype(bool)]
    cells = {(min(3, int(x*4/w)), min(2, int(y*3/h))) for x, y in support}
    span = np.ptp(support, axis=0)/[w, h]
    flow = (m[:, :2]@np.array([w/2, h/2])+m[:, 2]-[w/2, h/2])/dt
    depth = height/max(rotation[2, 2], .5)
    camera_v = -(depth/fx)*(flow-fx*np.asarray(gyro)[:2])
    velocity = (rotation@[-camera_v[1], camera_v[0], 0])[:2]
    disagreement = float(np.linalg.norm(velocity-frontend_velocity)*fx*dt/depth)
    ok = len(cells) >= 6 and min(span) >= .4 and disagreement <= 1.5
    return dict(ok=bool(ok), cells=len(cells), span=span.tolist(),
                disagreement_px=disagreement, velocity_xy=velocity.tolist(),
                reason='agreement' if ok else 'ambiguous spatial support/flow')
