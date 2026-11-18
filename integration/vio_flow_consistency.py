"""Independent check of each flow velocity against the IMU-propagated prior.

Why (run 20261117-flight6f32c08e7aaf, stream.jsonl vs Gazebo GT): in flight,
14 of 55 fused flow velocities were wrong by one fixed displacement,
(-2.0, +2.0) cm (twice that once), whenever the true displacement between the
two images exceeded ~1.45 cm. That is aliasing on a ground texture repeating
every ~2 cm: LK locks onto the neighbouring copy, every feature moves together,
so RANSAC inliers stay high, the frontend's own sigma stays tiny (~0.002 m/s)
and the distributed audit agrees (same images). The EKF's NIS gate cannot
reject it either: its measurement noise floor (sigma_vis 0.08 m/s) is larger
than the 0.14 m/s error.

Two checks, both against the EKF prior after IMU propagation to the frame time
(before this measurement is fused):

1. Kinematic: Mahalanobis distance of (flow - prior) with S = P_prior +
   frontend sigma_v^2, chi-square 2 dof 99.9 % (13.82, the EKF's own xy gate
   value). It differs from the EKF gate only in using the frontend's measured
   precision instead of the fixed 0.08 m/s floor.
2. Alias ambiguity: if the image has a periodic texture (an off-centre local
   maximum of the normalised autocorrelation), a flow measurement is only
   meaningful when the IMU prediction can tell the aliases apart. Along each
   period vector p (pixels) the measured displacement must agree with the
   predicted one within k*sigma (k = 3.29, 1 dof 99.9 %), and |p| must exceed
   2*k*sigma, so no neighbouring alias is also consistent.

Rejecting is fail-closed: the frame is not fused and the gate withholds
odometry. Nothing is unwrapped or corrected. No existing threshold changes.
"""
import math

import numpy as np

CHI2_2DOF_999 = 13.82      # same value as VioEKF.gate_nis_xy
K_1DOF_999 = 3.29
PERIOD_MIN_PEAK = .5       # normalised autocorrelation of a repeating texture
PERIOD_CROP_PX = 128
PERIOD_MAX_OFFSET_PX = 32

_ROTATE = np.array([[0., 1.], [-1., 0.]])   # v_cam = _ROTATE @ v_body_xy (frontend mount)


def texture_periods(gray, crop=PERIOD_CROP_PX, max_offset=PERIOD_MAX_OFFSET_PX,
                    min_peak=PERIOD_MIN_PEAK, limit=3):
    """Period vectors (dx, dy, peak) of a repeating texture in the image centre.

    Normalised, Hann-windowed autocorrelation; a period is an off-centre local
    maximum >= min_peak. A smooth or random texture has no such maximum (its
    central lobe decays monotonically), so it returns [].
    """
    g = np.asarray(gray, dtype=np.float32)
    h, w = g.shape[:2]
    c = min(crop, h, w)
    y0, x0 = (h-c)//2, (w-c)//2
    g = g[y0:y0+c, x0:x0+c]
    g = g-g.mean()
    window = np.outer(np.hanning(c), np.hanning(c)).astype(np.float32)
    g = g*window
    spectrum = np.fft.rfft2(g, s=(2*c, 2*c))
    ac = np.fft.irfft2(spectrum*np.conj(spectrum), s=(2*c, 2*c))
    if not ac[0, 0] > 0:
        return []
    ac = np.fft.fftshift(ac/ac[0, 0])
    m = min(max_offset, c-2)
    centre = c   # fftshift of a 2c array puts zero lag at index c
    roi = ac[centre-m-1:centre+m+2, centre-m-1:centre+m+2]
    core = roi[1:-1, 1:-1]
    neighbours = np.stack([roi[1+dy:roi.shape[0]-1+dy, 1+dx:roi.shape[1]-1+dx]
                           for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx])
    peaks = (core >= neighbours.max(axis=0)) & (core > neighbours.min(axis=0)) & (core >= min_peak)
    peaks[m, m] = False   # zero lag
    found = []
    for iy, ix in zip(*np.nonzero(peaks)):
        dx, dy = int(ix-m), int(iy-m)
        if dy < 0 or (dy == 0 and dx < 0):
            continue      # -p is the same period
        found.append((dx, dy, float(core[iy, ix])))
    found.sort(key=lambda p: (-p[2], p[0]**2+p[1]**2))
    return found[:limit]


def velocity_to_flow_jacobian(rotation, depth, fx):
    """d(image flow px/s)/d(NED xy velocity) for the frontend's model.

    Frontend: v_cam = -(depth/fx)(flow - flow_rot), v_body = [-v_cam1, v_cam0, 0],
    v_ned = R v_body. With v_body_z = 0, v_ned_xy = R[:2,:2] v_body_xy.
    """
    r2 = np.asarray(rotation, float)[:2, :2]
    return -(fx/depth)*_ROTATE@np.linalg.inv(r2)


def flow_px(v_ned_xy, gyro, rotation, depth, fx):
    """Image flow (px/s) the frontend would measure for this NED xy velocity."""
    rot = fx*np.array([float(gyro[0]), float(gyro[1])])
    return rot+velocity_to_flow_jacobian(rotation, depth, fx)@np.asarray(v_ned_xy, float)


def check(measured_xy, sigma_v, prior_xy, prior_cov, gyro, rotation, depth, dt, fx, periods):
    """Both checks; returns a diagnostics dict with 'ok' and 'reason'.

    measured_xy / prior_xy: NED xy velocity in the frontend's frame (lever
    term already removed from both or from neither). prior_cov: 2x2 EKF prior.
    """
    measured = np.asarray(measured_xy, float)
    prior = np.asarray(prior_xy, float)
    cov = np.asarray(prior_cov, float)
    values = [*measured, *prior, *cov.ravel(), sigma_v, depth, dt]
    out = dict(ok=False, periods_px=[list(p) for p in periods])
    if not all(map(math.isfinite, values)) or depth <= 0 or dt <= 0 or sigma_v < 0:
        out['reason'] = 'flow consistency inputs unavailable'
        return out
    innovation = measured-prior
    s = cov+np.eye(2)*sigma_v**2
    try:
        nis = float(innovation@np.linalg.solve(s, innovation))
    except np.linalg.LinAlgError:
        out['reason'] = 'flow consistency covariance singular'
        return out
    out.update(innovation_xy=innovation.tolist(), nis=nis, nis_limit=CHI2_2DOF_999)
    if not nis <= CHI2_2DOF_999:
        out['reason'] = 'flow inconsistent with IMU prediction'
        return out
    jac = velocity_to_flow_jacobian(rotation, depth, fx)
    d_meas = flow_px(measured, gyro, rotation, depth, fx)*dt
    d_pred = flow_px(prior, gyro, rotation, depth, fx)*dt
    d_cov = jac@s@jac.T*dt*dt
    out.update(displacement_px=d_meas.tolist(), predicted_displacement_px=d_pred.tolist())
    alias = []
    for dx, dy, peak in periods:
        p = np.array([dx, dy], float)
        length = float(np.linalg.norm(p))
        unit = p/length
        sigma = math.sqrt(max(float(unit@d_cov@unit), 0.))
        error = float(unit@(d_meas-d_pred))
        alias.append(dict(period_px=[dx, dy], peak=peak, along_error_px=error, along_sigma_px=sigma))
        if not length > 2*K_1DOF_999*sigma:
            out.update(alias=alias, reason='periodic texture: IMU cannot resolve flow alias')
            return out
        if not abs(error) <= K_1DOF_999*sigma:
            out.update(alias=alias, reason='flow aliased on periodic texture')
            return out
    out.update(alias=alias, ok=True, reason=None)
    return out
