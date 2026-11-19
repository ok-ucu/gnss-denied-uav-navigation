"""One bounded redetection of a failed match on the same compensated image pair."""
import time

from vio_scale_compensation import to_previous_coordinates


def prefer_redetection(previous_inliers, min_inliers, scale):
    """Try distributed recovery first after weak support and strong shrinkage."""
    return (previous_inliers is not None and previous_inliers < 3 * min_inliers
            and scale is not None and scale < .85)


def redetect(frontend, previous, gray, dt, height, gyro, rotation, scale, deadline):
    import cv2
    import numpy as np
    from vio_frontend import FlowResult

    if (previous is None or scale is None or gray.std() < frontend.min_contrast
            or not np.isfinite(dt) or dt <= 0 or not np.isfinite(height) or height <= 0
            or time.monotonic() >= deadline):
        return None
    p0 = cv2.goodFeaturesToTrack(previous, min(frontend.max_corners, 80), .005, 8)
    if p0 is None or len(p0) < frontend.min_inliers or time.monotonic() >= deadline:
        return None
    p1, forward, _ = cv2.calcOpticalFlowPyrLK(previous, gray, p0, None, **frontend.lk)
    if p1 is None or time.monotonic() >= deadline:
        return None
    # Points already rejected by forward LK cannot pass the round-trip gate.
    valid = forward.ravel() == 1
    if valid.sum() < frontend.min_inliers:
        return None
    p0, p1 = p0[valid], p1[valid]
    back, backward, _ = cv2.calcOpticalFlowPyrLK(gray, previous, p1, None, **frontend.lk)
    if back is None or time.monotonic() >= deadline:
        return None
    good = ((backward.ravel() == 1) & (np.linalg.norm(p0-back, axis=2).ravel() < 1.))
    if good.sum() < frontend.min_inliers:
        return None
    a, b = p0[good, 0], p1[good, 0]
    affine, mask = cv2.estimateAffinePartial2D(a, b, method=cv2.RANSAC, ransacReprojThreshold=1.5)
    if affine is None or mask is None or not np.isfinite(affine).all():
        return None
    mask = mask.ravel().astype(bool)
    count = int(mask.sum())
    if count < frontend.min_inliers:
        return None
    h, w = gray.shape
    points = to_previous_coordinates(a[mask], gray.shape, scale)
    cells = {(min(3, max(0, int(x*4/w))), min(2, max(0, int(y*3/h)))) for x,y in points}
    if len(cells) < 6 or min(np.ptp(points, axis=0)/[w,h]) < .4:
        return None
    centre = np.array([w/2, h/2])
    flow = (affine[:, :2]@centre + affine[:, 2] - centre)/dt
    residual = b[mask] - (a[mask]@affine[:, :2].T + affine[:, 2])
    scatter = np.sqrt(np.mean(np.sum(residual**2, axis=1))/2.)
    depth = height/max(float(rotation[2, 2]), .5)
    sigma_px = max(1.25*scatter/np.sqrt(count), frontend.min_sigma_px)
    sigma_v = float(depth/frontend.fx*sigma_px/dt)
    camera = -(depth/frontend.fx)*(flow-frontend.fx*np.asarray(gyro)[:2])
    velocity = (rotation@[-camera[1], camera[0], 0.])[:2]
    if not np.isfinite(velocity).all() or not np.isfinite(sigma_v) or time.monotonic() >= deadline:
        return None
    return FlowResult(velocity, count, float(np.clip(np.hypot(1.,sigma_v/frontend.sigma_ref),1.,10.)), sigma_v)
