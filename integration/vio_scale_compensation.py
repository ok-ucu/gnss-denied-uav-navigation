"""Height-predicted image scale compensation for the downward camera.

Root cause it addresses (run 20261117-flight6f32c08e7aaf): at liftoff and
touchdown the camera is 0.2-0.6 m above ground, so a 0.5-0.8 m/s climb changes
image scale by 10-18 % per 10 Hz frame. Pyramidal LK models each 21x21 window
as a pure translation; past ~10 % scale change its forward-backward check fails
and frontend inliers collapsed (1.07->94, 1.11->49, 1.16->17, 1.17->none).

The frame-to-frame scale is not unknown: the camera height is already an
estimator input. Magnifying the PREVIOUS image about the image centre by
s = depth_prev/depth_now leaves only the unknown translation for LK to solve.

Why the velocity is unchanged: with x' = c + s(x - c) the true mapping
previous->current is M_true = M_residual o S. The frontend and audit both
measure flow at the centre c, and S(c) = c, so M_true(c) - c == M_residual(c) - c.
The previous image (not the current one) is warped for exactly this reason:
magnifying the current image would scale the translation by 1/s.

No thresholds change. If the prediction is unusable the pair is left
uncompensated and the existing gates decide.
"""
import math

# Outside this range the height input itself is suspect (a 2x height change in
# one 100 ms frame is not flight); leave the pair alone and let the gates judge.
MIN_SCALE, MAX_SCALE = .5, 2.


def predicted_scale(previous_depth, depth):
    """Image magnification previous->current for a downward camera, or None."""
    if previous_depth is None or depth is None:
        return None
    if not (math.isfinite(previous_depth) and math.isfinite(depth)) or previous_depth <= 0 or depth <= 0:
        return None
    scale = previous_depth/depth
    return scale if MIN_SCALE <= scale <= MAX_SCALE else None


def centre(shape):
    """Same centre convention as vio_frontend.FlowFrontend (w/2, h/2)."""
    return shape[1]/2., shape[0]/2.


def compensate_previous(previous, scale):
    """Return previous magnified by scale about the image centre.

    BORDER_REPLICATE (only reached when scale < 1, i.e. climbing) produces
    rank-1 streaks with no corner response, so goodFeaturesToTrack does not
    seed features in the band that has no real content.
    """
    if previous is None or scale is None or abs(scale-1.) < 1e-6:
        return previous
    import cv2
    import numpy as np
    cx, cy = centre(previous.shape)
    m = np.float32([[scale, 0, cx*(1-scale)], [0, scale, cy*(1-scale)]])
    return cv2.warpAffine(previous, m, (previous.shape[1], previous.shape[0]),
                          flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


def to_previous_coordinates(points, shape, scale):
    """Map points from the compensated previous image back to the original."""
    if scale is None or abs(scale-1.) < 1e-6:
        return points
    cx, cy = centre(shape)
    return (points-[cx, cy])/scale+[cx, cy]
