from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class FlowResult:
    v_xy: np.ndarray
    n_inliers: int
    sigma_scale: float


class FlowFrontend:
    """Оптичний потік камери, що дивиться вниз.

    Монтаж: вісь x кадру = права сторона дрона (body Y), вісь y кадру = назад
    (мінус body X). Саме з цим монтажем збігаються формули нижче; якщо камеру
    повернуто інакше - треба міняти і flow_rot, і v_body разом."""

    def __init__(self, fx, min_inliers=15, min_contrast=8.0, max_corners=200):
        self.fx = fx
        self.min_inliers = min_inliers
        self.min_contrast = min_contrast
        self.max_corners = max_corners
        self.prev = None
        self.prev_t = None
        self.lk = dict(winSize=(21, 21), maxLevel=3,
                       criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

    def process(self, gray, t, height, gyro_body, R_body_to_ned):
        """gray - кадр (uint8), t - час КАДРУ (с), height - висота над землею по
        вертикалі (м), gyro_body - [gx,gy,gz] рад/с (FRD), R - матриця body->NED.
        Повертає FlowResult або None, якщо кадр непридатний."""
        prev, prev_t = self.prev, self.prev_t
        self.prev, self.prev_t = gray, t
        if prev is None or t <= prev_t or height <= 0.0:
            return None
        if float(gray.std()) < self.min_contrast:
            return None

        p0 = cv2.goodFeaturesToTrack(prev, self.max_corners, 0.01, 8)
        if p0 is None or len(p0) < self.min_inliers:
            return None
        p1, st1, _ = cv2.calcOpticalFlowPyrLK(prev, gray, p0, None, **self.lk)
        if p1 is None:
            return None
        p0b, st2, _ = cv2.calcOpticalFlowPyrLK(gray, prev, p1, None, **self.lk)
        if p0b is None:
            return None
        fb_err = np.linalg.norm(p0 - p0b, axis=2).ravel()
        good = (st1.ravel() == 1) & (st2.ravel() == 1) & (fb_err < 1.0)
        if good.sum() < self.min_inliers:
            return None
        a, b = p0[good].reshape(-1, 2), p1[good].reshape(-1, 2)

        _, inl = cv2.estimateAffinePartial2D(a, b, method=cv2.RANSAC,
                                             ransacReprojThreshold=1.5)
        if inl is None or int(inl.sum()) < self.min_inliers:
            return None
        inl = inl.ravel().astype(bool)
        n = int(inl.sum())
        dt = t - prev_t
        flow = np.median(b[inl] - a[inl], axis=0) / dt

        # потік від обертання дрона (гіроскоп) віднімаємо, лишається потік від руху
        gx, gy = float(gyro_body[0]), float(gyro_body[1])
        flow_rot = self.fx * np.array([gx, gy])

        # відстань до землі ВЗДОВЖ променя камери: при нахилі вона більша за висоту
        cos_tilt = max(float(R_body_to_ned[2, 2]), 0.5)
        depth = height / cos_tilt

        v_cam = -(depth / self.fx) * (flow - flow_rot)
        v_body = np.array([-v_cam[1], v_cam[0], 0.0])
        v_ned = R_body_to_ned @ v_body
        sigma_scale = float(np.clip(40.0 / n, 1.0, 5.0))
        return FlowResult(v_ned[0:2], n, sigma_scale)
