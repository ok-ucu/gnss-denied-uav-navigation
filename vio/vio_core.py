from collections import deque
import numpy as np

G = 9.80665

def rot_body_to_ned(roll, pitch, yaw):
    """Матриця повороту з осей дрона (FRD) у світові осі NED."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp,     sr * cp,                cr * cp],
    ])

class VioEKF:
    def __init__(self, sigma_acc=0.3, sigma_bias=0.01, sigma_vis=0.08,
                 gate_nis=16.27):
        self.x = np.zeros(9)
        self.P = np.diag([0.01] * 3 + [0.01] * 3 + [0.04] * 3)
        self.sigma_acc = sigma_acc
        self.sigma_bias = sigma_bias
        self.sigma_vis = sigma_vis
        self.gate_nis = gate_nis

    def predict(self, f_body, R, dt):
        """f_body - вимір акселерометра (м/с^2, осі дрона), R - body->NED."""
        p, v, ba = self.x[0:3], self.x[3:6], self.x[6:9]
        a = R @ (f_body - ba) + np.array([0.0, 0.0, G])
        self.x[0:3] = p + v * dt + 0.5 * a * dt * dt
        self.x[3:6] = v + a * dt

        F = np.eye(9)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -R * dt
        Q = np.zeros((9, 9))
        Q[0:3, 0:3] = np.eye(3) * 1e-6 * dt
        Q[3:6, 3:6] = np.eye(3) * self.sigma_acc ** 2 * dt
        Q[6:9, 6:9] = np.eye(3) * self.sigma_bias ** 2 * dt
        self.P = F @ self.P @ F.T + Q

    def update_velocity(self, v_meas, sigma_scale=1.0):
        """Корекція за виміряною швидкістю (NED). Повертає (прийнято?, NIS)."""
        H = np.zeros((3, 9))
        H[:, 3:6] = np.eye(3)
        Rm = np.eye(3) * (self.sigma_vis * sigma_scale) ** 2
        y = v_meas - self.x[3:6]
        S = H @ self.P @ H.T + Rm
        nis = float(y @ np.linalg.solve(S, y))
        if nis > self.gate_nis:
            return False, nis
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I_KH = np.eye(9) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return True, nis

    def update_velocity_xy(self, v_xy, sigma_scale=1.0):
        """Те саме, але тільки горизонталь (камера вниз не міряє вертикаль)."""
        H = np.zeros((2, 9))
        H[:, 3:5] = np.eye(2)
        Rm = np.eye(2) * (self.sigma_vis * sigma_scale) ** 2
        y = np.asarray(v_xy) - self.x[3:5]
        S = H @ self.P @ H.T + Rm
        nis = float(y @ np.linalg.solve(S, y))
        if nis > 13.82:
            return False, nis
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I_KH = np.eye(9) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return True, nis

    def pose_cov(self, inflate=1.0, att_var=3e-4):
        """21 число: верхній трикутник 6x6 (x,y,z,roll,pitch,yaw)."""
        C = np.zeros((6, 6))
        C[0:3, 0:3] = self.P[0:3, 0:3] * inflate
        C[3:6, 3:6] = np.eye(3) * att_var * inflate
        return [float(c) for c in C[np.triu_indices(6)]]

    def speed_cov(self, inflate=1.0):
        """9 чисел: матриця 3x3 по рядках."""
        return [float(c) for c in (self.P[3:6, 3:6] * inflate).flatten()]


class QualityMonitor:
    """Самооцінка якості: частка прийнятих вимірювань + свіжість даних."""

    def __init__(self, n=20, grace_s=0.2, stale_s=1.0):
        self.hist = deque(maxlen=n)
        self.last_ok = None
        self.grace_s = grace_s
        self.stale_s = stale_s

    def push(self, accepted, t):
        self.hist.append(1.0 if accepted else 0.0)
        if accepted:
            self.last_ok = t

    def quality(self, t):
        """0.0 (дані непридатні) ... 1.0 (все чудово)."""
        if not self.hist or self.last_ok is None:
            return 0.0
        rate = sum(self.hist) / len(self.hist)
        age = t - self.last_ok
        fresh = float(np.clip(1.0 - (age - self.grace_s) / self.stale_s, 0.0, 1.0))
        return rate * fresh

    @staticmethod
    def inflate(q):
        """Чим гірша якість - тим більша заявлена коваріація (1x ... 10x)."""
        return 1.0 + 9.0 * (1.0 - q)
