"""Attitude from raw IMU for recordings that have no autopilot ATTITUDE log.

The team's VIO does not estimate attitude: live, it takes roll/pitch/yaw from
the autopilot. A recording made by dataset/recorder.py has attitude.csv, so
this file is only needed for third-party datasets (and as a cross-check of the
autopilot attitude).

Mahony complementary filter: gyro integration, roll/pitch pulled towards the
accelerometer's gravity direction when the specific force is close to 1 g.
Yaw is NOT corrected (no magnetometer): it starts at `yaw0` and drifts with
the gyro bias.
"""
import numpy as np

from rotations import R_to_euler, euler_to_R

G = 9.80665


def _exp_so3(w):
    angle = np.linalg.norm(w)
    if angle < 1e-12:
        return np.eye(3)
    k = w / angle
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def initial_tilt(acc):
    """Roll and pitch from the mean specific force of a (roughly) static window, body FRD."""
    f = np.mean(np.atleast_2d(acc), axis=0)
    roll = np.arctan2(-f[1], -f[2])
    pitch = np.arctan2(f[0], np.hypot(f[1], f[2]))
    return roll, pitch


def mahony(t, gyro, acc, yaw0=0.0, kp=0.6, ki=0.02, acc_gate=0.15, init_window_s=0.5):
    """Return (N, 3) roll/pitch/yaw [rad], body FRD -> NED, for every IMU sample.

    kp, ki: proportional/integral gain of the accelerometer correction.
    acc_gate: the correction is applied only while | |f| - g | < acc_gate * g."""
    t, gyro, acc = np.asarray(t, float), np.asarray(gyro, float), np.asarray(acc, float)
    n = len(t)
    out = np.zeros((n, 3))
    if n == 0:
        return out
    n_init = max(1, int(np.searchsorted(t, t[0] + init_window_s)))
    roll, pitch = initial_tilt(acc[:n_init])
    R = euler_to_R(roll, pitch, yaw0)
    bias = np.zeros(3)
    out[0] = roll, pitch, yaw0
    for i in range(1, n):
        dt = t[i] - t[i - 1]
        if not 0.0 < dt < 0.5:           # gap or bad stamp: hold attitude
            out[i] = out[i - 1]
            continue
        f = acc[i]
        norm = np.linalg.norm(f)
        w = gyro[i] - bias
        if abs(norm - G) < acc_gate * G:
            predicted = R.T @ np.array([0.0, 0.0, -1.0])      # specific force direction at rest
            err = np.cross(f / norm, predicted)
            bias -= ki * err * dt
            w = w + kp * err
        R = R @ _exp_so3(w * dt)
        out[i] = R_to_euler(R)
    return out
