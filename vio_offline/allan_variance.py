#!/usr/bin/env python3
"""IMU noise parameters from a STATIC recording (Allan deviation).

    python3 allan_variance.py RECORDING --out OUT          # drone on the table, motors off, >= 20 min (2-4 h is better)

Kalibr (calibration/imu.yaml) and the OpenVINS baseline both need four numbers
per IMU. This script reads them off the Allan deviation curve:

  noise density   white noise:      value of the -1/2 slope at tau = 1 s
  random walk     bias diffusion:   value of the +1/2 slope at tau = 3 s

  accelerometer_noise_density  [m/s^2/sqrt(Hz)]    gyroscope_noise_density  [rad/s/sqrt(Hz)]
  accelerometer_random_walk    [m/s^3/sqrt(Hz)]    gyroscope_random_walk    [rad/s^2/sqrt(Hz)]

The result is for a QUIET sensor. In flight, vibration adds much more noise:
multiply the densities by 5-10 for the estimator (standard Kalibr/OpenVINS advice),
or check quality_checks.py's vibration line on a flight recording.

Output: OUT/imu_noise.json (accepted by baseline/run_baseline.py --imu-noise),
OUT/imu_noise_kalibr.yaml (paste into calibration/imu.yaml), OUT/allan.png.
"""
import argparse
import json
from pathlib import Path

import numpy as np

import recording as rec_io


def allan_deviation(x, rate, taus_per_decade=10):
    """Overlapping Allan deviation of a rate signal x sampled at `rate` Hz. Returns (tau, adev)."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    theta = np.concatenate([[0.0], np.cumsum(x)]) / rate          # integrated signal
    max_m = (n - 1) // 2
    if max_m < 2:
        raise ValueError("recording too short for an Allan deviation")
    m = np.unique(np.round(np.logspace(0, np.log10(max_m), int(taus_per_decade * np.log10(max_m)) + 1)).astype(int))
    tau, adev = [], []
    for k in m:
        d = theta[2 * k:] - 2 * theta[k:-k] + theta[:-2 * k]
        tau.append(k / rate)
        adev.append(np.sqrt(np.mean(d ** 2) / (2.0 * (k / rate) ** 2)))
    return np.array(tau), np.array(adev)


def fit_slope(tau, adev, slope, lo, hi):
    """Least-squares offset of a line with the given log-log slope over tau in [lo, hi]; returns value at tau = 1."""
    m = (tau >= lo) & (tau <= hi)
    if m.sum() < 2:
        return float("nan")
    return float(np.exp(np.mean(np.log(adev[m]) - slope * np.log(tau[m]))))


def noise_parameters(tau, adev):
    """(white-noise density, random-walk density) for one axis."""
    span = tau[-1]
    density = fit_slope(tau, adev, -0.5, tau[0] * 2, min(1.0, span / 10))           # N: adev = N / sqrt(tau)
    k_at_1 = fit_slope(tau, adev, +0.5, max(span / 10, 10.0), span / 2)             # K: adev = K * sqrt(tau / 3)
    walk = k_at_1 * np.sqrt(3.0) if np.isfinite(k_at_1) else float("nan")
    # A +1/2 slope region may simply not exist in a short recording: only trust it when the curve rises there.
    rising = adev[-1] > 1.3 * adev.min()
    return density, (walk if rising else float("nan"))


def analyse(t, gyro, acc):
    rate = 1.0 / np.median(np.diff(t))
    result = dict(rate_hz=round(float(rate), 2), duration_s=round(float(t[-1] - t[0]), 1), axes={})
    curves = {}
    for name, data in (("gyro", gyro), ("acc", acc)):
        dens, walks = [], []
        for k, axis in enumerate("xyz"):
            tau, adev = allan_deviation(data[:, k] - data[:, k].mean(), rate)
            d, w = noise_parameters(tau, adev)
            dens.append(d)
            walks.append(w)
            curves[f"{name}_{axis}"] = (tau, adev)
            result["axes"][f"{name}_{axis}"] = dict(noise_density=d, random_walk=w)
        result[name] = float(np.nanmean(dens))
        result[f"{name}_walk"] = float(np.nanmean(walks)) if np.any(np.isfinite(walks)) else None
    return result, curves


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", help="static recording folder (only mav0/imu0/data.csv is read)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--inflate", type=float, default=1.0, help="multiply the densities written to the output files")
    args = ap.parse_args()

    root = Path(args.recording)
    t_ns, imu = rec_io._table(root / "mav0" / "imu0" / "data.csv", 7)
    if len(t_ns) < 1000:
        raise SystemExit("need at least 1000 IMU rows")
    t = (t_ns - t_ns[0]) * 1e-9
    result, curves = analyse(t, imu[:, 0:3], imu[:, 3:6])
    if result["duration_s"] < 1200:
        print(f"WARNING: only {result['duration_s']:.0f} s of data; random walk needs 20 min or more and is unreliable here")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    fallback = dict(acc_walk=3.0e-3, gyro_walk=2.0e-5)        # typical MEMS values when the recording is too short
    noise = dict(acc=result["acc"] * args.inflate, gyro=result["gyro"] * args.inflate,
                 acc_walk=(result["acc_walk"] or fallback["acc_walk"]) * args.inflate,
                 gyro_walk=(result["gyro_walk"] or fallback["gyro_walk"]) * args.inflate)
    (out / "imu_noise.json").write_text(json.dumps(noise, indent=2) + "\n")
    (out / "allan.json").write_text(json.dumps(result, indent=2) + "\n")
    (out / "imu_noise_kalibr.yaml").write_text(
        f"rostopic: /imu0\nupdate_rate: {result['rate_hz']:.1f}\n"
        f"accelerometer_noise_density: {noise['acc']:.6e}\naccelerometer_random_walk: {noise['acc_walk']:.6e}\n"
        f"gyroscope_noise_density: {noise['gyro']:.6e}\ngyroscope_random_walk: {noise['gyro_walk']:.6e}\n")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        for ax, name, unit in ((axes[0], "gyro", "rad/s"), (axes[1], "acc", "m/s²")):
            for axis, color in zip("xyz", ("#2a78d6", "#eb6834", "#1baf7a")):
                tau, adev = curves[f"{name}_{axis}"]
                ax.loglog(tau, adev, color=color, lw=1.6, label=axis)
            ax.set_xlabel("averaging time τ, s")
            ax.set_ylabel(f"Allan deviation, {unit}")
            ax.set_title(name, loc="left")
            ax.grid(alpha=0.25, which="both")
            ax.legend(frameon=False)
        fig.tight_layout()
        fig.savefig(out / "allan.png", dpi=130)
    except ImportError:
        pass
    print(json.dumps(dict(rate_hz=result["rate_hz"], duration_s=result["duration_s"], **noise), indent=2))


if __name__ == "__main__":
    main()
