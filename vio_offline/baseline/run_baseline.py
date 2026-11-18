#!/usr/bin/env python3
"""Run the open baseline (OpenVINS 2.7, monocular + IMU) on a recording.

    docker build -t gnss-vio-baseline:openvins-2.7 vio_offline/baseline      # once, ~10 min
    python3 vio_offline/baseline/run_baseline.py RECORDING --out OUT [--camera-params camera_params.json]

Works on the same folders as replay_vio.py: the team's recorder format, a
EuRoC/ASL folder, or convert_insane.py output. It

  1. writes an OpenVINS config into OUT/config from the calibration
     (intrinsics, T_cam_imu, time shift) and an IMU noise preset;
  2. runs the estimator in Docker on RECORDING (mounted read-only);
  3. converts the result to the common trajectory files
     OUT/baseline.csv, baseline.tum, baseline_euroc.csv (see trajectory.py).

OpenVINS's world frame is gravity-aligned (z up) with an ARBITRARY heading, so
compare it with ground truth after a yaw alignment (evaluate.py does that).
"""
import argparse
import csv
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import recording as rec_io                       # noqa: E402
from rotations import FRD_FLU, R_to_quat, quat_to_R   # noqa: E402
from trajectory import Trajectory                # noqa: E402

IMAGE = "gnss-vio-baseline:openvins-2.7"

# Continuous-time noise densities: accel [m/s^2/sqrt(Hz)], accel bias walk [m/s^3/sqrt(Hz)],
# gyro [rad/s/sqrt(Hz)], gyro bias walk [rad/s^2/sqrt(Hz)].
IMU_NOISE = {
    # values shipped with OpenVINS for the EuRoC ADIS16448
    "euroc": dict(acc=2.0e-3, acc_walk=3.0e-3, gyro=1.6968e-4, gyro_walk=1.9393e-5),
    # consumer MEMS IMU of an autopilot on a vibrating airframe: datasheet-level values inflated
    # about 10x, the usual practice until an Allan-variance result is available (allan_variance.py)
    "autopilot": dict(acc=2.0e-2, acc_walk=4.0e-3, gyro=1.0e-3, gyro_walk=4.0e-5),
    # the same on an airframe with strong vibration (quality_checks.py reports > 2 m/s^2 RMS)
    "autopilot_vibration": dict(acc=8.0e-2, acc_walk=4.0e-3, gyro=4.0e-3, gyro_walk=4.0e-5),
}


def set_key(text, key, value):
    """Replace `key: old # comment` in the OpenVINS YAML, keeping the comment."""
    pattern = re.compile(rf"^({re.escape(key)}:\s*)([^#\n]*?)(\s*(#.*)?)$", re.M)
    if not pattern.search(text):
        raise KeyError(key)
    return pattern.sub(lambda m: f"{m.group(1)}{value}{m.group(3)}", text, count=1)


def matrix_yaml(M, indent="    "):
    return "\n".join(indent + "- [" + ", ".join(f"{v:.12g}" for v in row) + "]" for row in M)


def write_config(out, calib, noise, imu_rate, cam_rate, args):
    out.mkdir(parents=True, exist_ok=True)
    text = (HERE / "config_template" / "euroc_estimator_config.yaml").read_text()
    settings = {
        "use_stereo": "false", "max_cameras": "1",
        "calib_cam_extrinsics": str(args.calib_online).lower(),
        "calib_cam_intrinsics": str(args.calib_online).lower(),
        "calib_cam_timeoffset": str(args.calib_timeoffset).lower(),
        "gravity_mag": f"{args.gravity:.5f}",
        "track_frequency": f"{cam_rate + 1.0:.1f}",
        "num_pts": str(args.num_pts),
        "init_dyn_use": "true" if args.init == "dynamic" else "false",
        "init_window_time": f"{args.init_window:.2f}",
        "init_imu_thresh": f"{args.init_imu_thresh:.3f}",
        "up_msckf_sigma_px": f"{args.sigma_px:g}", "up_slam_sigma_px": f"{args.sigma_px:g}",
        "num_opencv_threads": "2",
        "verbosity": f'"{args.verbosity}"',
    }
    for item in args.set:                       # free-form overrides, e.g. --set try_zupt=true
        key, _, value = item.partition("=")
        settings[key.strip()] = value.strip()
    for key, value in settings.items():
        text = set_key(text, key, value)
    (out / "estimator_config.yaml").write_text(text)

    (out / "kalibr_imu_chain.yaml").write_text(f"""%YAML:1.0

imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  accelerometer_noise_density: {noise['acc']:.6e}
  accelerometer_random_walk: {noise['acc_walk']:.6e}
  gyroscope_noise_density: {noise['gyro']:.6e}
  gyroscope_random_walk: {noise['gyro_walk']:.6e}
  rostopic: /imu0
  time_offset: 0.0
  update_rate: {imu_rate:.1f}
  model: "kalibr"
  Tw:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  R_IMUtoGYRO:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  Ta:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  R_IMUtoACC:
    - [ 1.0, 0.0, 0.0 ]
    - [ 0.0, 1.0, 0.0 ]
    - [ 0.0, 0.0, 1.0 ]
  Tg:
    - [ 0.0, 0.0, 0.0 ]
    - [ 0.0, 0.0, 0.0 ]
    - [ 0.0, 0.0, 0.0 ]
""")

    model = {"radial-tangential": "radtan", "plumb_bob": "radtan"}.get(calib.distortion_model, calib.distortion_model)
    if model not in ("radtan", "equidistant"):
        raise SystemExit(f"OpenVINS supports radtan and equidistant distortion, not {calib.distortion_model}")
    dist = (list(calib.dist) + [0.0] * 4)[:4]
    (out / "kalibr_imucam_chain.yaml").write_text(f"""%YAML:1.0

cam0:
  T_imu_cam: # camera -> IMU (inverse of Kalibr's T_cam_imu)
{matrix_yaml(np.linalg.inv(calib.T_cam_imu))}
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [{', '.join(f'{d:.10g}' for d in dist)}]
  distortion_model: {model}
  intrinsics: [{calib.fx:.6f}, {calib.fy:.6f}, {calib.cx:.6f}, {calib.cy:.6f}]
  resolution: [{calib.width}, {calib.height}]
  rostopic: /cam0/image_raw
  timeshift_cam_imu: {calib.timeshift_cam_imu_s + args.extra_timeshift:.6f}
""")
    return settings


def to_trajectory(states_csv, t0_ns, imu_frame):
    with open(states_csv, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    col = lambda k: np.array([float(r[k]) for r in rows])
    t = col("t_s")
    p = np.column_stack([col("px"), col("py"), col("pz")])
    q = np.column_stack([col("qx"), col("qy"), col("qz"), col("qw")])
    if imu_frame == "frd":      # export the body as FLU, like our VIO and the ground truth
        q = np.array([R_to_quat(quat_to_R(x) @ FRD_FLU) for x in q])
    v = np.column_stack([col("vx"), col("vy"), col("vz")])
    xx, xy, xz, yy, yz, zz = (col(k) for k in ("cov_pxx", "cov_pxy", "cov_pxz", "cov_pyy", "cov_pyz", "cov_pzz"))
    cov = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]).transpose(2, 0, 1)
    extra = {k: col(k) for k in ("dt_cam_imu", "n_slam", "n_msckf", "proc_ms")}
    return Trajectory(t, p, q, v, cov, t0_ns=t0_ns, name="baseline", extra=extra)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--out", required=True)
    ap.add_argument("--camera-params", default=None)
    ap.add_argument("--imu-noise", default="autopilot",
                    help="preset (%s) or a JSON file with acc, acc_walk, gyro, gyro_walk" % ", ".join(IMU_NOISE))
    ap.add_argument("--imu-frame", choices=["frd", "native"], default="frd",
                    help="frd: recording IMU is FRD (team format); native: keep the dataset's body axes (EuRoC)")
    ap.add_argument("--init", choices=["static", "dynamic"], default="static",
                    help="static: needs a standstill followed by a jerk (take-off); dynamic: starts in motion")
    ap.add_argument("--init-window", type=float, default=2.0)
    ap.add_argument("--init-imu-thresh", type=float, default=1.5)
    ap.add_argument("--start", type=float, default=0.0, help="seconds from the first frame")
    ap.add_argument("--end", type=float, default=None)
    ap.add_argument("--num-pts", type=int, default=200)
    ap.add_argument("--sigma-px", type=float, default=1.0)
    ap.add_argument("--gravity", type=float, default=9.81)
    ap.add_argument("--extra-timeshift", type=float, default=0.0,
                    help="seconds added to timeshift_cam_imu_s from the calibration file")
    ap.add_argument("--no-calib-online", dest="calib_online", action="store_false",
                    help="keep intrinsics/extrinsics fixed (default: OpenVINS refines them online)")
    ap.add_argument("--no-calib-timeoffset", dest="calib_timeoffset", action="store_false")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="override any key of estimator_config.yaml, e.g. --set try_zupt=true")
    ap.add_argument("--verbosity", default="INFO")
    ap.add_argument("--image", default=IMAGE)
    args = ap.parse_args()

    rec = rec_io.load(args.recording, args.camera_params, apply_timeshift=False)
    out = Path(args.out).resolve()
    noise = IMU_NOISE[args.imu_noise] if args.imu_noise in IMU_NOISE else json.loads(Path(args.imu_noise).read_text())
    imu_rate = 1.0 / np.median(np.diff(rec.imu_t))
    cam_rate = 1.0 / np.median(np.diff(rec.frame_t))
    settings = write_config(out / "config", rec.calib, noise, imu_rate, cam_rate, args)

    cmd = ["docker", "run", "--rm", "--user", f"{os.getuid()}:{os.getgid()}",
           "-v", f"{rec.root.resolve()}:/data:ro", "-v", f"{out}:/out", args.image,
           "/out/config/estimator_config.yaml", "/data", "/out", "--start", str(args.start)]
    if args.end is not None:
        cmd += ["--end", str(args.end)]
    print(" ".join(cmd))
    wall0 = time.time()
    with open(out / "openvins.log", "w") as logf:
        rc = subprocess.run(cmd, stdout=logf, stderr=subprocess.STDOUT).returncode
    wall = time.time() - wall0

    commit = subprocess.run(["docker", "run", "--rm", "--entrypoint", "cat", args.image, "/opt/baseline/OPENVINS_COMMIT"],
                            capture_output=True, text=True).stdout.strip()
    image_id = subprocess.run(["docker", "image", "inspect", "-f", "{{.Id}}", args.image],
                              capture_output=True, text=True).stdout.strip()
    summary = dict(baseline="OpenVINS (ov_msckf), monocular + IMU", openvins_commit=commit, docker_image=args.image,
                   docker_image_id=image_id, recording=str(rec.root), exit_code=rc, wall_s=round(wall, 1),
                   imu_noise=noise, imu_noise_preset=args.imu_noise, imu_rate_hz=round(float(imu_rate), 1),
                   camera_rate_hz=round(float(cam_rate), 2), calibration=rec.calib.source,
                   timeshift_cam_imu_s=rec.calib.timeshift_cam_imu_s + args.extra_timeshift,
                   init=args.init, start=args.start, end=args.end, config_overrides=settings)
    if (out / "openvins_run.json").exists():
        summary["runner"] = json.loads((out / "openvins_run.json").read_text())
    traj = to_trajectory(out / "openvins_states.csv", rec.t0_ns, args.imu_frame) if (out / "openvins_states.csv").exists() else None
    if traj is not None and len(traj):
        summary["trajectory"] = traj.write_all(out, "baseline")
        summary["first_state_s"], summary["last_state_s"] = float(traj.t[0]), float(traj.t[-1])
    (out / "run.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({k: summary[k] for k in summary if k not in ("config_overrides",)}, indent=2))
    if traj is None or not len(traj):
        raise SystemExit(f"OpenVINS produced no states (exit {rc}); see {out}/openvins.log")


if __name__ == "__main__":
    main()
