# vio_offline — offline replay, baseline and evaluation (Person 3, Phase 2)

Run the team's VIO and an open baseline (OpenVINS) **from files**, on the same
recording, and compare both with ground truth. No MAVLink, no ROS, no simulator.

Results of the first runs (open real dataset, 16 Nov): [`results/20261116/REPORT.md`](results/20261116/REPORT.md).
What the VIO and the baseline need from calibration: [`CALIBRATION_REQUIREMENTS.md`](CALIBRATION_REQUIREMENTS.md).

## One command for a new recording

```bash
python3 vio_offline/pipeline.py recordings/run_XXXX \
    --camera-params calibration/camera_params.json --out results/run_XXXX
```

It runs the five steps below and writes `results/run_XXXX/SUMMARY.md`
(verdicts, accuracy table, plots). A recording is a folder in the format of
`dataset/recorder.py` (see `dataset/README.md`); plain EuRoC folders work too.

| Step | Script | What it answers |
|---|---|---|
| 1 | `calib_check.py` | Is the camera–IMU time offset, axis mapping and focal length consistent with `camera_params.json`? Uses the flight itself, no target. |
| 2 | `replay_vio.py` | What does our VIO output on this recording? Deterministic, same logic as `vio/vio_ekf.py`. |
| 3 | `baseline/run_baseline.py` | What does OpenVINS (monocular + IMU) output on the same files? |
| 4 | `evaluate.py` | ATE, drift, RPE, scale of both against ground truth, on the same time window, with plots. |
| 5 | `quality_checks.py` | OK/WARN/FAIL: IMU gaps, dropped frames, tracking loss, covariance realism, drift, divergence. Does the flight need to be re-recorded? |

Other tools: `trajectory.py` (convert CSV / TUM / EuRoC), `allan_variance.py`
(IMU noise numbers from a static recording, for Kalibr and the baseline),
`convert_insane.py` (open INSANE dataset → our recording format).

## Setup

```bash
pip install numpy opencv-python-headless matplotlib          # Python >= 3.10
docker build -t gnss-vio-baseline:openvins-2.7 --build-arg BUILD_JOBS=3 vio_offline/baseline   # baseline only
cd vio_offline && python3 -m unittest discover -s tests      # 22 tests, ~3 s, no data needed
```

The baseline image is 4.2 GB and compiling needs about 2 GB of RAM per job:
keep `BUILD_JOBS` at 3 or below on a machine with 8–10 GB (WSL crashed here during a build with 8 jobs; 3 jobs built in about 2.5 minutes).

The estimator code is **not copied** into this folder. `vio_source.py` imports
`vio_core.py` and `vio_frontend.py` unmodified from `vio/` if the checkout has
it, otherwise from the pinned `vio-setup` commit `d33f660` via `git show`
(cached in `_vio_src/`, git-ignored). Every run records the file hashes in `run.json`.

## Output files and conventions

Every trajectory (ours, baseline, ground truth) is written in the same three formats:

| File | Content |
|---|---|
| `*.tum` | `t x y z qx qy qz qw` — for `evo` and `rpg_trajectory_evaluation` |
| `*_euroc.csv` | `t_ns, p_x, p_y, p_z, q_w, q_x, q_y, q_z, v_x, v_y, v_z` — EuRoC ground-truth layout |
| `*.csv` | the above plus NED columns, position covariance, quality, baseline feature counts |

- **World: ENU** (x east, y north, z up). **Body: FLU.** Quaternion: Hamilton, body → world.
  This is the same frame as Person 1's `gps_enu.tum`.
- **Time: seconds from the first camera frame** of the recording; `t0_ns` (the absolute
  stamp of that frame) is in the `*.json` next to each trajectory and in the `t_ns` column.
- Our VIO works in NED internally; `states.csv` keeps the raw NED state, covariance and flags.
- OpenVINS has an arbitrary heading, and our VIO has one only when the autopilot's attitude
  is recorded. `evaluate.py` therefore aligns rotation about the vertical + translation by default
  (`--align yaw`); use `--align origin` to test absolute heading, `--align-seconds N` to fit only the start.

`evaluate.py --ref` takes a recording folder (its `groundtruth.csv`, EuRoC ground truth or
recorder GPS) or a trajectory file such as Person 1's `gps_enu.tum`
(`--ref-time-offset` shifts a reference that is on another time base).
`metrics.json` holds every number per estimate; `evo_ape tum ref.tum est.tum -a` gives the same ATE
(checked on EuRoC: 0.0701 m from both).

## Running the steps by hand

```bash
cd vio_offline
REC=../recordings/run_XXXX ; CAL=../calibration/camera_params.json

python3 calib_check.py   $REC --camera-params $CAL --out out/calib_check
python3 replay_vio.py    $REC --camera-params $CAL --out out/vio
python3 baseline/run_baseline.py $REC --camera-params $CAL --out out/baseline
python3 evaluate.py --ref $REC --est vio=out/vio/vio.csv --est baseline=out/baseline/baseline.csv \
                    --out out/eval --common-window
python3 quality_checks.py $REC --camera-params $CAL --vio out/vio --baseline out/baseline --out out/quality
```

`replay_vio.py` options worth knowing:

| Option | Meaning |
|---|---|
| `--attitude recorded\|ahrs\|gt` | `recorded` = `attitude.csv` from the autopilot (default when present). `ahrs` = own filter on the IMU, heading from gyro only (default when there is no `attitude.csv`). `gt` = ground truth, for diagnosis only. |
| `--vertical auto\|range\|ap\|gt` | Height source, same meaning as in `vio_ekf.py`; `gt` uses ground-truth height in place of the autopilot's. |
| `--timeshift S` | Override `timeshift_cam_imu_s` (use the value `calib_check.py` reports). |
| `--gyro sample\|mean` | Rate used to remove rotation from the optical flow. `sample` is what `vio_ekf.py` does; `mean` averages the gyro between the two frames. |
| `--ekf K=V`, `--frontend K=V` | What-if runs with other constructor arguments, e.g. `--ekf sigma_vis=0.6 --frontend sigma_ref=0.6`. Logged as non-default. |
| `--scale F`, `--start S`, `--end S` | Downscale frames; process part of the recording. |

`run_baseline.py`: `--imu-noise euroc|autopilot|autopilot_vibration|file.json`, `--init static|dynamic`,
`--extra-timeshift S`, `--no-calib-online`, `--set key=value` for any key of the OpenVINS config
(`baseline/config_template/` is the stock OpenVINS EuRoC config the settings are applied to).
Static initialisation needs about 2 s of standstill followed by the take-off jerk inside the recording.

## How the replay relates to the live VIO

- Same classes, same thresholds, same order of operations as `vio/vio_ekf.py`; all times come
  from the recording instead of the wall clock.
- `vio_frontend.py` assumes one camera mounting (image x = right, image y = back). The replay
  reads `T_cam_imu` and rotates IMU and attitude into a camera-aligned body frame first, so any
  calibrated mounting works offline. The **live** `vio_ekf.py` does not do this, does not undistort
  frames and uses only `--fx`.
- Per-frame processing time is measured, but on this computer, not on the on-board one.

## Known limits

- The baseline is monocular + IMU: it gets no range finder, so its scale depends on accelerometer
  excitation. Our VIO gets height from the range finder or the autopilot. The two do not use the same information.
- `--attitude ahrs` has no absolute heading and its roll/pitch degrade during long accelerations.
- Lever arms (camera–IMU, GNSS antenna–IMU) are ignored in our VIO and in the evaluation.
- OpenVINS is sensitive on vibrating airframes: see the report for the settings that worked.
