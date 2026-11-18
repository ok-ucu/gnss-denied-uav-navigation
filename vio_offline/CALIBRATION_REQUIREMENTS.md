# Calibration and recording requirements for VIO and the baseline

What Person 3 needs from Person 2 (camera, IMU, dataset) and Person 1 (GPS ground truth)
before the first real recording on 17 November, and how each item is checked.

## 1. Parameters

All of these go into `calibration/camera_params.json` (written by `calibration/make_report.py`
from Kalibr output). The file format is already right; no change needed.

| Parameter | Field | Used by | Accept when | Checked by |
|---|---|---|---|---|
| Focal lengths, principal point | `fx fy cx cy`, `width height` | replay (undistortion, flow scale), OpenVINS | reprojection error mean < 0.5 px (1 px at most); `fx` within 1 % across 2–3 calibrations | Kalibr report; `calib_check.py` slope 1.0 ± 0.15 |
| Distortion | `distortion_model` (`radtan` or `equidistant`), `dist` | replay, OpenVINS | same report; model must be one of the two (OpenVINS has no `fov` model) | Kalibr report |
| Camera–IMU rotation | `T_cam_imu` (3×3 part) | replay (axis mapping), OpenVINS | within 10° of the mounting `vio_frontend.py` assumes if the **live** VIO is to be used unchanged; any value works offline | `make_report.py` mount deviation; `calib_check.py` axis correlation (matching axes > 0.5, crossed axes near 0) |
| Camera–IMU translation | `T_cam_imu` (last column) | OpenVINS only | ±1–2 cm is enough | Kalibr report |
| Time offset camera → IMU | `timeshift_cam_imu_s` (`t_imu = t_cam + shift`) | replay, OpenVINS | within ±5 ms between recordings; `calib_check.py` remaining offset below 5 ms | `calib_check.py` on every flight |
| IMU noise: accel/gyro noise density, accel/gyro random walk, rate | `calibration/imu.yaml`; `--imu-noise file.json` | Kalibr, OpenVINS | from a static recording of 20 min or more | `allan_variance.py` |

Why the time offset matters: on the open INSANE flight the camera stamps were 37 ms late and the
dataset does not say so. Correcting it halved our VIO's error (15.3 → 7.9 m ATE) and cut
OpenVINS's by two thirds (35.8 → 12.0 m). `calib_check.py` found −36.7 ms from the flight alone;
OpenVINS's own online estimate converged to −37.1 ms.

Conventions that must hold (they are what `dataset/recorder.py` already writes):
IMU in the autopilot body frame FRD, gyro in rad/s, accelerometer as specific force in m/s²
(about −9.8 on z at rest), all stamps in nanoseconds on one clock.

## 2. What every flight recording must contain

| Stream | File | Needed for | Minimum |
|---|---|---|---|
| Frames | `mav0/cam0/` | VIO, baseline | ≥ 10 Hz, global shutter, fixed focus and exposure, no gap above 0.5 s |
| IMU | `mav0/imu0/data.csv` | VIO, baseline | ≥ 100 Hz (200 preferred), no gap of 0.2 s or more; the VIO stops using the accelerometer across such a gap |
| Autopilot attitude | `attitude.csv` | our VIO (it does not estimate attitude) | ≥ 20 Hz |
| Height | `distance.csv` (range finder) or `local_position.csv` | our VIO (flow scale) | ≥ 10 Hz; range finder preferred, it is independent of the autopilot's EKF |
| GPS | `mav0/gps0/data.csv` or Person 1's `gps_enu.tum` | ground truth | fix ≥ 3 for the whole flight, on the recording computer's clock or with a known offset |
| Calibration | `camera_params.json` | everything | from the same camera, focus and mounting as the flight |

For the baseline specifically: start recording **at least 3 s before take-off with the drone
standing still**, and fly with some acceleration (turns, speed changes). A monocular VIO gets its
scale only from accelerometer excitation; straight constant-speed flight leaves it unobservable.

For the IMU noise numbers: one **static recording of 20 min or more** (motors off, same recorder).

## 3. Decision after the first real recording (17 Nov)

Run `python3 vio_offline/pipeline.py RECORDING --camera-params calibration/camera_params.json --out OUT`
and read `OUT/SUMMARY.md`. Ask Person 2 for a new recording if any of these appear:

| Line in the summary | Meaning | Action |
|---|---|---|
| IMU rate FAIL, or IMU gaps FAIL/WARN with gaps of 0.2 s and more | autopilot link too slow or dropping | raise baud / stream rate, re-record |
| camera gaps FAIL | frames dropped (disk or CPU) | faster storage or lower PNG compression, re-record |
| overlap WARN | IMU does not cover the frames | start the streams earlier, re-record |
| attitude WARN (missing) | no `attitude.csv` | re-record; the VIO needs it |
| calib check: time offset above 5 ms | `timeshift_cam_imu_s` is off | no re-record: rerun with `--vio-args "--timeshift X"`, then fix the calibration file |
| calib check: negative or crossed axes | camera mounted differently from `T_cam_imu` | recalibrate; a re-record is not needed if the mounting itself is unchanged |
| heading FAIL (~180° or ~90°) | motion sign or axes wrong between camera and IMU data | check mounting and `T_cam_imu` before flying again |
| vibration WARN/FAIL | airframe shakes the IMU | damp the autopilot mount; use `--imu-noise autopilot_vibration` for the baseline |
| baseline did not initialise | no standstill before take-off | re-record with 3 s standstill, or use `--baseline-args "--init dynamic --start S"` |

## 4. Open points for teammates

- **Person 2:** `dataset/generate_sample_dataset.py` scrolls the ground texture the wrong way for the
  mounting in `camera_params.json`: forward flight should move ground features towards the bottom of
  the image (+y), the generator moves them up. Our VIO therefore reports −2 m/s on `recordings/sample_flight`
  while its GPS says +2 m/s (`quality_checks.py` flags it: heading 180°). The frontend's sign is confirmed on
  real data (INSANE) and on rendered test frames, so the generator is the side to change.
- **Person 2:** `Dataset.export_tum_trajectory` writes `north east up`, which is a left-handed frame; Person 1's
  `gps_enu.tum` is `east north up`. A yaw alignment cannot fix a mirrored track. Suggest switching to ENU.
- **Person 2:** `distance.csv` does not store the range finder's orientation; `vio_ekf.py` ignores readings that
  are not "down" (orientation 25). Offline the range finder is assumed to point down.
- **Person 1:** `gps_enu.tum` uses seconds from the first GPS point on the autopilot's boot clock; recordings use
  the companion computer's clock. `evaluate.py --ref gps_enu.tum --ref-time-offset S` accepts a known offset;
  the offset itself has to come from the same-session clock model.
- **Person 4:** trajectory formats and `metrics.json` are described in `README.md`; tell me if you want other
  columns or another alignment as the default.
