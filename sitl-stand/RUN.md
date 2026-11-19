# Loss-of-VIO demo on ArduCopter 4.6.3 SITL (DISARMED)

Two runs on a Linux machine with Docker. No Gazebo and no flight: the vehicle is never armed. The
script exits if it ever sees ARMED.

- **A, baseline (hold OFF).** Vision is looped back from SITL, then stopped for good. Shows how long
  EKF3 coasts on its own IMU before it drops absolute horizontal position.
- **B, hold ON (option D).** 10 Hz frames go through the real `QualityGate` and
  `integration/vio_gate_hold.py`, with the simulation-only budget (0.5 s sim / 1.2 s host).
  Labelled injections: first 2 bad frames, then 7 bad frames. A 500 ms watchdog, the same rule as
  the transport, decides.

## 0. Setup (once, about 2 min)

```bash
tar xzf gnss-sitl-demo.tar.gz && cd gnss-sitl-demo/sitl-stand   # or the repo worktree's sitl-stand/
python3 -m venv ~/.venv-sitl && ~/.venv-sitl/bin/pip install pymavlink numpy
export PYTHON=~/.venv-sitl/bin/python
```

## 1. Commands, in order

| step | command | time |
|---|---|---|
| build image (4.6.3 pinned) | `./run_demo.sh build` | **first build 30–60 min** (clone, submodules, apt prerequisites, `waf copter`); cached later |
| optional offline check, no Docker | `./run_demo.sh dry-run` | about 30 s |
| A | `./run_demo.sh A` | about 1–2 min (EKF warm-up 10–60 s, then 25 s observe) |
| B | `./run_demo.sh B` | about 1–2 min |
| B control (hold off) | `./run_demo.sh B-nohold` | about 1–2 min |

Start the build first; it is the long step. If port 5760 is busy, stop any other SITL.

## 2. Where the results land

`~/sitl-demo/<date>-<run>/`:
- `summary.json`: the verdict;
- `events.jsonl`: the timeline;
- `logs/*.BIN`: DataFlash, with `LOG_DISARMED=1`;
- `sitl_console.txt`.

The last line printed is `SUMMARY {...}`. The exit code is 0 for PASS.

## 3. What a pass looks like

- **A:**
  - `"result": "PASS"` with `"ekf_coast_s"` between 6 and 11 s. About 7 s is expected: position and
    velocity both stopped, so `posRetryTimeNoVel_ms = 7000`.
  - `events.jsonl` shows `ekf_pos_horiz_abs` going false that long after `vision_stopped_for_good`.
- **B:**
  - `outage_2_frames` has `"bridged": true`.
  - `outage_7_frames` has `"latch_reason": "VIO gate-closed hold exceeded cap"` at
    `"latch_after_first_bad_s"` ≈ 0.70.
  - `"ekf_coast_after_latch_s"` ≈ 7.
  - `ekf_pos_horiz_abs` must **not** go false during the bridged 2-frame outage.
- **B-nohold:** `outage_2_frames` latches with `"VIO availability watchdog"`, which is today's
  behaviour.

## 4. What B is and is not

B runs the **real** gate and hold state machines, with the three hold settings loaded through
`load_budget(…, simulation=True)` (a hardware-scope budget is refused). Their output goes to the
**real** ArduCopter 4.6.3 SITL, so EKF3 sees exactly the gaps the hold allows.

B does **not** run `phase1_transport.py`, MAVROS or the camera VIO (`run_vio_stream.py`). Those
only exist in the Phase 1 Gazebo stack, which is pinned to patched 4.4.4. Bad frames are labelled
injections (inliers = 0), not camera images. In this stand, simulated time is host time (speedup 1).

No real-flight path is touched: `launch_phase1.py`, `run_phase1_validation.py`, the permits and the
hardware scope are all unchanged.
