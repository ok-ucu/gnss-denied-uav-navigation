# Marta SITL demo results

## Provenance and isolation

The original `/home/marta/gnss-sitl-demo.tar.gz` was unavailable on the ZBook. Local homes, mounted-drive directories, and worktrees yielded no copy. The GitHub ref `origin/feat/vio-degraded-imu-odometry` contains Marta's committed demo at `974f80c1a6eb2463fb8eeb48796b5acfff364bbd`. The demo and integration dependencies were exported into an independent directory. There is no checksum for the missing original tarball.

- Dockerfile SHA-256 `98849b9ebda355aaeba8d1e973d4fd6b6fe65319ab9f5f961f91eb4d1b332ad0`.
- Demo Python SHA-256 `1bebcc5a7b09341eaf56580d9963c07049410416734cf7c15847aa95fcadfa6c`.
- The Dockerfile pins ArduCopter `Copter-4.6.3` to `92b0cd788ec29406f26c6f9c31d5ceedbd1cc538` and checks the resolved commit during the build.
- The upstream script's fixed container name and host networking were excluded from live invocation. The local wrapper uses unique owned containers, isolated per-run networking and an ephemeral loopback-only TCP port. The Python scenario code is unchanged. It sends SITL setup and vision telemetry and inhibits operation on an armed heartbeat. No ARM, TAKEOFF or LAND command was issued.

## Results

Offline B: **PASS**. The two-frame injected outage was bridged. Seven bad frames latched `VIO gate-closed hold exceeded cap` 0.699665 s after the first bad frame. Offline B-nohold: **PASS**, with the two-frame outage tripping `VIO availability watchdog`. These checks use a dry link with no EKF3.

Docker build: **PASS**, image `sha256:8adeff15e766a506ac9a3074ef23a7c23540f163673bb8d177a848b3f79846b3`. Both live containers reported the pinned firmware commit.

| Live test | Result | Observed outcome |
|---|---|---|
| A | **PASS** | EKF3 reached absolute horizontal position while disarmed. After vision stopped, the flag cleared in **7.060229 s**. |
| B | **PASS** | Two-frame outage bridged. Seven-frame outage latched `VIO gate-closed hold exceeded cap` after **0.699568 s**. EKF3 lost absolute horizontal position **7.106444 s** after latch. |

Live A evidence: `live-A.7dmmxf/`. Live B evidence: `live-B.qrFZhP/`. Both contain `summary.json`, `events.jsonl`, the SITL console, DataFlash BIN, firmware/image identity and `evidence.sha256`. Each owned container was confirmed stopped and removed and its owned network was removed. Running container inventories matched before and after. B-nohold was exercised in dry-run only.

Evidence directory: `../marta-974f80c/` relative to the checkout root, including `demo-files.sha256`, `docker-build.log`, `dry-run.log` and the owned-run artifacts. This stand exercises labelled injections and looped-back vision on ArduCopter 4.6.3. The Phase 1 camera/VIO, MAVROS and Gazebo stack is a separate validation.
