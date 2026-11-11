# Preserved evidence summary — 11 November 2026

**HOLD; execution_allowed=false; forwarding_enabled=false.** This compact summary
ships in Git; the original local captures and reports do not. No new simulation
was run while preparing this handoff. [Linux workflow](../LINUX_GPU_HANDOFF.md).

## Confirmed software rendering

Actual Gazebo render-thread GL query and Ogre log agree:

```
GL_VENDOR=Mesa
GL_RENDERER=llvmpipe (LLVM 20.1.2, 128 bits)
GL_VERSION=4.5 (Core Profile) Mesa 25.2.8-0ubuntu0.24.04.4
EGL device=EGL_MESA_device_software EGL_EXT_device_drm_render_node #0
```

Host Apple Silicon ARM64, Docker Desktop29.8.2 / Linux7.0.14-linuxkit/aarch64.
VM10CPU, 8,319,504,384bytes RAM; physics quota3CPU/100ms, memory2GiB,
effective cpuset0–9. No `/dev/dri` exposed. Gazebo Harmonic8.15.0,
sensors8.2.2, rendering8.2.3, OgreNext2.3.3, GL3Plus/headless EGL,
ROS2 Jazzy. No x86 emulation or hardware GPU acceleration was observed.

## Two bounded camera-only captures

Identical copied SDFs,640×480 RGB, HFOV2.0rad/fx205.4696,10Hz simulation
acquisition, physical model, JSON exchange, scene/lights and visual fixture.
Only the candidate's `LP_NUM_THREADS=2` differs. No VIO/MAVROS/transport;
separate DISARMED FCU supplies the same JSON physics exchange. Full-window
p95 / p99 / max, host milliseconds:

|Metric|Baseline10workers,30.046s|Candidate2workers,30.026s|
|---|---:|---:|
|CameraSensor::Update wall|287.846 /375.848 /490.715|294.282 /406.676 /453.527|
|Ogre2Camera::Render wall|286.148 /373.062 /489.189|290.409 /403.971 /451.416|
|Render calling-thread CPU|146.936 /199.911 /228.970|131.852 /163.928 /257.651|
|Native receiver gaps|297.154 /382.978 /494.111|292.512 /424.209 /455.863|
|Frames /wall Hz|129 /4.293|123 /4.096|
|Observed acquisition Hz|10.000|10.000|
|CFS throttled periods|297/300|111/300|
|Cumulative throttled accounting delta|51.398s|5.079s|
|Render-thread runqueue delta|5.937s|0.844s|

Candidate **FAIL as a latency correction**, not promoted to the main Compose.
Thread limits reduced scheduling pressure but did not improve Render p95/p99
or throughput. Cumulative CFS accounting is not world-pause time. Sequential
samples are not a hard real-time bound or repeated randomized A/B experiment.

`CompositorWorkspace::_update(bool)` spans almost all Render.
`CompositorPassScene::execute(Camera const*)` nested-interval **union** covers
median91.49% of Render wall time; inclusive child spans must not be summed.
Baseline perf cpu-clock:u/99Hz/DWARF:7746samples,0lost; libgallium50.62%,
anonymous executable/JIT-labelled mappings31.54%, llvmpipe workers65.75% of
process CPU samples. Copy/readback max4.085ms; download wait max3.448ms.
Private Mesa kernel names remain unresolved; incomplete ARM64 stacks and absent
JIT symbol map prevent exact shader attribution. Shader compilation was not
established as the dominant cause. GPU synchronization/lock ownership was not
fully resolved. The historical2s outlier was not reproduced here.

Original local evidence, absent after clone:

- `integration/phase1_runs/20261111-renderdevice1/REPORT.md`, `COMMANDS.md`,
  `evidence/METRICS.json`, `SHA256SUMS.json`: methods, aggregates and raw-file hashes.
- `integration/phase1_runs/20261111-renderbase1/evidence/native_render/` and
  `20261111-renderlp2/evidence/native_render/`: perf, stacks, CPU/scheduler samples,
  actual GL identity, Ogre logs and real frames.
- `integration/phase1_runs/20261111-nativeparts1/REPORT.md`: historical full
  capture-only workload, Render p95/p99/max463.593/796.460/1998.367ms;
  Update466.456/799.013/2006.079ms; Odometry gap4538.278ms. Different workload
  and instrumentation: **not** an improvement A/B versus camera-only results.
- `integration/phase1_runs/20261111-camqueue1/REPORT.md`,
  `20261111-sourcecpu1/REPORT.md`: upstream camera/consumer and synchronous
  fault-file I/O investigation. Logs remain locally unchanged.

## Independent transport deadline remains open

Historical nativeparts1 acquisition13.1s arrived at age162sim ms. Its original
permission budget was200−162−18.001=19.999ms; after MAVROS enqueue17.1195ms
remained. Userspace wire receive followed20.792916ms later. Final speed/pose
checks were4.896/5.088ms late and correctly blocked. This includes DDS/MAVROS/
receiver scheduling; no kernel RX timestamp establishes packet arrival time.
16complete valid pairs,32valid/34capture packets; expired packets were rejected.
No vision/control packet was forwarded to FCU. Faster rendering does not by
itself prove this independent final-boundary deadline issue resolved.

Full continuous-normal remains **FAIL**; no60s acceptance or subsequent fault
matrix PASS. Acquisition→FCU uncertainty bound remains NOT ESTABLISHED.
