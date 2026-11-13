# External inputs and source-only reproduction boundary

**HOLD. No forwarding, fusion or flight approval.** The allowlist contains no
full upstream repositories, compiler output, shared libraries, firmware binaries
or raw recordings. The source can be inspected and its pure tests reproduced;
full runtime provisioning is still an explicit external prerequisite.
The [Linux GPU handoff](LINUX_GPU_HANDOFF.md) adds a clean-checkout camera-only
source build path; it does not admit a new binary into the historical guards.
The current [amd64 preparation](AMD64_TRANSPORT_READINESS.md) adds a public-base
MAVROS2.16.0 recipe, explicit artifact exports and PREPARE-ONLY candidates.
[VIO continuity](VIO_SOURCE_PROBE.md) uses an independent subscriber; neither
procedure grants FCU forwarding or a native firmware admission.

| Dependency | Recorded version/pin | Expected provision |
|---|---|---|
| ArduPilot Copter | [9279df6191a772c098b2e4797c95ac954f258d88](https://github.com/ArduPilot/ardupilot/tree/9279df6191a772c098b2e4797c95ac954f258d88), 4.4.4 | Fetch upstream recursively outside the commit; build only a new copy with the included boolean/instrumentation patches and actual C++ tests |
| ArduPilot Gazebo plugin | [f116b03552d05c8d0b0bed5ed67033a1183121d1](https://github.com/ArduPilot/ardupilot_gazebo/tree/f116b03552d05c8d0b0bed5ed67033a1183121d1) | Ignored `ardupilot_gazebo/source/` or an external build context; do not vendor checkout/meshes/build tree |
| Gazebo / ROS | Harmonic / ROS 2 Jazzy, Linux native arm64/amd64 | `gz.transport13`, `gz.msgs10`, `ros_gz_bridge`, rclpy, rosidl_runtime_py and ROS message packages; native amd64 runtime remains unverified here |
| MAVROS / libmavconn | 2.16.0 | Packaged vision plugins plus separate included `mavros_time_patch/`; original sys_time excluded |
| Python | Python 3.10+; NumPy, OpenCV; Matplotlib for figures | Existing runtime packages; no install or version upgrade during this audit |
| Teammate VIO | `vio_core.py`, `vio_frontend.py`, `vio_ros_source.py`, `vio_ros_out.py` | Pinned at `d33f66008fd7b6de332c96dd324b8ddbd320c194`; the verified `linux_gpu/prepare_vio.py` exports four files to a new ignored dependency directory, separately from teammates' files. `vio/` is untracked here and **not** included or modified by this allowlist |
| MAVLink codec | v2 ardupilotmega, generated file SHA-256 `011577d7432f6e5e2cb8fe6c15ab4637175ac09ad663bb1e951a09c5d9a872ff` | Explicit single-file protocol-source exception at its current path; not a pymavlink repository or captured data |

The codec is 2,263,701 bytes and is the largest proposed file. Keeping these exact
generated **source** bytes preserves the current importlib-based runtime without
silently replacing the protocol implementation. Its generator version is not
recorded; a future change to installed pymavlink/code generation needs a separate
equivalence/provenance review. The upstream pymavlink generator is an external
dependency; its copied repository is excluded. No claim of pinned code-generation
reproducibility is made from the file hash alone.
`fastcrc` is optional in this generated codec; its actual pure-Python CRC
implementation is retained when that acceleration package is absent.

## Existing local image artifacts, not portable registry references

The current Compose file uses exact local ARM64 image IDs:

| Role | Local image ID |
|---|---|
| Gazebo/VIO | `sha256:6a5215efd7798818523c309eb220753f5755115ed493cd70ada8ae4ed0b61e6b` |
| FCU base | `sha256:f5dffdb9f79ed8a7ff58aa7e6f6dfcbe2ebd2cead9fe7b20daea8d953b491540` |
| MAVROS | `sha256:acd5f652ca1e72744c6427ed63d7d0805412ead3b1b1da95f2a2f08dcbfa7698` |

These are **not registry repo digests that a teammate can simply pull**.
[MAVROS.isolated.Dockerfile](MAVROS.isolated.Dockerfile) and
[plugin Dockerfile](ardupilot_gazebo/Dockerfile) retain the available build
definitions, but depend on local base tags `gnss-gazebo-ardupilot:dev` and `gz-sim`.
A published/pinned base-image build chain or controlled transfer is still needed
for the legacy defaults. The new `linux_gpu/MAVROS.Dockerfile` accepts a public
ROS image digest and exact2.16.0 package versions, rebuilding the existing
overlay at its original prefix. Repository/package availability must be checked
on Linux. These version-constrained records are not a bit-identical apt snapshot.
No build/pull is performed by the offline validation.

## Source versus required generated runtime paths

The current launcher expects these **ignored generated** paths:

- `isolated_blockers/20261109_a/evidence/arducopter_patched`, tested SHA-256
  `047e4fbe21de2f88eba6c453e8aa3091ffd4b89b2b5568544f16cbf4c38d5970`.
- `timesync_patch_runs/20261108/patch_install/setup.bash`, plus its generated
  colcon install tree. At runtime it must be mounted at `/tmp/patch_install`,
  the prefix used when built.

The tested binary hash is a provenance pin, **not** a promise that another
compiler/toolchain produces identical bytes. A newly built firmware must undergo
the parser/clock tests and explicit pin review; do not weaken the check to make a
fresh build appear prevalidated. The included old SHA manifest is required by
`launch_phase1.verified_file()` for selected source/config and external artifacts;
its other historical recordings are deliberately not shipped. Do not run a
whole-history manifest verifier and interpret omitted recordings as source loss.

The unique parser/build helpers remain at their existing paths under
`isolated_blockers/20261109_a/code/`. Their historical build scripts hardcode
that experiment root and write outputs: **do not rerun them over existing
evidence**. For a future rebuild, use a new external workspace/output, the pinned
upstream checkout, the included instrumentation and exact boolean patch, and
`test_sim_json.cpp`. The new `linux_gpu/SITL.Dockerfile` applies the selected patches in a fresh
pinned upstream copy and runs actual parser C++ tests; it does not rerun these
historical scripts. Bootstrap for the guarded full pipeline still needs reviewed
Linux artifact admission. Do not call that full pipeline turnkey reproducible.

[build_mavros_time_patch.sh](build_mavros_time_patch.sh) can build the included
overlay only into a **new** caller-selected output, using a new disposable
builder and the existing base image. It does not patch the original install.
No build output belongs in the source allowlist.

The new replay uses only the reviewed56KB
`tests/fixtures/mavros_timesync_replay.jsonl` and its checksum/selection metadata.
It retains20historical ROS pairs and associated clock labels, not a live VIO or
FCU test. The original ignored ingress recording is informational provenance;
the replay and unit tests do not require it after cloning.
`probe_patched_timesync.py` is required by that replay and is explicitly included.
If the VIO pin is absent from a fresh clone, obtain its Git object explicitly
before `prepare_vio.py`; do not silently use the untracked local `vio/` files.

```bash
git fetch origin d33f66008fd7b6de332c96dd324b8ddbd320c194
python3 -B integration/linux_gpu/prepare_vio.py --output integration/linux_gpu_runs/vio-NEW
```

This fetch obtains the dependency object without switching or merging branches;
the helper verifies all four source hashes before export.

## Small immutable test/config inputs deliberately retained

- `matched_physics/results/SIM_JSON_review_20261109.cpp` and `.h`: small pinned
  upstream source fixtures for instrumentation regression, not a full repository.
- `matched_physics/results/camera_mount_review_20261109.json`: recorded bad-mount
  regression input; changing it to pass is prohibited.
- `isolated_blockers/20261109_a/models/patched_c*` and original sensor SDF:
  integration-only model/world templates with upstream resource references.
- `AP444_BOOLEAN_PATCH_PROPOSAL.patch`: historical **superseded proposal**, not
  the runtime fix. `code/AP444_BOOLEAN_EXACT.patch` is the tested parser change.
- `evidence/instrumentation.patch`: actual trace/test-access source changes.
- `mavros_216_timesync_signed.patch` and `mavros_time_patch/`: proposed diff plus
  actual overlay source. Upstream sources retain their notices/licenses; do not
  represent these files as an original replacement of MAVROS or ArduPilot.

The combined historical `evidence/experimental_source.patch` is not selected:
its SIM_JSON instrumentation/header-access and boolean changes are represented
by the two selected patches. Copies of the signed TIMESYNC proposal under old
`code_after/` snapshots are also omitted; they are not a second runtime patch.
All those historical files remain on disk unchanged.

## New clean-checkout camera-only path

`linux_gpu/prepare_context.py` exports only the exact recipe/COPY sources into
NEW ignored tiny contexts. This preparation and pinned VIO export were verified;
no upstream checkout or recording is sent as Docker build context. Camera recipe
includes ROS Jazzy, pinned plugin, build dependencies, Mesa tools, native witness,
NumPy/OpenCV/Matplotlib and optional perf. SITL recipe pins4.4.4, applies exact
parser/trace patches and requires real parser C++ tests to pass. Linux Docker
builds, package availability, driver integration and runtime are NOT TESTED here.
Mutable distro/base repositories prevent a bit-identical build guarantee.

Camera-only Compose starts no VIO/MAVROS/transport, publishes no host ports and
leaves navigation source/GPS unchanged. Source-only preparation can be validated
without those generated binary/overlay dependencies. All generated contexts,
VIO exports and results stay under ignored `linux_gpu_runs/`.
