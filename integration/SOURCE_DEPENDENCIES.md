# External inputs and source-only reproduction boundary

**HOLD. No forwarding, fusion or flight approval.** The allowlist contains no
full upstream repositories, compiler output, shared libraries, firmware binaries
or raw recordings. The source can be inspected and its pure tests reproduced;
full runtime provisioning is still an explicit external prerequisite.

| Dependency | Recorded version/pin | Expected provision |
|---|---|---|
| ArduPilot Copter | [9279df6191a772c098b2e4797c95ac954f258d88](https://github.com/ArduPilot/ardupilot/tree/9279df6191a772c098b2e4797c95ac954f258d88), 4.4.4 | Fetch upstream recursively outside the commit; build only a new copy with the included boolean/instrumentation patches and actual C++ tests |
| ArduPilot Gazebo plugin | [f116b03552d05c8d0b0bed5ed67033a1183121d1](https://github.com/ArduPilot/ardupilot_gazebo/tree/f116b03552d05c8d0b0bed5ed67033a1183121d1) | Ignored `ardupilot_gazebo/source/` or an external build context; do not vendor checkout/meshes/build tree |
| Gazebo / ROS | Harmonic / ROS 2 Jazzy, Linux ARM64 | `gz.transport13`, `gz.msgs10`, `ros_gz_bridge`, rclpy and ROS message packages |
| MAVROS / libmavconn | 2.16.0 | Packaged vision plugins plus separate included `mavros_time_patch/`; original sys_time excluded |
| Python | Python 3.10+; NumPy, OpenCV; Matplotlib for figures | Existing runtime packages; no install or version upgrade during this audit |
| Teammate VIO | `vio_core.py`, `vio_frontend.py`, `vio_ros_source.py`, `vio_ros_out.py` | Provided separately by teammate. `vio/` is untracked here and **not** included or modified by this allowlist |
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
for fresh-machine runtime reproduction. No build/pull is performed here.

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
`test_sim_json.cpp`. Bootstrap path parameterization and a reviewed artifact
import are still needed before calling the source package turnkey reproducible.

[build_mavros_time_patch.sh](build_mavros_time_patch.sh) can build the included
overlay only into a **new** caller-selected output, using a new disposable
builder and the existing base image. It does not patch the original install.
No build output belongs in the source allowlist.

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
