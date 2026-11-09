# Proposed source-only staging scope

**Review only. Nothing has been staged, committed or pushed. Phase 1 HOLD;
`execution_allowed=false`, `forwarding_enabled=false`.**

[SOURCE_FILES.txt](SOURCE_FILES.txt) is the exact allowlist. Do not replace it
with a directory-wide add. The list includes both ignore files, this review,
the allowlist itself and the source-audit tool. It deliberately excludes local
historical reports whose evidence links would not exist in the source package.

| Include | Why |
|---|---|
| Current capture-only launcher, Compose/MAVROS/profile configuration and runner | Reproduce the present owned integration path, subject to documented runtime prerequisites |
| VIO/frame/velocity/covariance conversion, quality, clock, persistent epoch/egress guards | Actual integration implementation and safe failure handling |
| Analysis modules imported by tests; all 17 unit-test files | Complete import closure; no tests discarded to make a package pass |
| Parser patch, trace patch, actual C++ parser tests and build/mount helpers | Unique source currently stored inside experiment folders |
| Four model/world/sensor SDF/config inputs and three small regression fixtures | Required paths for launcher and existing tests, not a raw recording dump |
| Actual MAVROS overlay package, signed-offset diff and build helper | Rebuild the separate plugin without vendoring full MAVROS |
| One generated MAVLink v2 Python dialect | Current runtime dependency at an exact path, 2.26 MB of protocol **source**; generated logs are excluded |
| Selected recorded artifact SHA manifest | Current launcher checks selected source/artifact hashes through this manifest; binaries and other historical data remain excluded |

Full per-file names are in the linked allowlist; exact current sizes and ignored
status are produced by [audit_source_scope.py](audit_source_scope.py). No selected
file is a BIN/TLOG, PNG, frame recording, archive, compiler object or full upstream
checkout. See [dependencies](SOURCE_DEPENDENCIES.md) for third-party pins,
generated-source provenance and fresh-machine runtime limitations.

## Exclude from this commit, preserve locally

- All raw `phase1_runs/`, camera/VIO/IMU, EKF/fcu, navigation and other run folders.
- `isolated_blockers/20261109_a/work/` and `state/`; almost all `evidence/`.
- Full `ardupilot_gazebo/source/`, its build directory, full MAVROS archive,
  generated `patch_install/`, colcon/waf outputs and local Docker artifacts.
- Private inventories, absolute-host-path `compose.env`, PID files, state stores,
  raw frames, images, CSV/JSONL, BIN/TLOG/PCAP and timing captures.
- Historical code snapshots and superseded duplicate build/source copies.
- Unselected root diagnostic/flight tools and historical Markdown are **source**,
  not garbage: they remain locally visible/untracked for a later separate review.
  This allowlist is the minimum current integration/test package, not a request
  to delete or globally ignore other unique source.
- `vio/`, `gazebo/`, `camera/`, `sitl-stand/`, `calibration/`, `dataset/` and the
  unrelated root flight script are outside scope. No teammate paths are selected.

Ignore rules are rooted directory paths with exact exceptions for required
source/fixtures. Global `*.csv`, `*.jsonl`, `*.bin` and generic build-folder rules
were removed from the previously uncommitted root additions; they could hide
legitimate teammate/test inputs. Existing cache/environment/credential policy
remains. No raw file is deleted or moved, and no historical evidence is edited.

## Read-only review commands

```bash
python3 -B integration/audit_source_scope.py
git status --short
git diff --check
git diff --cached --name-only
```

The audit checks sorted/unique/existing paths, ignore-rule visibility, Python
import closure and AST, XML, package-local Markdown links, host-specific path
patterns and common secret signatures. Pattern scanning is not a security
certification. It reports external generated runtime dependencies separately;
their presence on this machine does not make a fresh clone turnkey reproducible.

**Manual command only after explicit review/approval; not executed here:**

```bash
git add --pathspec-from-file=integration/SOURCE_FILES.txt
git diff --cached --check
git diff --cached --stat
git diff --cached --name-only
```

Compare the final command's output against the allowlist before any commit.
Do not use force-add to override a failed ignore check. No commit or push command
is part of this preparation.
