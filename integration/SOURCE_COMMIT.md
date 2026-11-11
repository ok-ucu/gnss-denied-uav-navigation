# Linux GPU source handoff scope

**Phase1 HOLD; execution_allowed=false; forwarding_enabled=false.**
[SOURCE_FILES.txt](SOURCE_FILES.txt) is the exact source-package allowlist.
[Handoff](LINUX_GPU_HANDOFF.md) · [Dependencies](SOURCE_DEPENDENCIES.md).

Include the existing safety/clock/transport and VIO timing changes since9298da8,
their regression tests, Linux camera-only recipes/probes/preparation helpers,
current status and concise measured evidence summary. The original native
witness and sampling/analyzer source were promoted from ignored run folders;
raw captures and compiled probes were not. Existing parser/trace patches,
model/SDF inputs, regression fixtures, MAVROS overlay source and generated
MAVLink protocol source remain tracked. Teammate VIO is an externally pinned,
verified dependency, not copied into this commit.

Exclude all raw run folders, CSV/JSONL/PCAP/BIN/TLOG/perf captures, PNG/PPM,
firmware/shared libraries, build/install trees, full upstream repositories and
all teammate directories. Unselected legacy integration tools/docs remain on
disk untracked, not deleted; they are unnecessary to this camera-only workflow.
`linux_gpu_runs/` joins the precise generated-directory exclusions. No global
extension rule hides required source/fixtures. Root .gitignore is unchanged.

The handoff's full source set is closure-tested in a clean temporary tree with
only the four pinned external VIO files mounted read-only. No historical run
folder/image/binary is provided to those tests. New Linux builds/hardware tests
are NOT TESTED; preparation, Compose resolution and source/test validation do
not claim runtime or continuous normal PASS. See validation result below.

Staging uses an explicit per-file list of changed/new allowlisted paths; never
`git add .`, `git add -A` or a directory add. Before commit check cached names,
sizes, diff --check, import/path closure, links, secrets and safety state. Existing
tracked protocol source (2.26MB) is not a new capture/binary. No force push or
other branch changes are authorized.

## Validation result

Clean-tree ARM64 runtime: **137/137 offline tests PASS, no skips**; only the
selected source files and four pinned external VIO files were available.
Passive native witness compiled against the installed ABI without simulation.
Python AST70files, shell syntax, both exact parser patches applied to committed
upstream fixtures, DRI/NVIDIA Compose resolution, primary capture Compose config,
Markdown links, secret patterns and source/COPY dependency closure PASS.
Historical evidence was absent from the clean tree. Final staged checks repeat
link/scope/size/secret and diff validation after documentation is finalized.
New Linux image builds, hardware GPU use, camera-only live run and full normal
remain NOT TESTED for this handoff. Existing normal FAIL is not relabelled.
