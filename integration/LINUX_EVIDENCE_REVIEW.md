# Independent audit — 14 November 2026

**HOLD; execution_allowed=false; forwarding_enabled=false.**
Base HEAD `35355f4f5489b8f16aa32fa93cb5cc242b7fc952` on `gazebo-sitl-integration`.
Existing untracked legacy/teammate files remain unchanged. No remote access,
Docker build, live test, parser change, commit or push was performed here.

## Work classification and minimal changes

|Task|Class|Current evidence / next action|
|---|---|---|
|Malformed-prefix/CRC, BAD_DATA, fragments, silence/EOF/connect failures|Offline now|New real-codec + synthetic-socket regressions; no MAVLink TX or parser behavior changes|
|Unexpected source IDs and MAVLink1|Offline now / runtime policy still HOLD|Characterize current acceptance explicitly; these tests cannot prove FCU identity or v2. Choose/verify expected IDs from actual Linux evidence before a policy patch|
|Frames/covariance|Offline now|Existing analytic NED→ENU, FRD→FLU, quaternion and full cross-covariance tests; native ROS/wire delivery still requires runtime|
|TIMESYNC and deadlines|Offline now + Linux evidence|Signed-offset overlay and existing boundary tests remain unchanged. Native plugin/offset/replay proof must be supplied|
|Evidence packaging|Offline now|Existing collectors save commands/logs but have no ZIP exporter or universal stage summary. New explicit-file packager only hashes/packages provided claims|
|T2000 GPU/vendor/build results|Requires Linux evidence|Reported PASS, not independently verified here; need source diff, image/binary provenance, actual renderer and build logs|
|DISARMED heartbeat|Requires verified live runtime|Reported BLOCKED; need TCP bytes/chunk timing, exception, version and sender IDs|
|Camera-only + VIO OFF/ON/OFF|Requires verified live runtime|Not confirmed; existing commands unchanged; independent observer and full-window continuity required|
|Capture transport / native admission / fusion|Requires verified live runtime|Still HOLD; no approved amd64 clock implementation, continuous normal or live-camera fusion evidence|

No VIO/performance tuning is justified by the untransferred Linux results.

## DISARMED helper: what the code actually establishes

`linux_gpu/disarmed.py` loads the pinned generated codec, whose default
`robust_parsing=False` raises MAVError on bad prefix/CRC. Fragmented valid
packets accumulate in the parser. A missing heartbeat times out; EOF and connect
errors fail. Only receive calls occur. A robust-mode BAD_DATA fixture exercises
the actual codec separately, not a changed production parser.

The helper returns on the first heartbeat with armed bit 128 clear. It does not
filter source system/component/autopilot role, enforce MAVLink2, prove sustained
DISARMED state or authenticate the sender. Existing valid/armed tests are kept;
new identity/version tests deliberately characterize these **known limitations**,
not endorse them. The full source witness checks ArduPilot role and keeps
monitoring heartbeat, but that also is not authenticated source-ID verification.

Malformed prefixes are a possible mechanism, **not the diagnosed Linux cause**.
Without the raw Linux stream/traceback, enabling robust parsing or inventing an
expected source ID would be speculative. Keep the live heartbeat gate BLOCKED.

## Other reviewed contracts and safety defaults

- Frame bridge preserves acquisition stamps and uses world NED→ENU and body
  FRD→FLU separately. Orientation is N*R*B; all 6×6 cross terms are rotated.
  The adapter rotates body velocity/covariance into world ENU before MAVROS.
  Odometry lacks joint pose/twist covariance; rotation uncertainty is not a
  calibrated addition. PSD alone is not covariance consistency with real error.
- TIMESYNC overlay uses signed offset arithmetic and a distinct plugin; the
  original sys_time must remain excluded. Replay PASS does not prove FCU clock
  mapping. reset_counter remains 0; durable upstream reset/epoch guard is needed.
- Deadline analysis distinguishes enqueue, userspace receive and final check.
  The historical 20.792916 ms aggregate exceeded 17.1195 ms remaining; sender,
  kernel arrival, socket queue and receiver scheduling are not individually
  measured. Final late validation stays rejected; no TTL extension is applied.
- Launcher defaults to preparation, candidates cannot execute/self-admit, clock
  mapper still accepts only its reviewed binary. No vision egress switch exists
  in capture transport. The 180 ms publication, 200 ms clock and 500 ms availability
  limits and persistent reset/permission semantics remain unchanged.
- Explicit legacy execution tools in the untracked tree are not source-package
  entry points and were not run. No ARM/TAKEOFF/GPS/EKF command is sent here.

## Exact evidence requested from the Linux teammate

1. `git rev-parse HEAD`, `git status --short`, complete `CHANGES.patch`, and
   copies of required untracked source/config files (git diff omits them).
   Do not apply patches here yet. Include the actual invocation/exit code.
2. Targeted Docker inspect (IDs/image IDs/platform, entrypoint/command, devices,
   CPU limits/network), versions and **only relevant GPU/ROS vendor-selection
   variables**, actual EGL/GL renderer + matching Ogre log. No credentials or
   full environment dump.
3. Firmware SHA, pinned commit/patch/context/compiler/package records, actual
   parser 5/5 output; MAVROS 2.16.0 versions, overlay SHA and plugin-load console.
4. TIMESYNC suite.json, per-case result/console/wire/responses for ±offset,
   zero/jitter/drift/domain/reset, including expected negative case. Raw messages
   and duplicate/count checks matter, not just a terminal PASS statement.
5. For heartbeat BLOCKED: full exception/traceback, stdout/stderr and bounded
   raw TCP stream on the actual FCU link, original receive-chunk sizes and host
   monotonic timing if available, a short hex prefix, decoded valid source IDs,
   MAVLink version and armed bit. State which parser settings were actually used.
   A raw recording alone does not recover original TCP recv chunk boundaries.
6. Camera/source results only if actually run: commands, DISARMED before/after,
   native camera/clock/IMU, IPC/CPU/stages, independent observer, SOURCE_RESULT,
   cleanup status. Otherwise label NOT TESTED/BLOCKED, not PASS.

## Offline evidence ZIP, not a new probe

Stop collectors or copy their closed outputs first. In a NEW export folder,
create FILES.txt with **explicit relative files**, one per line: command log,
source diff/required local sources, relevant logs and summaries. Do not include
whole repositories, Docker images, builds, private keys or credentials. The
tool rejects symlinks, traversal/reserved paths, duplicate selection, overwrite
and a 256 MiB byte budget overflow. It is **not a secret-redaction tool**: review
the explicit selection and contents before sharing.

Provide summary.json, for example (replace fixture text with actual claims):

```json
{
  "source_commit": "35355f4f5489b8f16aa32fa93cb5cc242b7fc952",
  "dirty": true,
  "changes": "CHANGES.patch",
  "commands": "commands.jsonl",
  "decision": "HOLD",
  "execution_allowed": false,
  "forwarding_enabled": false,
  "stages": [
    {"name": "heartbeat", "status": "BLOCKED", "evidence": ["heartbeat.log"], "note": "Actual error and traceback retained"},
    {"name": "vio_source", "status": "NOT TESTED", "evidence": [], "note": "Not run"}
  ]
}
```

```bash
python3 -B integration/package_linux_evidence.py \
  --root /path/to/closed-export \
  --files /path/to/closed-export/FILES.txt \
  --summary /path/to/closed-export/summary.json \
  --output integration/linux_gpu_runs/NEW-linux-evidence.zip
```

Output parent must already exist. PASS entries require selected evidence, but
the tool does **not** validate their truth. `_bundle/summary.json` always records
claims_verified=false and HOLD; `_bundle/manifest.json` binds included bytes to
SHA256. Unknown source completeness/authenticity remains unknown. Same inputs
produce identical ZIPs under the same Python/zlib implementation (fixed entry
ordering/timestamps/modes); no cross-compressor byte guarantee is claimed.
Source files are never modified. ZIP output stays in ignored runtime folders;
no archive is transmitted or added to Git automatically.
