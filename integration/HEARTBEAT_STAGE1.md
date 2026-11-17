# Stage 1 — ownership, cleanup and supervision (offline implementation)

**PARTIAL; Phase 1 HOLD / NO GO. `execution_allowed=false`,
`forwarding_enabled=false`. No runtime authorization.**

This implements portable policy plus a Linux kernel/Docker adapter boundary.
It does not implement the heartbeat proxy, witness-only launcher or any FCU
connection. Existing capture/VIO mode, algorithm, freshness/watchdogs, renderer,
MAVLink, TIMESYNC and forwarding are unchanged. Approval for offline code/tests
does not approve Linux containment setup, systemd changes or live fault injection.

## Code and trust boundaries

- [Ownership/cleanup](heartbeat_test_owner.py): immutable UUID ledger, full
  Docker ID/Created/image, host and inner PID, PPID/PGID/SID, proc starttime,
  boot ID, PID/mount/network/time/user namespaces, cgroup device/inode/path and
  explicit test-only process roles. The manager must inventory/protect FCU,
  SITL, Gazebo and foreign resources separately; they are never registered as
  workload targets. Names/labels/PGID/kill-0 are not signal authority.
- `LinuxPidfdCgroup`: verifies a real cgroup2 mount and retained directory
  identity, rejects root/outside paths and symlinks, binds individual pidfds
  around identity checks, checks again before TERM, writes `cgroup.kill` relative
  to the retained descriptor for final descendants, verifies `populated=0`.
  Empty cgroup does not prove zombies reaped; independent exit/reap proof is
  mandatory. No cgroup creation, migration, broad group signals or fallback.
- `DockerByIdentity`: bounded inspect of original full ID, secondary name
  cross-check, exact-ID stop only. Unknown/error/timeout is never stopped proof.
- `LinuxCleanupBackend`: combines kernel authority, Docker, host clock/domain
  checks and independent process-exit/foreign-preservation callbacks. Recovery
  itself must be outside the workload cgroup. No auto-discovery of targets.
- [Supervision policy](linux_gpu/heartbeat_test.py): `StartupBarrier` registers
  and independently rechecks manager authority before release;
  `IndependentWatchdog` requests manager recovery on liveness/authority failure
  or the reserved cleanup boundary. It never signals guessed IDs itself.

**Missing Linux integration:** `ManagerRecovery` deliberately raises BLOCKED by
default. No systemd units are installed/generated, no privileged manager or
root-controlled ledger provisioning is claimed. There is no production workload
launch entrypoint in this Stage 1 module. `StartupBarrier.release()` returns a
decision only. An arbitrary dict/token is not real manager registration proof.

A future reviewed host-manager adapter must supply `register`, `verify` and
`recover` with: persistent root-controlled private ledger; independent timer and
recovery helper; startup barrier; verified permissions; protected-container
inventory; outside-cgroup watchdog/helper identities; original container/cgroup
handles. Launcher/watchdog SIGKILL must not disarm recovery. No workload starts
unless this is independently verified. The bootstrap/init may only establish
identity while held behind the barrier; it must not connect to FCU.

Pidfds are process-local descriptors, not serializable integers in a ledger.
The live manager must retain the handles; recovery restart cannot reconstruct
authority from stale PID/PGID values. New handles require reviewed revalidation
under the startup barrier. Manager/boot/time-namespace changes invalidate
authority. This remaining native integration is **BLOCKED**, not a fake working
recovery guarantee.

## Deadline and cleanup contract

`Deadline` contains integer HOST CLOCK_MONOTONIC nanoseconds, boot/time namespace,
`start_ns`, immutable `end_ns`, and explicit positive TERM/KILL/verify/export
reservations. There are **no approved default numeric budgets**. Restoring the
ledger preserves `end_ns`; retries/restarts cannot rebase it.

```text
D = original start + explicitly approved total budget
TERM_end = min(cleanup_start + G_TERM, D - G_KILL - R_VERIFY - R_EXPORT)
KILL_end = D - R_VERIFY - R_EXPORT
verification/export ends no later than D
```

`CleanupController.run()` logs expected identities, revalidates kernel authority,
requests per-process TERM, bounded wait, final verified-cgroup KILL, Docker stop,
terminal/reaping and foreign-resource checks. Every backend operation receives
its absolute phase deadline. A Docker API failure does not cancel safe retained
kernel containment, but prevents a cleanup PASS. Expiry never creates a new
budget. Only authorized emergency containment can still be attempted after
expiry, with FAIL; no late retry, Docker stop or new observation window.

The backend is a trusted interface: an unbounded/blocking backend cannot be made
safe merely by this Python loop. Host-manager supervision, kernel blocked I/O,
host suspend and persistent-storage stalls remain native validation concerns.

The existing [source runtime](linux_gpu/run_source_runtime.py) gets only the
explicit fail-closed correction: retain initial full IDs, reject empty/missing
records and same-name replacement, bound final inspect, inspect error never
cleanup PASS. It still has legacy per-operation budgets/finally; it is **not**
the independent Stage 1 watchdog and must not be used as a substitute. Partial
Compose startup without an original-ID snapshot remains INCOMPLETE/manual
review, never an invitation to discover/stop by name.

## Durable evidence

`ownership.json` is exclusively published in a fresh manager-owned directory.
Per-attempt `cleanup.<uuid>.jsonl` events have test UUID, sequence, monotonic
timestamps, ownership/actions/errors. Events are flushed/fsynced. The cleanup
receipt records original container/cgroup, ledger hash, deadline, start/end,
terminal states, foreign preservation and event SHA-256/count/size.

`durable_new` uses exclusive temporary creation, file fsync, atomic no-overwrite
hard-link publication and parent directory fsync. Filesystems without those
semantics fail publication. Partial receipts are retained and never overwritten.
`verify_receipt` parses all event lines, validates identity/order/count/hash and
rejects same-size corruption. Relative paths permit verification of copies.

A receipt is **cleanup evidence only**, never a Phase 1/test final PASS marker.
The controller also checks the deadline after writing/re-reading it. If final
I/O overruns D it returns FAIL even if the provisional receipt was published.
The future manager must observe this post-write outcome and finalize the bundle
only after all writers stop, export hashes are re-read and the same D is met.
Host storage/power failure durability is not established. Full witness evidence,
final bundle manifest and heartbeat expected-failure verdict belong to later
stages; none are fabricated here.

- Cleanup PASS: identities verified, all terminal proofs true, foreign resources
  preserved, no errors, receipt verified and finalization before D.
- FAIL: observed survivor/foreign impact/deadline violation. Evidence loss cannot
  downgrade an observed violation into merely INCOMPLETE.
- INCOMPLETE: unknown ownership/state, inspect failure, missing receipt, malformed
  event export or unverified authority. Never PASS.

## Offline checks (no runtime)

From the repository root, with existing dependencies already available:

```bash
PYTHONPATH=integration:integration/tests python3 -B -m unittest discover \
  -s integration/tests -p 'test_heartbeat_test*.py' -v
PYTHONPATH=integration python3 -B -m unittest discover \
  -s integration/tests -p 'test_source_runtime.py' -v
git diff --check
```

Tests inject process/Docker/manager adapters. A separate real-local-subprocess
fixture verifies manager survival of launcher/watchdog SIGKILL, hung launcher,
TERM-resistant `setsid` worker cleanup and a surviving foreign control. Those
workers are direct children of the synthetic manager, which retains/reaps Popen
handles. This is **not** a real orphan/cgroup/systemd proof. The orphan/namespace/
PID-reuse cases are modeled; native orphan and manager recovery remain skipped.
No process-name/group-wide kills are used by the new code or fixture.

## Linux work still required — NOT TESTED, not authorized here

1. Implement/review the root-controlled manager adapter and independent timer,
   private ledger, ownership registration, init barrier and permissions. No
   timer registration/permission => BLOCKED before workload release.
2. Verify Linux pidfd behavior, proc namespaces, actual cgroup v2 permissions,
   retained handle replacement/deletion races and protected/foreign cgroups.
3. Native synthetic launcher and watchdog SIGKILL, TERM-resistant descendants,
   orphaned `setsid` child, reaping, Docker hang/replacement and API loss; use
   fresh separately approved test-only containment, never existing simulators.
4. Prove global deadline through systemd recovery and durable final export;
   repeated recoveries must retain original D and never act on PID reuse.
5. Only then review later witness-only/proxy stages, endpoint approval and
   receive-only isolation. Exact endpoint and numeric budgets remain unapproved.

**Stage 1 PARTIAL. Phase 1 HOLD / NO GO. No Linux runtime PASS.**
