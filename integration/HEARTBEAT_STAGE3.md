# Stage 3 — integrated OFFLINE synthetic expected-failure workflow

**Phase 1 HOLD / NO GO. `execution_allowed=false`, `forwarding_enabled=false`.**
This is an executable local fixture, not permission or proof for a real FCU,
Linux runtime, source continuity, ExternalNav or flight.

[Coordinator](synthetic_heartbeat_e2e.py) connects the existing
[Stage 1](HEARTBEAT_STAGE1.md) ownership/deadline/cleanup/watchdog policy and
[Stage 2](HEARTBEAT_LOSS_TEST.md) collector/proxy/witness/evidence components.
The native manager implementation remains BLOCKED. Default production
`StartupBarrier` and `WitnessAuthority` are unchanged and still deny startup.

## Actual execution

1. Fresh run UUID, private directory and one immutable `Deadline` are created.
   Source HEAD, branch, full Git status, staged/unstaged diff hashes and the
   source-allowlist SHA-256 snapshot are recorded; dirty work is not hidden.
2. The coordinator reserves its own `127.0.0.1:0` listeners. There are no host,
   port, FCU configuration, image, Docker or native-adapter arguments.
3. Owned launcher, watchdog, control and worker subprocesses bootstrap behind
   an authenticated activation barrier. A separate control fixture is excluded
   from primary cleanup. Retained direct `Popen` creation handles authorize
   signals; no name, guessed PID, group signal or kill-0 authority is used.
   Ledger container/cgroup/namespace identifiers are explicitly synthetic; its
   portable start token is bootstrap monotonic ns, not Linux `/proc` start ticks.
   The real PID/PPID/PGID/SID handshake and retained Popen handles remain distinct
   from the NOT TESTED native ownership/containment backend.
4. The test-only recovery adapter verifies the exact fixture/nonce/ledger/D.
   Stage 1 barrier policy is exercised with explicit synthetic dependency
   injection, never as Linux recovery proof. It refuses all production endpoint
   admission and cannot be selected by a native configuration flag.
5. Private AF_UNIX socketpairs carry [HMAC-SHA256 control](synthetic_heartbeat_ipc.py).
   Per-run/per-peer keys pass only through owned bootstrap pipes, not argv,
   environment or evidence. Run ID, sender, sequence and original D are bound
   into each MAC. Wrong keys/IDs, replay and unexpected commands fail closed.
6. The fixture generates actual synthetic MAVLink 2 DISARMED heartbeats.
   Collector fans out unchanged bytes over read-only pipes. An independent
   control process runs `HeartbeatConsumer`. The parent runs the witness/proxy.
7. Only after witness AND control readiness does an authenticated launcher
   request trigger `RELAY → SILENCE`. Source/control delivery continues while
   witness TCP stays open. Actual stale >3s is the expected witness failure.
8. The control process must receive a further synthetic heartbeat after witness
   FAIL. Witness export closes, Stage 1 cleanup runs with the SAME D, process
   exits/reaping and preservation of the excluded control are recorded.
9. Stage 2 verifies its bundle. Stage 3 adds the full source/IPC/process/timeline
   report and a portable outer SHA-256 manifest. No positive outer completion
   marker is published for FAIL/INCOMPLETE. A known post-publication error
   creates an explicit finalization-failure receipt; retained provisional files
   do not override that receipt. Storage/host failure durability is NOT TESTED.

The excluded control is itself created by the fixture. It stays alive through
primary cleanup, then is disposed with the fixture's own creation handle. No
external service or unrelated process is ever a target. A deliberate survivor
is recorded as FAIL before emergency fixture disposal; the failed cleanup
receipt is not rewritten as PASS afterwards.

## Clock and budget

Every Stage 3 component explicitly receives `host_now()` using
`clock_gettime_ns(CLOCK_MONOTONIC)`. Parent/child bootstrap clocks are bracketed
and a regression checks their common epoch. macOS Python 3.9's per-process
`monotonic_ns()` epoch must not be compared across child/parent. Acquisition
timestamps are not rewritten or shifted. Existing production clocks/guards are
unchanged. Missing shared kernel clock => failure, no relative-clock fallback.

The explicit fixture budget is 12..60 seconds; TERM/KILL reserves are 0.5s each,
verification 1s, export 2s. These are **synthetic test administrative settings**,
not approved Linux runtime budgets or changed heartbeat/watchdog thresholds.
Fixture heartbeats are generated every 0.5s. Readiness >=3 over >=2s and stale
>3s remain the existing guard criteria. All subprocess commands/observations
retain the original absolute D; no reconnect/recovery grants a new deadline.
If D is exceeded, late best-effort failure evidence/emergency disposal is
reported FAIL, never a fresh successful window or a bounded-native guarantee.

## Commands — synthetic only

From repository root, using installed Python dependencies only:

```bash
# This parent is already ignored by the source package. Never overwrite a run.
mkdir -p integration/source_scope_audit/stage3
python3 -B integration/synthetic_heartbeat_e2e.py run --synthetic-offline \
  --directory integration/source_scope_audit/stage3/example-normal \
  --scenario normal --budget-seconds 20

# Read-only, portable verification; no sockets/process launch:
python3 -B integration/synthetic_heartbeat_e2e.py verify \
  --directory integration/source_scope_audit/stage3/example-normal

PYTHONPATH=integration:integration/tests python3 -B -m unittest discover \
  -s integration/tests -p 'test_heartbeat_e2e.py' -v
```

Run a new directory per scenario. Loopback bind permission is necessary for real
synthetic TCP tests; denied bind is an environment failure, never mock PASS.
The scenario matrix runs 14 cases sequentially, including real readiness/stale
waits and an intentional 12-second deadline failure. A measured offline matrix
took about 73 seconds; quiet unittest output previously looked like a hang.
Each case now prints START/END, elapsed seconds, verdict, child exit codes and
teardown details. For a separately bounded targeted run on Linux (GNU timeout):

```bash
timeout --signal=TERM --kill-after=5s 180s env PYTHONPATH=integration:integration/tests \
  python3 -u -B -m unittest test_heartbeat_e2e -v
```

Timeout exit 124/137 is a failed external bound, never a synthetic PASS. SIGKILL
of the coordinator cannot run Python teardown; native independent recovery is
still BLOCKED. The external timeout is only a test-run bound, not recovery proof.
Ordinary exceptions and Ctrl+C now enter fixture teardown even during startup
or observation. Reaping has one shared 2-second cap, bounded by the original D
while D remains live; interrupts are deferred until all owned handles are tried.
After an already-expired D, a separately recorded maximum 2-second emergency
disposal may reap fixture children, but D stays unchanged and the result is FAIL.
Survivors, signal/reap errors or interrupted teardown cannot produce PASS.
CLI exit 0 = scoped synthetic test PASS; 1 = FAIL; 2 = INCOMPLETE.
The expected witness component exit is 1 even when the overall expected-failure
test passes. `source_continuity=NOT TESTED` always. Do not interpret any of these
as a live heartbeat-loss, Linux containment or Phase 1 PASS.

Scenarios: `normal`, `missing_readiness`, `unauthorized_trigger`,
`mismatched_run_id`, `control_interruption`, `unexpected_upstream_write`,
`launcher_hang`, `launcher_sigkill`, `watchdog_crash`, `ownership_mismatch`,
`cleanup_deadline_exceeded`, `incomplete_evidence`, `corrupted_evidence`,
`partial_export`, `surviving_owned_descendant`.

Missing/corrupted evidence scenarios damage only a verification COPY; originals
remain available. Partial export injects a per-call publisher error at the
inner completion boundary, without changing global publication behavior.
Launcher/watchdog crash scenarios use real owned subprocess signals. The
watchdog child runs `IndependentWatchdog` policy via authenticated RPC; the
synthetic supervisor retains recovery authority when that child dies. It is
not a systemd manager and has no cgroup/pidfd containment claim.

## Evidence and verdicts

`SYNTHETIC_E2E_REPORT.json` contains UUID, exact source state, peer identity,
timeline, immutable deadline, expected heartbeat-loss/component exit,
cleanup/foreign preservation, evidence status, message counts, supervision
requests, disabled safety flags and explicit Linux-native NOT TESTED fields.
`source.json`, `fixture.json`, `process_exit.json`, `control_ipc.jsonl`,
`timeline.jsonl`, `watchdog.jsonl`, the Stage 1 receipt and Stage 2 bundle are
retained. `synthetic.manifest.json` hashes available artifacts; invalid/empty
negative evidence is inventoried explicitly and cannot qualify for PASS.
`synthetic.complete.json` is only for a fully verified positive synthetic test.

PASS requires expected post-readiness stale failure, authenticated cutoff,
continuous independent control, cleanup PASS, all owned processes reaped,
excluded control preserved, evidence PASS and completion within D.
Verified behavior/supervision/ownership/deadline violations are FAIL. Missing,
corrupted or partial proof is INCOMPLETE unless a verified violation already
requires FAIL. Linux-native NOT TESTED never becomes PASS through this adapter.

Remaining blockers: reviewed Linux manager recovery/timer, kernel pidfd/cgroup
authority and orphan reaping, native role bootstrap/peer credentials/firewall,
approved FCU endpoint/noninterference, real crash/storage durability and separate
runtime approval. No Docker/systemd/Gazebo/SITL/VIO/MAVROS is used by this tool.
