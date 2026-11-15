# Stage 2 — witness-only isolation and evidence (offline only)

**Phase 1 HOLD / NO GO. `execution_allowed=false`, `forwarding_enabled=false`.**
**Implementation verdict: STAGE 2 PARTIAL.** Portable paths are implemented;
native role bootstrap, authenticated IPC/network enforcement and manager-owned
recovery are still BLOCKED. Offline validation: 298 Phase 1 tests collected,
296 PASS / 2 native SKIP, including 39 new Stage 2 regressions. This is not a
Linux runtime or containment PASS.
Person 1's offline implementation approval is not permission to connect to an
FCU, inject a live fault or create Linux containment/services. Native manager
recovery remains BLOCKED as described in [Stage 1](HEARTBEAT_STAGE1.md).

## Implemented paths

The [existing shell launcher](run_vio_source_probe.sh) recognizes `--witness-only`
before Linux/ROS/VIO prerequisites, cleanup traps, fixture and ON-phase. The
entire legacy body is unchanged. It dispatches only to
[heartbeat admission/consumers](heartbeat_witness.py), never the VIO launcher.

```bash
# Admission shape ONLY. Currently returns BLOCKED/exit 2 without connecting.
# This command is NOT a runtime approval or a request to launch this test.
bash integration/run_vio_source_probe.sh --witness-only \
  --plan /path/to/reviewed-test-plan.json --ledger /path/to/ownership.json
```

No endpoint is implicit. Plan schema 2 requires mode `witness-only`, matching
test UUID/ledger hash, both safety flags false, explicit expected system/component,
separate runtime approval reference and endpoint fields: transport/address/port,
host network namespace, FCU output identity, current client inventory,
noninterference evidence SHA-256 and Person 1 approval reference. These strings
are declarations, **not authentication**. A trusted host authority must verify
endpoint noninterference and network containment; then Stage 1 independently
registers/rechecks recovery and releases its barrier before role startup.
Default `WitnessAuthority` cannot satisfy these checks. No environment flag,
JSON boolean or `--allow-*` option bypasses the barrier. No connector is supplied.

## RX and silence contract

[Collector/proxy](heartbeat_fault_proxy.py) accepts already-created descriptors
from a trusted bootstrap. Synthetic fixtures use only private `127.0.0.1:0`
listeners. There is no FCU default, `connect()` implementation or image/runtime
launcher in this module.

- Only the collector has `RxOnlyIngress`; `send`, `sendall` and `write` latch
  `UPSTREAM_WRITE_ATTEMPT` and fail before any upstream TX syscall. Application
  writes and rejected attempts are separate counters.
- Two one-way OS pipes expose only read descriptors to control and proxy.
  Independent bounded nonblocking queues preserve byte order; slow witness
  delivery cannot block control. Overflow/disconnect is a latched failure.
- Proxy state is `RELAY → SILENCE → CLOSED`. Trigger requires matching UUID,
  private controller capability and a readiness proof issued by the witness
  after protocol-valid DISARMED readiness: >=3 heartbeats spanning >=2 seconds.
  The registry verifies the issued object's identity and the actual guard's
  current latched state; copying proof fields does not create authorization.
  Wrong/early/repeated trigger fails closed; no return to RELAY.
- SILENCE discards pending witness bytes, continues draining upstream input and
  leaves witness TCP open. Logs record trigger time, offsets, last send,
  discarded bytes, sequences and final counters. Control continues independently.
- In-flight kernel bytes can arrive after cutoff. Staleness is measured from
  the witness's actual last accepted heartbeat, never just the trigger time.
- Every consumer iteration checks the existing monotonic freshness guard.
  Stale >3 seconds, EOF/disconnect, malformed packets, armed/wrong-ID heartbeats,
  missing initial heartbeat and stale partial packet are distinct outcomes.
  If EOF/reset or queued late bytes are observed at the stale boundary, they
  cannot be classified as expected open-connection silence.
  BAD_DATA cannot establish readiness. A buffered partial packet at timeout is
  `PARTIAL_MAVLINK_STALE`, not a successful expected-stale test.

Python capability separation is not an OS security boundary. The native host
bootstrap must place roles in separate processes, prevent descriptor inheritance,
authenticate private control IPC/peer identity and deny FCU connections from
proxy/witness with reviewed network namespace/firewall policy. It must also prove
zero upstream application writes at that boundary. This native integration is
**NOT IMPLEMENTED / BLOCKED**, not inferred from synthetic counters. The real
local control-subprocess regression inherits only its pipe read descriptor.
Collector/FCU connection is a shared dependency: its failure is never expected
witness-only silence PASS.

## Witness and bundle evidence

[Evidence implementation](heartbeat_test_evidence.py) uses Stage 1's exclusive
atomic no-overwrite file publication, file fsync and directory fsync. Every
new-schema JSONL row has test UUID, contiguous sequence and host monotonic ns.
TCP receipt ns is recorded separately from journal-write time. No ROS/simulation
timestamp is substituted for this host clock.

1. `witness.ready.json`: atomically published after verified readiness, with
   count, first/last receive times and native-event cursor.
2. `witness.failed.json`: atomically published immediately on latched failure,
   before potentially slower final export. Witness stops consuming after failure.
3. `native.jsonl`: heartbeat-only TCP chunks/guard events, not camera/IMU capture.
4. `resources.json`: own-process resource snapshot; no simulated sensor proof.
5. `witness.json`: timestamps, bytes/counts, failure/exit status and source IDs.
6. `witness.complete.json`: last witness marker, only after writer closure,
   parsing and SHA-256/size/count validation of its finalized files.
7. `collector.jsonl`, `proxy.jsonl`, `proxy.json`, `control.jsonl`, `control.json`,
   `watchdog.jsonl`, `ownership.json`, `admission.json`, `cleanup.receipt.json`
   and its referenced cleanup journal: separate writers/authority.
8. `bundle.manifest.json`: exact required relative-path set with SHA-256,
   sizes and JSONL record counts, after all writers stop and cleanup is verified.
9. `bundle.complete.json`: separate final verdict referencing manifest and
   receipt hashes. Witness completion alone can never imply final PASS.

Verification parses strict JSON/JSONL, rejects duplicate keys/nonfinite numbers,
symlinks/path escapes, partial records, changed files, wrong hashes, same-size
corruption and inconsistent counts. It replays captured fragmented heartbeat
bytes through the pinned MAVLink codec, validates readiness/stale timing, control
continuity, unchanged witness byte prefix, proxy offsets and cleanup receipt.
Copied directories verify without rewriting original paths or evidence.

Read-only verification of an exported copy (no runtime or file creation):

```bash
python3 -B integration/heartbeat_test_evidence.py \
  --directory /path/to/copied-bundle --ledger /path/to/copied-bundle/ownership.json
# Exit 0 = scoped synthetic PASS, 1 = FAIL, 2 = INCOMPLETE; always HOLD.
```

Synthetic evidence can report `PASS` **only for the synthetic expected-failure
fixture**, always marked `synthetic_expected_failure_only` and HOLD. Native JSON
claims cannot produce PASS while authority/backend verification is unavailable.
Missing cleanup receipt, missing logs, partial export or absent final manifest
is INCOMPLETE, never PASS. A verified behavior violation remains FAIL even when
some other proof is missing. Finalization stays under the original Stage 1
deadline; no new timeout budget is introduced. Filesystem durability failures or
post-write deadline overruns prevent a successful finalization result; ultimate
power-loss/host-failure guarantees remain unverified.

A known error after provisional final-marker publication records
`bundle.finalization.failed.json`; verification rejects the revoked marker.
Preserve this file when copying/exporting failed evidence. If storage cannot
persist revocation, the failed result must remain in independent manager logs;
a file copy alone cannot certify historical fsync or deadline completion. This
native external acknowledgement remains BLOCKED with the manager integration.

The [legacy evidence action](vio_source_probe.py) dispatches schema 2 to the new
hash verifier. Old capture schema/behavior and historical evidence are unchanged;
they are not silently upgraded to the new integrity contract.

## Offline regression commands

No Docker, FCU, ROS, Gazebo, VIO or MAVROS process is needed:

```bash
PYTHONPATH=integration:integration/tests python3 -B -m unittest discover \
  -s integration/tests -p 'test_heartbeat*.py' -v
bash -n integration/run_vio_source_probe.sh
python3 -B integration/audit_source_scope.py
git diff --check
```

TCP tests require OS permission for synthetic loopback listeners, not network
access to a teammate/FCU. No fallback replaces denied TCP tests with a mock PASS.
Phase 1's full source-allowlisted suite additionally uses the existing pinned VIO
export and installed NumPy/OpenCV environment for offline algorithm regressions.

## Before any separately approved Linux test

- Complete Stage 1 native manager/timer/cgroup/pidfd/reaping validation.
- Approve exact endpoint, existing-client noninterference and numeric budgets.
- Implement/review native process bootstrap, authenticated control IPC and
  receive-only network enforcement; retain all recovery/ownership authority.
- Verify the entire final evidence bundle and deadline export under real Linux
  crash conditions. Do not treat synthetic control/cleanup as native proof.
- Obtain separate runtime approval. No live command is authorized by this file.

No Phase 1 readiness, fusion, source continuity or flight success is claimed.
