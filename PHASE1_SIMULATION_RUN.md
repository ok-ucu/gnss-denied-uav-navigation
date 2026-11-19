# Phase 1 simulation run: no-shadows (C1) and single-rejection tolerance (C2)

For whoever runs the next Phase 1 simulation on the Linux NVIDIA host.
Phase 1 stays **HOLD** until this run passes. Nothing here is verified in a
simulator yet; that is what the run is for.

## What changed on `main` since the last flight (abe2)

| Item | What | Default |
|---|---|---|
| C1 | Gazebo sun `cast_shadows=false` in the camera image and both runtime worlds; Gazebo refuses to start if the active world still casts shadows (`launch_phase1.verify_runtime_world`) | **on** (always) |
| C2 | `--tolerate-single-rejection`: a ready VIO gate survives ONE frame rejected only for image quality (inliers, contrast, distributed flow, flow consistency). That frame is still not published; a second consecutive rejection blocks as before; the tolerance re-arms only after 3 clean frames spanning ≥250 ms | **off** |
| B4 | Non-repeating noise ground fixture, abe2/fbf7 evidence and tests | on |
| A/B | Audit repairs: tests, diagnostics, tooling. No gate, watchdog, VIO pin or transport limit changed by these | — |

C1 changes bound sources (`launch_phase1.py`, `phase1.compose.yaml`), so old
runtime candidates and DISARMED proofs are **invalid**. Everything below must be
fresh.

What to expect from each fix, based on the recorded flights:

- **abe2 (liftoff failure)** began with two image rejections in a row (shadow).
  C2 would not have helped; only C1 can fix that.
- **fbf7 at 45.3 s** had exactly one rejected frame followed by a 500 ms
  readiness rebuild. C2 is for that case: the gap becomes 200 ms.

## 1. Get the code

The launcher only runs from a branch named `phase1-integration-merge` or
`gazebo-sitl-integration` (deliberate, `launch_phase1.validate_branch`). Use a
new checkout and name the local branch accordingly:

```bash
set -euo pipefail
: "${LINUX_CHECKOUT:?new absolute directory}"
test ! -e "$LINUX_CHECKOUT"
git clone https://github.com/ok-ucu/gnss-denied-uav-navigation.git "$LINUX_CHECKOUT"
cd "$LINUX_CHECKOUT"
git switch -c phase1-integration-merge origin/main
git rev-parse HEAD            # record this; it is REVIEWED_SHA below
git cat-file -e d33f66008fd7b6de332c96dd324b8ddbd320c194^{commit}   # pinned VIO source
```

## 2. Follow `integration/linux_gpu/NO_SHADOWS_PREPARATION.md`, from section 2

**Skip its section 1** (bundle transfer); step 1 above replaces it. Set
`REVIEWED_SHA` to the commit you recorded above wherever the guide asks for it.

1. **Section 2**: build the camera image, verify both worlds, create the
   runtime candidate. The image build must print
   `Phase 1 sun cast_shadows=false; replacements=1`.
2. **Section 3**: fresh DISARMED fusion proof (needs its usual approval).
3. **Section 4**: raw ground camera evidence. Check before any flight:
   - no drone silhouette in the downward frames on the ground and at ~1 m;
   - roughly **≥150 inliers** on the ground (abe2 had 24-40 with the shadow).

   If the silhouette is still there, **stop**: C1 did not take effect.

## 3. Flight

Use the commands in `integration/README.md` ("Separate FIRST simulated flight
scenario"), with the DISARMED baseline from step 2. Each flight needs its own
explicit approval (`--execute-simulation-flight`).

Recommended order, so the two fixes can be told apart:

1. **Flight A: without `--tolerate-single-rejection`.** Tests C1 alone.
2. **Flight B: only if** flight A ends on the VIO watchdog after a **single**
   rejected frame. Same command plus `--tolerate-single-rejection`. Needs
   approval from the safety owner: it loosens the readiness gate.

The flag is recorded in `evidence/flight-source/configuration.json`
(`tolerate_single_rejection`). Tolerated frames appear in
`gate_transitions.json` with `"tolerated": true`, and in `skips.jsonl`
(`gate_suspended`, `gate_tolerated`).

## 4. After each flight

```bash
python3 -B integration/analyze_simulated_flight.py "$P1_FLIGHT" \
  --bin "$P1_BIN" --output "$P1_FLIGHT/analysis-postflight-01"
```

Send back: `REPORT.md`/`report.json` from that output, the run's
`evidence/flight-source/{configuration.json,skips.jsonl,gate_transitions.json}`,
the ground camera frames from section 4, and the commit SHA. On failure, also
the time and reason of the first withheld frame after the last publication.

## Stop conditions (unchanged)

Stop on any failed preflight, latch, heartbeat, clock, permission or source
check, partial or late send, or incomplete cleanup. Do not retry with weakened
limits, edited PASS flags or reused evidence.
