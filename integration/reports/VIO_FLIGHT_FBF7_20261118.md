# VIO watchdog failure, run 20261118-flightfbf7d8854ed1

Evidence: `integration/phase1_runs/20261118-flightfbf7d8854ed1/evidence/`.
Code that ran: `53eeddf` (source hashes in `flight-source/configuration.json`); it did
not include `9f3226a` (flow consistency check). Tests: `tests/test_vio_ground_fixture.py`.

## Result

`VIO availability watchdog`, 5.9 s into CLIMB. Last sample delivered: acquisition 42.3 s;
watchdog at sim 42.904 (500 ms host age). Source output resumed at 43.0 s.

| sim t | source | reason |
|---|---|---|
| 42.2 | published | flow +0.40 m/s east, truth ~0 (wrong, published) |
| 42.3 | published | |
| 42.4, 42.5 | withheld | distributed flow disagreement (audit) |
| 42.6 | withheld | frontend: inliers |
| 42.7, 42.8 | withheld | readiness hysteresis 1/3, 2/3 |
| 42.9 | skipped | admission (publication age) |
| 43.0 | published | |

## Root cause: the flight ground fixture is a 2 cm checkerboard

`run_simulated_flight.py:91` installs `vio_ground_fixture.py --size-m 4 --tile-size-m .02`.
A checkerboard maps onto itself under a one-tile diagonal shift, so flow is ambiguous by the
lattice {(2a, 2b) cm : a+b even}.

* Every airborne flow error with >= 19 inliers is such a lattice vector; level frames that
  reached the EKF are exact to 0.3 cm (e.g. 47.1: true (+0.60, -2.06) cm, measured
  (-1.39, 0.00) cm = true - (2, -2)). 4 wrong velocities were published (42.2, 43.2, 50.5,
  50.7; up to 1.2 m/s).
* 2 cm tiles shrink to ~3 px at 1.3 m, so features collapse with height (200 inliers on the
  ground, 15-50 in flight), which drives the audit/inlier rejections and readiness resets.
* Reproduced with the pinned frontend on rendered views of the same fixture texture,
  including the exact 47.1 error and the inlier collapse during a 0.8 m/s climb.

The camera-probe frames in `evidence/integrated/` predate the fixture (host log lines 8-10);
they show the bare runway (textureless) and the drone's shadow covering ~half the image.

## Not the cause

* FCU messages `vision reception not ready` / `aided window shorter than 5s` /
  `DataFlash witness is stale/future`: pre-flight polling of 20 DataFlash snapshots while
  vision accumulated; resolved (`current_DataFlash_fusion_witness_verified`) before arming.
* Transport: all 199 published samples received, acquisition stamps strictly increasing,
  covariances finite and PSD. 40 unmapped = 23 before the clock reference, the reference
  sample itself, 16 after the watchdog latch.
* Coordinate frames / signs: flow matches truth wherever it is not aliased
  (e.g. 44.6-45.0), and the render bench confirms NED signs end to end.

## Changes (VIO side)

* `vio_ground_fixture.py`: opt-in `--pattern noise`, a non-repeating multi-scale texture
  (detail from --tile-size-m to 16x). Default `checker` output and SDF are byte-identical,
  so the DISARMED validations (`run_phase1_validation.py`, hash-bound) are unchanged.
  Bench: exact flow (< 0.5 mm) at 0.19-1.3 m and during a 0.8 m/s climb, 150-200 inliers,
  no texture period.
* Already on `main` (`9f3226a`): with the 2 cm checker the flow check rejects every
  airborne flow, including all 4 wrong published ones (replay of this run), so VIO fails
  closed instead of publishing wrong velocities.

## Required outside VIO (flight runner owner)

`run_simulated_flight.py:91`, scene only, no sensor/physics/model change:

    --size-m 4 --tile-size-m .02   ->   --size-m 16 --tile-size-m .02 --pattern noise

16 m covers the camera footprint (about 4.1 x 3.0 m at 1.3 m) plus drift; 6400 px texture.
`run_simulated_flight.py` is not in `BOUND_SOURCES`; the flight plan recomputes its own
digest, so the DISARMED proof stays valid.

## Open risks after the texture change

* Readiness needs 4 frames; one withheld frame still costs >= 0.5 s, i.e. the watchdog.
* The drone's shadow is large in the image at takeoff; on a textured ground it is a
  minority of features, but this is untested in flight.
* Frontend search range: at 0.19 m it cannot follow > ~8 cm per frame (fails closed).
* Nothing here has been flown.
