# HP3 trial-002 return settling and recording performance

Status: complete diagnosis and fix; full autocal retry pending.

## Abstract

Two stopped manual browser recordings show that sampled extended recording increased
achieved playback from 0.570x to 5.980x while reducing file bytes per simulation
second by 11.18x. Bytes per wall second remained similar because simulation ran
10.49x faster. Trial-002 failed after the fourth edge-force trial: return-to-origin
waited for encoder stability while the 0.623633 N pull remained active. An isolated
production JS world using the saved HP3 scene and recorded torque reproduced the
30-second timeout and similar terminal angles; position mode then settled in two
simulation seconds. A synthetic control-flow regression fails before the fix and
returns to origin afterwards. Return routines now hold motors before settling,
including zero-distance paths. These checks do not establish full autocal success
or hardware behavior.

## Topics

HP3, autocalibration, return to origin, force tuning, encoder settling, Rerun,
wall clock, simulation clock, recording performance, sampling.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Recorder changes improve playback and storage per simulation second | E1 | Supported for these recordings | [Performance](performance.json) |
| The failed return waits with the trial force still applied | E2 | Supported | [Recorded events](failure-events.json) |
| Position mode resolves this failure in the production JS scene | E3 | Supported for isolated reproduction | [Probe results](hold-probe-results.json) |
| Holding before return fixes the control-flow regression | E4 | Supported | [Regression script](return-regression.mjs), [report](report.md) |
| Full manual autocal completes after the fix | None | Not tested | Retry pending |

## References

- [Full report](report.md): methods, measurements, changes and limitations.
- [Research notebook](research.md): scope and execution bounds.
- [Trial-001 manifest](../../extended-autocal/trial-001/hangprinter-20261009T073546.933824Z-9d7ded0f.json): initial every-step run.
- [Trial-002 manifest](../../extended-autocal/trial-002/hangprinter-20261009T090037.975347Z-a58c273c.json): stopped sampled run.
- [Trial-002 log](../../extended-autocal/trial-002/sweeps.full_auto.log): failed collection.
- [Source revision](revision.txt): base revision inspected before the fix.

## Measurements

E1 reads both adjacent manifests and uses actual file sizes. Playback is the
last active browser segment's simulated duration divided by source wall duration.
File size includes fixed provenance and every browser segment, so bytes per
simulation second is a whole-file normalization, not exact isolated physics cost.
The two trials differ in completion stage and event volume.

| Quantity | Trial-001 | Trial-002 |
| --- | ---: | ---: |
| File bytes | 1,203,903,321 | 366,290,666 |
| Active simulation seconds | 534.154 | 1816.840 |
| Active wall seconds | 936.780 | 303.806 |
| Achieved speed | 0.570x | 5.980x |
| MB per simulation second (decimal) | 2.254 | 0.202 |
| MB per wall second (decimal) | 1.285 | 1.206 |

RrdReader selected clock components confirm 90,843 observations in generation 8,
with consecutive step indices 1, 11, ..., 908421. The initial paused generation
contributes one more observation. The manifest reports stride 10, compact
geometry, no rejected messages, and autocal_complete=false. The requested 25x
was not achieved. Collector G-code round trips have median 4 ms and p95 6 ms.
Recorder receipt latency is separately recorded in recorder-latency.json;
browser events can queue behind physics even though G-code does not wait for them.

## Failure and fix

E2 finds the last force change at wall_time_ms=1791536592143:
M569.4 P40.0:41.0:42.0:43.0
T0.011363575731199994:0.6236330361282555:0.011363575731199994:0.0.
No position-mode or return G1 followed it before failure at 1791536602253.
The final encoder report was [102.17, -189.24, 102.17, -0.05] degrees;
preceding samples still moved. Force trial 4 reached its bounded observation
window without stalling, then returnMotorsToOriginOneAtATime entered its initial
settling wait while the force remained active. The 30-second limit detected real
movement. This is distinct from trial-001's history-pruning defect.

E3 builds a fresh isolated production JS world from scene.usda extracted from
trial-002's scene_context. It enables the recorded layering settings and applies
recorded torque values [-0.000340907, -0.018709, -0.000340907] Nm to A/B/C, with D
in position mode. After 30 simulation seconds, the unchanged pull times out during
30.5 more simulation seconds. Terminal angles are [102.1692, -189.2818, 102.1693,
0.0610] degrees. Setting all motors to position mode settles after two simulation
seconds. This is a fresh reproduction, not restoration of the original solver
state; the D-angle difference and missing preceding trials remain limitations.
The probe advances at most 95 simulation seconds and never connects to the user's
browser or firmware.

E4 uses a moving-encoder model to exercise the actual runForceTrial control flow.
The saved baseline modules reproduce the same 30000 ms timeout. The patched
routine returns moved=true, stalled=false, travelDeg=60 and final angles [0,0,0].
This synthetic result verifies command ordering, not physical calibration accuracy.

Both return routines now enter position mode before reading stable return
coordinates. One-at-a-time return restores that hold even if its move distance is
zero. The force-trial end measurements remain captured before this preparation;
return still precedes release to idle. Neither timeout limits nor stability
thresholds changed. The aggregate collector and bridge tests pass: 17 suites,
91 tests. git diff --check passes.

## Reproduction

From the repository root:

```bash
node output/research/2026-10-09-trial002-return-settling/hold-probe.mjs
node output/research/2026-10-09-trial002-return-settling/return-regression.mjs output/research/2026-10-09-trial002-return-settling/before-force-tuning.mjs
node output/research/2026-10-09-trial002-return-settling/return-regression.mjs autocal/control/behaviors/force_tuning.mjs
```

The baseline regression is expected to exit 1. These scripts use production
modules from this checkout; preserve revision.txt and the working diff when
repeating them. Trial recordings remain untouched. A fresh manual visual trial
is required to establish full collection success; no numerical optimizer run was
performed during this diagnosis.
