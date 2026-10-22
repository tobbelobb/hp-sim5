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
