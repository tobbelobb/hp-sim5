# HP3 trial-003 workspace measurement exceeds the settling deadline

Status: complete diagnosis and focused fix; full manual retry pending.

## Abstract

Trial-003 completed force tuning but failed during workspace-size measurement, before the first sweep. Its final commands pull two cables at 9.6637 N while releasing the third. The third encoder travels about 1,157 degrees during the 30-second wait. The timeout path then disconnects without holding the motors. Replaying recorded commands and polling times in an isolated production JavaScript physics world finds encoder stability after 37.7 simulation seconds. Active workspace measurement and sweep pullout now have a 120-second selected-clock budget with the existing independent wall deadline and stability tolerances. Size measurement holds motors on exit, and the collector attempts a hold on any collection error. Synthetic regressions cover achieved speeds of 0.57x and 25x, continued drift, a stopped clock and encoder disconnection; all 79 collector unit tests pass. These focused checks do not establish full browser collection or hardware success.

## Topics

HP3, autocalibration, workspace-size measurement, encoder stability, force travel, timeout, observed simulation clock, cleanup, Rerun replay.

## Tested hypotheses

| Hypothesis | Experiment | Outcome | Evidence |
| --- | --- | --- | --- |
| Size measurement rather than force tuning causes the timeout | E1 | Supported | [Recorded events](failure-events.json) |
| Motion needs more than 30 simulation seconds under recorded polling | E2 | Supported for isolated replay | [Replay](replay.mjs), [results](replay-results.json) |
| Patched measurement finishes and sends position hold | E3 | Supported for isolated replay | [Probe](measurement-probe.mjs), [results](measurement-probe-results.json) |
| Deadlines remain finite and cleanup runs on failure | E4 | Supported with synthetic clocks | [Tests](../../../autocal/control/tests/behaviors/measure_max_travel.test.mjs) |
| Full manual collection completes | None | Not tested | Manual retry pending |

## References

- [Full report](report.md): diagnosis, measured results and limitations.
- [Research notebook](research.md): experiments and execution bounds.
- [Trial-003 manifest](../../extended-autocal/trial-003/hangprinter-20261009T095245.092268Z-25141452.json) and [recording](../../extended-autocal/trial-003/hangprinter-20261009T095245.092268Z-25141452.rrd): recorded clocks and events.
- [Trial-003 full auto log](../../extended-autocal/trial-003/sweeps.full_auto.log): successful tuning followed by timeout.
- [Previous return fix](../2026-10-09-trial002-return-settling/report.md): preceding timeout had a different cause.
- [Source revision](revision.txt), [changes](working.diff), [saved scene](scene.usda), [command arrivals](commands.json), [encoder poll times](poll-times.json): replay provenance.
