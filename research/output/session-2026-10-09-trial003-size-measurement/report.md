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

## Diagnosis

The collector finishes tuneForce and its return-to-origin, then completes tighten/release and position hold. It selects pair anchors B/C for size tuning. Three force ramp steps reach 25%, 50% and 75% of forceMax=12.88491133549 N. A is in low-force mode and D remains in position mode. The ordinary waitForStableEncoders uses 30,000 ms on the observed browser simulation clock. This wait includes the whole workspace traversal, not just the final quiet window. The final recorded reading is [1156.93, -296.36, -296.35, 0.21] degrees. Continuous movement and late small slips prevent stability before the deadline; increasing requested speed would not change that selected-clock requirement.

The old cleanup in measureMaxTravelMm was below the wait rather than in finally. The CLI catch only printed the error before closing the bridge. Thus the pair pull was left active in the recorded failed run.

## Replay method and limits

The isolated replay builds the production HP3 JS world from recorded USDA and enables the recorded layering settings. It queues all 220,379 recorded motor commands at their browser research-clock arrival times, using fixed 2-ms physics steps. It retains the recorded encoder-poll cadence until the captured collector exit and then uses 500-ms polls. It uses no browser, live firmware, WebSocket server or recording service. The original scalar torque rounding is preserved. This reproduces motion and the deadline issue, not bit-identical browser execution: command execution follows receipt on a subsequent physics update, floating point and batching can affect slip phases, and browser scheduling ceases to be observable after the collector exits.

With the captured poll cadence, the ordinary stability checks accept at 37.7 seconds, endpoint A approximately 1167.59 degrees. Under regular 500-ms polls a shorter replay accepted a transient quiet period at 28 seconds. This is why the captured poll schedule matters and why a narrow deadline increase is fragile. The active movement budget is 120 seconds, retaining the independent 120-second wall deadline and existing quiet/vibration tolerances. It neither assumes the requested 25x speed nor converts a timeout into a measurement.

## Validation

All collector behavior, primitive and CLI unit tests passed: 16 suites, 79 tests. New tests cover motion lasting 40 selected-clock seconds at achieved rates 0.57x and 25x, continuous drift past the new finite travel budget, no simulation-clock progress, and encoder transport failure. Both failure and success paths send position hold. Syntax checks and git diff --check passed.

The same bounded travel budget is applied to the first sweep's intentional force pullout, which otherwise had the same 30-second whole-travel limit. Other settling and point-measurement waits retain the normal default. No complete browser sweep dataset or numerical calibration has been produced by these checks.

The actual patched measureMaxTravelMm routine was also run on the replayed physics world (E3). It measured 611.35 mm raw travel in 38.16 simulation seconds including ramp time, then sent position hold to all four motors. See [probe results](measurement-probe-results.json). The replay remains isolated and is not a full firmware/browser collection run.
