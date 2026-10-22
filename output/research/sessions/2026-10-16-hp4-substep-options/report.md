# HP4 substep options and bounded size-tuning travel

Substeps are now selectable in the browser and through autocal's --physics-substeps setting for browser/headless collection, defaulting to one. A fixed-torque no-layering HP4 experiment shows the unchanged settling check passing after about 472 simulated seconds at one step versus 120 at four steps. Running the production size-tuning routine against native physics with its real ramp and 500 ms polling passed at one step after 477.75 simulated seconds. The size-tuning deadline is increased from 120 to 600 seconds, with a finite independent wall deadline and unchanged settling criteria. This validates the size-tuning routine, not an entire full-autocal run or hardware accuracy. Timestep-dependent numerical damping remains an accuracy concern.

## Implementation

The browser has a Physics Substeps selector next to Line Layering and Closed Loop Motors. Valid counts are integers 1–16; default one is retained. Explicit counts are forwarded from autocal.py to the collector and simulation_settings, checked in acknowledgements, retained over headless resets, accepted by createHeadlessWorld, and included in browser/headless recording context. Omitting the CLI flag preserves a browser selection. No speedup or motor physics code changed.

Only workspace size tuning receives the longer deadline: 600000 simulation milliseconds, with a separate 630000 ms wall deadline. Hardware uses wall time for its travel clock. Other force-travel limits and all quiet-window/drift/vibration criteria are unchanged. Continued movement still fails after the finite limit.

## Tested hypotheses

| Hypothesis | Experiment | Outcome | Evidence |
| --- | --- | --- | --- |
| Size tuning can still be moving legitimately at 120 s | E01 fixed-torque one-step pull | Supported | [600 s trace](pull-1-600.json), [long trace](pull-1.json) |
| The same settling test can pass after a longer bounded wait | E02 trace readback | Supported | [Results](settling-results.json), [script](check-settling.mjs) |
| Four steps change the approach time | E01/E02 controlled fresh worlds | Supported | [Four-step trace](pull-4.json) |
| Real production size-tune routine succeeds at one step with longer deadline | E03 native physical adapter | Supported | [Result](size-tune-result.json), [script](size-tune.mjs) |
| Browser/headless options agree and survive reset | E04 unit/integration and visible UI checks | Supported | Tests listed below |
| Entire original full-autocal completes | None this turn | Not tested | No full-run success claim |

## Physical evidence and interpretation

Fresh HP4 stock scene, no layering, closed-loop position motors, D held, A torque -0.000334089 Nm and B/C -0.037733 Nm, applied after 2 s. One-step y speed fell from about 1.58 mm/s at 120 s to 0.10 mm/s at 600 s. At 1200 s the position and motor encoders were stationary to numerical precision in the sampled trace. A small reconstructed angular velocity (~2.6e-5 rad/s) persisted, so this is not a claim that every numerical variable becomes exactly zero.

E02 replays one-second sampled encoder observations through the actual waitForStableEncoders function, using its existing tolerances and a one-second poll interval. One-step travel fails the old 120 s limit and passes after 472 s; four steps passes at 120 s. This is a controlled comparison, not an exact replay of the user's force-tuning history.

E03 advances real production native physics through the actual measureMaxTravelMm ramp and default 500 ms polling. A narrow in-process command adapter applies the recorded torque conversion (-0.015 Nm/N) and reads the real spool encoders; RRF and browser transport are excluded. It returns 535.802 mm maximum travel after 477.75 simulated seconds. This checks the previously failing behavior with actual physics, rather than synthesizing a successful encoder response.

The implicit torque-mode cable load update divides by 1 + dt*D/I + dt²*K/I. Its timestep-dependent damping is a plausible contributor to the slower coarse-step approach; no isolated attribution experiment was performed. Four smaller steps change that discretization. The final sampled positions differ slightly between step counts, and full trajectory/accuracy convergence has not been established. A longer waiting budget addresses this finite travel without resolving that numerical accuracy issue.

The native experiments ran without physics recording; raw samples are retained. Timing values in their files are not a matched performance benchmark (some runs overlapped tests).

## Validation

- JS browser/scene tests plus collector acknowledgement and size-tuning tests: 169 passed.
- Headless settings/reset tests: 2 passed.
- Additional headless CLI/recorder checks: 25 passed; an initial socket-restricted invocation was rerun with local sockets enabled.
- Visible browser check: default 1, selection 4, restore 1. Browser remains stopped.
- Python compile and git diff --check passed.

Tests cover invalid counts, remote/UI agreement, acknowledgement mismatch, reset persistence, late finite settling, indefinitely moving encoders, wall-clock failure, and holding motors after errors. No hardware was used and no full-autocal accuracy result is claimed.

## References

- [Previous one-step failure](../2026-10-16-hp4-one-step-autocal/report.md): actual original browser command failed while still moving.
- [Earlier rocking investigation](../2026-10-16-hp4-effector-instability/report.md): separate loaded-state corner-rocking case.
- [Working diff](working.diff): current implementation.


## Follow-up: slow half-sweep pullout in reference-006

User requested only a timeout increase to retry. The actual reference-006/sweeps.full_auto_1.log shows successful size tuning (590.418 mm raw), then first half-sweep pullout hitting its 120 s limit, holding the drive, and timing out after the subsequent 30 s preload-settling limit. Increased both pullout travel and its immediate preload-settling budget to 600 simulated seconds, each with a 630 s wall deadline. Force selection, substep defaults and stability criteria are unchanged. No new physical trial or full-autocal retry was run. The two targeted settling/size-tune suites pass: 13 tests. This supersedes the earlier report statement that other force-travel limits were unchanged.
