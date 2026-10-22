# Production JS headless full-auto autocalibration

Status: complete.

## Abstract

The integration runs the existing full-auto autocal loop against production
JavaScript physics in Node. The final fresh HP3 global-radius trial completed
after nine iterations and eleven sweeps/220 points, applied M669/M666, and
returned a 39.02 mm radius. It used 6487.832 simulated seconds and 435.65 wall
seconds. The solver rated the fit ideal (1.039), but its D height differs from
the nominal preset, so completion and fit score do not establish parameter
accuracy. Independent matched Chromium/Node collection
measured sixty points per backend. Maximum differences were 0.02 degrees raw
angles and 0.005236 mm lengths, within documented 0.03 degree/0.01 mm tolerances.
Absolute simulated timestamps differed by one second after an independent
settling boundary; sampling durations agreed within one fixed step. A SIGTERM
test retained completed points and stopped owned services. Failed initialization
and overly strict comparison trials are retained. Final review corrected a
remaining force-ramp wall sleep; both extended checks passed after that
correction. These are simulated tests, not hardware calibration evidence.

## Topics

autocalibration, HP3, headless Node, Chromium, measurement parity, global radius,
simulation clock, lifecycle

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Raw unbaked scenes can initialize collection | HP3-FULL-01 | Rejected | [Failed manifest](../../headless-verification/hp3/sweeps.headless_1/manifest.json) |
| Chromium and Node produce matched collection | HP3-FINAL-CLOCK-01 | Supported within stated tolerances | [Matched measurements](../../headless-verification/final-clock/test_fresh_hp3_full_auto_globa0/browser/parity.json) |
| Fresh HP3 full-auto completes automatically | HP3-FINAL-CLOCK-01 | Supported; accuracy not established | [Complete manifest](../../headless-verification/final-clock/test_fresh_hp3_full_auto_globa0/hp3/sweeps.headless/manifest.json) |
| Bootstrap interruption retains evidence and cleans services | HP3-FINAL-CLOCK-01 | Supported | [Interrupted manifest](../../headless-verification/final-clock/test_interrupted_bootstrap_pre0/sweeps.headless/manifest.json) |

## References

- [Notebook](research.md): experiment decisions and constraints.
- [Report](report.md): results and limitations.
