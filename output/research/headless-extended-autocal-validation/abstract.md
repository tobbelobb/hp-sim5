# Headless extended autocal recording validation

Status: complete; full recording verified, pre-existing parity discrepancy retained.

## Abstract

The production headless JavaScript world completed default full-auto calibration
with one finalized extended-reference RRD: eight sweeps, 160 points, 267,711
physics observations and 616,025 events in 1525.41 wall seconds. Archive checks
verified exact final datasets/logs, all measurements, paired command outcomes,
stage artifacts and explicit source clocks. Live reset, 25×, external-recorder,
disconnection and interruption checks passed. Failed trials exposed a drain
wait and a backlog-sensitive disconnect; their partial evidence is retained.
The separate Chromium/Node comparison reproduced a one-point 0.16-degree
discrepancy, including with the original headless service. Tolerances were not
relaxed. These are simulated recording-tool checks, not evidence of calibration
accuracy or hardware behavior.

## Topics

autocalibration, headless simulation, HP3, extended reference, Rerun, RRD,
acknowledgements, source clocks, interruption, process ownership.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Ten seconds total is sufficient to drain all live events during stepping | 001 | Rejected | [Failed trial](../../headless-extended-verification/) |
| Interrupted bootstrap preserves partial evidence and finalizes recording | 003 | Supported | [Interrupted manifest](../../headless-extended-interruption-progress/test_interrupted_bootstrap_pre0/sweeps.headless/manifest.json) |
| Recorded force tuning can complete under high-rate control traffic | 002, 004 | Supported after transport change | [Failed trial](../../headless-extended-verification-progress/), [rerun](../../headless-extended-verification-no-ping/) |
| Default full-auto completes and preserves all sources in one RRD | 004 | Supported | [Archive verification](../../headless-extended-verification-no-ping/test_fresh_hp3_full_auto_globa0/hp3/rrd-verification.json) |
| Matched Chromium/Node collection meets the existing parity tolerance | 004, 005, 006 | Rejected for these inputs, also with original service | [Repeated discrepancy](../../headless-extended-parity-repeat/parity.json), [original-service baseline](../../headless-extended-parity-baseline/parity.json) |

## References

- [Notebook](research.md): decisions and failed trials.
- [Report](report.md): current verification status.
- [Extended reference guide](../../../autocal/data/references/extended-reference.md): protocol and clocks.
- [Earlier headless verification](../../../research/headless-autocal-verification.md): default calibration and matched-input parity contract.
