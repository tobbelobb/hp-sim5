# HP3 calibration research: preserve force selection and cut weak routes

Status: bounded cycle complete; program paused.

## Abstract

The latest cycle tested a 60-point collection redesign, cut it after it failed the paired improvement gate, and investigated force adequacy instead. Existing encoder-based force selection and recovery produced a nominal fit with 3.29 mm conventional scene-reference error and soft-scene fits around 10 to 12 mm. These are limited simulation results, not demonstrated full-auto or hardware accuracy. Two force searches were interrupted after completing their expensive tuning stage, exposing a production data-retention gap. The retained fix atomically saves selected forces and diagnostics before sweep collection, distinguishes tuning, fallback and manual selection, and never automatically reuses a stale cache. It preserves later force changes in the final dataset. Independent review kept the fix and separate optional per-sweep target support. Sixty-nine JavaScript and seventeen Python checks passed. Earlier Node-runtime and cloud-review fixes remain published. The next unfunded question is reducing force-search cost without degrading collected data.

## Topics

HP3, encoder-only calibration, force tuning, failure recovery, force-selection checkpoint, fixed-target sweep coverage, native simulation, cloud runtime

## Tested hypotheses

| Hypothesis | IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Tested equal-point coverage allocation improves all native comparisons | CVF-E01 | Rejected for stated gate | [Latest report](../attempt-902d9713dcbe436d8ddc1ba23fd82baa/report.md) |
| Existing force selection and recovery produce useful same-scene data | CVF-E02/E03/E04 | Supported with reference/history limits | [Review](review-force-checkpoint.json) |
| A new solver policy is required for the nominal sentinel | CVF-E03-R | Rejected in this case; existing recovery succeeds | [Recovery evidence](../attempt-902d9713dcbe436d8ddc1ba23fd82baa/recovery-results.json) |
| Early force selection survives later failure | CVF-C01 | Supported by lifecycle regressions | [Review](review-force-checkpoint.json) |
| Faster force bracketing preserves quality | None | Not tested | [Ledger](approaches.md) |

## Decision handoff

Publish the reviewed checkpoint and optional per-sweep target support. Keep calibration defaults. No numerical work remains funded. Next compare force-search cost while preserving learned decisions and measured data quality.

## References

- [Report](report.md), [ledger](approaches.md), [notebook](research.md), [exact final review](review-force-checkpoint.json).
