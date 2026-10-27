# Native HP3 stiffness pair and bootstrap fitting

Status: manager closure; separate review supported/keep, code ready.

## Abstract

Two production native-Rust HP3 bootstrap collections, with 20000 and 8000 N cable stiffness, each completed three sweeps and 60 points. Together they took 75.02 measured world seconds and 602454 physics steps. Four ordinary fit-stage calls and four geometry-metadata-stripped checks survived a worker usage-limit interruption. All stripped checks produced numerically identical anchors, metrics and optimizer histories. Conventional scene-home anchor RMSE ranges from 2003 to 3648 mm, despite finite score and covariance checks. These are bootstrap fit stages, not complete automatic calibration. Encoder-zero pose and rotation were unavailable, so the scene-home metric has a reference limitation. A retained collection helper now checks pinned Node before movement and preserves partial failures. Two earlier zero-step crashes resulted from an unsourced Node24 shell and were corrected with the documented Node23 environment. No new physics or fitting was performed during manager closure.

## Topics

HP3, native collection, stiffness variation, bootstrap identifiability, encoder-only calibration, Node runtime, model interruption

## Tested hypotheses

| Hypothesis | IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Two bounded native stiffness collections complete under pinned Node | NP-E02 | Supported in producer artifacts, review pending | [Summary](native-saved-summary.json) |
| Removing stored geometry changes these fits | NP-E03/NP-E04 | Rejected in saved results, review pending | [Report](native-report.md) |
| These bootstrap fits establish reliable automatic calibration | None | Not tested | [Limits](native-report.md) |

## Decision handoff

Keep defaults and preserve all evidence. No new numerical work is funded.

## References

- [Report](native-report.md), [summary](native-saved-summary.json), [analysis](analyze_native_saved.py).


Separate review completed as review-5f62ee3093f94f7a9af98157ecb24066, applied by admission-7d84479eb45c4688a07fd8c62f9f3cf0. The fixed reviewed bytes remain unchanged in the submission store.
