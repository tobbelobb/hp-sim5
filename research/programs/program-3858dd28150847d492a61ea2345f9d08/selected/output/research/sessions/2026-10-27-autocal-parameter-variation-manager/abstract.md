# HP3 autocal robustness across parameter variation

Status: first iteration reviewed; native collection followup planned.

## Abstract

This program tests encoder-only HP3 calibration across varied parameter assumptions while preserving fixed radii, zero buildup and one-click stopping. A new analytical benchmark evaluates eight frozen worlds with 32 core fits in 118.24 seconds. Only six fits pass complete status and coverage checks. A clean TRF example improves unseen encoder residual from 0.02362 to 0.02234 mm after thinning while anchor RMSE worsens from 5.89 to 40.20 mm. A fresh reviewer recomputed the saved measurements and supported the scoped findings and benchmark candidate. Two short native Rust probes establish stiffness sensitivity, not calibration accuracy. A cloud review path bug was reproduced and fixed, with 51 tests passing and a successful real reviewer dispatch. No production solver or point-count default changed. Matched native collection is the next discriminator. The reference-003 raw dataset remains unavailable.

## Topics

HP3, encoder-only autocalibration, parameter variation, analytical benchmark, solver robustness, point thinning, native stiffness sensitivity, cloud review repair

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Lower unseen residual always means better geometry | PV-E02 | Rejected by a complete-coverage counterexample | [Review](review-first.json), [plot](analytical-screen.png) |
| A broad solver or thinning replacement passes the declared screen | PV-E02 | Rejected for the predefined gates | [Worker report](../attempt-3b2e360ec1834eee9bab4e06f6f880fd/report.md) |
| Native stiffness affects encoder response | PV-E03 | Supported for two four-second probes | [Recomputed review](review-first.json) |
| Default-store cloud reviewers can dispatch | ENV-01 | Supported after repair ae0b20c4 | [Issue and fix](cloud-environment-issues.json) |
| Analytical failures predict ordinary native calibration behavior | None | Not tested | Next bounded assignment |

## Decision handoff

Keep benchmark cc52307f and retain production defaults. The exact scientific submission is submission-63290af5030747d2be1f7d75d7891e8c. Review-f674059afe874bdca72e7f4f5f8366ee recomputed saved data without new optimization or physics. Admission-d4a35af838c44f1f9b7c10f88df01741 records supported/keep and ready code. Native collection transfer remains unresolved.

## References

- [Report](report.md), [notebook](research.md), [literature notes](literature.md).
- [Worker report and limits](../attempt-3b2e360ec1834eee9bab4e06f6f880fd/report.md).
- [Cloud repair checks](cloud-review-path-after.log), [runtime checks](runtime-check.json).
