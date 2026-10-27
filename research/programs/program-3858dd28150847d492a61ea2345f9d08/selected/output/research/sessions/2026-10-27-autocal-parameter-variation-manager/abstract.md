# HP3 autocal robustness across parameter variation

Status: bounded investigation complete; program paused.

## Abstract

This program investigated encoder-only HP3 calibration across varied parameter assumptions. A bounded analytical benchmark evaluated eight frozen worlds with 32 core fits in 118.24 seconds. Only six fits passed complete status and coverage checks. In one complete-coverage case, lower unseen residual accompanied anchor RMSE worsening from 5.89 to 40.20 mm after thinning. Two native stiffness worlds then each collected 60 bootstrap points, taking 75.02 measured world seconds together. Their poor conventional scene-home estimates and identical raw/geometry-metadata-stripped fits did not justify solver, point-count or stopping changes. Separate reviewers verified both scoped findings and marked both tools ready. A cloud review path bug was fixed; unsourced Node 24 crashes were corrected with pinned Node 23 and a pre-movement guard. Platform model limits interrupted one worker narrative, but saved measurements and deadline cleanup survived. No reliable fewer-point calibration gain or direct reference-003 win was established.

## Topics

HP3, encoder-only autocalibration, parameter variation, candidate ranking, point thinning, native collection, stiffness, cloud environment

## Tested hypotheses

| Hypothesis | IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Lower unseen residual always means better geometry | PV-E02 | Rejected by complete-coverage counterexample | [Review](review-first.json) |
| Broad solver or thinning replacements pass the declared screen | PV-E02 | Rejected for the stated gates | [First report](../attempt-3b2e360ec1834eee9bab4e06f6f880fd/report.md) |
| Native stiffness affects encoder response | PV-E03 | Supported for short probes | [Review](review-first.json) |
| Two bounded native stiffness collections complete under pinned Node | NP-E02 | Supported | [Native review](review-native.json) |
| Removing stored geometry changes the native fits | NP-E03/NP-E04 | Rejected for these four pairs | [Native report](native-report.md) |
| These studies establish robust fewer-point automatic calibration | None | Not tested | [Limits](report.md) |

## Decision handoff

Keep both reviewed tools and production defaults. No numerical work remains funded. Next, test automatic force tuning and additional independent sweeps in a new allocation. Both reviews verified scoped evidence; neither establishes hardware performance or full-loop native acceptance.

## References

- [Full report](report.md), [notebook](research.md), [first review](review-first.json), [native review](review-native.json).
- [Cloud faults and remedies](cloud-environment-issues.json), [literature](literature.md).
