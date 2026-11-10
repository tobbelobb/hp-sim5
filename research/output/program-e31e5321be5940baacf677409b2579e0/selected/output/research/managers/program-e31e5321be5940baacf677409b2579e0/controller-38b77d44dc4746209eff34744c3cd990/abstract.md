# Saved consensus evidence rejects blanket nonworsening adoption

Status: complete; separately reviewed and admitted.

## Abstract

This iteration asks whether saved CM-E1 evidence justifies unconditional finite-prediction-only final-consensus adoption. One bounded local worker inspected four archived sources; a fresh local reviewer checked the frozen claim, report and nine selected assets. The archived additional cohort contains one improvement, two regressions and five unchanged outcomes. Its summed per-anchor error falls from 401.100276 to 400.797031 mm, but individual datasets worsen by 0.502998 and 0.094156 mm. All three changed selections lack a common unseen validation sweep. The reviewer returned supported/keep, and the controller admitted the scoped finding. Retire the blanket nonworsening proposal in this archived scope and preserve current defaults. These are inspected producer simulation measurements, with 91 passing structural consistency checks and no new numerical experiments. The review does not establish numerical correctness, hardware accuracy, stopping behavior or current-code equivalence. The next useful observation is provenance of compatible later sweeps unseen by both applied models.

## Topics

Hangprinter, CDPR, autocalibration, consensus membership, saved evidence, per-dataset regression, common unseen sweeps, separate review

## Tested hypotheses

| Hypothesis | Investigation | Outcome | Evidence |
| --- | --- | --- | --- |
| Saved CM-E1 records warrant withholding unconditional adoption while common-unseen predictive superiority remains unresolved | CM-SAVED-A1; historical CM-E1 | Supported by separate review | [Report](report.md), [review](review.json) |
| Blanket finite-only final consensus never worsens an additional dataset | Historical CM-E1 inspected by CM-SAVED-A1 | Rejected within archived scope | [Frozen producer results](/home/torbjorn/repos/hp-sim5/research/programs/program-e31e5321be5940baacf677409b2579e0/submissions/cm-fixed-submission/source-results-json) |
| Changed selections establish predictive superiority on common unseen data | Historical CM-E1 inspected by CM-SAVED-A1 | Inconclusive | [Frozen training coverage](/home/torbjorn/repos/hp-sim5/research/programs/program-e31e5321be5940baacf677409b2579e0/submissions/cm-fixed-submission/source-audit-json) |
| A narrower rule improves fresh fits or hardware | None | Not tested | Outside this iteration |

## Decision handoff

- Scope: eleven archived simulated final-selection histories, eight names beyond the motivating screen; summed per-anchor Euclidean distance in mm.
- Review: fresh local Codex context `review-run-beb299c90c6441278e176474b7f72fee`; supported/keep/report_only. Evidence inspected, not numerically reproduced.
- Decision: retire only the unconditional nonworsening proposal; preserve current defaults. No production change.
- Next: separately scope a saved-source compatibility search for later whole sweeps unseen by every member and chosen spool fit in both arms, prioritizing the two regressions. No further work is funded.

## References

- [Manager report](report.md), [notebook](research.md) and [review receipt](review.json): scope, decisions, cleanup and reviewer limitations.
- [Full worker report](/home/torbjorn/repos/hp-sim5/output/research/sessions/attempt-8cc3e522df934dafbc74a9639e34721e/report.md) and [evidence extract](/home/torbjorn/repos/hp-sim5/output/research/sessions/attempt-8cc3e522df934dafbc74a9639e34721e/evidence.json): individual saved outcomes, sweep IDs, source hashes and structural checks.
- [Frozen submission manifest](/home/torbjorn/repos/hp-sim5/research/programs/program-e31e5321be5940baacf677409b2579e0/submissions/cm-fixed-submission/manifest.json): exact reviewed bytes and source/base identities.
