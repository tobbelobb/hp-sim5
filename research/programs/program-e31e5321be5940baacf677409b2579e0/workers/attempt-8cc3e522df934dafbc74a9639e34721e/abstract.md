# CM-E1 saved-source audit: aggregate improvement fails per-dataset nonworsening

Status: complete; inspection finished, fresh review pending.

## Abstract

Do four archived CM-E1 sources support unconditional finite-prediction-only final-consensus nonworsening? This audit inspected their bytes and JSON, copied 219,323 bytes of immutable source evidence, and checked hashes, cross-file identities, stored summary arithmetic and saved training/fold sets. Among eight dataset names beyond the motivating screen, the inspected producer evidence records one improvement, two regressions and five unchanged outcomes. Summed anchor error decreases from 401.100276 to 400.797031 mm, while flexible_lines_2000 and ten_points_w_buildup worsen by 0.502998 and 0.094156 mm. Thus the aggregate gain does not satisfy per-dataset nonworsening. For all three changed selections, the baseline trained on every available sweep; each variant leaves one terminal sweep unseen, but none is unseen by both models. Predictive superiority remains unresolved. All 91 structural consistency checks pass. These are inspected historical simulation measurements, with no independent numerical replay, fresh fits, physics, collection or hardware validation. Source identities and internal agreement do not validate the producer's numerical implementation or current-code equivalence.

## Topics

CM-E1, consensus membership, autocalibration, finite prediction, final selection, per-dataset nonworsening, aggregate error, common unseen sweeps, training overlap, simulated fixtures, provenance audit, scoped-holdout.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Finite-only final consensus never worsens each additional archived dataset | CM-SAVED-A1; historical CM-E1 | Rejected within archived scope | [Rows 6 and 9](evidence.json) |
| The additional cohort's aggregate error decreases | CM-SAVED-A1; historical CM-E1 | Supported as inspected producer evidence | [Saved summary](sources/results.json) |
| Changed cases establish common-unseen predictive superiority | CM-SAVED-A1; historical CM-E1 | Inconclusive | [Empty folds and saved training coverage](evidence.json) |
| Archived regressions warrant withholding unconditional adoption | CM-SAVED-A1 | Supported within archived scope | [Report](report.md) |
| A narrower rule improves fresh fits, stopping or hardware | None | Not tested | Outside allocation |

## Decision handoff

- Scope: CM-E1's eleven archived simulated histories, including eight additional dataset names; unconditional finite-prediction-only final consensus; sum of per-anchor distances in mm.
- Review: this worker inspected four producer sources and 91 structural checks. Numerical correctness was not independently replayed; a fresh reviewer is pending.
- Decision: retire only the unconditional nonworsening default-change proposal in this scope. Do not infer broader solver/model inadequacy or change production defaults.
- Next: separately inspect whether compatible saved later whole sweeps exist outside training of all members and the chosen spool fit in both arms for rows 6, 9 and 10. No search or evaluation was performed here.

## References

- [Report](report.md) and [notebook](research.md): reasoning, constraints and handoff.
- [CM-E1 report, preceding session 2026-10-23-consensus-membership](sources/report.md): archived method and interpretation; outgoing sources not followed.
- [CM-E1 results](sources/results.json): exact saved outcomes and fold IDs.
- [CM-E1 audit](sources/audit.json): saved training coverage and observation identities.
- [CM-E1 input manifest](sources/inputs.json): cohorts, simulation truth and producer provenance.
- [Evidence extract](evidence.json) and [extractor](extract.py): exact source hashes, JSON pointers and consistency checks.
