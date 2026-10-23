# Finite-prediction consensus: broader replay rejects a blanket restriction

Status: complete.

## Abstract

Should final calibration consensus exclude fits without a finite held-out
prediction? An earlier three-history screen improved two cases. This frozen
comparison used eleven older simulated histories, including eight dataset names
beyond that screen. Among the eight additional datasets, one improved, two
worsened and five were unchanged; summed per-anchor error changed from 401.100
to 400.797 mm. Excluding the newest fit increased error by 0.503 mm on flexible
lines and 0.094 mm on one buildup dataset. All eleven chosen single-fit
prediction scores matched saved values under current code, and every applied
model diagnostic was finite. However, the three changed comparisons had no
common sweep unseen by both applied models, so predictive improvement remains
unestablished. The probe used 22 selections and 49 frozen prediction evaluations,
with no fits, physics or collection. Keep production defaults and retire the
blanket restriction as a default-change proposal. These are saved simulation
results, not hardware accuracy or changed stopping trajectories.

## Topics

autocalibration, encoder-only calibration, consensus membership, newest fit,
held-out sweeps, applied spool model, Slideprinter, HP3, negative results

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Finite-prediction-only final consensus improves accuracy without worsening an additional dataset | CM-E1 | Rejected: one improvement, two regressions, five unchanged | [Measurements](results.json), [comparison](report.md#cm-e1-broader-accuracy-comparison) |
| Finite member predictions establish predictive safety of the applied average and chosen spool model | CM-E1 | Inconclusive: finite diagnostics, no common unseen fold for changed selections | [Fold coverage](results.json), [guardrails](report.md#prediction-guardrails-and-source-compatibility) |

## Decision handoff

- Scope: eleven archived simulated datasets at producer revision 72523b9d; final selection with current 0.01 mm merge/5 mm spread guards and frozen ranking. Eight dataset names extend the motivating screen; three overlap.
- Review: this agent recomputed selection and applied-model predictions once, checked saved-score compatibility and audited hashes/dataset uniqueness. No independent reviewer, fresh fitting or hardware validation.
- Decision: H1 rejects the blanket default change. Preserve current all-candidate consensus; no fixture-specific or tuned replacement is justified.
- Next: task 006 remains next in queue order, followed by task 005's agenda. Reopening membership needs common unseen-sweep evidence on the regression cases; search compatible saved later sweeps before new fits or collection.

## References

- [Full report](report.md): policy comparison, exceptions and scope.
- [Research notebook](research.md): predeclared budget and decisions.
- [Input manifest](inputs.json), [compressed original envelopes](inputs/) and [audit](audit.json): source identities, truth used only for evaluation and distinct observation datasets.
- [Initial screen](../2026-10-23-program-task-sanity/report.md): motivating result reused without rerunning.
- [October 15 spread-gate study](../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/report.md): origin and prior scope of the eleven-history corpus.
