# HP3 force excitation improves saved fits; preserve tuning before collection

Status: complete.

## Abstract

This bounded native HP3 study compared acquisition choices at 60 measured points in 20000 N and 8000 N stiffness scenes. Six independent ten-point sweeps did not pass the declared coverage criterion against saved three-sweep baselines. Two ordinary-fit comparisons worsened; the other improvements were 3.69% and 15.13%. Production automatic force tuning found movement thresholds near 0.23 to 0.25 N, but both collections exhausted their one-million-step caps. Fresh worlds replaying those learned forces completed 60 points each. Existing recovery produced a 3.29 mm nominal conventional anchor RMSE, and ordinary soft-scene fits gave 10.43 to 11.64 mm, compared with metre-scale baseline errors. Frozen models covered all 20 and 40 independently recorded holdout points. These are same-scene simulation findings with an unsettled-reference limitation, not full automatic acceptance or hardware accuracy. The production change preserves force selections and diagnostics before sweeps, including interrupted runs. It does not automatically reuse saved settings.

## Topics

HP3, autocalibration, force excitation, independent sweeps, interrupted collection, native Rust, encoder data, recovery.

## Tested hypotheses

| Hypothesis | Experiment | Outcome | Evidence |
| --- | --- | --- | --- |
| Additional fixed-target levels improve geometry at equal 60 points | CVF-E01 | Rejected in this tested scope | [Measured summary](measured-summary.json) |
| Encoder-derived forces improve the acquisition pipeline | CVF-E02, CVF-E03 | Supported for these fresh-world replays, original trials incomplete | [Force summary](force-partial-summary.json), [measurements](measured-summary.json) |
| Existing recovery repairs the nominal sentinel fit | CVF-E03-R | Supported | [Recovery results](recovery-results.json) |
| Replayed-force models predict saved independent native sweeps | CVF-E04 | Supported within these two scenes | [Holdout results](holdout-results.json) |
| Early force selection survives later collection failure | CVF-C01 | Supported by lifecycle regression tests | [Checks](js-regression.log) |

## Decision handoff

Keep the force-selection checkpoint as a production robustness change. Optional per-config fixed-target syntax is a separate experiment-enabling commit. Keep acquisition and solver defaults. The next useful research question is reducing force-search work while preserving force decisions and measured data quality.

Review: worker measured and recomputed these results; independent parent review is pending. Scope excludes a full automatic stopping comparison and hardware measurements.

## References

- [Full report](report.md) explains comparisons and limits.
- [Research notebook](research.md) records budget changes and decisions.
- [Saved native baseline report](../2026-10-27-autocal-parameter-variation-manager/native-report.md) supplies the fixed baseline.
