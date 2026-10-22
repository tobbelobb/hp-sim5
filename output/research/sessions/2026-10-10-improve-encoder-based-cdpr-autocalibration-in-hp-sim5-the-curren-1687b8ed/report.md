# HP3 encoder autocalibration: reference failure and runtime bottlenecks

## Abstract

This investigation asks why zero-buildup HP3 encoder autocalibration is slow and can return poor anchors, and whether fewer points are safe. The supplied browser-js reference recorded 4,706,150 physics steps over 9,412.3 simulated seconds and ended after eight sweeps with a rank-zero, underconstrained fit; its recorder marks autocal incomplete. Encoder noise was reported as zero and fixed-target errors stayed small, while later anchor estimates became geometrically implausible. Recovery fitting took about 1.35 seconds per exhausted pass, making collection and settling the likely primary wait cost. Detailed event timing could not be completed because streaming the 2.0 GB RRD did not return within the bounded tool window. Earlier recorded trials found fixed point thinning worsened anchor RMSE on three of four fixtures; a guarded settling change saved about 25% in separate fixed HP4 collection. No source changes or hardware measurements were made. A coverage-aware, collection-time-aware planner and adaptive point count remain untested.

## Objective and constraints

Investigate speed, robustness and fewer-point collection for encoder-only HP3 autocalibration while preserving one-click collection and automatic stopping. Use zero buildup and disabled line layering. Prefer the supplied run at `output/extended-autocal/no-buildup-reference-002`; do not search spool radii. Source changes should remain small. No hardware measurements were made. The user clarified that the run’s “no valid sweep” stop bug is already fixed; it is not an active issue.

## Experiments and outcomes

| ID | Hypothesis | Outcome | Evidence |
|---|---|---|---|
| E01 | The supplied HP3 reference can complete and automatically accept a calibration. | Rejected for this historical run; the specific stop bug is fixed and is not a current finding. | [Run log](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log), [full-auto events](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto_log.jsonl) |
| E02 | Fitter/recovery wall time dominates the long user wait. | Rejected for this run; collection dominates simulated time. Detailed RRD event timing was not completed. | [Recorder manifest](../../../../extended-autocal/no-buildup-reference-002/hangprinter-20261010T151618.949443Z-c9ffe4cd.json), [run log](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log) |
| E03 | Simply halving points per sweep is a robust default improvement. | Rejected based on prior recorded fixtures (3/4 anchor RMSE worsened); no new trial here. | [Prior report](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md#disproved-ideas-and-remaining-experiments) |
| E04 | Selection scores can disagree with geometric anchor accuracy. | Supported by prior audits; this run independently exhibits unstable geometry despite low residual noise. | [Prior physical audit](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E01-E01-baseline-audit.json), [run log](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log) |
| E05 | Encoder quietness and zero measured noise ensure identifiable anchor geometry. | Rejected for the supplied simulated data. | [Dataset](../../../../extended-autocal/no-buildup-reference-002/sweeps.json), [full-auto log](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log) |

### Supplied run observations

The recorder manifest identifies browser-js HP3 data over 4,706,150 physics steps (9,412.3 simulation seconds), 470,616 physics samples and 612,620 events. The RRD is 2.0 GB. It reports `autocal_complete=false`. The dataset ended with eight sweeps (160 points): three initial pair geometries, three additional sweeps with the same `[2,3]` fixed pair and `[0,1]` drive/sensor pair, then two new selections. The historical full-auto log shows an early finite low-score solution (UI score 0.1952), followed by unstable anchor estimates, rank 0/12 and the 100-point underconstraint sentinel. Two recovery passes were exhausted before the old no-valid-sweep stop. The user reports that stop bug has since been fixed, so the outcome does not diagnose current stopping behavior.

Points report sigma zero, and fixed-target tracking errors are generally within 0.06 degrees. Yet later estimates place anchors thousands of millimetres away and become rank deficient. This supports the narrower conclusion that low encoder noise and quiet fixed motors do not by themselves guarantee useful geometric excitation. The run’s recovery entries report about 1.35 seconds per exhausted attempt; collection includes repeated 15-second stability windows and long force tuning. Collection is the likely primary user-wait target, but the exact wall-time share was not measured.

### RRD profiling limitation

I attempted the documented `rerun.chunk.RrdReader` query for `/autocal/python` event JSON. Materializing all event chunks from this 2.0 GB recording did not return within the bounded 10-second tool window. No per-event runtime attribution is claimed. Stage snapshots and JSONL summaries establish that individual recovery attempts were short; a bounded streaming RRD profile is still needed to quantify waits, physics stepping, recording and fitting precisely.

## Encoder-only improvement ideas

1. **Identifiability-aware collection and stopping.** Compute the measurement Jacobian/Fisher information after gauge freedoms are removed, in addition to residual score. Detect repeated sweep geometry and avoid treating a quiet, low-residual history as sufficient when parameter directions remain unconstrained. This must be paired with an alternate feasible sweep plan to avoid simply collecting longer.
2. **Collection-time-aware sweep planning.** Rank feasible anchor pairs and spans by expected information gain per measured collection second, enforcing pair and workspace-height coverage. This directly addresses the repeated `[2,3]` fixed pair in the supplied dataset. Not implemented or tested in this turn.
3. **Adaptive point count.** Start with fewer points and collect additional positions only while fit uncertainty or held-out encoder prediction remains high. Prior simple thinning degraded anchor RMSE on most tested fixtures; a fixed reduced count is rejected, while adaptive stopping remains untested.
4. **Guarded settling-window optimization.** Prior work measured about 25% less simulated collection time with a recent-window guard while retaining all points, but this was fixed HP4 collection, not a fresh HP3 full-auto trajectory. It is the clearest measured acquisition-speed opportunity so far.
5. **Score/geometry disagreement audits.** Prior ranking audits found physical-error disagreements. Held-out scoring, age-bonus removal and bounded updates did not generalize enough to adopt. Keep synthetic known-truth and identifiability measures in evaluation, not as a presumed production ranking fix.

## Recommendations

Do not reduce the default point count based on current evidence. The next experiment should compare the current sweep planner with a rank/coverage-aware, collection-time-aware planner on identical fresh HP3 worlds, then test adaptive point count against that planner. Require both automatic completion and noninferior known-truth anchor error across multiple independent simulated seeds. Stream RRD event timings in bounded time windows or batches before attributing wall-time shares. No source change is justified by this historical failed run alone.

## RRD size and whether quiet time can be removed

Yes, sparse time series are valid and intended in Rerun. A component need only be logged when it changes; latest-at queries carry its last value forward. In hp-sim5 this optimization is already used for unchanged poses, and the supplied recording samples physics every tenth step. Collector commands, measurements and Python events are meaningful event streams and should stay at their actual timestamps.

I ran `.venv/bin/rerun rrd stats` against the supplied 2.0 GB RRD. It contained 473,771 chunks and 24,840,564 rows. Rerun's object-store compaction estimate was 2,232 chunks, or 212.3 times fewer than the current count. It identified roughly 37,668 Python/collector log event chunks, clock chunks, and many geometry chunks as substantial groups. The stats scan read about 470k messages in roughly 16 seconds. The file had no unsorted timelines.

I then ran the supported `.venv/bin/rerun rrd optimize --profile object-store` and wrote a separate output to `/tmp/no-buildup-reference-002-optimized.rrd`. Rerun reported chunk count 473,771 → 1,438 (-99.696%) and size 1.9 GiB → 1.1 GiB (-41.338%) in 66.88 seconds. The RRD CLI reported compaction and output encoding completed; the original was not modified. A separate `verify` and a comparison command did not return a final status in the observation window, so exact semantic equivalence was not independently established in this investigation. Rerun's supported optimizer is therefore a proven size reduction and successful rewrite, but retain the original until desired content is checked.

This does not discard quiet time. To shrink a future diagnostic recording further, increase physics `sample_stride` and use compact geometry, while retaining full-rate autocal/collector events and measurements. To trim an old recording, make a distinct derived RRD: identify intervals from motor commands, encoder motion and collector activity; retain margin samples before/after transitions; preserve original absolute timestamps and all calibration events; and annotate gaps or provide a source-time mapping. This is a lossy selection of observations, not a special RRD-format feature. Compressing the timeline by closing gaps would alter timing semantics, so avoid that for evidence used to infer durations or align physics to commands.

## Experiment outcomes added

| ID | Hypothesis | Outcome | Evidence |
|---|---|---|---|
| E06 | Chunk overhead is a substantial part of the large RRD. | Supported: 473,771 chunks; object-store estimate 2,232. | [Rerun stats output](../../../../extended-autocal/no-buildup-reference-002/), source RRD and manifest |
| E07 | Supported RRD compaction substantially reduces file size without intentional sample removal. | Supported as a successful rewrite and size reduction; semantic equality was not separately verified. | `/tmp/no-buildup-reference-002-optimized.rrd`; Rerun optimize output captured in the research conversation |

## References

- [Supplied HP3 zero-buildup dataset](../../../../extended-autocal/no-buildup-reference-002/sweeps.json): encoder measurements and sweep identities for E01/E05.
- [Supplied full-auto log](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log) and [iteration JSONL](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto_log.jsonl): fits, stopping and recovery timings for E01/E02/E04.
- [Supplied RRD manifest](../../../../extended-autocal/no-buildup-reference-002/hangprinter-20261010T151618.949443Z-c9ffe4cd.json): simulated recording coverage and completion status for E02.
- [Prior October 8 report](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md): recorded anchor-score mismatch, point-thinning results and measured guarded settling improvement.
- [Autocal algorithm overview](../../../../../autocal/optimization_algorithm_overview) and [objective overview](../../../../../autocal/objective_functions_overview): current stages, ranking and identifiability diagnostics.
- [Extended reference guide](../../../../../autocal/data/references/extended-reference.md): RRD event and clock model used to interpret the supplied run.
- [Research integration guide](../../../../../research/README.md) and [integration rationale](../../../../../research/investigation.md): experiment and evidence conventions.
- [Rerun: Optimize chunk count](https://rerun.io/docs/howto/logging-and-ingestion/optimize-chunks): supported chunk compaction and stats workflow.
- [Rerun: Query semantics and partial updates](https://rerun.io/docs/concepts/logging-and-ingestion/latest-at): latest-at semantics supporting sparse unchanged values.
- [Rerun: RRD format](https://rerun.io/docs/concepts/logging-and-ingestion/rrd-format): format and rewrite tools for RRD artifacts.
- Code revision captured by dataset stages: `9422620c8e6e0ac0bffad26909b8b6ae6f39bf7a`.
