# HP3 encoder autocalibration: reference failure and runtime bottlenecks

Status: complete for the supplied reference; no source change justified.

## Objective and constraints

Investigate speed, robustness and fewer-point collection for encoder-only HP3 autocalibration while preserving one-click collection and automatic stopping. Use zero buildup and disabled line layering. Prefer the supplied run at `output/extended-autocal/no-buildup-reference-002`; do not search spool radii. Source changes should remain small. No hardware measurements were made.

## Hypotheses and experiments

| ID | Hypothesis | Outcome | Evidence |
|---|---|---|---|
| E01 | The supplied HP3 reference can complete and automatically accept a calibration. | Rejected | [Run log](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log), [full-auto events](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto_log.jsonl) |
| E02 | Fitter/recovery wall time dominates the long user wait. | Rejected for this run; collection dominates simulation time. Detailed RRD event timing was not completed. | [Recorder manifest](../../../../extended-autocal/no-buildup-reference-002/hangprinter-20261010T151618.949443Z-c9ffe4cd.json), [run log](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log) |
| E03 | Simply halving points per sweep is a robust default improvement. | Rejected based on prior recorded fixtures (3/4 anchor RMSE worsened); no new trial here. | [Prior report](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md#disproved-ideas-and-remaining-experiments) |
| E04 | Selection scores can disagree with geometric anchor accuracy. | Supported by prior audits; this run independently fails geometry/rank despite low residual noise. | [Prior physical audit](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E01-E01-baseline-audit.json), [run log](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log) |
| E05 | Encoder quietness plus zero measured noise ensures identifiable anchor geometry. | Rejected for the supplied simulated run. | [Dataset](../../../../extended-autocal/no-buildup-reference-002/sweeps.json), [full-auto log](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log) |

## E01/E04 observations

The supplied RRD manifest records browser-js HP3 data over 4,706,150 physics steps (9,412.3 simulation seconds), 470,616 physics samples, and 612,620 events. The RRD is 2.0 GB. Its `autocal_complete` is false. The dataset ended with eight sweeps (160 points): three initial pair geometries, three additional sweeps with the same `[2,3]` fixed pair and `[0,1]` drive/sensor pair, then two new selections. The full-auto log shows an early finite low-score solution (UI score 0.1952), then unstable anchor estimates, rank 0/12, 100-point underconstraint sentinel and two exhausted recovery modes. It stopped because no valid candidate sweep was found. This is a failed trial, not evidence that the one-click workflow completed.

Points report sigma zero and fixed-target tracking errors generally within 0.06 degrees. Yet later estimates place anchors thousands of millimetres away and become rank deficient. Low encoder noise and quiet fixed motors therefore do not by themselves ensure useful geometric excitation or a valid final selection. The run log's per-iteration recovery reports about 1.35 seconds for restore attempts; collection includes repeated 15-second stability windows and long force tuning. This makes data acquisition the first user-wait target, while the failed geometry means an unconditional point-count cut is unsafe.

The requested detailed RRD profile was attempted with the documented `rerun.chunk.RrdReader` event query. A full `.to_chunks()` on the 2.0 GB recording did not return within the 10-second tool window; no claim about per-event compute allocation is made. Stage snapshots and JSONL summaries provide the fitting/recovery measurements above. A bounded streaming RRD profile remains proposed.

## Ideas assessed

1. **Encoder-only identifiability checks:** Use the measurement Jacobian/Fisher information after removing gauge freedoms, alongside residual score; flag repeated sweep geometry and reject early stopping when geometry is rank deficient. Promising, but adding a gate alone could increase collection time unless paired with a better feasible sweep planner.
2. **Collection-time-aware sweep planning:** Rank next feasible anchor pairs and spans by expected information gain per measured collection second, with explicit coverage of pair/height geometry. This addresses the repeated `[2,3]` pair seen here. Not implemented or tested in this turn.
3. **Adaptive point count:** Begin with sparse points and add points only where fit uncertainty or held-out encoder prediction remains high. Prior simple thinning degraded anchor RMSE on most tested recorded fixtures, so a fixed reduced count is rejected; adaptive stopping is untested.
4. **Settling-window optimization:** Prior research found recent-window checks can save about 25% simulated collection time with a guard while retaining all points, but evidence was fixed HP4 collection rather than this HP3 full-auto trajectory. See linked report.
5. **Score/geometry disagreement:** Prior candidate ranking audits found many physical-error disagreements. Held-out scoring, age-bonus removal and bounded updates did not generalize enough to adopt. Keep physical synthetic truth audits and identifiability diagnostics as evaluation criteria.

## References

- [Supplied HP3 zero-buildup dataset](../../../../extended-autocal/no-buildup-reference-002/sweeps.json): encoder measurements and sweep identities for E01.
- [Supplied full-auto log](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log) and [iteration JSONL](../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto_log.jsonl): fits, stopping and recovery timings for E01/E02/E04.
- [Supplied RRD manifest](../../../../extended-autocal/no-buildup-reference-002/hangprinter-20261010T151618.949443Z-c9ffe4cd.json): simulated recording coverage and completion status for E02.
- [Prior October 8 report](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md): recorded anchor-score mismatch, point-thinning results and measured guarded settling improvement.
- [Autocal algorithm overview](../../../../../autocal/optimization_algorithm_overview) and [objective overview](../../../../../autocal/objective_functions_overview): current stages, ranking and identifiability diagnostics.
- [Extended reference guide](../../../../../autocal/data/references/extended-reference.md): RRD event and clock model used to interpret the supplied run.
- [Research integration guide](../../../../../research/README.md) and [integration rationale](../../../../../research/investigation.md): experiment and evidence conventions.
- Code revision captured by dataset stages: `9422620c8e6e0ac0bffad26909b8b6ae6f39bf7a`.

## Next decision

No code change is justified from this failed collection alone. Next run should use a rank/coverage-aware planner or bootstrap candidate audit, compare its selected sweeps against the current planner on the same fresh HP3 world, and stream RRD event timing in bounded batches. Keep default collector point counts until adaptive-count trials show noninferior anchor geometry and automatic acceptance on multiple independent HP3 seeds.

## Follow-up: RRD size, sparsity and compaction

The question was whether RRD permits discarding time pockets with no activity. Rerun's official format/model docs and local data-model skill support sparse temporal samples: latest-at holds a component's prior value until a later update, so unchanged state need not be repeated. The recorder already logs at stride 10 here and Rerun suppresses unchanged poses. However, removing time pockets from a finished run is a lossy derived recording; it needs an explicit activity predicate and must preserve absolute timelines/events and enough state around transitions. It is not the same as format compaction.

Experiment E06 ran `.venv/bin/rerun rrd stats` on the supplied RRD: 473,771 chunks, 24,840,564 rows and 1.9 GiB native chunk bytes; Rerun's object-store estimate was 2,232 chunks, indicating roughly 212x chunk-count overhead. Top large groups include physics line geometry, 37,668 event-log chunks, and per-step clocks. E07 ran supported `.venv/bin/rerun rrd optimize --profile object-store` to a separate `/tmp` file: 473,771 to 1,438 chunks (-99.696%), 1.9 GiB to 1.1 GiB (-41.338%), in 66.88 seconds. The original remained unchanged. Rerun reported it completed compaction and encoded output. Separate `verify` and `compare` CLI processes did not return a final result in the available observation window; semantic equivalence is therefore not claimed as independently verified. This safe, schema-preserving operation shrinks the artifact but remains too large to eliminate the need for bounded queries.

For future recordings, reduce cost at source where detail is not needed: increase physics sample stride or use compact geometry; keep full-rate command, encoder, measurement and event records. For an old RRD, `rrd optimize` compacts chunks without discarding timeline samples. An activity-window derived RRD can go further, but should be a separate file, retain a map from output to original timestamps, preserve transition margins and all autocal/collector events, and explicitly label gaps; never silently replace the raw record.
