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

## Measured comparison

All physical work used the production native Rust engine, HP3, fixed 30 mm radii, zero buildup and half-width, no layering, sensor preload 1 N, feed 1400 mm/min and sixteen noise samples at 40 Hz requested. Actual Node 23.11.1 was checked after sourcing the cloud environment. Supervisor preflight and the separate command socket probe passed. Recording was none. The same native artifact hash was retained. No shared build or dependency change ran.

The baseline used the automatic size tuner, not literal origin constraints. Its observed lower-anchor fixed offsets were 1.2692 mm and 57.8430 mm. Before fitting, CVF-E01 declared two levels per role around those values, with a delta of the smaller of 40 mm and one quarter of observed encoder travel. Machine bounds clamp the nominal negative request to zero. Actual levels were 0/24.7958 mm nominal and 17.8430/97.8430 mm soft. Five samples per physical direction make ten per combined sweep, so all six sweeps total exactly 60 points. All recorded values were finite.

The exploratory continue threshold was at least 20% conventional geometry improvement in both worlds, no validity/coverage regression and at most twice the wall/step work. No coverage fit passed the 20% threshold. The route was cut at 12:28:40 UTC, before the requested checkpoint window elapsed.

| World and solver | Baseline anchor RMSE, mm | Six-sweep RMSE, mm | Replayed-force RMSE, mm |
| --- | ---: | ---: | ---: |
| Nominal LBFGSB ordinary | 2023.36 | 3214.08 | 1867.09, sentinel and invalid |
| Nominal TRF ordinary | 2003.45 | 1929.61 | 3.29 |
| Soft LBFGSB ordinary | 3648.26 | 3096.42 | 10.43 |
| Soft TRF ordinary | 2676.18 | 3972.08 | 11.64 |

RMSE is the square root of mean per-anchor squared Euclidean error against the repository's conventional effective scene-home anchors. That reference lacks the observed settled encoder-zero pose and rotation. It is not established physical absolute accuracy. Geometry metadata was stripped from every new fit input. Seeds, collection offsets, forces and online choices used observations or returned fits, never reference anchors.

## Force rival and recovery

Each production force search used eleven minimum-force probes and two bisections. Nominal used ten edge probes, soft nine. Exact selected low/mid/max forces were 0.0123966280704/0.247932561408/2.613544005371242 N and 0.0113635757312/0.227271514624/1.711249051135933 N. These are existing production autotuning outputs, not a new force algorithm.

Through the first post-tuning global-mid-force command, the worlds had used 551484 and 569073 physics steps, or 1102.968 and 1138.146 simulated seconds. Phase wall timing was not recorded. The original helper's one-million-step cap stopped collection with 30 nominal and 40 soft points. Those trials are incomplete accuracy comparisons, not evidence that automatic force tuning gives poor geometry. Their logs, journals, traces and manifests remain intact.

Decision decision-b044bb8844bb44fbb318395b8b877e4a authorized exactly two additional fresh-world collections, expanded the cumulative cap to six collections and six million steps, and kept the original deadline and 24-fit cap. CVF-E03 replayed the exact trace-derived force values without tuning again. The automatic size tuner remained active. Fresh-world state history differs from the interrupted trials. Both replays completed three sweeps of twenty points.

The nominal ordinary LBFGSB fit hit the existing underconstraint sentinel. The actual production recovery tried restore_sweeps, which remained invalid, then restore_points, which reached 3.29 mm conventional error in 0.46 s. Thus a solver change is not justified. The soft default ordinary fit gave 10.43 mm. Information rank deficiency still appears in the plans, and covariance summaries remain broad. No false acceptance or full-loop stopping improvement is claimed.

Compared with the original manual-force data, replay drive spans grew from about 94/235 mm to 1235/1362 mm. Drive-plus-sensor sum spans grew from about 1.54/8.88 mm to 168/258 mm. Matching drive and sensor spans in the baseline did not imply zero curvature. Replay points all reported noise below the configured floor, so this is quantized, quiet simulation data. Maximum recorded fixed-anchor mean error was 0.0241 mm nominal and 0.00733 mm soft, with no slip recovery events. The force policy also changes the size tuner's output and physical history; these measurements do not isolate one mechanical cause.

## Independent saved-sweep predictions

[The exact reconstruction and prediction script](holdout-prediction-measured.py) groups the earlier force-trial point journals by fixed anchors and physical direction. Only complete pairs of ten-point directions are used. It remaps the reversed physical direction with the production canonical orientation rule and takes observed fixed lengths from the final saved point. The extra incomplete nominal direction is excluded. This creates one whole nominal sweep with 20 points and two soft sweeps with 40 points. They are independently recorded movements in the same authored scenes, not new scenes or hardware measurements. They were never fit by the fresh-world replay models.

All models were frozen. No fit or physics ran for CVF-E04. [Results](holdout-results.json) identify every source stage and reconstructed dataset. RMS values below use unfiltered physical Sampson residuals; production prediction also enforces complete coverage.

| Model | Nominal valid/expected | Nominal RMS, mm | Soft valid/expected | Soft RMS, mm |
| --- | ---: | ---: | ---: | ---: |
| Baseline LBFGSB | 20/20 | 31.1688 | 40/40 | 31.9185 |
| Baseline TRF | 20/20 | 31.5984 | 0/40 | Failed prediction |
| Force default with existing recovery | 20/20 | 0.02045 | 40/40 | 0.10302 |
| Force TRF | 20/20 | 0.02045 | 40/40 | 0.10803 |

The zero-coverage baseline TRF result is a failure, not a low score. Prediction checks support the native force comparison within these scenes. They do not establish that other geometries, noise levels or hardware will behave similarly.

## Timing and limits

| Acquisition | Nominal wall, s | Soft wall, s | Nominal steps | Soft steps |
| --- | ---: | ---: | ---: | ---: |
| Saved manual-force baseline | 26.324 | 48.699 | 192909 | 409545 |
| Six-sweep coverage | 33.840 | 77.032 | 243641 | 639730 |
| Interrupted automatic forces | 95.987 | 94.715 | 1000000 | 1000000 |
| Fresh learned-force replay | 76.868 | 55.249 | 741617 | 541269 |

These are measured world wall times, including startup and cleanup. The fresh replay pair excludes repeated learning, and no speedup claim is made against the original baseline. Nominal replay itself exceeds the declared twice-baseline wall/step bound. Total actual work in this assignment was six collection invocations, ten fit stages and 4166884 total physics steps, including 627 regression steps, with 433.691 seconds of measured world wall time. [Usage](usage.json) lists every world and the revised authorization. This is bootstrap collection plus fitting/recovery, not a complete one-click automatic stopping run.

## Production change and validation

Commit 25822cbbde6703d68b693f6e93bcfa23b2b78593 atomically writes OUTPUT.force-tuning.json after force selection, before positioning and sweeps. Before this change, an interruption after successful tuning left no machine-readable force configuration because the dataset was assembled only after all sweeps. The sidecar now preserves exact selected forces, diagnostics and machine/spool context without M669 geometry. Stages distinguish successful tuning, failed-tuning fallback and manual selection. A manual rerun refreshes old automatic metadata. The same forceTuning object and tuned_at value feed the sidecar and final dataset; later recovery can update final collection forces. There is no automatic cache loading or change to acceptance/stopping policy.

The final code passes 69 behavior/CLI tests and 17 Python helper/native-collector tests, with five slow cases deselected. Tests exercise interruption after success or fallback, stale-auto replacement by manual selection and checkpoint write failure before collection. The actual native measurements predate the checkpoint delivery change; no seventh native collection was run to test the sidecar. That implementation has lifecycle regression validation, not an extra end-to-end native trial.

Commit 39510991a07057496bcd645b8ae9465cdfbd8660 separately adds optional per-config fixed targets, preserving anchor/target association when sorting and keeping independent levels through deduplication. It enabled the actual six-sweep test. It changes no acquisition default and carries no accuracy claim. Both commits are unpushed pending parent review. The generated research index is excluded from commits. Neither change touches physics, motors, cable transport or scene dynamics, so the shared rebuild physics gate does not apply.

## Approach ledger and next decision

Coverage allocation is rejected for the tested bounded offsets and equal point count. More random starts is not a fresh hypothesis here; current source already samples full bounds after its production seed. No new seed experiment ran. A new solver policy is rejected as unnecessary because existing recovery succeeds. Existing force adequacy is supported for these two replayed scenes, with time, reference and generalization limits. Faster minimum-force bracketing remains untested. The retained production improvement is durable force selection after interruption; the next research decision should compare force-search cost while preserving learned decisions and physical data quality.

## Evidence references

- [Predeclared coverage contract](coverage-contract.json), [force contract](force-contract.json) and [authorized replay contract](force-replay-contract.json) record decisions before fits.
- [Measured summary](measured-summary.json), [recovery](recovery-results.json) and [holdout results](holdout-results.json) support numerical claims.
- [Native provenance](provenance.json) hashes raw inputs, measured source snapshots, stage artifacts and all six manifests.
- [JS checks](js-regression.log), [Python checks](python-regression.log) and [source patch](candidate.patch) support code readiness.
- [Prior reviewed native report](../2026-10-27-autocal-parameter-variation-manager/native-report.md) and [pinned baseline artifacts](../attempt-982dc44e4c9b4e5da9b3232c26d54bf3/pinned) supply the comparison. The prior scene-home reference caveat remains in force.

Regression accounting: the research collections used 4166257 steps. The 17 Python checks additionally used 627 bounded steps across native Rust, independent Python and Warp test worlds. [Accounting](regression-step-accounting.json) preserves small traces and the two asserted post-reset steps absent from final events. Cumulative physics work was 4166884 steps, below the revised six-million cap.
