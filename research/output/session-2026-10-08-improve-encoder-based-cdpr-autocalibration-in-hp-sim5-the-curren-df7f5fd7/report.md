# Independently testable encoder calibration stages

Status: complete.

## Abstract

Encoder autocalibration was split into modeled-length transformation, fitting and assessment, frozen sweep planning, prediction evaluation and pure history ranking. Versioned JSON artifacts allow downstream experiments without rerunning fitting or collection. Eleven recorded simulated baselines reproduced current references and showed 20 adjacent score/parameter-error disagreements, with physically worse final selections on ten datasets. The final implementation preserves every returned and intermediate calibration exactly. A small metadata-copying optimization reduced mean fitting time from 3.290 to 2.992 s in seven alternating pairs; the complete regression matrix changed from 201.512 to 192.633 s (4.4% faster). Frozen planning, prediction evaluation and ranking take about 32.7 ms, 14.5 ms and 2.27 microseconds. All 337 Python tests pass. Accuracy is preserved rather than improved; evidence is simulated, runtime samples are bounded, and fitting must keep borrowed nested measurement metadata read-only.

## Topics

autocalibration, encoder, CDPR, modular pipeline, stage replay, runtime, metadata copying, selection regret

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Existing ranking can favor physically worse parameters | E01 | Supported | [Physical audit](E01/physical-audit.json), [baseline results](E01/results.json) |
| Filtering can repeatedly fail to restore constraints | E01 | Supported: fifth HP3 has six failed recovery attempts | [Recovery audit](E01/recovery-audit.json), [log](E01/fifth_hp3_dataset.log) |
| Stage extraction preserves calibration and permits downstream-only trials | E02a, E03, E04, E08 | Supported on all 11 fixtures; frozen fit replay exact | [Paired results](E02a/results.json), [stage checks](E03/benchmark.json), [final comparison](E08/comparison.json) |
| Information reuse removes redundant planning work without changing ranks | E03, E06, E07 | Supported; standalone extraction initially had inconclusive full-fit timing | [Alternating planner measurements](E07/benchmark.json) |
| Borrowing read-only metadata accelerates fitting without changing lengths or calibration | E07, E08 | Supported within measured battery | [All-dataset transformations](E07/transforms.json), [fit pairs](E07/benchmark.json), [full regression](E08-regression.log) |
| Raw-rank-only selection improves the small frozen case | E05 | Supported for one fixture; not accepted as a general policy | [Frozen-history pilot](E03/raw-rank-pilot.json) |

## References

- [Research notebook](research.md): objective, steering, budgets, failed driver trial and decisions.
- [Final regression command, runtime and status](E08-results.json), [full output](E08-regression.log): all eleven reference checks pass with zero deltas.
- [Source/input provenance](provenance.json), [source patch](final-source.patch), [saved source](source/): baseline revision, final source hashes, dataset/reference hashes and numerical environment.
- [Prior rejected selection and solver experiments](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md): why a single-fixture accuracy improvement is insufficient for changing selection.

## What changed and why

The user's Flyvbjergian requirement is interpreted as small components with explicit inputs, outputs and fast experiments. No external Flyvbjerg paper or quotation was used. The original implementation already had separate initialization, alternating refinement, scale polish and filter modules, but a planning pass combined fitting, uncertainty assessment, candidate generation, ranking and collector command writing. Final history prediction and ordering were nested in the full-auto loop.

`fit_stage.fit_ellipse_dataset` now returns a frozen fitted model, raw and modeled data, diagnostics, and the observed information matrix. `planning_pass.plan_ellipse_sweep` uses that model without fitting. The existing `plan_next_ellipse_sweep` composes these stages for existing callers. Planning reuses the unregularized matrix at the same finite-difference setting; a changed step size recomputes it. Candidate regularization is applied once without mutating the matrix. The exact modeled sweep configurations are retained, including when the estimation dataset has been filtered.

`history_selection.evaluate_history_candidates` evaluates the saved models; `rank_history_candidates` applies the existing prediction veto and ordering to saved evaluations. The full-auto loop still controls patience, recovery, collection and firmware application. No objective, stopping rule, point count or selection policy changed.

`--stage-artifacts DIR` optionally saves typed, versioned JSON data with source hashes. Repeated runs choose numbered sibling directories. `python -m autocal.tools.replay_stage` runs `fit`, `plan`, `evaluate` or `rank` from those artifacts. Fitting uses embedded sweeps rather than a dataset file that may have grown. Planning writes a config but executes no collection. Default calibration does not write these artifacts. The regression runner adds optional `--jobs` and repeated `--dataset-name` controls; default scheduling and acceptance rules remain unchanged.

Profiling then identified repeated deep copies inside the existing modeled-length transformation component. In fitting, each new model now copies the root/sweep/point record dictionaries and deep-copies configuration, while borrowing nested encoder/noise metadata that the solver reads. The public `dataset_with_modeled_lengths` keeps full isolation by default. Scalar conversion arithmetic is untouched. This is the small performance change that cleared the runtime concern after extraction alone had mixed timing.

## Reproduction before changes

E01 used revision `8811814c1c80ac52ebb5dabf380713f14e8d68a4` and the actual regression helper on all eleven recorded simulated fixtures, with two workers and the current October 8 reference logs. Every reference comparison passed exactly. There were 20 adjacent rank/ground-truth direction disagreements. The final selected candidate was physically worse than an available candidate on ten of eleven fixtures. The diagnostic total is sum of Euclidean anchor errors plus `2*pi*sum(abs(radius error))`; it is not end-effector positioning error. Logs print rounded parameters, while stage artifacts preserve full precision.

| Recorded fixture | Returned total error mm | Best available total error mm | Best iteration | Returned selection regret mm |
| --- | ---: | ---: | ---: | ---: |
| five_points_bigger_deltas | 6.809 | 6.241 | 3 | 0.568 |
| flexible_lines_2000 | 27.671 | 23.393 | 3 | 4.278 |
| layered_manual_some_bad_points | 19.974 | 4.919 | 3 | 15.055 |
| ten_points_bigger_deltas | 18.207 | 3.306 | 1 | 14.901 |
| ten_points_w_buildup | 8.277 | 4.645 | 3 | 3.632 |
| ten_points_w_buildup2 | 5.795 | 4.130 | 3 | 1.665 |
| fifth_hp3_dataset | 70.787 | 70.314 | 3 | 0.473 |
| seventh_hp3_dataset | 202.974 | 161.499 | 3 | 41.475 |
| sixteenth_hp3_dataset_even_more_pressure | 52.008 | 41.866 | 5 | 10.142 |
| nineteenth_hp3_dataset_even_more_pressure | 169.642 | 169.642 | 2 | 0.000 |
| 23_april_2026 | 234.793 | 97.541 | 6 | 137.252 |

For `ten_points_bigger_deltas`, iteration 3 returns 17.529 mm summed anchor error and 0.108 mm summed radius error, total 18.207 mm; iteration 1 has 1.534 mm anchor error and 0.282 mm radius error, total 3.306 mm. Fifth HP3 reproduces three underconstrained selected iterations and six failed recovery attempts. Across six fixtures there are sixteen recovery attempts and six exhausted episodes. These demonstrate fragility to lost constraints without claiming a new native-process crash. No fresh process crash occurred in these bounded runs. Prior full-concurrency crashes are linked historical evidence.

## Measured performance and preservation

| Experiment | Scope | Wall / stage time | Outcome |
| --- | --- | ---: | --- |
| E01 | Baseline 11-fixture replay, two workers | 201.512 s | Every current reference exact |
| E02 | First paired driver | Not an accepted measurement | Failed reference lookup; preserved |
| E02a | Extracted pipeline vs frozen E01 | 202.144 s | Every parameter and score exact; early load overlap |
| E04 | Actual regression CLI after extraction | 206.313 s | All pass; 2.4% longer than E01, prompted further work |
| E06 | Seven old/new full-plan pairs, extraction only | Medians 3.197 / 3.346 s | Exact results; variable timing, no speed acceptance |
| E07 | Seven old/new full-plan pairs, optimized copying | Means 3.290 / 2.992 s | All seven faster, all results exact |
| E08 | Actual final 11-fixture regression CLI | 192.633 s | All pass; 4.4% faster than E01 |
| E07 | Frozen planning / evaluation / ranking | 32.70 ms / 14.47 ms / 2.27 us | Decisions and prediction keys preserved |

All E08 summary and intermediate parameters, scores and iteration counts match E01 exactly; the summed returned diagnostic error remains 816.936 mm and trajectory deltas are zero. That preserves accuracy and existing failures. Transform-only trials use 20 alternating pairs per fixture: all modeled fields compare exactly, and inputs remain unchanged. The copying optimization is faster on all eleven transformations. Full-fit E07 pairs compare the original composition/full-copy behavior with the extracted composition/borrowed metadata behavior; this is a combined final implementation comparison. The transform-only tests isolate copying.

The profiler's 736 transformations and 2.285 seconds in `deepcopy` were obtained with profiling overhead and are used only to locate work. They are not an uninstrumented speed measurement. Timing runs warm JAX before paired full-plan comparisons; initial fit artifacts include a different cache state. Frozen-stage times exclude process startup and artifact I/O. Whole-matrix times include subprocesses, cache reuse and reporting. The final matrix is a bounded observed improvement, not a universal runtime guarantee across machines.

![Reproduced parameter error and independent stage runtimes](stage_evidence.png)

Left: full-precision simulated candidate errors, with the existing selected iteration marked. Right: mean full-fit time and median downstream-only times from E07; all exclude interpreter startup and artifact I/O.

## Cheap hypothesis trial and failed evidence

E05 reranked one frozen history using only raw rank. It picked iteration 1 rather than 3, changing full-precision total error 18.247 to 3.365 mm. This trial took about 70 microseconds after loading history and used no fitting, residual reevaluation or collection. These numbers differ from rounded-log diagnostics. The policy remains unshipped because prior matrix evidence found that removing the age adjustment worsened other datasets. Modularity makes that question cheap to test without quietly adopting an oracle based on simulator truth.

E02 failed because the experiment driver renamed baseline logs to `<name>.log`, while the reference resolver expected dated reference names. The calibration subprocess logs and failure trace remain under E02. E02a corrected that path lookup; no reference logs were changed. E03/E06 preliminary timing results are retained even though they did not satisfy the runtime requirement.

## Verification and limitations

The final Python suite has 337 passing tests and two deselected slow collection tests. Tests cover exact cached versus recomputed information, matrix nonmutation, changed finite-difference settings, frozen JSON planner replay with fitting forbidden, history vetoes/fallback, metadata-copy equality, source/config mutation isolation, artifact version validation and bounded regression arguments. Actual recorded-data fit replay preserves full-precision anchors and cost.

A fresh `python -m autocal.tools.replay_stage plan` invocation also passed ([CLI output](E08/cli-plan.log)). A one-iteration export smoke test passed, created `stages_1` beside the existing artifact directory and skipped firmware application ([output](E08/export-smoke.log)). These are tool checks, not accuracy measurements.

All experiments here are offline replays or transformations of simulated encoder collections, plus synthetic component tests. No hardware movement or new physics collection occurred. This is warranted because measurement arithmetic, collection, the optimizer and movement selection remain unchanged. Live end-to-end collection was not newly validated. Nested measurement metadata borrowed during fitting must stay read-only; public callers requiring independent mutable metadata retain the default full-copy mode.

The fit/assessment stage and shared `_autocal_common.py` still contain substantial code. These changes expose useful boundaries rather than claiming a complete architecture rewrite. Ranking can still select poor anchors, and different candidates still use different future-sweep sets. Improving that accuracy is a subsequent independently measurable task, with the complete matrix as its acceptance gate.

## Sources actually used

- [Autocal README](../../../../autocal/README.md), [algorithm overview](../../../../autocal/optimization_algorithm_overview), [objective overview](../../../../autocal/objective_functions_overview), and [detailed guide](../../../../autocal/README_elliptical_feature_calibration.md): pipeline, model, scoring and stopping interpretation.
- [Research tools README](../../../../research/README.md) and [integration rationale](../../../../research/investigation.md): session/world ownership and choice of recorded replay instead of unnecessary fresh collection.
- [E01 inputs and final code hashes](provenance.json): every actually used dataset, resolved reference, baseline revision and final source revision represented by hashes and saved source.
- [Prior negative-results session](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md): score/accuracy disagreement, rejected alternatives and prior native crashes.
- [Reference-refresh comparison](../../reference-refresh-october-7-2026/report.md): historical selection tradeoffs and scheduling controls; its older measurements are not relabeled as current results.
- [E01 matrix driver](matrix.py), [E07 paired benchmark](benchmark_copying.py), [E07 transform benchmark](benchmark_transforms.py), [actual E08 command wrapper](run_final_regression.py): reproducible experiment workflows.

- [Experiment registry](experiment_registry.json): completed and failed experiment IDs, statuses and evidence paths.
- [Final Python test output](tests-final.txt): 337 passed, two slow collection tests deselected.
