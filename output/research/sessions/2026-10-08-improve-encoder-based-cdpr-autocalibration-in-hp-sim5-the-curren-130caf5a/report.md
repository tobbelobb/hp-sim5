# Faster encoder autocalibration through exact spool-model dataset reuse

Status: complete.

## Abstract
This investigation asks whether encoder-only CDPR calibration can obtain reliable results with less collection or computation. Eleven fresh browser-simulation fixture replays reproduce 20 score/physical-error disagreements and nine underconstrained recoveries, six unsuccessful. Two comparable-validation rules remain unreliable. Uniform and geometry-aware half-point selection each worsen anchors on three of four recorded fixtures; 18 analytical synthetic fits show conditional, mixed benefits. Profiling instead finds thousands of repeated spool-model dataset conversions. A small implementation caches up to 16 exact radius/buildup transformations per filter pass, enabling existing residual-cache reuse. All eleven candidate regressions preserve parameters, stopping decisions and 5,184 numerical log fields exactly. Isolated three-repeat comparisons reduce median full-auto replay time by 2.5% on layered_manual_some_bad_points, 10.4% on seventh_hp3_dataset. The retained change reduces fitting work; collection point counts and physical ranking remain unchanged. Evidence is recorded simulation and analytical synthetic data, not hardware or new-data positioning validation.

## Topics
CDPR, Hangprinter, HP3, Slideprinter, encoder-only autocalibration, exact spool cache, numerical speed, informative points, fewer points, common held-out sweeps, robustness, automatic stopping

## Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Current scores agree with physical accuracy | E01 | Rejected | [Ranking audit](E01-E01-baseline-audit.json), [recovery audit](E01-fragility.json) |
| A common last sweep improves final ranking robustly | E02 (E00 archived control) | Mixed; rejected as default | [Shared comparison](E02-common-audit.json) |
| Every-shared-sweep dominance avoids physical regressions | E02 | Rejected as a general guard | [Paired comparisons](E02-common-audit-paired.json) |
| Geometry-aware thinning preserves full-data accuracy | E03 | Mixed; rejected as default | [Recorded comparisons](E03-recorded-comparison.json), [18 synthetic fits](E03-synthetic-results.json) |
| Exact bounded transformed-dataset reuse preserves behavior and speeds fitting | E04–E06 | Supported on tested fixtures | [Profiles](E04-profile), [11 regressions](E05-cache-regressions.json), [timing record](E06-isolated.txt) |
| Exact keys, eviction and pass isolation work | E07 | Supported tool checks | [Spool tests](E07-spool-tests.txt), [322 Python checks](E07-python-tests.txt) |
| Joint nuisance-aware acquisition, continuous acquisition and settling reuse improve collection | none | Not tested | [Proposals](report.md#remaining-experiments) |

## References

- [Full report](report.md): method, measurements and limitations.
- [Research notebook](research.md): decisions, constraints, steering and budgets.
- [Source and dataset manifest](manifest.json): revision, hashes and numerical environment.
- [Retained source diff](final-source.diff): exact bounded cache and tests.
- [Prior collection investigation](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md): earlier thinning failures and guarded settling already in the baseline.

## Decision and scope

Retain exact, bounded spool-model dataset reuse in `autocal/filter_pass.py`. The 19-line production change caches `(radii, buildup)` pairs without rounding, for one fixed filter pass, retaining at most 16 transformations with FIFO eviction. Repeated calls receive the same model/dataset pair, so existing identity-keyed cost-function and evaluation-bundle caches can also hit. A new pass creates a new cache. Near-distinct values remain distinct. Existing tests were extended for copied input arrays, tiny radius/buildup changes, eviction and isolation across passes; the public fitting function is used in that updated test.

The collector and all scoring, solver search, validation and stopping rules retain their behavior. No additional setting, user decision or manual stop is introduced. Acquisition point counts stay at their existing defaults. This does not fix the known disagreement between fit score and physical geometry, and does not claim a smaller reliable calibration dataset.

![Recorded accuracy and isolated compute timings](results.png)

[Figure PDF](results.pdf), [plot code](plot_results.py). Bars in the timing panel show medians; dots show all three repetitions.

## Ideas, literature and prior research

The initial list, derived from first principles and literature before source tracing, included redundant cable-distance closure with latent poses; circle/ellipse invariants; joint anchor, spool and offset estimation; informative point selection; comparable whole-sweep validation; bidirectional hysteresis checks; staged fitting and numerical reuse; reusing settling/noise windows; and continuous acquisition with a dynamics model. Relative measurements require a coordinate gauge and a metric reference. If spool scale is freely unknown, geometry and scale can trade off; low encoder residual is insufficient to establish physical accuracy.

The current implementation already covers much of this list. It collects both physical drive directions and canonicalizes their roles, minimizes predicted-ellipse Sampson residuals in squared lengths with staged robust filtering, optionally nests spool and anchor fits, and proposes sweeps by geometry-only D-optimal gain. Rank/age/coverage controls patience. At final acceptance, an own-model future-sweep prediction acts as a validity gate and a shared-length prediction compares anchors. Candidates can have different validation movement sets.

Prior sessions tested age-bonus removal, alternative objectives, bounded updates, whole-sweep ranking and simple thinning. Those failures narrowed this study to common-set comparison and informative placement rather than repeating every rejected optimizer. Persistent JAX compilation and guarded settling are already implemented in baseline revision `b04de035b4d6e5cc64f12dccfede0b63b55f4997`. The substantive new retained change is within-pass transformed-data reuse.

## E01: fresh reproduction

Each of the eleven immutable reference JSONs was copied into its own session experiment directory. The normal full-auto CLI ran with each fixture's registered machine, radius, buildup and bound settings, `--sim --no-collect`, and automatic stopping. Ground truth was used only after fitting. These fixtures are recorded browser physics measurements, not hardware. Expected effective anchors and radii come from the regression definitions, as explained by the reference-data provenance README.

Twenty adjacent rank/physical-error direction mismatches occur across nine fixtures. Nine iterations invoke underconstrained recovery; six recoveries fail. The finite sentinel is not evidence of a constrained fit. Fresh baseline replay takes 407.459 aggregate wall seconds, with some bounded overlapping analysis work; these times are workload observations rather than isolated performance measurements.

| Fixture | Returned anchor RMSE mm | Regret versus best fitted candidate mm | Iterations | Replay wall s |
| --- | ---: | ---: | ---: | ---: |
| five_points_bigger_deltas | 2.848 | 0.449 | 6 | 33.53 |
| flexible_lines_2000 | 9.660 | 2.167 | 4 | 24.82 |
| layered_manual_some_bad_points | 6.229 | 5.063 | 7 | 51.57 |
| ten_points_bigger_deltas | 5.930 | 5.378 | 6 | 35.73 |
| ten_points_w_buildup | 2.301 | 1.264 | 5 | 31.57 |
| ten_points_w_buildup2 | 2.088 | 1.065 | 7 | 44.75 |
| fifth_hp3_dataset | 18.471 | 0.088 | 7 | 46.21 |
| seventh_hp3_dataset | 55.812 | 15.232 | 5 | 35.09 |
| sixteenth_hp3_dataset_even_more_pressure | 14.339 | 2.653 | 6 | 35.02 |
| nineteenth_hp3_dataset_even_more_pressure | 42.130 | 0.000 | 5 | 31.22 |
| 23_april_2026 | 64.416 | 37.641 | 6 | 37.94 |

Regret in this table uses all fitted iteration candidates, including attempted fits that could be underconstrained; it is an evaluation diagnostic, not a proposal to accept those candidates. Seventh HP3 has a constrained physically better candidate: the returned anchors have 15.232 mm more RMSE than that candidate. The largest all-candidate regret is April HP3, 37.641 mm RMSE. The existing history-rank diagnostic and actual final summary select using different rules, so their reported selections need not coincide.

[Fresh commands/results](E01-baseline-runs.json), [raw calibration logs/events](E01-baseline), [physical ranking audit](E01-E01-baseline-audit.json), [recovery details](E01-fragility.json).

## E02: comparable-validation hypotheses

Frozen models from fresh E01 histories were rescored; no geometry was refitted and no additional data were collected. The own-model validity gate stayed intact. The first proposal uses the common last complete sweep for the shared-length anchor comparison, so eligible candidates see identical movement difficulty. It improves four fixtures, worsens two, and leaves five unchanged. Five-points anchor RMSE rises 1.409 mm and ten-points rises 3.398 mm. Aggregate gain does not establish robustness.

The second proposal starts at the current selected model and accepts a challenger only when it improves every common future-sweep score, with at least two common sweeps. It still worsens five-points by 1.409 mm RMSE. Scalar prediction dominance therefore does not certify better physical anchors. Both proposals were rejected as defaults. E00 used prior c812e7b9 histories as an exploratory control; E02 repeats the outcome with new E01 fits. Archived models are not mislabeled as fresh calibration runs. The scorer uses logged two-decimal M669/M666 parameters; final production equivalence instead checks the full numerical JSONL fields.

[All candidates and common scores](E02-common-audit.json), [paired common-sweep differences](E02-common-audit-paired.json), [archived control](E00-prior-common.json). Last-sweep results are also less statistically stable than a large independent configuration holdout; no hardware inference follows.

## E03: fewer points and informative placement

Uniform thinning preserves endpoints and at least five points per physical drive direction. The informative variant takes the same point budget, computes a finite-difference anchor Jacobian of signed mm residuals, whitens identifiable directions with the full-data SVD, then greedily maximizes D-optimal gain while retaining endpoints. Selection uses bootstrap-derived geometry and radius-prior midpoints (37.5 mm for Slideprinter, 39.5 mm for bounded HP3), not ground truth. There is no spool-nuisance column in this experiment.

This is retrospective selection from already measured observations, including whitening over the recorded candidate pool. It is an optimistic screening experiment, not a ready online acquisition planner or a measurement of saved collection seconds. The initial HP3 trials failed because nominal geometry invalidated some sweeps. Their script/results are preserved as v1. The tested v2 falls back to uniformly spaced points for those invalid-seed groups. That fallback is an algorithm trial; the original failures are not evidence that every informative acquisition strategy fails.

| Fixture | Full RMSE mm / points / iterations | Uniform half RMSE mm / points / iterations | Informative half RMSE mm / points / iterations |
| --- | ---: | ---: | ---: |
| layered_manual_some_bad_points | 6.229 / 180 / 7 | 13.799 / 90 / 7 | 3.704 / 80 / 6 |
| ten_points_bigger_deltas | 5.930 / 160 / 6 | 7.816 / 80 / 6 | 16.161 / 70 / 5 |
| seventh_hp3_dataset | 55.812 / 140 / 5 | 45.006 / 80 / 6 | 69.321 / 70 / 5 |
| 23_april_2026 | 64.416 / 160 / 6 | 67.049 / 100 / 8 | 110.671 / 80 / 6 |

Each point count is the observations available through that replay's stopping horizon, not all observations in the stored JSON. Uniform and informative thinning each worsen anchors on three of four fixtures. More sweeps can offset point savings. When available, frozen returned models are evaluated on the same complete sweeps beyond every model's training horizon. If no such suffix exists, the result records no independent holdout rather than inventing one. [Recorded outcomes and unseen prediction coverage](E03-recorded-comparison.json), [point-selection method](informative_points.py), [initial failed selection](E03-selected-points-v1.json).

Analytical HP4 experiments independently generate circle-constrained poses, convert known geometric distances into relative length observations, and fit anchors with known spool conversion. Nine sweep configurations train and three new fixed-length configurations validate. Two seeds each test clean 0.02 mm noise, 0.5 mm noise with two +20 mm sensor outliers per sweep, and 0.5 mm noise on short arcs. Full/ uniform/ informative use 216/108/108 training observations, for 18 fits. Starting geometry is randomly perturbed by 60 mm; selection sees that seed, not truth. Solver filtering is disabled and pseudo-Huber is enabled to isolate point placement.

The conditional advantage on short arcs does not transfer uniformly. In short-arc seed 0, uniform RMSE 7.373 becomes 3.321 mm with informative selection, compared with 5.564 mm on full data. In clean seed 1, full 0.083 becomes 0.179 mm with informative selection. Outlier seed 0 worsens from full 16.123 to informative 23.927 mm, while seed 1 improves from 14.562 to 11.222 mm. Synthetic known-spool results do not establish robustness of recorded joint anchor/radius fitting. [18 measured fits](E03-synthetic-results.json), [generator/fit code](synthetic_design.py), [synthetic input datasets](E03-synthetic-clean-0.json).

## E04–E06: exact numerical reuse

Source tracing showed `_build_dataset_and_params` deep-copies/recomputes length data even for identical radii/buildup. Existing cost-function and evaluation caches key by transformed-dataset identity, so equivalent new objects miss them. E04 instruments conversion and profiles normal full-auto replay before any production change:

| Fixture | Conversion calls | Exact duplicate calls | Profiled conversion s |
| --- | ---: | ---: | ---: |
| bad-points Slideprinter | 5330 | 2283 | 41.000 |
| seventh HP3 | 1730 | 1289 | 12.824 |

Profiling substantially inflates wall time (110.606 and 65.153 seconds), so those seconds are evidence of where work occurs, not benchmark gains. Duplicates are counted over each fixed source-dataset identity and exact spool model; not every historical duplicate remains in the 16-entry bounded cache.

E05 loads an immutable candidate filter-pass snapshot into the normal CLI module, leaving the tracked source unchanged during that experiment. All eleven candidate/reference and candidate/fresh-baseline comparisons pass at 0.01 mm. Returned geometry/radii, every intermediate parsed fit, all 5,184 non-timing numerical JSONL fields and all stopping/collection decisions match. The candidate snapshot is byte-identical to the implemented production filter-pass file; their hashes are recorded. Aggregate E05 time is 360.880 seconds versus E01 407.459 (11.4% less), but overlapping workloads differ. E06 therefore isolates repeated timing before making a controlled claim.

[Baseline profile data](E04-profile), [prototype and baseline snapshots](manifest.json), [11 regression comparisons](E05-cache-regressions.json), [full comparator output](E05-cache-regressions.txt), [numeric comparison script](compare_cache.py).

E06 runs three baseline/candidate repetitions per fixture, reversing baseline/candidate order in the middle repeat. Both snapshots use the same CLI, interpreter, fixture settings, warmed persistent JAX cache and counter instrumentation. No competing calibration/physics jobs run.

| Fixture | Baseline median s | Cached median s | Median reduction |
| --- | ---: | ---: | ---: |
| layered_manual_some_bad_points | 48.866 | 47.626 | 2.5% |
| seventh_hp3_dataset | 32.770 | 29.376 | 10.4% |

Conversion calls fall from 5,330 to 5,093 on bad-points and from 1,730 to 929 on seventh HP3. Cost-function builds fall from 4,066 to 3,857 and from 1,478 to 837 respectively. All twelve timed replays pass the same reference/fresh-baseline and exact numerical comparisons. The bounded cache captures only some previously observed duplicates; cross-pass reuse would require more cache-state invariants and was not implemented.

[Repeated numerical checks](E06-repeated-regressions.json), [all repetitions and counters](E06-isolated-runs.json), [timing summary](E06-isolated-summary.json), [runner](timing_cache.py). These are complete recorded-data full-auto fitting/replay timings, not isolated objective microbenchmarks or new physical collection timings. Three repeats do not establish a hardware-speed confidence interval.

## E07: verification and workflow

The production test suite passes 322 tests with two slow tests deselected; targeted spool checks pass 71. Cache tests verify exact keys, copied input arrays, tiny distinct parameter values, eviction and new cache identity for each filter pass. Those are correctness/tool checks, separate from physical accuracy. `git diff --check` passes. There is no stale experimental ranking path in production; rejected alternatives live only in session evidence scripts.

The one-click entry point retains bootstrap, force tuning, both measurement directions, adaptive sweep planning and automatic patience acceptance. Normal recorded replays use that loop, and numerical event/decision equality covers this change's stopping behavior. No physical collection routine changed, so launching new physics collection would add no discriminator for exact computation reuse. No collection job or hardware movement was started. Fresh collection/new-data positioning accuracy is not measured in this session. Existing ranking inaccuracies, spool/anchor ambiguity and compiler-path crashes under other workloads remain unresolved.

[322 production tests](E07-python-tests.txt), [71 targeted tests](E07-spool-tests.txt), [source diff](final-source.diff).

## Short negative findings

- Common last-sweep ranking: worsens two recorded fixtures despite four improvements.
- Unanimous paired future-sweep dominance: still worsens five-points; better residuals can compensate for physically worse geometry.
- Geometry-only informative halving: three of four recorded anchors worsen; do not cut default acquisition counts.
- Uniform halving: independently reproduces the earlier three-of-four regressions.
- A valid Jacobian at the initial HP3 geometry: rejected for the two tested full recorded pools; uniform fallback was tested instead.
- Synthetic sparse success as general evidence: rejected; known conversion and analytical circle geometry omit real collector/model errors.

## Remaining experiments

Proposed, **not tested** here: joint anchor/spool/latent-pose information with Schur-complement nuisance elimination; robust information per collection second; branch/pose prediction beyond a Sampson residual; acquisition using multiple plausible geometry models rather than one poor seed; bootstrap geometry stability as an automatic stopping guard; continuous acquisition with dynamics; bidirectional hysteresis modeling; and reusing encoder quiet/noise history. An explicit scale reference and independent configuration holdout should accompany any point-count reduction. A next acquisition study should collect identical movement budgets in fresh production physics worlds, measuring settling/noise, predictive coverage, final geometry and held-out positioning. It should keep the present one-click automatic stop.

## Sources actually used

- Han Yuan, Xianghui You, Yongqing Zhang, Wenjing Zhang and Wenfu Xu (2019), [A Novel Calibration Algorithm for Cable-Driven Parallel Robots with Application to Rehabilitation](https://www.mdpi.com/2076-3417/9/11/2182), DOI [10.3390/app9112182](https://doi.org/10.3390/app9112182). Publisher abstract supports redundant internal encoder-length mapping and simulation validation; it does not prove fewer points in hp-sim5. Direct page later returned 429; indexed publisher abstract was available.
- Julien Alexandre dit Sandretto, David Daney and Marc Gouttefarde (2012 preprint; 2013 proceedings), [Calibration of a Fully-Constrained Parallel Cable-Driven Robot](https://www-sop.inria.fr/coprin/PDF/a_romansy2012.pdf), DOI [10.1007/978-3-7091-1379-0_10](https://doi.org/10.1007/978-3-7091-1379-0_10). Full primary PDF supports joint pose/geometry fitting, reference-frame constraints, and validation on separate configurations; its massless/nonelastic assumptions constrain transfer.
- Yier Wu, Alexandr Klimchik, Anatol Pashkevich, Stéphane Caro and Benoît Furet (2013), [Efficiency Improvement of Measurement Pose Selection Techniques in Robot Calibration](https://arxiv.org/abs/1311.6658), DOI [10.48550/arXiv.1311.6658](https://doi.org/10.48550/arXiv.1311.6658). Primary abstract supports test-pose experiment-design motivation; serial-manipulator simulations are not CDPR acquisition evidence.
- M. R. Dindarloo, A. S. Mirjalili, S. A. Khalilpour, Rooholla Khorrambakht, Stephan Weiss and Hamid D. Taghirad (2024), [A Graph-Based Self-Calibration Technique for Cable-Driven Robots with Sagging Cable](https://doi.org/10.1109/IROS58592.2024.10802627); [authors' implementation](https://github.com/MohammadrezaDindarloo/ARASFactorSLAC). Primary abstract and repository motivate a joint kineto-static/catenary graph; internal sensors are not automatically encoder-only, and its FE evidence is not hp-sim5 hardware accuracy.
- [Prior ranking-negative study](../b7deb1bcc9cb418388273e2b7b9adc8c/report.md): score removal, alternative objectives and bounded-update failures informed rejection thresholds.
- [Prior candidate-model validation](../fdb616a454af4a6d9c1db074fbdc207d/report.md): own-model validity and compensation ambiguities informed E02's retained validity gate.
- [Prior persistent JAX cache study](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/report.md): compilation-cache change already in baseline, distinct from E04–E06 within-pass reuse.
- [Prior guarded-settling/thinning study](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md): earlier acquisition results; its replay/audit helpers were adapted and E00 uses its archived histories. Prior measured settling gains are not new results from this session.
- [Reference fixture provenance](../../../../autocal/data/references/README.md), [fixture commands/truth/comparator](../../../../autocal/tools/regress_calibration_logs.py): browser-simulation origin, effective truth, registered settings and regression metrics.
- [Autocal README](../../../../autocal/README.md), [optimization overview](../../../../autocal/optimization_algorithm_overview), [objective overview](../../../../autocal/objective_functions_overview), [ellipse guide](../../../../autocal/README_elliptical_feature_calibration.md): source-tracing starting points and workflow contract.
- [Research tools](../../../../research/README.md), [integration rationale](../../../../research/investigation.md), [native collection contract](../../../../research/native-collection.md), [topic guide](../../../../research/catalog.md): world identity, simulation clocks, evidence limits and prior-report discovery. Native services were not moved by this investigation.
- [Baseline revision and frozen hashes](manifest.json), [E01 replay manifests](E01-baseline-runs.json), [E05 candidate manifests](E05-cache-runs.json): reproducible source/data/environment and actually tested commands. All failed selection trials and raw logs remain in the assigned stable session directory.
