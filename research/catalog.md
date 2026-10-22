# Research topic guide

These summaries describe archived evidence, not new experiments or a statement about the current solver. They were compiled on 2026-10-08 from the reports and notebooks linked below. Raw session artifacts are ignored by Git and may be absent in another checkout. The [local catalog](../output/research/index.md) also lists integration checks, incomplete sessions and their status.

Four hash-named sessions contain substantive autocal work: three have final reports and one has an unfinished notebook. Missing research.md does not imply that no research happened.

## Configurable continuing HP3/HP4 worlds through MCP

Status: complete, verified 2026-10-12. Topics: MCP, HP3, HP4, configuration,
layering, reset, firmware, encoder origin, source reload.

Fresh MCP configuration now validates and freezes stock HP3/HP4 settings, bakes
layering-off geometry before construction and restarts firmware/collector
references coherently. A real six-point HP3 collection with 30 mm radii and zero
buildup showed zero baked cable half-width and about 183° encoder travel on each
moving axis. Lifecycle and production-construction checks passed. An initial
zero-step failure is preserved; no calibration accuracy or hardware result was
measured. Custom anchor layouts and arbitrary layered designs remain unsupported.

[Verification, API limits and full evidence](configurable-worlds.md).

## Default-force HP3 settling and no-layering geometry correction

Status: complete, verified 2026-10-10. Topics: autocalibration, HP3, settling, no layering, scene baking, MCP, reset settings.

The reported pullout failure was reproduced exactly through 2,457 encoder readings. Bounding endpoint search, then holding the drive and requiring ordinary measurement settling, recovered collection. Review of commit 383c6d4 found that unset layering enabled layered physics while job status reported it disabled; explicit flags, matching zero-width baking and reset settings were corrected. A production default-force MCP job then accepted automatically after seven iterations / 360 points, with 2.604 mm applied-anchor RMSE. Independent inspection verified every measurement, 300,551 physical samples, the physical stop and a finalized 1.58 GB RRD. Checks and two unchanged HP3 replay regressions passed. This is one full simulated default-force trial, not evidence of fewer points or hardware transfer; candidate ranking remains unchanged.

- [Settling fix, commit review and commands](autocal-settling-review.md).
- [Updated session abstract and tested hypotheses](../output/research/sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/abstract.md).
- [Acceptance and geometric accuracy](../output/research/sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E12-acceptance.json).

## Supervisor-owned full-autocal through restricted MCP clients

Status: archived tooling validation; that trial's calibration quality was concerning. See the correction above.

Topics: autocalibration, HP3, HP4, MCP, sandbox, supervisor, cancellation, process groups, Rerun.

A socket-denied SDK MCP client started an isolated HP3 production CLI job in 3 ms. With known 30 mm radii, zero buildup, layering requested disabled and explicit manual forces, it collected eight sweeps / 320 physical measurements and automatically accepted at iteration six through the default patience rule. The finalized RRD contains 209,908 physical samples and exact dataset/log/stage artifacts; the attached HP4 world stayed at step zero. Physical cancellation, HTTP/SIGTERM supervisor exit, owner SIGKILL and physics-budget exhaustion retained readable partial recordings. The preceding default-autotuning trial failed pullout settling and is preserved. The selected score was concerning (6018), so this establishes workflow access and evidence integrity, not calibration accuracy. The follow-up above found that physical layering was actually enabled. Direct Codex chat attachment was unavailable; restricted SDK stdio MCP was exercised.

- [Verification and exact commands](autocal-jobs-verification.md).
- [Session abstract and tested hypotheses](../output/research/sessions/2026-10-10-supervised-full-autocal/abstract.md).
- [Detailed report and retained artifacts](../output/research/sessions/2026-10-10-supervised-full-autocal/report.md).

## Faster collection with guarded encoder settling

Status: complete. Topics: autocalibration, encoder-only, HP4, settling, collection time, point thinning.

Fresh replays reproduce 20 ranking/physical-error disagreements. Halving points worsens anchors on three of four fixtures. A small collector change checks the recent quiet window with a tighter range limit, retaining the longer vibration fallback. Matched 36-point production-JS/RRF collection takes 320.364 versus 429.668 simulation seconds (25.4% less); all eleven recorded calibration regressions and stopping decisions remain unchanged. New-data anchor accuracy and hardware transfer remain unverified.

[Report, measurements and limitations](../output/research/sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md).

## Autocal history ranking with held-out sweeps

Status: complete.

Topics: autocalibration, HP4, HP3, Slideprinter, candidate ranking, held-out sweeps, D-optimal selection, outliers, poor starts.

This investigation reproduced disagreement between calibration ranking and known physical geometry, then tested synthetic sweep selection and whole-sweep prediction. On seventh HP3, the selected fit had 100.737 mm more summed anchor error than another candidate. A synthetic held-out scorer separated exact and perturbed geometry by 40.55x in robust prediction cost, but poor starts still produced physically bad fits. The archived implementation added future-sweep history selection with the existing rank as fallback. Its static regression replays did not measure an accuracy gain from that gate. Native HP4 collection was cancelled before a complete sweep after tracking losses. The report supports ranking diagnostics and a synthetic discriminator; it does not establish reliable calibration accuracy.

[Abstract and tested hypotheses](../output/research/sessions/autocal-held-out-history-ranking-7ec4260f/abstract.md).

- [Archived implementation, measurements and limits](../output/research/sessions/7ec4260f7410499a96f6b6bfe9c3de59/report.md)
- [Hypotheses, E0–E3 decisions and retained collection path](../output/research/sessions/7ec4260f7410499a96f6b6bfe9c3de59/research.md)

## Autocal ranking, bounded updates and identifiability

Status: complete.

Topics: autocalibration, HP3, Slideprinter, history age bonus, held-out prediction, bounded updates, D-optimal movements, radius uncertainty, negative results.

Seven hypotheses were investigated using recorded histories, controlled replay, synthetic bounded updates and native radius sensitivity. Ranking disagreed with physical accuracy in 22 adjacent transitions across seven of eleven datasets. Whole-sweep prediction selected worse physical results on five datasets. Removing the iteration-age bonus improved one case but increased aggregate selected parameter error by 200.701 mm over the full matrix. Bounded residual updates and informative movement selection worked in some synthetic conditions and failed under others. No tested solver or selection patch met the acceptance criteria. The retained change added ranking/ground-truth diagnostics. This session has a detailed report and experiment registry despite lacking a research.md notebook; online collection accuracy remained unvalidated.

[Abstract and tested hypotheses](../output/research/sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/abstract.md).

- [Seven hypotheses, accuracy criteria and negative findings](../output/research/sessions/b7deb1bcc9cb418388273e2b7b9adc8c/report.md)
- [Experiment inputs and log paths](../output/research/sessions/b7deb1bcc9cb418388273e2b7b9adc8c/experiment_registry.json)
- [Frozen source, dataset and evidence hashes](../output/research/sessions/b7deb1bcc9cb418388273e2b7b9adc8c/artifact_hashes.json)

## Autocal candidate-model validation and collection throughput

Status: complete.

Topics: autocalibration, HP3, HP4, candidate-owned spool models, winding mismatch, elasticity, bounded updates, D-optimal movement, sentinel rejection, headless JS, Python parity, Rerun throughput.

This investigation tested candidate-specific spool validation, local bounded adjustments and informative movements on recorded HP3 and synthetic data. A guarded validation veto and tie-breaker rejected impossible candidates in 15/15 deliberate counterexamples, while all eleven recorded final selections and replay counts remained unchanged. Bounded updates recovered clean synthetic geometry but failed under winding mismatch and poor recorded starts; no HP3 accuracy gain was shown. A real HP4 collector run completed twelve observations in 1693.12 seconds without establishing anchor accuracy. A tooling follow-up measured a 19.03x improvement for matched short recorded HP4 workloads using headless JS. Frozen firmware replay retained encoder parity within 0.002242 degrees. Fresh end-to-end collection throughput and hardware accuracy were not measured.

[Abstract and tested hypotheses](../output/research/sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/abstract.md).

- [Guarded validation and performance follow-up with limitations](../output/research/sessions/fdb616a454af4a6d9c1db074fbdc207d/report.md)
- [H1–H4, E01–E07 and P01–P04 budgets and decisions](../output/research/sessions/fdb616a454af4a6d9c1db074fbdc207d/research.md)
- [Separate physical and prediction errors](../output/research/sessions/fdb616a454af4a6d9c1db074fbdc207d/prediction-metrics.json)
- [Benchmark method, timings and reproducible replay commands](../research/collection-performance.md)

## Autocal history-ranking baseline, unfinished record

Status: incomplete record; final research outcome not documented.

Topics: autocalibration, Slideprinter, history ranking, ground-truth regret, proposed held-out validation, HP4 collection.

The notebook records a baseline audit of historical Slideprinter histories. Repeated ten_points_bigger_deltas runs selected physically worse candidates with 10.96–14.90 mm parameter regret. It proposes whole-sweep prediction and bounded residual updates, and records a native HP4 collection job plus a running regression baseline. The last saved decision still awaits those results. There is no final report or documented completion of the proposed selector experiments. Although exit.json records a successful process exit, that does not resolve the unfinished research record. Treat this as useful baseline evidence and a record of intended experiments, rather than a completed assessment of a new algorithm.

[Abstract and tested hypotheses](../output/research/sessions/autocal-history-ranking-baseline-incomplete-a71aec03/abstract.md).

- [Saved hypotheses, baseline measurements and outstanding decisions](../output/research/sessions/a71aec03021f470796a789ba7382e28a/research.md)

## Why the fifth HP3 reference trajectory differs today

Status: complete.

Topics: autocalibration, HP3, historical reference, numerical environment, sentinel fits, regression metrics.

Isolated March and current source replays used the same present-day numerical environment. They produced identical intermediate fits, both differing from the historical trajectory by 390.788 mm in the diagnostic mean. March code still returned the saved 70.314 mm calibration; current history selection returned 70.787 mm. The large trajectory penalty is dominated by underconstrained attempts that are excluded from final selection. Subsequent tracked algorithm changes are therefore unnecessary to reproduce the trajectory difference. Numerical environment or execution differences remain a leading explanation, but the original runtime lacked enough provenance to establish the cause. Source rollback alone did not restore the historical trajectory.

[Abstract and tested hypotheses](../output/research/fifth-hp3-reference-audit/abstract.md).

- [Metric decomposition, source history and replay controls](../output/research/fifth-hp3-reference-audit/report.md)
- [Controlled comparison measurements](../output/research/fifth-hp3-reference-audit/results.json)
- [Preserved historical settings](../output/research/fifth-hp3-reference-audit/reference-settings.json)

## Future-sweep selection versus refreshed autocal references

Status: complete.

Topics: autocalibration, HP3, Slideprinter, whole-sweep prediction, history selection, regression references, concurrency.

Eleven new main-branch reference logs were generated with source and environment provenance, then reproduced exactly in a bounded two-worker regression run. Against that baseline, research_findings_1 changed final history selection while preserving every intermediate fit. Summed returned parameter error fell from 885.158 to 816.936 mm, a 7.71% reduction. Three datasets improved, five worsened and three were unchanged. This is a net gain on this battery with explicit regressions, not a uniform accuracy improvement. Candidates were evaluated on different future-sweep sets. Earlier full-concurrency runs suffered native crashes; the successful bounded runs do not establish a root-cause fix or guaranteed crash prevention.

[Abstract and tested hypotheses](../output/research/reference-refresh-october-7-2026/abstract.md).

- [Per-dataset errors, selections and validation-set limitations](../output/research/reference-refresh-october-7-2026/report.md)
- [Machine-readable branch comparison](../output/research/reference-refresh-october-7-2026/comparison.json)
- [Source, runtime and scheduling controls](../output/research/reference-refresh-october-7-2026/provenance.json)

## Autocal native crash isolation to the compiler path

Status: investigation incomplete; no verified production fix.

Topics: autocalibration, JAX, jaxlib, XLA, protobuf, SIGSEGV, SIGABRT, compiler lifetime, Valgrind.

Native crashes were reproduced in objective replay outside the sandbox and in compile-only runs without objective execution. A direct jaxlib replay of saved compiler modules reproduced two segfaults among eleven children without importing JAX or autocal. GDB observed a crash during protobuf object destruction inside the XLA compilation path, but did not locate the first invalid write or free. Small reduced controls passed; they are finite controls rather than fixes. Valgrind timed out with uninitialized-value reports, and the sanitizer build failed to produce a usable wheel. No production fix was established. These observations narrow the necessary trigger set toward native compilation and object lifetime.

[Abstract and tested hypotheses](../output/research/autocal-crash-fix-20261007/abstract.md).

- [Isolation decisions and budgets](../output/research/autocal-crash-fix-20261007/research.md)
- [Tracked findings and compiler reduction results](../research/bug-reports/autocal-native-crashes-october-7-2026/autocal-crash-compiler-update.md)

## HP4 Print Squares substeps and PR 77

Status: partial physics correction validated, 2026-10-21. Topics: HP4, Print Squares,
missed steps, open-loop torque balance, rotor damping, physics substeps.

Removing cable-only electromagnetic stiffness corrects artificial holding power
and passes the default-one-substep stability gate. The PR's damping correction
fails loaded-state settling and is deferred. Matched native Squares prefixes
still pull B out at finer steps; coarse zero-loss behavior is unreliable evidence
of accuracy. [Review, measurements and integration decision](hp4-squares-substeps.md).

## Autocal flag audit and experiment handoff

Status: static investigation complete, 2026-10-22; numerical experiments not run.
Topics: autocalibration, CLI flags, robust filtering, scale polish, solver budgets,
parameter interactions, ranking, experiment design.

[Assessment and bounded experiments](autocal-flag-audit/report.md),
[92-option inventory](autocal-flag-audit/inventory.md),
[abstract](autocal-flag-audit/abstract.md), and
[notebook](autocal-flag-audit/research.md). Preserves the user's KEEP annotations;
no numerical default was established as optimal and no runtime flags were removed.

The [six-lane launch index](autocal-flag-audit/handoffs/README.md) provides separate
parallel handoffs with ownership, budgets, output isolation and integration rules.

## HP3 nominal truth and lane 01 solver integration

Status: complete, 2026-10-22. Topics: effective anchors, simulation truth, solver
tracing, iteration budgets, TRF fallback. [Integration report](autocal-solver-lane01-integration.md).
Historical/current HP3 geometry agrees with the existing nominal truth; poor fit
agreement with an older fitted reference does not establish a scene mismatch.
Compact solver diagnostics and an invalid-primary retry fix preserve default
numerics on five frozen fits and three full replay regressions.

## Autocal Lane 02 integration: unused fit controls

Status: complete, 2026-10-22. Topics: autocalibration, filtering, noise normalization,
CLI cleanup, old stage artifacts, reference provenance and consensus.

[Integration report](autocal-filtering-lane02-integration.md): removes unused
anchor-fit threshold and misleading sigma-used override; retains numerical
defaults and old artifact replay. Four bounded fits and three full fixture
regressions preserve default results; 267 targeted tests pass. The alleged layered
fallback regression compared an individual fit with a selected final consensus.
[Session abstract](../output/research/sessions/2026-10-22-autocal-filtering-integration/abstract.md)
and [notebook](../output/research/sessions/2026-10-22-autocal-filtering-integration/research.md)
retain raw evidence and limitations.
