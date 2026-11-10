# Faster repeated encoder autocalibration with persistent JAX compilation

Status: complete.

## Abstract
This study investigated encoder-only CDPR calibration using eleven fresh recorded-data replays, synthetic circle sweeps, and solver profiling. Baseline scoring disagrees with physical error in 20 adjacent transitions across nine fixtures. Seventh HP3 selects anchors with 15.23 mm more RMSE than its best fitted candidate; nine initial fits require underconstrained recovery and six recoveries fail. Numerical-scale caching, shape padding, corrected padded arithmetic and a blanket gradient fallback were tested; none justified a default solver change. The implemented improvement automatically stores compiled CPU objectives across launches. Isolated repeated runs reduce wall time from 56.77 to 50.86 s on bad-points Slideprinter (10.4%) and 43.29 to 34.73 s on seventh HP3 (19.8%). All eleven calibration regressions and 300 tests pass, with identical outputs and stopping behavior. Evidence uses simulated fixture truth and analytical synthetic data, not hardware. First-time compilation remains slow, and candidate-ranking accuracy is unchanged.

## Topics
encoder-only autocalibration, CDPR, HP3, HP4, Slideprinter, persistent JAX compilation cache, speed, physical ranking, negative results

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Current ranking agrees with known physical anchors | E01 | Rejected: 20 adjacent disagreements; substantial selected-anchor regret | [Fresh audit](E01-baseline-audit.json), [fragility](E01-fragility.json) |
| Dynamic numerical scales speed current recorded fits | E02–E04 | Mixed: synthetic compilation drops 3 to 1; scales constant on recorded runs | [Synthetic values/times](E02-compilation.json), [equivalence](E02-equivalence.json), [11 recorded controls](E07-candidate-comparison.json) |
| Sweep and point shape padding preserves gradients | E05/E06 | Rejected on tested synthetic data | [Sweep/point failures](E05-bucket-tests-expanded.txt), [point-only failures](E06-pointbucket-tests.txt) |
| Finite padded arithmetic plus bucketing preserves physical fits | E07 | Rejected as accuracy-neutral: five-points RMSE 2.848 → 3.716 mm | [Counterexample](E07-safe-comparison.json), [passing synthetic checks](E07-safe-tests.txt) |
| All padded exact gradients are unusable | E07–E09 | Rejected: synthetic counterexamples, but all 740 recorded padded calls finite | [Synthetic failure](E07-baseline-mixed-points.json), [recorded profile](baseline_profile_a/layered_manual_some_bad_points/compile-stats.json) |
| Blanket padded-gradient fallback improves the general default | E08 | Mixed; rejected default: bad-points gain, seventh HP3 worse | [All 11 comparisons](E07-guard-comparison.json) |
| Persistent compilation caching speeds repeated launches while preserving calibration | E10 | Supported on measured replay pair; 11/11 regressions preserved | [Timings](E10-controlled-timing.json), [regressions](E10-final-regressions.json), [raw event checks](E10-raw-events-comparison.json) |
| Unavailable cache allows computation to finish | E10 | Supported tool check, not calibration-accuracy evidence | [Check](E10-unavailable-cache.txt) |
| Joint latent-pose fitting, nuisance-aware movement selection, elasticity identification and uncertainty stopping improve accuracy | None in this session | Not tested; proposed follow-up | [Ideas and constraints](research.md) |

## References
- [Full report](report.md): measurements, method, limitations and literature.
- [Notebook](research.md): decisions, budgets and accepted steering.
- [Experiment registry](experiment-registry.json): completed, failed and interrupted replays and commands.
- [Source/dataset manifest](manifest.json): revision, fixture truth, hashes and versions.
- [Final source diff](final-source.diff): the retained small change.
- [Previous model-validation research](../autocal-candidate-model-validation-and-throughput-fdb616a4/report.md): prior warm-start and own-model-ranking failures.
- [Previous negative ranking research](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md): prior ambiguity and age-score failures.

## Retained change and its scope

`autocal/ellipse_objective_jax.py` supplies default `JAX_COMPILATION_CACHE_DIR=output/autocal-jax-cache` and `JAX_PERSISTENT_CACHE_MIN_COMPILE_TIME_SECS=0` before importing JAX. Existing environment settings take priority. `autocal/README.md` documents cache location, deletion and `JAX_ENABLE_COMPILATION_CACHE=false`. The source change is seven added lines; the README adds six. No fitted objective, gradient, filtering policy, warm start, collector, planning score, acceptance score or stopping threshold was changed. No new dependency was added.

The existing one-click entry point continues to bootstrap, fit, collect and stop automatically. Native movement was not warranted for a compilation-cache change: every recorded replay used existing datasets with `--sim --no-collect`, and no native motor commands were issued. Collection count is unchanged.

## Encoder-only ideas and current algorithm

For an ideal point robot, squared distances satisfy `(r_i + delta_l_it)^2 = ||p_t - a_i||^2`. Redundant cables give constraints between encoder observations and unknown anchors/poses. Translation, rotation and reflection require gauge conventions. Bare encoder angles also need some known metric parameter to resolve physical scale; known spool geometry, winding dimensions, bounds and priors contribute assumptions in the current solver. These are first-principles identifiability considerations, not measured hardware results.

Ideas formed before the prior-report search were: eliminate latent poses with constrained circle/ellipse geometry; fit latent poses jointly with anchors; validate on whole unseen sweeps and require complete prediction coverage; use fixed robust residual scales and bounded proposals; fit spool/elasticity nuisances with uncertainty diagnostics; choose movements using information projected over nuisance parameters; warm-start repeated fits; and reuse compilation. The first ellipse idea is already the production approach. Prior sessions found adverse warm starts, primary own-model prediction, removed age bonuses and bounded updates, so this session did not repeat those as default candidates.

The existing loop fits anchors repeatedly, optionally alternates spool fitting across four filter passes, ranks movements by D-optimal gain, and controls patience with history rank. Final acceptance compares saved anchors on later sweeps and retains a candidate-owned prediction veto/tie-breaker. Neither residual score nor information rank establishes physical accuracy. I read the current README, algorithm/objective overviews, detailed guide, pointwise solver, JAX packing/objective, regression code and prior reports before choosing the retained change.

## Reproduction and measurement contract

Baseline revision is `be1111d7ecbf5dfa6722b25e887d52006b4f6ef8`; the initial worktree was clean. All Python commands use `.venv/bin/python` (Python 3.12, JAX/jaxlib 0.9.2). [Manifest](manifest.json) records exact versions, dataset hashes, fixture truth and changed-source hashes. [Registry](experiment-registry.json) records actual commands, process status and log paths. [Replay driver](replay.py) copies every dataset into an isolated trial directory. The snapshot modules preserve rejected changes.

Anchor accuracy is RMS Euclidean distance per anchor to the declared regression fixture truth, in mm; radius accuracy is RMS mm per spool. Fixture truth is simulated geometry, not independent hardware metrology. Logged coordinates are rounded, so very small errors are approximate. The older diagnostic `anchor distance sum + 2π radius absolute-error sum` is reported separately in the audit and never substituted for anchor RMSE.

Fresh baseline runs take 31.84–63.11 s (median 50.58 s) and consume 64 replay iterations. These matrix timings overlap exploratory jobs and are not controlled speed estimates. Nine initial fits need underconstrained recovery, six fail recovery, and all eleven CLI processes still complete. [Fragility evidence](E01-fragility.json) separates these outcomes.

The ranking audit finds 20 adjacent score/true-error disagreements across nine fixtures. Seventh HP3's selected anchor RMSE is 55.812 mm, versus 40.580 mm for the best candidate encountered. April HP3 is 64.416 versus 26.775 mm. Different held-out folds and spool/elasticity compensation remain plausible mechanisms; this session does not show that a new selection score fixes them. [Raw audit](E01-baseline-audit.json).

## Measured final accuracy and stopping

| Dataset | Anchor RMSE mm | Radius RMSE mm | Replay iterations | Cache-enabled result |
| --- | ---: | ---: | ---: | --- |
| five_points_bigger_deltas | 2.848 | 0.0140 | 6 | Identical |
| flexible_lines_2000 | 9.660 | 0.0440 | 4 | Identical |
| layered_manual_some_bad_points | 6.229 | 0.0760 | 7 | Identical |
| ten_points_bigger_deltas | 5.930 | 0.0360 | 6 | Identical |
| ten_points_w_buildup | 2.301 | 0.0940 | 5 | Identical |
| ten_points_w_buildup2 | 2.088 | 0.0240 | 7 | Identical |
| fifth_hp3_dataset | 18.471 | 0.1845 | 7 | Identical |
| seventh_hp3_dataset | 55.812 | 0.1745 | 5 | Identical |
| sixteenth_hp3_dataset_even_more_pressure | 14.339 | 0.1845 | 6 | Identical |
| nineteenth_hp3_dataset_even_more_pressure | 42.130 | 0.1845 | 5 | Identical |
| 23_april_2026 | 64.416 | 0.1845 | 6 | Identical |

The existing regression comparator passes 11/11 at 0.01 mm parameter-distance tolerance. Selected anchor/radius outputs, per-iteration scores, iteration counts and stop reasons are identical. Beyond rounded logs, all 5,184 non-timing numeric fields in raw JSONL events match exactly; decisions also match. Only recovery elapsed-time fields differ. [Regressions](E10-final-regressions.txt), [raw event comparison](E10-raw-events-comparison.json), [audit script](event_audit.py). All 300 autocal tests pass with no warnings in 11.09 s: [test log](final-tests-unbounded.txt).

## Controlled repeated-launch speed

After every competing calibration job finished, [controlled_timing.py](controlled_timing.py) ran two fresh CLI processes with caching disabled, then two fresh processes against the populated cache. Threads, data, flags, models and fitting budgets match. Both configurations request compiler diagnostics. One paired run per fixture was measured; no confidence interval is claimed. The warm cache was populated by complete prior fits. This comparison measures repeated-launch performance rather than first-run acceleration.

| Fixture | Cache disabled s | Populated cache s | Reduction |
| --- | ---: | ---: | ---: |
| Bad-points Slideprinter | 56.773 | 50.864 | 10.4% |
| Seventh HP3 | 43.285 | 34.732 | 19.8% |

The dataset, fitted output, replay count and stop reason are unchanged in both pairs. [Measured times](E10-controlled-timing.json), [disabled regression](E07-cache_actual_disabled-comparison.json), [warm regression](E07-cache_actual_repeat-comparison.json). The final session cache uses about 38 MiB. JAX 0.9.2 emits executable-deserialization warnings in the warm-run logs; the numerical comparisons pass. First launch, changed graph shapes/settings and cache misses still require compilation. The cache does not bound solver work or improve anchor accuracy.

![Ranking reproduction and isolated cache timings](ranking-and-cache.png)

Export: [scientific figure PDF](ranking-and-cache.pdf).

## Disproved and failed trials

- Dynamic noise-floor argument: all twelve synthetic value/gradient comparisons are bit-identical and compiled variants drop three to one, but recorded scales stay constant; eleven full replays preserve results without establishing a speed gain. Retained as `candidate/`, E02 and E03/E04 logs.
- Sweep/point bucketing: produces nonfinite gradients in tested synthetic cases. The completed bad-points trial is preserved in `bucket/`; the subsequent trial was interrupted and is not an accuracy result.
- Point-only bucketing: also fails finite-gradient tests. Its recorded fit was interrupted; no completed accuracy claim. `pointbucket/` and E06.
- Correcting padded arithmetic plus point bucketing: ten synthetic checks pass, but five-points anchor RMSE worsens 2.848 → 3.716 mm. Two completed replays and partial cancellation evidence remain in `safe/`; no general default change.
- Blanket padded-gradient fallback: completes eleven replays, improves bad-points RMSE 6.229 → 1.839 mm and uses one fewer iteration, but worsens seventh HP3 55.812 → 56.979 mm. Reject as the default. Original recorded profile has zero nonfinite gradients in 33,115 calls, including 740 padded calls: synthetic failures were conditional, not universal. `guard/` and E08/E09.
- Bounded persistent cache: the attempted 256 MiB bound needs the missing optional `filelock` package. Tests still pass while caching warns/disables itself. Installation was unavailable through restricted DNS. The bound was removed, with no dependency change; failed bounded-cache logs remain in `cache_actual/` and `final-tests.txt`.
- Early test stimulus assumed a changing floor must change a loss even when the residual scale dominates. That assertion was invalid; its failures are preserved in E04 and do not count as a calibration failure. Rejected tests are saved in [rejected-tests.diff](rejected-tests.diff).

Partial/planned experiments are listed as interrupted or not tested in the registry; they are not negative accuracy results. Failed variants are not in production.

## References and remaining decisions

- Julien Alexandre dit Sandretto, David Daney and Marc Gouttefarde (2012), [Calibration of a Fully-Constrained Parallel Cable-Driven Robot](https://www-sop.inria.fr/coprin/PDF/a_romansy2012.pdf). Supports redundant proprioceptive calibration under cable assumptions; not proof of Hangprinter encoder-only accuracy.
- Sen Qian, Kunlong Bao, Bin Zi and Ning Wang (2018), [Kinematic Calibration of a Cable-Driven Parallel Robot for 3D Printing](https://www.mdpi.com/1424-8220/18/9/2898), DOI [10.3390/s18092898](https://doi.org/10.3390/s18092898). Supports measurement-configuration optimization; its hardware validation uses external motion capture.
- Filippo Zoffoli, Edoardo Idà and Marco Carricato (2025), [A comparative analysis of initial-pose self-calibration algorithms for underactuated cable-driven parallel robots](https://www.cambridge.org/core/journals/robotica/article/comparative-analysis-of-initialpose-selfcalibration-algorithms-for-underactuated-cabledriven-parallel-robots/CD2FD0CE34646B8809C4248A7D81D6F6). Supports distinguishing geometric/statics and extra-sensor initial-pose assumptions from pure encoder anchor calibration.
- JAX authors, [JIT compilation](https://docs.jax.dev/en/latest/jit-compilation.html) and [persistent compilation cache](https://docs.jax.dev/en/latest/persistent_compilation_cache.html). Supports static-argument recompilation and reuse of executables keyed by graph, compiler version and hardware; measured autocal speed comes from E10, not generic documentation claims.
- [Current README](../../../../autocal/README.md), [algorithm overview](../../../../autocal/optimization_algorithm_overview), [objective overview](../../../../autocal/objective_functions_overview), [detailed guide](../../../../autocal/README_elliptical_feature_calibration.md) at baseline revision above. Supports the existing solver, selection and automatic-stop description.
- [Native research tools](../../../../research/README.md) and [integration rationale](../../../../research/investigation.md). Supports choosing offline numerical evidence and separating process/tool checks from physical calibration accuracy.
- Previous sessions linked in the abstract support the rejected warm-start/selection directions. Current E01–E10 evidence, input manifest, final diff and scripts are linked above.

The best next accuracy experiment is a deliberately separate train/validation/test sweep design that includes spool/elasticity nuisance identifiability, with informative configurations tested against physical fixture truth. It is proposed, not tested here. No native accuracy, hardware transfer, first-click speed gain or corrected physical ranking is claimed.

## Follow-up: simulator usefulness, trust and tooling

The user asked whether Rerun, the Python simulator or browser simulator provided useful, trustworthy data. None was directly operated in this session. Running Python autocal fits and analytical circle generators is not running the Python physics simulator. Recorded fixtures were reused; their collection provenance was not independently re-established. This was sufficient for the retained compilation-cache change: the same inputs produced exactly equal non-timing numerical events and identical stop decisions. It is insufficient to establish fresh collection performance, hardware accuracy or physical validity of an improved calibration model. The physical-error comparisons use the regression suite's declared fixture anchors and coordinate conventions, not new metrology.

The synthetic sweeps helped expose conditional nonfinite gradients and reject padding ideas. Their limits were demonstrated in this session: all 740 padded gradient calls in the profiled recorded fit were finite, despite synthetic counterexamples. Synthetic failure is evidence about those tested conditions, not proof of a common failure in realistic collection. No new simulator or Rerun experiment has been added by this follow-up.

Two preceding tooling reports provide relevant evidence, separately from this session:

- [Collection performance, 2026-10-07](../../../../research/collection-performance.md): the frozen firmware schedule produced 1,120 encoder reads with maximum Python/production-JavaScript difference 0.002242 degrees, below the 0.01-degree tolerance. The matched HP4 recording stepping benchmark was 19.03 times faster in headless JavaScript. This supports numerical backend agreement under that schedule, not hardware accuracy or fresh end-to-end collection equivalence. Recording observation logging took 27.104 seconds versus 1.562 seconds for the numeric replay; optional recording could reduce experiment cost.
- [Native collector verification, 2026-10-06](../../../../research/native-collection-verification.md): native collection and finalized Rerun readback/live inspection worked. An earlier stationary sensor nevertheless had a manifest marked complete; response checks were subsequently added. A short fit reported optimizer success with cost 100 and zero valid sweeps. Firmware anchors were guesses, and distributed pulley geometry did not define a unique equivalent point-anchor truth. These results illustrate why successful transport, a readable recording and optimizer success cannot by themselves certify calibration accuracy.

Proposed tooling improvements, not tested or implemented here:

1. Put a truth/provenance contract in each dataset: scene and loaded-code hashes, encoder reference conventions, units, spool/winding settings, cable properties, coordinate gauge, and the definition or absence of effective point-anchor truth. Report held-out position/length error when the physics geometry differs from the solver model.
2. Provide one bounded reset–collect–fit–validate workflow with the production JavaScript backend for fast fresh collection, Python differential replay, fixed-step/sampling metadata and saved manifests. Include independent validation movements so collector success and fitted accuracy have separate outcomes.
3. Surface data-quality and solver-health summaries automatically: sensor travel per direction, valid sweeps, failed recovery, nonfinite-gradient counts, model mismatch and stop reason. Existing response checks should remain part of the workflow. Never reduce a result to process exit code or optimizer success.
4. Generate a synchronized Rerun diagnostic view of commanded motion, actual encoder travel, tension/stretch, tracking error and fit residuals. Use numeric comparisons for pass/fail; the viewer helps explain failures but is not independent measurement truth. Save selected intervals or failed trials, with explicit recording cadence, to reduce logging overhead.
5. Make the offline comparator reusable: preserved raw datasets, baseline/variant event comparison, compilation/cache-hit telemetry and repeated serial paired timing. This session needed custom scripts; such tooling would have exposed the synthetic/recorded gradient difference sooner and reduced unnecessary full replays.

Fresh simulator accuracy testing remains proposed. It was not blocked by a demonstrated tool failure in this session; it was not pursued after the supported change became a computation-only optimization. That leaves the original accuracy and physical collection questions open.

### Clarification: existing browser fixture definitions

The user clarified that all reference fixtures were collected by `autocal/autocal.py --sim ...` without an explicit dataset, using browser physics simulation. The source and fixture configuration already contain much of the information proposed above. In particular, [regression truth constants](../../../../autocal/tools/regress_calibration_logs.py) declare 2D/3D anchors and effective radii; representative fixture JSON stores angle/length units, firmware M666 settings, mm/degree conversion and force/noise settings. [Slideprinter scene](../../../../public/usd_scenes/slideprinter.usda) and [HP3 scene](../../../../public/usd_scenes/hp3_rigid_body.usda) define geometry and physical settings, while the [browser encoder endpoint](../../../../hp-sim/app/appBootstrap.js) reads mapped spool orientation in degrees. These files demonstrate available definitions; this follow-up does not assert that today's default scene is the exact historical generating scene of every fixture.

Recommendation 1 is therefore narrowed to automatically recording the association between each dataset and its generating revision/scene/runtime overrides, reusing metadata already saved. It does not request additional sensors, hardware measurements or manually duplicating all simulator configuration. The existing source and declared fixture truth are usable for simulation regression research. Lack of automatic linkage is not evidence of inaccurate browser data, and the prior native distributed-pulley caveat must not be generalized to every historical browser fixture. No new accuracy result is claimed by this clarification.

### Minimal provenance documentation implemented

Added one small [README beside the reference fixtures](../../../../autocal/data/references/README.md). It points to existing JSON configuration and expected regression truth, lists the first data commits (following renames), identifies historical config/scene locations and known radius corrections, and distinguishes saved replay commands from original collection commands. The first data commits are source-history pointers, not invented exact clean collection revisions. Existing stored-data fits do not require reconstructing the original browser session. Future fixture additions can record their collection revision and nondefault scene/settings in the commit message. No fixture schema, data, algorithm or collection code changed.

All eleven first-data commit references, local documentation links and cited historical source/config paths were checked successfully; `git diff --check` passes. [History/recorded-override audit](provenance-history.json) is documentation evidence, not a new calibration experiment.
