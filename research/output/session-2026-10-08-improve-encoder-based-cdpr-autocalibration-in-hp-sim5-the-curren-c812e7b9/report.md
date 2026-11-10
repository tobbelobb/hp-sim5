# Faster encoder-only CDPR calibration collection with guarded settling windows

Status: complete.

## Abstract

This study asks whether Hangprinter calibration can collect encoder data faster while preserving its one-click workflow and automatic stopping. Eleven fresh recorded-data replays reproduce 20 score/physical-error disagreements and six failed recoveries. Halving points worsens anchor RMSE on three of four recorded fixtures, despite promising conditional synthetic results. Settling experiments found that old motion was retained in the short quiet check. The implemented change checks recent readings with a tighter range limit while retaining the existing vibration fallback. Matched production-JS/RRF collection from fresh HP4 worlds reduces 36-point collection from 429.668 to 320.364 simulation seconds (25.4%). Wall time falls from 250.220 to 180.898 seconds in this pair. All 36 observations pass residual ingestion; this is throughput evidence, not proof of equal anchor accuracy on newly collected data. Eleven calibration regressions, stopping decisions and 5,184 numerical event fields remain unchanged on recorded inputs. Hardware transfer and generic ranking accuracy remain unverified.

## Topics

CDPR, Hangprinter, HP4, HP3, encoder-only autocalibration, settling window, collection speed, drift guard, fewer points, physical ranking, synthetic data, one-click, automatic stopping

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Current ranking agrees with physical accuracy | E01 | Rejected: 20 adjacent disagreements across 9 fixtures | [Baseline summary](E01-baseline-summary.json), [candidate audit](E01-E01-baseline-audit.json) |
| Halving recorded points preserves robust calibration | E02 | Mixed; rejected as a default (3/4 anchors worsen) | [Physical comparison](E02-comparison.json), [untouched suffix predictions](E02-heldout-comparison.json) |
| A recent quiet window removes unnecessary post-transient waits | E03/E04/E06 | Supported, with premature-return drift limitation | [Controlled traces](E03-synthetic-settling.json), [unguarded native manifest](native/77bb83dffe5c4a74920b2d6ad6e296d6/a4498f9082e44e85b1c1b66c66bd229b/manifest.json), [trace replay](E06-recorded-settling.json) |
| A duration-scaled quiet threshold retains speed while reducing later drift | E06/E08 | Supported in measured simulation; hardware not tested | [Guarded trace replay](E06-recorded-rate-settling.json), [matched collection](E08-native-comparison.json) |
| Sparse clean synthetic success generalizes to recorded spool fits | E02/E05 | Rejected as a general rule; known-radius synthetic results are conditional | [Synthetic fits](E05-synthetic-points.json) |
| Six observations per sweep suffice for the current fitter | E05 | Rejected for the current min_points=10 guard; these fits are invalid | [Synthetic fits](E05-synthetic-points.json) |
| Recorded calibration results and automatic stopping survive the collection change | E09 | Supported on all 11 fixtures | [Regressions](E09-regressions.json), [detailed comparisons](E09-regressions.txt) |
| Joint latent-pose fitting, nuisance-projected information and uncertainty stopping reduce collection count | none | Not tested; proposed follow-up | [Ideas and proposed experiment](research.md) |

## References

- [Full report](report.md): measured outcomes, implementation and limitations.
- [Notebook](research.md): objectives, budgets, experiment decisions and accepted steering.
- [Manifest](manifest.json): revisions, hashes, datasets and runtime versions.
- [Experiment registry](experiment-registry.json): actual trials, failed attempts and evidence paths.
- [Final patch](final-source.diff): retained collector, tests and documentation changes.
- [Prior calibration/cache research](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/report.md): earlier ranking failures and compilation improvements.
- [Prior model-validation research](../fdb616a454af4a6d9c1db074fbdc207d/report.md): whole-sweep and spool ambiguity findings.
- [Prior negative ranking research](../b7deb1bcc9cb418388273e2b7b9adc8c/report.md): bounded updates, age-score and active-selection failures.

## Implemented change

`waitForStableEncoders` in [uncalibrated_actions.mjs](../../../../autocal/control/primitives/uncalibrated_actions.mjs) retained up to the vibration window plus one poll, then used all retained samples in its quiet-range test. A settled reading could therefore wait for old transient samples to expire. The patch uses the recent quiet interval, including the sample at or before its boundary so irregular polling still covers the entire interval. It leaves older samples available to the vibration check.

Simply shortening the history increases the tolerated rate of slow drift. The final patch therefore compares recent range with `toleranceDeg * stableWindowMs / vibrationWindowMs` (using the existing effective window durations). With defaults this is 0.45 degrees across 1.5 seconds; the longer vibration fallback retains its 1.5-degree tolerance. This does not guarantee zero drift or bound all behavior accepted by the vibration fallback. Stationary noise exceeding the tighter quiet range may wait for the longer fallback. Timeout, malformed-response clearing, physical motor roles, noise sampling, collection commands, point count and full-auto patience/acceptance continue through the existing paths. No new user option or manual decision is introduced.

The runtime patch adds 14 lines and removes 5; deterministic tests cover recent-window completion, every-motor coverage, poll jitter, malformed replies and slow drift. The README documents the behavior. No optimizer, selection objective or calibration stopping threshold changed.

## Ideas, literature and current algorithm

The initial idea list was written before searching prior reports: joint latent-position distance closure; physical residuals instead of algebraic ellipse error; diverse/high-information observations; warm starts and numerical reuse; whole-sweep validation/bootstrap stability; robust losses and balanced sweep weights; bidirectional/changed-held-length sweeps; and encoder-based settling or continuous acquisition. See the original ordering and constraints in [research.md](research.md).

For a point robot, `||p_t-a_i|| = l_i0 + delta_l_it`. Redundant cable observations constrain unknown anchor/initial lengths and latent poses. Translation, rotation/reflection and, if spool scale is unknown, uniform scale require gauge or known mechanical information. Encoder-only here means calibration observes motor encoders and uses known/assumed mechanical parameters and force commands; no camera, external position measurement or measured tension was added. Simulation truth enters only evaluation. These are first-principles deductions, not measured accuracy results.

Lau's relative-length work establishes initial-length/pose optimization using relative cable feedback; it does not by itself show recovery of arbitrary unknown Hangprinter anchors and spool scale. Sandretto, Daney and Gouttefarde formulate proprioceptive self-calibration with latent poses and discuss identifiable frames. Borm and Meng motivate observability-based measurement selection. Zoffoli, Idà and Carricato compare acquisition/calibration strategies and discuss settling cost; their extra internal sensors make their results broader than the encoder-only scope here. Full source citations below identify those distinctions.

I read [autocal README](../../../../autocal/README.md), [algorithm overview](../../../../autocal/optimization_algorithm_overview), [objective overview](../../../../autocal/objective_functions_overview), [detailed elliptical guide](../../../../autocal/README_elliptical_feature_calibration.md), the production solver, cost/JAX representation, spool transform, planning pass, validation and collector primitives. The current method already eliminates position along constrained circles via predicted ellipses, uses robust/filter stages, optional alternating spool fitting, D-optimal next-sweep selection and final whole-sweep history validation. Dataset growth normally refits without a previous-iteration warm start. Default patience is three non-improving iterations; the final history choice has a shared anchor comparison plus candidate-owned prediction veto/tie-breaker. These scores are not physical anchor accuracy.

Previous sessions already rejected generic primary own-model ranking, removed age bonuses and bounded warm-start loops under some winding/elasticity or poor-start conditions. The preceding session already implemented persistent JAX compilation caching. Repeating those changes without new model evidence would not be justified. This investigation instead measured point thinning and collection settling.

## E01: fresh reproduction before implementation

Baseline revision: `c5e6b90f2c97139e92c4e7e5a4a82b448f448e38`, initially clean checkout. [replay.py](replay.py) creates isolated dataset copies and runs the existing full-auto CLI with the regression fixture's spool settings, `--sim --no-collect`, one process per fixture, CPU thread limits and bounded process deadlines. It retains stdout, calibration logs, raw JSONL events and commands. Dataset growth/automatic stopping remains active.

All 11 processes complete, but that is not calibration success: nine initial fits attempt underconstrained recovery and six fail recovery before later dataset growth. The fits take 31.39–60.69 seconds (median 44.78), with 64 replay iterations in total. These times overlap other jobs and are not controlled solver-speed estimates. Current history scoring has 20 adjacent disagreements with the diagnostic physical parameter error across nine fixtures. Seventh HP3 returns anchor RMSE 55.812 mm despite having encountered a 40.580 mm candidate; April HP3 returns 64.416 mm despite a 26.775 mm candidate. See [baseline summary](E01-baseline-summary.json) and [all candidates](E01-E01-baseline-audit.json).

Anchor error is RMS Euclidean distance per anchor against declared simulation-fixture truth; spool error is RMS mm per radius. The regression diagnostic `sum(anchor distances)+2*pi*sum(radius errors)` is separate. The audit's history-score choice can differ from the final prediction-based choice; returned-anchor regret is measured separately. Logged coordinates/radii are rounded. Stored HP3 fixture truth is browser-simulation geometry, not hardware metrology; provenance is documented in the [reference-data README](../../../../autocal/data/references/README.md).

## E02: fewer recorded observations

The trial keeps both drive directions, endpoints and five evenly distributed observations per direction, reducing a 20-point sweep to 10. It does not change fixed-length configurations, spool settings, solver or stop policy. It is retrospective deletion of recorded points, not a measured physical collection with larger point-to-point movements.

| Fixture | Anchor RMSE full → half (mm) | Radius RMSE full → half (mm) | Seen observations full → half | Iterations full → half |
| --- | ---: | ---: | ---: | ---: |
| layered_manual_some_bad_points | 6.229 → 13.799 | 0.0760 → 0.3740 | 180 → 90 | 7 → 7 |
| ten_points_bigger_deltas | 5.930 → 7.816 | 0.0360 → 0.0860 | 160 → 80 | 6 → 6 |
| seventh_hp3_dataset | 55.812 → 45.006 | 0.1745 → 0.1345 | 140 → 80 | 5 → 6 |
| 23_april_2026 | 64.416 → 67.049 | 0.1845 → 0.1845 | 160 → 100 | 6 → 8 |

Three of four selected anchor estimates worsen. Seventh HP3 improves but needs an extra iteration; April needs two extra iterations and worsens. A default half-point policy is rejected. Selected physical and prediction outcomes must be checked together.

Two complete untouched suffix sweeps (40 observations each) remain beyond the common replay horizon for ten-points and seventh HP3. Ten-points held-out geometric RMS is 0.831 → 0.835 mm and nearest-branch sensor RMS 1.344 → 1.347 mm, while anchor error grows. Seventh HP3 improves geometric RMS 1.172 → 1.022 mm and sensor RMS 2.062 → 1.746 mm, with 40/40 predictions. These suffixes never enter either fit or history selection. Bad-points and April exhaust their available common suffix; no independent prediction result is available there. Sensor error uses the nearest of two encoder-only circle branches and is an optimistic prediction-set residual, not a unique trajectory forecast. [Held-out comparison](E02-heldout-comparison.json) preserves IDs and invalid counts. Helpers are explicitly reused from [prior audit.py](../fdb616a454af4a6d9c1db074fbdc207d/audit.py).

## E05: controlled analytical data

[synthetic_points.py](synthetic_points.py) generates 12 HP4 circle configurations, 24 samples each, with raw angles from an assumed known constant-radius spool. Nine sweeps train and three entirely different fixed-delta configurations remain test-only (72 points). Two seeds use 60 mm initial parameter perturbations. Clean data have 0.02 mm encoder-equivalent noise; the stressed condition has 0.5 mm noise and a 20 mm sensor outlier per sweep. The current anchor solver uses two restarts, 400 iterations, a robust loss and disabled rejection; known radius and exact circle mechanics are experimental controls, not a production spool-fit improvement.

Clean 24-point fits have 0.083–0.123 mm anchor RMSE; 12-point fits have 0.083–0.146 mm, with held-out geometric RMS about 0.019–0.021 mm. Under noise/outliers, 24-point anchor RMSE is 5.465–6.441 mm versus 2.279–3.224 mm at 12 points. That subset happens to omit the injected outlier, so it does not prove that arbitrary sparse data are more robust. Six-point fits violate the current `min_points=10` guard: zero valid training sweeps, cost sentinel 100, and unchanged poor seeds (74–130 mm anchor error). Their reported optimizer status is not accuracy evidence. All cases and generated inputs are preserved in [E05-synthetic-points.json](E05-synthetic-points.json) and `E05-*.json`.

## E03/E06: settling reproduction and drift

Controlled traces use the same wait function with synthetic encoder replies and an injected clock; no actual sleep or physics movement is involved. A two-second transient returns at 6.5 s in baseline versus 3.5 s using the recent window, with identical final encoder values. Quiet-from-start, sustained-drift timeout, longer vibration acceptance and invalid-response recovery are preserved in the corrected trial. Deterministic regression tests additionally cover irregular polls and every motor.

An initial implementation filtered only samples inside the recent interval, losing boundary coverage under jitter and dereferencing empty history after a malformed reply. [V1 code](E03-candidate-actions-v1.mjs) and [failure results](E03-synthetic-settling-v1.json) are retained. The production code retains the initial sample-count guard and the sample at/before the quiet-window boundary.

[settling_recorded.mjs](settling_recorded.mjs) extracts consecutive 0.5-second encoder responses from baseline native collection, deduplicates identical bridge responses and splits on motion commands or noise-sampling intervals. These are heuristic episodes, not exact function-call traces. The first extractor missed duplicate replies and produced zero episodes; [V1 result](E06-recorded-settling-v1.json) is preserved and superseded.

Of 121 episodes, the unguarded candidate and baseline both complete 118, with three censored by trace exhaustion. It returns earlier in 39 pairs; summed paired waits are 347.0 → 219.5 s, but maximum drift to the later recorded endpoint is 1.932 degrees. The duration-scaled threshold completes 114 pairs (seven censored), with waits 341.0 → 242.0 s and maximum later drift 0.325 degrees; its 95th percentile is 0.181 degrees. Different censoring means these totals are not a direct end-to-end speed comparison. The recording cannot show the altered controller/physics feedback after early returns; that motivates fresh native collection. [Unguarded replay](E06-recorded-settling.json), [guarded replay](E06-recorded-rate-settling.json).

## E04/E08: matched simulated collection

Each trial starts from a freshly reset world using the same authored HP4 scene, RRF binary/configuration, production headless-JS physics, forces, feed, sensor preload and collector settings. It collects three fixed-role configurations `[2,3]/0/1`, `[1,3]/0/2`, `[0,3]/1/2`, each in both directions with six points per direction. Point acquisition uses the real production collector; encoder readings are physics observations. Force autotuning is explicitly disabled by native defaults. Three complete jobs collect 108 total points; no hardware moved.

| Variant | Job ID | Simulation s | Wall s | Points/predictions | Firmware-anchor residual RMS mm | Maximum measured point sigma deg |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| baseline | `5c3fe5661f19484cb54361b1250ce5cc` | 429.668 | 250.220 | 36/36 | 0.2822 | 0.01483 |
| unguarded | `a4498f9082e44e85b1c1b66c66bd229b` | 291.864 | 168.480 | 36/36 | 0.2897 | 0.02965 |
| final_guarded | `7f11879604bb4ce6bc577625c0259cd8` | 320.364 | 180.898 | 36/36 | 0.2878 | 0.02224 |

The retained guard saves 109.304 simulation seconds (25.44%) and 69.322 wall seconds (27.70%) in this measured pair. All three trials complete automatically and produce version-2 datasets with 36 finite calibration residuals. Scene, physics, firmware and configuration hashes match; collector code hashes differ as intended. Simulation-time reduction is the most direct collection result. Wall times include telemetry/Rerun and overlap bounded offline fits, so there is no isolated hardware-speed estimate or confidence interval.

Measurements are similar rather than bit-identical: final guarded mean length observations differ by at most 0.08165 base-model mm from corresponding baseline observations; drive setpoints differ by at most 0.01047 mm. Maximum measured point sigma rises from 0.01483 to 0.02224 degrees, and firmware-anchor residual RMS rises about 0.0056 mm. These disclose the speed/settling tradeoff. Point count, complete prediction coverage and schema checks are verified; newly collected anchor/radius accuracy is not. Authored HP4 has distributed attachment/pulley geometry, and firmware M669 is a nominal point-anchor model, not external ground truth. Three short sweeps do not establish joint anchor/spool identifiability or performance on long flexible cables. The unguarded speed gain was not retained because of its larger observed later drift.

[Matched metrics](E08-native-comparison.json) links each frozen manifest and dataset. Manifests link finalized RRDs, encoder/command traces, point journals, authored scenes and firmware configuration. Rerun inspection showed the collection and settling signals; [final screenshot](E08-final-viewer.png) is illustrative, not numerical accuracy evidence. Viewer warnings included an early missing `sim_time` and memory-pressure eviction of two background recordings; all immutable RRDs and numeric evidence remain on disk. [Viewer logs](E08-viewer-logs.json).

## E07/E09: verification and automatic completion

Final recorded regressions run as two bounded sequential workers, rather than eleven simultaneous native compiler jobs. All eleven outputs match both fresh baseline and stored references at 0.01 mm parameter-distance tolerance. Returned anchors/radii, iteration counts, scores and stopping decisions are identical. All 5,184 non-timing numerical JSONL fields match exactly; only path text and recovery elapsed times differ. [Regressions](E09-regressions.json), [full comparator output](E09-regressions.txt), [comparison script](compare_regressions.py). This proves preservation on recorded inputs, not equality of new measurements.

Python checks: 310 passed, five slow tests deselected ([log](E07-python-tests.txt)). The initial wider Jest run had 59 passes and one existing HTTP test that could not bind loopback (`listen EPERM`) and timed out ([log](E07-collector-tests.txt)). After excluding that environment-blocked file, all 55 relevant collector tests pass ([final log](E08-final-collector-tests.txt)); the excluded file contains six tests, five had passed in the wider run. The final tests include nine settling cases. The actual supervised RRF/bridge was exercised successfully by all three native jobs, but that does not turn the sandbox-blocked HTTP test into a pass. `git diff --check` passes.

The entry point remains one-click: bootstrap, fit, collect, choose and stop retain their existing code. The change requires no new CLI option or human acceptance step. The final native collection has completed; no collection job remains active. The service was reset after verification to reload the changed collector sources, leaving a fresh idle world; the completed datasets and recordings remain immutable.

## Disproved ideas and remaining experiments

- Unconditional half-point collection: rejected as a default; anchor error worsens on three of four fixtures and extra sweeps offset some point savings.
- Six-point sweeps in the current fitter: invalid due to minimum-point guard, not a completed poor-accuracy fit.
- Synthetic sparse success as general robustness: rejected; known radius, exact geometry and omitted injected outliers do not represent joint spool/anchor ambiguity.
- Unguarded recent-window settling: useful speed but insufficient protection against later drift; superseded by the guarded implementation.
- Naive window slicing and initial recorded parser: failed implementation/tool trials, corrected; failures retained.

The most promising untested count-reduction experiment is nuisance-aware movement design. Form a joint Jacobian over anchors, spool parameters and latent poses, eliminate latent/nuisance directions with a Schur complement, and rank feasible sweeps by information per measured collection second. Compare against the current geometry-only D-optimal selector using identical movement budgets, fresh configurations and holdouts. An encoder-only metric reference/gauge must be explicit. A bootstrap ensemble should test whether low residual reflects stable physical geometry before enabling an uncertainty-based early stop. These are proposed experiments, not negative results or implemented gains.

Continuous acquisition and trajectory shaping may reduce settling cost further, but would change the measurement model and deserve independent synthetic dynamics plus production-physics collection comparisons. Hardware verification of the retained guard should measure post-window drift/noise and final positioning under realistic cable lengths and friction, before claiming equal physical calibration accuracy.

![Thinning, collection time and drift tradeoff](results.png)

[Figure PDF](results.pdf), [plot script](plot_results.py).

## Sources actually used

- Darwin Lau (2018), [Initial length and pose calibration for cable-driven parallel robots with relative length feedback](https://research.cuhk.edu.hk/en/publications/initial-length-and-pose-calibration-for-cable-driven-parallel-rob-2/), DOI [10.1007/978-3-319-61431-1_13](https://doi.org/10.1007/978-3-319-61431-1_13). Supports relative-length initial-pose optimization, distinct from proving unknown-anchor/spool-scale recovery.
- Julien Alexandre dit Sandretto, David Daney and Marc Gouttefarde (2012 preprint; 2013 proceedings), [Calibration of a Fully-Constrained Parallel Cable-Driven Robot](https://www-sop.inria.fr/coprin/PDF/a_romansy2012.pdf), DOI [10.1007/978-3-7091-1379-0_10](https://doi.org/10.1007/978-3-7091-1379-0_10). Supports proprioceptive/latent-pose formulation, gauge constraints and held-out validation under its robot/model assumptions.
- Jin-Hwan Borm and Chia-Hsiang Meng (1991), [Determination of Optimal Measurement Configurations for Robot Calibration Based on Observability Measure](https://journals.sagepub.com/doi/10.1177/027836499101000106). Supports information-based pose selection; abstract consulted, not a cable-specific hardware result.
- Filippo Zoffoli, Edoardo Idà and Marco Carricato (2025), [A comparative analysis of initial-pose self-calibration algorithms for underactuated cable-driven parallel robots](https://www.cambridge.org/core/journals/robotica/article/comparative-analysis-of-initialpose-selfcalibration-algorithms-for-underactuated-cabledriven-parallel-robots/CD2FD0CE34646B8809C4248A7D81D6F6). Supports consideration of settling, pose distribution and acquisition strategy; includes internal sensors beyond the encoder-only scope.
- [Prior negative ranking report](../b7deb1bcc9cb418388273e2b7b9adc8c/report.md): why age-score removal, held-out prediction and bounded updates were not adopted wholesale.
- [Prior model-validation report](../fdb616a454af4a6d9c1db074fbdc207d/report.md): candidate-owned validation, winding mismatch, geometry/radius tradeoffs and D-optimal limits; its audit helpers support E02 suffix evaluation.
- [Prior compilation/cache report](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/report.md): existing speed change and reproducible baseline conventions; its replay/ranking helper was adapted here.
- [Research integration guide](../../../../research/README.md), [integration rationale](../../../../research/investigation.md), [native collection contract](../../../../research/native-collection.md): tool ownership, clock/reset/provenance and calibration-accuracy limits.
- [Fixture definitions/provenance](../../../../autocal/data/references/README.md), [regression tool](../../../../autocal/tools/regress_calibration_logs.py), [manifest](manifest.json): simulated truth, command flags, revisions and input hashes behind E01/E02/E09.

## Verification follow-up: scope of fresh collection
The user subsequently started a visible-browser full-auto HP3 run with global radius fitting and r0 bounds 39–40 mm; no result is available yet. E04/E08 used the production JavaScript collector and physics in headless Node, with RRF firmware, rather than the separate native-python physics backend. They tested fixed HP4 collection configurations with force autotuning disabled, not a fresh-data adaptive full-auto CLI run through final calibration acceptance. Browser controls were available: the choice of headless collection provided controlled clocks and fresh resets, but leaves that end-to-end browser workflow untested in this study. E09 stopping evidence applies to recorded inputs.
