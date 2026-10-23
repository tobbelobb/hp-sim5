# Research session catalog

Short abstracts and tested hypotheses from local sessions. Missing or pending summaries are not evidence of completed experiments.

Regenerate with `.venv/bin/python scripts/research_index.py`.

## [2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/)

[abstract.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/abstract.md) · [report.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/report.md) · [research.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/research.md)

### Faster encoder autocalibration through exact spool-model dataset reuse

Status: complete.

#### Abstract
This investigation asks whether encoder-only CDPR calibration can obtain reliable results with less collection or computation. Eleven fresh browser-simulation fixture replays reproduce 20 score/physical-error disagreements and nine underconstrained recoveries, six unsuccessful. Two comparable-validation rules remain unreliable. Uniform and geometry-aware half-point selection each worsen anchors on three of four recorded fixtures; 18 analytical synthetic fits show conditional, mixed benefits. Profiling instead finds thousands of repeated spool-model dataset conversions. A small implementation caches up to 16 exact radius/buildup transformations per filter pass, enabling existing residual-cache reuse. All eleven candidate regressions preserve parameters, stopping decisions and 5,184 numerical log fields exactly. Isolated three-repeat comparisons reduce median full-auto replay time by 2.5% on layered_manual_some_bad_points, 10.4% on seventh_hp3_dataset. The retained change reduces fitting work; collection point counts and physical ranking remain unchanged. Evidence is recorded simulation and analytical synthetic data, not hardware or new-data positioning validation.

#### Topics
CDPR, Hangprinter, HP3, Slideprinter, encoder-only autocalibration, exact spool cache, numerical speed, informative points, fewer points, common held-out sweeps, robustness, automatic stopping

#### Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Current scores agree with physical accuracy | E01 | Rejected | [Ranking audit](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/E01-E01-baseline-audit.json), [recovery audit](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/E01-fragility.json) |
| A common last sweep improves final ranking robustly | E02 (E00 archived control) | Mixed; rejected as default | [Shared comparison](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/E02-common-audit.json) |
| Every-shared-sweep dominance avoids physical regressions | E02 | Rejected as a general guard | [Paired comparisons](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/E02-common-audit-paired.json) |
| Geometry-aware thinning preserves full-data accuracy | E03 | Mixed; rejected as default | [Recorded comparisons](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/E03-recorded-comparison.json), [18 synthetic fits](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/E03-synthetic-results.json) |
| Exact bounded transformed-dataset reuse preserves behavior and speeds fitting | E04–E06 | Supported on tested fixtures | [Profiles](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/E04-profile), [11 regressions](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/E05-cache-regressions.json), [timing record](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/E06-isolated.txt) |
| Exact keys, eviction and pass isolation work | E07 | Supported tool checks | [Spool tests](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/E07-spool-tests.txt), [322 Python checks](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/E07-python-tests.txt) |
| Joint nuisance-aware acquisition, continuous acquisition and settling reuse improve collection | none | Not tested | [Proposals](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/report.md#remaining-experiments) |

#### References

- [Full report](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/report.md): method, measurements and limitations.
- [Research notebook](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/research.md): decisions, constraints, steering and budgets.
- [Source and dataset manifest](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/manifest.json): revision, hashes and numerical environment.
- [Retained source diff](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/final-source.diff): exact bounded cache and tests.
- [Prior collection investigation](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md): earlier thinning failures and guarded settling already in the baseline.

## [2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/)

[abstract.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/abstract.md) · [report.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/report.md) · [research.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/research.md)

### Faster repeated encoder autocalibration with persistent JAX compilation

Status: complete.

#### Abstract
This study investigated encoder-only CDPR calibration using eleven fresh recorded-data replays, synthetic circle sweeps, and solver profiling. Baseline scoring disagrees with physical error in 20 adjacent transitions across nine fixtures. Seventh HP3 selects anchors with 15.23 mm more RMSE than its best fitted candidate; nine initial fits require underconstrained recovery and six recoveries fail. Numerical-scale caching, shape padding, corrected padded arithmetic and a blanket gradient fallback were tested; none justified a default solver change. The implemented improvement automatically stores compiled CPU objectives across launches. Isolated repeated runs reduce wall time from 56.77 to 50.86 s on bad-points Slideprinter (10.4%) and 43.29 to 34.73 s on seventh HP3 (19.8%). All eleven calibration regressions and 300 tests pass, with identical outputs and stopping behavior. Evidence uses simulated fixture truth and analytical synthetic data, not hardware. First-time compilation remains slow, and candidate-ranking accuracy is unchanged.

#### Topics
encoder-only autocalibration, CDPR, HP3, HP4, Slideprinter, persistent JAX compilation cache, speed, physical ranking, negative results

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Current ranking agrees with known physical anchors | E01 | Rejected: 20 adjacent disagreements; substantial selected-anchor regret | [Fresh audit](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E01-baseline-audit.json), [fragility](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E01-fragility.json) |
| Dynamic numerical scales speed current recorded fits | E02–E04 | Mixed: synthetic compilation drops 3 to 1; scales constant on recorded runs | [Synthetic values/times](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E02-compilation.json), [equivalence](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E02-equivalence.json), [11 recorded controls](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E07-candidate-comparison.json) |
| Sweep and point shape padding preserves gradients | E05/E06 | Rejected on tested synthetic data | [Sweep/point failures](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E05-bucket-tests-expanded.txt), [point-only failures](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E06-pointbucket-tests.txt) |
| Finite padded arithmetic plus bucketing preserves physical fits | E07 | Rejected as accuracy-neutral: five-points RMSE 2.848 → 3.716 mm | [Counterexample](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E07-safe-comparison.json), [passing synthetic checks](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E07-safe-tests.txt) |
| All padded exact gradients are unusable | E07–E09 | Rejected: synthetic counterexamples, but all 740 recorded padded calls finite | [Synthetic failure](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E07-baseline-mixed-points.json), [recorded profile](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/baseline_profile_a/layered_manual_some_bad_points/compile-stats.json) |
| Blanket padded-gradient fallback improves the general default | E08 | Mixed; rejected default: bad-points gain, seventh HP3 worse | [All 11 comparisons](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E07-guard-comparison.json) |
| Persistent compilation caching speeds repeated launches while preserving calibration | E10 | Supported on measured replay pair; 11/11 regressions preserved | [Timings](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E10-controlled-timing.json), [regressions](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E10-final-regressions.json), [raw event checks](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E10-raw-events-comparison.json) |
| Unavailable cache allows computation to finish | E10 | Supported tool check, not calibration-accuracy evidence | [Check](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/E10-unavailable-cache.txt) |
| Joint latent-pose fitting, nuisance-aware movement selection, elasticity identification and uncertainty stopping improve accuracy | None in this session | Not tested; proposed follow-up | [Ideas and constraints](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/research.md) |

#### References
- [Full report](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/report.md): measurements, method, limitations and literature.
- [Notebook](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/research.md): decisions, budgets and accepted steering.
- [Experiment registry](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/experiment-registry.json): completed, failed and interrupted replays and commands.
- [Source/dataset manifest](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/manifest.json): revision, fixture truth, hashes and versions.
- [Final source diff](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/final-source.diff): the retained small change.
- [Tooling assessment](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/report.md#follow-up-simulator-usefulness-trust-and-tooling): no direct physics/viewer use here; evidence limits and proposed improvements.
- [Earlier backend performance/parity](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/../../../../research/collection-performance.md) and [native collector/viewer verification](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/../../../../research/native-collection-verification.md): preceding tooling evidence, not experiments in this session.
- [Previous model-validation research](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/../autocal-candidate-model-validation-and-throughput-fdb616a4/report.md): prior warm-start and own-model-ranking failures.
- [Previous negative ranking research](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md): prior ambiguity and age-score failures.

Browser fixture provenance clarified by the user: reference data came from browser physics collection; much configuration is already in source and JSON. The proposed metadata improvement is automatic linkage to the generating revision/scene, not additional measurement. See [clarification](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/report.md#clarification-existing-browser-fixture-definitions).

Implemented the follow-up as one compact [reference-data README](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/../../../../autocal/data/references/README.md), using existing JSON and verified git history. [History audit](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/provenance-history.json); no new calibration experiment or runtime change.

## [2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b4c1444b](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b4c1444b/)

[abstract.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b4c1444b/abstract.md) · [research.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b4c1444b/research.md)

### Improve encoder-based CDPR autocalibration in hp-sim5. The current algorithm can

Status: not yet summarized.

#### Abstract

The session has started. No research result has been summarized yet.

#### Topics

Improve encoder-based CDPR autocalibration in hp-sim5. The current algorithm can

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| None recorded yet | None | Not tested | None |

#### References

- [Original task](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b4c1444b/prompt.txt)
- [Research notebook](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b4c1444b/research.md)

## [2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/)

[abstract.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/abstract.md) · [report.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md) · [research.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/research.md)

### Faster encoder-only CDPR calibration collection with guarded settling windows

Status: complete.

#### Abstract

This study asks whether Hangprinter calibration can collect encoder data faster while preserving its one-click workflow and automatic stopping. Eleven fresh recorded-data replays reproduce 20 score/physical-error disagreements and six failed recoveries. Halving points worsens anchor RMSE on three of four recorded fixtures, despite promising conditional synthetic results. Settling experiments found that old motion was retained in the short quiet check. The implemented change checks recent readings with a tighter range limit while retaining the existing vibration fallback. Matched production-JS/RRF collection from fresh HP4 worlds reduces 36-point collection from 429.668 to 320.364 simulation seconds (25.4%). Wall time falls from 250.220 to 180.898 seconds in this pair. All 36 observations pass residual ingestion; this is throughput evidence, not proof of equal anchor accuracy on newly collected data. Eleven calibration regressions, stopping decisions and 5,184 numerical event fields remain unchanged on recorded inputs. Hardware transfer and generic ranking accuracy remain unverified.

#### Topics

CDPR, Hangprinter, HP4, HP3, encoder-only autocalibration, settling window, collection speed, drift guard, fewer points, physical ranking, synthetic data, one-click, automatic stopping

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Current ranking agrees with physical accuracy | E01 | Rejected: 20 adjacent disagreements across 9 fixtures | [Baseline summary](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E01-baseline-summary.json), [candidate audit](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E01-E01-baseline-audit.json) |
| Halving recorded points preserves robust calibration | E02 | Mixed; rejected as a default (3/4 anchors worsen) | [Physical comparison](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E02-comparison.json), [untouched suffix predictions](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E02-heldout-comparison.json) |
| A recent quiet window removes unnecessary post-transient waits | E03/E04/E06 | Supported, with premature-return drift limitation | [Controlled traces](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E03-synthetic-settling.json), [unguarded native manifest](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/native/77bb83dffe5c4a74920b2d6ad6e296d6/a4498f9082e44e85b1c1b66c66bd229b/manifest.json), [trace replay](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E06-recorded-settling.json) |
| A duration-scaled quiet threshold retains speed while reducing later drift | E06/E08 | Supported in measured simulation; hardware not tested | [Guarded trace replay](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E06-recorded-rate-settling.json), [matched collection](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E08-native-comparison.json) |
| Sparse clean synthetic success generalizes to recorded spool fits | E02/E05 | Rejected as a general rule; known-radius synthetic results are conditional | [Synthetic fits](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E05-synthetic-points.json) |
| Six observations per sweep suffice for the current fitter | E05 | Rejected for the current min_points=10 guard; these fits are invalid | [Synthetic fits](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E05-synthetic-points.json) |
| Recorded calibration results and automatic stopping survive the collection change | E09 | Supported on all 11 fixtures | [Regressions](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E09-regressions.json), [detailed comparisons](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E09-regressions.txt) |
| Joint latent-pose fitting, nuisance-projected information and uncertainty stopping reduce collection count | none | Not tested; proposed follow-up | [Ideas and proposed experiment](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/research.md) |

#### References

- [Full report](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md): measured outcomes, implementation and limitations.
- [Notebook](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/research.md): objectives, budgets, experiment decisions and accepted steering.
- [Manifest](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/manifest.json): revisions, hashes, datasets and runtime versions.
- [Experiment registry](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/experiment-registry.json): actual trials, failed attempts and evidence paths.
- [Final patch](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/final-source.diff): retained collector, tests and documentation changes.
- [Prior calibration/cache research](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/report.md): earlier ranking failures and compilation improvements.
- [Prior model-validation research](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/../fdb616a454af4a6d9c1db074fbdc207d/report.md): whole-sweep and spool ambiguity findings.
- [Prior negative ranking research](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/../b7deb1bcc9cb418388273e2b7b9adc8c/report.md): bounded updates, age-score and active-selection failures.

## [2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/)

[abstract.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/abstract.md) · [report.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/report.md) · [research.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/research.md)

### Independently testable encoder calibration stages

Status: complete.

#### Abstract

Encoder autocalibration was split into modeled-length transformation, fitting and assessment, frozen sweep planning, prediction evaluation and pure history ranking. Versioned JSON artifacts allow downstream experiments without rerunning fitting or collection. Eleven recorded simulated baselines reproduced current references and showed 20 adjacent score/parameter-error disagreements, with physically worse final selections on ten datasets. The final implementation preserves every returned and intermediate calibration exactly. A small metadata-copying optimization reduced mean fitting time from 3.290 to 2.992 s in seven alternating pairs; the complete regression matrix changed from 201.512 to 192.633 s (4.4% faster). Frozen planning, prediction evaluation and ranking take about 32.7 ms, 14.5 ms and 2.27 microseconds. All 337 Python tests pass. Accuracy is preserved rather than improved; evidence is simulated, runtime samples are bounded, and fitting must keep borrowed nested measurement metadata read-only.

#### Topics

autocalibration, encoder, CDPR, modular pipeline, stage replay, runtime, metadata copying, selection regret

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Existing ranking can favor physically worse parameters | E01 | Supported | [Physical audit](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E01/physical-audit.json), [baseline results](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E01/results.json) |
| Filtering can repeatedly fail to restore constraints | E01 | Supported: fifth HP3 has six failed recovery attempts | [Recovery audit](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E01/recovery-audit.json), [log](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E01/fifth_hp3_dataset.log) |
| Stage extraction preserves calibration and permits downstream-only trials | E02a, E03, E04, E08 | Supported on all 11 fixtures; frozen fit replay exact | [Paired results](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E02a/results.json), [stage checks](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E03/benchmark.json), [final comparison](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E08/comparison.json) |
| Information reuse removes redundant planning work without changing ranks | E03, E06, E07 | Supported; standalone extraction initially had inconclusive full-fit timing | [Alternating planner measurements](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E07/benchmark.json) |
| Borrowing read-only metadata accelerates fitting without changing lengths or calibration | E07, E08 | Supported within measured battery | [All-dataset transformations](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E07/transforms.json), [fit pairs](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E07/benchmark.json), [full regression](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E08-regression.log) |
| Raw-rank-only selection improves the small frozen case | E05 | Supported for one fixture; not accepted as a general policy | [Frozen-history pilot](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E03/raw-rank-pilot.json) |

#### References

- [Research notebook](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/research.md): objective, steering, budgets, failed driver trial and decisions.
- [Final regression command, runtime and status](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E08-results.json), [full output](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/E08-regression.log): all eleven reference checks pass with zero deltas.
- [Source/input provenance](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/provenance.json), [source patch](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/final-source.patch), [saved source](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/source/): baseline revision, final source hashes, dataset/reference hashes and numerical environment.
- [Prior rejected selection and solver experiments](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-df7f5fd7/../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md): why a single-fixture accuracy improvement is insufficient for changing selection.

## [2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-f5e17805](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-f5e17805/)

[abstract.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-f5e17805/abstract.md) · [research.md](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-f5e17805/research.md)

### Improve encoder-based CDPR autocalibration in hp-sim5. The current algorithm can

Status: not yet summarized.

#### Abstract

The session has started. No research result has been summarized yet.

#### Topics

Improve encoder-based CDPR autocalibration in hp-sim5. The current algorithm can

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| None recorded yet | None | Not tested | None |

#### References

- [Original task](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-f5e17805/prompt.txt)
- [Research notebook](sessions/2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-f5e17805/research.md)

## [2026-10-09-headless-recording-review](2026-10-09-headless-recording-review/)

[abstract.md](2026-10-09-headless-recording-review/abstract.md) · [report.md](2026-10-09-headless-recording-review/report.md) · [research.md](2026-10-09-headless-recording-review/research.md)

### Critical review of headless autocal recording finalization

Status: complete.

#### Abstract

Review of commit 8ded7420 examined headless recording startup, clock labels, acknowledgement backpressure and shutdown. Nine fast Python headless tests and thirteen JavaScript recorder tests passed. A short fault injection killed the real owned Node physics process after successful requests, using synthetic optimization-completion markers to isolate lifecycle validation. The process exited -9, yet context shutdown succeeded and the manifest reported complete status and normal_completion=true. The recorder finalized its RRD and recorded autocal_complete=true, with no physics service_stopped event. Recorder finalization does not establish graceful physics-source draining. The test establishes a failure-reporting gap, not a measured amount of lost telemetry or any calibration-accuracy result. No production code was changed; the slow full-auto/parity suite was not rerun.

#### Topics

autocalibration, headless simulation, Rerun RRD, extended reference, finalization, crash detection, wall clock, acknowledgement backpressure

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Unexpected physics death prevents successful recording completion | E1 | Rejected | [Result](2026-10-09-headless-recording-review/result.json), [manifest](2026-10-09-headless-recording-review/sweeps.headless_1/manifest.json), [probe](2026-10-09-headless-recording-review/probe.py) |
| Existing focused lifecycle and delivery checks pass | T1, T2 | Supported within their tested scope | [Report](2026-10-09-headless-recording-review/report.md) |

#### References

- [Full report](2026-10-09-headless-recording-review/report.md): finding, checks and limitations.
- [Research notebook](2026-10-09-headless-recording-review/research.md): constraints, permission limitation and outcome.
- [Reviewed lifecycle](2026-10-09-headless-recording-review/../../../autocal/headless_sim.py): commit 8ded7420 shutdown logic.
- [Implementation verification](2026-10-09-headless-recording-review/../../../research/headless-autocal-verification.md): full-run evidence and documented parity failure; not independently rerun here.

## [2026-10-09-slip-recovery-and-encoder-model](2026-10-09-slip-recovery-and-encoder-model/)

[abstract.md](2026-10-09-slip-recovery-and-encoder-model/abstract.md) · [report.md](2026-10-09-slip-recovery-and-encoder-model/report.md) · [research.md](2026-10-09-slip-recovery-and-encoder-model/research.md)

### Recovering autocal slips and retaining fixed encoder errors

Status: complete.

#### Abstract

This investigation implements bounded reduced-force recovery after a fixed encoder slips and automatic closed-loop/layering/recording startup. An isolated production JS/RRF fault injection caused 8.01-degree fixed B drift. The collector restored position at lower commands and completed both directions, retaining 20 valid points with B span 0.47 degrees and maximum fixed error 0.38 degrees. A closed-loop bootstrap with automatic travel measurement completed 60 points. A real recorder connection preserved its session and acknowledged initial physics before G-code events. Trial-005 fixed encoders span at most 0.01 degrees; its final rank-zero fit had no valid next candidate. Finite-residual comparisons show that current objectives ignore per-point fixed encoder variation: perturbing raw fixed observations by ±0.7 degrees, or reversing fixed-angle associations while preserving modeled medians, leaves residuals identical. The recommended extension uses measured per-point fixed lengths and propagated uncertainty in both NumPy/JAX pointwise objectives and held-out validation. That objective extension is proposed, not implemented. No hardware recovery or improved calibration accuracy is established.

#### Topics

autocalibration, slip recovery, fixed encoders, closed loop, line layering, Rerun recording, pointwise objectives

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Reduced force can recover a slipped direction | E1 | Supported | [Verified recovered dataset](2026-10-09-slip-recovery-and-encoder-model/overload-guard-recovery-verified/sweeps.json), [log](2026-10-09-slip-recovery-and-encoder-model/overload-guard-recovery-verified/collector.log) |
| Settings update UI and wait for recorder startup | E2 | Supported | [Bootstrap dataset](2026-10-09-slip-recovery-and-encoder-model/patched-bootstrap-auto-size/sweeps.json), [recording result](2026-10-09-slip-recovery-and-encoder-model/recording-startup/result.json) |
| The objective discards pointwise fixed encoder variation | E3 | Supported | [Finite residual audit](2026-10-09-slip-recovery-and-encoder-model/encoder-objective-audit.json) |
| Trial-005 stopped without a valid next candidate | E4 | Supported | [Trial-005 log](2026-10-09-slip-recovery-and-encoder-model/../../extended-autocal/trial-005/sweeps.full_auto.log) |

#### References

- [Notebook](2026-10-09-slip-recovery-and-encoder-model/research.md), [report](2026-10-09-slip-recovery-and-encoder-model/report.md).
- [Preceding slipping investigation](2026-10-09-slip-recovery-and-encoder-model/../2026-10-09-trial004-upper-sweeps/report.md): original high-force failure.
- [Source revision](2026-10-09-slip-recovery-and-encoder-model/revision.txt) and [working patch](2026-10-09-slip-recovery-and-encoder-model/working.diff): tested production code.
- [Trial-005 dataset](2026-10-09-slip-recovery-and-encoder-model/../../extended-autocal/trial-005/sweeps.json): closed-loop fixed measurements.
- [Optimization overview](2026-10-09-slip-recovery-and-encoder-model/../../../autocal/optimization_algorithm_overview), [objective overview](2026-10-09-slip-recovery-and-encoder-model/../../../autocal/objective_functions_overview): current and proposed error treatment.

## [2026-10-09-trial002-return-settling](2026-10-09-trial002-return-settling/)

[abstract.md](2026-10-09-trial002-return-settling/abstract.md) · [report.md](2026-10-09-trial002-return-settling/report.md) · [research.md](2026-10-09-trial002-return-settling/research.md)

### HP3 trial-002 return settling and recording performance

Status: complete diagnosis and fix; full autocal retry pending.

#### Abstract

Two stopped manual browser recordings show that sampled extended recording increased
achieved playback from 0.570x to 5.980x while reducing file bytes per simulation
second by 11.18x. Bytes per wall second remained similar because simulation ran
10.49x faster. Trial-002 failed after the fourth edge-force trial: return-to-origin
waited for encoder stability while the 0.623633 N pull remained active. An isolated
production JS world using the saved HP3 scene and recorded torque reproduced the
30-second timeout and similar terminal angles; position mode then settled in two
simulation seconds. A synthetic control-flow regression fails before the fix and
returns to origin afterwards. Return routines now hold motors before settling,
including zero-distance paths. These checks do not establish full autocal success
or hardware behavior.

#### Topics

HP3, autocalibration, return to origin, force tuning, encoder settling, Rerun,
wall clock, simulation clock, recording performance, sampling.

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Recorder changes improve playback and storage per simulation second | E1 | Supported for these recordings | [Performance](2026-10-09-trial002-return-settling/performance.json) |
| The failed return waits with the trial force still applied | E2 | Supported | [Recorded events](2026-10-09-trial002-return-settling/failure-events.json) |
| Position mode resolves this failure in the production JS scene | E3 | Supported for isolated reproduction | [Probe results](2026-10-09-trial002-return-settling/hold-probe-results.json) |
| Holding before return fixes the control-flow regression | E4 | Supported | [Regression script](2026-10-09-trial002-return-settling/return-regression.mjs), [report](2026-10-09-trial002-return-settling/report.md) |
| Full manual autocal completes after the fix | None | Not tested | Retry pending |

#### References

- [Full report](2026-10-09-trial002-return-settling/report.md): methods, measurements, changes and limitations.
- [Research notebook](2026-10-09-trial002-return-settling/research.md): scope and execution bounds.
- [Trial-001 manifest](2026-10-09-trial002-return-settling/../../extended-autocal/trial-001/hangprinter-20261009T073546.933824Z-9d7ded0f.json): initial every-step run.
- [Trial-002 manifest](2026-10-09-trial002-return-settling/../../extended-autocal/trial-002/hangprinter-20261009T090037.975347Z-a58c273c.json): stopped sampled run.
- [Trial-002 log](2026-10-09-trial002-return-settling/../../extended-autocal/trial-002/sweeps.full_auto.log): failed collection.
- [Source revision](2026-10-09-trial002-return-settling/revision.txt): base revision inspected before the fix.

## [2026-10-09-trial003-size-measurement](2026-10-09-trial003-size-measurement/)

[abstract.md](2026-10-09-trial003-size-measurement/abstract.md) · [report.md](2026-10-09-trial003-size-measurement/report.md) · [research.md](2026-10-09-trial003-size-measurement/research.md)

### HP3 trial-003 workspace measurement exceeds the settling deadline

Status: complete diagnosis and focused fix; full manual retry pending.

#### Abstract

Trial-003 completed force tuning but failed during workspace-size measurement, before the first sweep. Its final commands pull two cables at 9.6637 N while releasing the third. The third encoder travels about 1,157 degrees during the 30-second wait. The timeout path then disconnects without holding the motors. Replaying recorded commands and polling times in an isolated production JavaScript physics world finds encoder stability after 37.7 simulation seconds. Active workspace measurement and sweep pullout now have a 120-second selected-clock budget with the existing independent wall deadline and stability tolerances. Size measurement holds motors on exit, and the collector attempts a hold on any collection error. Synthetic regressions cover achieved speeds of 0.57x and 25x, continued drift, a stopped clock and encoder disconnection; all 79 collector unit tests pass. These focused checks do not establish full browser collection or hardware success.

#### Topics

HP3, autocalibration, workspace-size measurement, encoder stability, force travel, timeout, observed simulation clock, cleanup, Rerun replay.

#### Tested hypotheses

| Hypothesis | Experiment | Outcome | Evidence |
| --- | --- | --- | --- |
| Size measurement rather than force tuning causes the timeout | E1 | Supported | [Recorded events](2026-10-09-trial003-size-measurement/failure-events.json) |
| Motion needs more than 30 simulation seconds under recorded polling | E2 | Supported for isolated replay | [Replay](2026-10-09-trial003-size-measurement/replay.mjs), [results](2026-10-09-trial003-size-measurement/replay-results.json) |
| Patched measurement finishes and sends position hold | E3 | Supported for isolated replay | [Probe](2026-10-09-trial003-size-measurement/measurement-probe.mjs), [results](2026-10-09-trial003-size-measurement/measurement-probe-results.json) |
| Deadlines remain finite and cleanup runs on failure | E4 | Supported with synthetic clocks | [Tests](2026-10-09-trial003-size-measurement/../../../autocal/control/tests/behaviors/measure_max_travel.test.mjs) |
| Full manual collection completes | None | Not tested | Manual retry pending |

#### References

- [Full report](2026-10-09-trial003-size-measurement/report.md): diagnosis, measured results and limitations.
- [Research notebook](2026-10-09-trial003-size-measurement/research.md): experiments and execution bounds.
- [Trial-003 manifest](2026-10-09-trial003-size-measurement/../../extended-autocal/trial-003/hangprinter-20261009T095245.092268Z-25141452.json) and [recording](2026-10-09-trial003-size-measurement/../../extended-autocal/trial-003/hangprinter-20261009T095245.092268Z-25141452.rrd): recorded clocks and events.
- [Trial-003 full auto log](2026-10-09-trial003-size-measurement/../../extended-autocal/trial-003/sweeps.full_auto.log): successful tuning followed by timeout.
- [Previous return fix](2026-10-09-trial003-size-measurement/../2026-10-09-trial002-return-settling/report.md): preceding timeout had a different cause.
- [Source revision](2026-10-09-trial003-size-measurement/revision.txt), [changes](2026-10-09-trial003-size-measurement/working.diff), [saved scene](2026-10-09-trial003-size-measurement/scene.usda), [command arrivals](2026-10-09-trial003-size-measurement/commands.json), [encoder poll times](2026-10-09-trial003-size-measurement/poll-times.json): replay provenance.

## [2026-10-09-trial004-upper-sweeps](2026-10-09-trial004-upper-sweeps/)

[abstract.md](2026-10-09-trial004-upper-sweeps/abstract.md) · [report.md](2026-10-09-trial004-upper-sweeps/report.md) · [research.md](2026-10-09-trial004-upper-sweeps/research.md)

### Fixed-motor slipping corrupts upper autocal sweeps

Status: complete.

#### Abstract

Trial-004 contained two upper sweeps with visibly irregular paths. Recorded commands and encoders, followed by isolated production JS physics and RRF collector experiments, identified loss of fixed B position: its encoder spanned 193.04 and 200 degrees. Simulator branch tensions exceeded B's modeled holding torque, while the collector accepted stationary encoders after slipping and repeatedly corrected B between points. A reproduced baseline matched the recorded first upper sweep to 0.01 degree. Lower default commands and a fixed-target encoder guard reduce B spans to 0.50 and 0.18 degrees in the two configurations; all six bootstrap directions also complete. A deliberate overload is rejected at 8.01 degrees before saving a dataset, with position-hold commands issued. No runtime force sensing is introduced: tensions are diagnostic simulator evidence only. Eighty-nine collector tests pass. These results establish collection behavior in the tested simulator geometry, not hardware safety or improved numerical calibration accuracy; a fresh visual capture and optimization remain untested.

#### Topics

autocalibration, HP3, fixed anchors, motor slipping, encoder validation, force preload, RRF, Rerun replay, candidate ranking

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Upper sweeps lose fixed B position | E1 | Supported | [Recorded and reproduced encoder spans](2026-10-09-trial004-upper-sweeps/sweep-metrics.json) |
| Applied cable load exceeds modeled B holding torque | E2 | Supported | [Command replay](2026-10-09-trial004-upper-sweeps/replay-results.json) |
| Lower forces preserve fixed geometry | E3 | Supported | [Baseline and reduced-force datasets](2026-10-09-trial004-upper-sweeps/reduced-force/sweeps.json) |
| Patched collector completes upper/bootstrap sweeps and rejects slipped fixed encoders | E4 | Supported | [Final metrics](2026-10-09-trial004-upper-sweeps/sweep-metrics.json), [overload log](2026-10-09-trial004-upper-sweeps/overload-guard-20x/collector.log) |
| Fits 4/5 are closer to known simulator geometry than fit 1 | E5 | Supported | [Fit comparison](2026-10-09-trial004-upper-sweeps/fit-errors.json) |
| A low pullout command also finishes long bootstrap travel | E6 | Mixed | [Five-times-start failure](2026-10-09-trial004-upper-sweeps/patched-bootstrap/collector.log), [ten-times-start failure](2026-10-09-trial004-upper-sweeps/patched-bootstrap-10x/collector.log), [twenty-times-start success](2026-10-09-trial004-upper-sweeps/patched-bootstrap-20x/sweeps.json) |

#### References

- [Full report](2026-10-09-trial004-upper-sweeps/report.md): cause, changes, measured behavior and limitations.
- [Research notebook](2026-10-09-trial004-upper-sweeps/research.md): hypotheses, bounds and accepted steering.
- [Trial-004 dataset](2026-10-09-trial004-upper-sweeps/../../extended-autocal/trial-004/sweeps.json) and [text log](2026-10-09-trial004-upper-sweeps/../../extended-autocal/trial-004/sweeps.full_auto.log): original bad sweeps and numerical fits.
- [Original RRD](2026-10-09-trial004-upper-sweeps/../../extended-autocal/trial-004/hangprinter-20261009T101909.456318Z-5eb541ea.rrd): recorded browser commands, encoders and scene settings.
- [Source revision](2026-10-09-trial004-upper-sweeps/revision.txt) and [working changes](2026-10-09-trial004-upper-sweeps/working.diff): production baseline and tested patch.
- [Preceding trial-003 investigation](2026-10-09-trial004-upper-sweeps/../2026-10-09-trial003-size-measurement/report.md): active travel timeout handling.

## [2026-10-10-autocal-recorder-waits](2026-10-10-autocal-recorder-waits/)

[abstract.md](2026-10-10-autocal-recorder-waits/abstract.md) · [report.md](2026-10-10-autocal-recorder-waits/report.md) · [research.md](2026-10-10-autocal-recorder-waits/research.md)

### Autocal return waits fixed by lossless recorder batching

Status: implementation complete; recorder replay and regression tests passed.

#### Abstract

The recorded reference run showed a 16,162-command return burst delaying event logging by 21.88 seconds and blocking physics after about 0.64 simulation seconds. The implementation batches browser/headless event transport and writes complete logical event rows as RRD columns, preserving payloads, original timestamps, receipt ordering and clock fields. Bounded queues, counted post-write acknowledgements, partial-batch flushes and failure handling retain capture integrity. An isolated localhost replay of the exact recorded envelopes acknowledged the following physics sample in 0.175 seconds with every envelope preserved. Direct writer replay improved from 1.670 to 0.109 seconds (15.4x). Recorder, headless lifecycle and calibration guard tests passed. These measurements used no Viewer and synthetic physics samples; they do not establish full browser collection speed or calibration accuracy. Observed-clock waits and force/slip guards remain unchanged.

#### Topics

Autocalibration, HP3, browser recording, force tuning, simulation clock, recorder backpressure, event batching, Rerun.

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Per-command recorder backlog explains return stalls | OBS-001 (existing-run observation) | Supported | [Original events](2026-10-10-autocal-recorder-waits/evidence.json), [report](2026-10-10-autocal-recorder-waits/report.md) |
| Columnar batching reduces writer overhead without losing original envelopes | EXP-001 (offline writer replay) | Supported | [Measurements](2026-10-10-autocal-recorder-waits/batch-benchmark/results.json) |
| Batching prevents a recorded command burst from blocking the following physics acknowledgement for seconds | EXP-002 (localhost recorder replay) | Supported | [Measurements](2026-10-10-autocal-recorder-waits/batch-benchmark/websocket/results.json) |
| Full browser collection speed and calibration decisions are unchanged | None | Not tested | [Limits](2026-10-10-autocal-recorder-waits/report.md) |

#### References

- [Full report](2026-10-10-autocal-recorder-waits/report.md): diagnosis, implementation, measurements and limitations.
- [Research notebook](2026-10-10-autocal-recorder-waits/research.md): constraints and accepted steering.
- [Reference log](2026-10-10-autocal-recorder-waits/../../extended-autocal/no-buildup-reference-001/sweeps.full_auto.log): original waits.
- [Writer replay results](2026-10-10-autocal-recorder-waits/batch-benchmark/results.json) and [WebSocket replay results](2026-10-10-autocal-recorder-waits/batch-benchmark/websocket/results.json): exact event preservation and timing.

## [2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/)

[abstract.md](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/abstract.md) · [report.md](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/report.md) · [research.md](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/research.md)

### HP3 encoder autocalibration: reference failure and runtime bottlenecks

Status: complete for the supplied reference; no source change justified.

#### Abstract

This investigation asks why zero-buildup HP3 encoder autocalibration is slow and can return poor anchors, and whether fewer points are safe. The supplied browser-js reference recorded 4,706,150 physics steps over 9,412.3 simulated seconds and ended after eight sweeps with a rank-zero, underconstrained fit; its recorder marks autocal incomplete. Encoder noise was reported as zero and fixed-target errors stayed small, while later anchor estimates became geometrically implausible. Recovery fitting took about 1.35 seconds per exhausted pass, making collection and settling the likely primary wait cost. Detailed event timing could not be completed because streaming the 2.0 GB RRD did not return within the bounded tool window. Earlier recorded trials found fixed point thinning worsened anchor RMSE on three of four fixtures; a guarded settling change saved about 25% in separate fixed HP4 collection. No source changes or hardware measurements were made. A coverage-aware, collection-time-aware planner and adaptive point count remain untested.

Follow-up inspection found 473,771 RRD chunks; Rerun's object-store estimate was 2,232. Its supported optimizer rewrote a separate copy in 66.9 seconds, reducing chunk count by 99.7% and file size by 41.3% (1.9 to 1.1 GiB), without intentional sample deletion. Exact semantic comparison did not complete. Rerun supports sparse unchanged values through latest-at semantics; trimming quiet intervals from existing data is a lossy derived transform and must preserve event timestamps and mark gaps.

#### Topics

autocalibration, HP3, encoder-only, anchor identifiability, rank deficiency, point count, collection time, sweep planning, Rerun profiling

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
|---|---|---|---|
| Supplied run completes automatic acceptance | E01 | Rejected | [Run log](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log) |
| Fitting/recovery dominates wait | E02 | Rejected for supplied run; detailed RRD profile incomplete | [Recorder manifest](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/../../../../extended-autocal/no-buildup-reference-002/hangprinter-20261010T151618.949443Z-c9ffe4cd.json) |
| Fixed halving of points is robust | E03 | Rejected in prior recorded trials | [Prior report](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md#disproved-ideas-and-remaining-experiments) |
| Selection score can disagree with physical accuracy | E04 | Supported by prior audits; consistent with this failed geometry | [Prior physical audit](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/E01-E01-baseline-audit.json) |
| Low encoder noise ensures identifiable geometry | E05 | Rejected for this simulated run | [Dataset](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/../../../../extended-autocal/no-buildup-reference-002/sweeps.json) |
| Chunk overhead materially inflates the RRD | E06 | Supported by Rerun stats | [Full report](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/report.md#rrd-size-and-whether-quiet-time-can-be-removed) |
| Supported RRD optimization reduces size | E07 | Supported as rewrite/size result; exact equivalence unverified | [Full report](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/report.md#rrd-size-and-whether-quiet-time-can-be-removed) |

#### References

- [Full report](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/report.md): analysis, measurements and limitations.
- [Research notebook](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/research.md): experiment decisions and next step.
- [Supplied dataset](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/../../../../extended-autocal/no-buildup-reference-002/sweeps.json), [log](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/../../../../extended-autocal/no-buildup-reference-002/sweeps.full_auto.log), [manifest](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/../../../../extended-autocal/no-buildup-reference-002/hangprinter-20261010T151618.949443Z-c9ffe4cd.json): failed HP3 simulated run.
- [Prior October 8 report](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-1687b8ed/../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md): ranking, thinning and settling evidence.
- [Rerun optimize-chunks guide](https://rerun.io/docs/howto/logging-and-ingestion/optimize-chunks) and [latest-at semantics](https://rerun.io/docs/concepts/logging-and-ingestion/latest-at): supported compaction and sparse component behavior.

## [2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/)

[abstract.md](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/abstract.md) · [report.md](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/report.md) · [research.md](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/research.md)

### Faster HP3 encoder autocalibration by reusing settled measurements

Status: complete.

#### Abstract

We profiled a supplied HP3 encoder-autocalibration recording, audited candidate anchors against simulated truth, tested sparse recorded datasets, and compared matched physical collectors with known radii and no line buildup. Settling dominated collection time. Raw score disagreed with geometry, and blind thinning often failed. Reusing a completed quiet check reduced 120-point collection from 2,733.214 to 2,571.350 simulated seconds (5.92%), while pooled anchor RMSE stayed 2.11845 mm. A follow-up reproduced the default-force pullout failure exactly through 2,457 encoder readings. Bounding endpoint search, then holding and strictly settling before measurement, recovered collection without relaxing guards. Review of supervised-job commit 383c6d4 exposed unset layering misreported as disabled, plus scene baking and reset settings; both were corrected. A full default-force HP3 job through a socket-denied MCP client then accepted automatically after seven iterations, with 360 points and 2.604 mm anchor RMSE. Its finalized 1.58 GB recording independently contains every measurement and the physical stop boundary. This is simulation evidence from one full default-force trial. Hardware transfer, reliable point-count reduction and score-based geometry selection remain unresolved; the full job is not a paired speed benchmark.

#### Topics

autocalibration, HP3, CDPR, encoder-only, collection timing, settling, candidate ranking, observability, sparse sweeps, fixed encoder errors, known radii, no buildup, supervised jobs, MCP, no-layering geometry, reset settings

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Settling is a larger collection cost than encoder averaging or motion waits in the supplied run | E01 | Supported | [Timing and geometry audit](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E01-profile-and-geometry.json) |
| Lower raw fit score reliably identifies more accurate anchors | E01, E02, E07–E08 | Rejected | [Original candidates](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E01-profile-and-geometry.json), [thinning fits](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E02-thinning.json), [matched fits](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E08-paired-comparison.json) |
| Uniformly keeping five or six points per direction, or one direction, is a safe default | E02 | Rejected | [Sixteen recorded-data trials](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E02-thinning.json) |
| Reusing the quiet check after an unchanged-force move saves time without materially degrading geometry | E03, E05–E08 | Supported in simulation | [Duplicate readings](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E03-duplicate-checks.json), [synthetic guards](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E05-synthetic-controller-tests.txt), [matched comparison](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E08-paired-comparison.json) |
| A single three-sweep span set preserves pooled-fit geometry | E07–E08 | Rejected for these datasets | [Independent and pooled fits](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E08-paired-comparison.json) |
| Shorter pullouts can preserve observability | E03 diagnostic only | Not tested | [Travel trace](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E03-long-pullouts.json); no altered collection |
| Bounded endpoint search followed by strict measurement settling recovers the reported failure | E11 | Supported in simulation | [Exact prefix](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E11-baseline-prefix-comparison.json), [settling result](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E11-settling-result.json) |
| Correct no-layering baking permits the default-force supervised workflow to finish with accurate anchors | E12 | Supported in one simulated job | [Acceptance](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E12-acceptance.json), [recording audit](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E12-rrd-verification.json) |
| Pointwise fixed-encoder geometry, pose elimination or information-per-time acquisition improves calibration | none | Not tested | Proposed in report; no new accuracy result |

| Command sandbox denies localhost sockets while launcher-owned MCP services work | E09 | Supported | [Tooling diagnostic](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E09-sandbox-and-tools.json) |

| Networking defaults to enabled while explicit overrides remain available | E10 | Supported, configuration checks only | [Launcher tests](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E10-launcher-tests.txt) |

#### References

- [Full report](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/report.md): methods, implementation, measured results and limitations.
- [Research notebook](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/research.md): objective, budgets, steering and decisions.
- [Session manifest](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/manifest.json): reference revisions and source/data hashes.
- [Supplied reference](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/../../../extended-autocal/no-buildup-reference-003/): original simulated collection and candidate history.
- [Matched comparison](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E08-paired-comparison.json): fresh baseline/candidate timing, sampling and geometric results.
- [Alexandre dit Sandretto, Daney and Gouttefarde (2012), ROMANSY](https://www-sop.inria.fr/coprin/PDF/a_romansy2012.pdf): encoder redundancy and joint geometry/pose calibration; not an HP3 validation.
- [Maye et al. (2016), DOI 10.1177/0278364915596232](https://journals.sagepub.com/doi/10.1177/0278364915596232): information selection and treatment of unobservable directions.

- [Tooling diagnosis](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E09-sandbox-and-tools.json): sandbox EPERM, healthy supervised services and proposed full-auto job mitigation.

- [Three implementation prompts](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/../../../../research/README.md#local-networking-for-full-auto-runs): scoped full-auto, world-configuration and preflight/test tasks; full-autocal jobs implemented, world-configuration and integration-test tools still proposed.
- [E10 launcher tests](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E10-launcher-tests.txt): twelve passing configuration/override checks, not live networking validation.

- [Settling fix and commit review](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/../../../../research/autocal-settling-review.md): E11 reproduction, P1 correction, checks and E12 limits.
- [Preceding supervised-job report](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/../2026-10-10-supervised-full-autocal/report.md): retained failure and manual-force lifecycle acceptance; its disabled flags did not prove correct unlayered geometry.

## [2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/)

[abstract.md](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/abstract.md) · [report.md](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/report.md) · [research.md](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/research.md)

### Covariance metrics for encoder based CDPR autocalibration

Status: complete; no production calibration algorithm change was justified.

#### Abstract

This study asks whether covariance metrics from Caverly et al.'s online CDPR self-calibration paper can improve hp-sim5's encoder-based autocalibration ranking. The full recorded regression matrix was replayed after fixing the regression tool's module invocation: all 11 outputs match their stored references exactly, while reproducing 20 adjacent score/physical-error disagreements across nine fixtures. Selected-history anchor-sum regret ranges from 0 to 163.828 mm. A heuristic that weighted the existing ellipse-feature information by point count and inverse encoder variance selected the physically best available history in only 2/11 fixtures and sometimes increased anchor RMSE substantially. Candidate planning itself took 0.039–0.819 s per fit in a 33/90-candidate audit, so it does not explain the tens-of-seconds replay runtime. The paper's PDOP/ODOP depend on joint pose/shared-bias covariance and cannot be transplanted as anchor covariance. No calibration selector change was retained; only the regression launcher was repaired. Evidence is simulated recorded data, not hardware.

#### Topics

CDPR, Hangprinter, encoder autocalibration, covariance, PDOP, ODOP, D-optimal design, ranking regret, runtime

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Current score/physical disagreements reproduce on current source | E01 | Supported: 20 disagreements, 9/11 fixtures | [Full report](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/report.md), replay output under ignored `.regress_parallel_runs` |
| Simple point-count/noise weighted covariance improves history ranking | E03 | Rejected: physically best candidate in 2/11; several large regressions | [Covariance audit](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/E03-weighted-covariance-audit.json) |
| Candidate ranking cost explains slow full-auto replay | E04 | Rejected for tested saved fits: 0.039–0.819 s per fit | [Planning cost audit](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/E04-plan-cost-audit.json) |
| Paper's pose covariance is equivalent to existing anchor information | E02 | Rejected: different estimator and estimated quantity | [Paper PDF](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/../../../../autocal/papers/Covariance_based_data_quality_assessment_metrics.pdf), [analysis](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/report.md) |

#### References

- [Full report](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/report.md)
- [Research notebook](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/research.md)
- Caverly, Cheah, Bunker, Patel, Sexton & Nguyen (2024; published online 2024, issue 2025), “Online Self-Calibration of Cable-Driven Parallel Robots Using Covariance-Based Data Quality Assessment Metrics,” DOI [10.1115/1.4065236](https://doi.org/10.1115/1.4065236); [local PDF](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/../../../../autocal/papers/Covariance_based_data_quality_assessment_metrics.pdf). Supports the paper method distinction and its covariance-based PDOP/ODOP claims.
- [Current regression runner](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/../../../../autocal/tools/regress_calibration_logs.py): fixture ground truth and score/accuracy audit.
- [Earlier ranking and cache study](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2c84dbc6/report.md): controlled 11-fixture baseline, recovery fragility and 20 adjacent disagreements.
- [Earlier ranking negative results](sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md): held-out prediction, age score, bounded updates and active movement failures.

## [2026-10-10-supervised-full-autocal](sessions/2026-10-10-supervised-full-autocal/)

[abstract.md](sessions/2026-10-10-supervised-full-autocal/abstract.md) · [report.md](sessions/2026-10-10-supervised-full-autocal/report.md) · [research.md](sessions/2026-10-10-supervised-full-autocal/research.md)

### Supervisor-owned HP3 full-autocal through restricted MCP clients

Status: complete for tooling validation; calibration quality remains concerning.

#### Abstract

This session validates launcher-owned asynchronous full-autocal jobs through MCP with client socket/connect syscalls denied by Linux seccomp. A fresh HP3 production JS/RRF job with known 30 mm radii, zero buildup, no layering and manual 0.01/0.23/9.2 N forces automatically accepted at iteration six through the default patience rule. It collected eight sweeps and 320 physical measurements over 4198.148 simulated seconds and 780.154 wall seconds. The finalized 1.13 GB RRD contains 209,908 physical samples, all 320 measurements, six collection completions and exact final dataset/log/stage artifacts. The attached HP4 world stayed at step zero. Physical cancellation, HTTP/SIGTERM shutdown, owner SIGKILL and physics-budget exhaustion retained readable partial recordings. An earlier default-autotuning trial failed pullout settling and is preserved. The selected fit score was 6018, which the CLI calls concerning. This establishes complete workflow access and recording integrity, not calibration accuracy or hardware behavior. A direct Codex chat attachment was unavailable; real restricted SDK stdio MCP was exercised.

#### Topics

autocalibration, HP3, HP4, supervisor, MCP, cancellation, process groups, Rerun, sandbox

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Socket-denied client can start and inspect real jobs through MCP | a792904dd547473ca0f6583f5ba547b9, 2ec0b06774624962a18bdbd2818eb134 | Supported | [First start](sessions/2026-10-10-supervised-full-autocal/mcp-start.json), [retry start](sessions/2026-10-10-supervised-full-autocal/mcp-start-2ec0b06774624962a18bdbd2818eb134.json) |
| Supervised physical cancellation, shutdown, crash and budgets finalize partial recordings | lifecycle trials | Supported | [Retained lifecycle evidence](sessions/2026-10-10-supervised-full-autocal/lifecycle/) |
| Default autotuned HP3 zero-buildup bootstrap completes | a792904dd547473ca0f6583f5ba547b9 | Rejected for this trial | [Failed manifest](sessions/2026-10-10-supervised-full-autocal/native/autocal/a792904dd547473ca0f6583f5ba547b9/job.json) |
| Bounded manual-force HP3 job physically reaches automatic acceptance | 2ec0b06774624962a18bdbd2818eb134 | Supported for workflow completion; quality concerning | [Acceptance](sessions/2026-10-10-supervised-full-autocal/acceptance.json), [RRD verification](sessions/2026-10-10-supervised-full-autocal/rrd-verification.json), [status](sessions/2026-10-10-supervised-full-autocal/mcp-status-2ec0b06774624962a18bdbd2818eb134.json) |

#### References

- [Research notebook](sessions/2026-10-10-supervised-full-autocal/research.md): objective and budgets.
- [Full report](sessions/2026-10-10-supervised-full-autocal/report.md): pending validation results.
- [Implementation request](sessions/2026-10-10-supervised-full-autocal/../../../../research/prompts/tooling_improvement_1.txt): acceptance requirements.
- [Earlier force investigation](sessions/2026-10-10-supervised-full-autocal/../../autocal-comfortable-force-20261010/report.md): preceding force-tuning limitations; the new default-autotuning trial still failed settling.
- [Compatibility regressions](sessions/2026-10-10-supervised-full-autocal/../../../../autocal/data/references/fifth_hp3_dataset.full_auto_reference_run_october_8_2026.log) and [seventh HP3 reference](sessions/2026-10-10-supervised-full-autocal/../../../../autocal/data/references/seventh_hp3_dataset.full_auto_reference_run_october_8_2026.log): unchanged solver outputs.

## [2026-10-12-configurable-research-worlds-fee56c1a](sessions/2026-10-12-configurable-research-worlds-fee56c1a/)

[abstract.md](sessions/2026-10-12-configurable-research-worlds-fee56c1a/abstract.md) · [report.md](sessions/2026-10-12-configurable-research-worlds-fee56c1a/report.md) · [research.md](sessions/2026-10-12-configurable-research-worlds-fee56c1a/research.md)

### Configurable HP3/HP4 research worlds through MCP

Status: complete.

#### Abstract

MCP world configuration was tested against production scene construction and the existing RRF collector. Tests validated HP4-to-HP3 configuration, layering-off geometry, reset retaining settings, source reload, active-job rejection and failed-start recovery. The final HP3 collection with known 30 mm spool radii, zero buildup and layering disabled produced six measurement points in 6.605 seconds wall time. Baked cable half-widths were zero; A/B encoder ranges were 182.975/183.026 degrees. The dataset passed schema and finite residual-ingestion checks. An earlier zero-step failed trial exposed the incorrect collector profile and RRF host path truncation; both were corrected and the failed evidence retained. This establishes simulated configuration/collection behavior, without demonstrating calibration accuracy, hardware transfer or rendered-browser collection parity.

#### Topics

HP3, HP4, MCP, world configuration, layering, scene baking, source reload, encoder origin, RRF, collection, lifecycle

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Layering-off geometry matches production construction | E1, E4 | Supported | [Report](sessions/2026-10-12-configurable-research-worlds-fee56c1a/report.md), [baked scene](sessions/2026-10-12-configurable-research-worlds-fee56c1a/native-final/2eab243afbdd4c06aa4ce36cdea60ad0/scene.usda) |
| Reset preserves settings and coherent generation identity | E1, E3, E4 | Supported | [Statuses](sessions/2026-10-12-configurable-research-worlds-fee56c1a/reset-status.json), [tests](sessions/2026-10-12-configurable-research-worlds-fee56c1a/../../../../tests/python/test_world_configuration.py) |
| Existing HP3 collector produces actual encoder movement and ingested measurements | E2, E3, E4 | Supported after failed caller trial | [Dataset](sessions/2026-10-12-configurable-research-worlds-fee56c1a/native-final/2eab243afbdd4c06aa4ce36cdea60ad0/7c7c2180a90348e783d73aae2c5367a1/sweeps.json), [failed manifest](sessions/2026-10-12-configurable-research-worlds-fee56c1a/native/c9bd24bcb3514e8b8cfb6c4f8838a355/62b3db82e9384f7f8ba390bc8579abfd/manifest.json) |
| Calibration accuracy improves | None | Not tested | No fitting or held-out accuracy trial |

#### References

- [Full report](sessions/2026-10-12-configurable-research-worlds-fee56c1a/report.md): measured outcomes and limitations.
- [Research notebook](sessions/2026-10-12-configurable-research-worlds-fee56c1a/research.md): decisions, budgets and preserved failures.
- [E4 manifest](sessions/2026-10-12-configurable-research-worlds-fee56c1a/native-final/2eab243afbdd4c06aa4ce36cdea60ad0/7c7c2180a90348e783d73aae2c5367a1/manifest.json): source/input hashes and generation provenance.
- [Verification](sessions/2026-10-12-configurable-research-worlds-fee56c1a/verification.md): test commands and results.

## [2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/)

[abstract.md](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/abstract.md) · [report.md](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/report.md) · [research.md](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/research.md)

### Stable encoder averaging and numerical fragility in HP3 autocalibration

Status: complete.

#### Abstract

This investigation asked how to improve encoder-only HP3 autocalibration with known radii, no line layering and automatic stopping. Profiling the supplied recording found settling dominant, and frozen candidates reproduced score/geometry disagreement. Sparse sphere-closure and initializer trials did not justify fewer-point defaults. A return shortcut saved time in bounded trials but worsened full-auto collection. Original noise samples exposed accumulation error in 413 of 640 encoder mean fields. Generic centered averaging reduced this to two fields and made successful/failed recorded 140-point inputs return the identical constrained 2.4937 mm anchor-RMSE fit. The retained eight-line averaging change finished automatically with 200 points, 5659.096 simulated seconds and 2.3843 mm anchor RMSE. Its unpaired total is 16.24% shorter and its anchor error 15.61% lower than the supplied reference; different adaptive stopping paths prevent attributing this to faster collection. Against the fresh current baseline it uses 25% more points and 12.93% more time for 5.79% less anchor error. The return shortcut was removed. Evidence is simulated or analytical on one stock HP3 geometry; general fewer-point reliability, score-based accuracy and hardware transfer remain unresolved.

#### Topics

HP3, CDPR, encoder-only autocalibration, centered averaging, numerical sensitivity, underconstrained sentinel, return-to-origin, sparse points, candidate ranking, automatic stopping, patience, fixed-budget benchmarking, Rerun profiling, research harness

#### Tested hypotheses

| Tested hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Fit scores reliably rank known geometry | E01, E02, E12 | Rejected on reference; fresh baseline final validation selects its best fit | [Reference timing/ranking](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E01-profile-and-geometry.json), [grid/anchor audit](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E12-geometry.json) |
| Lost fixed-encoder variation explains the reference failure | E01 | Rejected for this dataset: variation below 1e-6 mm | [Fixed-encoder audit](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E01-fixed-encoder-audit.json) |
| Sparse sphere closure robustly replaces current fitting | E03, E05 | Rejected as a default | [48 recorded fits](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E03-closure-pilot.json), [36 synthetic fits](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E05-synthetic-closure.json) |
| Closure initialization enables three points per direction | E04 | Rejected: all four sparse prefixes return the sentinel | [Eight initializer trials](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E04-seed-pilot.json) |
| Faster individual returns improve full-auto collection | E06, E09, E07b, E19, E24 | Rejected as default; positive local timings do not yield a net gain | [Bounded returns](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E09-physical-returns.json), [return-only failure](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E13-live-comparison.json), [mean-only control](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E26-mean-only-comparison.json) |
| Near-identical encoder means can change recovery and collection | E15 | Supported on paired recorded inputs | [Six numerical controls](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E15-numeric-pilot.json) |
| Centered averaging improves mean accuracy and fixes the paired sentinel failure | E16–E18 | Supported on captured samples and paired fits | [Original sample audit](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E16-mean-audit.json), [paired generic averages](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E17-stable-mean-pilot.json), [arithmetic tests](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E18-mean-tests.txt) |
| Centered averaging improves every intermediate geometrical fit | E20 | Rejected; mixed prefix outcomes | [Five additional prefixes](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E20-stable-prefixes.json) |
| Averaging alone improves the complete one-click workflow | E22, E24, E26 | Mixed: better geometry, but more points/time than fresh baseline | [Stored replay](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E22-replays/stable.log), [fresh comparison](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E26-mean-only-comparison.json) |
| Stored-data fitting remains compatible | E11 | Supported: all 11 regressions exactly equal | [Regression log](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E11-regressions.txt) |
| Continuous acquisition, task-space precision or pointwise fixed geometry improves calibration | none | Not tested | [Initial hypotheses](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/research.md) |

#### References

- [Full report](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/report.md): measurements, implementation and limitations.
- [Notebook](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/research.md): objective, hypotheses, budgets, steering, experiment decisions and failures.
- [Manifest](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/manifest.json), [retained diff](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/final-source.diff), [return-only diff](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E07-return-only-source.diff), [combined diff](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E19-combined-source.diff): baseline revision `45862559e95af66a5ed749540763d41d859f5b7e`, frozen code/input hashes and which changes survived.
- [Supplied reference](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/../../../extended-autocal/no-buildup-reference-003/), its [RRD](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/../../../extended-autocal/no-buildup-reference-003/hangprinter-20261010T182010.757112Z-814ac4d9.rrd), [dataset](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/../../../extended-autocal/no-buildup-reference-003/sweeps.json) and [stage snapshots](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/../../../extended-autocal/no-buildup-reference-003/sweeps.stages/): E01/E12 recorded evidence at revision `1e026534483e2175f4c101b645b1f54f65f9faba`.
- [Fresh baseline manifest](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/native/output/research/sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E02-baseline-live/sweeps.headless/manifest.json), [return-only manifest](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E07b-candidate-live/sweeps.headless/manifest.json), [combined manifest](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/native/E19-stable-mean-live/sweeps.headless/manifest.json), [retained averaging-only manifest](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E24-mean-only-live/sweeps.headless/manifest.json): used HP3 configuration, effective layering, geometry, firmware, timings and applied M669.
- [Independent RRD verification](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E14-rrd-verification.json): measurement counts, encoder means, frozen scene/config hashes, physical stop and finalized recordings; tool/recording integrity does not establish calibration accuracy.
- [Prior settling study](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/report.md): optimized point waits and endpoint recovery already in the current baseline.
- [Prior thinning/ranking study](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-130caf5a/report.md), [covariance study](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76802a07/report.md), [bounded-update study](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/../b7deb1bcc9cb418388273e2b7b9adc8c/report.md): preceding negative results narrowed this study; their experiments are not counted as new results.
- Yuan, Han; You, Xianghui; Zhang, Yongqing; Zhang, Wenjing; Xu, Wenfu (2019), “A Novel Calibration Algorithm for Cable-Driven Parallel Robots with Application to Rehabilitation,” DOI [10.3390/app9112182](https://www.mdpi.com/2076-3417/9/11/2182): redundant encoder-length least squares motivates the closure pilot; ideal simulated accuracy does not establish noisy HP3 performance.
- Caverly, Ryan J.; Cheah, Sze Kwan; Bunker, Keegan R.; Patel, Samir; Sexton, Niko; Nguyen, Vinh L. (2025), “Online Self-Calibration of Cable-Driven Parallel Robots Using Covariance-Based Data Quality Assessment Metrics,” DOI [10.1115/1.4065236](https://experts.umn.edu/en/publications/online-self-calibration-of-cable-driven-parallel-robots-using-cov/): institutional abstract motivates covariance-guided data retention and stopping; PDOP/ODOP are distinct from the current anchor covariance.
- Alexandre dit Sandretto, Julien; Daney, David; Gouttefarde, Marc (2012), “Calibration of a Fully-Constrained Parallel Cable-Driven Robot,” ROMANSY, [author PDF](https://www-sop.inria.fr/coprin/PDF/a_romansy2012.pdf): redundancy-based joint geometry/pose self-calibration motivates latent-pose elimination; its embodiment differs from HP3.
- [Current algorithm](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/../../../../autocal/optimization_algorithm_overview), [objectives](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/../../../../autocal/objective_functions_overview), [ellipse guide](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/../../../../autocal/README_elliptical_feature_calibration.md), [extended recording guide](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/../../../../autocal/data/references/extended-reference.md): solver, collector, selection and recording contracts actually inspected.


[Benchmarking and harness review](sessions/2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/tooling-review.md) responds to the user's stopping-policy correction. Full-run totals are descriptive policy outcomes, not isolated collection-speed estimates. Proposed fixed-work, common-budget and tool improvements are not tested. No new experiments were run.

## [2026-10-12-network-enabled-preflight-revalidation-31a423c7](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/)

[abstract.md](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/abstract.md) · [report.md](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/report.md) · [research.md](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/research.md)

### Network-enabled preflight revalidation and lifecycle fixes

Status: complete.

#### Abstract

After command networking was enabled, socket creation, loopback binding and HTTP all passed in the actual command context in 0.013 seconds. Further testing reproduced two bugs: preflight accepted Cartesian firmware using cached startup identity, and detached-service cleanup missed a grandchild after its group leader was reaped. The fixes check live read-only firmware kinematics/anchors and verify surviving process ownership with private job markers. Final direct regressions passed 165 tests (16 deselected), plus three live-launcher tests. Named supervisor suites passed 26 and 138 tests through a deliberately socket-denied SDK stdio client. Healthy MCP preflight took 0.738 seconds, CLI preflight 0.840 seconds. Real graceful and signal shutdown retired services from running test jobs. The attached HP4 world stayed at step zero and retained its recording. This is Linux tooling evidence, not hardware or calibration-accuracy validation; a live Codex research MCP attachment remains unavailable.

#### Topics

preflight, networking, firmware compatibility, anchors, process groups, reaped leader, test jobs, recorder, shutdown

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Actual command socket/bind/HTTP now work | command-network | Supported | [Probe](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/command-network.json) |
| Cached startup identity detects live incompatible firmware | firmware-reproduction | Rejected before fix | [Cartesian while preflight passed](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/firmware-mismatch-reproduction.json) |
| Cleanup retires grandchildren after detached leader is reaped | reaped-leader | Rejected before fix; supported after fix | [Failed trial](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/failed-cleanup-reproduction.txt), [fixed regression JUnit](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/direct-regressions.xml) |
| Live kinematics/anchor drift is detected without world movement | live-mcp-preflight | Supported after fix | [Regression JUnit](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/direct-regressions.xml), [supervised research job](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/research-services-result.json) |
| Graceful and signal shutdown stop test-owned socket services | live-supervisor-shutdown | Supported | [Regression JUnit](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/direct-regressions.xml) |
| A stale registry can retire an unrelated live group | stale-registry | Rejected | [Regression JUnit](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/stale-registry.xml) |
| Updated suites remain available from a socket-denied SDK client | 6c2742ced4524fc3b19ed1e47053fd2a, 3afc3c2920024414ba40b3c9bc4fbf29 | Supported | [Headless job](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/headless-autocal-result.json), [research job](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/research-services-result.json) |

#### References

- [Notebook](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/research.md): objectives, constraints and decisions.
- [Full report](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/report.md): exact commands, outcomes and limitations.
- [Previous validation](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/../2026-10-12-preflight-and-supervised-integration-tests-718724fb/report.md): baseline evidence and archived pre-fix source.
- [Acceptance harness](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/verify.py): real supervisor/MCP, reuses the existing seccomp client restriction.
- [Source hashes](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/source-hashes.json): snapshots of every changed/new file under test.
- [Feature prompt](sessions/2026-10-12-network-enabled-preflight-revalidation-31a423c7/../../../../research/prompts/tooling_improvement_3.txt): original requirements.

## [2026-10-12-preflight-and-supervised-integration-tests-718724fb](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/)

[abstract.md](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/abstract.md) · [report.md](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/report.md) · [research.md](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/research.md)

### Preflight and supervised integration tests with command sockets denied

Status: complete.

#### Abstract

Can agents check recording/connectivity and run socket-dependent integration tests without command networking? Implemented bounded non-moving preflight and allowlisted supervisor test jobs using the existing runtime. A real SDK stdio MCP client denied socket/connect syscalls by Linux seccomp completed preflight in 0.737 seconds, including an isolated recorder acknowledgment, finalization and readable RRD. The final headless/recorder suite passed 26 tests and research-services passed 130 tests. Invalid suites, mismatched settings, disconnected services, cancellation, one-second timeout and detached-child retirement were checked; three failed development trials remain available. The attached HP4 world stayed at step zero with its recording unchanged. The command sandbox itself denied socket creation with EPERM; its HTTP capability remains unknown. CLI preflight passed in 0.736 seconds. This validates tooling on Linux, not calibration accuracy or hardware behavior. A live Codex chat attachment with network_access=false was not available; the socket-denied SDK path and launcher override tests were exercised.

#### Topics

preflight, supervisor, socket policy, integration tests, recorder finalization, autocal, cancellation, process groups

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Agent command context permits localhost sockets | command-network | Rejected | [Actual probe](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/command-network.json) |
| Supervisor preflight works with client socket calls denied | b042c5e15bed4c5a8a4506afde7e76a2 | Supported | [Preflight](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/mcp-preflight.json), [denied client](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/restricted-client-network.json) |
| Fixed suites pass through supervised MCP jobs | 6d55ab3885944680b1450857a0e33ca2, 6b1a474b446547839d1f457f997eeb2a | Supported | [Headless suite](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/headless-autocal-result.json), [research suite](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/research-services-result.json) |
| Cancel/timeout/disconnection report clear outcomes and preserve the attached world | b89593a989274378ba10f5f5445d9d38, 5c810c59adb24cccb521e90541cac51b | Supported | [Cancellation](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/mcp-cancelled.json), [timeout](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/mcp-timeout.json), [disconnection](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/disconnected-supervisor.json), [summary](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/acceptance.json) |
| CLI preflight leaves step zero without Viewer or Codex startup | cli-preflight | Supported | [CLI result](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/cli-preflight.stdout.json) |
| Tool success establishes calibration accuracy | none | Not tested | No accuracy experiment requested |

#### References

- [Report](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/report.md): exact jobs, test results, failure history and limits.
- [Notebook](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/research.md): objective, budgets and decisions.
- [Acceptance harness](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/verify.py): uses real supervisor and MCP, not a replay or synthetic collector.
- [Implementation prompt](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/../../../../research/prompts/tooling_improvement_3.txt): requirements.
- [Previous socket-denied client validation](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/../2026-10-10-supervised-full-autocal/verify.py): reused Linux seccomp restriction helper.
- Base repository revision `06f190dde4626998f21e75c7634797641fa48fc2` plus this working tree: code under test; [working diff](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/working.diff) records tracked changes; [source hashes and snapshots](sessions/2026-10-12-preflight-and-supervised-integration-tests-718724fb/source-hashes.json) preserve every changed/new source file.

## [2026-10-13-headless-js-profiling](sessions/2026-10-13-headless-js-profiling/)

[abstract.md](sessions/2026-10-13-headless-js-profiling/abstract.md) · [report.md](sessions/2026-10-13-headless-js-profiling/report.md) · [research.md](sessions/2026-10-13-headless-js-profiling/research.md)

### Headless JavaScript physics profiling

Status: complete.

#### Abstract

The production headless JavaScript HP3/HP4 physics was measured on fixed firmware logo trajectories and complete recorded autocal movement/settling schedules, with layering enabled and disabled. Eight cases used three uninstrumented repetitions and separate system-timing, native V8 CPU and allocation-sampling runs: 48 measured trajectories and 7,544,232 physics updates. Every profiler mode matched its baseline numerical checkpoints exactly. The constraint solver, attachment updates and over-correction dominate runtime. Allocation is substantial, but reported GC contributes about 1–2% of replay time. Layering removal speeds the logo workload but gives little benefit on the autocal schedules and can change the physical trajectory substantially. The faithful HP3 autocal replay also reproduces 1,956 saved encoder reads to floating-point rounding. These results establish a reproducible baseline; no physics optimization was made. Measurements exclude firmware, transport, rendering and recording costs, and frozen settling schedules cannot establish adaptive collector or hardware performance.

#### Topics

HP3, HP4, JavaScript, headless, performance, profiling, V8, allocation, garbage collection, autocal settling, layering.

#### Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Attachment is always the largest system | P01: eight system profiles | Rejected | Solver leads seven of eight cases |
| Temporary allocation is substantial | P02: eight native allocation profiles | Supported | Hundreds of KiB to more than 1 MiB allocated/update |
| GC is the dominant current bottleneck | P02: GC events and CPU profiles | Rejected | Reported GC approximately 1–2%; solver/attachment math dominates |
| Disabling layering consistently speeds this simulation | P03: all on/off pairs | Mixed | Logo improves; autocal changes little, with physical divergence |
| The profiling modes preserve numerical checkpoints | P04: 48 measured trajectories | Supported | Exact checkpoint hashes within every case |

#### References

- [Full report](sessions/2026-10-13-headless-js-profiling/report.md): results and scope.
- [Research notebook](sessions/2026-10-13-headless-js-profiling/research.md): constraints and experiment choices.
- [Matrix](sessions/2026-10-13-headless-js-profiling/matrix.json): measured numerical/timing evidence.

## [2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76fad208](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76fad208/)

[abstract.md](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76fad208/abstract.md) · [research.md](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76fad208/research.md)

### Improve encoder-based CDPR autocalibration in hp-sim5. The current algorithm can

Status: not yet summarized.

#### Abstract

The session has started. No research result has been summarized yet.

#### Topics

Improve encoder-based CDPR autocalibration in hp-sim5. The current algorithm can

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| None recorded yet | None | Not tested | None |

#### References

- [Original task](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76fad208/prompt.txt)
- [Research notebook](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-76fad208/research.md)

## [2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-881b7eb9](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-881b7eb9/)

[abstract.md](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-881b7eb9/abstract.md) · [research.md](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-881b7eb9/research.md)

### Improve encoder-based CDPR autocalibration in hp-sim5. The current algorithm can

Status: not yet summarized.

#### Abstract

The session has started. No research result has been summarized yet.

#### Topics

Improve encoder-based CDPR autocalibration in hp-sim5. The current algorithm can

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| None recorded yet | None | Not tested | None |

#### References

- [Original task](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-881b7eb9/prompt.txt)
- [Research notebook](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-881b7eb9/research.md)

## [2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/)

[abstract.md](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/abstract.md) · [report.md](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/report.md) · [research.md](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/research.md)

### Faster, more robust encoder-only autocalibration for a Hangprinter 3: a shorter encoder-quiet window plus a bracketed geometric consensus of fitted anchors

Status: complete (all planned runs finished or reported; evidence is simulated).

#### Abstract

Encoder-only autocalibration infers anchor geometry from cable-length
measurements, so its two user-facing costs are collection time and the risk of
applying a fit that scattered. Using a supervised HP3 headless run as reference
(9 sweeps x 20 points, 6756 simulated seconds, applied-anchor RMSE 2.828 mm) we
profiled where simulated time goes, replayed 817 recorded encoder-quiet waits,
and evaluated anchor-selection and stopping rules over candidate sets persisted
by 40 real full-auto runs. Two changes are shipped. First, the encoder-quiet
window drops from 1500 ms to 1000 ms, saving 528 s (7.8%) of simulated
collection time with a p99 recorded angle change of only 0.12 degrees. Second,
the applied geometry becomes the mean of the closest pair of distinct fitted
anchor sets while the existing ranked score stays the reported diagnostic; over
11 de-duplicated real candidate sets this cuts mean applied error from 4.273 mm
to 3.092 mm and the worst case from 10.060 mm to 4.353 mm. A third end-to-end
run at 6 points/sweep (E18) was accepted at 2.192 mm from 120 data points and
5032 simulated seconds - the best of four runs - but the replay corpus trends
slightly the other way, so the default point density is left unchanged.
Limitation: every number here is simulated headless JS physics, not a hardware
measurement.

#### Topics

autocalibration, CDPR, Hangprinter 3, encoder-only calibration, anchor selection,
candidate ranking, held-out sweeps, encoder settle window, collection time,
early stopping, robustness, simulated headless physics.

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| The encoder-quiet wait loop dominates collection time, and a 1000 ms window is its largest safe reduction | E01, E01b, E07, E16, E16b | Supported (shipped) | [E07 / E16b](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/report.md#the-quiet-window-e07-e16-e16b) |
| Relaxing the quiet tolerance or the vibration window buys more time safely | E16 | Rejected | [E16](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/report.md#negative-and-unshipped-results) |
| More data points improve accuracy | E05 | Rejected | [E05](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/report.md#more-points-do-not-help-e05) |
| Fewer points per sweep is a proportional time saving with no accuracy cost | E13, E18, E37, replay corpus | Mixed / rejected as a lever | [E18](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/report.md#fewer-points-per-sweep-e13-e18) |
| Fewer points per sweep keeps end-to-end accuracy | E13, E14b, E18 | Mixed (best single run, inside run-to-run noise) | [E18](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/report.md#fewer-points-per-sweep-e13-e18) |
| The shipped selection score disagrees with geometric accuracy, and the newest iteration is structurally unselectable | E14b, E15, E19, E33 | Supported | [E19 / E33](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/report.md#selecting-better-anchors-e19-e33-e35) |
| Replacing the ranking score fixes selection | E14b (this session); prior sessions | Already rejected; not retried | [prior session](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md) |
| The geometric consensus of the fitted candidates is closer to truth than the best-ranked single candidate | E33, E35, E34, E36 | Supported (shipped) | [E33 / E35](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/report.md#selecting-better-anchors-e19-e33-e35) |
| Any ensemble of candidates is a safe selection rule | E19-E32 | Rejected | [E19-E32](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/report.md#negative-and-unshipped-results) |
| Single fits are stable enough to trust without cross-candidate evidence | E38 | Rejected | [E38](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/report.md#single-fits-are-unstable-e38) |
| Consensus stability can stop collection early without losing accuracy | E39 | Rejected | [E39](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/report.md#early-stop-by-consensus-stability-e39) |

#### References

- [Full report](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/report.md): analysis, measurements, shipped diff and limitations.
- [Research notebook](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/research.md): objectives, budgets, steering and the E01-E39 log.
- Local evidence: reference run `output/extended-autocal/no-buildup-reference-003`
  (the run to beat); supervised runs `native/autocal/E05-baseline-head`,
  `E17-window1000-refdensity`, `E18-sweeppoints6`; offline decks `E19`-`E39` in
  this directory. Repository revision `ded5842e`.
- Prior sessions: [held-out history ranking](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/../autocal-held-out-history-ranking-7ec4260f/report.md),
  [bounded rank updates](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md),
  [candidate model validation](sessions/2026-10-13-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b65aa69c/../autocal-candidate-model-validation-and-throughput-fdb616a4/report.md).

## [2026-10-13-pbd-constraint-hotspot](sessions/2026-10-13-pbd-constraint-hotspot/)

[abstract.md](sessions/2026-10-13-pbd-constraint-hotspot/abstract.md) · [report.md](sessions/2026-10-13-pbd-constraint-hotspot/report.md) · [research.md](sessions/2026-10-13-pbd-constraint-hotspot/research.md)

### PBD constraint hotspot optimization

We tested whether settled-constraint skipping and per-update invariant caching can speed up the JavaScript 3D PBD solver without changing physics. Across frozen HP3/HP4 logo and autocal replays, the retained endpoint/path cache matched all checkpoints exactly and reduced median whole-physics time by 1.2–3.6% in the final batch. An A/B/A source isolation showed the later compliance fallback did not cause the smaller result; run-condition variability remains the likely but unverified explanation. Active-set skipping regressed timing, and scratch-vector reuse added no reliable gain. Frozen replays do not validate adaptive collector behavior or hardware accuracy.

## [2026-10-13-three-optimizations](sessions/2026-10-13-three-optimizations/)

[abstract.md](sessions/2026-10-13-three-optimizations/abstract.md) · [report.md](sessions/2026-10-13-three-optimizations/report.md) · [research.md](sessions/2026-10-13-three-optimizations/research.md)

### Cumulative validation of three headless physics optimizations

Status: complete.

#### Abstract

This experiment checked the cumulative effect of three JavaScript simulation optimization commits on exact trajectory behavior and unprofiled physics timing. Frozen HP3/HP4 logo and autocal cases matched exact baseline checkpoints; cumulative median physics time fell 6.7–16.0%, with a 9.0% median across the eight workloads. Six other cataloged 3D machine designs—Four High Anchors, CubeCorners, and four Slideprinter variants—also matched the unoptimized `0cfe920c` checkout exactly through 200 production-pipeline steps with small commanded spool moves. The timing result is a single-host, fixed-prefix benchmark; no separate performance timings were collected for those six designs.

#### Topics

HP3, HP4, headless simulation, exact trajectory checkpoints, component-store lookup, attachment transforms, performance.

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| The three cumulative optimizations preserve exact trajectory checkpoints and reduce median physics time across the eight frozen workloads. | CUM-1 | supported | [Cumulative report and matrix](sessions/2026-10-13-three-optimizations/report.md), [raw matrix](sessions/2026-10-13-three-optimizations/cumulative-matrix.json). |
| The three optimizations preserve commanded physics behavior in all other 3D machine designs in the app catalog. | CUM-2 | supported | [Cumulative report](sessions/2026-10-13-three-optimizations/report.md), [per-scene exact snapshots](sessions/2026-10-13-three-optimizations/non-hp-scenes.json). |

#### References

- [Full report](sessions/2026-10-13-three-optimizations/report.md): cumulative measurements and limitations.
- [Research notebook](sessions/2026-10-13-three-optimizations/research.md): objective, constraints and experiment status.
- [Original profiling study](sessions/2026-10-13-three-optimizations/../2026-10-13-headless-js-profiling/report.md): frozen inputs and numerical baseline.
- [Attachment reuse study](sessions/2026-10-13-three-optimizations/../2026-10-13-attachment-reuse/report.md): evidence for the third optimization commit's two stages.
- [Headless profiling recipe](sessions/2026-10-13-three-optimizations/../../../../tests/benchmark/HEADLESS_JS_PROFILING.md): runner behavior and comparison protocol.

## [2026-10-14-native-throughput](sessions/2026-10-14-native-throughput/)

[abstract.md](sessions/2026-10-14-native-throughput/abstract.md) · [report.md](sessions/2026-10-14-native-throughput/report.md) · [research.md](sessions/2026-10-14-native-throughput/research.md)

### Native Rust headless throughput versus Wasm and Python

Status: complete.

#### Abstract

Native Rust commit ab833759 was benchmarked against the exact preceding Rust/Wasm headless default at 7905e64f, and against the former Python default for fresh research trials. Three sequential repetitions covered HP3/HP4, layering on/off, frozen logo and autocal schedules, continuing worlds, every-step fresh telemetry, real six-point firmware collections, and the standalone autocal service with and without recording. Native shared-pipeline throughput improved 1.145–1.235×, with an aggregate ratio of 1.181×. Recording reduced end-to-end gains; short real firmware collections showed only 1.02–1.03× without recording and approximately 0.99× with recording. Fresh numerical trials improved 3.11–7.17× over Python; recorded trials improved 1.46–2.12×. Repeats were deterministic, but five long physics comparisons exceeded the declared pointwise tolerance. These host measurements do not establish calibration accuracy or complete autocal optimization speedup.

#### Topics

Rust, native, Wasm, headless, throughput, HP3, HP4, Rerun, autocal, recording, firmware

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Native improves shared-pipeline throughput. | N01 | Supported for fixed schedules; numerical agreement mixed | [Physics matrix](sessions/2026-10-14-native-throughput/physics/matrix.json) |
| Recording/transport reduce gains. | N02, N05 | Supported | [Consumer matrix](sessions/2026-10-14-native-throughput/consumer-matrix.json), [service matrix](sessions/2026-10-14-native-throughput/autocal-service-matrix.json) |
| Fresh trials improve over Python. | N03 | Supported for 200-step trials | [Consumer matrix](sessions/2026-10-14-native-throughput/consumer-matrix.json) |
| Fresh firmware collections improve. | N04 | Mixed; no useful recorded gain | [Consumer matrix](sessions/2026-10-14-native-throughput/consumer-matrix.json), [measurement audit](sessions/2026-10-14-native-throughput/live-numerics.json) |

#### References

- [Full report](sessions/2026-10-14-native-throughput/report.md): results, numerical drift and limits.
- [Research notebook](sessions/2026-10-14-native-throughput/research.md): decisions and budgets.
- [Source manifest](sessions/2026-10-14-native-throughput/source-manifest.json): measured revisions and compiled artifact.
- [Frozen input origins](sessions/2026-10-14-native-throughput/../2026-10-13-headless-js-profiling/inputs/origins.json): schedules and provenance.
- [Earlier Wasm investigation](sessions/2026-10-14-native-throughput/../2026-10-14-rust-engine/report.md): prior engine change; older timings were not reused.

## [2026-10-14-rust-engine](sessions/2026-10-14-rust-engine/)

[abstract.md](sessions/2026-10-14-rust-engine/abstract.md) · [report.md](sessions/2026-10-14-rust-engine/report.md) · [research.md](sessions/2026-10-14-rust-engine/research.md)

### Fuller Rust simulation engine for hp-sim-3d

Status: complete.

#### Abstract

The full Rust/Wasm engine is 2.37–2.86 times faster than the previous Wasm default
on eight frozen HP3/HP4 logo and autocal workloads. Three repeats per backend
cover 7,544,232 measured updates. Rust owns persistent physics state, winding,
topology, constraints, friction, integration, motors and encoders. JavaScript
retains scene preparation, commands, edits and observations. The same source
builds and steps natively. Short differential checks and bounded one-step forks
agree tightly, but five long feedback cases fail the initial pointwise tolerance.
The layered HP3 autocal replay reaches 2.85 degrees of encoder-read difference;
this is recorded as drift. Repeats remain deterministic within each backend.
These are simulated results from one host, with no hardware or fresh adaptive
calibration accuracy claim.

#### Topics

Rust, Wasm, native physics, hp-sim-3d, HP3, HP4, simulation performance, autocal
replay, floating-point drift, cable winding.

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Persistent Rust physics improves throughput beyond the old kernel | E01 | Supported | [Final matrix](sessions/2026-10-14-rust-engine/matrix-precision/matrix.json) |
| Rust math retains close JS trajectories | E02 | Mixed: short/fork agreement, long feedback drift | [Early forks](sessions/2026-10-14-rust-engine/one-step-forks.json), [late forks](sessions/2026-10-14-rust-engine/late-forks.json), [matrix errors](sessions/2026-10-14-rust-engine/matrix-precision/matrix.json) |
| The same core compiles and steps natively | E03 | Supported | [Native tests](sessions/2026-10-14-rust-engine/../../../../src/wasm/cable_solver/tests/native_engine.rs) |

#### References

- [Report](sessions/2026-10-14-rust-engine/report.md): measurements and limits.
- [Notebook](sessions/2026-10-14-rust-engine/research.md): constraints, decisions and failed trials.
- [Final source manifest](sessions/2026-10-14-rust-engine/final-source-manifest.json): source and binary hashes.
- [Previous solver report](sessions/2026-10-14-rust-engine/../../../../research/rust-wasm-solver.md): prior boundary.
- [Original inputs](sessions/2026-10-14-rust-engine/../2026-10-13-headless-js-profiling/inputs/): frozen workloads.

## [2026-10-15-autocal-recording-modes](sessions/2026-10-15-autocal-recording-modes/)

[abstract.md](sessions/2026-10-15-autocal-recording-modes/abstract.md) · [report.md](sessions/2026-10-15-autocal-recording-modes/report.md) · [research.md](sessions/2026-10-15-autocal-recording-modes/research.md)

### Autocal recording-mode cost on a frozen production trace

Status: complete.

#### Abstract

We measured the time and disk cost of four full-autocal recording modes using
one frozen production HP3 command/encoder schedule. Four sequential repetitions
per mode replayed the same 200,000 fixed steps (400 simulated seconds), 30,758
commands and 1,060 nonempty encoder reads through the current native Rust engine.
All 16 trials had identical full-state checkpoint and encoder hashes; recorded
modes also had identical 34,017-event streams after timestamp/session
normalization and disk readback. Median replay, drain and finalization times were
4.82 s (none), 5.05 s (events), 5.49 s (sampled) and 15.23 s (full); RRD sizes were
0, 3.44, 10.96 and 54.23 MB, plus a shared 6.58 MB journal. These results support
cheaper modes for routine experiments. The replay excludes fresh planning,
adaptive collection, fitting and Python/collector logs. It does not reproduce
the original older engine's encoders: maximum disagreement was 943.55 degrees.
Exact equality was established only among the current mode comparisons.

#### Topics

autocalibration, HP3, recording modes, Rerun, recorder overhead, frozen replay,
performance, experiment design, native Rust

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Removing physics recording reduces time/bytes while preserving current physics and command/encoder evidence | matrix/R1–R4, all modes | Supported for this frozen replay | [Raw matrix](sessions/2026-10-15-autocal-recording-modes/matrix/results.json) |
| Stride-100 sampled recording offers intermediate cost | matrix/R1–R4 sampled/full | Supported for this frozen replay | [Raw matrix](sessions/2026-10-15-autocal-recording-modes/matrix/results.json) |
| The current replay reproduces original older-engine encoders | pilot; matrix/R1–R4 | Rejected; 943.55 degree maximum difference in main workload | [Per-read observations](sessions/2026-10-15-autocal-recording-modes/matrix/R1-none/checkpoints.json), source trace in [input](sessions/2026-10-15-autocal-recording-modes/matrix/input/contract.json) |
| Final harness preserves measured trajectory and event evidence | revalidation/R1, all modes | Supported; same checkpoint/schedule/event hashes as main matrix | [Revalidation](sessions/2026-10-15-autocal-recording-modes/revalidation/results.json) |
| These ratios predict total full-autocal wall time or other-backend outcomes | none | Not tested | Scope excludes fitting, planning and adaptive decisions |

#### References

- [Full report](sessions/2026-10-15-autocal-recording-modes/report.md): methods, result table and interpretation.
- [Raw matrix](sessions/2026-10-15-autocal-recording-modes/matrix/results.json): per-trial timings, bytes, identities, equality gates and native artifact metadata.
- [Frozen input contract](sessions/2026-10-15-autocal-recording-modes/matrix/input/contract.json): exact source/prefix hashes and source manifest beside it.
- [Original accepted source job](sessions/2026-10-15-autocal-recording-modes/../2026-10-10-supervised-full-autocal/native/autocal/2ec0b06774624962a18bdbd2818eb134/sweeps.headless/manifest.json): actual production HP3 schedule; not an equivalent original-engine trajectory or timing baseline.
- [Final harness revalidation](sessions/2026-10-15-autocal-recording-modes/revalidation/results.json): four additional trials reproduce the main matrix hashes after deferring reads to saved response boundaries.
- [Pilot](sessions/2026-10-15-autocal-recording-modes/pilot/results.json): short 20,000-step / 32-command preliminary check; not used for headline numbers.
- [Archived measured harness](sessions/2026-10-15-autocal-recording-modes/harness-v1/autocal_recording_modes.py) and [Node runner](sessions/2026-10-15-autocal-recording-modes/harness-v1/autocal_recording_modes.mjs): exact measured source; main Git base aba370774108cd856e88b524e4b65658021b4d7c with [dirty diff](sessions/2026-10-15-autocal-recording-modes/matrix/working.diff) and hashes in results.
- [Published cost guidance](sessions/2026-10-15-autocal-recording-modes/../../../../research/autocal-recording-performance.md): researcher-facing numbers, scope and reproduction command.
- [Research notebook](sessions/2026-10-15-autocal-recording-modes/research.md): accepted steering, budgets and boundary audit.

## [2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/)

[abstract.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/abstract.md) · [report.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/report.md) · [research.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/research.md)

### History-seeded fit variants for robust encoder autocalibration (HP3, no buildup)

Status: complete.

#### Abstract

Encoder autocalibration repeatedly fits the growing sweep dataset, and a cold
multi-start fit sometimes lands on the cost-cap sentinel or stalls in a bad local
optimum. We reproduced that offline on the supplied 9-sweep HP3 reference
dataset and in live HP3 runs, then implemented a small change: each iteration
also fits the present dataset seeded from the best anchors found so far, and the
two candidates compete through the existing validity and ranking selection. The
change adds no data collection and leaves the one-click flow and automatic
stopping untouched. In a matched live A/B on the same unlayered HP3 world with
the identical 9-sweep/180-point layout, the applied anchor RMSE improved from
2.706 to 2.440 mm (reference 2.828 mm) at the same seven iterations and the same
patience stop; on the identical reference dataset replayed offline it improved
from 2.623 to 2.552 mm applied and 2.528 to 2.426 mm best candidate. Cold fits
are bit-identical between variants, so the change is purely additive, and it
costs roughly 2-8% of a live run's wall time in extra fitting. All evidence is
simulated/synthetic; no hardware measurements were taken, and host load made
wall-clock comparisons unusable.

#### Topics

autocalibration, HP3, CDPR, encoder-only, local optima, cost-cap sentinel,
multi-start, warm start, candidate ranking, patience stopping, simulation

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Selection score disagrees with geometric accuracy in the reference run | E01 | Supported | `offline/audit.py` on `offline/stages/history-007.json` |
| Fewer points per sweep preserve accuracy | E02 | Rejected | `offline/thin.py`, `offline/thinruns/*.log` |
| Unmodelled cable elasticity dominates the anchor error | E03 | Rejected | `offline/joint_fit.py`-adjacent `--flex` replay |
| Fixed-line encoder deltas drift inside sweeps | E03 | Rejected (no drift) | `offline/trilat.py` |
| Nuisance offsets or unknown home pose improve anchors | E03 | Rejected | `offline/joint_fit.py` |
| Broadening the final consensus beyond the closest pair helps | E04 | Inconclusive / not supported | `offline/audit.py` candidate ensembles |
| Refitting each iteration from the best-so-far anchors (competing warm start) improves robustness and accuracy | E04, E05, E05b, E05c | Supported in simulation | `offline/loop_emul.py`; `offline/base-replay.log` vs `offline/warm-replay.log`; live `native/autocal/c75c027e...` (divergence rescue) and the matched pair `native/autocal/d7a7507f...` vs `native/autocal/aa8435df...` |
| Cutting collected sweeps (early stop) preserves accuracy | E05 | Rejected | `offline/loop_emul.py` cumulative-sweep ablation |
| Force-selection sensitivity explains most cross-world accuracy spread | none | Not tested | no matched-world fixture available in the task regime |

#### References

- [Full report](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/report.md): implementation, measurements, limitations.
- [Research notebook](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/research.md): budgets, experiment decisions, steering.
- [Reference run to beat](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/../../../extended-autocal/no-buildup-reference-003/): supplied 9-sweep HP3 baseline, applied anchor RMSE 2.828 mm.
- [Controlled replay](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/offline/base-replay.log) and [warm replay](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/offline/warm-replay.log): identical-data offline A/B.
- [Matched live A/B](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/native/autocal/d7a7507f7b60486da3c0d626be819afd/sweeps.full_auto.log) (base@10) vs [H6@10](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/native/autocal/aa8435dfa54c4c28a8f040c43fa2ebce/sweeps.full_auto.log): same 9 sweeps/180 points, 2.706 vs 2.440 mm.
- [Prior session, settling and score-vs-geometry](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/abstract.md): earlier HP3 collection-timing and ranking findings reused here.
- [Recording-cost budget](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/../../../../research/autocal-recording-performance.md): recording modes are cheap relative to collection.
- [Truth anchors](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/../../../../autocal/tools/regress_calibration_logs.py): `SIM_3D_TRUE_ANCHORS` used for all RMSE numbers.

## [2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/)

[abstract.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/abstract.md) · [report.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/report.md) · [research.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/research.md)

### Faster HP3 encoder-only autocalibration: a consensus spread gate and a compliance-aware fixed-anchor tolerance

Status: complete for the measured hypotheses (E01-E14). Both source changes
shipped; a third candidate (a `trf` solver default) was measured and rejected.
All evidence is simulated.

#### Abstract

Question: can HP3 encoder-only autocalibration collect the same data faster, and
can it stop choosing worse anchors because its selection score disagrees with
geometry? Two changes were tested on simulated HP3 data. First, a spread gate
was added so that `consensus_anchor_estimate` refuses to average a candidate
pair separated by more than 5 mm and keeps the best-ranked fit instead. On 11
stored datasets this lowers the summed applied-anchor error from 788.58 mm
(best-ranked only) and 627.48 mm (unconditional consensus) to 615.87 mm; on the
recorded regression corpus it is 11.62 mm better than the previous behaviour, and
a stop-chain replay is mixed. Second, the fixed-anchor guard tolerance was raised
from 0.01 mm to 0.05 mm, because in the reference run the residual it chased was
bit-identical across all 20 points of every sweep despite a correction before
each point, so it was compliance rather than positioning error. A matched live
HP3 A/B measured 223.870 s vs 170.032 s of simulated time for identical 40-point
sweep sets (24.05% less, 1.35 s per point), a third collection reproduced the
patched arm bit-for-bit, and a second arm pair at 3 sweeps reproduced the saving
(323.984 s vs 245.432 s, 24.25%). Two further measurements bound the risk of the
tolerance change without settling it: the 9-sweep replay fit is itself
ill-conditioned, moving 0.14-1.32 mm when a single point's length is nudged by
0.001 mm, so the tolerance change's data difference is bounded to a few
millimetres rather than proven neutral; and switching the default solver to
trust-region-reflective removes the replay's rare 19-34 mm flip but costs 59-202
mm on three recorded-corpus datasets, so it was rejected as a default. Limitation:
all evidence is simulated; the gate's gain is corpus-dependent and mixed on the
stop chain, and the end-to-end anchor-quality equivalence of the tolerance change
is not settled at the reference's sweep width.

#### Topics

autocalibration, encoder-only calibration, Hangprinter HP3, CDPR, candidate
ranking, held-out sweep ordering, consensus averaging, fixed-anchor compliance,
collection time budget, slack-line pullout, deterministic simulation replay

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Averaging the closest candidate pair is unsafe when that pair is wide, so a spread gate improves the applied anchors | E01, E02, E03, E04 | Mixed (corpus-supported, stop-chain neutral-to-slightly-negative) | [E02/E03](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/artifacts/scripts/verify_gate.py), [E03](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/artifacts/scripts/reg-gate.txt), [E04](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/report.md#e04-stop-chain-replay) |
| The 0.01 mm fixed-anchor guard only chases a compliance residual, so raising it to 0.05 mm removes overhead without changing the machine state | E06 | Supported | [E06](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/report.md#e06-the-correction-moves-nothing) |
| Raising the tolerance cuts collection time for an identical sweep set | E07, E08 | Supported | [E07](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/artifacts/ab-matched/), [E08](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/report.md#e08-determinism-control) |
| Time saving reproduces at a second sweep count | E11 | Supported (time only) | [E11](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/artifacts/e11/e11_summary.json) |
| The same-sweep data still fits to the same anchors end-to-end | E11 | Not settled (sweep 2 unmatched, k=3 unidentifiable) | [E11](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/report.md#e11---end-to-end-ab-on-the-same-3-sweeps-time-only) |
| Fewer points per sweep gives equivalent accuracy | E09 | Rejected | [E09](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/artifacts/points_study.json) |
| Fewer sweeps give equivalent accuracy | E10 | Inconclusive | [E10](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/artifacts/sweep_count_study.json) |
| `sigma_floor` or more solver restarts is a usable accuracy lever | E12 | Rejected | [E12](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/artifacts/sigma_floor_study.json) |
| The shipped fit is robust to the per-point length differences the tolerance change introduces | E13b, E14 | Rejected: the 9-sweep fit is knife-edge (0.001 mm -> up to 1.32 mm) | [E13b](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/artifacts/sensitivity-perpoint/), [E14](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/artifacts/solver-study/) |
| `trf` is a better default solver than `lbfgsb` | E14 | Rejected as default; better on replay, 59-202 mm worse on 3 corpus datasets | [E14](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/report.md#e14---solver-choice-better-on-the-replay-rejected-on-the-corpus) |
| Shortening the slack-line pullout waits is the next available speedup | E11-proposed | Not tested | [E05](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/report.md#e05-where-the-time-goes) |

#### References

- [Full report](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/report.md): analysis, measurements and limitations.
- [Research notebook](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/research.md): experiment decisions and accepted steering.
- [Original task](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/prompt.txt).
- Reference run to beat: `output/extended-autocal/no-buildup-reference-003`
  (local evidence; the comparison target named in the task).
- [Extended autocal reference data](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/autocal/data/references/extended-reference.md):
  how to make full recorded collection runs.
- [Recording performance budgets](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/research/autocal-recording-performance.md):
  matched HP3 replay cost of `none`/`events`/`sampled`/`full` recording.
- [Research catalog](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/research/catalog.md): index of prior sessions.
- [Research index tooling](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/scripts/research_index.py).

## [2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/)

[abstract.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/abstract.md) · [report.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/report.md) · [research.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/research.md)

### Warm-starting the HP3 encoder autocalibration anchor solve (and a negative result on point thinning)

Status: complete for the simulated evidence in this session.

#### Abstract

Encoder-only HP3 autocalibration is slow, and its candidate score disagrees with true geometry
(selection regret 5.47 mm on the supplied reference). This session asked whether the same anchors are
reachable from fewer or cheaper data points without adding a user decision. The anchor solve restarted
from a crude synthetic layout every iteration, so removing points destabilised it; carrying the
previous accepted anchors into the next solve (an 11-line change in `autocal.py`) removes much of that
dependence. Two supervised live one-click jobs that collected *bit-identical* datasets (9 sweeps, 180
points, 2,590,397 physics steps) differ only by that change: warm seeding accepted 2.420 mm anchor RMSE
where the unmodified loop accepted 2.706 mm, reproducibly. At half point density two matched live jobs
accepted identical anchors, but the unmodified loop spent two extra iterations on failed cold fits
(9 sweeps / 90 points / 2.35e6 steps against 7 / 70 / 2.06e6). Offline re-fitting then separated density
from measurement quality: half the points of a standard collection are redundant (90 points reach
2.53 mm against 2.44 mm for 180), yet *collecting* at half density yields worse data at the same point
count (3.7-3.8 mm), so lowering point density is not a safe way to save collection time. Main
limitations: all evidence is simulated, radii/buildup fitting was off, the live A/B pairs are single
observations, and wall-clock throughput varied up to 69% between nominally identical jobs, so effort is
reported in points and physics steps.

#### Topics

autocalibration, HP3, CDPR, encoder-only, warm start, initial guess, point-count reduction, sweep
density, sampling grid, automatic stopping, patience, candidate score versus geometry, settling,
simulated versus hardware evidence, no line buildup

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| A low `rank_score` identifies geometrically accurate anchors | E01 | Rejected | [Stage audit](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e01_reference_stage_audit.json) |
| Blindly halving points per sweep keeps geometry | E02, E05 | Rejected cold, supported warm | [Thinning](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e02_point_subsets.json), [warm chains](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e05_warmstart.json) |
| Warm-starting iterations stabilises and improves the anchor solve | E05, E10, E11, E13, E15, E16, E17, E18 | Supported (simulated) | [Live stats](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e13_live_runs.json), [E16](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e16_live_thinning.json) |
| Cold-start failure is a multi-start budget problem; more restarts fix it | E07, E08 | Rejected | [Multi-start](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e07_multistart.json), [restarts](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e08_restarts.json) |
| The cold seed only has the wrong scale | E09, E12 | Mixed | [Length seed](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e09_length_seed.json), [scale ladder](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e12_scale_ladder.json) |
| Warm start also reduces the number of iterations at full density | E10, E14, E17, E18 | Rejected (same 7 iterations, 9 sweeps) | [Live stats](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e13_live_runs.json) |
| Halving the point density is accuracy-neutral in the live one-click job | E13, E14, E15, E16 | Rejected: coarse collection gives worse data | [Live stats](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e13_live_runs.json), [E16](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e16_live_thinning.json) |
| Point-count-aware patience recovers the accuracy lost by halving points | E15, E16 | Rejected: extra sweeps at half density do not improve accuracy | [E16](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e16_live_thinning.json) |
| A 3-sweep 10-point live dataset is enough for HP3 | E06 | Rejected (near-degenerate data) | [E06](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e06_live_halfpoints.json) |
| Collection wall time is set by requested settling, not physics work | E03 | Supported | [Sweep manifest](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/native/00403d4402004c09aaac4ccedd97b7c4/86feab22c3934b469686a3c39ae57b5f/manifest.json) |
| Wall-clock time is a usable comparison metric between jobs | E14, E18 | Rejected (identical config, 69% apart) | [Live stats](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e13_live_runs.json) |
| A coarser drive grid is worse because of residual settling / hysteresis per step | none | Not tested | Proposed in [report.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/report.md#next-steps-proposed-not-tested) |
| Shorter proven-stable settle timeouts preserve geometry | none | Not tested | Proposed in [report.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/report.md#next-steps-proposed-not-tested) |

#### References

- [Full report](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/report.md): methods, matched live and offline results, implementation and limitations.
- [Research notebook](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/research.md): objective, budgets, accepted steering and per-experiment log.
- [Reference dataset](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/data/reference_sweeps.json) and [stage artifacts](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/data/reference_stages/):
  supplied HP3 no-buildup run (9 sweeps x 20 points, 573 s wall) used as the baseline.
- [Live job stats](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e13_live_runs.json) (E13, E14, E15, E17, E18),
  [live iterations](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e13_live_iterations.json) and
  [E16 live-data thinning](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/experiments/e16_live_thinning.json).
- Preceding sessions: [settling reuse and thinning audit](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/),
  [ranking and bounded-update negative results](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/../autocal-ranking-bounded-updates-negative-results-b7deb1bc/),
  [candidate validation and throughput](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/../autocal-candidate-model-validation-and-throughput-fdb616a4/).

## [2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/)

[abstract.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/abstract.md) · [report.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/report.md) · [research.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/research.md)

### Trust-region anchor solving makes HP3 encoder autocalibration more accurate, cheaper to compute and less selection-sensitive

Status: complete in the research worktree; its default change was not imported
into main after review against the newer main commits.

#### Abstract

Encoder-based CDPR autocalibration in hp-sim5 searches for anchor geometry with a
multi-start local solver, and earlier sessions showed its selection score often
disagrees with true geometric accuracy. This session profiled a full headless HP3
run (5180.8 simulated seconds; 21% per-sweep pull-outs, 24% force tuning, 24%
per-point settling, 23% travel), re-tested force tuning, pull-out bounds, point
counts and solver settings, and measured what the algorithm can actually achieve.
Fit-only replay showed the dominant error term is not selection but the common error
of the fitted pool: the applied consensus (2.706 mm) is only 0.32 mm worse than the
best candidate in the pool (2.385 mm). The largest reproducible gain is the anchor
solver: switching the default from `lbfgsb` to bounded `trf` cut mean applied RMSE
over 10 recorded datasets from 4.348 to 3.590 mm (-17%, worst case 9.963 -> 9.206 mm),
took a full headless run from 2.706 to 2.410 mm with byte-identical collected data,
identical 5180.8 simulated seconds and ~4x less solver CPU, and reproduced on an
independent clean 22-pool benchmark of staged fits (target group 4.158 -> 3.485 mm,
selection regret 1.295 -> 0.267 mm). The worktree default was verified end-to-end.
The gain does not transfer to the repository's 11 legacy fixtures (Slideprinter/HP4
recordings with line buildup and fitted spool radii, outside the scoped regime):
there `trf` raises mean applied error from 16.2 to 27.9 mm, because its more
self-consistent early fits change which candidates the tightest-pair consensus picks
(the mechanism was isolated and no alternative selection rule tested, including a
pool of both solvers, repairs it). The proposed default change was verified in the
research worktree but was not imported into main. Fewer-data-point ideas were measured and rejected:
even-point decimation per sweep blows up unpredictably (individual datasets 19-456 mm
at N = 10-14), while planned 6-point sweeps are only a mild accuracy/time trade
(2.514 mm at 4868 s, or 2.031 mm at 4773 s with a lower 0.91 N sensor force).
Limitations: all evidence is simulated. A new paired matrix across 10 HP3 worlds,
varying mover mass (0.80-1.25x) and gravity (8.5-11.0 m/s²), modestly favors `trf`:
mean applied RMSE 2.874 vs 2.967 mm and per-world iteration-mean delta -0.064 mm.
Neither solver had an iteration above 20 mm in that matrix, so it does not test
extreme-failure avoidance. A current-main replay of the three worst legacy fixtures
still favors `lbfgsb` (mean 32.2 vs 76.8 mm), so the default change was not imported.

E36 reviewed the user's subsequent per-restart fallback regression. On the seventh
dataset, fallback removed an extreme first fit but created a tight biased early
pair: summed applied anchor error rose 197.410 -> 257.120 mm while paired
iteration-mean error improved by 1501.281 mm. Seventeen of nineteen retries were
iteration-limit exits. Per-restart fallback is now opt-in; solver status and better
intermediate fits do not guarantee a better applied consensus.

#### Topics

autocalibration, Hangprinter 3, CDPR, anchor solver, trust-region least squares,
L-BFGS-B, local optima, multi-start restarts, consensus anchors, candidate
selection, simulator harness validity, fewer data points, sensor collection force,
simulated seconds, HP3

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Raising sweep feed rate is free accuracy-wise | E02 | rejected | B2/B3 cost table |
| Lowering the sensor collection force monotonically improves anchors | E21 | rejected | force sweep 0.68/0.91/1.36/2.2 N |
| Fewer points per sub-sweep is an equal-accuracy speedup | E02/E25 | rejected (`lbfgsb`), mixed (`trf`) | T3 vs T1 |
| The pull-out settle can be bounded below its physical endpoint | E20 | rejected | 435 mm endpoint loss at a 5 s cut |
| The selection score disagrees with true geometric accuracy | E01-lab, E23 | supported, but only 0.32 mm here | oracle 2.385 vs applied 2.706 |
| Solver choice, not data, explains most of the pool error | E23-E26, E32, E35 | supported for tested no-buildup HP3 | 10-dataset battery, `V1-verify-default`, clean 22-pool benchmark, 10 varied simulated worlds |
| Consensus spread is a usable early-stop signal | E27 | supported as a signal, insufficient as a rule | saves a sweep on 2 of 11 datasets |
| `trf` also improves the legacy buildup/radii fixtures | E28, E29 | **rejected** | legacy mean 16.2 -> 27.9 mm applied RMSE |
| Per-restart TRF fallback safely improves applied results | E36 | **rejected as a default policy** | seventh-dataset applied anchor error +59.710 mm despite paired iteration mean -1501.281 mm; preserved on/off logs |
| `trf` modestly improves no-buildup HP3 across varied simulated mass/gravity | E35 | supported, small effect | 10 paired worlds; applied mean 2.967 -> 2.874 mm; no >20 mm iteration failures for either solver |
| A different pool-selection rule repairs the weak solver cases | E30, E33 | **rejected** | current rule best in 3 of 4 groups; solver-union no fix |
| Fewer points per sweep by decimation stays accurate | E34 | **rejected** | N = 10-14 blow-ups of 19-456 mm on individual datasets |
| Divide-and-conquer sub-experiments can isolate parameters | none | not tested | proposed |
| Many plausible worlds are needed to judge robustness | none | not tested | proposed (cost) |

#### References

- [Full report](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/report.md): measurements, rejected hypotheses, the legacy-fixture
  caveat and limitations.
- [Research notebook](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/research.md): experiment decisions and accepted steering.
- [Reference run](/home/torbjorn/repos/hp-sim5/output/extended-autocal/no-buildup-reference-003/)
  (2.828 mm applied RMSE), the run this work had to beat.
- Clean pool benchmark: `runs_refset/pool_evidence_clean.txt`,
  `runs_refset/pool_evidence.json`, `runs_refset/union_rule_clean.txt`,
  `runs_refset/refset_solver_compare.txt`, `runs_refset/thin_study_boundary.json`.
- Varied-world experiment: `environment-matrix-20261015-retry1/matrix_summary.json`
  and the compact per-world artifacts; collection and metric scripts are under
  `tools/`.
- Earlier sessions: [quiet-window reuse](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/abstract.md),
  [ranking vs physical accuracy](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/../autocal-held-out-history-ranking-7ec4260f/abstract.md).
- Code revision: `1404c5ac` (repo HEAD) plus the one-line solver default change.

## [2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/)

[abstract.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/abstract.md) · [report.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/report.md) · [research.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/research.md)

### Consensus-spread early stopping: fewer HP3 autocalibration sweeps without losing geometry

Status: complete. Simulation evidence; one live HP3 confirmation run reported below.

#### Abstract

hp-sim5's full-auto encoder autocalibration keeps collecting sweeps after the
applied geometry has converged, because it stops on patience or an absolute
cost/std threshold whose ranking scalar disagrees with geometrical accuracy. We
tested an automatic early stop that fires when the shipped bracketed-consensus
estimate has stabilised geometrically: the two mutually most consistent *distinct*
candidate fits must agree to within `1.5e-3` of the machine's own anchor extent for
two consecutive iterations, from at least four candidates. Parameters were
selected on rule-independent replay decks from 13 recorded runs. On the recorded
reference-003 deck the stop saves 2 of 9 sweeps for +0.18 mm applied-anchor RMSE;
on the shipped 11-dataset regression harness it saves 6 sweeps with aggregate RMSE
improving 0.64 mm; across 8 converged offline decks it saves 23 sweeps with mean
regret 0.051 mm (worst 0.221 mm). A live HP3 job stopped at iteration 6 instead of
7 (about 11% less simulated collection time) for +0.21 mm. The same job exposed,
and this session fixed, a `normal_completion` regression that had reported the
successful early stop as a failed job; the fixed re-run was accepted and reproduced
the physics and geometry exactly. Limit: a stopping rule cannot improve a converged
estimate, and two wrong fits can still agree on uninformative data.

#### Topics

autocalibration, encoder-only, HP3 hangprinter, full-auto early stopping,
consensus spread, stopping rule, candidate ranking, sweep budget,
regression matrix, simulated calibration

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| A distinct-fit spread stop is safe and saves sweeps | E05, E06, E08, E11, E12, E13, E09b | Supported (shipped) | [E13](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/evidence/E13-converged-regret.json), [E11](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/evidence/E11-regression-harness-ab.json), [E12](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/evidence/E12-reference-003-ab.json) |
| `n_min = 4` and `k = 2` are needed for safety | E05, E06 | Supported | [E06](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/evidence/E06-stop-regret-grid.json) |
| `n_min = 3` also works | E06 | Rejected | [E05](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/evidence/E05-spread-deck.json) |
| `frac = 3e-3` is safe | E06 | Rejected | [E06](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/evidence/E06-stop-regret-grid.json) |
| Consensus-delta stability is a better stop | E07 | Rejected | [E07](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/evidence/E07-rule-family-compare.json) |
| The new stop reason is compatible with the headless job lifecycle | E09b, E09c | Rejected as implemented, fixed, then supported | [E09 pair](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/evidence/E09-live-hp3-pair.json) |
| Better sweep *placement* rather than stopping | none | Not tested | - |

#### References

- [Full report](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/report.md): method, measurements, tests and limitations.
- [Research notebook](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/research.md): experiment log, budgets and accepted steering.
- [Original task](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/prompt.txt).
- Reference run to beat: `output/extended-autocal/no-buildup-reference-003`
  (main checkout); see [extended-reference.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/../../../../autocal/data/references/extended-reference.md).
- Recording-workload costs used to choose `events` recording:
  [autocal-recording-performance.md](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/../../../../research/autocal-recording-performance.md).
- Prior session `2026-10-10-...-51aa157a` (default-force HP3 acceptance at
  iteration 7, 360 points, 2.604 mm; score-vs-geometry audit) and its tracked
  [review](sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-b75b3bdf/../../../../research/autocal-settling-review.md).
- Prior session `2026-10-13-...-b65aa69c` (stability-stop and single-pick rejections).
- Prior session `2026-10-12-...-2f03cb59` (centered encoder averaging).
- Session `autocal-ranking-bounded-updates-negative-results-b7deb1bc` (ranking negatives).

## [2026-10-15-native-optimization](sessions/2026-10-15-native-optimization/)

[abstract.md](sessions/2026-10-15-native-optimization/abstract.md) · [report.md](sessions/2026-10-15-native-optimization/report.md) · [research.md](sessions/2026-10-15-native-optimization/research.md)

### Native PGO and columnar Rerun performance

Status: complete.

#### Abstract

This study compares native headless simulation with the fuller Rust/Wasm runtime, using the same Rust physics code, commands, fixed timesteps and output cadence. Raw column buffering replaces per-observation Rerun object construction in both recorders, and optimized native builds now use deterministic profile-guided compilation with explicit opt-out and artifact selection. Three sequential repetitions per case measure HP3/HP4, recording on/off, fresh and continuing worlds, the standalone autocal service and real six-point RRF collections, including finalization. Recorded runs improve 1.73–3.83× over Wasm with the earlier logging; most of that gain is logging and also benefits Wasm. Native physics improves 1.25–1.31× on eight 30,000-step prefixes and 1.39×/1.27× on full HP3/HP4 autocal replays. Native checkpoint hashes and audited RRD values remain unchanged, and all 42 real collections retain exact selected measurements and step counts. These measurements exclude full optimizer/fitting runs and do not establish calibration accuracy.

#### Topics

Native Rust, Wasm, PGO, Rerun, send_columns, logging overhead, headless simulation, HP3, HP4, firmware collection.

#### Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Raw column batching reduces recorder overhead | O01 | Supported | [Paired consumers](sessions/2026-10-15-native-optimization/summary.json), [RRD audit](sessions/2026-10-15-native-optimization/final-recording-audit.json) |
| Redundant-work source micro-optimizations improve physics | O02 initial | Rejected on tested HP4 workload; changes reverted | [Notebook](sessions/2026-10-15-native-optimization/research.md) |
| PGO and host tuning improve native physics | O02 follow-up | Supported | [Prefixes](sessions/2026-10-15-native-optimization/final-physics-matrix.json), [long replays](sessions/2026-10-15-native-optimization/long-matrix.json) |
| Reusing captured snapshots reduces observation work | O03 | Supported as part of combined logging change; not isolated | [Report](sessions/2026-10-15-native-optimization/report.md) |

#### References

- [Report](sessions/2026-10-15-native-optimization/report.md): method, timing scope, results and limitations.
- [Notebook](sessions/2026-10-15-native-optimization/research.md): decisions, failed trials and budgets.
- [Source manifest](sessions/2026-10-15-native-optimization/source-manifest.json): exact source/artifact identity.
- [Previous study](sessions/2026-10-15-native-optimization/../2026-10-14-native-throughput/report.md): earlier Wasm/native measurements.

## [2026-10-16-hp4-effector-instability](sessions/2026-10-16-hp4-effector-instability/)

[abstract.md](sessions/2026-10-16-hp4-effector-instability/abstract.md) · [report.md](sessions/2026-10-16-hp4-effector-instability/report.md) · [research.md](sessions/2026-10-16-hp4-effector-instability/research.md)

### HP4 effector rocking: separate the firmware tick from physics integration

Status: complete for the reproduced vibration; original full-autocal retry remains untested.

#### Abstract

The production pipeline used the 2 ms firmware tick as one mechanical integration step. Loaded HP4 states with stiff cables and light rotating members can sustain a numerical rocking cycle at that step size. An offline replay of the supplied recording produced a stressed checkpoint whose corner moved approximately 16.05 mm peak to peak even after all motors returned to position mode. Four internal 0.5 ms physics steps settled that same checkpoint in JavaScript, native Rust and browser Wasm, with angular velocity reaching zero. Command dispatch, extrusion and recording retain their 2 ms cadence. A separate cable-damping sign error allowed compressive corrections; clamping it fixed a simpler trial but did not fix the full stress case alone. A small-angle quaternion cancellation error in rotational cable damping was also corrected across backends. No autocal settling/acceptance thresholds changed. The original recording also shows substantial slow travel during size tuning, so successful full-autocal collection is not established. This is simulated stability evidence, not calibration-accuracy or hardware validation.

#### Topics

HP4, effector corner vibration, rigid body, numerical stability, physics substeps, firmware cadence, unilateral cable damping, Rust, Wasm, autocalibration.

#### Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| One 2 ms integration step sustains loaded-state rocking | E05/E06 | Supported | [Report](sessions/2026-10-16-hp4-effector-instability/report.md) |
| Four 0.5 ms internal steps settle the same state | E07 | Supported | [Measurements](sessions/2026-10-16-hp4-effector-instability/stress-summary.json) |
| Cable damping sign correction alone is sufficient | E05 | Rejected | [Report](sessions/2026-10-16-hp4-effector-instability/report.md) |
| Original full-autocal succeeds after correction | None | Not tested | [Limits](sessions/2026-10-16-hp4-effector-instability/report.md) |

#### References

- [Report](sessions/2026-10-16-hp4-effector-instability/report.md): causes, implementation, experiments and limitations.
- [Notebook](sessions/2026-10-16-hp4-effector-instability/research.md): accepted steering and budgets.
- [Numerical results](sessions/2026-10-16-hp4-effector-instability/stress-summary.json) and [plot](sessions/2026-10-16-hp4-effector-instability/stress-stability.png): final before/after comparison.


Follow-up: default-four integration was rejected for performance and restored to one. The original browser command still fails at one step. See [single-step follow-up](sessions/2026-10-16-hp4-effector-instability/../2026-10-16-hp4-one-step-autocal/report.md).

## [2026-10-16-hp4-load-bookkeeping](sessions/2026-10-16-hp4-load-bookkeeping/)

[abstract.md](sessions/2026-10-16-hp4-load-bookkeeping/abstract.md) · [report.md](sessions/2026-10-16-hp4-load-bookkeeping/report.md) · [research.md](sessions/2026-10-16-hp4-load-bookkeeping/research.md)

### HP4 motor load bookkeeping: one contribution per physical constraint

Recording motor cable load torque, stiffness and damping once per constraint, rather than per solver iteration, improves a matched one-step 30 s pull from56.8to84.8mm without extra integration work. The correction is mirrored in JavaScript, Rust native/Wasm, Python and Warp. A fresh headless one-substep autocal run completed three bootstrap sweeps (60 points) plus a fourth sweep, then stopped at its configured one-iteration limit. It did not time out. Force tuning fell back to1N and the fit remained underconstrained, so this is collection validation, not successful calibration. Default substeps, forces and settling criteria are unchanged.

Topics: HP4, motor loads, solver iterations, implicit stiffness, autocal.

See [report](sessions/2026-10-16-hp4-load-bookkeeping/report.md) for evidence and limitations.

## [2026-10-16-hp4-one-step-autocal](sessions/2026-10-16-hp4-one-step-autocal/)

[abstract.md](sessions/2026-10-16-hp4-one-step-autocal/abstract.md) · [report.md](sessions/2026-10-16-hp4-one-step-autocal/report.md) · [research.md](sessions/2026-10-16-hp4-one-step-autocal/research.md)

### Restore HP4 single-step performance; autocal remains unresolved

Default physics integration is restored to one 2 ms step in JavaScript and Python. Four steps remain explicit opt-in and are not an accepted production fix. The exact browser autocal command was retried with only its dataset directory relocated: force tuning completed, then size tuning failed its unchanged 120000 ms encoder-stability deadline. During the last seconds of the pull, the effector still travelled approximately 2 mm/s horizontally with small rotation. This failure is not established to be the earlier corner-rocking mode. Focused tests pass. The original-command completion requirement remains unmet; no throughput recovery is claimed.

Topics: HP4, performance, autocal, optional substeps, simulation stability.

See [report](sessions/2026-10-16-hp4-one-step-autocal/report.md) for tested hypotheses, evidence and limitations.

## [2026-10-16-hp4-roller-propagation](sessions/2026-10-16-hp4-roller-propagation/)

[abstract.md](sessions/2026-10-16-hp4-roller-propagation/abstract.md) · [report.md](sessions/2026-10-16-hp4-roller-propagation/report.md) · [research.md](sessions/2026-10-16-hp4-roller-propagation/research.md)

### HP4 sluggish torque response: implicit motor stiffness dominates a roller redistribution counterfactual

A controlled 30 s HP4 pull at fixed torques tested slow local rest-length propagation against motor-load numerical damping. Replacing local frictionless redistribution with an exact whole-path solve changed forward travel from 56.84 to 57.43 mm; two substeps reached 105.25 mm. Suppressing only the implicit motor stiffness term, retaining the implicit damping branch, reached 588.48 mm. This supports implicit motor/cable stiffness as the dominant bottleneck for this pull, rather than redistribution latency. The stabilizing term was removed only in an offline diagnostic, not deployed. A separate check found reported motor stiffness doubling from 36 to 72 Nm/rad when solver iterations increase from one to two. This is not an exact reproduction of the user's second half-sweep or a proof that disabling stabilization is safe.

Topics: HP4, rollers, cable transport, implicit motor coupling, numerical damping, substeps.

See [report](sessions/2026-10-16-hp4-roller-propagation/report.md) for hypotheses, history, and limitations.

## [2026-10-16-hp4-substep-options](sessions/2026-10-16-hp4-substep-options/)

[abstract.md](sessions/2026-10-16-hp4-substep-options/abstract.md) · [report.md](sessions/2026-10-16-hp4-substep-options/report.md) · [research.md](sessions/2026-10-16-hp4-substep-options/research.md)

### HP4 substep options and bounded size-tuning travel

Substeps are now selectable in the browser and through autocal's --physics-substeps setting for browser/headless collection, defaulting to one. A fixed-torque no-layering HP4 experiment shows the unchanged settling check passing after about 472 simulated seconds at one step versus 120 at four steps. Running the production size-tuning routine against native physics with its real ramp and 500 ms polling passed at one step after 477.75 simulated seconds. The size-tuning deadline is increased from 120 to 600 seconds, with a finite independent wall deadline and unchanged settling criteria. This validates the size-tuning routine, not an entire full-autocal run or hardware accuracy. Timestep-dependent numerical damping remains an accuracy concern.

Topics: HP4, autocal, settling, optional substeps, headless settings, numerical damping.

See [report](sessions/2026-10-16-hp4-substep-options/report.md).


#### Follow-up: slow half-sweep pullout in reference-006

User requested only a timeout increase to retry. The actual reference-006/sweeps.full_auto_1.log shows successful size tuning (590.418 mm raw), then first half-sweep pullout hitting its 120 s limit, holding the drive, and timing out after the subsequent 30 s preload-settling limit. Increased both pullout travel and its immediate preload-settling budget to 600 simulated seconds, each with a 630 s wall deadline. Force selection, substep defaults and stability criteria are unchanged. No new physical trial or full-autocal retry was run. The two targeted settling/size-tune suites pass: 13 tests. This supersedes the earlier report statement that other force-travel limits were unchanged.

## [2026-10-16-interactive-research-1c2eae68](sessions/2026-10-16-interactive-research-1c2eae68/)

[abstract.md](sessions/2026-10-16-interactive-research-1c2eae68/abstract.md) · [research.md](sessions/2026-10-16-interactive-research-1c2eae68/research.md)

### interactive research

Status: not yet summarized.

#### Abstract

The session has started. No research result has been summarized yet.

#### Topics

interactive research

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| None recorded yet | None | Not tested | None |

#### References

- [Original task](sessions/2026-10-16-interactive-research-1c2eae68/prompt.txt)
- [Research notebook](sessions/2026-10-16-interactive-research-1c2eae68/research.md)

## [2026-10-16-interactive-research-2ecf1b18](sessions/2026-10-16-interactive-research-2ecf1b18/)

[abstract.md](sessions/2026-10-16-interactive-research-2ecf1b18/abstract.md) · [research.md](sessions/2026-10-16-interactive-research-2ecf1b18/research.md)

### interactive research

Status: not yet summarized.

#### Abstract

The session has started. No research result has been summarized yet.

#### Topics

interactive research

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| None recorded yet | None | Not tested | None |

#### References

- [Original task](sessions/2026-10-16-interactive-research-2ecf1b18/prompt.txt)
- [Research notebook](sessions/2026-10-16-interactive-research-2ecf1b18/research.md)

## [2026-10-19-interactive-research-050ce13b](sessions/2026-10-19-interactive-research-050ce13b/)

[abstract.md](sessions/2026-10-19-interactive-research-050ce13b/abstract.md) · [research.md](sessions/2026-10-19-interactive-research-050ce13b/research.md)

### interactive research

Status: not yet summarized.

#### Abstract

The session has started. No research result has been summarized yet.

#### Topics

interactive research

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| None recorded yet | None | Not tested | None |

#### References

- [Original task](sessions/2026-10-19-interactive-research-050ce13b/prompt.txt)
- [Research notebook](sessions/2026-10-19-interactive-research-050ce13b/research.md)

## [2026-10-19-interactive-research-41c8bab5](sessions/2026-10-19-interactive-research-41c8bab5/)

[abstract.md](sessions/2026-10-19-interactive-research-41c8bab5/abstract.md) · [report.md](sessions/2026-10-19-interactive-research-41c8bab5/report.md) · [research.md](sessions/2026-10-19-interactive-research-41c8bab5/research.md)

### HP4 one-substep autocal wobble: XPBD damping feedback and acceptance status

Status: in progress (fix verified in replay; full autocal acceptance incomplete).

#### Abstract

Loaded HP4 autocal wobbled with the default one 2 ms physics step, disturbing
settling and force selection. We froze a production checkpoint, replayed it in
the native Rust solver, and recorded the effector in Rerun. With two XPBD
Gauss-Seidel passes the effector entered a deterministic bounded limit cycle
(6.27 mm late span, 1.79 rad/s). The cause was order-dependent damping:
each joint's velocity-dependent displacement was recomputed from poses already
changed by earlier visits, so damping differentiated its own correction and
injected energy; velocity reconstruction then fed the biased pose into the next
step. Caching each joint's first-visit displacement per substep removed the
cycle at the default one 2 ms step and at explicit two substeps (late span
about 1e-10 to 1e-9 mm, zero angular speed). Exact requested substeps replaced
implicit 1 ms promotion. The full headless default-one-substep autocal still
has not completed: the clean run was externally interrupted at 210,000 steps
after entering sweep collection, and earlier runs hit a now-fixed deadline bug.
This is simulated evidence, not hardware validation.

#### Topics

autocalibration, Hangprinter HP4, XPBD, PBD, Gauss-Seidel, cable damping,
effector wobble, substeps, force tuning, headless simulation, Rerun

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| One 2 ms XPBD step with two passes produces a bounded effector limit cycle under load | E01, E02 | Supported | `wobble-baseline.rrd`; `baseline-i1-i2/` |
| Recomputing damping displacement from partially solved poses feeds energy into the cycle | E04, E05 | Supported | `trace-i2.stderr`; `final-cache/` |
| A per-substep first-visit damping cache removes the default one-step wobble | E05 | Supported | `wobble-damping-cache.rrd`; `recheck/` |
| A global beginning-of-substep snapshot is a better damping source | E06 | Rejected | `snapshot/` |
| Implicit promotion to two 1 ms steps is required for stability | E05 | Rejected | `1 x 2 ms`, i2 settles with the cache |
| The full default one-substep autocal completes end-to-end | E08 | Not tested (incomplete) | Last clean run interrupted at 210,000 steps |
| Default-substep force-max selection completes without wobble or timeout | E08 | Not tested (incomplete) | Clean run reused force tuning; earlier failures predate deadline fix |
| The damping-cache idea is novel to this branch | none | Rejected (known upstream) | Upstream `bf9f5399` |

#### References

- [Full report](sessions/2026-10-19-interactive-research-41c8bab5/report.md): analysis, measurements and limitations.
- [Research notebook](sessions/2026-10-19-interactive-research-41c8bab5/research.md): experiment decisions and accepted steering.
- [Baseline RRD](sessions/2026-10-19-interactive-research-41c8bab5/wobble-baseline.rrd), SHA-256 `ff27609f93d4c1dbfc23d7af1fc0a15c88a3feffbd82c8cf70f64da90462736c`.
- [Fixed RRD](sessions/2026-10-19-interactive-research-41c8bab5/wobble-damping-cache.rrd), SHA-256 `229663f079530af0fd020a95fbeb3c5f08b2f9548af10d0ccb6a165f7314af9a`.
- [Freeze fixture](sessions/2026-10-19-interactive-research-41c8bab5/../../../../tests/fixtures/hp4_loaded_stability.json), SHA-256 `3d0ce9e30b2eaf35ff315b1102f9a911965b4b96a1625e998399503c7e725341`.
- [Prior HP4 effector-instability session](sessions/2026-10-19-interactive-research-41c8bab5/../2026-10-16-hp4-effector-instability/report.md).
- [Prior HP4 substep-options session](sessions/2026-10-19-interactive-research-41c8bab5/../2026-10-16-hp4-substep-options/report.md).
- [Prior one-step-autocal session](sessions/2026-10-19-interactive-research-41c8bab5/../2026-10-16-hp4-one-step-autocal/report.md).
- [Prior load-bookkeeping session](sessions/2026-10-19-interactive-research-41c8bab5/../2026-10-16-hp4-load-bookkeeping/report.md).
- [Prior roller-propagation session](sessions/2026-10-19-interactive-research-41c8bab5/../2026-10-16-hp4-roller-propagation/report.md).

## [2026-10-19-interactive-research-8d7a82a2](sessions/2026-10-19-interactive-research-8d7a82a2/)

[abstract.md](sessions/2026-10-19-interactive-research-8d7a82a2/abstract.md) · [report.md](sessions/2026-10-19-interactive-research-8d7a82a2/report.md) · [research.md](sessions/2026-10-19-interactive-research-8d7a82a2/research.md)

### HP4 autocal failures: numerical slowing, missing travel budget and a repeatable four-substep workflow

Status: complete investigation; the underlying dynamics correction remains open.

#### Abstract

We investigated recurring HP4 autocal failures using October 16–19 logs and Rerun recordings, native Rust force comparisons, an exact replay and bounded headless calibration trials. Both movers weigh 81 g. At equal configured effective force, one-substep HP3 reaches Y=423 mm at 30 seconds versus HP4 80 mm; four substeps raises HP4 to 335 mm. Motor stabilization introduces timestep-dependent slowing amplified by reeving. The reproduced failure at 515.42 seconds instead expires in a separate 30-second preparation wait while free motors still pull. We extended that transit budget and corrected headless speedup to preserve physical wait durations. A faster motor formula was rejected because calibration fits deteriorated. With original physics and four substeps, 60 bootstrap measurements exactly repeat prior evidence. A fresh-world continuation from 100 measured points collects 40 more and stops automatically, with applied-anchor RMSE 4.02 mm versus nominal geometry. This establishes a controlled continuation workflow, not universal reliability or a completed fresh single-invocation run. All evidence is simulated; hardware accuracy and a complete dynamics repair remain untested.

#### Topics

HP4, HP3, Hangprinter, autocalibration, torque mode, numerical damping, reeving, implicit integration, physics substeps, headless speedup, settling, dataset continuation, Rerun

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| HP4 moves substantially less at equal configured effective force | E02, E06 | Supported for controlled pulls | [Motion comparison](sessions/2026-10-19-interactive-research-8d7a82a2/motion-comparison.png), [matched forces](sessions/2026-10-19-interactive-research-8d7a82a2/E06-matched-forces/) |
| The remaining default timeout is missed fixed targets or rapid wobble | E04, E07 | Rejected for this failure: held motors stationary, free motors pulling | [Exact replay](sessions/2026-10-19-interactive-research-8d7a82a2/E07-coast/replay.json), [failure screenshot](sessions/2026-10-19-interactive-research-8d7a82a2/failure-viewer.png) |
| Longer size/pullout budgets cover the failing preparation wait | E04, E07 | Rejected: a separate ordinary 30 s wait remains | [Failure log](sessions/2026-10-19-interactive-research-8d7a82a2/E04-sub1-clock/sweeps.full_auto.log), [coast check](sessions/2026-10-19-interactive-research-8d7a82a2/coast-settling.json) |
| Headless speedup preserved physical collection durations before the fix | E01, E04, E17 | Rejected before the fix; corrected durations supported at 1/10/25 | [Original source](sessions/2026-10-19-interactive-research-8d7a82a2/autocal_headless-before.mjs), [clock invariance](sessions/2026-10-19-interactive-research-8d7a82a2/E17-clock-invariance.json) |
| Correcting the local steady motor slowing term is sufficient for production | E08–E12, E15 | Rejected: better travel but poor data and degenerate fits | [Rejected patch](sessions/2026-10-19-interactive-research-8d7a82a2/rejected-motor-correction.patch), [residual audit](sessions/2026-10-19-interactive-research-8d7a82a2/nominal-model-residuals.json), [seeded diagnostic](sessions/2026-10-19-interactive-research-8d7a82a2/E15-seeded-fit-diagnostic.json) |
| Extending preparation completes stock one-substep calibration | E16 | Inconclusive: 50 bootstrap points before wall budget; former failure crossed | [Trial log](sessions/2026-10-19-interactive-research-8d7a82a2/E16-preparation-budget/sweeps.full_auto.log), [manifest](sessions/2026-10-19-interactive-research-8d7a82a2/E16-preparation-budget/sweeps.headless/manifest.json) |
| Stock four-substep data and fits repeat with the prior manual forces | E14 and prior f3c115bf control | Supported for bootstrap and fit; fresh full automatic completion budget-censored | [Exact 60-point repeat](sessions/2026-10-19-interactive-research-8d7a82a2/E14-bootstrap-repeat.json), [control log](sessions/2026-10-19-interactive-research-8d7a82a2/E14-stock-sub4/sweeps.full_auto.log) |
| A fresh matching world can continue measured four-substep data to automatic acceptance | E19 | Supported for this controlled trial: 100 seed + 40 fresh points | [Acceptance audit](sessions/2026-10-19-interactive-research-8d7a82a2/E19-acceptance-audit.json), [fresh-sweep comparison](sessions/2026-10-19-interactive-research-8d7a82a2/E19-fresh-continuation-parity.json) |

#### References

- [Full report](sessions/2026-10-19-interactive-research-8d7a82a2/report.md): equations, measured results, operating recipe and limitations.
- [Research notebook](sessions/2026-10-19-interactive-research-8d7a82a2/research.md): steering, compute budgets and chronological decisions.
- [Prior damping investigation, f3c115bf](sessions/2026-10-19-interactive-research-8d7a82a2/../2026-10-19-interactive-research-f3c115bf/report.md): the preceding four-substep control and unresolved post-move drift.
- [Prior endpoint investigation, bf5d807a](sessions/2026-10-19-interactive-research-8d7a82a2/../2026-10-19-interactive-research-bf5d807a/report.md): rejected local controller and interpretation of logs 008/009.
- [Prior motor load study](sessions/2026-10-19-interactive-research-8d7a82a2/../2026-10-16-hp4-roller-propagation/report.md): older stiffness counterfactual and alternative mechanisms.
- [Original reference-005 log](sessions/2026-10-19-interactive-research-8d7a82a2/../../../extended-autocal/no-buildup-reference-005/sweeps.full_auto_1.log) and [RRD audit](sessions/2026-10-19-interactive-research-8d7a82a2/E03-rrd-summary.json): 120 measurements, four completed collections and final browser-clock stall.
- [Scene complexity](sessions/2026-10-19-interactive-research-8d7a82a2/scene-complexity.json), [world configuration audit](sessions/2026-10-19-interactive-research-8d7a82a2/world-configuration-audit.json): actual mass, solver work and HP4/Q0/R30/native settings.
- [E19 dataset](sessions/2026-10-19-interactive-research-8d7a82a2/E19-clean-continuation/sweeps.json), [seed contract](sessions/2026-10-19-interactive-research-8d7a82a2/E19-clean-continuation/seed-contract.json), [run manifest](sessions/2026-10-19-interactive-research-8d7a82a2/E19-clean-continuation/sweeps.headless/manifest.json), [finalized event RRD](sessions/2026-10-19-interactive-research-8d7a82a2/E19-clean-continuation/sweeps.headless/hangprinter-20261019T215304.204951Z-8626c63b.rrd): provenance and automatic completion evidence.
- [Code provenance](sessions/2026-10-19-interactive-research-8d7a82a2/provenance.json), [retained patch](sessions/2026-10-19-interactive-research-8d7a82a2/retained-changes.patch), [source snapshot](sessions/2026-10-19-interactive-research-8d7a82a2/retained-source/): starting revision 3b3a7fec and the six retained changes.

## [2026-10-19-interactive-research-bf5d807a](sessions/2026-10-19-interactive-research-bf5d807a/)

[abstract.md](sessions/2026-10-19-interactive-research-bf5d807a/abstract.md) · [report.md](sessions/2026-10-19-interactive-research-bf5d807a/report.md) · [research.md](sessions/2026-10-19-interactive-research-bf5d807a/research.md)

### Physical validation of HP4 local endpoint discovery and backoff

Status: interrupted at user request — no validated HP4 fix.

#### Abstract

Following a failed manual HP4 test, we inspected actual effector translation, rotation and cable slack in sampled production Rust Native recordings, rather than accepting point counts as evidence of sound motion. The committed baseline reproduced a roughly34cm size-probe snap-back. Earlier plateau rollback and all-line backoff alone did not cure it. A smaller-step reeved probe stopping at sensor/drive encoder response3, before the observed gain peak, reduced the size-probe turnback to about33mm in E44 and reached about0.85m effector X during the first sweep probe. Backoff now retraces observed line-length states with all motors held; size return retraces the observed path. HP3 completed a four-point bidirectional sweep in E46; HP4 E45 failed at the first measurement in its second direction. These are simulated motion checks at requested25x with an explicit1physical-second-per-probe-RPC delay model, not hardware tests or measured browser timing. Persistent postfailure jostling from the user's browser was not reproduced during the15-second Native held observations. No calibration-accuracy or speedup claim is made.

E45 reached both HP4 endpoints, but the second direction failed its30-second physical encoder-settling deadline. Its final held observation retained body/cable oscillation. E47 increased the relative-response stop to90% of peak; it prematurely selected only44.5mm size travel and was cancelled at the user’s request. The candidate controller/profile changes were reverted; the full patch is preserved in `rejected-candidate.patch`. Existing d9620fd8 behavior is still unfixed.

#### Topics

HP4, HP3, autocalibration, endpoint discovery, pullout, encoder response, cable slack, effector trajectory, backoff, Rust Native, physical smoke test, Rerun.

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Previous numerical collection success establishes sound physical motion | E39, user008 | Rejected | [Baseline physical metrics](sessions/2026-10-19-interactive-research-bf5d807a/E39-baseline-physical-smoke/physical-metrics.json), [plot](sessions/2026-10-19-interactive-research-bf5d807a/E39-baseline-physical-smoke/physical-smoke.png) |
| Earlier plateau rollback alone removes the snap | E40 | Rejected | [Metrics](sessions/2026-10-19-interactive-research-bf5d807a/E40-early-retreat-physical-smoke/physical-metrics.json) |
| All-line observed-state backoff alone removes the snap | E41 | Rejected | [Plot](sessions/2026-10-19-interactive-research-bf5d807a/E41-observed-backoff-physical-smoke/physical-smoke.png) |
| Waiting for relative-response decline sufficiently avoids the problematic region | E43 | Mixed; still90mm size turnback | [Plot](sessions/2026-10-19-interactive-research-bf5d807a/E43-early-response-stop-smoke/physical-smoke.png) |
| Stop on rising encoder gain before its peak improves physical endpoint motion | E44 | Supported for one size probe and first sweep endpoint | [Plot](sessions/2026-10-19-interactive-research-bf5d807a/E44-before-gain-peak-smoke/physical-smoke.png), [metrics](sessions/2026-10-19-interactive-research-bf5d807a/E44-before-gain-peak-smoke/physical-metrics.json) |
| Observed-path size return supports complete HP3/HP4 collection | E45,E46 | Rejected for HP4: second-direction point settling timeout; HP3 four points complete | [HP4 log](sessions/2026-10-19-interactive-research-bf5d807a/E45-hp4-retrace-full-sweep/trial.log), [HP3 result](sessions/2026-10-19-interactive-research-bf5d807a/E46-hp3-retrace-full-sweep/result.json) |
| Persistent postfailure held-body motion is reproduced | E39–E44 | Not reproduced under this model | Recorded15-second held observations |

| A90%-of-peak relative stop preserves useful range | E47 | Rejected for size discovery:44.5mm free travel; remainder interrupted | [Partial log](sessions/2026-10-19-interactive-research-bf5d807a/E47-hp4-before-response-decline/trial.log) |

#### References

- [Research notebook](sessions/2026-10-19-interactive-research-bf5d807a/research.md): accepted steering, budgets and experimental decisions.
- [Previous numerical report at d9620fd8](sessions/2026-10-19-interactive-research-bf5d807a/report-d9620fd8.md): preserved previous conclusions; the new physical checks show their acceptance criteria were insufficient.
- [User008 log](sessions/2026-10-19-interactive-research-bf5d807a/../../../extended-autocal/no-buildup-reference-008/sweeps.full_auto.log):91.369wall-second endpoint failure before measurement collection.
- [Production smoke harness](sessions/2026-10-19-interactive-research-bf5d807a/smoke_trial.mjs), [launcher](sessions/2026-10-19-interactive-research-bf5d807a/run_smoke.py), [physical inspection](sessions/2026-10-19-interactive-research-bf5d807a/inspect_smoke.py): experiment implementation; each experiment archives its tested controller/profile sources and diff.
- [Pullout terminology](sessions/2026-10-19-interactive-research-bf5d807a/../../../../autocal/control/behaviors/PULLOUTS.md): physical anchors stay stationary; positioning sets held line lengths.

Tool audit: replay/telemetry already exist in scripts/profile_headless.mjs, scripts/replay_collection.py and tests/benchmark/autocal_recording_modes.mjs, with production FlightRecorder and MCP experiment comparison/context tools. No general world-plus-RRF restoration found; README says snapshots are observations. Missing capability narrowed to integrated production single-probe stage selection, phase-aligned diagnostics and explicit starting-state/replay-versus-live contracts. Wrote research/prompts/single-probe-diagnostic-workflow.md; corrected earlier handoff. Source audit only; no new simulation or runtime validation.

Owner steering: HP4 wobble is unrealistic and is to be treated as a simulation bug. Created research/prompts/hp4-simulation-wobble-bug.md for a separate root-cause investigation, reduced physical reproducer and Native before/after validation. Specific subsystem remains unproven; no new simulation or source changes this turn.

## [2026-10-19-interactive-research-f3c115bf](sessions/2026-10-19-interactive-research-f3c115bf/)

[abstract.md](sessions/2026-10-19-interactive-research-f3c115bf/abstract.md) · [report.md](sessions/2026-10-19-interactive-research-f3c115bf/report.md) · [research.md](sessions/2026-10-19-interactive-research-f3c115bf/research.md)

### HP4 autocal wobble: one XPBD damping displacement per step (per-visit re-evaluation is an energy source)

Status: wobble root cause fixed and reproduced in RRDs; full one-substep autocal
completion still blocked by a separate post-move drift.

#### Abstract

HP4 autocal at the default `--physics-substeps 1` did not settle: the effector
limit-cycled, settling checks timed out and force selection moved. A
same-backend A/B Rerun recording of a loaded HP4 checkpoint reproduces the
wobble: with the legacy damping term the late-window effector Z range is
4.07 mm with 3.63 rad/s peak angular speed at about a 24 ms period and cable
force swinging 18.0-46.2 N; with the fix it is 7.2e-14 m at zero angular speed
with steady force. Root cause: the XPBD damping displacement
`grad(C)·(x - x_prev)` was re-evaluated on every Gauss-Seidel visit, so HP4's
second of two visits consumed its own correction; at dt = 2 ms with
`damping = 5` and `compliance = 1/20000` (`gamma = 0.125`, `alphaTilde = 12.5`)
the term adds rather than removes energy. Freezing the displacement at its
first visit per step, mirrored in the JavaScript, Rust/native/Wasm, Python and
Warp solvers, settles the state and matches PBD guidance to keep damping out of
the position projection. Autocal still fails at one and two substeps, but on a
monotone 14-15 deg/s post-move encoder drift with no G-code in flight. A
four-substep control completes the same bootstrap and fit (score 0.5295), while
eight solver iterations at one substep fail identically and run 2.5x slower.
All evidence is simulated; no calibration-accuracy claim is made.

#### Topics

HP4, autocalibration, XPBD, damping displacement, Gauss-Seidel visit order,
energy injection, limit cycle, physics substeps, effector settling, PBD best
practice, Rerun RRD reproduction, headless simulation.

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Per-visit re-evaluation of the XPBD damping displacement adds energy and sustains the HP4 wobble | E05, E06, E07, E08, E09, E13, E14, E15 | Supported | [E13 A/B RRD](sessions/2026-10-19-interactive-research-f3c115bf/experiments/E13-rrd-js-pair/), [E14 visit trace](sessions/2026-10-19-interactive-research-f3c115bf/experiments/E14-visit-trace/visit-trace-summary.json), [E07 matrix](sessions/2026-10-19-interactive-research-f3c115bf/experiments/E07-matrix/matrix-fixed-vs-legacy.json) |
| More physics substeps mask the wobble by shrinking dt | E01, E07, E12 | Supported as masking, not a fix | [E07 matrix](sessions/2026-10-19-interactive-research-f3c115bf/experiments/E07-matrix/matrix-fixed-vs-legacy.json), [E12 timing](sessions/2026-10-19-interactive-research-f3c115bf/experiments/E12-timing/timing.jsonl) |
| Gravity, slack cables or non-taut friction cause the wobble | E05, E06 | Rejected | E05 gravity-off and 51/51 taut |
| Earlier load-bookkeeping and compliance fixes were sufficient | previous sessions, E01 | Rejected | [One-step autocal](sessions/2026-10-19-interactive-research-f3c115bf/../2026-10-16-hp4-one-step-autocal/report.md) |
| Freezing the displacement per step makes a full one-substep autocal run complete | sub1-s1, sub2-s1, sub1-fixedforce | Rejected as stated; still fails | [sub1 fixed-force log](sessions/2026-10-19-interactive-research-f3c115bf/autocal-sub1-fixedforce/sweeps.full_auto.log) |
| The remaining failure is a monotone post-move drift, not the 36 Hz wobble | sub1-fixedforce, prior session | Supported | [encoder trace](sessions/2026-10-19-interactive-research-f3c115bf/autocal-sub1-fixedforce/sweeps.headless/events.jsonl) |
| The post-move drift is timestep-controlled | sub4-fixedforce | Supported: four substeps complete bootstrap, a fourth sweep and a fit (score 0.5295) where 1 and 2 substeps time out | [sub4 log](sessions/2026-10-19-interactive-research-f3c115bf/autocal-sub4-fixedforce/sweeps.full_auto.log) |
| More solver iterations substitute for substeps at one step | sub1-iter8 | Rejected: same timeout, 2.5x slower | [iter8 log](sessions/2026-10-19-interactive-research-f3c115bf/autocal-sub1-iter8/sweeps.full_auto.log) |
| Damping only on the first visit is sufficient | E15 | Rejected (under-damps) | [E15 decay](sessions/2026-10-19-interactive-research-f3c115bf/experiments/E15-damping-splitting/decay.json) |

#### References

- [Full report](sessions/2026-10-19-interactive-research-f3c115bf/report.md): equation order, fix, measurements and limitations.
- [Research notebook](sessions/2026-10-19-interactive-research-f3c115bf/research.md): decisions, budgets, steering and next step.
- Macklin et al., *XPBD*, MIG 2016, DOI 10.1145/2994258.2994272, sec. 5.
- Macklin et al., *Small Steps in Physics Simulation*, SIGGRAPH 2019,
  DOI 10.1145/3345768.3355926, sec. 4.1, 6, 7.
- Müller et al., *Detailed Rigid Body Simulation with XPBD*, SCA 2020, sec. 3.6.
- Prior sessions: [effector rocking](sessions/2026-10-19-interactive-research-f3c115bf/../2026-10-16-hp4-effector-instability/report.md),
  [substep options](sessions/2026-10-19-interactive-research-f3c115bf/../2026-10-16-hp4-substep-options/report.md),
  [one-step autocal](sessions/2026-10-19-interactive-research-f3c115bf/../2026-10-16-hp4-one-step-autocal/report.md).

## [2026-10-20-interactive-research-38da70f4](sessions/2026-10-20-interactive-research-38da70f4/)

[abstract.md](sessions/2026-10-20-interactive-research-38da70f4/abstract.md) · [report.md](sessions/2026-10-20-interactive-research-38da70f4/report.md) · [research.md](sessions/2026-10-20-interactive-research-38da70f4/research.md)

### Stateful pinhole gates for the experimental HP4

Status: complete; experimental model delivered with measurement gaps.

#### Abstract

This study replaced the ideal, massless ABC reeving abstraction in the
experimental HP4 scene with six stateful two-port pinhole gates. Each gate has
one hidden cable-flow state, summed bearing inertia, viscous damping and dry
friction, while retaining distinct unilateral entry and exit constraints and no
internal reeving CableJoints. Native Rust, Wasm and JavaScript were kept on the
same state law, and the twelve hidden bearings were rendered as cylinders driven
by material transport. The model reduced cable joints from 51 to 33. An isolated
probe produced a nonzero static breakaway and up to 0.0157 N entry/exit
difference; a 200-step HP4 prefix produced up to 0.0781 N. A matched 1000-step
native trial was 2.64 times faster with a 0.000184 m final-Z difference. The
final mandatory physics gate passed all 11 checks. All evidence is simulated;
the friction parameters are not hardware-calibrated, reduced numerical damping
was not isolated, and long-run accuracy and browser appearance were not
established.

#### Topics

HP4, Hangprinter, CableJoints, pinhole gate, reeving, bearing inertia, dry
friction, cable transport, Native Rust, Wasm, visualization, numerical damping

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Summed bearing inertia and resistance make entry and exit tensions differ while retaining static friction and unilateral stopping | PG-01, PG-02 | Supported in isolated and short HP4 trials | [Evidence summary](sessions/2026-10-20-interactive-research-38da70f4/evidence/gate-summary.json), [isolated JS](sessions/2026-10-20-interactive-research-38da70f4/evidence/gate-isolated-js-summed.json), [HP4 native](sessions/2026-10-20-interactive-research-38da70f4/evidence/gate-native-summed-200.json) |
| JS, Native Rust and Wasm produce the same gate response | PG-01, PG-02, PG-04 | Supported for the checked state law and production prefix | [Wasm isolated](sessions/2026-10-20-interactive-research-38da70f4/evidence/gate-isolated-wasm-summed.json), [HP4 Wasm](sessions/2026-10-20-interactive-research-38da70f4/evidence/gate-wasm-summed-200.json), [final gate](sessions/2026-10-20-interactive-research-38da70f4/evidence/stability-gate.log) |
| The condensed topology improves runtime while staying near the original short trajectory | PG-03 | Supported only for one matched 1000-step native trial | [Evidence summary](sessions/2026-10-20-interactive-research-38da70f4/evidence/gate-summary.json), [performance probe](sessions/2026-10-20-interactive-research-38da70f4/probes/gate_perf_probe.mjs) |
| Hidden bearings can be rendered as cylinders whose rotation follows material transport and original counter-rotation | PG-03 | Mixed: structure and signs checked headlessly; ratio is approximate and browser appearance was not inspected | [Wheel ratios](sessions/2026-10-20-interactive-research-38da70f4/evidence/original-wheel-ratio-1000.json), [scene generator](sessions/2026-10-20-interactive-research-38da70f4/../../../../scripts/create_hp4_gate_scene.mjs) |
| The gate reduces numerical damping | None | Not tested: short trajectories do not isolate numerical damping | [Full report](sessions/2026-10-20-interactive-research-38da70f4/report.md) |
| The synthetic gate parameters reproduce a physical HP4 | None | Not tested: no hardware measurements or calibration were available | [Full report](sessions/2026-10-20-interactive-research-38da70f4/report.md) |

#### References

- [Full report](sessions/2026-10-20-interactive-research-38da70f4/report.md): implementation, measurements and limitations.
- [Research notebook](sessions/2026-10-20-interactive-research-38da70f4/research.md): experiment decisions and accepted steering.
- [Müller et al. (2018), Cable Joints](https://matthias-research.github.io/pages/publications/cableJoints.pdf):
  condensed cable constraints, unilateral distance limits, dynamic rest length
  and pinholes.
- [Prior gate investigation](sessions/2026-10-20-interactive-research-38da70f4/../2026-10-20-interactive-research-62300644/report.md):
  motivation, failed simple bridge and ideal-gate baseline.
- Baseline revision `b9983100`; final evidence was collected from the
  uncommitted stateful-gate worktree.

## [2026-10-20-interactive-research-62300644](sessions/2026-10-20-interactive-research-62300644/)

[abstract.md](sessions/2026-10-20-interactive-research-62300644/abstract.md) · [report.md](sessions/2026-10-20-interactive-research-62300644/report.md) · [research.md](sessions/2026-10-20-interactive-research-62300644/research.md)

### HP4 bearing spin, tension reporting and a two-port pinhole gate

Status: two-port gate implemented as an opt-in variant; physical validation remains open.

#### Abstract

Implemented an opt-in HP4 two-port pinhole_gate attachment with a condensed unilateral length solve, common tension and summed carrier force/torque Jacobians. It removes internal CableJoints rather than replacing them with sequential pinholes. The variant preserves hidden material length and series compliance, reduces cable joints from 51 to 33 and projection visits from 102 to 42 per two-iteration substep. Four analytic JS/Rust checks pass for combined mass response and asymmetric torque; three one-second native/Wasm/JS logo checks pass with sampled positions within one micrometre. The focused JS suites pass 350 checks and backend suites pass 119. Existing stability gate completion is recorded in gate-stability-final.log. All evidence is simulated. This first version has zero flow inertia and internal friction; the independent Python engine explicitly rejects it. Speedup, timestep-bias improvement, physical accuracy and long-run stability were not established. Visualization is independently configurable through a local polyline. The production default scene remains unchanged.

#### Topics

HP4, Hangprinter, Cable Joints, passive pulley, bearing spin, pinhole gate, reeving, XPBD, numerical damping, tension telemetry, multiplier accumulation, torque mode, timestep convergence, energy conservation, autocalibration, Print Logo, stability regression, headless validation

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Rolling links transport material independently of wheel rotation | E01, E15, E16 | Supported: frozen rotation still permits rest redistribution; isolated pulley spins under equalized tension | [Frozen wheel](sessions/2026-10-20-interactive-research-62300644/E01-frozen-wheel.json), [pulley](sessions/2026-10-20-interactive-research-62300644/E15-pulley.json), [energy audit](sessions/2026-10-20-interactive-research-62300644/pulley-energy.json) |
| This redistribution explains startup bearing runaway | E02, E03, E06, E07, E18, E28 | Supported in held HP4 trials; removing it suppresses persistent spin | [Motion summary](sessions/2026-10-20-interactive-research-62300644/motion-summary.json), [plot](sessions/2026-10-20-interactive-research-62300644/mechanics-summary.png) |
| Removing rolling redistribution alone repairs HP4 dynamics | E09, E18–E20, E26; candidate integration checks | Rejected as a sufficient correction: slow motion/timestep dependence and loaded-state instability remain | [Candidate patch](sessions/2026-10-20-interactive-research-62300644/no-slip-candidate.patch), [integration failures](sessions/2026-10-20-interactive-research-62300644/backend-tests.log) |
| Large displayed tension differences necessarily mean equally large physical imbalance | E27, E28 | Rejected for the held D path: accumulated multipliers nearly agree | [Multiplier audit](sessions/2026-10-20-interactive-research-62300644/multiplier-audit.json) |
| A rigid two-port gate preserves ideal ABC path geometry and wrench | E25 | Supported for geometry and an ideal common tension; no dynamic validation | [Gate contract](sessions/2026-10-20-interactive-research-62300644/E25-gate-contract.json), [prototype](sessions/2026-10-20-interactive-research-62300644/pinhole_gate.py) |
| Replacing the internal route with a simple two-pinhole bridge is sufficient to remove timestep bias | E10–E13, E21, E22 | Rejected as sufficient; gate abstraction remains promising | [Inputs and motion](sessions/2026-10-20-interactive-research-62300644/motion-summary.json), [bridge harness](sessions/2026-10-20-interactive-research-62300644/mechanics.mjs) |
| The faster motor formula remains a promising mechanics direction | E14, E23, E24, E29–E31, compared with E19, E20, E26 | Supported as a direction by shrinking formula disagreement with timestep; physical accuracy remains inconclusive | [Native convergence results](sessions/2026-10-20-interactive-research-62300644/motion-summary.json) |
| Removing passive-wheel backdrive through a pinhole is sufficient | E17 | Rejected as sufficient; this isolated change reduces Y travel further | [E17](sessions/2026-10-20-interactive-research-62300644/E17.json), [frozen variant source](sessions/2026-10-20-interactive-research-62300644/E17-candidate-core.mjs) |
| Accumulated motor-load feedback alone or with the faster update improves dynamics | E33–E36 | Mixed: better force coupling/lateral response; timestep error and integrated stability remain unresolved | [Saved trials](sessions/2026-10-20-interactive-research-62300644/motion-summary.json) |
| The mandatory headless gate catches the shipped default-logo regression | S01–S03, S06 | Supported: broken engine fails after 0.586 s; corrected prefix and loaded checks pass | [Negative control](sessions/2026-10-20-interactive-research-62300644/integration/gate-rejects-broken-final.log), [gate](sessions/2026-10-20-interactive-research-62300644/integration/stability-gate-final.log) |
| Terminal tracking loss is reproduced by the original engine from the same checkpoint | S04, S07 | Supported: 5500 identical body samples and identical 300 lost steps; no whole-job accuracy pass | [Tail comparison](sessions/2026-10-20-interactive-research-62300644/integration/tail-comparison.json) |
| 64-substep comparison | E32 | Not tested: the supported 1–16 range rejects construction before physics advances | [Failure log](sessions/2026-10-20-interactive-research-62300644/E32.log), [input](sessions/2026-10-20-interactive-research-62300644/E32-input.usda) |

#### References

- [Full report](sessions/2026-10-20-interactive-research-62300644/report.md): mechanics, measured comparisons, failed checks and limits.
- [Research notebook](sessions/2026-10-20-interactive-research-62300644/research.md): accepted steering, budgets and next decision.
- [Provenance](sessions/2026-10-20-interactive-research-62300644/provenance.json): input/model mapping, source and binary hashes.
- [Rollout summary](sessions/2026-10-20-interactive-research-62300644/motion-summary.json), [multiplier audit](sessions/2026-10-20-interactive-research-62300644/multiplier-audit.json), [pulley energy](sessions/2026-10-20-interactive-research-62300644/pulley-energy.json), [gate contract](sessions/2026-10-20-interactive-research-62300644/E25-gate-contract.json): measured and synthetic evidence.
- [Previous HP4 investigation](sessions/2026-10-20-interactive-research-62300644/../2026-10-19-interactive-research-8d7a82a2/report.md): continued faster-motor and autocal work.
- Müller, Chentanez, Jeschke and Macklin (2018), *Cable Joints*, [DOI 10.1111/cgf.13507](https://doi.org/10.1111/cgf.13507): rolling/no-slip and pinhole model.
- Macklin, Müller and Chentanez (2016), *XPBD*, [DOI 10.1145/2994258.2994272](https://doi.org/10.1145/2994258.2994272): accumulated multiplier force estimate.
- [PinholeGate prototype](sessions/2026-10-20-interactive-research-62300644/pinhole_gate.py) and [reproduction commands](sessions/2026-10-20-interactive-research-62300644/REPRODUCE.md): reusable ideal geometry implementation and all candidate workflows.

#### Subsequent requested integration

The user requested three separate commits: `9ebc8337` removes independent rolling slip; `41010aa1` fixes total-multiplier force readback and final force transfers; `628a8963` fixes signed motor-load accumulation and counts K/D once. The alternative motor formula is unchanged. [Separate formulation report](sessions/2026-10-20-interactive-research-62300644/../../../../research/faster-motor-update.md) explains its origin, derivation and timestep behavior.

Final checks: 188 Python, 110 Warp CPU, 20 JS and 115 native/Wasm passes, with six CUDA/GC skips. Five known HP4 failures remain: three legacy loaded-checkpoint settling cases and two tight JS/Wasm trajectory comparisons. [Integration manifest](sessions/2026-10-20-interactive-research-62300644/integration/provenance.json) links the exact logs, rebuilt binaries and I01–I04 trials. I01/I03 have exactly equal sampled physical state across the display-only fix; I04 reproduces the earlier feedback candidate. These checks do not establish calibration accuracy or a complete dynamics repair.

The bridge experiment solved each remaining span sequentially; deleting wheels did not combine those spans into one gate/path constraint. That finite-iteration splitting still changes with timestep. A properly coupled two-port gate remains a proposed extension.

#### Stability correction

The prior three-commit acceptance was invalid. See [S01–S06 evidence](sessions/2026-10-20-interactive-research-62300644/integration/),
[notebook](sessions/2026-10-20-interactive-research-62300644/research.md#stability-correction-progress) and the forthcoming correction
section in the full report. S01/S02/S03 support restoring stable logo behavior;
S06 supports the gate detecting the rejected engine. The complete native replay and original-engine terminal comparison are complete; the inherited terminal tracking loss is documented below.

See the [stability correction report](sessions/2026-10-20-interactive-research-62300644/../../../../research/hp4-logo-stability.md) and [validation index](sessions/2026-10-20-interactive-research-62300644/integration/STABILITY_VALIDATION.md) for the final source decision and executed checks.

Corrective source revision: `06868dd8d1d8f45f950b629da8d73a180708c9c7`; unsafe shortcut withdrawn.

#### Gate implementation

Implemented an opt-in HP4 two-port pinhole_gate attachment with a condensed unilateral length solve, common tension and summed carrier force/torque Jacobians. It removes internal CableJoints rather than replacing them with sequential pinholes. The variant preserves hidden material length and series compliance, reduces cable joints from 51 to 33 and projection visits from 102 to 42 per two-iteration substep. Four analytic JS/Rust checks pass for combined mass response and asymmetric torque; three one-second native/Wasm/JS logo checks pass with sampled positions within one micrometre. The focused JS suites pass 350 checks and backend suites pass 119. Existing stability gate completion is recorded in gate-stability-final.log. All evidence is simulated. This first version has zero flow inertia and internal friction; the independent Python engine explicitly rejects it. Speedup, timestep-bias improvement, physical accuracy and long-run stability were not established. Visualization is independently configurable through a local polyline. The production default scene remains unchanged.

- G01: combined mass/Jacobian and separate-port wrench, four analytic JS/Rust checks: supported. [Final checks](sessions/2026-10-20-interactive-research-62300644/gate-tests-final3.log).
- G02: one-second production motion and shared-tension readback, native/Wasm/JS: supported for this bounded prefix; sampled position agreement within1e-6m. [Checks](sessions/2026-10-20-interactive-research-62300644/gate-tests-final3.log).
- G03: baseline preservation and automatic protection:119 backend checks and350 focused Jest checks pass; mandatory gate log below. [Backend](sessions/2026-10-20-interactive-research-62300644/gate-backends.log), [JS](sessions/2026-10-20-interactive-research-62300644/gate-js-final2.log), [stability](sessions/2026-10-20-interactive-research-62300644/gate-stability-final.log).
- Reduced numerical damping or overall wall time: not tested. Structural projection-count reduction is not a speed benchmark.

Initial failures are preserved: native enum validation initially rejected kind5; a test cleanup called an unavailable dispose method; JS initially double-counted spool rotation in damping, corrected by separating carrier angular and rotor coordinates. A broad textual edit also touched the unrelated merge function; focused tests caught it and the duplicate block was removed. Source wiring/preset test fixtures were corrected and rerun.

[Implementation and usage](sessions/2026-10-20-interactive-research-62300644/../../../../research/pinhole-gate.md),
[generated gate scene](sessions/2026-10-20-interactive-research-62300644/../../../../public/usd_scenes/hp4_pinhole_gate.usda),
[scene generator](sessions/2026-10-20-interactive-research-62300644/../../../../scripts/create_hp4_gate_scene.mjs),
[executable checks](sessions/2026-10-20-interactive-research-62300644/../../../../tests/benchmark/pinhole_gate.test.mjs).
No browser experiments, long collection or full-logo runs were performed for this feature.

## [2026-10-20-interactive-research-ae0174ce](sessions/2026-10-20-interactive-research-ae0174ce/)

[abstract.md](sessions/2026-10-20-interactive-research-ae0174ce/abstract.md) · [report.md](sessions/2026-10-20-interactive-research-ae0174ce/report.md) · [research.md](sessions/2026-10-20-interactive-research-ae0174ce/research.md)

### HP3 autocalibration at fresh centered and asymmetric encoder references

Status: completed bounded study through E17; no production solver or stopping change accepted.

#### Abstract

Does moving the encoder reference near the anchors' centroid improve HP3 calibration or reduce collection effort? A retrospective fixed-offset rebase worsened 33/36 paired fits, but a causal audit showed changed exact-reference/noise assumptions and origin-dependent seeds/bounds; those failures cannot establish an inherent advantage of the original origin. Removing or falsifying stored M669 anchors gave bit-identical fits, with no direct truth leakage found. We then collected fresh native-Rust HP3 datasets at the original reference, at the exact virtual-anchor centroid (+265 mm vertically), and at an asymmetric reference, using identical manual forces, known 30 mm radii, zero buildup, no layering and normal automatic stopping. Final anchor RMSE was 4.588/4.277/12.602 mm using 140/180/300 points and 1.051/1.131/2.072 million physics steps. Centering slightly improved final accuracy but increased collection effort. A larger asymmetric start failed initial encoder settling. Sampled recordings show small measurement-time rotation for symmetric references but up to 2.11° for the asymmetric run. Limitations: single conditions on one nominal layout, deterministic simulation, changed physical trajectories, and no demonstrated fewer-point or hardware gain.

#### Topics

HP3, CDPR, encoder autocalibration, fresh centered reference, origin shift, anchor scale, solver conditioning, symmetric initialization, geometry prerequisites, ground truth leakage audit, warm start, automatic stopping, settling, Rerun, fewer data points

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Low fitting scores reliably identify accurate geometry | E01 | Rejected: seven material adjacent disagreements | [Baseline](sessions/2026-10-20-interactive-research-ae0174ce/E01/results.json), [summary](sessions/2026-10-20-interactive-research-ae0174ce/summary.json) |
| Exact centroid translation uniformly improves local conditioning | E02-bootstrap, E02-corrected | Mixed: modest bootstrap improvement, worse at five sweeps | [Bootstrap](sessions/2026-10-20-interactive-research-ae0174ce/E02-bootstrap/results.json), [five sweeps](sessions/2026-10-20-interactive-research-ae0174ce/E02-corrected/results.json) |
| The implemented nearest-center rebase improves recovery | E03–E06 | Rejected for this workflow; general coordinate-invariant relocation not tested | [Corrected matrix](sessions/2026-10-20-interactive-research-ae0174ce/E05/results.json), [CLI replay](sessions/2026-10-20-interactive-research-ae0174ce/E06/results.json) |
| A feasible asymmetric retrospective reference improves recovery with less data | E05–E06 | Mixed at three sweeps; rejected as a robust policy | [Corrected matrix](sessions/2026-10-20-interactive-research-ae0174ce/E05/results.json), [CLI replay](sessions/2026-10-20-interactive-research-ae0174ce/E06/results.json) |
| Rebased fits are useful seeds after restoring the original objective | E07 | Rejected as a reliable improvement | [Polish](sessions/2026-10-20-interactive-research-ae0174ce/E07/results.json) |
| Reference transformation preserves residuals at one exactly translated geometry and supports ground-truth-free inversion | E02, E08, unit tests | Supported locally; this does not establish equivalent objective families/noise | [CLI checks](sessions/2026-10-20-interactive-research-ae0174ce/E08-cli/checks.log), [tests](sessions/2026-10-20-interactive-research-ae0174ce/tests.log) |
| Shifting reduces catastrophic intermediate failures under default L-BFGS-B | E09 | Rejected for retrospective workflow: chosen fits >100 mm, original 0/17, center 7/17, edge 5/17 | [Reliability audit](sessions/2026-10-20-interactive-research-ae0174ce/E09-reliability.json) |
| Stored M669 ground truth is used by the tested fit path | E10 | Rejected: deletion/falsification produce bit-identical fits | [Controls](sessions/2026-10-20-interactive-research-ae0174ce/E10-causal-audit/results.json) |
| The fixed-offset rebase is a statistically equivalent coordinate change | E11 | Rejected | [Reference residuals](sessions/2026-10-20-interactive-research-ae0174ce/E11-reference-assumptions/results.json) |
| Correctly translating the bootstrap estimate can prevent shifted warm-chain catastrophes | E12 | Supported for one diagnostic chain; no full-auto policy claim | [Warm chain](sessions/2026-10-20-interactive-research-ae0174ce/E12-warm-chain/results.json) |
| Fresh centered-origin collection improves accuracy with less effort | E13–E15 | Mixed: 4.588 -> 4.277 mm, but 140 -> 180 points | [Fresh results](sessions/2026-10-20-interactive-research-ae0174ce/fresh-results.json) |
| TRF resolves the new cold bootstrap failures | E15 | Supported for these two recorded three-sweep prefixes; not a default-policy validation | [Fresh fits](sessions/2026-10-20-interactive-research-ae0174ce/E15-fresh-fits/results.json) |
| A larger asymmetric start yields usable calibration data | E16 | Inconclusive for accuracy: initial settling timed out before measurements | [Failed trial](sessions/2026-10-20-interactive-research-ae0174ce/E16-offset/sweeps.full_auto.log) |
| The smaller asymmetric start improves collection efficiency | E17 | Rejected in this trial: 12.602 mm, 300 points; rotation complicates interpretation | [Fresh results](sessions/2026-10-20-interactive-research-ae0174ce/fresh-results.json), [rotation](sessions/2026-10-20-interactive-research-ae0174ce/sampled-rotation.json) |

#### References

- [Full report](sessions/2026-10-20-interactive-research-ae0174ce/report.md): methods, corrected interpretation, measurements and limitations.
- [Research notebook](sessions/2026-10-20-interactive-research-ae0174ce/research.md): decisions, budgets, accepted steering and failed pilots.
- [Fresh dataset/plant/result summary](sessions/2026-10-20-interactive-research-ae0174ce/fresh-results.json): E13–E17 manifests, stage fits and metrics.
- [Fresh source and input audit](sessions/2026-10-20-interactive-research-ae0174ce/fresh-manifest.json): exact scripts, scene/firmware inputs, native artifact and readiness checks.
- [Fresh baseline dataset](sessions/2026-10-20-interactive-research-ae0174ce/E14-baseline/sweeps.json), [centered dataset](sessions/2026-10-20-interactive-research-ae0174ce/E14-center/sweeps.json), [asymmetric dataset](sessions/2026-10-20-interactive-research-ae0174ce/E17-offset-small/sweeps.json): newly collected production encoder/noise records.
- [Causal controls](sessions/2026-10-20-interactive-research-ae0174ce/E10-causal-audit/results.json), [reference-assumption audit](sessions/2026-10-20-interactive-research-ae0174ce/E11-reference-assumptions/results.json) and [warm-chain audit](sessions/2026-10-20-interactive-research-ae0174ce/E12-warm-chain/results.json): qualify earlier retrospective results.
- [Median and reliability audit](sessions/2026-10-20-interactive-research-ae0174ce/E09-reliability.json): process completion, automatic stopping, cold/warm fits and chosen iterations counted separately.
- [Offline revision and source hashes](sessions/2026-10-20-interactive-research-ae0174ce/manifest.json): initial experiment provenance; original inputs verified unchanged.
- [Original transform snapshot](sessions/2026-10-20-interactive-research-ae0174ce/source/autocal/tools/rebase_dataset.py): numerical transform used in the earlier trials; its later documentation explicitly warns about changed reference assumptions.
- [Supplied HP3 reference](sessions/2026-10-20-interactive-research-ae0174ce/../../../extended-autocal/no-buildup-reference-003/sweeps.json): REF source observations.
- [Varied-world study](sessions/2026-10-20-interactive-research-ae0174ce/../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/report.md) and [world matrix](sessions/2026-10-20-interactive-research-ae0174ce/../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/environment-matrix-20261015-retry1/matrix_summary.json): W00/W08 source simulations and evaluation geometry.
- [Warm-start study](sessions/2026-10-20-interactive-research-ae0174ce/../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/report.md): prior sparse-data and initialization failures motivating matched seeds.

## [2026-10-20-interactive-research-edf2eb7d](sessions/2026-10-20-interactive-research-edf2eb7d/)

[abstract.md](sessions/2026-10-20-interactive-research-edf2eb7d/abstract.md) · [research.md](sessions/2026-10-20-interactive-research-edf2eb7d/research.md)

### interactive research

Status: not yet summarized.

#### Abstract

The session has started. No research result has been summarized yet.

#### Topics

interactive research

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| None recorded yet | None | Not tested | None |

#### References

- [Original task](sessions/2026-10-20-interactive-research-edf2eb7d/prompt.txt)
- [Research notebook](sessions/2026-10-20-interactive-research-edf2eb7d/research.md)

## [2026-10-21-interactive-research-cdfb5278](sessions/2026-10-21-interactive-research-cdfb5278/)

[abstract.md](sessions/2026-10-21-interactive-research-cdfb5278/abstract.md) · [report.md](sessions/2026-10-21-interactive-research-cdfb5278/report.md) · [research.md](sessions/2026-10-21-interactive-research-cdfb5278/research.md)

### HP4 Print Squares substep bias and PR 77 review

Status: complete; inertia correction validated, damping correction deferred.

#### Abstract

PR #77 proposes two corrections for HP4’s greater step loss at finer physics substeps. We reproduced both local analytic defects on main, replayed the bundled Squares commands in fresh native production worlds, and separated the corrections with the mandatory default-step stability gate. Removing cable-only electromagnetic stiffness changes a 2 ms spring-loaded spool equilibrium from 0.002386 to the analytic 0.005946 rad; 20 checks across JS, Wasm kernel/full engine and native pass, with independent Python/Warp validation. The separate rotor damping correction fails loaded-state settling, alone and combined, producing 5.008 mm persistent vertical motion; it is deferred. The retained inertia correction passes all 11 mandatory physics checks. Squares still loses two B magnetic periods at finer settings, and one C period at 8–16 substeps. This corrects artificial coarse-step holding power, not full-print tracking. All evidence is simulated; hardware, continuum accuracy and calibration accuracy remain unmeasured.

#### Topics

HP4, Print Squares, missed steps, physics substeps, open-loop torque balance, rotor inertia, cable damping, loaded settling, PR 77

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| H1: Cable-only motor stiffness biases static torque balance | E01, E03, E06, E07 | Supported | [Baseline failures](sessions/2026-10-21-interactive-research-cdfb5278/E01-baseline-analytic.log), [final equilibrium checks](sessions/2026-10-21-interactive-research-cdfb5278/E07-static-equilibrium-final.log) |
| H2: Standalone rotor damping double-counts displacement, and its local correction integrates safely | E01, E03–E05 | Mixed: local law supported, integrated safety rejected | [Analytic passes](sessions/2026-10-21-interactive-research-cdfb5278/E03-candidate-analytic.log), [damping-only failure](sessions/2026-10-21-interactive-research-cdfb5278/E05-damping-only-physics-gate.log) |
| H3: Finer-step Squares pull-out persists after correction | E02, E03, E06, E08 | Supported for bounded prefixes | [Baseline](sessions/2026-10-21-interactive-research-cdfb5278/E02-baseline.json), [retained correction](sessions/2026-10-21-interactive-research-cdfb5278/E06-inertia-only.json), [20 s prefixes](sessions/2026-10-21-interactive-research-cdfb5278/E08-inertia-only-20s.json) |
| H4: Both PR corrections can land together on current main | E04, E05 | Rejected | [Combined failure](sessions/2026-10-21-interactive-research-cdfb5278/E04-candidate-physics-gate.log), [damping-only failure](sessions/2026-10-21-interactive-research-cdfb5278/E05-damping-only-physics-gate.log) |
| H5: Inertia-only correction repairs local balance and preserves required stability | E06, E07 | Supported for tested scope | [Passing gate](sessions/2026-10-21-interactive-research-cdfb5278/E06-inertia-only-physics-gate.log), [analytic checks](sessions/2026-10-21-interactive-research-cdfb5278/E07-static-equilibrium-final.log), [Python/Warp](sessions/2026-10-21-interactive-research-cdfb5278/E07-python-warp-final.log) |
| H6: Exact equilibrium assertions remain valid at four nonlinear projections | E07 audit | Rejected at strict roundoff tolerances; small remaining projection error | [Audit](sessions/2026-10-21-interactive-research-cdfb5278/E07-equilibrium-audit.json), [initial failures](sessions/2026-10-21-interactive-research-cdfb5278/E07-static-equilibrium-initial-failed.log) |
| QC refresh fixes startup HUD in an interactive browser | None | Not tested; source review only | [PR input](sessions/2026-10-21-interactive-research-cdfb5278/pr77-review-input.json) |

#### References

- [Full report](sessions/2026-10-21-interactive-research-cdfb5278/report.md), [research notebook](sessions/2026-10-21-interactive-research-cdfb5278/research.md): accepted scope, decisions and limitations.
- [PR #77](https://github.com/tobbelobb/hp-sim5/pull/77), reviewed head [fcdaf7fe](https://github.com/tobbelobb/hp-sim5/commit/fcdaf7fed43a4cb1a9efdd289a58980ea2840984), physics commit [63b847c8](https://github.com/tobbelobb/hp-sim5/commit/63b847c842f79519104f7626563f1a2e71e09658): source of proposed corrections and analytic laws; [archived connector input](sessions/2026-10-21-interactive-research-cdfb5278/pr77-review-input.json).
- [Provenance](sessions/2026-10-21-interactive-research-cdfb5278/provenance.json), [replay harness](sessions/2026-10-21-interactive-research-cdfb5278/squares-replay.mjs): baseline revision 2c0b86ff, hashes of production scene, firmware axis map, source CAN, decoded commands and native artifacts.
- [Tracked integration review](sessions/2026-10-21-interactive-research-cdfb5278/../../../../research/hp4-squares-substeps.md): derivation, measured tables, test commands and code references.
- [Previous HP4 stability session](sessions/2026-10-21-interactive-research-cdfb5278/../2026-10-20-interactive-research-62300644/report.md), [stability correction](sessions/2026-10-21-interactive-research-cdfb5278/../../../../research/hp4-logo-stability.md), [motor formulation](sessions/2026-10-21-interactive-research-cdfb5278/../../../../research/faster-motor-update.md): prior dynamics limitations and rationale for the unchanged gate.
- [Recording cost guidance](sessions/2026-10-21-interactive-research-cdfb5278/../../../../research/autocal-recording-performance.md): reason to choose numerical-only evidence; no performance comparison is claimed here.

- [Commit patch](sessions/2026-10-21-interactive-research-cdfb5278/hp4-open-loop-torque-balance.patch) and [bundle](sessions/2026-10-21-interactive-research-cdfb5278/hp4-open-loop-torque-balance.bundle): packaged commit `838133d4`; original `.git` is read-only and source edits remain in the working tree.

## [2026-10-22-autocal-cli-orchestration](sessions/2026-10-22-autocal-cli-orchestration/)

[abstract.md](sessions/2026-10-22-autocal-cli-orchestration/abstract.md) · [report.md](sessions/2026-10-22-autocal-cli-orchestration/report.md) · [research.md](sessions/2026-10-22-autocal-cli-orchestration/research.md)

### Everyday autocal help and extracted recovery/stopping

Status: complete.

#### Abstract

Ordinary autocal help now shows routine settings and a short usage line; extended
`--help-research` lists every control by topic. At an 80-column terminal, ordinary
help shrinks from 243 to 72 lines. All existing flags remain parseable without an
extra enable switch. Representative parser namespaces match the starting revision
exactly. Recovery orchestration and stopping state move into dedicated modules,
reducing `autocal.py` from 1,836 to 1,434 lines. Recovery ordering, sparse seeding,
patience, consensus and stopping precedence retain their behavior. The non-slow
Python suite passes 538 tests, and four recorded calibration regressions preserve
parameter comparisons and totals of 27 sweeps / 470 points. These are interface,
refactoring and compatibility results, not an accuracy or speed improvement.
No physics, motor, cable transport or scene dynamics implementation changed.

#### Topics

autocalibration, CLI help, research controls, recovery, stopping, refactoring,
compatibility, parser defaults

#### Tested hypotheses

| Hypothesis | ID | Outcome | Evidence |
| --- | --- | --- | --- |
| Help views preserve existing defaults and acceptance of tuning options | P01 | Supported on representative namespaces and parser tests | [Parser comparison](sessions/2026-10-22-autocal-cli-orchestration/parser-equivalence.json), [tests](sessions/2026-10-22-autocal-cli-orchestration/tests.log) |
| Recovery/stopping extraction preserves recorded calibration behavior | R01 | Supported on four fixtures | [Regressions](sessions/2026-10-22-autocal-cli-orchestration/regressions.log) |
| Ordinary help is substantially shorter | H01 | Supported at 80 columns, 243 → 72 lines | [Before](sessions/2026-10-22-autocal-cli-orchestration/help-before.txt), [after](sessions/2026-10-22-autocal-cli-orchestration/help.txt), [summary](sessions/2026-10-22-autocal-cli-orchestration/summary.json) |

#### References

- [Report](sessions/2026-10-22-autocal-cli-orchestration/report.md), [notebook](sessions/2026-10-22-autocal-cli-orchestration/research.md).
- [Full help](sessions/2026-10-22-autocal-cli-orchestration/help-research.txt), [parser comparison](sessions/2026-10-22-autocal-cli-orchestration/parser-equivalence.json).

## [2026-10-22-autocal-filtering-integration](sessions/2026-10-22-autocal-filtering-integration/)

[abstract.md](sessions/2026-10-22-autocal-filtering-integration/abstract.md) · [report.md](sessions/2026-10-22-autocal-filtering-integration/report.md) · [research.md](sessions/2026-10-22-autocal-filtering-integration/research.md)

### Autocal Lane 02 cleanup and reference-comparison audit

Status: complete.

#### Abstract

Lane 02 found an unused anchor-fit threshold and a sigma override that did not
control per-point normalization. Four bounded frozen-fit replays confirmed that
changing threshold 250 to 0.01 preserves anchors, radii and both costs exactly;
the 1 mm sigma override preserved parameters and normalized cost but changed raw
cost. The cleanup removes both options and their plumbing, consolidates paired
filter booleans, clarifies filtering/noise help and preserves old stage-artifact
replay. A post-change replay exactly matches the control. Three complete fixture
regressions preserve final and iteration parameters, scores and collection counts;
267 targeted tests pass. A provenance check also resolves the reported layered
accuracy gap: Lane 02's own final consensus is 1.58 mm anchor RMSE, matching the
later October reference; its 7.68 mm result is an individual fit. No filtering,
solver or stopping default was changed, and no accuracy or speed gain is claimed.
Evidence is recorded simulation and source tracing, not new hardware or collection.


#### Topics

autocalibration, filtering, noise sigma, dead flags, threshold, parser inheritance,
stage artifacts, reference provenance, consensus, solver fallback

#### Tested hypotheses

| Hypothesis | IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Anchor-fit threshold changes the frozen fit | T1 | Rejected | [Before checks](sessions/2026-10-22-autocal-filtering-integration/before-checks.json) |
| Sigma override is entirely inert | S1 | Rejected for raw cost; parameters unchanged | [Before checks](sessions/2026-10-22-autocal-filtering-integration/before-checks.json) |
| Default numerics survive cleanup | C1/R1 | Supported on tested replay/fixtures | [After checks](sessions/2026-10-22-autocal-filtering-integration/after-checks.json), [regressions](sessions/2026-10-22-autocal-filtering-integration/regression.log) |
| Layered comparison establishes fallback regression | H1 | Rejected as inference | [Summary comparison](sessions/2026-10-22-autocal-filtering-integration/summary-comparison.json), [history](sessions/2026-10-22-autocal-filtering-integration/fallback-history.json) |

#### References

- [Report](sessions/2026-10-22-autocal-filtering-integration/report.md): implementation and evidence.
- [Notebook](sessions/2026-10-22-autocal-filtering-integration/research.md): decisions and commands.

## [2026-10-22-autocal-flags-06-planning-stopping](sessions/2026-10-22-autocal-flags-06-planning-stopping/)

[abstract.md](sessions/2026-10-22-autocal-flags-06-planning-stopping/abstract.md) · [report.md](sessions/2026-10-22-autocal-flags-06-planning-stopping/report.md) · [research.md](sessions/2026-10-22-autocal-flags-06-planning-stopping/research.md)

### Autocal planning, recovery and stopping flags: candidate grid, ridge, stop rules and sparse recovery

Status: complete for the bounded initial screen (lane 06). No fresh collection, movement, browser or hardware evidence.

#### Abstract

Question: which of the 16 planning, recovery, stopping and candidate-count options in `autocal.py` are used, and which could be removed, hard-coded, redesigned or retained? Method: three lane-local full-auto replays on the development fixtures (no collection); 12 `replay_stage plan` executions (four variants: baseline, grid 21/81, ridge 1e-8 on each of three iteration-3 fits); three sparse-recovery replays; counterfactual stop rules on the logged prefixes; non-ranking information and candidate-feasibility diagnostics. Measured results: the default ridge (1e-6) is the same order as the observed information (about 1e-7 to 2e-6) and changes the HP3 choice when lowered to 1e-8; the chosen sweep on two fixtures moves with the grid (938, 990 and 1016 mm); `--fd-eps-mm` in 0.1–10 mm changes candidate information negligibly, and exact-repeat exclusion is redundant at the default 20 mm spacing; `--stop-std-mm` and `--stop-cost` are not usable thresholds on these fixtures; sparse recovery is mixed. Main limitations: each replay ran five iterations and stopped with one to three reference sweeps unused, so stop rules are judged only inside that prefix; the independent error is simulation truth, not held-out prediction; no planner choice was validated by a new measurement.

#### Topics

autocalibration, Hangprinter, CDPR, D-optimal sweep planning, Fisher information ridge, candidate grid density, duplicate exclusion, underconstrained recovery, sparse seed, patience, consensus stopping, uncertainty stop thresholds, replay counterfactuals.

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| H1. The chosen sweep is stable when the candidate grid density changes 21/41/81 | 06-P-{five,layered,hp3}-{baseline,grid21,grid81} | Rejected: sweep changes (five and layered delta 938/990/1016 mm; HP3 different fixed anchors at grid 21) | [trials.json](sessions/2026-10-22-autocal-flags-06-planning-stopping/trials.json), [report §3](sessions/2026-10-22-autocal-flags-06-planning-stopping/report.md#3-planning-screen) |
| H2. The default ridge 1e-6 is a negligible numerical stabilizer | 06-P-*-ridge1e-8, 06-D-hp3-* | Rejected: same order as observed information; HP3 choice changes at 1e-8 | [report §3.2](sessions/2026-10-22-autocal-flags-06-planning-stopping/report.md#32-ridge-and-information-scale) |
| H3. `--fd-eps-mm` between 0.1 and 10 mm does not change candidate information | 06-D-*-eps0.1, 06-D-*-eps10.0 | Supported (information-level; ranking not re-run) | [diag_eps_exclusion.json](sessions/2026-10-22-autocal-flags-06-planning-stopping/diag_eps_exclusion.json) |
| H4. Existing-sweep exclusion is redundant at default spacing | 06-D-*-spacing20.0, 06-D-*-spacing0.0 | Supported at 20 mm; false at 0 mm (removes 5 exact repeats per fixture) | [diag_eps_exclusion.json](sessions/2026-10-22-autocal-flags-06-planning-stopping/diag_eps_exclusion.json) |
| H5. Patience 1/3/5 changes the final result on the development replays | 06-S-* (counterfactual on 06-B1..B3) | Mixed and inconclusive: patience 1 would have chosen a better prefix on five-point (7.04 vs 11.78 mm) and saved two sweeps on HP3 with the same error; patience 2 and 5 are unobserved beyond iteration 5 | [report §5.4](sessions/2026-10-22-autocal-flags-06-planning-stopping/report.md#54-patience) |
| H6. A stop threshold on `max_std_mm` or `cost` can be chosen without fixture-specific tuning | 06-B1..B3, 06-R1..R3 | Rejected: std 7,000–29,000 mm; cost 46–248 with the lowest cost at 262 mm error | [report §5.2](sessions/2026-10-22-autocal-flags-06-planning-stopping/report.md#52-threshold-rules) |
| H7. Sparse recovery improves underconstrained recovery | 06-R1..R3 | Mixed: repaired HP3 iteration 1 from no candidate to 229 mm; five-point iteration 1 worsened 6.75 → 13.48 mm; HP3 final worsened 202 → 236 mm | [report §6](sessions/2026-10-22-autocal-flags-06-planning-stopping/report.md#6-sparse-recovery) |
| H8. The spread-consensus stop indicates convergence | 06-B1, 06-B2 (stops at iteration 5 with 1 and 2 sweeps unused) | Not supported: two agreeing fits (spread 0.98 mm and 4.26 mm) gave applied errors of 11.2 and 7.0 mm | [report §5.3](sessions/2026-10-22-autocal-flags-06-planning-stopping/report.md#53-consensus-stop) |
| H9. `--top-k` changes which sweep is collected | code trace only | Rejected by code trace (`ranked[0]` alone sets the collection hint) | [report §4](sessions/2026-10-22-autocal-flags-06-planning-stopping/report.md#4-flag-by-flag-assessment) |

#### References

- [Full report](sessions/2026-10-22-autocal-flags-06-planning-stopping/report.md): analysis, measurements, dispositions and limitations.
- [Research notebook](sessions/2026-10-22-autocal-flags-06-planning-stopping/research.md): decisions, budget and steering.
- [Trial table](sessions/2026-10-22-autocal-flags-06-planning-stopping/trials.json) and [CSV](sessions/2026-10-22-autocal-flags-06-planning-stopping/trials.csv): one row per trial with overrides and outcomes.
- [Commands and overrides](sessions/2026-10-22-autocal-flags-06-planning-stopping/commands.md) and [provenance](sessions/2026-10-22-autocal-flags-06-planning-stopping/provenance.json): revision `4f175079`, fixture SHA-256, package versions.
- [Handoff](sessions/2026-10-22-autocal-flags-06-planning-stopping/../../../../research/autocal-flag-audit/handoffs/06-planning-stopping.md) and [audit](sessions/2026-10-22-autocal-flags-06-planning-stopping/../../../../research/autocal-flag-audit/report.md) that defined the screen.
- [Reference provenance](sessions/2026-10-22-autocal-flags-06-planning-stopping/../../../../autocal/data/references/README.md) and [regression runner](sessions/2026-10-22-autocal-flags-06-planning-stopping/../../../../autocal/tools/regress_calibration_logs.py): ground truth (`DatasetSpec.true_anchors`, `true_radii`) and `error_to_true`.
- [Recording performance note](sessions/2026-10-22-autocal-flags-06-planning-stopping/../../../../research/autocal-recording-performance.md): recording costs; not used for timing here (recording none).

## [2026-10-22-autocal-lane04-cleanup](sessions/2026-10-22-autocal-lane04-cleanup/)

[abstract.md](sessions/2026-10-22-autocal-lane04-cleanup/abstract.md) · [report.md](sessions/2026-10-22-autocal-lane04-cleanup/report.md) · [research.md](sessions/2026-10-22-autocal-lane04-cleanup/research.md)

### Autocal lane 04 integration and history selection replay

Status: complete.

#### Abstract

This integration removes an unused histogram CLI option and extracts final history
selection into a function shared by full-auto and offline replay. The standalone
histogram utility remains supported. Three saved simulated histories reproduce
the previous inline selection exactly. All 14 saved candidate plans contain 83
noise-metric fields, correcting the earlier report's claim that those metrics
were missing. Offline rank artifacts now distinguish the best-ranked single fit
from applied consensus anchors. No numerical defaults changed and no fitting,
collection or hardware experiment was run. Focused validation passed 88 tests;
the results establish refactor equivalence, not improved calibration accuracy.

#### Topics

autocalibration, ranking, consensus, CLI cleanup, frozen replay, noise metrics

#### Tested hypotheses

| Hypothesis | Experiment | Outcome | Evidence |
| --- | --- | --- | --- |
| Extraction preserves selection on the three saved histories | I04-1 | Supported | [Checks](sessions/2026-10-22-autocal-lane04-cleanup/checks.json) |
| Saved history candidate plans lack noise metrics | I04-2 | Rejected for inspected artifacts | [Counts](sessions/2026-10-22-autocal-lane04-cleanup/checks.json) |
| Validated-only consensus or penalty 2 improves general calibration | None | Not tested | [Integration report](sessions/2026-10-22-autocal-lane04-cleanup/report.md) |

#### References

- [Report](sessions/2026-10-22-autocal-lane04-cleanup/report.md): decisions and source evidence.
- [Notebook](sessions/2026-10-22-autocal-lane04-cleanup/research.md): scope and budgets.
- [Checks](sessions/2026-10-22-autocal-lane04-cleanup/checks.json), [script](sessions/2026-10-22-autocal-lane04-cleanup/check_histories.py): artifact hashes and reproduction.

## [2026-10-22-autocal-lane06-integration](sessions/2026-10-22-autocal-lane06-integration/)

[abstract.md](sessions/2026-10-22-autocal-lane06-integration/abstract.md) · [report.md](sessions/2026-10-22-autocal-lane06-integration/report.md) · [research.md](sessions/2026-10-22-autocal-lane06-integration/research.md)

### Lane 06 integration: planning CLI cleanup and stop attribution

Status: complete.

#### Abstract

This investigation asks which lane 06 findings justify changes to autocal.
Six current-source, no-collection replays compare default and disabled spread
stopping with an eight-iteration ceiling. Two fixtures retain identical applied
parameters; the outlier fixture's combined truth error improves from 7.009 to
6.705 mm while its anchor error worsens. This mixed simulation evidence does not
justify a default stopping-policy change. Replaying the saved HP3 history explains
the alleged final-anchor discrepancy: held-out ranking selects iteration 4,
while the running training best is iteration 2. The implementation removes seven
CLI controls, fixes one-sided ranges and repeat-exclusion fallback, and makes
final selection explicit. The focused suite passes 140 tests; the broader suite
passes 533 with seven slow tests deselected. All four selected calibration
regression fixtures pass with unchanged parameters, iteration comparisons and
replay sweep/point totals. No hardware, fresh collection or physics changes are
involved.

#### Topics

autocalibration, planning, CLI simplification, stopping, spread consensus,
range resolution, duplicate sweeps, held-out selection, regression

#### Tested hypotheses

| Hypothesis | IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Disabling spread stopping improves physical calibration generally | I02, six paired replay controls | Mixed, no default change | [Paired results](sessions/2026-10-22-autocal-lane06-integration/paired-results.json) |
| HP3 final output disagrees with the actual selected model | I01 | Rejected on saved history | [Report](sessions/2026-10-22-autocal-lane06-integration/report.md) |
| Cleanup preserves tested contracts and fixes range/exclusion bugs | T01/T02 | Supported tool checks, not accuracy evidence | [Tests](sessions/2026-10-22-autocal-lane06-integration/tests.log) |

#### References

- [Report](sessions/2026-10-22-autocal-lane06-integration/report.md), [notebook](sessions/2026-10-22-autocal-lane06-integration/research.md), [provenance](sessions/2026-10-22-autocal-lane06-integration/provenance.json).
- [Preceding lane](sessions/2026-10-22-autocal-lane06-integration/../2026-10-22-autocal-flags-06-planning-stopping/report.md).
- [Replay harness](sessions/2026-10-22-autocal-lane06-integration/replay.py), [paired results](sessions/2026-10-22-autocal-lane06-integration/paired-results.json).

## [2026-10-22-autocal-physical-model-cleanup](sessions/2026-10-22-autocal-physical-model-cleanup/)

[abstract.md](sessions/2026-10-22-autocal-physical-model-cleanup/abstract.md) · [report.md](sessions/2026-10-22-autocal-physical-model-cleanup/report.md) · [research.md](sessions/2026-10-22-autocal-physical-model-cleanup/research.md)

### Autocal ellipse flex removal and zero-tension validation

Status: complete, 2026-10-22.

#### Abstract

Lane 05 found that ellipse flex correction never applied on the reference corpus
because the collector writes no per-point tensions. This integration removes its
two CLI switches and ellipse fitting/validation plumbing, preserving independent
legacy point and plotting consumers. Failed zero-tension extrapolation now stops
collection, and explicit projected fits require projected angle fields. Fit-free
before/after comparisons match exactly in 72 checks across twelve fixtures. Two
full recorded-data replays preserve reference results and collection counts.
Python and collector checks pass. A small saved-geometry probe finds no valid
sweeps and a zero numerical gradient in two cost-100 production failures, even
with sweep rejection disabled. That narrows the next solver investigation without
changing solver or filtering policy. Calibration evidence is recorded simulation;
no new movement, hardware measurement or accuracy/speed gain was established.

#### Topics

autocalibration, physical models, flex removal, zero-tension projection, CLI cleanup,
reference regression, invalid geometry, underconstrained sentinel, solver stagnation

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Removing inert ellipse flex preserves current objective and replays | C05-01, C05-02 | Supported on tested data | [Report](sessions/2026-10-22-autocal-physical-model-cleanup/../../../../research/autocal-physical-model-cleanup/report.md) |
| Invalid projection can be rejected before recording/fitting | C05-03, C05-04 | Supported by code checks | [Report](sessions/2026-10-22-autocal-physical-model-cleanup/../../../../research/autocal-physical-model-cleanup/report.md) |
| Turning off sweep rejection makes saved sentinel geometries valid | C05-05 | Rejected for two checked production failures | [Report](sessions/2026-10-22-autocal-physical-model-cleanup/../../../../research/autocal-physical-model-cleanup/report.md) |
| Different solver starts cure the original failure trajectory | None | Not tested | [Next investigation](sessions/2026-10-22-autocal-physical-model-cleanup/../../../../research/autocal-physical-model-cleanup/report.md#solver-follow-up) |

#### References

- [Report](sessions/2026-10-22-autocal-physical-model-cleanup/../../../../research/autocal-physical-model-cleanup/report.md): changes, metrics, source links and raw evidence.
- [Notebook](sessions/2026-10-22-autocal-physical-model-cleanup/../../../../research/autocal-physical-model-cleanup/research.md): scope, decisions and failure preservation.
- [Verification script](sessions/2026-10-22-autocal-physical-model-cleanup/../../../../research/autocal-physical-model-cleanup/verify.py): fit-free before/after and saved-geometry checks.

## [2026-10-22-autocal-solver-integration](sessions/2026-10-22-autocal-solver-integration/)

[abstract.md](sessions/2026-10-22-autocal-solver-integration/abstract.md) · [report.md](sessions/2026-10-22-autocal-solver-integration/report.md) · [research.md](sessions/2026-10-22-autocal-solver-integration/research.md)

### HP3 nominal truth verification and lane 01 solver integration

Status: complete.

#### Abstract

The lane 01 report inferred a seventh-fixture scene mismatch from poor fits and
agreement with an older fitted reference. Static reconstruction from the scene at
first data commit `0a5a0f6d` and today's HP3 scene instead reproduces the existing
nominal effective-anchor truth within 0.002 mm rounding. Rescored saved fits have
63.84–65.05 mm anchor RMSE against that truth; the lane's smaller distances describe
reference-fit agreement. Five bounded frozen-fit replays preserve the lane's
numerical results while new traces establish actual solver methods and termination.
A separate scalar control exercises the production TRF retry; function-level tests
expose and fix an enabled-fallback bug that skipped finite but invalid converged
primary fits. Three full replay regressions pass without parameter or collection
changes. No physical-accuracy improvement, hardware result or new default is
claimed; original unrecorded browser settings remain a provenance limitation.


#### Topics

autocalibration, HP3, effective anchors, simulation truth, solver diagnostics,
TRF fallback, restarts, iteration budgets, calibration bias

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Nominal scene matches existing truth | T1 | Supported | [Derivation](sessions/2026-10-22-autocal-solver-integration/truth-verification.json) |
| Diagnostic propagation preserves fitted numerics | D1 | Supported on measured replays | [Controls](sessions/2026-10-22-autocal-solver-integration/replay-checks.json), [variants](sessions/2026-10-22-autocal-solver-integration/followup-checks.json) |
| Reduced iteration budget is non-binding | I1 | Rejected on measured seventh trajectory | [Variant trace](sessions/2026-10-22-autocal-solver-integration/followup-checks.json) |
| TRF retry can activate without being selected | F1 | Supported tool control | [Scalar result](sessions/2026-10-22-autocal-solver-integration/forced-fallback.json) |
| Retry correctly handles converged invalid primary fits | F2 | Fixed and verified by targeted test | [Tests](sessions/2026-10-22-autocal-solver-integration/pytest.log) |

#### References

- [Report](sessions/2026-10-22-autocal-solver-integration/report.md): findings, code changes, source provenance and limits.
- [Notebook](sessions/2026-10-22-autocal-solver-integration/research.md): budget, intermediate hypothesis and outcome.
- [Tracked integration report](sessions/2026-10-22-autocal-solver-integration/../../../../research/autocal-solver-lane01-integration.md).

## [2026-10-22-autocal-spool-simplification](sessions/2026-10-22-autocal-spool-simplification/)

[abstract.md](sessions/2026-10-22-autocal-spool-simplification/abstract.md) · [report.md](sessions/2026-10-22-autocal-spool-simplification/report.md) · [research.md](sessions/2026-10-22-autocal-spool-simplification/research.md)

### Integrating spool scale policy and removing two autocal controls

Status: complete.

#### Abstract

Lane 03 identified per-pass scale polish and the default inner spool cap as
simplification candidates. Twelve sequential fits on three additional simulated
fixture prefixes compared default/off/inner60 before simplification and defaults
afterward, with the final three sweeps held out. The cap of 60 exactly reproduced
30, with all-pass iteration maxima 19, 16 and 11. Removing polish worsened physical
error from 108.4 to 776.1 mm on flexible lines and from 8.24 to 301.3 mm on the
ten-point fixture; the pressure fixture was unchanged. Fixing the previous
default policy exactly preserved anchors, radii, costs, held-out statistics and
iteration counts. Two CLI/API controls and their obsolete paths were removed.
The flexible prefix still has a pre-existing cost sentinel; no general accuracy,
speed or hardware claim follows. All twelve recorded regression fixtures exactly preserve the compared summaries,
iteration parameters and collection counts.

#### Topics

autocalibration, CLI simplification, scale polish, spool refinement, coordinate
descent, validation, held-out sweeps, default equivalence

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Cap 60 improves the tested validation prefixes over 30 | V01–V09 | Rejected: identical outputs and counts | [Before trials](sessions/2026-10-22-autocal-spool-simplification/before.log) |
| Removing polish preserves the two additional scale-sensitive fits | V01–V09 | Rejected: physical/held-out errors worsen | [Before trials](sessions/2026-10-22-autocal-spool-simplification/before.log) |
| Fixed policy preserves the default validation outputs | V10–V12 | Supported exactly on the three prefixes | [After trials](sessions/2026-10-22-autocal-spool-simplification/after.log), [trials](sessions/2026-10-22-autocal-spool-simplification/trials.json) |

#### References

- [Notebook](sessions/2026-10-22-autocal-spool-simplification/research.md): objective, budgets and constraints.
- [Report](sessions/2026-10-22-autocal-spool-simplification/report.md): integration scope and evidence.
- [Validation harness](sessions/2026-10-22-autocal-spool-simplification/harness/validate.py): frozen input selection and measurements.
- [Tracked integration report](sessions/2026-10-22-autocal-spool-simplification/../../../../research/autocal-spool-lane03-integration.md).

## [2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/)

[abstract.md](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/abstract.md) · [report.md](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/report.md) · [research.md](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/research.md)

### Faster failure handling for encoder-only CDPR autocalibration

Status: complete.

#### Abstract

Can encoder-only CDPR autocalibration become faster without degrading geometry or changing one-click stopping? We audited the supplied zero-buildup HP3 reference, tested recorded-data solver and thinning changes, twelve varied kinematic worlds, and three production native HP3 runs. The reference's lowest training score has 8.00 mm anchor RMSE versus 2.53 mm for its last fit. Several proposed estimator changes fail controls. The retained machine-independent change bypasses L-BFGS-B compilation when fixed point counts make every geometry underconstrained. Two matched, reversed-order pairs of the complete twelve-fixture regression reduce mean wall time from 276.01 to 268.25 seconds (2.81%) and child CPU from 955.72 to 914.55 seconds (4.31%). All 48 fixture executions pass, with identical numerical diagnostics and stopping decisions. The shortcut occurs in 48 of 4014 scalar solves per suite, across eight fixtures including Slideprinter and Hangprinter recordings. A paired native HP3 collection preserves all 160 points, 4503.840 simulated seconds, 3.204 mm anchor RMSE and automatic stopping exactly. This is a conditional compute saving in the shared ellipse L-BFGS-B path; TRF/LM are unchanged. No collection-time or accuracy gain is established. Evidence is recorded, simulated or synthetic, not hardware.

#### Topics

HP3, HP4, Slideprinter, CDPR, encoder-only autocalibration, regression benchmark, CPU time, constant objective, JAX compilation, underconstrained fits, candidate ranking, fewer points, synthetic worlds, automatic stopping

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Lower fit score identifies better anchors | E01 | Rejected on reference | [Audit](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E01-reference-audit.json) |
| Within-sweep fixed encoder drift explains this reference | E01 | Rejected: range exactly zero; fixed deltas match observations | [Audit](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E01-reference-audit.json) |
| Common future sweeps repair the individual-candidate ranking | E24 | Rejected on this pool: still selects iteration 4 over more accurate 6 | [Common validation](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E24-common-validation.json) |
| Endpoint-preserving 40% thinning preserves robust fits | E02 | Rejected: two of three sparse prefixes return sentinels | [Baseline fits](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E02-baseline/results.json) |
| Centered squared-length arithmetic improves robustness | E03 | Rejected: dense nine-sweep fit fails | [Paired fits](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E03-centered/results.json) |
| TRF resolves sparse fitting failures | E04 | Mixed: rescues 108 points, still fails at 36 points | [TRF fits](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E04-trf/results.json) |
| Resetting an invalid recovery seed produces useful sparse geometry | E06–E07 | Mixed: avoids boundary trap, but about 41 mm error remains | [L-BFGS-B](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E06-recovery/results.json), [TRF](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E07-recovery-trf/results.json) |
| Favoring constrained restarts improves the applied calibration safely | E08, E12–E14 | Rejected as a general default: seventh fixture worsens | [Regression evidence](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E13-regressions.log), [live comparison](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E22-live-comparison.json) |
| Lowering the minimum points per sweep to three fixes sparse calibration | E11 | Rejected: one sentinel removed with worse geometry; another remains | [Fits](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E11-min3/results.json) |
| Structurally constant L-BFGS-B objectives can skip compilation exactly | E16–E23 | Supported within tested scope | [Timing](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E19-benchmark/summary.json), [regressions](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E20-final-regressions.log), [live equality](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E22-live-comparison.json), [synthetic controls](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E23-final-synthetic/results.json) |
| The shared guard measurably reduces complete regression compute without changing results | E26 | Supported: 2.81% wall and 4.31% child CPU reduction in two pairs | [Timing report](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/regression-timing.md), [audited measurements](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E26-regression-timing/summary.json) |
| Reusing quiet samples, force-isolation experiments, cost-aware movement or task-space uncertainty improves acquisition | none | Not tested | [Notebook](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/research.md) |

#### References

- [Full report](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/report.md): interpretation, reproduction and limitations.
- [E26 complete regression timing](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/regression-timing.md), [manifest](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E26-regression-timing/manifest.json), [audited summary](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/E26-regression-timing/summary.json): four full suites at revision `79e53cbb562a9e139c6a23b73e91b8c69bbaca85` quantify total cost and machine scope.
- [Notebook](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/research.md): hypotheses, budgets, failures and decisions.
- [Supplied HP3 reference](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/../../../extended-autocal/no-buildup-reference-003/sweeps.json) and [saved candidates](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/../../../extended-autocal/no-buildup-reference-003/sweeps.stages/history-007.json): ranking and thinning inputs.
- [Reference event extraction from the October 12 session](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/../2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/E01-events.jsonl): original recording events used for the timing audit; the original RRD is absent here.
- [Final source patch](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/final-source.diff), [provenance](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/provenance.json): implementation based on revision `9e40f995da14102612f4a7f57c3fc0a0665bbdf5`.
- [Prior solver/force study](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9d784051/report.md), [prior thinning/roundoff study](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/../2026-10-12-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-2f03cb59/report.md), [prior warm-start study](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-267d719f/report.md): existing gains and negative results that informed the pilots.
- Caverly, R. J.; Cheah, S. K.; Bunker, K. R.; Patel, S.; Sexton, N.; Nguyen, V. L. (2025), “Online Self-Calibration of Cable-Driven Parallel Robots Using Covariance-Based Data Quality Assessment Metrics,” DOI [10.1115/1.4065236](https://experts.umn.edu/en/publications/online-self-calibration-of-cable-driven-parallel-robots-using-cov/): joint pose/bias estimation and data-quality stopping motivate future experiments; these methods were not implemented here.
- Wang, H.; Gao, T.; Kinugawa, J.; Kosuge, K. (2017), “Finding Measurement Configurations for Accurate Robot Calibration: Validation with a Cable-Driven Robot,” DOI [10.1109/TRO.2017.2707562](https://scholars.cityu.edu.hk/en/publications/finding-measurement-configurations-for-accurate-robot-calibration/): observability-based configuration selection motivates targeted acquisition.
- Alexandre dit Sandretto, J.; Daney, D.; Gouttefarde, M. (2012), “Calibration of a Fully-Constrained Parallel Cable-Driven Robot,” [author PDF](https://www-sop.inria.fr/coprin/PDF/a_romansy2012.pdf): redundant internal length constraints and joint nuisance-pose estimation motivate encoder-only alternatives. Its eight-cable, six-DOF embodiment differs from HP3.
- [Algorithm overview](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/../../../../autocal/optimization_algorithm_overview), [objective overview](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/../../../../autocal/objective_functions_overview), [stage API](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/../../../../autocal/README.md), [recording workload costs](sessions/2026-10-22-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-9b850a37/../../../../research/autocal-recording-performance.md): current workflow and budget contracts actually consulted.

## [2026-10-22-interactive-research-49c8bfde](sessions/2026-10-22-interactive-research-49c8bfde/)

[abstract.md](sessions/2026-10-22-interactive-research-49c8bfde/abstract.md) · [report.md](sessions/2026-10-22-interactive-research-49c8bfde/report.md) · [research.md](sessions/2026-10-22-interactive-research-49c8bfde/research.md)

### Can cable-flex compensation improve HP3 autocalibration?

Status: complete, 2026-10-22.

#### Abstract

I tested why the removed autocal `--flex` flag was inactive and whether a repaired flex correction can improve Hangprinter 3 calibration. Saved HP3 fixtures have no drive or fixed-cable tension fields, so flex-on and flex-off fits are necessarily identical. Native HP3 fixed-pose tests show repeatable material extension up to 0.382 mm from 0.256 to 3.817 N, but the deterministic solver uses constant joint stiffness, so it cannot establish material length dependence, hysteresis or creep. In a controlled 12-fit synthetic HP3 matrix, the exact old correction recovered anchors to numerical precision only when fixed cables were rigid and origin extension was zero. With all cables compliant and nonzero origin extension, the old path was worse than no correction (6.632 versus 6.452 mm anchor RMSE), while an all-role delta correction reached zero. With 0.1 mm noise, all-role correction reduced RMSE from 10.825 to 6.125 mm and held-out error from 0.200 to 0.097 mm. Halving stiffness removed the benefit. This supports the physical idea conditionally, not the removed implementation or a hardware gain.

#### Topics

autocalibration, Hangprinter 3, cable elasticity, flex compensation, encoder deltas, measurement origin, tension, synthetic identification, held-out sweeps, native Rust simulation

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| The removed flag is inactive on saved data because tension inputs are absent | Lane 05 activation probe; 2026-10-22 existing-data check | Supported | All fixtures lack drive/fixed tension; on/off objectives are identical |
| Correct flex compensation is identifiable and improves anchor recovery under a matched model | Synthetic fit matrix in this session | Mixed | Correct all-role model reaches zero error; exact old path succeeds only in the favorable zero-origin rigid-fixed case; wrong stiffness fails |
| Native HP3 has repeatable load-dependent extension | Native IDs `6b50bde7...`, `40508c31...` | Supported in simulation | Extension 0.02543-0.38159 mm over four loads; revisits below `2e-15 m`; no physical hysteresis test |
| Elastic extension is too small to affect calibration in the measured range | None | Inconclusive | 0.526 mm synthetic strain produced 10.8-13.1 mm anchor error; native law and real-hardware scale are unresolved |

#### References

- [Full report](sessions/2026-10-22-interactive-research-49c8bfde/report.md): methods, results, limitations and decision.
- [Research notebook](sessions/2026-10-22-interactive-research-49c8bfde/research.md): hypotheses, budgets, experiment IDs and steering.
- [Synthetic fit matrix](sessions/2026-10-22-interactive-research-49c8bfde/synthetic-flex-results.json): 12 analytic fits, held-out residuals and generated cases.
- [Synthetic harness](sessions/2026-10-22-interactive-research-49c8bfde/harness/synthetic_flex.py): truth model and exact correction variants.
- [Native HP3 load summary](sessions/2026-10-22-interactive-research-49c8bfde/high-load-summary.json): fixed-pose force/extension measurements.
- [Longer-line native control](sessions/2026-10-22-interactive-research-49c8bfde/high-load-longer-summary.json): length-invariance limitation.
- [Failed optimizer pilot](sessions/2026-10-22-interactive-research-49c8bfde/synthetic-flex-optimizer-pilot.json): preserved non-truth local-minimum result.
- [Invalid zero-buildup fixture fit](sessions/2026-10-22-interactive-research-49c8bfde/existing-fifth-uncorrected.json): existing-data configuration mismatch.
- Git revision `06bc6c41`: removal of `--flex` and `--spring-k-multiplier`.

## [2026-10-22-interactive-research-7345527c](sessions/2026-10-22-interactive-research-7345527c/)

[abstract.md](sessions/2026-10-22-interactive-research-7345527c/abstract.md) · [research.md](sessions/2026-10-22-interactive-research-7345527c/research.md)

### interactive research

Status: not yet summarized.

#### Abstract

The session has started. No research result has been summarized yet.

#### Topics

interactive research

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| None recorded yet | None | Not tested | None |

#### References

- [Original task](sessions/2026-10-22-interactive-research-7345527c/prompt.txt)
- [Research notebook](sessions/2026-10-22-interactive-research-7345527c/research.md)

## [2026-10-22-interactive-research-a430536f](sessions/2026-10-22-interactive-research-a430536f/)

[abstract.md](sessions/2026-10-22-interactive-research-a430536f/abstract.md) · [report.md](sessions/2026-10-22-interactive-research-a430536f/report.md) · [research.md](sessions/2026-10-22-interactive-research-a430536f/research.md)

### HP3 encoder autocal: where simulated collection time goes, and feed rate as a speed lever

Status: complete for the bounded scope; no production change accepted.

#### Abstract

Can encoder-only autocal collect data faster without losing robustness? We profiled the user's HP3 no-buildup reference run (9 sweeps, 180 points, 6,756 s simulated) from its saved per-point timestamps and ran three native headless single-sweep collections (20 points each) at feed 700, 1400 and 2800 mm/min. Sampling is only about 6% of simulated time; the rest is motion plus settling, roughly proportional to travel. Feed 1400, the current default, gave the shortest collection of the three (51.6 s vs 62.9 s and 62.2 s from sweeps.json). Faster feeds did not shorten collection because settle time grew with move speed. No anchor accuracy was scored: the reference has no authored point-anchor ground truth, and each experiment is a single sweep.

#### Topics

autocalibration, HP3, encoder-only calibration, collection time, settling, feed rate, native Rust headless collection, reference run profiling, ground truth limits

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Per-point collection cost is dominated by sampling | Reference profile | Rejected: sampling is 410 s of 6,756 s (about 6%) | [reference sweeps](sessions/2026-10-22-interactive-research-a430536f/../../../extended-autocal/no-buildup-reference-003/sweeps.json), [notebook](sessions/2026-10-22-interactive-research-a430536f/research.md) |
| Per-point cost has a large fixed settle constant | Reference profile, E01–E03 | Rejected for the reference (intercept ≈ 0); native intercepts 0.15–0.73 s | [research notes](sessions/2026-10-22-interactive-research-a430536f/research.md) |
| A higher drive feed shortens collection | E01, E02, E03 | Rejected: 2800 mm/min took 62.2 s vs 51.6 s at 1400; 700 mm/min took 62.9 s | [E01](sessions/2026-10-22-interactive-research-a430536f/native/ab74bf4b36fc4bf2a00375f3a2234055/5452996a02a3490ea37ead50a55f7eb1/sweeps.json), [E02](sessions/2026-10-22-interactive-research-a430536f/native/ab74bf4b36fc4bf2a00375f3a2234055/c45535d5062240c28aa37fad1b59df1e/sweeps.json), [E03](sessions/2026-10-22-interactive-research-a430536f/native/ab74bf4b36fc4bf2a00375f3a2234055/42f64ae035934f56968ac5ff84657b95/sweeps.json) |
| Settle time per mm of travel grows with feed | E01–E03 gap fits | Supported within this native sweep: 0.235 s/mm (1400), 0.296 (700), 0.393 (2800) | [research notes](sessions/2026-10-22-interactive-research-a430536f/research.md) |
| Anchor-selection disagreement with geometry can be fixed by a cheaper selector | Not tested | Not tested | None |
| Reference fit anchors are within a known error of truth | Not tested | Not tested: no authored point-anchor truth available | [native-collection guide](sessions/2026-10-22-interactive-research-a430536f/../../../../research/native-collection.md) |

#### References

- [Full report](sessions/2026-10-22-interactive-research-a430536f/report.md): analysis, measurements and limitations.
- [Research notebook](sessions/2026-10-22-interactive-research-a430536f/research.md): experiment decisions and accepted steering.
- [Reference run](sessions/2026-10-22-interactive-research-a430536f/../../../extended-autocal/no-buildup-reference-003/sweeps.json): HP3 no-buildup run used for the profile.
- [Native collection guide](sessions/2026-10-22-interactive-research-a430536f/../../../../research/native-collection.md): authored-scene ground truth limits.
- [Recording-mode cost](sessions/2026-10-22-interactive-research-a430536f/../2026-10-15-autocal-recording-modes/abstract.md): earlier recording-cost measurements, relevant to choosing `none` for sweeps.
- [HP4 timing and transit budget](sessions/2026-10-22-interactive-research-a430536f/../2026-10-19-interactive-research-8d7a82a2/abstract.md): earlier HP4 collection-duration work; not re-tested here.

## [2026-10-22-interactive-research-d1d37c7f](sessions/2026-10-22-interactive-research-d1d37c7f/)

[abstract.md](sessions/2026-10-22-interactive-research-d1d37c7f/abstract.md) · [research.md](sessions/2026-10-22-interactive-research-d1d37c7f/research.md)

### interactive research

Status: not yet summarized.

#### Abstract

The session has started. No research result has been summarized yet.

#### Topics

interactive research

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| None recorded yet | None | Not tested | None |

#### References

- [Original task](sessions/2026-10-22-interactive-research-d1d37c7f/prompt.txt)
- [Research notebook](sessions/2026-10-22-interactive-research-d1d37c7f/research.md)

## [2026-10-23-consensus-membership](sessions/2026-10-23-consensus-membership/)

[abstract.md](sessions/2026-10-23-consensus-membership/abstract.md) · [report.md](sessions/2026-10-23-consensus-membership/report.md) · [research.md](sessions/2026-10-23-consensus-membership/research.md)

### Finite-prediction consensus: broader replay rejects a blanket restriction

Status: complete.

#### Abstract

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

#### Topics

autocalibration, encoder-only calibration, consensus membership, newest fit,
held-out sweeps, applied spool model, Slideprinter, HP3, negative results

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Finite-prediction-only final consensus improves accuracy without worsening an additional dataset | CM-E1 | Rejected: one improvement, two regressions, five unchanged | [Measurements](sessions/2026-10-23-consensus-membership/results.json), [comparison](sessions/2026-10-23-consensus-membership/report.md#cm-e1-broader-accuracy-comparison) |
| Finite member predictions establish predictive safety of the applied average and chosen spool model | CM-E1 | Inconclusive: finite diagnostics, no common unseen fold for changed selections | [Fold coverage](sessions/2026-10-23-consensus-membership/results.json), [guardrails](sessions/2026-10-23-consensus-membership/report.md#prediction-guardrails-and-source-compatibility) |

#### Decision handoff

- Scope: eleven archived simulated datasets at producer revision 72523b9d; final selection with current 0.01 mm merge/5 mm spread guards and frozen ranking. Eight dataset names extend the motivating screen; three overlap.
- Review: this agent recomputed selection and applied-model predictions once, checked saved-score compatibility and audited hashes/dataset uniqueness. No independent reviewer, fresh fitting or hardware validation.
- Decision: H1 rejects the blanket default change. Preserve current all-candidate consensus; no fixture-specific or tuned replacement is justified.
- Next: task 006 remains next in queue order, followed by task 005's agenda. Reopening membership needs common unseen-sweep evidence on the regression cases; search compatible saved later sweeps before new fits or collection.

#### References

- [Full report](sessions/2026-10-23-consensus-membership/report.md): policy comparison, exceptions and scope.
- [Research notebook](sessions/2026-10-23-consensus-membership/research.md): predeclared budget and decisions.
- [Input manifest](sessions/2026-10-23-consensus-membership/inputs.json), [compressed original envelopes](sessions/2026-10-23-consensus-membership/inputs/) and [audit](sessions/2026-10-23-consensus-membership/audit.json): source identities, truth used only for evaluation and distinct observation datasets.
- [Initial screen](sessions/2026-10-23-consensus-membership/../2026-10-23-program-task-sanity/report.md): motivating result reused without rerunning.
- [October 15 spread-gate study](sessions/2026-10-23-consensus-membership/../2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-35233092/report.md): origin and prior scope of the eleven-history corpus.

## [2026-10-23-durable-handoff](sessions/2026-10-23-durable-handoff/)

[abstract.md](sessions/2026-10-23-durable-handoff/abstract.md) · [report.md](sessions/2026-10-23-durable-handoff/report.md) · [research.md](sessions/2026-10-23-durable-handoff/research.md)

### Research handoffs retain scoped evidence after worktree turnover

Status: complete.

#### Abstract

Can another researcher inspect a useful result after its original worktree or
supervisor is gone? Task 006 added a standard-library archive CLI that copies
explicitly selected evidence, records hashes and relative references, and reads
saved autocal job outcomes offline. DH-E1 packaged task 004's saved consensus
result: 19 files, including eleven compressed inputs, totaling 4,509,141 bytes.
After relocating the bundle and deleting every temporary source path, all
selected hashes and its scoped decision remained readable. Focused tests also
preserved partial records for absent/damaged evidence and separated terminal job
observations from historical running/cancelling states. All 21 archive/handoff
tests passed. This demonstrates portability and historical lifecycle inspection,
not independent reproduction of the calibration result, current process liveness
or scientific success. Evidence selection remains explicit; runtime dependencies
and unselected document links are outside the bundle. No physics, fitting,
prediction evaluation or collection was performed.

#### Topics

research archive, evidence portability, durable handoff, autocal job status,
worktree turnover, SHA-256, partial evidence, lifecycle acceptance

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Explicit selected evidence preserves a scoped result without its source paths | DH-E1 | Supported for the copied CM-E1 handoff | [Results](sessions/2026-10-23-durable-handoff/results.json), [recovery](sessions/2026-10-23-durable-handoff/evidence/recovered.json) |
| Saved lifecycle records can be read without treating CLI acceptance as science or stale running as liveness | DH-T1 | Supported for synthetic lifecycle and corruption fixtures | [Checks](sessions/2026-10-23-durable-handoff/verification.json), [tests](sessions/2026-10-23-durable-handoff/../../../../tests/python/test_research_handoff.py) |

#### Decision handoff

- Scope: offline archive export/inspection and saved full-autocal job records; base revision 166f0352 and implementation hashes in verification.json.
- Review: this agent ran the focused tests and one byte-portability demonstration, then inspected the implementation/evidence. No independent scientific reproduction or live-job test.
- Decision: keep the explicit export/show/job route; retain live supervisor ownership semantics and report limitations with partial evidence.
- Next: task 005 can use the living research agenda. Revisit automatic reference discovery only if explicit selections prove insufficient in actual handoffs.

#### References

- [Full report](sessions/2026-10-23-durable-handoff/report.md), [notebook](sessions/2026-10-23-durable-handoff/research.md): scope, implementation and limitations.
- [Demonstration](sessions/2026-10-23-durable-handoff/demonstrate.py), [export manifest](sessions/2026-10-23-durable-handoff/evidence/export-manifest.json): copied-file/source identities and original-to-relative mapping.
- [Producer CM-E1 report](sessions/2026-10-23-durable-handoff/../2026-10-23-consensus-membership/report.md): scientific decision reused, not numerically rerun.
- [Operator procedure](sessions/2026-10-23-durable-handoff/../../../../research/handoffs.md): export selection and offline job interpretation.

## [2026-10-23-program-task-sanity](sessions/2026-10-23-program-task-sanity/)

[abstract.md](sessions/2026-10-23-program-task-sanity/abstract.md) · [report.md](sessions/2026-10-23-program-task-sanity/report.md) · [research.md](sessions/2026-10-23-program-task-sanity/research.md)

### Research-program sanity check: a useful consensus lead and larger task outcomes

Status: complete.

#### Abstract

Reviewed the three committed workflow changes and independently recomputed the
finite-held-out-only consensus comparison proposed by task 003. On two recorded
Slideprinter histories and one HP3 history, all input hashes and baseline errors
matched saved producer evidence. Restricting consensus membership improved summed
per-anchor Euclidean error from 10.893581 to 6.988852 mm on five-points and from
4.670121 to 2.552240 mm on layered. HP3 remained at 197.407724 mm because both
policies rejected their candidate pairs and retained the same single fit. Six
selection evaluations used no fitting, prediction reevaluation, physics or
collection. This supports a larger saved-history investigation, not a default
change or hardware claim. Archive tests passed. Remaining tasks now cover a real
research decision, a durable evidence handoff and a living research agenda.
These are workflow decisions from inspection, not a controlled productivity study.

#### Topics

research workflow, consensus, held-out validation, applied anchors, evidence reuse,
program agenda, artifact portability, task granularity, negative results.

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Finite-prediction-only consensus is nonworse on the three specified histories | H1, six selection evaluations | Supported on this screen | [Exact metrics and inputs](sessions/2026-10-23-program-task-sanity/results.json) |
| New prompt structure causally improves research productivity | None | Not tested; task revisions are practical judgments | [Inspection and limits](sessions/2026-10-23-program-task-sanity/report.md) |

#### Decision handoff

- Scope: three frozen simulated histories from 4f175079; selection functions at ea2b7d7a; no changed trajectories or hardware.
- Review: current agent recomputed selection/anchor errors and matched saved baselines; no independent physics reproduction or consensus prediction check.
- Decision: retain production defaults; make task 004 a broader research-and-integration decision. Improve evidence handoff before building generic graph tooling.
- Next: test independent histories and the applied anchors with their actual spool model; this can expose membership regressions not seen in the initial screen.

#### References

- [Report](sessions/2026-10-23-program-task-sanity/report.md), [notebook](sessions/2026-10-23-program-task-sanity/research.md), [reproduction script](sessions/2026-10-23-program-task-sanity/probe.py), [results](sessions/2026-10-23-program-task-sanity/results.json).
- [Task-003 proposal](sessions/2026-10-23-program-task-sanity/../../../../research/program-tasks/003-one-next-question.md): predeclared comparison and success rule.
- [Producer checks](sessions/2026-10-23-program-task-sanity/../2026-10-22-autocal-lane04-cleanup/checks.json): hashes and recorded equivalence.
- [Copied inputs](sessions/2026-10-23-program-task-sanity/inputs/): history inputs and producer policy results retained locally.

## [architecture-batch](architecture-batch/)

[abstract.md](architecture-batch/abstract.md) · [report.md](architecture-batch/report.md) · [research.md](architecture-batch/research.md)

### Retained batch research entry route

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

Capabilities and runtime_status returned successfully through the batch launcher. The native Python world remained at step zero with no queued movement. This is a connectivity check; no experiment or robot motion ran.

#### Topics

integration verification, batch, MCP capabilities, persistent runtime

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Batch entry can reach research tools without advancing physics | 60187ea2c8d44cbf84f916674af34a3b (world ID) | Supported as a connectivity check | [Report](architecture-batch/report.md) |

#### References

- [Runtime status and route result](architecture-batch/report.md)
- [Tracked route verification](architecture-batch/../../../research/architecture-verification.md)

## [architecture-interactive](architecture-interactive/)

[abstract.md](architecture-interactive/abstract.md)

### Architecture interactive

Status: local summary incomplete; see tracked verification.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

This directory has no final report and its local notebook does not summarize completed checks. The tracked architecture verification document records the related integration outcomes. Do not infer calibration findings from the starter notebook or the session exit code.

#### Topics

integration verification, partial session summary

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| No local completed hypothesis test summarized | See tracked architecture verification | Not documented locally | [Verification](architecture-interactive/../../../research/architecture-verification.md) |

#### References

- [Original saved artifact](architecture-interactive/prompt.txt)
- [Tracked account of route, resume and Viewer checks; consult this for outcomes](architecture-interactive/../../../research/architecture-verification.md)

## [architecture-interactive-2](architecture-interactive-2/)

[abstract.md](architecture-interactive-2/abstract.md) · [research.md](architecture-interactive-2/research.md)

### Architecture interactive 2

Status: local summary incomplete; see tracked verification.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

This directory has no final report and its local notebook does not summarize completed checks. The tracked architecture verification document records the related integration outcomes. Do not infer calibration findings from the starter notebook or the session exit code.

#### Topics

integration verification, partial session summary

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| No local completed hypothesis test summarized | See tracked architecture verification | Not documented locally | [Verification](architecture-interactive-2/../../../research/architecture-verification.md) |

#### References

- [Original saved artifact](architecture-interactive-2/research.md)
- [Original saved artifact](architecture-interactive-2/prompt.txt)
- [Tracked account of route, resume and Viewer checks; consult this for outcomes](architecture-interactive-2/../../../research/architecture-verification.md)

## [architecture-interactive-3](architecture-interactive-3/)

[abstract.md](architecture-interactive-3/abstract.md) · [report.md](architecture-interactive-3/report.md) · [research.md](architecture-interactive-3/research.md)

### Interactive research tools and accepted steering

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

A two-step baseline and a steered three-step A-axis trial returned finite telemetry in one conversation. The report retains both IDs and numerical observations. Duration, commands and source hashes differ, so this verifies steering and tool round trips without supporting an optimization comparison.

#### Topics

integration verification, interactive conversation, steering, HP4, numerical telemetry

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Interactive conversation retains baseline and accepts a steered trial | aeb62bbdbdda4527978599f294bb852d / 165d697d3d5f4e18b62065fa65ec5703 | Supported as a route check | [Report](architecture-interactive-3/report.md) |

#### References

- [Both trials and source-hash caveat](architecture-interactive-3/report.md)
- [Tracked interactive/resume integration findings](architecture-interactive-3/../../../research/architecture-verification.md)

## [architecture-resume](architecture-resume/)

[abstract.md](architecture-resume/abstract.md) · [research.md](architecture-resume/research.md)

### Architecture resume

Status: local summary incomplete; see tracked verification.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

This directory has no final report and its local notebook does not summarize completed checks. The tracked architecture verification document records the related integration outcomes. Do not infer calibration findings from the starter notebook or the session exit code.

#### Topics

integration verification, partial session summary

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| No local completed hypothesis test summarized | See tracked architecture verification | Not documented locally | [Verification](architecture-resume/../../../research/architecture-verification.md) |

#### References

- [Original saved artifact](architecture-resume/research.md)
- [Original saved artifact](architecture-resume/prompt.txt)
- [Tracked account of route, resume and Viewer checks; consult this for outcomes](architecture-resume/../../../research/architecture-verification.md)

## [architecture-validation](architecture-validation/)

[abstract.md](architecture-validation/abstract.md)

### Architecture validation

Status: local summary incomplete; see tracked verification.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

This directory has no final report and its local notebook does not summarize completed checks. The tracked architecture verification document records the related integration outcomes. Do not infer calibration findings from the starter notebook or the session exit code.

#### Topics

integration verification, partial session summary

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| No local completed hypothesis test summarized | See tracked architecture verification | Not documented locally | [Verification](architecture-validation/../../../research/architecture-verification.md) |

#### References

- [Original saved artifact](architecture-validation/prompt.txt)
- [Tracked account of route, resume and Viewer checks; consult this for outcomes](architecture-validation/../../../research/architecture-verification.md)

## [architecture-viewer-final](architecture-viewer-final/)

[abstract.md](architecture-viewer-final/abstract.md) · [research.md](architecture-viewer-final/research.md)

### Architecture viewer final

Status: local summary incomplete; see tracked verification.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

This directory has no final report and its local notebook does not summarize completed checks. The tracked architecture verification document records the related integration outcomes. Do not infer calibration findings from the starter notebook or the session exit code.

#### Topics

integration verification, partial session summary

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| No local completed hypothesis test summarized | See tracked architecture verification | Not documented locally | [Verification](architecture-viewer-final/../../../research/architecture-verification.md) |

#### References

- [Original saved artifact](architecture-viewer-final/research.md)
- [Original saved artifact](architecture-viewer-final/prompt.txt)
- [Tracked account of route, resume and Viewer checks; consult this for outcomes](architecture-viewer-final/../../../research/architecture-verification.md)

## [architecture-viewer-validation](architecture-viewer-validation/)

[abstract.md](architecture-viewer-validation/abstract.md) · [research.md](architecture-viewer-validation/research.md)

### Architecture viewer validation

Status: local summary incomplete; see tracked verification.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

This directory has no final report and its local notebook does not summarize completed checks. The tracked architecture verification document records the related integration outcomes. Do not infer calibration findings from the starter notebook or the session exit code.

#### Topics

integration verification, partial session summary

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| No local completed hypothesis test summarized | See tracked architecture verification | Not documented locally | [Verification](architecture-viewer-validation/../../../research/architecture-verification.md) |

#### References

- [Original saved artifact](architecture-viewer-validation/research.md)
- [Original saved artifact](architecture-viewer-validation/prompt.txt)
- [Tracked account of route, resume and Viewer checks; consult this for outcomes](architecture-viewer-validation/../../../research/architecture-verification.md)

## [autocal-candidate-model-validation-and-throughput-fdb616a4](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/)

[abstract.md](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/abstract.md) · [report.md](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/report.md) · [research.md](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/research.md)

### Autocal validation, collection throughput and compiled Python physics

Status: complete.

Saved autocal and throughput evidence plus compiled-solver follow-up, updated 2026-10-08.

#### Abstract

Recorded and synthetic HP3 experiments tested candidate validation, bounded adjustments and informative movements. A guarded veto and tie-breaker rejected impossible candidates in 15/15 deliberate counterexamples without changing the eleven recorded selections; no recorded HP3 accuracy gain was established. Bounded updates failed under winding mismatch and poor recorded starts. A production HP4 collection took 1693.12 seconds for twelve observations. Headless JS subsequently improved matched short recorded workloads by 19.03× and retained frozen-replay encoder agreement. A double-precision Warp CPU solver passed 99 fixtures and accelerated complete HP4 updates by 1.96×. Longer controlled comparisons used 20 simulated seconds per trial, three repetitions per HP3 backend and one per HP4 backend. CUDA took 37.24/168.61 seconds on HP3/HP4, versus CPU 26.87/105.95 and JS 1.27/6.63 seconds. Maximum sampled CUDA-to-JS encoder difference was 0.002161°, below the 0.01° comparison bound. Profiling identifies costly serial CUDA execution/readbacks and remaining Python systems. CUDA was slower for these single-world workloads. Fresh complete collection throughput, independent-world GPU batching and hardware accuracy remain unestablished. CPU-only JAX remains a deliberate prior performance choice.

#### Topics

autocalibration, HP3, HP4, candidate-owned spool models, winding mismatch, elasticity, bounded updates, D-optimal movement, sentinel rejection, headless JS, Python parity, Rerun throughput, Warp CPU, CUDA, sequential XPBD, compact state arrays

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| A common unseen sweep fixes history ranking | E01 / E03 | Not consistently supported | [Corrected audit](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/E03-selection-audit-corrected.json) |
| Candidate-owned models and complete coverage reject false validation | E07 | Supported on 15 counterexamples; recorded accuracy unchanged | [Stress results](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/E07-ranking-stress.json), [regressions](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/final-regressions.json) |
| Bounded residual updates improve geometry under model mismatch | E02 / E04 | Mixed; winding and poor starts defeat validation | [Synthetic](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/E02-E04-synthetic-results.json), [recorded](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/E04-recorded-results.json) |
| D-optimal selection chooses the best physical movement | E06 | Mixed; best in 2/3 starts | [Movement results](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/E06-movement-results.json) |
| Native production collection yields ingestible measurements | E05 | Supported for 12 HP4 observations; accuracy unestablished | [Collection summary](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/E05-native-summary.json) |
| Headless JS reduces runtime while retaining encoder parity | P01 / P02 / P03 | Supported on controlled workloads and frozen replay; fresh collection unmeasured | [Matrix](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/P02-matrix.json), [recorded replay](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/P03-final-recorded/replay.json) |
| Compiling the ordered solver accelerates full native physics | C02 / C03 | Supported; HP4 solver 7.90×, whole world 1.96×, still below realtime | [Repeated timing](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/compiled-solver/C03-timing.json), [fixture checks](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/compiled-solver/C02-final-focused.txt) |
| CUDA matches compiled CPU or JS speed | L01 / L02 | Rejected on the tested 20-s workload: 38.6% / 59.1% slower than CPU | [Complete timing](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/cuda-long-run/results.json) |
| CUDA solver matches targeted reference fixtures | C04B | Supported on five short fixtures; longer runs measured separately below | [CUDA comparisons](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/compiled-solver/C04-cuda-parity.txt) |
| Longer sampled encoder agreement stays within 0.01° | L01 / L02 | Supported at all 201 sampled observations; hardware accuracy unestablished | [Comparisons](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/cuda-long-run/results.json), [coverage](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/cuda-long-run/validation.json) |
| Serial CUDA execution/readbacks add cost | L03 | Supported by instrumented profiles; individual timings overlap | [CPU](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/cuda-long-run/L03-native-warp.txt), [CUDA](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/cuda-long-run/L03-native-warp-cuda.txt), [GPU intervals](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/cuda-long-run/L03-kernel-events.json) |

#### References

- [Guarded validation and performance follow-up with limitations](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/report.md)
- [H1–H4, E01–E07 and P01–P04 budgets and decisions](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/research.md)
- [Separate physical and prediction errors](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/prediction-metrics.json)
- [Benchmark method, timings and reproducible replay commands](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/../../../../research/collection-performance.md)

- [Compiled Python solver methods, API, numerical and GPU limits](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/../../../../research/compiled-python-solver.md)
- [Warp CPU implementation](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/../../../../src/python/cable_joints_3d/warp_solver.py): the typed double-precision core and ECS adapter.

- [GPU visibility diagnosis](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/compiled-solver/C04-device-visibility.json): host RTX 3070 Ti exists; isolated `/dev` prevents this process from accessing it.

- [Unsandboxed CUDA detection](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/compiled-solver/C04-unsandboxed-diagnostic.txt): Warp access restored; JAX still lacks its CUDA plugin packages.

- [Long-run benchmark](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/cuda-long-run/benchmark.py), [measurement manifest](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/cuda-long-run/results.json), [repeatability](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/cuda-long-run/repeatability.json): frozen inputs, software/device provenance, timing and sampled consistency.
- [CPU-only JAX setup](sessions/autocal-candidate-model-validation-and-throughput-fdb616a4/../../../../autocal/README.md): existing dependency choice; user supplied its prior profiling rationale, not re-benchmarked here.

## [autocal-comfortable-force-20261010](autocal-comfortable-force-20261010/)

[abstract.md](autocal-comfortable-force-20261010/abstract.md) · [report.md](autocal-comfortable-force-20261010/report.md) · [research.md](autocal-comfortable-force-20261010/research.md)

### HP3 comfortable force tuning and PJRT cache warnings

Status: complete.

#### Abstract

This investigation tested why a no-buildup HP3 reference run selected 12.8849 N and why autocal printed native PJRT compatibility warnings. Offline replay of the saved force curve and one fresh isolated headless HP3 tuning run both selected 2.3957 N using a decline in travel gained per added newton. The minimum moving force remained 0.2273 N. The fresh run used 976.542 simulated seconds and 54.06 wall seconds. A separate JAX 0.9.2 reproduction emitted three warnings on a warm disk cache, versus none with the revised default; numerical output was identical. Autocal now leaves disk caching opt-in, uses equal-duration edge probes, and removes the logistic extrapolation and force-cap success fallback. Ninety-five collector tests and ten JAX tests passed. These are simulated tuning and software checks; no new full calibration or hardware accuracy improvement was measured.

#### Topics

HP3, autocalibration, force autotuning, comfortable workspace, incremental compliance, simulation timing, JAX, PJRT, persistent cache.

#### Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| A plateau-seeking definition permits excessive maximum force | E1, E2 | Supported for the measured curve | [Replay](autocal-comfortable-force-20261010/reference-replay.json), [fresh tuning](autocal-comfortable-force-20261010/tuning.json) |
| Recording delays alone explain the changed force curve | E2 | Inconclusive; a fresh unrecorded headless run reproduced it closely | [Tuning log](autocal-comfortable-force-20261010/tuning.log) |
| Persistent-cache hits cause the exact native warning | E3 | Supported | [Reproduction](autocal-comfortable-force-20261010/jax-repro.json) |
| Revised default and force logic pass regressions | E4 | Supported for tested cases | [Checks](autocal-comfortable-force-20261010/checks.md) |
| Lower force improves full calibration accuracy | none | Not tested | User will repeat manual collection |

#### References

- [Report](autocal-comfortable-force-20261010/report.md): decisions, results and limitations.
- [Notebook](autocal-comfortable-force-20261010/research.md): objective, budget and steering.
- [Supplied reference](autocal-comfortable-force-20261010/../../extended-autocal/no-buildup-reference-001/sweeps.json): original force curve.
- [Recorder manifest](autocal-comfortable-force-20261010/../../extended-autocal/no-buildup-reference-001/hangprinter-20261010T135406.255382Z-2af430aa.json): supplied recording provenance. The large RRD was preserved and was not decoded for this focused fix.
- [JAX issue #36294](https://github.com/jax-ml/jax/issues/36294): upstream report of native warning spam on persistent-cache hits.

Follow-up: [full cache timing investigation](autocal-comfortable-force-20261010/../autocal-jax-cache-20261010/report.md) measured a 43.64% warm-cache saving and restored caching by default with overridable native logging. The earlier opt-in decision described here was provisional and is superseded.

## [autocal-crash-fix-20261007](autocal-crash-fix-20261007/)

[abstract.md](autocal-crash-fix-20261007/abstract.md) · [research.md](autocal-crash-fix-20261007/research.md)

### Autocal native crash isolation to the compiler path

Status: investigation incomplete; no verified production fix.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

Native crashes were reproduced in objective replay outside the sandbox and in compile-only runs without objective execution. A direct jaxlib replay of saved compiler modules reproduced two segfaults among eleven children without importing JAX or autocal. GDB observed a crash during protobuf object destruction inside the XLA compilation path, but did not locate the first invalid write or free. Small reduced controls passed; they are finite controls rather than fixes. Valgrind timed out with uninitialized-value reports, and the sanitizer build failed to produce a usable wheel. No production fix was established. These observations narrow the necessary trigger set toward native compilation and object lifetime.

#### Topics

autocalibration, JAX, jaxlib, XLA, protobuf, SIGSEGV, SIGABRT, compiler lifetime, Valgrind

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Sandbox restrictions are necessary for the crash | Outside-sandbox objective replay | Rejected | [Notebook](autocal-crash-fix-20261007/research.md) |
| Objective execution or JAX tracing is necessary | Compile-only; direct jaxlib replay | Rejected; 2/11 direct replay children segfaulted | [Batch summary](autocal-crash-fix-20261007/raw-prefix-three-run/summary.json) |
| Protobuf destruction is the original corruption site | GDB compilation trace | Inconclusive; observed crash site only | [GDB summary](autocal-crash-fix-20261007/compile-gdb-outside/summary.json) |
| The diagnostic controls establish a production fix | Reduced replay, Memcheck, sanitizer build | Not supported | [Compiler update](autocal-crash-fix-20261007/../../../research/bug-reports/autocal-native-crashes-october-7-2026/autocal-crash-compiler-update.md) |

#### References

- [Isolation decisions and budgets](autocal-crash-fix-20261007/research.md)
- [Tracked findings and compiler reduction results](autocal-crash-fix-20261007/../../../research/bug-reports/autocal-native-crashes-october-7-2026/autocal-crash-compiler-update.md)

## [autocal-held-out-history-ranking-7ec4260f](sessions/autocal-held-out-history-ranking-7ec4260f/)

[abstract.md](sessions/autocal-held-out-history-ranking-7ec4260f/abstract.md) · [report.md](sessions/autocal-held-out-history-ranking-7ec4260f/report.md) · [research.md](sessions/autocal-held-out-history-ranking-7ec4260f/research.md)

### Autocal history ranking with held-out sweeps

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

This investigation reproduced disagreement between calibration ranking and known physical geometry, then tested synthetic sweep selection and whole-sweep prediction. On seventh HP3, the selected fit had 100.737 mm more summed anchor error than another candidate. A synthetic held-out scorer separated exact and perturbed geometry by 40.55x in robust prediction cost, but poor starts still produced physically bad fits. The archived implementation added future-sweep history selection with the existing rank as fallback. Its static regression replays did not measure an accuracy gain from that gate. Native HP4 collection was cancelled before a complete sweep after tracking losses. The report supports ranking diagnostics and a synthetic discriminator; it does not establish reliable calibration accuracy.

#### Topics

autocalibration, HP4, HP3, Slideprinter, candidate ranking, held-out sweeps, D-optimal selection, outliers, poor starts

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| History ranking can select worse known anchors | E1 | Supported on recorded fixtures | [Measurements](sessions/autocal-held-out-history-ranking-7ec4260f/report.md#measurements) |
| Held-out prediction distinguishes exact and perturbed geometry | E2b | Supported in one synthetic test; integrated accuracy unmeasured | [Notebook](sessions/autocal-held-out-history-ranking-7ec4260f/research.md) |
| D-opt ranking remains reliable from poor starts | E2a | Not supported; all poor-start fits physically bad | [Measurements](sessions/autocal-held-out-history-ranking-7ec4260f/report.md#measurements) |
| Production HP4 collection yields a usable sweep | E3 / 4b4d1aeb4eed4f3fafe9c45393c02f52 | Inconclusive; cancelled before sweep completion | [Notebook](sessions/autocal-held-out-history-ranking-7ec4260f/research.md) |

#### References

- [Archived implementation, measurements and limits](sessions/autocal-held-out-history-ranking-7ec4260f/report.md)
- [Hypotheses, E0–E3 decisions and retained collection path](sessions/autocal-held-out-history-ranking-7ec4260f/research.md)

## [autocal-history-ranking-baseline-incomplete-a71aec03](sessions/autocal-history-ranking-baseline-incomplete-a71aec03/)

[abstract.md](sessions/autocal-history-ranking-baseline-incomplete-a71aec03/abstract.md) · [research.md](sessions/autocal-history-ranking-baseline-incomplete-a71aec03/research.md)

### Autocal history-ranking baseline, unfinished record

Status: incomplete record; final research outcome not documented.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

The notebook records a baseline audit of historical Slideprinter histories. Repeated ten_points_bigger_deltas runs selected physically worse candidates with 10.96–14.90 mm parameter regret. It proposes whole-sweep prediction and bounded residual updates, and records a native HP4 collection job plus a running regression baseline. The last saved decision still awaits those results. There is no final report or documented completion of the proposed selector experiments. Although exit.json records a successful process exit, that does not resolve the unfinished research record. Treat this as useful baseline evidence and a record of intended experiments, rather than a completed assessment of a new algorithm.

#### Topics

autocalibration, Slideprinter, history ranking, ground-truth regret, proposed held-out validation, HP4 collection

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| History ranking can prefer physically worse fits | Historical baseline audit in notebook | Supported in saved audit | [Notebook](sessions/autocal-history-ranking-baseline-incomplete-a71aec03/research.md) |
| Held-out prediction improves selection under noise and winding | None completed in saved record | Not tested to completion in this record | [Proposed hypotheses](sessions/autocal-history-ranking-baseline-incomplete-a71aec03/research.md) |
| Bounded residual updates improve robustness | None completed in saved record | Not tested to completion in this record | [Proposed hypotheses](sessions/autocal-history-ranking-baseline-incomplete-a71aec03/research.md) |
| Native collection supports follow-up validation | d2e98d4fa0f7491cbaaede24521eecfa | Started; final outcome not summarized | [Notebook](sessions/autocal-history-ranking-baseline-incomplete-a71aec03/research.md) |

#### References

- [Saved hypotheses, baseline measurements and outstanding decisions](sessions/autocal-history-ranking-baseline-incomplete-a71aec03/research.md)

## [autocal-jax-cache-20261010](autocal-jax-cache-20261010/)

[abstract.md](autocal-jax-cache-20261010/abstract.md) · [report.md](autocal-jax-cache-20261010/report.md) · [research.md](autocal-jax-cache-20261010/research.md)

### Autocal persistent-cache performance and quiet native logging

Status: complete.

#### Abstract

Five complete 11-dataset calibration-log regression batteries compared a fresh persistent cache, cache disabled, and warm persistent caching using JAX/jaxlib 0.9.2. Cache-off runs took 97.82 and 96.48 seconds; warm-cache runs took 53.68 and 55.83 seconds. Mean completion time fell from 97.15 to 54.75 seconds, saving 42.40 seconds (43.64%, 1.77x speedup). All 55 calibration children exited zero, all reference comparisons passed, and parsed final and per-iteration numerical outputs were exactly identical. A warm unsuppressed battery emitted 292 PJRT compatibility warnings; the warm battery with final defaults emitted zero. Twelve JAX/cache tests passed. Autocal now restores persistent caching by default and sets overridable TF_CPP_MIN_LOG_LEVEL=2, retaining native errors, Python warnings and ordinary stderr. The warning is an upstream unimplemented compatibility-check notice; JAX cache keys include library/backend versions. Timing includes ordinary background load and one off-run overlapped small tests; these measurements demonstrate benefit on this fixture suite, not universal performance or hardware accuracy.

#### Topics

Autocalibration, JAX, PJRT, persistent compilation cache, regression replay, CPU timing, native logging, numerical reproducibility.

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Persistent caching reduces end-to-end regression time | C1–C5 | Supported: warm-cache mean 43.64% lower | [Analysis](autocal-jax-cache-20261010/analysis.json) |
| Caching changes fitted results or causes failures on this suite | C1–C5 | Rejected for 55 tested children; no differences or failures | [Numerical comparisons](autocal-jax-cache-20261010/analysis.json), per-child logs in C1–C5 |
| Native log level 2 silences the compatibility warning without changing JIT output | D2, C5 | Supported: zero warnings, identical outputs | [Reproduction](autocal-jax-cache-20261010/warning-repro.json), [final battery](autocal-jax-cache-20261010/C5/summary.json) |
| Explicit cache/logging overrides and Python diagnostics remain functional | D3 | Supported by four fresh-process settings cases | [Checks](autocal-jax-cache-20261010/checks.md) |

#### References

- [Full report](autocal-jax-cache-20261010/report.md): timing, validation and implementation details.
- [Notebook](autocal-jax-cache-20261010/research.md): objective, accepted steering and experiment choices.
- [Previous investigation](autocal-jax-cache-20261010/../autocal-comfortable-force-20261010/report.md): initial warning reproduction and provisional opt-in decision.
- [Provenance](autocal-jax-cache-20261010/provenance.json): interpreter, dependency versions, revision, workers and CPU affinity.
- [Benchmark wrapper](autocal-jax-cache-20261010/benchmark.py): observational instrumentation around the unchanged regression main entry point.
- [Analysis script](autocal-jax-cache-20261010/analyze.py): exact numerical comparison and timing arithmetic.
- [Implementation diff](autocal-jax-cache-20261010/changes.diff): cache/logging defaults, docs and test changes.
- [JAX cache warning issue #36294](https://github.com/jax-ml/jax/issues/36294): upstream cache-hit warning report.
- [JAX 0.9.2 cache-key source](https://github.com/jax-ml/jax/blob/jax-v0.9.2/jax/_src/cache_key.py): library/backend versions are hashed into cache keys.
- [XLA pinned-source compatibility notice](https://github.com/openxla/xla/blob/187a5eb58277a85847d1516bd1e20b7faf03d5ef/xla/python/pjrt_ifrt/pjrt_executable.cc): an unimplemented version check emits the warning rather than detecting an actual mismatch.

## [autocal-jax-reference-cache-20261010](autocal-jax-reference-cache-20261010/)

[abstract.md](autocal-jax-reference-cache-20261010/abstract.md) · [report.md](autocal-jax-reference-cache-20261010/report.md) · [research.md](autocal-jax-reference-cache-20261010/research.md)

### JAX cache reuse on the recorded HP3 extended reference

Status: complete.

#### Abstract

Does the autocal JAX cache help the recorded HP3 no-buildup reference, and do
executables warmed by the regression battery transfer? Fifteen offline replays
used the regression runner's unchanged launch and comparison helpers with fixed
radii supplied for anchor-only log parsing. Two-run mean times were 24.19 s
with caching disabled, 23.35 s with the regression-warmed cache at its original
path, and 16.79 s after warming on the reference dataset. Dataset warming saved
7.39 s (30.57%). Compiler diagnostics showed 4/16 objective variants reused
from the battery and 16/16 from the dataset-warmed cache. All numerical results
matched, including full-precision fit checkpoints and the original score 8161.
The cache is a general compilation optimization whose variants depend on array
shapes and static solver settings. A methodology correction was necessary:
JAX 0.9.2 also hashes an auxiliary cache path, so relocated cache copies were
excluded from transfer conclusions. Timings measure replay computation on one
CPU, with two repeats per main arm; they exclude collection and recording time.

#### Topics

autocalibration, HP3, Hangprinter, JAX, CPU, persistent compilation cache,
cache transfer, shapes, static arguments, replay, numerical reproducibility

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Cache modes preserve calibration outputs. | D1–D15 | Supported for this dataset. | [Analysis](autocal-jax-reference-cache-20261010/analysis.json), [original comparison](autocal-jax-reference-cache-20261010/original-comparison.txt), [D15](autocal-jax-reference-cache-20261010/D15/summary.json). |
| The regression-warmed cache transfers fully to the reference. | D12–D14 | Mixed: partial reuse, 4 of 16 objective variants. | [D14 compiler counts](autocal-jax-reference-cache-20261010/D14/summary.json). |
| Warming on this dataset reduces replay time. | D1/D5 vs D4/D7, D15 | Supported: 30.57% saved, all variants reused. | [Analysis](autocal-jax-reference-cache-20261010/analysis.json), [D15](autocal-jax-reference-cache-20261010/D15/summary.json). |
| Copying cache files to another directory preserves usable keys. | D2/D6/D8/D9/D11 vs D12–D15 | Rejected under current JAX defaults. | [Report](autocal-jax-reference-cache-20261010/report.md), [relocated D11](autocal-jax-reference-cache-20261010/D11/summary.json). |

#### References

- [Full report](autocal-jax-reference-cache-20261010/report.md): method, correction, measurements and limitations.
- [Research notebook](autocal-jax-reference-cache-20261010/research.md): decisions and accepted steering.
- [Input dataset](autocal-jax-reference-cache-20261010/../../extended-autocal/no-buildup-reference-001/sweeps.json): simulated recorded measurements.
- [Provenance](autocal-jax-reference-cache-20261010/provenance.json): source SHA-256, revision 9422620c, original battery cache manifest.
- [Preceding cache battery investigation](autocal-jax-reference-cache-20261010/../autocal-jax-cache-20261010/report.md): eleven-dataset timings and cache provenance.
- [JAX persistent cache documentation](https://docs.jax.dev/en/latest/persistent_compilation_cache.html): compiled-program reuse and key inputs.

## [autocal-ranking-bounded-updates-negative-results-b7deb1bc](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/)

[abstract.md](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/abstract.md) · [report.md](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md)

### Autocal ranking, bounded updates and identifiability

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

Seven hypotheses were investigated using recorded histories, controlled replay, synthetic bounded updates and native radius sensitivity. Ranking disagreed with physical accuracy in 22 adjacent transitions across seven of eleven datasets. Whole-sweep prediction selected worse physical results on five datasets. Removing the iteration-age bonus improved one case but increased aggregate selected parameter error by 200.701 mm over the full matrix. Bounded residual updates and informative movement selection worked in some synthetic conditions and failed under others. No tested solver or selection patch met the acceptance criteria. The retained change added ranking/ground-truth diagnostics. This session has a detailed report and experiment registry despite lacking a research.md notebook; online collection accuracy remained unvalidated.

#### Topics

autocalibration, HP3, Slideprinter, history age bonus, held-out prediction, bounded updates, D-optimal movements, radius uncertainty, negative results

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Ranking agrees with physical accuracy | Recorded ranking audit | Rejected | [Audit](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/recorded_ranking_audit.json) |
| Whole-sweep prediction repairs ranking generally | Held-out audit | Rejected; worse in 5/11 fixtures | [Predictions](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/heldout_audit.json) |
| Objective guard, TRF or reconstruction objective improves replay | ten_points_objective_guard / ten_points_trf / ten_points_reconstruction | Rejected | [Controlled trials](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md#controlled-replay-trials) |
| Removing history age bonus improves the full matrix | ten_points_no_age; paired regression runs | Rejected; aggregate error +200.701 mm | [Paired results](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/paired_regressions.json) |
| Bounded joint fitting and D-opt movements repair poor starts | Synthetic and recorded joint trials | Mixed; rejected for production | [Synthetic results](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/synthetic_results.json), [recorded results](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/recorded_joint_results.json), [movement results](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/active_results_v2.json) |
| Good motor tracking makes radius uncertainty negligible | Native sensitivity comparison | Rejected | [Comparison](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/native_comparison.json) |

#### References

- [Seven hypotheses, accuracy criteria and negative findings](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md)
- [Experiment inputs and log paths](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/experiment_registry.json)
- [Frozen source, dataset and evidence hashes](sessions/autocal-ranking-bounded-updates-negative-results-b7deb1bc/artifact_hashes.json)

## [fifth-hp3-reference-audit](fifth-hp3-reference-audit/)

[abstract.md](fifth-hp3-reference-audit/abstract.md) · [report.md](fifth-hp3-reference-audit/report.md) · [research.md](fifth-hp3-reference-audit/research.md)

### Why the fifth HP3 reference trajectory differs today

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

Isolated March and current source replays used the same present-day numerical environment. They produced identical intermediate fits, both differing from the historical trajectory by 390.788 mm in the diagnostic mean. March code still returned the saved 70.314 mm calibration; current history selection returned 70.787 mm. The large trajectory penalty is dominated by underconstrained attempts that are excluded from final selection. Subsequent tracked algorithm changes are therefore unnecessary to reproduce the trajectory difference. Numerical environment or execution differences remain a leading explanation, but the original runtime lacked enough provenance to establish the cause. Source rollback alone did not restore the historical trajectory.

#### Topics

autocalibration, HP3, historical reference, numerical environment, sentinel fits, regression metrics

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Later source changes caused the historical trajectory penalty | March df699275 and current isolated replay | Rejected as a necessary cause | [Results](fifth-hp3-reference-audit/results.json) |
| The large trajectory penalty measures returned calibration quality | Reference/current metric decomposition | Rejected | [Report](fifth-hp3-reference-audit/report.md#what-the-regression-row-measures) |
| The exact original runtime difference is identified | Historical provenance inspection | Inconclusive; original environment missing | [Replay](fifth-hp3-reference-audit/report.md#controlled-replay) |

#### References

- [Metric decomposition, source history and replay controls](fifth-hp3-reference-audit/report.md)
- [Controlled comparison measurements](fifth-hp3-reference-audit/results.json)
- [Preserved historical settings](fifth-hp3-reference-audit/reference-settings.json)

## [headless-autocal-validation](headless-autocal-validation/)

[abstract.md](headless-autocal-validation/abstract.md) · [report.md](headless-autocal-validation/report.md) · [research.md](headless-autocal-validation/research.md)

### Production JS headless full-auto autocalibration

Status: complete.

#### Abstract

The integration runs the existing full-auto autocal loop against production
JavaScript physics in Node. The final fresh HP3 global-radius trial completed
after nine iterations and eleven sweeps/220 points, applied M669/M666, and
returned a 39.02 mm radius. It used 6487.832 simulated seconds and 435.65 wall
seconds. The solver rated the fit ideal (1.039), but its D height differs from
the nominal preset, so completion and fit score do not establish parameter
accuracy. Independent matched Chromium/Node collection
measured sixty points per backend. Maximum differences were 0.02 degrees raw
angles and 0.005236 mm lengths, within documented 0.03 degree/0.01 mm tolerances.
Absolute simulated timestamps differed by one second after an independent
settling boundary; sampling durations agreed within one fixed step. A SIGTERM
test retained completed points and stopped owned services. Failed initialization
and overly strict comparison trials are retained. Final review corrected a
remaining force-ramp wall sleep; both extended checks passed after that
correction. These are simulated tests, not hardware calibration evidence.

#### Topics

autocalibration, HP3, headless Node, Chromium, measurement parity, global radius,
simulation clock, lifecycle

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Raw unbaked scenes can initialize collection | HP3-FULL-01 | Rejected | [Failed manifest](headless-autocal-validation/../../headless-verification/hp3/sweeps.headless_1/manifest.json) |
| Chromium and Node produce matched collection | HP3-FINAL-CLOCK-01 | Supported within stated tolerances | [Matched measurements](headless-autocal-validation/../../headless-verification/final-clock/test_fresh_hp3_full_auto_globa0/browser/parity.json) |
| Fresh HP3 full-auto completes automatically | HP3-FINAL-CLOCK-01 | Supported; accuracy not established | [Complete manifest](headless-autocal-validation/../../headless-verification/final-clock/test_fresh_hp3_full_auto_globa0/hp3/sweeps.headless/manifest.json) |
| Bootstrap interruption retains evidence and cleans services | HP3-FINAL-CLOCK-01 | Supported | [Interrupted manifest](headless-autocal-validation/../../headless-verification/final-clock/test_interrupted_bootstrap_pre0/sweeps.headless/manifest.json) |

#### References

- [Notebook](headless-autocal-validation/research.md): experiment decisions and constraints.
- [Report](headless-autocal-validation/report.md): results and limitations.

## [headless-extended-autocal-validation](headless-extended-autocal-validation/)

[abstract.md](headless-extended-autocal-validation/abstract.md) · [report.md](headless-extended-autocal-validation/report.md) · [research.md](headless-extended-autocal-validation/research.md)

### Headless extended autocal recording validation

Status: complete; full recording verified, pre-existing parity discrepancy retained.

#### Abstract

The production headless JavaScript world completed default full-auto calibration
with one finalized extended-reference RRD: eight sweeps, 160 points, 267,711
physics observations and 616,025 events in 1525.41 wall seconds. Archive checks
verified exact final datasets/logs, all measurements, paired command outcomes,
stage artifacts and explicit source clocks. Live reset, 25×, external-recorder,
disconnection and interruption checks passed. Failed trials exposed a drain
wait and a backlog-sensitive disconnect; their partial evidence is retained.
The separate Chromium/Node comparison reproduced a one-point 0.16-degree
discrepancy, including with the original headless service. Tolerances were not
relaxed. These are simulated recording-tool checks, not evidence of calibration
accuracy or hardware behavior.

#### Topics

autocalibration, headless simulation, HP3, extended reference, Rerun, RRD,
acknowledgements, source clocks, interruption, process ownership.

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Ten seconds total is sufficient to drain all live events during stepping | 001 | Rejected | [Failed trial](headless-extended-autocal-validation/../../headless-extended-verification/) |
| Interrupted bootstrap preserves partial evidence and finalizes recording | 003 | Supported | [Interrupted manifest](headless-extended-autocal-validation/../../headless-extended-interruption-progress/test_interrupted_bootstrap_pre0/sweeps.headless/manifest.json) |
| Recorded force tuning can complete under high-rate control traffic | 002, 004 | Supported after transport change | [Failed trial](headless-extended-autocal-validation/../../headless-extended-verification-progress/), [rerun](headless-extended-autocal-validation/../../headless-extended-verification-no-ping/) |
| Default full-auto completes and preserves all sources in one RRD | 004 | Supported | [Archive verification](headless-extended-autocal-validation/../../headless-extended-verification-no-ping/test_fresh_hp3_full_auto_globa0/hp3/rrd-verification.json) |
| Matched Chromium/Node collection meets the existing parity tolerance | 004, 005, 006 | Rejected for these inputs, also with original service | [Repeated discrepancy](headless-extended-autocal-validation/../../headless-extended-parity-repeat/parity.json), [original-service baseline](headless-extended-autocal-validation/../../headless-extended-parity-baseline/parity.json) |

#### References

- [Notebook](headless-extended-autocal-validation/research.md): decisions and failed trials.
- [Report](headless-extended-autocal-validation/report.md): current verification status.
- [Extended reference guide](headless-extended-autocal-validation/../../../autocal/data/references/extended-reference.md): protocol and clocks.
- [Earlier headless verification](headless-extended-autocal-validation/../../../research/headless-autocal-verification.md): default calibration and matched-input parity contract.

## [hp4-baseline-a-axis-tool-check-93c49db4](sessions/hp4-baseline-a-axis-tool-check-93c49db4/)

[abstract.md](sessions/hp4-baseline-a-axis-tool-check-93c49db4/abstract.md) · [report.md](sessions/hp4-baseline-a-axis-tool-check-93c49db4/report.md)

### HP4 baseline and A-axis motion tool check

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

Two 200-step native experiments completed. The candidate used a 0.01 rad A-axis target and ended 0.157217 mm from the baseline effector position. Numerical tools passed; Viewer inspection was blocked by the approval policy. This tests tool execution and readback rather than calibration improvement.

#### Topics

integration verification, HP4, native experiment, A-axis motion, Rerun approval

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Native tools execute and compare a controlled motion | 59de4042e3e747bf910dc9957c710dc9 / 22f857e8987149ab93220cfc18f568a6 | Supported as a tool check | [Report](sessions/hp4-baseline-a-axis-tool-check-93c49db4/report.md) |
| Viewer inspection completes in this session | Candidate recording | Blocked; not completed | [Report](sessions/hp4-baseline-a-axis-tool-check-93c49db4/report.md) |

#### References

- [Saved numeric check](sessions/hp4-baseline-a-axis-tool-check-93c49db4/report.md)
- [Actual tool calls](sessions/hp4-baseline-a-axis-tool-check-93c49db4/events.jsonl)
- [Tracked verification details and experiment IDs](sessions/hp4-baseline-a-axis-tool-check-93c49db4/../../../../research/verification.md)

## [hp4-motion-check-no-results-3c944b0c](sessions/hp4-motion-check-no-results-3c944b0c/)

[abstract.md](sessions/hp4-motion-check-no-results-3c944b0c/abstract.md)

### Planned HP4 motion check with no recorded results

Status: no research result documented.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

The saved artifacts contain no completed research report or updated experiment summary. No tested hypothesis can be established from this record. Startup notebook text and process status are not evidence that an experiment ran.

#### Topics

incomplete archive, no summarized experiments

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| No completed hypothesis test documented | None recorded | Not tested in the saved summary | Original artifacts below |

#### References

- [Original saved artifact](sessions/hp4-motion-check-no-results-3c944b0c/prompt.txt)

## [hp4-saved-recording-viewer-check-338606b1](sessions/hp4-saved-recording-viewer-check-338606b1/)

[abstract.md](sessions/hp4-saved-recording-viewer-check-338606b1/abstract.md) · [report.md](sessions/hp4-saved-recording-viewer-check-338606b1/report.md)

### Saved HP4 recording Viewer inspection

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

The existing HP4 candidate recording was opened at step 200. All six blueprint views rendered and no Viewer warnings or errors were reported. Physics was not rerun. This verifies recording inspection and screenshot capture.

#### Topics

integration verification, Rerun, HP4, recording inspection, sim_step 200

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Viewer can inspect a saved recording at its final step | 22f857e8987149ab93220cfc18f568a6 | Supported as a tool check | [Screenshot](sessions/hp4-saved-recording-viewer-check-338606b1/sim_step_200.png) |

#### References

- [Saved inspection result](sessions/hp4-saved-recording-viewer-check-338606b1/report.md)
- [Actual tool calls](sessions/hp4-saved-recording-viewer-check-338606b1/events.jsonl)
- [Tracked validation account](sessions/hp4-saved-recording-viewer-check-338606b1/../../../../research/verification.md)

## [native-and-viewer-mcp-check-3725e1e9](sessions/native-and-viewer-mcp-check-3725e1e9/)

[abstract.md](sessions/native-and-viewer-mcp-check-3725e1e9/abstract.md) · [report.md](sessions/native-and-viewer-mcp-check-3725e1e9/report.md)

### Native and Viewer MCP round-trip check

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

A fresh three-step HP4 numerical experiment returned finite telemetry at 0.006 seconds. The existing candidate recording was then inspected at step 200 with zero Viewer warnings or errors. This checks both MCP servers; it makes no calibration-accuracy claim.

#### Topics

integration verification, MCP, HP4, numeric telemetry, Rerun

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Both numerical and Viewer MCP servers work in one session | Fresh three-step check; 22f857e8987149ab93220cfc18f568a6 | Supported as a tool check | [Report](sessions/native-and-viewer-mcp-check-3725e1e9/report.md) |

#### References

- [Saved results](sessions/native-and-viewer-mcp-check-3725e1e9/report.md)
- [Actual MCP calls](sessions/native-and-viewer-mcp-check-3725e1e9/events.jsonl)
- [Tracked SDK and route verification](sessions/native-and-viewer-mcp-check-3725e1e9/../../../../research/verification.md)

## [reference-refresh-october-7-2026](reference-refresh-october-7-2026/)

[abstract.md](reference-refresh-october-7-2026/abstract.md) · [report.md](reference-refresh-october-7-2026/report.md) · [research.md](reference-refresh-october-7-2026/research.md)

### Future-sweep selection versus refreshed autocal references

Status: complete.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

Eleven new main-branch reference logs were generated with source and environment provenance, then reproduced exactly in a bounded two-worker regression run. Against that baseline, research_findings_1 changed final history selection while preserving every intermediate fit. Summed returned parameter error fell from 885.158 to 816.936 mm, a 7.71% reduction. Three datasets improved, five worsened and three were unchanged. This is a net gain on this battery with explicit regressions, not a uniform accuracy improvement. Candidates were evaluated on different future-sweep sets. Earlier full-concurrency runs suffered native crashes; the successful bounded runs do not establish a root-cause fix or guaranteed crash prevention.

#### Topics

autocalibration, HP3, Slideprinter, whole-sweep prediction, history selection, regression references, concurrency

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| The refreshed main references reproduce exactly | Baseline verification | Supported across 11 datasets | [Baseline log](reference-refresh-october-7-2026/baseline-verification-bounded.log) |
| Future-sweep history selection improves all datasets | research_findings_1 comparison | Mixed; 3 improve, 5 worsen, 3 unchanged | [Comparison](reference-refresh-october-7-2026/comparison.json) |
| Two-worker scheduling fixes the native crash cause | Bounded baseline and branch runs | Inconclusive; successful runs only | [Provenance](reference-refresh-october-7-2026/provenance.json) |

#### References

- [Per-dataset errors, selections and validation-set limitations](reference-refresh-october-7-2026/report.md)
- [Machine-readable branch comparison](reference-refresh-october-7-2026/comparison.json)
- [Source, runtime and scheduling controls](reference-refresh-october-7-2026/provenance.json)

## [unassigned-research-no-results-1cb5b420](sessions/unassigned-research-no-results-1cb5b420/)

[abstract.md](sessions/unassigned-research-no-results-1cb5b420/abstract.md) · [research.md](sessions/unassigned-research-no-results-1cb5b420/research.md)

### Unassigned research session with no summarized result

Status: no research result documented.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

The saved artifacts contain no completed research report or updated experiment summary. No tested hypothesis can be established from this record. Startup notebook text and process status are not evidence that an experiment ran.

#### Topics

incomplete archive, no summarized experiments

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| No completed hypothesis test documented | None recorded | Not tested in the saved summary | Original artifacts below |

#### References

- [Original saved artifact](sessions/unassigned-research-no-results-1cb5b420/research.md)
- [Original saved artifact](sessions/unassigned-research-no-results-1cb5b420/prompt.txt)
- [Original saved artifact](sessions/unassigned-research-no-results-1cb5b420/exit.json)

## [unassigned-research-unsuccessful-exit-f9120c9b](sessions/unassigned-research-unsuccessful-exit-f9120c9b/)

[abstract.md](sessions/unassigned-research-unsuccessful-exit-f9120c9b/abstract.md) · [research.md](sessions/unassigned-research-unsuccessful-exit-f9120c9b/research.md)

### Unassigned research session with unsuccessful exit

Status: no research result documented.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

The saved artifacts contain no completed research report or updated experiment summary. No tested hypothesis can be established from this record. Startup notebook text and process status are not evidence that an experiment ran.

#### Topics

incomplete archive, no summarized experiments

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| No completed hypothesis test documented | None recorded | Not tested in the saved summary | Original artifacts below |

#### References

- [Original saved artifact](sessions/unassigned-research-unsuccessful-exit-f9120c9b/research.md)
- [Original saved artifact](sessions/unassigned-research-unsuccessful-exit-f9120c9b/prompt.txt)
- [Original saved artifact](sessions/unassigned-research-unsuccessful-exit-f9120c9b/exit.json)

## [unsuccessful-launch-no-record-32819264](sessions/unsuccessful-launch-no-record-32819264/)

[abstract.md](sessions/unsuccessful-launch-no-record-32819264/abstract.md)

### Unsuccessful launch with no research record

Status: no research result documented.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

The saved artifacts contain no completed research report or updated experiment summary. No tested hypothesis can be established from this record. Startup notebook text and process status are not evidence that an experiment ran.

#### Topics

incomplete archive, no summarized experiments

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| No completed hypothesis test documented | None recorded | Not tested in the saved summary | Original artifacts below |

#### References

- [Original saved artifact](sessions/unsuccessful-launch-no-record-32819264/exit.json)

## [unsuccessful-launch-no-record-4d08c2cf](sessions/unsuccessful-launch-no-record-4d08c2cf/)

[abstract.md](sessions/unsuccessful-launch-no-record-4d08c2cf/abstract.md)

### Unsuccessful launch with no research record

Status: no research result documented.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

The saved artifacts contain no completed research report or updated experiment summary. No tested hypothesis can be established from this record. Startup notebook text and process status are not evidence that an experiment ran.

#### Topics

incomplete archive, no summarized experiments

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| No completed hypothesis test documented | None recorded | Not tested in the saved summary | Original artifacts below |

#### References

- [Original saved artifact](sessions/unsuccessful-launch-no-record-4d08c2cf/exit.json)

## [unsuccessful-launch-no-record-8225a40f](sessions/unsuccessful-launch-no-record-8225a40f/)

[abstract.md](sessions/unsuccessful-launch-no-record-8225a40f/abstract.md)

### Unsuccessful launch with no research record

Status: no research result documented.

Retrospective summary of saved evidence, written 2026-10-08.

#### Abstract

The saved artifacts contain no completed research report or updated experiment summary. No tested hypothesis can be established from this record. Startup notebook text and process status are not evidence that an experiment ran.

#### Topics

incomplete archive, no summarized experiments

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| No completed hypothesis test documented | None recorded | Not tested in the saved summary | Original artifacts below |

#### References

- [Original saved artifact](sessions/unsuccessful-launch-no-record-8225a40f/exit.json)

## [2026-10-23-autocal-agenda](sessions/2026-10-23-autocal-agenda/)

[abstract.md](sessions/2026-10-23-autocal-agenda/abstract.md) · [report.md](sessions/2026-10-23-autocal-agenda/report.md) · [research.md](sessions/2026-10-23-autocal-agenda/research.md)

### Autocal research allocation after mixed consensus evidence

Status: complete (desk exercise; next numerical assignment not tested).

#### Abstract

What should receive the next research allocation after broader replay rejects
blanket finite-prediction consensus membership? This desk exercise compares saved
consensus, ranking, solver-attribution and model-adequacy evidence with current
selection code. CM-E1 reports one gain and two regressions across eight additional
datasets; all three changed comparisons lack common unseen sweeps. The agenda
therefore retains production defaults and proposes a bounded compatible-sweep
search, with at most six later frozen prediction evaluations if eligible data
exist. It compares four routes and demonstrates how withdrawing CM-E1 would
replace that allocation with evidence recovery while preserving other findings.
Direct claim links suffice; no graph infrastructure was added. Saved results
were inspected, not independently reproduced. Some historical raw artifacts are
unavailable here. No fits, selection/prediction experiments, physics, collection
or hardware measurements were performed; improved calibration accuracy is not
established by this workflow result.

#### Topics

autocalibration, research agenda, common unseen sweeps, consensus, model adequacy,
evidence dependencies, negative results, withdrawal exercise, research allocation

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Direct evidence links suffice for a scoped allocation and next brief | AG-D1 | Supported for this four-route desk exercise | [Agenda](sessions/2026-10-23-autocal-agenda/../../../../research/autocal-agenda.md), [checks](sessions/2026-10-23-autocal-agenda/checks.json) |
| Removing a supporting result changes the allocation while preserving other scope | AG-D2 | Supported for a counterfactual CM-E1 withdrawal | [Exercise](sessions/2026-10-23-autocal-agenda/../../../../research/autocal-agenda.md#withdrawal-exercise-what-if-cm-e1-is-unavailable) |
| The proposed common-fold probe improves calibration | None | Not tested | [Proposed assignment](sessions/2026-10-23-autocal-agenda/../../../../research/autocal-agenda.md#next-assignment-common-unseen-sweeps-for-changed-applied-models) |

#### Decision handoff

- Scope: task 005, evidence/source inspection at fd10fd94; no runtime changes.
- Review: this agent inspected saved reports, CM-E1 JSON and current consumers;
  hashes/links/queue/catalog checked. No independent numerical reproduction.
- Decision: keep defaults; use the linked agenda and defer graph tooling.
- Next: commission the compatible-unseen-sweep assignment; absent eligible data,
  scope model adequacy separately. The numerical assignment remains proposed.

#### References

- [Full report](sessions/2026-10-23-autocal-agenda/report.md), [notebook](sessions/2026-10-23-autocal-agenda/research.md), [verification](sessions/2026-10-23-autocal-agenda/checks.json): actual work and limitations.
- [Working agenda](sessions/2026-10-23-autocal-agenda/../../../../research/autocal-agenda.md): route priorities, claim dependencies and bounded next assignment.
- [Producer CM-E1](sessions/2026-10-23-autocal-agenda/../2026-10-23-consensus-membership/report.md), [saved measurements](sessions/2026-10-23-autocal-agenda/../2026-10-23-consensus-membership/results.json): reused scientific finding.
- [Handoff procedure](sessions/2026-10-23-autocal-agenda/../../../../research/handoffs.md): durable evidence recovery.
