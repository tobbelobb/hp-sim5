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
- [Tooling assessment](report.md#follow-up-simulator-usefulness-trust-and-tooling): no direct physics/viewer use here; evidence limits and proposed improvements.
- [Earlier backend performance/parity](../../../../research/collection-performance.md) and [native collector/viewer verification](../../../../research/native-collection-verification.md): preceding tooling evidence, not experiments in this session.
- [Previous model-validation research](../autocal-candidate-model-validation-and-throughput-fdb616a4/report.md): prior warm-start and own-model-ranking failures.
- [Previous negative ranking research](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md): prior ambiguity and age-score failures.

Browser fixture provenance clarified by the user: reference data came from browser physics collection; much configuration is already in source and JSON. The proposed metadata improvement is automatic linkage to the generating revision/scene, not additional measurement. See [clarification](report.md#clarification-existing-browser-fixture-definitions).

Implemented the follow-up as one compact [reference-data README](../../../../autocal/data/references/README.md), using existing JSON and verified git history. [History audit](provenance-history.json); no new calibration experiment or runtime change.
