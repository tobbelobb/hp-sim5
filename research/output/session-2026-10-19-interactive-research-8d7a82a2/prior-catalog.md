# Research session catalog

Short abstracts and tested hypotheses from local sessions. Missing or pending summaries are not evidence of completed experiments.

Regenerate with `.venv/bin/python scripts/research_index.py`.

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

### Reliable HP4 autocal and HP3/HP4 simulation motion

Status: in progress.

#### Abstract
This session investigates recurring HP4 autocal failures, simulation motion differences from HP3, and the reliability of four physics substeps. Historical reports and logs identify multiple distinct failures, including motor load bookkeeping, damping feedback, remaining post-move motion and browser wall deadlines. Current source and native headless experiments will distinguish these mechanisms. No new experiment result or reliable calibration result is available yet. All planned experiments are simulated; no hardware measurement is proposed.

#### Topics
HP4, HP3, autocalibration, torque mode, numerical damping, physics substeps, headless speedup, settling, Rerun

#### Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Current HP4 failures and reliable correction | none | Not tested | [Notebook](sessions/2026-10-19-interactive-research-8d7a82a2/research.md) |

#### References

- [Full report](sessions/2026-10-19-interactive-research-8d7a82a2/report.md): pending results.
- [Research notebook](sessions/2026-10-19-interactive-research-8d7a82a2/research.md): scope, budgets and prior sessions.

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
