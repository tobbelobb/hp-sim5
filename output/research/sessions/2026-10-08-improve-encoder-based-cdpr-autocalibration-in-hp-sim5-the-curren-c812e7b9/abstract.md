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
