# Faster, more robust encoder-only autocalibration for a Hangprinter 3: a shorter encoder-quiet window plus a bracketed geometric consensus of fitted anchors

Status: complete (all planned runs finished or reported; evidence is simulated).

## Abstract

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

## Topics

autocalibration, CDPR, Hangprinter 3, encoder-only calibration, anchor selection,
candidate ranking, held-out sweeps, encoder settle window, collection time,
early stopping, robustness, simulated headless physics.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| The encoder-quiet wait loop dominates collection time, and a 1000 ms window is its largest safe reduction | E01, E01b, E07, E16, E16b | Supported (shipped) | [E07 / E16b](report.md#the-quiet-window-e07-e16-e16b) |
| Relaxing the quiet tolerance or the vibration window buys more time safely | E16 | Rejected | [E16](report.md#negative-and-unshipped-results) |
| More data points improve accuracy | E05 | Rejected | [E05](report.md#more-points-do-not-help-e05) |
| Fewer points per sweep is a proportional time saving with no accuracy cost | E13, E18, E37, replay corpus | Mixed / rejected as a lever | [E18](report.md#fewer-points-per-sweep-e13-e18) |
| Fewer points per sweep keeps end-to-end accuracy | E13, E14b, E18 | Mixed (best single run, inside run-to-run noise) | [E18](report.md#fewer-points-per-sweep-e13-e18) |
| The shipped selection score disagrees with geometric accuracy, and the newest iteration is structurally unselectable | E14b, E15, E19, E33 | Supported | [E19 / E33](report.md#selecting-better-anchors-e19-e33-e35) |
| Replacing the ranking score fixes selection | E14b (this session); prior sessions | Already rejected; not retried | [prior session](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md) |
| The geometric consensus of the fitted candidates is closer to truth than the best-ranked single candidate | E33, E35, E34, E36 | Supported (shipped) | [E33 / E35](report.md#selecting-better-anchors-e19-e33-e35) |
| Any ensemble of candidates is a safe selection rule | E19-E32 | Rejected | [E19-E32](report.md#negative-and-unshipped-results) |
| Single fits are stable enough to trust without cross-candidate evidence | E38 | Rejected | [E38](report.md#single-fits-are-unstable-e38) |
| Consensus stability can stop collection early without losing accuracy | E39 | Rejected | [E39](report.md#early-stop-by-consensus-stability-e39) |

## References

- [Full report](report.md): analysis, measurements, shipped diff and limitations.
- [Research notebook](research.md): objectives, budgets, steering and the E01-E39 log.
- Local evidence: reference run `output/extended-autocal/no-buildup-reference-003`
  (the run to beat); supervised runs `native/autocal/E05-baseline-head`,
  `E17-window1000-refdensity`, `E18-sweeppoints6`; offline decks `E19`-`E39` in
  this directory. Repository revision `ded5842e`.
- Prior sessions: [held-out history ranking](../autocal-held-out-history-ranking-7ec4260f/report.md),
  [bounded rank updates](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md),
  [candidate model validation](../autocal-candidate-model-validation-and-throughput-fdb616a4/report.md).
