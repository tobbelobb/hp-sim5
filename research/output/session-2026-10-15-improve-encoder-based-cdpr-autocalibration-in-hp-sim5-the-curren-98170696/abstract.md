# Warm-starting the HP3 encoder autocalibration anchor solve (and a negative result on point thinning)

Status: complete for the simulated evidence in this session.

## Abstract

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

## Topics

autocalibration, HP3, CDPR, encoder-only, warm start, initial guess, point-count reduction, sweep
density, sampling grid, automatic stopping, patience, candidate score versus geometry, settling,
simulated versus hardware evidence, no line buildup

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| A low `rank_score` identifies geometrically accurate anchors | E01 | Rejected | [Stage audit](experiments/e01_reference_stage_audit.json) |
| Blindly halving points per sweep keeps geometry | E02, E05 | Rejected cold, supported warm | [Thinning](experiments/e02_point_subsets.json), [warm chains](experiments/e05_warmstart.json) |
| Warm-starting iterations stabilises and improves the anchor solve | E05, E10, E11, E13, E15, E16, E17, E18 | Supported (simulated) | [Live stats](experiments/e13_live_runs.json), [E16](experiments/e16_live_thinning.json) |
| Cold-start failure is a multi-start budget problem; more restarts fix it | E07, E08 | Rejected | [Multi-start](experiments/e07_multistart.json), [restarts](experiments/e08_restarts.json) |
| The cold seed only has the wrong scale | E09, E12 | Mixed | [Length seed](experiments/e09_length_seed.json), [scale ladder](experiments/e12_scale_ladder.json) |
| Warm start also reduces the number of iterations at full density | E10, E14, E17, E18 | Rejected (same 7 iterations, 9 sweeps) | [Live stats](experiments/e13_live_runs.json) |
| Halving the point density is accuracy-neutral in the live one-click job | E13, E14, E15, E16 | Rejected: coarse collection gives worse data | [Live stats](experiments/e13_live_runs.json), [E16](experiments/e16_live_thinning.json) |
| Point-count-aware patience recovers the accuracy lost by halving points | E15, E16 | Rejected: extra sweeps at half density do not improve accuracy | [E16](experiments/e16_live_thinning.json) |
| A 3-sweep 10-point live dataset is enough for HP3 | E06 | Rejected (near-degenerate data) | [E06](experiments/e06_live_halfpoints.json) |
| Collection wall time is set by requested settling, not physics work | E03 | Supported | [Sweep manifest](native/00403d4402004c09aaac4ccedd97b7c4/86feab22c3934b469686a3c39ae57b5f/manifest.json) |
| Wall-clock time is a usable comparison metric between jobs | E14, E18 | Rejected (identical config, 69% apart) | [Live stats](experiments/e13_live_runs.json) |
| A coarser drive grid is worse because of residual settling / hysteresis per step | none | Not tested | Proposed in [report.md](report.md#next-steps-proposed-not-tested) |
| Shorter proven-stable settle timeouts preserve geometry | none | Not tested | Proposed in [report.md](report.md#next-steps-proposed-not-tested) |

## References

- [Full report](report.md): methods, matched live and offline results, implementation and limitations.
- [Research notebook](research.md): objective, budgets, accepted steering and per-experiment log.
- [Reference dataset](data/reference_sweeps.json) and [stage artifacts](data/reference_stages/):
  supplied HP3 no-buildup run (9 sweeps x 20 points, 573 s wall) used as the baseline.
- [Live job stats](experiments/e13_live_runs.json) (E13, E14, E15, E17, E18),
  [live iterations](experiments/e13_live_iterations.json) and
  [E16 live-data thinning](experiments/e16_live_thinning.json).
- Preceding sessions: [settling reuse and thinning audit](../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/),
  [ranking and bounded-update negative results](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/),
  [candidate validation and throughput](../autocal-candidate-model-validation-and-throughput-fdb616a4/).
