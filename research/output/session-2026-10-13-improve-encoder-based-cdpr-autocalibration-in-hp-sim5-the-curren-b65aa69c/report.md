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

## What changed

1. `autocal/control/primitives/uncalibrated_actions.mjs:14` - the
   encoder-stability quiet window `DEFAULT_STABILITY_WINDOW_MS` goes from
   `1500` to `1000`. This is the whole production speed change; expectations in
   `autocal/control/tests/primitives/simulation_clock.test.mjs` and
   `autocal/control/tests/behaviors/run_force_trial.test.mjs` follow it.
2. `autocal/history_selection.py:126` (`consensus_anchor_estimate`) and
   `autocal/history_selection.py:208` (`apply_consensus_plan`), wired in at
   `autocal/autocal.py:670`, replace only the *applied* anchors with the mean of
   the closest pair of distinct fitted anchor sets (near-duplicates under
   0.01 mm are merged first). The ranked candidate that the loop already
   computes stays the reported fit-quality diagnostic and still decides when to
   stop; `history_select: consensus ...` is logged and
   `history_consensus_members` / `history_consensus_spread_mm` are added to the
   summary metadata. New unit tests: `autocal/tests/test_history_selection.py`.
3. Docs: `autocal/README.md` lists the new stage experiment and
   `autocal/objective_functions_overview` records that the ordering chooses the
   reported candidate while the applied geometry comes from the consensus.

Collection, planning, fitting, patience and automatic stopping are unchanged,
so the standard autocal regressions still describe the shipped behaviour. Both
changes are small: one numeric constant, one new pure function plus its call
site.

## How it was measured

- **Metric: applied-anchor RMSE.** `sqrt(mean_i ||a_i - truth_i||^2)` over the
  four anchors, parsed from each run's own final `Anchors (M669):` line - the
  geometry a user would actually get. Truth is the simulator's nominal HP3
  layout (`A=[0,-1900,-280]`, `B=[1645.45,950,-280]`, `C=[-1645.45,950,-280]`,
  `D=[0,0,1900]` mm) and is never an input to any selector.
- **Fresh supervised runs** (HP3, no layering, `--buildup-factor 0`,
  `--find-radii off`, `--extended-reference`): `E05` at 20 points/sweep,
  `E17` at 10 points/sweep with the shipped window, `E18` at 6 points/sweep with
  the shipped selector. The user's reference run `no-buildup-reference-003`
  (10 points/sweep, 1500 ms window) is the baseline.
- **Offline decks** reuse candidates persisted by real runs
  (`sweeps.stages/fit-NNN-default.json` and `history-NNN.json`), so selector and
  stopping questions were answered by replay instead of new physics.

## Results

### The quiet window (E07, E16, E16b)

E01/E01b attributed collection time from the reference recording: the
encoder-quiet poll loop owns ~63% of simulated time (817 multi-poll waits =
3886 s of 6756 s), moves ~20%, force tuning ~10%, noise sampling ~6%. Replaying
those 817 recorded waits (E07) showed a 1000 ms quiet window ends 791 of them
earlier and saves 490 s (7.3%) of the reference run's time while moving the
recorded angle by at most 0.04 deg (p99). E16 extended the sweep to vibration
windows and tolerances: every relaxation beyond the quiet window raised the p99
angle change to 7-20 deg, so it was rejected. E16b repeated the replay on HEAD's
own trace (E05) and measured 528.5 s / 7.8% saved at p99 0.12 deg, max 1.25 deg.
The change is unconditional and does not touch the stopping logic.

### More points do not help (E05)

At HEAD with the 1500 ms window, doubling density to 20 points/sweep produced
440 data points in 11 sweeps and 6805 simulated seconds (1345 s wall) for an
applied RMSE of 2.977 mm - no better than the reference's 2.828 mm at half the
points. More data is not the lever.

### Fewer points per sweep (E13, E18)

E13's offline dense-thinning replay suggested 6/10 points keep the last
iteration's accuracy (2.64 vs 2.53 mm), but that did not survive the shipped
selection path in E14b. E18 is the end-to-end test: 6 points/sweep, the shipped
1000 ms window and the shipped consensus selector. It was accepted
(`patience-or-threshold`) after 10 sweeps and 120 data points, applying
2.192 mm - the best of the four runs:

| run | window ms | points/sweep | data points | sweeps | sim s | wall s | applied RMSE |
| --- | --- | --- | --- | --- | --- | --- | --- |
| reference-003 | 1500 | 10 | 180 | 9 | 6756 | n/a | 2.828 mm |
| E05 (2x density) | 1500 | 20 | 440 | 11 | 6805 | 1345 | 2.977 mm |
| E17 (shipped window) | 1000 | 10 | 220 | 11 | 5807 | 1054 | 2.581 mm |
| E18 (shipped window + selector) | 1000 | 6 | 120 | 10 | 5032 | 934 | 2.192 mm |
| E34 (consensus on reference data) | 1500 | 10 | 180 | 9 | replay | replay | 2.623 mm |

So the reduced-density run used 33% fewer data points and 26% less simulated
time than the reference while applying a *better* geometry. E37 explains why the
time saving is smaller than the point reduction: per-command simulated-time
attribution (`events.jsonl`) shows force changes and pullouts dominate, and they
scale with *sweeps*, not with the points inside a sweep. E17 (220 points) took
5807 simulated seconds against E05's 6805 (440 points) at the same sweep count,
so halving the points bought only ~15%.

The reason no point-density default is shipped is the offline corpus: over the
thinning replays, `replay_pts10*` families settle at 2.62 mm, `pts8*` at 2.83,
`pts6*` at 2.90, and `pts4*` degenerates to a 1220 mm fit. One end-to-end run
cannot separate a real 6-point effect from run-to-run noise of that size, and
E18's accuracy win is already explained by the consensus selector (E34 applies
2.623 mm to the reference's own data). Trading a possible small accuracy cost
for ~13% of collection time is not worth it when a failed calibration means
re-collecting everything, so `--sweep-points` stays the user's knob.

### Selecting better anchors (E19-E33, E35)

The task names the case where the selection score disagrees with geometric
accuracy, and it reproduces: on the reference candidate set the shipped ranking
applies a 2.826 mm fit while a 2.528 mm candidate is sitting in the same list,
and the newest iteration can never win because no sweep is left to predict
(E19). Replacing the ranking was already a documented negative result in
[the bounded-updates session](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md),
so this session keeps the ranking as the diagnostic and changes only the applied
geometry.

E19-E32 are the rule matrix: unguarded ensembles are unsafe. Averaging all
candidates gave 907 mm, the mean of the top three by rank 485 mm, and a
"twice-best-rank band" mean 291 mm, all because one near-planar fit can dominate
the mean. The rule that survived every dataset is bounded by real fits: merge
near-duplicates, then average the closest remaining pair. Its estimate can move
by at most half a real separation, it never leaves the convex hull of two fitted
candidates, and it degrades gracefully to the single best candidate when only
one exists.

E33 runs the production `consensus_anchor_estimate` over 11 de-duplicated
candidate sets persisted by real runs: it is better on 10 and worse on 1, moving
the mean applied error 4.273 -> 3.092 mm, the worst case 10.060 -> 4.353 mm and
the mean regret against an oracle pick 1.594 -> 0.414 mm. E35 shows the shipped
implementation reproduces the validated reference rule exactly across a merge
tolerance sweep of 0.001-1.0 mm (max anchor delta 0.0e+00 mm), and E34 shows the
same-data replay of the reference improves 2.828 -> 2.623 mm. The residual risk
is the one dataset that regresses (3.114 -> 4.154 mm); it is accepted because
the mean, the worst case and the regret all improve, and because the rule cannot
leave the span of two real fits.

### Single fits are unstable (E38)

E38 replays each run's `fit-NNN-default.json` sequence - accuracy attainable
after N sweeps. Individual fits diverge wildly (E05 jumps to 2112 mm and
2333 mm on two iterations; E17 to 1877 mm and 1395 mm), while the consensus
estimate stays near the final value. In E05 the consensus was already at its
final 2.459 mm at sweep 2 and never moved through sweep 9; the reference reached
2.809 mm at sweep 4 of 7; E18 sat at 2.2 mm from its third sweep. So the shipped
change buys robustness as well as accuracy: it turned two catastrophic single
fits into ordinary ones, and it shows the loop routinely collects far more
sweeps than the geometry needs.

### Early stop by consensus stability (E39)

Because E38 shows saturation, a natural next step is to stop when the applied
estimate stops moving. E39 evaluated that offline over the whole corpus: a rule
that stops when the last K consensus deltas fall below a tolerance saves only
9-15% of sweeps and can cost up to 2.6 mm of accuracy (E17 would stop on a
5.110 mm plateau for three consecutive sweeps). A stable estimate is not
evidence of a correct one, so this was rejected and no stopping logic changed.
The automatic stopping behaviour and the one-click user experience are
untouched.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| The encoder-quiet wait loop dominates time and 1000 ms is the largest safe window | E01, E01b, E07, E16, E16b | Supported, shipped | `E07_settle_criteria.py`, `E16b_head_settle_replay.py` |
| Causal early exit of individual moves saves much more | E03, E03b, E03c | Mostly rejected (8.8% of move-wait time) | `E03_settles.py` |
| Relaxing tolerance / vibration window is a safe extra saving | E16 | Rejected (p99 7-20 deg) | `E16_settle_variants.py` |
| More points help accuracy | E05 | Rejected (2.977 mm at 2x points) | `native/autocal/E05-baseline-head` |
| Fewer points per sweep keeps accuracy end-to-end | E13, E14b, E18 | Mixed (best run, but within run-to-run noise; corpus trends worse) | `native/autocal/E18-sweeppoints6/sweeps.full_auto.log`, `E38-sweep-budget.json` |
| Fewer points per sweep saves time proportionally | E37 | Rejected (sweep overhead dominates) | `E37_profile_headless.py` |
| The selection score disagrees with geometric accuracy | E14b, E15, E19, E33 | Supported | `E19-selection.json`, `E33-validate-impl.json` |
| The newest iteration can be selected | E19 | Rejected (structurally unselectable) | `E19-selection.json` |
| Any candidate ensemble is safe | E19-E32 | Rejected (291-907 mm blow-ups) | `E19`-`E32` decks |
| The bracketed consensus pair is closer to truth than the shipped pick | E28, E33, E34, E35, E36 | Supported, shipped | `E33-validate-impl.json`, `E35-impl-vs-md2.json` |
| Single fits are stable enough to trust alone | E38 | Rejected | `E38-sweep-budget.json` |
| Consensus stability is a safe early stop | E39 | Rejected (up to 2.6 mm loss for <=15%) | `E39-stop-rule.json` |

## Negative and unshipped results (short notes)

- Replacing the ranking score: not retried; already rejected in
  [the bounded-updates session](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md)
  (22 rank/accuracy mismatches; removing the age bonus cost +200.7 mm).
- Mean-of-candidates, rank-band means, centrality-k means: blow up to 291-907 mm
  when one candidate is a degenerate fit (E19-E32). Only the bracketed pair is safe.
- Stable-mean / medoid single pick: cannot beat the bracketed pair because a
  single pick has no bracket (E28).
- Convergence-based early stop: rejected (E39).
- Relaxing the settle tolerance or the vibration window: rejected (E16).

## Limitations

- All measurements are simulated headless JS physics. No hardware run was made,
  so the 7.8% saving and the anchor RMSEs are simulator evidence, not machine
  truth. Real encoders have different noise and vibration structure.
- The 2.6 mm-truth anchors are the simulator's nominal layout; a real machine's
  true anchors are unknown, which is the reason the consensus rule is bounded by
  fitted candidates rather than trusted to a score.
- `research_preflight` reported only a launcher-restart warning caused by an
  unrelated supervisor file; the session's production changes are `autocal/*.py`
  and one `.mjs` constant, which that hash check does not cover.
- `scripts/research_socket_probe.py` was not run in the agent command context;
  supervised connectivity is not command-socket proof. The supervisor-owned
  collection, recorder and firmware paths were exercised end-to-end anyway.
- The consensus regressed on one of eleven candidate sets; the change is
  justified on mean, worst case and regret, not on unanimity.

## Reproducing

```bash
# unit / regression
.venv/bin/python -m pytest autocal/tests -q --ignore=autocal/tests/test_headless_sim.py
npx jest autocal/control/tests/primitives autocal/control/tests/behaviors \
         autocal/control/tests/bridge autocal/control/tests/cli

# offline decks (no physics)
MPLCONFIGDIR=/tmp/mpl .venv/bin/python E33_validate_impl.py
MPLCONFIGDIR=/tmp/mpl .venv/bin/python E35_impl_vs_md2.py
MPLCONFIGDIR=/tmp/mpl .venv/bin/python E36_applied_rmse.py
MPLCONFIGDIR=/tmp/mpl .venv/bin/python E38_sweep_budget.py
MPLCONFIGDIR=/tmp/mpl .venv/bin/python E39_stop_rule.py
```

## References

- [Searchable abstract](abstract.md) and [research notebook](research.md).
- Reference run to beat: `output/extended-autocal/no-buildup-reference-003`
  (9 sweeps x 20 points, 6756 simulated seconds, applied RMSE 2.828 mm).
- This session's supervised runs and offline decks: `native/autocal/E05-baseline-head`,
  `native/autocal/E17-window1000-refdensity`, `native/autocal/E18-sweeppoints6`,
  `E01`-`E39` in this directory. Repository revision `ded5842e`.
- Prior sessions continued here:
  [held-out history ranking](../autocal-held-out-history-ranking-7ec4260f/report.md),
  [bounded rank updates (negative)](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/report.md),
  [candidate model validation and throughput](../autocal-candidate-model-validation-and-throughput-fdb616a4/report.md),
  [history ranking baseline (incomplete)](../autocal-history-ranking-baseline-incomplete-a71aec03/report.md).
- Method references consulted for the code path and tooling:
  `autocal/README.md`, `autocal/optimization_algorithm_overview`,
  `autocal/objective_functions_overview`,
  `autocal/README_elliptical_feature_calibration.md`,
  `autocal/data/references/extended-reference.md`, `research/README.md`.
