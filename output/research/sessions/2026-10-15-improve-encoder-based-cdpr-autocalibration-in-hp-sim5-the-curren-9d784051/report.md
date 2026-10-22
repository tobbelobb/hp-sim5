# Trust-region anchor solving improves the tested HP3 regime, with a legacy-fixture regression

Status: complete. All evidence below is **simulated** HP3 (hp-sim5, line layering
disabled, `--buildup-factor 0`, `--find-radii off`); nothing here is
hardware-measured, and tool checks below are labelled as such.

## Abstract

Encoder-based CDPR autocalibration in hp-sim5 searches for anchor geometry with a
multi-start local solver, and earlier sessions showed its selection score often
disagrees with true geometric accuracy. This session profiled a full headless HP3
run (5180.8 simulated seconds; 21% per-sweep pull-outs, 24% force tuning, 24%
per-point settling, 23% travel), re-tested force tuning, pull-out bounds, point
counts and solver settings, and measured what the algorithm can actually achieve.
Fit-only replay of the same recorded data showed the dominant error term is not
selection but the common error of the fitted pool: the applied consensus (2.706 mm)
is only 0.32 mm worse than the best candidate in the pool (2.385 mm). The largest
reproducible gain is the anchor solver: switching the default from `lbfgsb` to
bounded `trf` cut mean applied RMSE over 10 recorded datasets from 4.348 mm to
3.590 mm (-17%, worst case 9.963 -> 9.206 mm) and, in a full headless run, took the
control result from 2.706 mm to 2.410 mm with byte-identical collected data,
identical 5180.8 simulated seconds and ~4x less solver CPU. The default was shipped
and verified end-to-end. A clean replay of the repository's 11 legacy fixtures
(Slideprinter/HP4 with line buildup and fitted spool radii, outside the scoped
regime) shows the opposite sign: mean applied error rises from 16.2 mm to 27.9 mm
there, because `trf`'s more self-consistent fits change which candidates the
tightest-pair consensus selects. That regime is therefore excluded from the claim
and flagged in the report. Limitations: all evidence is simulated HP3 in one physics
world; the winning solver was validated on 10 datasets from a single world
configuration, so cross-world robustness remains unmeasured.

Review note for the main checkout (2026-10-15): this worktree changed and tested
the default, but the change was not imported into main. The wider simulated
environment matrix in section 10 modestly favors `trf` in the tested no-buildup
HP3 regime, but does not exercise extreme fit failures and does not remove the
current-main legacy regressions in section 9.

## 1. What the research worktree changed

`autocal/_autocal_common.py`: the default value of `--solve-optimizer` changed from
`lbfgsb` to `trf`, with help text. Docs updated in `autocal/README.md`,
`autocal/README_elliptical_feature_calibration.md` and
`autocal/optimization_algorithm_overview`; the default expectation in
`autocal/tests/test_autocal_cli.py` was updated to match.

Why this is safe to ship and cheap to evaluate:

- No collection code changed. The stored dataset format, the force tuning, the
  pull-outs and the per-point sampling are untouched, so all collection
  regressions remain valid and the one-click flow is unchanged.
- End-to-end evidence shows the *collected data is identical* to the previous
  default (`truth_score` 14.7953 identical, 5180.8 simulated seconds identical).
  Only the fitted anchors change.
- The result beats the reference run `no-buildup-reference-003` (2.828 mm applied
  RMSE) by 15% at the same simulated cost.
- Known scope limit: the gain is a no-buildup HP3 result. On the 11 checked-in
  legacy fixtures (line buildup plus spool-radius fitting) `trf` is *worse*
  (16.2 -> 27.9 mm mean applied error, section 4.8), so those configurations should
  keep `--solve-optimizer lbfgsb` until they get their own validation.

## 2. How the current algorithm works (short)

`autocal.py --headless-sim` bootstraps 3 sweeps (with a force-tuning pass), then
loops: fit anchors, plan the next ellipse sweep, collect it, re-fit, and stop when
the fit score stops improving for `--patience` iterations or a threshold is hit.
Two sub-sweeps per sweep traverse the same arc in opposite drive directions. The
applied geometry is the *consensus* (mean of the two closest fitted anchor sets)
over all iterations, not the single best-ranked fit.

## 3. Cost model of one headless run (`H1-control`, 5180.8 simulated s)

| block | simulated s | share |
| --- | ---: | ---: |
| per-sweep pull-out waits (13 commands of 73.5-100 s) | 1097 | 21.2% |
| force tuning before the first sweep (`M569.4` ramps + releases) | 1240 | 23.9% |
| per-point `G1` quiet/settle (180 points x ~7 s) | 1257 | 24.3% |
| `G1` sweep travel (28.4 m commanded at 1400 mm/min) | 1217 | 23.5% |
| micro-moves, releases, other modes | ~370 | 7.1% |

Inside the force-tuning block: 11 bracket probes of 25-27 s (bounded by
`AUTO_TUNE_STALL_WINDOW_MS = 25000`), 2 bisect probes of ~28 s, 11 edge-ramp
trials of 30.5 s each (`AUTO_TUNE_SAMPLE_WINDOW_MS * 3`), and 6 force-release
`M569.4` commands that grow 8 -> 40 s as the spooled-out length grows. Making the
probes or ramps shorter changes the force they measure, so shortening those
windows was analysed and deliberately **not** attempted: the collected travel at
the shorter window would fall below the `minUsefulTravelDeg = 0.5` and
`thetaActThr = 0.5` thresholds and re-select a different start/mid/max force.

## 4. Tested hypotheses

### 4.1 Solver choice dominates (E23, E24, E25, E26) - supported

`E23` on the control dataset: the pool of 7 fitted anchor sets from the run has an
**oracle** (best single candidate) RMSE of 2.385 mm, while the applied consensus is
2.706 mm. So selection regret is only 0.32 mm; the other 2.4 mm is shared by the
whole pool. Fit-side battery on the same dataset (applied RMSE): `lbfgsb` 2.706,
`trf` **2.410**, `lm` 2.423, `lbfgsb --solve-restarts 16` 2.736,
`trf --solve-restarts 16` 2.410, `trf --solve-iterations 800` 2.410,
`trf --optimizer-mode fast-fd` 2.410, `--solve-restarts 1` 2337.7 (diverged).

`E24` extended the comparison to every recorded dataset (fit-only replay, ~15 s
each):

| dataset | `lbfgsb` | `trf` | delta |
| --- | ---: | ---: | ---: |
| `H1-control` | 2.706 | 2.410 | -0.296 |
| `reference` (browser run's dataset) | 2.623 | 2.409 | -0.214 |
| `G2-sforce091` | 2.609 | 2.054 | -0.555 |
| `G3-sforce136` | 3.824 | 2.823 | -1.001 |
| `D3-points8` | 3.184 | 3.163 | -0.021 |
| `B6-points14` | 2.969 | 2.614 | -0.355 |
| `B5-sforce220` | 6.289 | 5.287 | -1.002 |
| `G1-sforce068` | 9.963 | 3.403 | -6.560 |
| `B1-points6` | 2.192 | 2.527 | +0.335 |
| `P1-cap15` | 7.118 | 9.206 | +2.088 |
| **mean** | **4.348** | **3.590** | **-17.4%** |
| **worst** | **9.963** | **9.206** | |

`trf` wins on 8 of 10 datasets and collapses the worst case. It also produces much
tighter consensus spreads (0.06-1.5 mm vs 2.2-32 mm), i.e. independent restarts
land in the same optimum instead of scattering - the direct mechanism behind the
gain. `lm` is not an alternative: 2.423 / 1269.8 / 2.19 mm on H1 / G1 / B1.

`E25/E26` end-to-end full headless runs confirmed the fit-only prediction:

| run | flags | sweeps | points | simulated s | applied RMSE |
| --- | --- | ---: | ---: | ---: | ---: |
| `H1-control` | default (`lbfgsb`) | 9 | 180 | 5180.8 | 2.706 |
| `T1-trf` | `--solve-optimizer trf` | 9 | 180 | 5180.8 | **2.410** |
| `V1-verify-default` | **no flags** (patched default) | 9 | 180 | 5180.8 | **2.410** |

`V1-verify-default` is the regression proof for the shipped change: identical
simulated seconds, identical force tuning
(`idle=0.0114 start=0.2273 edge=3.3540 d_max=1677.770deg`), identical consensus
(`members=[5,6] spread_mm=0.06416`) and an identical `truth_score` (14.7953) to the
old default, with the fit result changing from 2.706 to 2.410 mm. Solver CPU on the
`H1` replay also dropped (19.4 s wall / 67.2 s CPU for `lbfgsb` vs 14.4 s / 16.2 s
for `trf`), because `trf` uses SciPy's bounded trust-region least squares instead
of the JAX L-BFGS-B path.

### 4.2 Selection score vs truth (E01-lab, E23) - supported, small in practice

The earlier disagreement is real (the loop's cost-based patience and the applied
consensus are not the same thing, and the applied consensus is not the best
candidate), but on the datasets measured here its price is 0.32 mm, far smaller
than the solver-induced 0.3-6.6 mm spread. Ranking fixes are therefore not the
first lever; fixing the common pool error is.

### 4.3 Sensor collection force (E21, E25) - valley, measured, not shipped

The default sensor collection force is `min(forceMax, 5 * forceMid)` = 1.136 N
here. Full headless runs with `trf` (fit-only replay marked *):

| sensor force | run | applied RMSE |
| ---: | --- | ---: |
| 0.68 N | `G1-sforce068`* | 3.403 |
| 0.91 N | `G2-sforce091`*, `T2-trf-sf091` | 2.054, **2.016** |
| 1.136 N (default) | `T1-trf`, `H1-control`* | 2.410, 2.706 |
| 1.36 N | `G3-sforce136`* | 2.823 |
| 2.2 N | `B5-sforce220`* | 5.287 |

The minimum sits at ~0.91 N, and it matches a physical criterion that the force
tuning already measures: the *peak incremental compliance* of the tuning ramp is
494 deg/N at 0.87 N (from `H1`'s `edge-ramp` samples), i.e. **3.8 x `forceMid`**,
while the current default is 5 x `forceMid`. This is the most promising follow-up
because it is data-driven rather than a magic multiplier, but it was **not** shipped:
the valley is steep on the low side (0.68 N is much worse), the 0.91 N runs exist
for only one simulated world, and lowering the force costs 353 simulated seconds
(`T2` 5533.8 s vs `T1` 5180.8 s) through slower per-point settles
(`g1-step quiet` 1257.0 -> 1607.1 s), so it is not a strict improvement on both axes.

### 4.4 Fewer or cheaper data points (E02, E22, E25) - mixed/rejected

- `--sweep-points 6` with `trf` (`T3`, 108 points, 9 sweeps, 4868.3 s) gives
  2.514 mm: 6% faster, 4% less accurate than `T1`. Sublinear: halving the points
  saves ~6%, not ~25%, because the per-sweep arc is re-scaled to the machine and the
  pull-out and force tuning remain.
- The two sub-sweep directions of one sweep trace the *same* arc. Keeping only the
  first direction gives 72.3 mm, only the second 3.41 mm, both 2.706 mm, while the
  truth-anchor residual of the two halves is nearly equal (14.56 / 15.03). So the
  asymmetry is an identifiability/fit effect, not a data-quality effect, and
  "one direction only" is not a safe speedup.
- Step thinning to 90 points gives 5.103 mm (even steps) or 2.641 mm (odd steps)
  with `lbfgsb` - a 2.5 mm swing from *which* points are kept, which is exactly the
  solver fragility that the `trf` change addresses.
- `T4` (`trf` + 6 points + 0.91 N) reaches 2.031 mm with 8 sweeps / 96 points and
  4773.0 s: 8% faster and 25% more accurate than the old control. This is the best
  measured configuration, but it bundles a weakened point count whose own effect
  was negative, so it is reported as an option rather than shipped.

### 4.5 Pull-out bound (E20) - rejected

Cutting the pull-out wait loses the endpoint by 435 mm (5 s cut), 271 mm (15 s) and
96 mm (30 s); extrapolating the measured creep makes it worse (570 mm vs 422 mm at a
25 s cut). The winch spools out until tension balances the command, so the wait is
bounded below by physics, not by a settle criterion. The pull-out block is 21% of a
run and is *not* recoverable this way.

### 4.6 Consensus-spread early stop (E27) - signal confirmed, rule rejected

Prefix replay of already-collected sweeps is a faithful simulation of "stop after
k sweeps" (the collector's next target depends only on decisions already taken,
and those were all "collect"). With `trf` on `H1`, the consensus spread is 9.7 mm
at 4-6 sweeps, 2.27 mm at 7 and 0.064 mm at 8, and the answer is final from 8
sweeps: one sweep (580 s, 11%) is pure waste. But over 11 datasets a
"stop when spread <= 1 mm" rule saved a sweep on only 2 of them (`H1`, `G2`, the
latter at +0.09 mm error), because most runs terminate on patience with a spread
above 1 mm. The signal is useful as a diagnostic (and is already logged), not as a
stop rule; it was left unimplemented.

### 4.7 Relative-cost multiples (E21) and force tuning stability

The force tuning output is identical in every run performed here
(`idle=0.0114 start=0.2273 edge=3.3540 d_max=1677.770deg`), which is why the sensor
force sweep isolates the *collection* force rather than the tuning.

### 4.8 Compatibility with the legacy fixtures, and harness validity (E28-E33)

The repository's 11 checked-in reference fixtures (`autocal/data/references/*.json`
with `DATASETS` in `autocal/tools/regress_calibration_logs.py`) are Slideprinter and
HP4 recordings with line buildup and spool-radius fitting - outside this session's
scoped regime, but they are the suite `regress_calibration_logs.py` compares against.
Replaying them with each solver (tools/refset_solver_compare.py, and the clean pools
of `tools/stage_clean_pools.py`) shows that **`trf` is a regression there**:

| group | solver | mean applied RMSE | mean oracle | mean selection regret | pools > 20 mm |
| --- | --- | ---: | ---: | ---: | ---: |
| target (11 no-buildup HP3 recordings) | `lbfgsb` | 4.158 | 2.863 | 1.295 | 0 |
| target | `trf` | **3.485** | 3.218 | **0.267** | 0 |
| legacy (11 checked-in fixtures) | `lbfgsb` | **16.151** | 13.929 | 2.222 | 3 |
| legacy | `trf` | 27.941 | 15.910 | 12.031 | 4 |

The three large legacy regressions are `23_april_2026` (27.4 -> 72.7 mm),
`seventh_hp3_dataset` (59.6 -> 85.7 mm) and
`sixteenth_hp3_dataset_even_more_pressure` (11.9 -> 71.4 mm). Root cause (E29): all
three are recorded with buoyant/high-pressure forces and fitted radii, and with
`trf` the *two earliest* full-auto iterations agree with each other very tightly
(`sixteenth`: z = -244.5 mm on iterations 1-2 against a true -280 mm) while later,
much better iterations scatter more. `_select_history_summary_candidate`
(`autocal/history_selection.py::consensus_anchor_estimate`) applies the mean of the
*tightest pair*, so a tight-but-wrong early pair wins. `lbfgsb`'s noisier restarts
happen to break that symmetry on these fixtures.

Every alternative pool-selection rule tested is worse (E30, clean pools, offline
recomputation verified against each run's own applied anchors to <= 0.004 mm):
`last3` 24.30, `last4` 27.37, `swhalf` 31.02, `swmax` 235.2, `topk3` 31.14 and
single-best 34.03 against the current 27.94 for legacy `trf`; and in the target group
*all* alternatives are worse than the current rule for both solvers (e.g. `swhalf`
7.28 vs 4.16 on `lbfgsb`). `last3` would fix the three legacy `trf` regressions
(28.1/51.8/11.9) but destroys `fifth_hp3_dataset` (18.4 -> 108.5), i.e. the
tightest-pair heuristic is fragile in both directions and no re-ranking tested here
generalises. Combining both solvers in one pool (E33) is also no fix: legacy 22.39
(20.56 with a data-volume floor) still loses to `lbfgsb` alone (16.15), and target
4.27 (3.57) loses to `trf` alone (3.485).

Harness validity (E31). The staged pools prepared for this comparison initially came
from `tools/gen_artifacts.py`, which gives both solver runs **one shared dataset
copy**. Concurrent runs then append into that sibling `<dataset>.replay_tmp.json`
resume file: a controlled test on `sixteenth_hp3_dataset_even_more_pressure` ended
with 10 and 12 sweeps where the source has 8, and produced 12.79 mm (`lbfgsb`) and
12.47 mm (`trf`) instead of the canonical 11.878 and 71.367 mm. Isolated or
sequential runs end at exactly the source sweep count and reproduce the reference
harness bit-for-bit, so all numbers in this section come from
`tools/stage_clean_pools.py` (one dataset copy per run) and
`autocal/tools/regress_calibration_logs.py`. The contaminated numbers in
`/tmp/arts_a|b` were discarded. This is also a small product finding: two autocal
processes replaying the same dataset file interfere through that resume file.

The checked-in `autocal/data/references/*.full_auto_reference_run_october_8_2026.log`
files were **not** regenerated: they record per-iteration output of the previous
default and were left as the historical baseline. A maintainer who wants them to
match today's default must rerun `autocal/tools/regress_calibration_logs.py` (or
`--solve-optimizer lbfgsb` the affected fixtures) as a separate, deliberate change.

### 4.9 Fewer data points by decimation (E34) - rejected

Thinning the recorded `data_points` of each sweep to an evenly spaced subset N and
re-fitting (`tools/thin_study.py`, 10 no-buildup HP3 datasets, `trf`) is a lottery,
not a smooth accuracy/cost trade:

| N | 16 | 14 | 13 | 12 | 11 | 10 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| valid / 10 | 10 | 10 | 10 | 10 | 10 | 10 |
| mean RMSE (mm) | 3.574 | 50.704 | 8.327 | **3.142** | 6.532 | 9.935 |
| catastrophic (> 20 mm) | 0 | 1 | 1 | 0 | 0 | 1 |

`G1` gives 455.9 mm at N=14 and 54.1 mm at N=13 while the full data gives 3.40 mm;
`H1` gives 19.2 mm at N=10. N=12 and N=16 are clean, but nothing makes the failures
predictable. This is why the `--sweep-points 6` result of section 4.4 (a planned
six-point arc, 2.514 mm at 4868 s) is reported as an option and no point count was
changed: only *planned* fewer-point sweeps were ever validated, and even they cost
accuracy.

## 5. Reproducing

```bash
# full headless run into RUN/<name> (extra CLI flags in RUN/<name>/extra_flags.txt)
tools/run_headless_autocal.sh RUN/<name>
# fit-only replay of a stored dataset, and its oracle over the candidate pool
.venv/bin/python tools/replay.py RUN/<name>/sweeps.json --args "--solve-optimizer trf"
.venv/bin/python tools/oracle.py RUN/<name>/sweeps.json
# sweep-prefix replay: what would have happened if the run had stopped earlier
.venv/bin/python tools/replay_prefix.py RUN/<name>/sweeps.json --n 3,4,5,6,7,8,9
# solve/fit-only comparison of the two solvers on the 11 legacy fixtures
.venv/bin/python tools/refset_solver_compare.py
# clean staged candidate pools (one dataset copy per run) + applied/oracle/rule RMSE
.venv/bin/python tools/stage_clean_pools.py /tmp/clean_pools --jobs 6
.venv/bin/python tools/pool_evidence.py /tmp/clean_pools
# fewer-points decimation study and solver-union check
.venv/bin/python tools/thin_study.py --ns 16 14 13 12 11 10 --jobs 8
.venv/bin/python tools/union_rule.py /tmp/clean_pools
# exact per-command cost accounting
.venv/bin/python tools/phase_cost.py RUN/<name>/sweeps.headless/events.jsonl
```

Session tools live in `tools/`: `batch.sh`, `run_headless_autocal.sh`, `replay.py`,
`replay_prefix.py`, `replay_variant.py`, `replay_args.sh`, `prefix_pool.sh`,
`trf_pool.sh`, `oracle.py`, `truth_score.py`, `phase_cost.py`, `final_table.py`,
`settle_vs_force.py`, `settle_extrap.py`, `history_eval.py`.

## 6. Evidence index

| evidence | what it establishes |
| --- | --- |
| [runs/H1-control](runs/H1-control), [runs/C2](runs/C2-control-b), [runs/C3](runs/C3-control-c), [runs/C4](runs/C4-control-d) | bit-identical repeats; determinism; control 2.706 mm at 5180.8 s |
| [runs/V1-verify-default](runs/V1-verify-default) | shipped default: no flags, 2.410 mm, 5180.8 s, unchanged data |
| [runs/T1-trf](runs/T1-trf) | `--solve-optimizer trf` reproduces 2.410 mm end-to-end |
| [runs/T2-trf-sf091](runs/T2-trf-sf091) | 0.91 N sensor force: 2.016 mm but +353 s |
| [runs/T3-trf-pts6](runs/T3-trf-pts6), [runs/T4-trf-pts6-sf091](runs/T4-trf-pts6-sf091) | 6 points/sweep: mixed; with 0.91 N it is 2.031 mm at 4773 s |
| [runs/G1](runs/G1-sforce068), [runs/G2](runs/G2-sforce091), [runs/G3](runs/G3-sforce136), [runs/B5](runs/B5-sforce220) | sensor-force valley and its steep low side |
| [runs/B1](runs/B1-points6), [runs/D3](runs/D3-points8), [runs/B6](runs/B6-points14), [runs/B2](runs/B2-feed2800), [runs/B3](runs/B3-feed4200), [runs/D4](runs/D4-points4) | feed and point-count knobs: no free win, `points=4` diverges |
| [runs/P1](runs/P1-cap15), [runs/P3](runs/P3-cap6), [runs/P2](runs/P2-cap30) | pull-out caps: worse accuracy or settle-timeout crash |
| [runs/DBG1-debug](runs/DBG1-debug) | `--debug-sweep-actions` trace locating the long pull-out waits |
| [reference run](/home/torbjorn/repos/hp-sim5/output/extended-autocal/no-buildup-reference-003/) | 2.828 mm applied RMSE, 9 sweeps / 180 points, same forces and feed |
| `runs_refset/pool_evidence_clean.txt`, `runs_refset/pool_evidence.json` | clean 22-pool benchmark: target `trf` 3.485 vs `lbfgsb` 4.158 mm; legacy `trf` 27.941 vs `lbfgsb` 16.151 mm; selection regret |
| `runs_refset/refset_solver_compare.txt` | canonical harness replay of the 11 legacy fixtures with each solver (E28) |
| `runs_refset/thin_study_boundary.json` | point decimation N = 10..16 on 10 datasets: no monotone trade, individual 55-456 mm failures (E34) |
| `runs_refset/union_rule_clean.txt` | pooling both solvers' candidates beats neither solver alone (E33) |

## 7. Limitations

- One physics world. Every dataset here comes from the same simulated HP3
  geometry, friction and stiffness, so the `trf` result is one configuration
  sampled in 10 places, not 10 machines. The user-requested wider-world study
  (H10) is still the missing piece; the cheap path is fit-only replay, since the
  pull-out cannot be shortened (section 4.5).
- Simulated, not hardware. Nothing here measures a real Hangprinter, and the
  reference run was recorded on a different (browser) clock, so its 6756 "sim s"
  are not comparable with the 5180.8 headless seconds used here.
- The 0.91 N sensor-force result and the 4x `forceMid` criterion come from one
  world; that is why the change was not shipped.
- The shipped solver default does not generalise to the checked-in legacy fixtures
  (line buildup + fitted radii): `trf` raises their mean applied error from 16.2 to
  27.9 mm (section 4.8). No tested selection rule repairs it, and the mechanism is
  the tightest-pair consensus interacting with `trf`'s more consistent early fits.
- The staged-pool experiments used to screen selection rules had to be rebuilt: the
  first staging harness shared one dataset copy between the two concurrent solver
  runs, and that shared `<dataset>.replay_tmp.json` resume file changed the effective
  dataset (8 source sweeps became 10-12). Only the rebuilt clean pools are reported.
- Three autocal tests fail in this worktree for a reason unrelated to this work
  (`autocal/auto-calibration-simulation-for-hangprinter/util.py` is missing from
  the checkout); they fail identically with the old solver default.

## 8. References

- [Research notebook](research.md): hypotheses, experiment IDs and steering.
- [Searchable abstract](abstract.md).
- Previous session, [faster HP3 autocal by reusing settled measurements](../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/abstract.md):
  quiet-window reuse (2733 -> 2571 simulated s) and the bounded-endpoint work this
  session's E20 extends.
- Previous session, [tightening the quiet window](../2026-10-08-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-c812e7b9/report.md):
  25.4% collection speedup; halving points rejected.
- Previous sessions, [held-out history ranking](../autocal-held-out-history-ranking-7ec4260f/abstract.md)
  and [bounded ranking updates](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/abstract.md):
  ranking metrics disagree with physical accuracy on 22 adjacent pairs over 11
  datasets; the baseline for this session's E23 oracle measurement.
- Previous session, [candidate model validation and throughput](../autocal-candidate-model-validation-and-throughput-fdb616a4/abstract.md):
  spool-radius model mismatch, D-optimal movement mixed.
- [Reference run](/home/torbjorn/repos/hp-sim5/output/extended-autocal/no-buildup-reference-003/)
  (`no-buildup-reference-003`), the target to beat.
- Repository documents consulted: [autocal/README.md](../../../../autocal/README.md),
  [autocal/optimization_algorithm_overview](../../../../autocal/optimization_algorithm_overview),
  [autocal/objective_functions_overview](../../../../autocal/objective_functions_overview).
- Code revision: repo HEAD `1404c5ac` plus the `--solve-optimizer` default change
  in `autocal/_autocal_common.py`.
- Session tools used for the compatibility and few-points work:
  `tools/stage_clean_pools.py` (one dataset copy per run),
  `tools/pool_evidence.py` (applied/oracle/rule RMSE), `tools/union_rule.py`,
  `tools/thin_study.py`, `tools/refset_solver_compare.py`, `tools/harness_probe.py`,
  `tools/cmd_probe.py`, `tools/conc_probe.py`.
- [regress_calibration_logs.py](../../../../autocal/tools/regress_calibration_logs.py)
  and [autocal/data/references/README.md](../../../../autocal/data/references/README.md):
  provenance of the 11 legacy fixtures.

## 9. Main-checkout review (b56d08e4)

The reported implementation was based on `1404c5ac`, before the current main
checkout's consensus bound and compliance-correction changes. To check whether
those changes remove the legacy risk, the three largest reported regressions
were replayed on main with isolated per-solver dataset copies. Main still strongly
prefers `lbfgsb` on all three:

| dataset | `lbfgsb` anchor RMSE | `trf` anchor RMSE |
| --- | ---: | ---: |
| `sixteenth_hp3_dataset_even_more_pressure` | 12.209 mm | 71.367 mm |
| `seventh_hp3_dataset` | 56.982 mm | 85.659 mm |
| `23_april_2026` | 27.428 mm | 73.473 mm |
| **mean of these three** | **32.206 mm** | **76.833 mm** |

These results use the current main code and therefore include its additional
history behavior. They confirm that the recent main commits do not make a global
`trf` default safe. The default change and its documentation edits were not
imported. `trf` remains available through `--solve-optimizer trf`; the worktree
evidence is useful for choosing that option in its tested HP3/no-buildup regime.
The reproducing table is [main_b56d08e4_solver_compare_subset.txt](runs_refset/main_b56d08e4_solver_compare_subset.txt).

The research worktree's [regression replay](evidence/regression_run_trf_default_vs_reference.log)
also compared its `trf` default against historical full-auto logs. Seven fixtures
improved and four worsened, but the sum of final applied true-error deltas was
**+245.759 mm worse**. Its `true_iter_mean_delta` sum was -8578.270 mm. This metric
must remain in the evaluation: it averages all solver iterations, including
extreme failed fits, and a large negative delta is evidence that `trf` avoids some
catastrophic intermediate failures. It measures a real reliability property even
when consensus later discards those candidates. It should be read alongside final
applied error and tail counts, since the legacy fixtures show that safer average
iterations do not guarantee a better applied answer. For example,
`sixteenth_hp3...` had a -1050.688 mm iteration-mean delta while its final applied
error was +192.443 mm worse. The main checkout's root
[regression_run.log](../../../../regression_run.log) is a separate same-solver
historical comparison and has zero iteration-mean delta in all fixtures; it is
not a solver A/B measurement.

## 10. Paired solver comparison across 10 simulated HP3 worlds

To test whether the solver result survived changes in simulated conditions, the
autocal code at main revision `b56d08e4` collected one dataset in each of 10 HP3
scenes. Main later advanced to `6f0ba5cd`, which changes only a research prompt
sentence and does not touch autocal code.
The same geometry and firmware were used throughout; each scene changed the three
mover masses together and/or gravity (0.80-1.25x mover mass; 8.5-11.0 m/s²).
Every collection used no line buildup, `--buildup-factor 0`, and
`--find-radii off`. Each resulting dataset was then copied into separate
`lbfgsb` and `trf` replay folders. All 20 replay input copies matched their source
dataset SHA-256. The solver evaluations used the same current-main code and
recorded up to six active collection steps after bootstrap; no collection data
were shared between separate processes.

| world | mass x | gravity | sweeps | applied RMSE lbfgsb | applied RMSE trf | per-world iteration mean lbfgsb | per-world iteration mean trf | max iteration lbfgsb / trf |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| nominal | 1.00 | 9.82 | 9 | 2.509 | 2.468 | 3.536 | 3.614 | 7.379 / 7.379 |
| light | 0.80 | 9.82 | 9 | 2.706 | 2.696 | 3.701 | 3.698 | 7.164 / 7.164 |
| heavy | 1.20 | 9.82 | 7 | 2.698 | 2.683 | 3.156 | 3.328 | 3.731 / 3.731 |
| low gravity | 1.00 | 8.50 | 7 | 3.208 | 2.861 | 3.122 | 3.136 | 4.553 / 4.542 |
| high gravity | 1.00 | 11.00 | 7 | 2.932 | 2.816 | 4.533 | 4.491 | 10.488 / 9.880 |
| light, low gravity | 0.85 | 8.80 | 7 | 3.531 | 3.531 | 3.296 | 3.295 | 3.961 / 3.960 |
| light, high gravity | 0.85 | 10.80 | 8 | 3.072 | 3.032 | 2.899 | 2.771 | 3.255 / 3.086 |
| heavy, low gravity | 1.15 | 8.80 | 7 | 2.285 | 2.310 | 3.719 | 2.969 | 6.384 / 4.584 |
| heavy, high gravity | 1.15 | 10.80 | 7 | 3.689 | 3.218 | 5.062 | 4.703 | 11.059 / 9.269 |
| heavy, high load | 1.25 | 10.50 | 7 | 3.035 | 3.125 | 2.995 | 3.373 | 3.140 / 4.711 |
| **mean across worlds** | | | | **2.967** | **2.874** | **3.602** | **3.538** | |

Applied RMSE is lower for `trf` in 7 worlds, lower for `lbfgsb` in 2, and tied
in 1. The per-world all-iteration mean is lower for `trf` in 6 worlds and lower
for `lbfgsb` in 4; the mean paired delta (`trf - lbfgsb`) is -0.064 mm. The
combined candidate pools contain 55 `lbfgsb` and 53 `trf` candidates because
the solvers take different iteration paths. Across those candidate records,
mean RMSE is 3.590 vs 3.545 mm, median is 3.088 vs 3.129 mm, and maximum is
11.059 vs 9.880 mm. Neither solver produced a candidate above 20 mm (or above
100/500 mm) in any of these worlds.

Interpretation: this is a small, repeatable accuracy edge for `trf` under these
mass/gravity perturbations, and its worst candidate was less bad. It is not
evidence that `trf` prevents extreme failures: there were no extreme failures
for either solver in this matrix. Nor does it justify a global default while
the current-main legacy replays still strongly favor `lbfgsb`. It supports
keeping `trf` as an explicit option for no-buildup HP3 and calls for additional
worlds that stress noise, line dynamics, and poor initial fits before changing
the default.

The reproducible matrix summary, per-world scenes/datasets, logs, and compressed
candidate records are in
[`environment-matrix-20261015-retry1/`](environment-matrix-20261015-retry1/).
The parser rebuilds iteration metrics from typed stage arrays with
[`summarize_environment_matrix.py`](tools/summarize_environment_matrix.py);
`run_environment_matrix.py` contains the collection and paired replay harness.

## 11. E36: user regression review of per-restart TRF fallback

Commit `bdd28703` added a default-on fallback that retries every unsuccessful
L-BFGS-B restart with TRF. Reviewing the user's existing seventh-dataset regression
logs at `5290851e` explains why that policy improved iteration error while worsening
the applied answer. This is an analysis of recorded simulated fixtures; no new
simulation was run for this review. The on/off source dataset SHA-256 values match.

| seventh_hp3_dataset | fallback off | fallback on |
| --- | ---: | ---: |
| Final summed anchor distance to simulation truth (mm) | 197.410 | 257.120 |
| Final anchor plus radius error (mm) | 202.047 | 261.757 |
| All-iteration mean anchor plus radius error (mm) | 1423.587 | 221.769 |
| Iterations | 5 | 4 |
| Closest pair spread (mm) | 23.52 | 0.578 |
| Applied selection | best-ranked later fit | iterations 1 and 2 consensus |

The regression runner's paired `true_iter_mean_delta` (on minus off, four matched
iterations) is -1501.281 mm. This is a real reduction in severe intermediate
failures. The final summed anchor error nevertheless worsens by 59.710 mm: the
fallback changes the first fit from extreme failure to nearly the same biased
answer as iteration 2. That creates a pair within the 5 mm consensus gate, and
the loop applies the early pair. Without fallback the gate rejects consensus and
the later best-ranked fit is applied. Stopping also changes from five to four
iterations, so unpaired iteration means and the paired delta differ.

Fallback ran 19 times: 17 iteration-limit exits and 2 abnormal terminations.
`success=False` therefore was a much broader trigger than having no usable fit.
The implementation also prefers a converged TRF result over a finite unsuccessful
L-BFGS-B result even when its objective is higher. Neither solver status nor a
lower fitting objective guarantees lower geometric error, and changing the pool
can change consensus discontinuously. Improvements from running TRF everywhere
do not predict which datasets improve under this conditional policy.

Decision: reject enabling per-restart fallback by default. Keep
`--solve-optimizer trf` and opt-in `--trf-solver-fallback`; the CLI now defaults to
`--no-trf-solver-fallback`. The 37 CLI tests pass, including the default and both
flag forms. Those tests verify the configuration contract, not calibration quality.

Proposals, not tested: a hard-failure rescue only after *all* L-BFGS-B restarts
have no usable candidate could improve completion reliability without replacing
finite nonconverged fits. To improve accuracy when solvers disagree, choose a
fresh sweep where their predicted measurements differ, then compare against that
independent data. This would require new collection and selection experiments;
neither proposal is shipped here.

Evidence: [review.json](evidence/fallback-policy-review/review.json) records the
original log paths, hashes, trigger counts, selections and metrics computed using
`autocal/tools/regress_calibration_logs.py`. Both console logs and JSONL traces
are copied beside it.
