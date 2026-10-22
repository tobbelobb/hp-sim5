# Research record: faster and more robust encoder-based CDPR autocalibration (HP3, no layering)

## Main-checkout follow-up: environment variation

User steering (2026-10-15): do not treat extreme failed fits as a metric artifact;
avoiding them is a key solver reliability outcome. Broaden `lbfgsb` vs `trf` to
about 10 simulated HP3 datasets made under varied physical environments.

- Objective: compare both solvers on the exact same collected dataset in each
  world, reporting final applied geometry plus all candidate/iteration failures
  and tail severity.
- Hypothesis: the scoped `trf` accuracy and reliability advantage persists when
  the simulated mover load changes.
- Variation: keep the HP3 geometry, firmware, collection flags, no layering,
  buildup zero and radii fitting off; vary physical mover mass and gravity in the
  headless USD scene. This changes load/dynamics while holding calibration truth
  and fitting model fixed.
- Planned set: 10 worlds spanning nominal, lighter/heavier mover masses, lower/
  higher gravity and combined mass/gravity conditions. Each world is collected
  once; paired solver replays use isolated copies of the same saved dataset.
- Budgets: 10 environment collections, at most 6 full-auto iterations after the
  3-sweep bootstrap, 10 points per sub-sweep, no RRD recording. Stop if collection
  stability or wall-time makes the matrix unreasonable; preserve any failures.
- Metrics: final applied anchor RMSE, mean/median/worst per-iteration candidate
  RMSE, counts over 20/100/500 mm and largest observed error. Keep raw logs and
  compact candidate summaries. Do not average away solver failures.
- Current status: design complete; collections not yet run. See
  `tools/run_environment_matrix.py` and `environment-matrix/` for the harness and
  outputs once started.

## Objective

Make encoder-based autocalibration in hp-sim5 faster, more robust and/or better at
selecting anchors, while preserving the one-click experience and automatic stopping.
Beat the reference run `output/extended-autocal/no-buildup-reference-003`
(applied-anchor RMSE **2.828 mm** against the simulated HP3 truth).

## Constraints and budgets

- HP3 (`--machine-type hp3`), line layering disabled, `--buildup-factor 0`,
  `--find-radii off`, `--find-buildup-factor off`.
- Simulated truth: `A(0,-1900,-280) B(1645.44826719,950,-280)
  C(-1645.44826719,950,-280) D(0,0,1900)` mm; `mm_per_degree = 0.5235988`.
- Speed is judged in **simulated seconds** the collector asks the simulator for
  (headless `--recording-mode none`), plus wall seconds. The reference run's own
  6756 "sim s" are measured on a browser clock and are not comparable.
- Accuracy is applied-anchor RMSE vs truth (all six/ten runs are deterministic,
  see E03, so single runs are valid A/B comparisons).
- Fit-only replay (`tools/replay.py`, `tools/oracle.py`) is the cheap path used
  for hypothesis screening before spending simulated seconds.

## What dominates a headless run (exact cost model, E01)

`tools/phase_cost.py` and `/tmp/longcmds.py` on `runs/H1-control` (default flags,
9 sweeps, 180 points, **5180.8 sim s**):

| block | sim s | share |
| --- | ---: | ---: |
| per-sweep pull-out waits (13 commands of 73.5-100 s) | 1097 | 21% |
| force tuning pre-sweep (`M569.4` ramps + releases) | ~1240 | 24% |
| per-point `G1` quiet/settle (180 points x ~7 s) | 1257 | 24% |
| `G1` sweep travel (28.4 m commanded) | 1217 | 23% |
| micro-moves, releases, other modes | ~370 | 8% |

Inside the force-tuning block: 11 bracket probes ~25-27 s each (bounded by
`AUTO_TUNE_STALL_WINDOW_MS = 25000`), 2 bisect probes ~28 s, 11 edge-ramp trials
`30.5 s` each (`AUTO_TUNE_SAMPLE_WINDOW_MS * 3`), and 6 force-release
`M569.4` commands that grow 8 -> 40 s as the spooled-out length grows.

## Hypotheses

| ID | Hypothesis | Status |
| --- | --- | --- |
| H1 | Sweep travel feed rate can be raised without degrading accuracy. | Rejected (feed 2800 neutral, 4200 worse) |
| H2 | Lowering the data-collection sensor force improves anchors. | Rejected as stated; non-monotone in force (E21) |
| H3 | Per-point encoder-noise sampling can be shortened. | Not tested (<=0.4 s/point ceiling) |
| H4 | Fewer points per sub-sweep gives equal accuracy in less time. | Rejected with lbfgsb; sublinear time saving |
| H5 | The pull-out settle can be bounded without changing collected data. | **Rejected** (E20) |
| H6 | The selection score disagrees with true geometric accuracy. | Supported but small in practice (E01-lab, E23) |
| H7 | Stopping 1-3 sweeps earlier is uniformly safe. | Rejected (inconsistent) |
| H8 | Force-tuned min/mid/max forces are the dominant accuracy knob. | Mixed: sensor force matters, max force not yet swept |
| H9 | Divide-and-conquer sub-experiments can isolate parameters. | Not tested |
| H10 | Many plausible physical worlds are needed to judge robustness. | Not tested (cost) |
| H11 | `lbfgsb` multi-start lands in different local optima, so the reported anchors depend on solver luck; a trust-region solver is more robust. | **Supported** (E23, E24) |
| H12 | The two sub-sweep directions of one arc are geometrically interchangeable. | Rejected: identifiability differs strongly (E22) |
| H13 | Consensus spread (spread of the two closest fitted anchor sets) is a usable convergence signal for early automatic stopping. | Supported as a signal (E05, E23); stop rule not yet implemented |

## Experiment log

| ID | What | Status |
| --- | --- | --- |
| E01 | Reference audit + cost model of a headless run. | Done |
| E02 | Knob sweeps (feed 2800/4200, points 4/6/8/14, sensor force 0.35/2.2, pull-out caps 6/15/30 s). | Done |
| E03 | Determinism: 4 repeats of the control flag set. | Done: bit-identical |
| E05 | Sweep-prefix replay: applied RMSE vs number of collected sweeps + consensus spread. | Done |
| E20 | Pull-out cap experiment (`AUTOCAL_PULLOUT_CAP_MS` 6/15/30 s). | Done: rejected |
| E21 | Sensor-force sweep 0.68/0.91/1.36 N full headless runs. | Done: rejected |
| E22 | Point-subset replay (direction 1 / direction 2 / even / odd steps). | Done |
| E23 | Fit-side battery on the control dataset (restarts, iterations, optimizer, mode) + oracle. | Done |
| E27 | Sweep-prefix replay over the whole dataset pool (with `trf`) to size an early-stop rule. | Done |
| E24 | `lbfgsb` vs `trf` over 10 recorded datasets (fit-only). | Done |
| E25 | Full headless runs with `--solve-optimizer trf` (+ 6-point and 0.91 N variants). | Done |
| E26 | Implement the change, re-run autocal regressions, verify end-to-end. | Done |

## Findings so far

- **E03** `C2/C3/C4` reproduce `H1` bit-identically (5181 sim s, 2.706 mm, identical
  M669); `D1/D2` reproduce `B1`. The pipeline has no run-to-run noise.
- **E05** On `H1`, replaying the first *k* collected sweeps gives
  k=3/4/5/6/7/8/9 -> 7.381/5.111/5.111/5.111/**2.706**/2.706/2.706 mm with
  consensus spread 0/9.7/9.7/9.7/**2.204**/2.204/2.204. Two of the nine sweeps
  are pure waste, and the spread identifies exactly where the answer stabilises.
  (Prefixes are faithful because the collector's next target depends only on
  decisions already made, and those were "collect" all the way to the stop.)
- **E20** Cutting the pull-out wait loses the endpoint by 435 mm (5 s cut) /
  271 mm (15 s) / 96 mm (30 s); extrapolating the creep makes it worse, not
  better. The pull-out wait is bounded below by physics (spool-out + 2 pi r turns).
- **E21** `--sensor-collection-force` 0.68/0.91/1.36/2.2 N -> 9.963/2.609/3.824/6.289 mm
  vs 2.706 control. Non-monotone; 0.91 N is mildly better, and the force tuning
  output itself is identical for every sensor force.
- **E22** `dir1` (drive=B) alone 72.3 mm, `dir2` (drive=C) alone 3.41 mm, both
  directions 2.706 mm; truth-anchor residual is the same for both directions
  (14.56 / 15.03). The asymmetry is an *identifiability/fit* effect, not a data
  quality effect: reducing to one direction per sweep makes the fit land in
  different optima. Even/odd step thinning (90 points) gave 5.103 / 2.641 mm, so
  half-arc variants are dominated by the same fit fragility.
- **E23** On `H1` the pool of seven fitted anchor sets has oracle (best single
  candidate) RMSE 2.385 mm while the applied consensus is 2.706 mm, so *selection*
  costs only 0.32 mm here; the dominant term is the common error of the whole pool.
  Fit-side battery (applied RMSE): `lbfgsb` 2.706, `trf` **2.410**, `lm` 2.423,
  `lbfgsb -R 16` 2.736, `trf -R 16` 2.410, `trf -i 800` 2.410, `trf fast-fd` 2.410.
- **E24** Fit-only replay over 10 recorded datasets, applied RMSE
  (`lbfgsb` -> `trf`): H1 2.706->2.410, reference 2.623->2.409, G2 2.609->2.054,
  G3 3.824->2.823, B6 2.969->2.614, D3 3.184->3.163, B5 6.289->5.287,
  G1 9.963->**3.403**, B1 2.192->2.527, P1 7.118->9.206.
  Mean 4.348 -> 3.590 mm (-17%), worst case 9.963 -> 9.206 mm. `trf` also gives
  much tighter consensus spreads (0.06-1.5 mm vs 2.2-32 mm), i.e. the restarts
  agree instead of scattering. `lm` is worse than `trf` (2.423/1269.8/2.19).
- All of this is **simulated** evidence from one physics world at a time; nothing
  here is hardware-measured. `trf` correctness rests on 10 datasets from a single
  world configuration, which is exactly the H10 weakness.

## Accepted steering

- 2026-10-15 (user): prioritise faster/robust collection with fewer or
  easier-to-collect points; no layering, no buildup, no radii; HP3; beat
  `no-buildup-reference-003`; keep one-click UX and automatic stopping.
- 2026-10-15 (user, issue list): profile the recording; treat force selection as
  the main accuracy knob; broaden world variety (needs cheaper simulation);
  divide-and-conquer the parameter search.

## Next decision

Read out E25 (full headless `trf` runs: sim seconds, sweeps, applied RMSE).
If `trf` end-to-end reproduces the >=10% accuracy gain at equal or lower sim
seconds, make `trf` the default anchor solver, re-run the autocal test suite,
and re-run the control flag set for a matched comparison.

## E25/E26 results (implementation turn)

Full headless runs (`--recording-mode none`, HP3, layering off, buildup 0):

| run | flags | sweeps | points | sim s | applied RMSE |
| --- | --- | ---: | ---: | ---: | ---: |
| `H1-control` | default (`lbfgsb`) | 9 | 180 | 5180.8 | 2.706 |
| `T1-trf` | `--solve-optimizer trf` | 9 | 180 | 5180.8 | **2.410** |
| `T2-trf-sf091` | `trf` + sensor 0.91 N | 9 | 180 | 5533.8 | **2.016** |
| `T3-trf-pts6` | `trf` + 6 points/sweep | 9 | 108 | 4868.3 | 2.514 |
| `T4-trf-pts6-sf091` | `trf` + 6 points + sensor 0.91 N | 8 | 96 | 4773.0 | **2.031** |
| `V1-verify-default` | **no flags** (patched default) | 9 | 180 | 5180.8 | **2.410** |

- `V1-verify-default` proves the shipped default reproduces `T1-trf` exactly:
  same 5180.8 sim s, same 2.410 mm, same force tuning
  (`idle=0.0114 start=0.2273 edge=3.3540 d_max=1677.770deg`), same consensus
  (`members=[5,6] spread_mm=0.06416`) and the same `truth_score` (14.7953) as
  `H1-control`, i.e. **the collected data is unchanged and only the fit changed**.
- Fit CPU on the `H1` dataset (single replay, `--no-collect`): `lbfgsb`
  19.4 s wall / 67.2 s CPU vs `trf` 14.4 s wall / 16.2 s CPU.
- Extra simulated time of the 0.91 N sensor force is per-point settle time
  (`g1-step quiet` 1257.0 -> 1607.1 s in `T2`); the pull-out and force-tuning
  blocks are unchanged.
- Tests: `345 passed`, plus 3 **pre-existing** failures caused by the empty
  legacy checkout `autocal/auto-calibration-simulation-for-hangprinter/`
  (`util.py` missing); they fail identically with the old default.
- E27 (prefix replay with `trf`): the consensus spread identifies when the answer
  stops moving (`H1`: 9.7 mm at 4-6 sweeps -> 2.27 at 7 -> 0.064 at 8, and the
  answer is final from 8 sweeps), but a spread-threshold stop saved a sweep on
  only 2 of 11 datasets, so it was not implemented.

## Implementation shipped

`autocal/_autocal_common.py`: `--solve-optimizer` default `lbfgsb` -> `trf`
(+ help text); docs updated in `autocal/README.md`,
`autocal/README_elliptical_feature_calibration.md`,
`autocal/optimization_algorithm_overview`;
`autocal/tests/test_autocal_cli.py` default expectation updated.
No collection code was touched, so the stored dataset format and all collection
regressions are unchanged.

## Continuation turn: does the shipped default generalise? (E28-E34)

### E28 - Canonical replay of the 11 legacy fixtures: `trf` regresses
`tools/refset_solver_compare.py` replays every `autocal/data/references/*.json`
through `autocal/tools/regress_calibration_logs.py` (the harness that owns the
checked-in `.log` fixtures) with `lbfgsb` and with `trf`.

| dataset | `lbfgsb` | `trf` | win |
| --- | ---: | ---: | --- |
| `five_points_bigger_deltas` | 4.096 | 4.096 | tie |
| `flexible_lines_2000` | 8.067 | 8.067 | tie |
| `layered_manual_some_bad_points` | 1.758 | 1.758 | tie |
| `ten_points_bigger_deltas` | 0.868 | 0.868 | tie |
| `ten_points_w_buildup` | 1.059 | 1.059 | tie |
| `ten_points_w_buildup2` | 1.439 | 1.439 | tie |
| `fifth_hp3_dataset` | 18.424 | 18.423 | tie |
| `nineteenth_hp3_dataset_even_more_pressure` | 43.055 | 41.908 | `trf` |
| `seventh_hp3_dataset` | 59.585 | 85.659 | `lbfgsb` |
| `sixteenth_hp3_dataset_even_more_pressure` | 11.878 | 71.367 | `lbfgsb` |
| `23_april_2026` | 27.428 | 72.698 | `lbfgsb` |
| **mean** | **16.151** | **27.940** | `trf` 2, `lbfgsb` 3, tie 6 |

These fixtures are Slideprinter/HP4 recordings with line buildup and
`--find-radii global`, i.e. outside this session's scoped regime, but they are the
suite the shipped default must not silently break.

### E29 - Root cause: tightest-pair consensus + `trf`'s consistent early fits
`autocal.py::_select_history_summary_candidate` applies
`history_selection.py::consensus_anchor_estimate`, the **mean of the two closest**
fitted anchor sets. For `sixteenth_hp3_dataset_even_more_pressure` the `trf` pool
applies the iterations-1/2 mean (z = -244.5 mm against a true -280 mm) while its
iterations 3-5 sit near -279.5 mm; `lbfgsb`'s noisier pool breaks that symmetry.
Selection, not solver quality, causes the three regressions.

### E30 - Every alternative selection rule is worse (clean pools)
`tools/stage_clean_pools.py` + `tools/pool_evidence.py`; the offline recomputation
reproduces each run's own applied anchors to `<= 0.004 mm`, so the screen is valid.
Mean applied RMSE over 11 pools per cell (`current` = shipped tightest-pair rule):

| group | solver | current | oracle | last3 | last4 | swhalf | swmax | topk3 | best |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| target | `trf` | **3.485** | 3.218 | 3.61 | 3.49 | 3.45 | 3.57 | 5.08 | 4.40 |
| target | `lbfgsb` | **4.158** | 2.863 | 6.02 | 7.28 | 7.28 | 8.93 | 4.41 | 5.05 |
| legacy | `lbfgsb` | **16.151** | 13.929 | 16.95 | 16.85 | 16.85 | 30.01 | 19.13 | 20.38 |
| legacy | `trf` | 27.941 | 15.910 | **24.30** | 27.37 | 31.02 | 235.2 | 31.14 | 34.03 |

Only `last3` on legacy `trf` beats its current rule, and it is catastrophic on
`fifth_hp3_dataset` (18.4 -> 108.5 mm), where the current rule is fine. The
alternatives are also catastrophic per-dataset elsewhere (H1 `lbfgsb` `swmax`
44.7 mm). Rejected: the tightest-pair rule is fragile in both directions and no
re-ranking tested here generalises across the two regimes.

### E31 - Harness validity: the first staging script cross-contaminated its pools
`tools/gen_artifacts.py` gives the two solver runs **one shared dataset copy**.
Concurrent replays then append into the sibling `<dataset>.replay_tmp.json` resume
file: a controlled two-in-one-directory run of
`sixteenth_hp3_dataset_even_more_pressure` ended with 10 (`lbfgsb`) and 12 (`trf`)
sweeps where the source has 8, and returned 12.793 / 12.471 mm instead of the
canonical 11.878 / 71.367 mm (`tools/conc_probe.py`, `tools/harness_probe.py`,
`tools/cmd_probe.py`). Isolated or sequential runs end at exactly the source sweep
count and reproduce the reference harness exactly. Decision: all legacy/target pool
numbers come from `tools/stage_clean_pools.py` (one dataset copy per run) and the
`/tmp/arts_a|b` numbers are discarded. Side product finding: two autocal processes
replaying the same dataset file interfere through that resume file.

### E32 - Target-regime clean pools confirm `trf`
Same clean benchmark restricted to the 11 no-buildup HP3 recordings: `trf`
3.485 mm (oracle 3.218, regret 0.267) vs `lbfgsb` 4.158 mm (oracle 2.863, regret
1.295) - a 16% mean gain, plus much lower sensitivity to the selection rule
(mean rule spread 2.4 vs 7.0 mm; 0 vs 3 pools above 20 mm in the legacy group).
This reproduces the E24/E26 ranking with an independent harness.

### E33 - Pooling both solvers is not a both-regimes fix
`tools/union_rule.py`: union consensus legacy 22.39 mm (20.56 with a data-volume
floor) is worse than `lbfgsb` alone (16.15); target union 4.27 (3.57) is worse than
`trf` alone (3.485). Rejected.

### E34 - Decimation of points per sweep is a lottery
`tools/thin_study.py`, 10 no-buildup HP3 recordings, `trf`, evenly spaced subset of
each sweep's `data_points`: N=16 mean 3.574 mm, N=14 50.704 (G1 455.9), N=13 8.327
(G1 54.1), N=12 **3.142**, N=11 6.532, N=10 9.935 (H1 19.2). No monotone trade and
unpredictable blow-ups, so decimation is rejected as a speedup; the earlier
`--sweep-points 6` end-to-end result (2.514 mm / 4868 s, and 2.031 mm / 4773 s with
a 0.91 N sensor force) remains the only validated fewer-points configuration and was
not shipped.

### Hypotheses updated

| ID | Hypothesis | Status |
| --- | --- | --- |
| H14 | `trf` generalises from the no-buildup HP3 regime to the legacy buildup/radii fixtures. | **Rejected** (E28/E29): legacy mean 16.2 -> 27.9 mm |
| H15 | Fewer points per sweep (by decimation) stays accurate. | **Rejected** (E34): individual datasets blow up by 5-450 mm |
| H16 | A different pool-selection rule repairs the solver's weak cases. | **Rejected** (E30/E33): current rule best in 3 of 4 cells; union no fix |
| H17 | Wider-world robustness can be screened from recorded datasets plus clean staged pools instead of new worlds. | Supported only as a screen (E30/E32): it reproduces solver rankings, not new physics |

### Final decision in the research worktree

The worktree changed the default to `trf`, based on the scoped HP3 result
(no layering, `--buildup-factor 0`, `--find-radii off`): -17% fit-only mean, -16%
on clean pools, and 2.706 -> 2.410 mm end-to-end at identical 5180.8 simulated
seconds and byte-identical collected data. This decision did not survive review
against the newer main checkout. Its three worst legacy fixtures still strongly
prefer `lbfgsb` (mean 32.206 vs 76.833 mm; `report.md` section 9), and the
worktree regression replay's final applied error sum is +245.759 mm worse despite
very favorable iteration-mean diagnostics. Do not import the global default
change. Keep `trf` as an explicit option for the tested regime; cross-world
robustness remains open. The 0.91 N sensor force (2.016 mm, +353 s on one world)
and the `--sweep-points 6` (+0.91 N) option (2.031 mm, 4773 s) also remain unshipped.

### Proposed but not tested

- A robust consensus rule for the fragile cases: cluster the pool with a
  *relative* radius (rather than "two closest members") and require a minimum
  cluster support, then validate it on worlds beyond the 11 legacy fixtures.
- Force-tuning-aware sensor force (peak compliance x a fitted constant instead of
  `5 x forceMid`), evaluated on several worlds.
- Cheaper multi-world simulation, which is what would let any of this be judged for
  real-world transfer.

### E35 - Paired solver replays across ten simulated HP3 worlds

User steering emphasized that `true_mean_iter_delta` matters because every
iteration is user-visible risk exposure: extreme failed fits should count even
when the final consensus discards them. Keep the metric and report it beside final
applied error and tail counts. The earlier regression log's -8578.270 mm sum is
not noise or an irrelevant diagnostic; it indicates `trf` avoided several very
large failed fits, although final applied errors still worsened in aggregate on
that legacy set.

To broaden the data beyond one physics world, ran current-main paired replays on
ten once-collected datasets. The scenes varied all three mover masses together
(0.80-1.25x) and gravity (8.5-11.0 m/s²), keeping HP3 geometry/firmware and the
no-buildup/radii-off regime fixed. Each dataset was copied into isolated solver
directories and SHA-256 checked. The resulting data contain 7-9 sweeps per world.
Applied RMSE favored `trf` in 7 worlds, `lbfgsb` in 2, and tied in 1 (mean 2.874
vs 2.967 mm). Per-world iteration means favored `trf` 6-4, with a paired mean
delta of -0.064 mm; candidate maxima averaged 9.880 vs 11.059 mm. However, neither
solver had any candidate above 20 mm in these worlds. So E35 supports a small
accuracy improvement under these perturbations, but cannot validate extreme-fit
avoidance. It does not overturn the main-current legacy regressions. Full results
and caveats are in `report.md` section 10; compact artifacts are in
`environment-matrix-20261015-retry1/`.

The first attempt was an import-path failure before physics started; those logs
remain in `environment-matrix-20261015/`. The corrected run is the retry1 matrix.

### E36 - User regression rejects default-on per-restart fallback

Accepted steering: investigate why fallback improves iteration mean but worsens
seventh_hp3_dataset, and default it off if no supported improvement is available.
Read the user's existing on/off logs (identical source dataset hashes), without
new simulation. The on run retries 19 times: 17 iteration-limit exits, 2 abnormal
terminations. It removes the extreme first fit but makes iterations 1/2 agree at
0.578 mm spread. Consensus then selects their biased answer. The off run's closest
pair is 23.52 mm apart, so the 5 mm gate keeps a later best-ranked fit. Applied
summed anchor error worsens 197.410 -> 257.120 mm; paired iteration-mean total error
improves by 1501.281 mm. Both effects matter; better iterations do not imply a
better selected answer. Iteration count changes from five to four as well.

Decision: keep fallback as opt-in and retain direct TRF selection. Source change
only flips the CLI default, updates documentation and covers the flag contract;
37 CLI tests pass. Preserve original console/JSONL logs and parsed metrics under
`evidence/fallback-policy-review/`, included in `.worktreeinclude`. Report section
11 discusses narrower hard-failure rescue and disagreement-driven fresh sweeps
as proposals, not tested changes. No additional collection budget used.
