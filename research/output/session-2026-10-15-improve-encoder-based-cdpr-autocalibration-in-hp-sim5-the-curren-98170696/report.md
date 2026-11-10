# Warm-starting the HP3 encoder autocalibration anchor solve (and a negative result on point thinning)

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

## Question and method

Can HP3 encoder autocalibration deliver robust anchors from fewer or faster-to-collect data
points, keeping the one-click job and automatic stopping?

Steps: audit the supplied reference (9 sweeps x 20 points = 180 points, 573 s wall, 2.62 mm RMSE
against simulated truth) against simulated truth; profile where wall time goes; test point
thinning and cold-start repairs offline with the reference run's frozen fit options; validate the
surviving idea through the *production* loop (replay and live supervised jobs). Point halving uses
`--sweep-points 5`, i.e. 5 points per sub-sweep and 10 points per sweep, against the collector
default of 10 per sub-sweep.

## Results

### Candidate score is not an accuracy proxy (E01)

On the reference's own stage artifacts the lowest `rank_score` (fit-001, 3 sweeps, 0.1835) has the
worst geometry (8.00 mm RMSE) and the best geometry (fit-007, 9 sweeps, 2.53 mm) has a worse score
(0.2359); selection regret is 5.47 mm. One candidate reported the finite sentinel cost 100.0 with
1457.8 mm RMSE. Evidence: `experiments/e01_reference_stage_audit.json`.

### Wall time tracks requested simulated settling, not physics (E03)

Of 573 s reference wall time, 364 s (64%) is `sweep pullout: waiting for encoder stability`, 30.5 s
force auto-tune, 30.5 s size-tune travel, and fitting 1.9-3.6 s per iteration. One live unlayered
HP3 sweep with 40 points cost 77.8 s wall / 173158 steps (346 s sim) with only 6.5 s of physics
compute. The simulator advances only while the collector waits, so removing points saves per-point
settle time but not the fixed pullout waits. Evidence:
`native/00403d4402004c09aaac4ccedd97b7c4/86feab22c3934b469686a3c39ae57b5f/manifest.json`.

### Cold thinning is fragile; warm chains are not (E02, E05)

Blind thinning collapses to a 1220.35 mm attractor for `every4`, `ends_mid_3per`, `ends_only_2per`
and `center_only_2per` at every sweep count. `every2` (10 points/sweep) is data-dependent cold -
2.30 mm at 5 sweeps but 2358.5 / 997.9 / 53.95 / 19.08 / 1359.6 mm at 3 / 4 / 6 / 7 / 9 sweeps -
and stable with warm chaining (2.30 / 2.30 / 2.45 / 2.47 mm). `full` becomes stable from 4 sweeps
(3.25 mm). A broken *first* stage cannot be rescued. Evidence:
`experiments/e02_point_subsets.json`, `experiments/e05_warmstart.json`.

### Restart budgets, seed perturbation and seed scaling do not fix cold starts (E07, E08, E09, E12)

`solve_restarts` 16/32/64 does not repair cold fits (`every2@7` stays at 1412/19/1359 mm; `every2@6`
at 96.81 mm); the base RNG is hard-seeded (`np.random.default_rng(0)`). Seed-only and perturbed-seed
arms fail. The cold seed radius is 706.9 mm against a true ~1900 mm anchor radius; scaling the
silhouette to ~2x the observed cable-length span works for some cases (`full@3` 8.00 mm,
`every2@3` 6.45 mm, `every2@9` 2.52 mm, `full@4` 3.25 mm) but not `every2@4`/`every2@6`, and the
factor is empirical. A deterministic ladder of scaled seeds is mixed: it rescues `full@4`
(1457.8 -> 3.25 mm), `every2@3` (2358.5 -> 6.45 mm) and `every2@9` (1359.6 -> 3.41 mm) but also
introduces new wrong answers when an extra start wins on cost (`full@3` 8.00 -> 11.45 mm,
`every2@5` 2.30 -> 6.44 mm, `full@9` 2.53 -> 2.73 mm). Cost is a bad selector across starts.
Evidence: `e07_multistart.json`, `e08_restarts.json`, `e09_length_seed.json`, `e12_scale_ladder.json`.

### Production replay: point-count dependence removed (E10, E11)

Matched replays of the recorded reference dataset through the production loop
(`autocal/autocal.py --sim --no-collect`), cold = unmodified `HEAD`, warm = the change:

| Run | points/sweep | accepted score | anchor RMSE vs truth | max anchor | recovery events | fit wall |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| E10 cold | 20 | 0.1928 | 2.62 mm | 3.81 mm | 1 | 119 s |
| E10 warm | 20 | 0.2009 | 2.52 mm | 2.76 mm | 0 | 74 s |
| E11 cold | 10 | 0.1374 | 4.12 mm | 6.80 mm | 5 | 119 s |
| E11 warm | 10 | 0.2136 | **2.47 mm** | 2.85 mm | 2 | 74 s |

The warm 10-point replay matches the cold 20-point reference geometry with half the data and beats
it on the worst anchor. The cold 10-point run degrades (anchor D 6.80 mm off) although its accepted
*score* is the best of the four - the E01 disagreement again. Evidence:
`experiments/e10_replay_warm/`, `experiments/e11_halfpoints_replay/`.

### Live one-click jobs (E13, E14, E15)

Three supervised isolated jobs, HP3, unlayered, base radii 30 mm, buildup 0, `recording_mode=events`,
`--optimizer-mode fast`, identical budgets except the point density. Source provenance is recorded in
each `job.json` (`autocal_source_sha256`).

| Run | points/subsweep | code | sweeps | points | iterations | wall | accepted RMSE | max anchor | recoveries |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| E13 | 5 | warm | 7 | 70 | 5 | 134.6 s | 4.25 mm | 6.50 mm | 1 |
| E15 | 5 | cold | 9 | 90 | 7 | 236.7 s | 4.25 mm | 6.50 mm | 1 |
| E14 | 10 | warm | 9 | 180 | 7 | 349.8 s | 2.42 mm | 2.74 mm | 0 |

E13/E15 is a matched A/B: same machine, world configuration, budgets and code path, differing only
in the 11-line change. Both accepted the identical final anchors
(`A0.00:-1903.40:-280.41 B1648.11:951.53:-280.41 C-1647.98:951.47:-280.41 D-0.05:0.11:1906.50`),
because the unmodified loop wasted iterations on failed cold fits (per-iteration costs 100 sentinel /
46.8 at iterations 2, 3, 5 with 823-2353 mm anchor error) and therefore had to collect two more
sweeps before patience expired. The change made iteration 2 a good fit (cost 0.1432, 3.80 mm), so
patience expired after 5 iterations: **identical calibration, 22% fewer points and 43% less wall
time**. Per-iteration detail: `experiments/e13_live_iterations.json`,
`experiments/e13_live_runs.json`.

E13 (half points) against E14 (default points), both warm: halving the density costs 61% of points
and 62% of E14's wall time, but final accuracy drops from 2.42 to 4.25 mm, mostly at anchor D
(6.50 mm). A first reading blamed the stopping rule (patience expired after 5 iterations / 7 sweeps
against 7 / 9). E16 and E17 now reject that reading: cold E15 ran the half-density job to 9 sweeps
and accepted the *same* 4.25 mm anchors, an offline warm chain on the same 90 collected points
reaches 3.84 mm, and an offline warm chain on 90 points *thinned out of the default collection*
reaches 2.53 mm. Half-density collection therefore produces worse data at the same point count, not
merely less data.

### Thinning a collection is free; thinning the collection is not (E16)

Offline warm/cold chains over sweep prefixes of the three live datasets and the reference dataset,
using the reference run's frozen fit options and the simulated M66x geometry as truth
(`experiments/e16_live_thinning.py`, `experiments/e16_live_thinning.json`). True-anchor RMSE (mm) at
5 / 7 / 9 sweeps:

| Source | points/sweep | variant | warm 5 / 7 / 9 | cold 5 / 7 / 9 |
| --- | ---: | --- | --- | --- |
| reference (browser) | 20 | full (180 pts) | 3.25 / 3.25 / 2.51 | 3.25 / 3.88 / 2.53 |
| reference | 20 | every2 (90 pts) | 2.30 / 2.45 / 2.47 | 2.30 / 19.08 / 1359.6 |
| live default (E14 data) | 20 | full (180 pts) | 3.09 / 3.09 / 2.44 | 3.09 / 2.38 / 3.17 |
| live default (E14 data) | 20 | every2 (90 pts) | 2.12 / 2.41 / 2.53 | 295.0 / 27.3 / 5.42 |
| live default (E14 data) | 20 | uniform5 (90 pts) | 2.81 / 2.80 / 2.71 | 1155.6 / 5.33 / 3.20 |
| live sp5 (E13 data) | 10 | full (90 pts) | 3.81 / 3.84 / 3.84 | 1296.9 / 824.5 / 3.84 |
| live sp5 (E13 data) | 10 | uniform3 (54 pts) | 1220.35 at every prefix | 1220.35 at every prefix |

Three findings:

- **Half the points of a standard collection are redundant.** Thinning the live default collection
  to every second point (90 points) tracks the full 180-point curve (2.12-2.53 mm against
  2.44-3.09 mm) while cold chains on the same thinned data collapse (295, 57, 27 mm). This is E05/E11
  reproduced on live data, and it re-confirms that the *fit*, not the point count, was the weak link.
- **Collecting at half density is not equivalent.** The same 90 points collected at 5 points per
  sub-sweep over the same drive ranges plateau at 3.71-3.84 mm no matter how many sweeps are added,
  and the collected 9-sweep dataset is no better than the 4-sweep one. Point count is therefore not
  what limits the half-density job; the *measurement quality* of a coarse drive grid is.
- **Which points you keep matters, and too few is fatal.** A uniform 5-per-sub-sweep selection from
  the default collection is clearly worse than every-second (2.71-3.52 against 2.12-2.53), and 3-4
  points per sub-sweep collapse to the 1220.35 mm sentinel at every sweep count.

### Live matched pair on bit-identical data (E17, E18)

Two more supervised one-click jobs complete the live design: E17 runs the unmodified `HEAD` source at
the default density, E18 repeats the E14 configuration (warm) in the same session, with E14 as an
earlier repeat of the same configuration.

| Run | code | sweeps | points | iterations | steps | wall | us/step | accepted RMSE | max anchor |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| E13 | warm | 7 | 70 | 5 | 2,060,116 | 134.6 s | 65 | 4.25 mm | 6.50 mm |
| E15 | cold | 9 | 90 | 7 | 2,347,678 | 236.7 s | 101 | 4.25 mm | 6.50 mm |
| E14 | warm | 9 | 180 | 7 | 2,590,397 | 349.8 s | 135 | 2.42 mm | 2.74 mm |
| E18 | warm | 9 | 180 | 7 | 2,590,397 | 207.4 s | 80 | 2.42 mm | 2.74 mm |
| E17 | cold | 9 | 180 | 7 | 2,590,397 | 254.6 s | 98 | 2.71 mm | 3.28 mm |

E17 and E18 collected **bit-identical datasets**: identical sweep count, drive ranges, role
assignments and all 180 measured length pairs, and identical step counts, so the simulated
trajectories are the same. The only difference is the 11-line source change, and it moved the
accepted anchors from 2.706 mm to 2.420 mm while both loops used 7 iterations, 9 sweeps and 180
points. E14 reproduces 2.420 mm to the fourth significant figure, so the accuracy difference is
reproducible rather than run-to-run scatter: at the default density the change costs nothing in
collection and buys a small, repeatable accuracy gain (the cold run's worst anchor is D at 3.28 mm
against 2.74 mm).

The same table shows why wall time is a poor metric here: E14 and E18 are the *same configuration*
with identical step counts and identical outputs, yet E14 took 69% longer. Per-step cost ranges
65-135 us across five jobs on one machine. Every warm job was faster than its cold counterpart, but
with one pair per density that ordering is not separable from this noise; points and physics steps
are the reliable effort metrics. Evidence: `experiments/e13_live_runs.json` (regenerated to include
E17/E18) and the per-job `sweeps.json` / `sweeps.full_auto.log`.

## Implemented change

`autocal/autocal.py`, +11 lines. The ordinary full-auto iteration carries the previous accepted
anchors as a warm `initial_guess` into `_execute_plan_run`, reusing `_warm_start_seeds_from_plan`
(the helper the recovery path already used) and only when the selected run is constrained. The
solver keeps its replacement restarts, so this replaces one start rather than removing search.
Radii/buildup seeds, recovery, sparse recovery and automatic stopping are unchanged.

Tests: `pytest autocal/tests/test_autocal_no_collect.py test_ellipse_solver.py test_calibrate.py
test_planning_pass_warm_start.py -q` -> 48 passed.

## Limitations

- All evidence is simulated; no hardware measurements. "Recovery" and "accepted" are collector and
  loop observations, not resumable physics checkpoints.
- Radii/buildup/spool fitting was off, per the task constraints. Session
  `autocal-candidate-model-validation-and-throughput-fdb616a4` rejected full-loop warm starts under
  a winding mismatch with spool fitting (18.47 -> 106.90 mm); that configuration is not retested.
- E13/E15 is one matched pair. The accepted datasets were not bit-identical (identical bootstrap
  sweeps, later sweeps differ by <0.05 mm drive range), so the two-iteration saving is a single
  observation, although its mechanism (cold-fit sentinel failures) is reproduced in the E11 replay.
  E17/E18 is the stronger pair: bit-identical datasets at the default density, so its accuracy
  difference is attributable to the change.
- Wall-clock times carry uncontrolled throughput variation on this host (E14 against E18: same
  configuration, same step count, 69% difference; 65-135 us/step across five jobs). Comparisons in
  this report use points and physics steps; wall numbers are quoted only with that caveat.
- E16 is offline re-fitting of already-collected data with the reference run's frozen options. The
  sweeps it thins were planned by the loop, so it cannot show what the loop would have planned from a
  thinner dataset; and its per-prefix numbers are subject to the cold/warm multistart luck that the
  chains are meant to expose.
- The 573 s reference used a browser-recorded harness and a since-fixed pullout timeout, so the
  cross-harness wall comparison is not matched; only E13/E15/E14 are matched to each other.
- `rank_score` remains a poor accuracy proxy (E01). Do not compare accepted scores across point
  counts.
- E11 half-point data is a subset of recorded points, so it does not capture collector-side effects
  of sampling fewer points.

## Next steps (proposed, not tested)

1. **Measure why a coarse drive grid gives worse data (H12, leading).** E16 isolates the effect -
   same range, same sweep count, same point count, 2.53 mm from the default collection against
   3.84 mm from the half-density collection - but not its cause. Candidate causes are residual
   settling after larger per-step motion, and per-point hysteresis. Every recorded point carries
   `sigma_below_floor` (per-point sigma 0 or 0.0074 deg, `sample_count` 16), so the fit has no
   per-point signal to down-weight a bad measurement; a re-measurement experiment at two step sizes
   with the physics recorded would settle this.
2. **Per-point cost rather than point count is the wall-time lever (H7).** 64% of reference wall time
   is waiting for encoder stability, and wall time scales with requested simulated settling, so the
   only remaining large saving is a shorter proven-stable settle (or fewer stability re-checks).
   Needs a matched live trial; the collector already uses a 1000 ms quiet window (1500 ms in the
   reference).
3. **Point-count-aware patience is refuted, not just untested.** E15 and E16 show extra sweeps at
   half density buy nothing, so scaling patience with density would spend time without buying
   accuracy. An uncertainty-based stop is only worth revisiting if H12 shows the coarse-grid data is
   salvageable.
4. Complete the design with a repeat of the half-density pair, since E13/E15 is one observation and
   its datasets were not bit-identical.

## References

- [Research notebook](research.md): objective, constraints, steering, per-experiment log.
- [Supplied reference run](data/reference_sweeps.json) (original at
  `/home/torbjorn/repos/hp-sim5/output/extended-autocal/no-buildup-reference-003/`, HP3, no buildup,
  9 sweeps x 20 points, 573 s wall, accepted score 0.1928): the run to beat; stage artifacts in
  `data/reference_stages/`.
- Evidence files: [E01 stage audit](experiments/e01_reference_stage_audit.json),
  [E02 thinning](experiments/e02_point_subsets.json), [E05 warm chains](experiments/e05_warmstart.json),
  [E06 live 3-sweep trial](experiments/e06_live_halfpoints.json),
  [E07 multi-start](experiments/e07_multistart.json), [E08 restarts](experiments/e08_restarts.json),
  [E09 length seed](experiments/e09_length_seed.json),
  [E12 scale ladder](experiments/e12_scale_ladder.json),
  [E10 replay logs](experiments/e10_replay_warm/), [E11 half-point replay logs](experiments/e11_halfpoints_replay/),
  [live job stats, all five jobs](experiments/e13_live_runs.json),
  [E13 live iterations](experiments/e13_live_iterations.json),
  [E16 live-data thinning](experiments/e16_live_thinning.json)
  (driver: `experiments/e16_live_thinning.py`, shared harness `experiments/fitharness.py`,
  truth/geometry helper `experiments/truth.py`).
- Live job artifacts (isolated supervisor worlds):
  `native/native/autocal-live-warm-sp5/` (job `2ce379f015a14cc5952d5711405e40b2`),
  `native/native/autocal-live-cold-sp5/` (job `bd43ca8516774bbf86cdb8d5a61da3ea`),
  `native/native/autocal-live-warm-sp10/` (job `265e7d6920d44229a884a752140edc7f`, E14),
  `native/native/autocal-live-warm-sp10-rep2/` (job `8e0a92a56eda4038a0b8db9030bff04d`, E18; warm
  repeat, bit-identical dataset to E17), and
  `native/output/research/sessions/2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696/native/native/autocal-live-cold-sp10/`
  (job `6cf391f2314741b09e8c77088f11e21f`, E17; cold at default density - the supervisor resolved the
  requested `artifact_dir` relative to `<session>/native`, hence the nested path).
  Each keeps `job.json` provenance, `sweeps.json`, `sweeps.full_auto.log`, stage artifacts and an
  event-mode RRD.
- Code: `autocal/optimization_algorithm_overview` (loop names, "ordinary iterations do not
  warm-start", patience and acceptance rules), `autocal/README.md` (one-click intent, sweep design),
  `autocal/ellipse_solver.py` (`solve_anchors` seed/restart construction),
  `autocal/calibrate.py` (`build_anchor_initial_guess` scale), `autocal/fit_stage.py`,
  `autocal/autocal.py` (`_warm_start_seeds_from_plan`, `_execute_plan_run`, `full_auto_loop`),
  `autocal/control/primitives/uncalibrated_actions.mjs` (`waitForStableEncoders`: 1000 ms quiet
  window, 500 ms poll, 1.5 deg tolerance - the H7 term).
- Preceding sessions: [settling reuse, thinning and reference audit](../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/)
  (settling dominates collection time; blind thinning rejected; point-count reduction unresolved),
  [ranking, bounded updates and identifiability](../autocal-ranking-bounded-updates-negative-results-b7deb1bc/)
  (ranking changes, bounded updates and D-optimal movements rejected or mixed),
  [candidate validation and throughput](../autocal-candidate-model-validation-and-throughput-fdb616a4/)
  (warm-starting the full loop rejected under spool fitting/winding mismatch).
- A. dit Sandretto, S. Daney, M. Gouttefarde (2012), ROMANSY,
  https://www-sop.inria.fr/coprin/PDF/a_romansy2012.pdf - encoder redundancy and joint geometry/pose
  calibration in CDPRs; supports treating anchors as parameters identifiable from line-length data.
- J. Maye et al. (2016), DOI 10.1177/0278364915596232 - information-based observation selection and
  unobservable directions; background for the D-optimal next-sweep ranking already in
  `planning_pass.py`.
