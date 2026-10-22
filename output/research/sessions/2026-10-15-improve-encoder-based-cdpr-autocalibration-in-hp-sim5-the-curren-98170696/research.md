# Improve encoder-based CDPR autocalibration in hp-sim5

Session: `2026-10-15-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-98170696`
Continued session: four turns share this directory (see "Prior turns").

## Objective

Improve encoder-only CDPR autocalibration in hp-sim5 on speed, robustness and user experience,
with priority on **robust results from fewer / faster-to-collect data points**, preserving the
one-click UX and automatic stopping. Implement the best-supported change and record disproved
hypotheses.

Reference to beat: `output/extended-autocal/no-buildup-reference-003` in the main checkout
(`/home/torbjorn/repos/hp-sim5/`): HP3, no buildup, 9 sweeps x 20 points = 180 points, 573 s wall,
accepted fit score 0.1928, final anchors 2.62 mm RMSE vs simulated truth. Copy used here:
`data/reference_sweeps.json` (+ `data/reference_stages/`).

## Constraints and budgets (from the task)

- `--buildup-factor` 0; Line Layering disabled; no spool-radius search.
- Hangprinter 3 (`hp3`), unlayered, base radii 30 mm.
- Preserve one-click UX and automatic stopping; small source changes; `.venv/bin/python`.
- Movement budget: simulated collection only. Compute: offline fit ~2-4 s, parallel fits up to 10
  processes, live collection one isolated job at a time with no competing CPU load.
- Turn-4 spend: one offline grid (3 sources x variants x cold/warm chains, ~370 fits, ~5 min wall)
  and three live one-click jobs (E17 cold sp10 254.6 s, E18 warm sp10 207.4 s wall, ~8 min total).
  `research_preflight` and `scripts/research_socket_probe.py` both passed before the live jobs, and
  the world matched the required configuration. No job was cancelled; all three accepted.

## Steering accepted

- Profile the run to prioritise work (user suggestion). Done: E03.
- Priority is robust results from fewer/faster data points. E02/E05/E09/E11/E12 target point count
  and cold-start robustness; E03/E06 target per-point collection cost.
- Preserve one-click UX: the change adds no user decision, and E13/E14/E15/E17/E18 run the unchanged
  `start_autocal` job, varying only point density and (for E15/E17) the source revision.
- Turn 4 followed up the two open questions from turn 3: is the half-density loss a stopping-rule
  artefact (E16: no), and does the change help on the default one-click path (E17/E18: yes, on
  bit-identical collected data).

## Prior turns (same session)

- **E01** reproduced the score/geometry disagreement: best `rank_score` (fit-001, 0.1835) has the
  worst geometry (8.00 mm) while the best geometry (fit-007, 2.53 mm) scores 0.2359; regret 5.47 mm.
  `fit_002` reported the finite sentinel cost 100.0 with 1457.8 mm RMSE.
  `experiments/e01_reference_stage_audit.json`.
- **E02** thinned recorded points (cold). `every4`, `ends_mid_3per`, `ends_only_2per`,
  `center_only_2per` collapse to a 1220.35 mm attractor at every sweep count; `every2` works at 5
  sweeps (2.30 mm) and fails at 3, 4, 6, 7, 9 (up to 1359.6 mm).
  `experiments/e02_point_subsets.json`.
- **E03** profiled the reference: 573 s wall, ~3.38e6 steps, 6756 s sim, 364 s (64%) in
  `sweep pullout: waiting for encoder stability`, fit 1.9-3.6 s per iteration. One live 40-point
  unlayered HP3 sweep: 77.8 s wall / 173158 steps (346 s sim), 6.5 s physics compute.
  `native/00403d4402004c09aaac4ccedd97b7c4/86feab22c3934b469686a3c39ae57b5f/manifest.json`.
- **E04/E06-E15** (turns 1-3) are summarised under "Measured facts".
- **Turn 4, E16-E18.** E16 refit the three live datasets and the reference offline with cold and
  warm chains over sweep prefixes to separate point density from collection quality; E17 (cold
  `HEAD`) and E18 (warm repeat of E14) added the default-density half of the live design on
  bit-identical collected data.

## Hypotheses and outcomes

- **H1 (supported).** Warm-starting each ordinary iteration from the previous accepted anchors
  removes the point-placement dependence of the cold synthetic seed, so fewer points suffice.
  Evidence: E05, E10, **E11 (production replay)**, **E13/E15 (matched live A/B)**.
- **H2 (supported).** Wall time tracks requested simulated settling, not physics compute (E03), so
  point reduction lowers wall time but fixed pullout waits do not shrink.
- **H3 (rejected).** "Blindly halving points keeps geometry" - E02/E05 cold; only true with H1.
- **H4 (rejected).** "More restarts fix cold starts" - E08 (16/32/64 restarts; hard-seeded RNG),
  E07 (seed-only and isotropic perturbation fail).
- **H5 (mixed, not implemented).** Deterministic scale-ladder multi-start: E12 rescues `full@4`
  (1457.8 -> 3.25 mm), `every2@3` (2358.5 -> 6.45 mm), `every2@9` (1359.6 -> 3.41 mm) but worsens
  `full@3` (8.00 -> 11.45 mm), `every2@5` (2.30 -> 6.44 mm), `full@9` (2.53 -> 2.73 mm) because a
  cheaper-but-wrong start wins on cost.
- **H6 (inconclusive).** Length-derived seed scaling (E09): works for `full@3`/`every2@3`/`every2@9`/
  `full@4`, fails for `every2@4`/`every2@6`; the ~2x factor is empirical.
- **H7 (not tested).** Shorter settle/pullout timeouts (collector `settlingTimeoutMs` 30 s,
  `FORCE_TRAVEL_TIMEOUT_MS` 120 s); biggest remaining wall-time term, needs matched live trials.
- **H8 (rejected).** "Warm start reduces the number of iterations on the full dataset" - E10 (both
  7 replay iterations), E14.
- **H9 (rejected, cause corrected in turn 4).** "Halving the point density is accuracy-neutral in
  the live one-click job" - E13 (4.25 mm) vs E14 (2.42 mm). The turn-3 explanation (the half-point
  job stopped two sweeps earlier) is now rejected: E15 ran that density to 9 sweeps and accepted the
  same 4.25 mm anchors, E16 shows the half-density collection plateaus at 3.71-3.84 mm at any sweep
  count, and 90 points *thinned* out of the default collection reach 2.53 mm. Coarse collection, not
  early stopping, is the limit.
- **H10 (not tested).** Score/geometry-aware acceptance (E01 regret); prior sessions rejected
  several ranking patches, not retried here.
- **H11 (rejected).** "Point-count-aware patience or an uncertainty-based stop recovers the accuracy
  lost by halving points" - E15 and E16: more sweeps at half density buy nothing (3.84 mm at 4 and at
  9 sweeps), so waiting longer would spend collection time for no accuracy.
- **H12 (not tested, new leading idea).** The coarse drive grid gives worse data because of larger
  residual motion/settling or hysteresis per step; every recorded point is `sigma_below_floor`
  (per-point sigma 0 or 0.0074 deg), so the fit cannot down-weight bad measurements.
- **H13 (rejected in turn 4, measurement method).** "Wall-clock time is a usable comparison metric
  between live jobs" - E14 and E18 are the same configuration with identical step counts and
  identical accepted anchors, yet 349.8 s vs 207.4 s (69%); 65-135 us/step across five jobs.

## Measured facts (all simulated; no hardware measurements)

- **E05 offline chains.** `every2` cold 3/4/5/6/7/9 sweeps -> 2358.5 / 997.9 / 2.30 / 53.95 / 19.08 /
  1359.6 mm; warm -> 2.30 / 2.30 / 2.45 / 2.47 mm from 5 sweeps. `full` cold 8.00 / 1457.8 / 3.25 /
  2.83 / 3.88 / 2.53 mm; warm 8.00 / 3.25 / ... / 2.51 mm.
- **E06 live 3 sweeps x 10 points.** Near-degenerate (both cold and warm ~3432 mm): too few sweeps
  at that density, not an algorithm failure.
- **E10 replay, production loop, 20 points/sweep.** Cold: 0.1928 score, 2.62 mm RMSE, 1 recovery,
  119 s fit wall. Warm: 0.2009, 2.52 mm, 0 recoveries, 74 s.
- **E11 replay, production loop, 10 points/sweep.** Cold: 0.1374 score, 4.12 mm RMSE (anchor D
  6.80 mm), 5 recovery events, 119 s. Warm: 0.2136, 2.47 mm (max 2.85 mm), 2 recoveries, 74 s.
- **E12 scale ladder (offline).** See H5; cost-selected best-of-ladder is mixed.
- **E13 live, warm, 5 points/subsweep (10/sweep).** Accepted, 7 sweeps / 70 points, 5 iterations,
  134.6 s wall, 2.06e6 steps, final anchors 4.25 mm RMSE (max 6.50 mm), 1 recovery.
- **E15 live, cold (`HEAD`), 5 points/subsweep.** Accepted, 9 sweeps / 90 points, 7 iterations,
  236.7 s wall, 2.35e6 steps, **identical final anchors** (4.25 mm). Per-iteration costs: warm
  0.1609, 0.1432, then plateau; cold 0.1609, 100 (2353 mm), 100 (1297 mm), 0.1432, 46.8 (824 mm),
  1.315, 0.1432 - the failures kept the job collecting.
- **E14 live, warm, 10 points/subsweep (20/sweep).** Accepted, 9 sweeps / 180 points, 7 iterations,
  349.8 s wall, 2.59e6 steps, 2.42 mm RMSE (max 2.74 mm), 0 recoveries.
- **E16 offline thinning/chaining of the live datasets** (`experiments/e16_live_thinning.json`).
  Live default collection (E14 data, 20 pts/sweep): warm full 3.09/3.09/2.44 mm at 7/8/9 sweeps;
  warm `every2` (90 pts) 2.12/2.41/2.53 mm at 5/7/9 sweeps; cold `every2` 295.0/27.3/5.42 mm at
  5/7/9. Warm `uniform5` 2.81/2.80/2.71 mm. Live half-density collection (E13 data, 10 pts/sweep,
  90 pts): warm full 3.81/3.84/3.84 mm at 5/7/9 sweeps, i.e. flat from 4 sweeps on; cold full
  1296.9/824.5/3.84 mm. 3-4 points/subsweep collapse to 1220.35 mm at every prefix. Reference
  dataset reproduces E05 (`every2` warm 2.30/2.45/2.47 mm, cold 2.30/19.08/1359.6 mm).
- **E17 live, cold (`HEAD`), 10 points/subsweep.** Accepted, 9 sweeps / 180 points, 7 iterations,
  254.6 s wall, 2.59e6 steps, 2.71 mm RMSE (max 3.28 mm at anchor D), 0 recoveries. `sweeps.json` is
  bit-identical to E18's (same 180 length pairs, same ranges and roles).
- **E18 live, warm, 10 points/subsweep (repeat of E14).** Accepted, 9 sweeps / 180 points, 7
  iterations, 207.4 s wall, 2.59e6 steps, 2.42 mm RMSE (max 2.74 mm). Identical anchors and step
  count to E14/E17's dataset, so the E17/E18 pair isolates the source change at equal collection
  cost: 2.420 mm warm against 2.706 mm cold.

## Implemented change

`autocal/autocal.py`, +11 lines: `full_auto_loop` carries `warm_anchor_seed` and passes it as
`initial_guess` into each ordinary iteration's `_execute_plan_run`, updating it from
`_warm_start_seeds_from_plan(plan)["initial_guess"]` whenever the selected run is constrained. The
same helper the recovery path used; the solver keeps its restarts so one start is replaced. Radii,
buildup, recovery, sparse recovery and stopping are untouched.

Validation: 48 focused tests pass (`pytest autocal/tests/test_autocal_no_collect.py
test_ellipse_solver.py test_calibrate.py test_planning_pass_warm_start.py -q`); replays E10/E11 and
live E13/E14/E15 above.

Reconciliation: `autocal-candidate-model-validation-and-throughput-fdb616a4` rejected full-loop warm
starts under spool/radius fitting with a winding mismatch (18.47 -> 106.90 mm). This session's
evidence covers the no-buildup, no-radius-search configuration; the earlier condition is untested here.

## Experiment IDs

| ID | What | Status |
| --- | --- | --- |
| E01 | Reference stage audit vs truth | done (turn 1) |
| E02 | Point-subset thinning, cold start | done (turn 1) |
| E03 | Wall/sim time profile of the reference and one sweep | done (turn 1) |
| E05 | Warm vs cold anchor-solve chains on thinned sweeps | done (turn 1) |
| E06 | Live 3-sweep x 10-point collection | done (turn 1) |
| E07 | Cold-start multi-start arms (seed-only, perturbation, box restart) | done (turn 2) |
| E08 | Restart budget 16/32/64 | done (turn 2) |
| E09 | Length-derived seed scale | done (turn 2) |
| E10 | Production replay, 20 points/sweep, cold vs warm | done (turn 2) |
| E11 | Production replay, 10 points/sweep, cold vs warm | done (turn 3) |
| E12 | Scale-ladder multi-start | done (turn 3) |
| E13 | Live one-click job, warm, 5 points/subsweep | done (turn 3) |
| E14 | Live one-click job, warm, 10 points/subsweep (default density) | done (turn 3) |
| E15 | Live one-click job, cold `HEAD`, 5 points/subsweep | done (turn 3) |
| E16 | Offline warm/cold chains and thinning on the three live datasets + reference | done (turn 4) |
| E17 | Live one-click job, cold `HEAD`, default 10 points/subsweep | done (turn 4) |
| E18 | Live one-click job, warm, default density, repeat of E14 | done (turn 4) |

## Next decision

Keep the implemented change. Turn 4 strengthened it on the user-visible default path: E17 vs E18
collected bit-identical datasets (180 points, 2,590,397 steps, 9 sweeps, 7 iterations) and warm
seeding accepted 2.420 mm against 2.706 mm, reproducibly (E14 repeats 2.420 mm). At half density
E13 vs E15 still shows the same anchors with 22% fewer points and 12% fewer steps.

Do **not** recommend `--sweep-points 5` on its own. Turn 4 replaced the turn-3 explanation of that
(the stopping rule) with a data-quality explanation: E16 shows the half-density collection plateaus
at 3.71-3.84 mm at any sweep count while 90 points thinned out of the default collection reach
2.53 mm, and E15 confirms more sweeps at half density change nothing. H11 (point-count-aware
patience) is therefore rejected rather than deferred.

Leading next ideas: H12 (why the coarse grid gives worse data - needs recorded physics or a
step-size A/B) and H7 (the 64% of wall spent waiting for encoder stability). Also worth one repeat
of the half-density pair, since E13/E15 datasets were not bit-identical.
