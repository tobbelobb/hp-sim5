# Research record

Objective: Improve encoder-based CDPR autocalibration in hp-sim5 (HP3, no line
buildup, no spool-radius search). The current algorithm can be slow, fragile, or
choose worse anchors because its selection score disagrees with actual
geometrical accuracy. Reproduce and measure those cases first, then improve
speed, robustness and/or user experience without losing the one-click flow or
automatic stopping.

## Reference to beat

`output/extended-autocal/no-buildup-reference-003` (browser JS, HP3 scene,
layering off, `--find-radii off`): 9 sweeps / 180 points, 6756 s simulated,
1128 s wall, ended by patience after 7 iterations. Applied anchors (from
`M669` in its log) vs the simulated truth
`A[0,-1900,-280] B[1645.45,950,-280] C[-1645.45,950,-280] D[0,0,1900]`:
A `[0.00,-1903.06,-279.29]`, B `[1648.98,952.66,-279.29]`,
C `[-1646.07,949.89,-279.29]`, D `[0.17,-0.07,1901.06]` -> anchor RMSE
**2.828 mm** (recomputed here; an earlier note said 2.82 mm).
Ground truth for all HP3 fixtures is `autocal/tools/regress_calibration_logs.py`
`SIM_3D_TRUE_ANCHORS`.

## Constraints and budgets

- HP3 (`--machine-type hangprinter_3`, which autocal normalises to the
  `hangprinter_4` 4-anchor bounds family), Line Layering disabled,
  `--buildup-factor 0`, no `--find-radii` search. `.venv/bin/python` only.
- Recording: `none`/`events` for numerical and routine trials; `sampled`/`full`
  only when physics inspection is the point
  (`research/autocal-recording-performance.md`).
- Offline replay of the 9-sweep reference dataset costs ~52 s of fitting and
  replaces a ~20 min live run for algorithm-only questions. Live full-auto runs
  are budgeted one at a time via `start_autocal`.
- Movement budget: no new live collection until an offline candidate looks
  supported.

## Steering accepted from the task

1. Profile where time goes; 2. force selection is suspected to dominate accuracy;
3. wider variety of worlds would be needed for robustness claims; 4. split the
calibration into smaller isolated sub-questions; 5. preserve one-click UX and
automatic stopping.

## Hypotheses and status

| ID | Hypothesis | Status | Evidence |
| --- | --- | --- | --- |
| H0 | The reference run's selection score disagrees with geometric accuracy | Supported | `offline/audit.py` on `offline/stages/history-007.json`: rank_score ordering is uncorrelated with truth error (iter1 lowest rank_score 0.1835, worst geometry 8.00 mm; iteration 7 best geometry 2.53 mm and ineligible for the prediction ranking because it has no held-out sweeps). Reproduced again in the new replays: base ref003 iteration 4 has the lowest cold cost (0.1928) of the run but is not the best geometry (2.826 vs 2.528 mm at iteration 7). |
| H1 | Fewer points per sweep (6 instead of 10 per direction) preserve accuracy | Rejected | `offline/thin.py` + offline replays: RMSE 2.62 mm (180 pts) -> 3.46/3.53 mm (108 pts); 8 pts/sweep falls under the `min_points=10` guard and degenerates. |
| H2 | Unmodelled cable elasticity dominates the anchor error | Rejected | `--flex` on the recorded dataset changes the answer only marginally (RMSE 2.62 -> 2.53 mm) and the reference dataset has no `assumed_tension_*` fields, so the flex correction is inert. |
| H3 | The measured lengths contain no fixed-line drift | Supported | `offline/trilat.py`: fixed-line encoder deltas are exactly constant inside every sweep (range 0.000 mm); the model error is in the *geometry*, not in encoder drift. |
| H4 | Nuisance parameters (per-line offsets / unknown home pose) explain the systematic residual and improve anchors | Rejected | `offline/joint_fit.py`: at the truth anchors the residual is 0.651 mm rms with +0.34 mm mean; 4 free offsets only reach 0.489 mm and the joint fit then walks to *worse* anchors (107 mm RMSE). The raw ellipse objective is flat/degenerate, so extra freedom does not buy accuracy. |
| H5 | Broadening the final consensus from the closest pair to a wider ensemble of candidates improves accuracy | Inconclusive / not supported | On the reference history, closest-pair 2.62 mm; all eligible candidates 2.60 mm; median 2.58-2.79 mm; the apparent 2.41 mm of the three newest candidates is a post-hoc subset choice. Not implemented. |
| H6 | The per-iteration fit is fragile to local optima, and refitting the present dataset from the best anchors found so far (a competing warm-start variant) fixes it | **Supported** | See "H6 evidence" below. Cold multi-start repeatedly lands on the cost-cap sentinel in both replay and live runs; the warm variant is always available as a better-scored candidate and is selected in most late iterations. Same-data applied RMSE 2.623 -> 2.552 mm; best candidate 2.528 -> 2.426 mm. |
| H7 | Force-selection sensitivity explains most cross-world accuracy spread | Not tested | Needs either fixture datasets with matched radii/buildup options or new live worlds. Not attempted: the shipped non-task regime (buildup 0.636619, global radii search) makes the available fixture datasets poor proxies. |
| H8 | Cutting the number of collected sweeps (early stop) preserves accuracy | Rejected | `offline/loop_emul.py` cumulative-sweep ablation on ref003: 3 sweeps 8.00 mm, 4 sweeps 1457.8 mm (diverged), 5 sweeps 3.25 mm, 6 sweeps 2.83 mm, **7 sweeps 3.88 mm**, 8 sweeps 2.73 mm, 9 sweeps 2.53 mm. Accuracy is non-monotone in sweep count and the last sweeps still help, so no safe reduction was found. |

## H6 evidence

- E04 emulation (`offline/loop_emul.py`, `offline/emul-*.json`): cold iteration
  sequence 8.00, 1457.8, 3.25, 2.83, 3.88, 2.73, 2.53 mm; warm-chained 8.00,
  3.25, 3.25, 3.25, 3.25, 2.53, 2.51 mm. Cold mode reproduces the recorded
  loop's per-iteration geometry, which validates the emulation.
- E05 controlled replay of the *identical* 9-sweep dataset (`offline/base-replay.log`,
  `offline/warm-replay.log`, stage artifacts in `offline/stages-base_1/`,
  `offline/stages-warm_1/`): base applies 2.623 mm, warm applies 2.552 mm.
  Per-iteration warm variants win iteration 5 (2.600 vs 3.881 mm) and
  iteration 6 (2.426 vs 2.733 mm) on score. Offline fit CPU rises 147.8 s ->
  274.4 s (1.86x) because every iteration now fits twice.
- E05 live HP3 (`events` recording, `sweep_points=20`, job
  `c75c027e59c142ae84bfa4997880cfe0`): accepted `patience-or-threshold` after 7
  iterations, wall 394.6 s, simulated 5798 s, 2 899 212 physics steps, 360
  collected points. Warm variants were selected in iterations 4, 5 and 6 and the
  final consensus used `members=[2,3,4,5]`, `spread_mm=0.2173`, applied 2.614 mm
  vs the reference's 2.828 mm -- but this run used 2x the reference's points per
  sweep, so it is not a same-data speed/accuracy comparison. Decisive detail:
  the cold iteration-4 fit hit the cost cap (`cost=100.0`, ~1209.9 mm) and the
  warm variant supplied 2.619 mm instead, so the diverged cold fit never reached
  the applied answer or the next sweep plan. Fitting was 48.6 s of the 394.6 s
  wall (12%), of which 8.3 s was warm-variant fitting.
- E05b live HP3 replay run `d7a7507f7b60486da3c0d626be819afd`, launched with
  `sweep_points=10` for a same-data comparison with the reference, turned out to
  be the **base** algorithm: `offline/xworld/matrix.py` swaps `autocal.py` between
  variants and raced the job start, and its log contains no `default.warm` line
  at iteration 2. It is therefore the matched *base@10* control: 9 sweeps /
  180 points with exactly the reference's sweep layout (fixed_anchors
  `[(0,3),(1,3),(2,3),(1,3),(2,3),(0,3),(2,3),(1,3),(0,3)]`, drives
  `[1,0,0,0,0,1,0,0,1]`), accepted `patience-or-threshold` after 7 iterations,
  wall 581.3 s, applied consensus `members=[2,3,4,5] spread_mm=2.204` ->
  **2.706 mm** (reference 2.828 mm, older code revision). Its cold fits stall at
  identical `fit_score_ui=0.2243` for iterations 2, 3 and 4, which is the
  fragility H6 is aimed at.
- E05c live HP3 H6@10 job `aa8435dfa54c4c28a8f040c43fa2ebce`, same settings and
  same 9-sweep/180-point layout as E05b with `autocal.py` = H6 (24 `default.warm`
  lines): accepted `patience-or-threshold` after 7 iterations, applied consensus
  `members=[6,7] spread_mm=0.7008` -> **2.440 mm** vs base@10's 2.706 mm and the
  reference's 2.828 mm. Cold per-iteration fits are numerically identical between
  E05b and E05c (7.379, 3.088, 3.088, 3.088, 2.385, 2.877, 3.169), proving the
  change is purely additive; the warm variants supplied 2.452 and 2.444 mm at
  iterations 6 and 7 where the cold fits gave 2.877 and 3.169 mm. Wall times
  (581.3 s vs 178.8 s) are not comparable because host load average fell from
  ~57 to ~10 between the two jobs.
- Variant fingerprinting: `default.warm` occurrences in `sweeps.full_auto.log`, or
  `source_sha256["autocal/autocal.py"]` inside the stage artifacts. The job
  manifest's `autocal_source_sha256` does **not** cover `autocal/autocal.py`
  (`autocal/headless_sim.py:source_hash` hashes only js/mjs/rs/c sources under
  `autocal/control`, `scripts`, `src/js`, `src/.../app`, `src/wasm/...`,
  `integrations`) and must never be used to identify the algorithm revision.
- Cross-world matrix (`offline/xworld/matrix.py`, `offline/xworld/runs/`):
  base variants completed on six fixture datasets; the warm half of the matrix
  produced log/dataset artifacts but no `stdout.log` and no `results.json`, so it
  is **incomplete**. More importantly these fixtures are shipped-regime datasets
  (buildup 0.636619, global radii search) replayed with `--buildup-factor 0
  --find-radii off`, which makes their absolute RMSE meaningless (485-2822 mm
  against the HP3 truth). Treated as not usable evidence.
- Shipped-regime regression matrix (`autocal/tools/regress_calibration_logs.py`,
  `offline/regress-base.txt`, `offline/regress-warm.txt`) is likewise outside the
  task regime (buildup 0.636619, find-radii used) but shows H6 is neutral there:
  `sum_true_err_total_delta` -128.631 (base) vs -125.521 (warm) against the
  stored reference logs, with the same per-dataset anchors except a `fit_ui`
  improvement on `sixteenth_hp3_dataset_even_more_pressure` (3.054 -> 1.299).

## Experiment IDs

- E01 offline profile + geometry audit of the reference run (`offline/time_profile.py`, `offline/audit.py`, `offline/resid_analysis.py`).
- E02 point-thinning sweep (`offline/thin.py`, `offline/thinruns/*.log`).
- E03 nuisance-parameter/model-error test (`offline/joint_fit.py`, `offline/trilat.py`).
- E04 cold-vs-warm iteration emulation (`offline/loop_emul.py`).
- E05 implementation of H6 in `autocal.py` + controlled offline replay of ref003
  + one supervised HP3 no-buildup live run (`native/autocal/c75c027e...`).
- E05b matched base@10 live HP3 control (`native/autocal/d7a7507f...`) - done, 2.706 mm.
- E05c matched H6@10 live HP3 run (`native/autocal/aa8435df...`) - done, 2.440 mm.
- E06 shipped-regime regression matrix base vs warm (`offline/regress-*.txt`).
- E07 cross-world fixture matrix (`offline/xworld/`) - incomplete, unusable.

## Decision

Ship H6: one extra history-seeded ("warm-start") run variant per
dataset-growth iteration, selected by the existing validity/ranking machinery
(`autocal/autocal.py`, `_execute_run_variant`). It needs no new collection,
leaves one-click behaviour and automatic stopping untouched, costs extra fit CPU
only, and keeps a good candidate in the pool when a cold multi-start lands on
the cost-cap sentinel. Matched live A/B at the reference's data volume: applied
2.706 -> 2.440 mm, same 7 iterations and stop rule. Targeted tests pass
(55 passed); full `autocal/tests` 368 passed with 3 pre-existing unrelated
failures. Remaining risks: one trial per variant, and all evidence is
simulated/synthetic - no hardware measurements were taken.

## Next decisions to make

- Repeat the matched live A/B a few times, since each variant has only one trial
  and the observed gain (2.706 -> 2.440 mm) is of the same order as the
  run-to-run spread implied by the non-monotone iteration sequences.
- If a future session wants a faster-collection change, H8 shows sweep-count
  reduction is unsafe on the available fixture; a force-selection study (H7) on
  varied worlds, built on no-buildup recordings, still looks like the
  highest-value untested direction.
- Only one HP3 world was used. The "many plausible physical worlds" programme
  needs cheap synthetic no-buildup worlds; the shipped fixture datasets cannot
  serve that role because they were recorded with line layering.
