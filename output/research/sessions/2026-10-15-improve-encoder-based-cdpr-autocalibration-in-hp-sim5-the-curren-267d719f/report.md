# History-seeded fit variants for robust encoder autocalibration (HP3, no buildup)

Status: complete.

## Abstract

Encoder autocalibration repeatedly fits the growing sweep dataset, and a cold
multi-start fit sometimes lands on the cost-cap sentinel or stalls in a bad local
optimum. We reproduced that offline on the supplied 9-sweep HP3 reference
dataset and in live HP3 runs, then implemented a small change: each iteration
also fits the present dataset seeded from the best anchors found so far, and the
two candidates compete through the existing validity and ranking selection. The
change adds no data collection and leaves the one-click flow and automatic
stopping untouched. In a matched live A/B on the same unlayered HP3 world with
the identical 9-sweep/180-point layout, the applied anchor RMSE improved from
2.706 to 2.440 mm (reference 2.828 mm) with the same seven iterations and the
same stop rule; on the identical reference dataset replayed offline it improved
from 2.623 to 2.552 mm applied and 2.528 to 2.426 mm best candidate. Cold fits
are bit-identical between variants, so the change is purely additive. All
evidence is simulated/synthetic; no hardware measurements were taken, and host
load made wall-clock comparisons unusable.

## Question and framing

The task: improve encoder-based CDPR autocalibration in hp-sim5 for Hangprinter 3
with line layering disabled, no buildup and no spool-radius search, without
losing the one-click user experience or the automatic patience stop. The named
suspicion was that the selection score disagrees with geometric accuracy, that
force tuning dominates accuracy, and that splitting the search into smaller
isolated sub-questions would help. Speed and robustness matter most, because
collection is the expensive part and having to redo it wrecks the experience.

Regime actually used everywhere in this session: `--machine-type hangprinter_3`
(autocal normalises `hp3`/`hangprinter_3` to the 4-anchor `hangprinter_4` bounds
family, `autocal/_autocal_common.py:115`), scene `hp3_rigid_body.usda`, firmware
`sys/config_hp3.g`, `--base-radii 30 --buildup-factor 0 --find-radii off
--find-buildup-factor off`, recording `events`. Reference to beat:
`output/extended-autocal/no-buildup-reference-003` (9 sweeps, 180 points,
6756 s simulated, 1128 s wall, patience-stopped after 7 iterations), whose
applied anchors are 2.828 mm from the simulated truth.

## What was reproduced (E01)

The reference run's own history shows the selection score ordering disagreeing
with geometry: in `offline/stages/history-007.json` the iteration with the
lowest `rank_score` (0.1835) has the worst geometry (8.00 mm), while the best
geometry (2.53 mm) belongs to an iteration that is *ineligible* for the
prediction ranking because it has no held-out sweeps. The same pattern recurs in
this session's fresh replays: in the base replay of the reference dataset,
iteration 4 has the lowest cold cost of the run (0.1928) but is not the most
accurate iteration (2.826 vs 2.528 mm at iteration 7). So "score disagrees with
geometry" is reproducible, but it is a property of which *candidate is chosen*,
not of the fitter.

The second reproduced case is fragility, and it is the one the shipped change
targets. A cold multi-start fit occasionally terminates on the cost cap and
returns a nonsense anchor set:

- offline replay of the reference dataset, iteration 2: `cost=100.0`,
  1457.8 mm;
- live HP3 run `c75c027e59c142ae84bfa4997880cfe0`, iteration 4:
  `cost=100.0`, 1209.9 mm.

Because that candidate is a normal, valid run result it enters the candidate
pool with a real score, and in the live run it would have been the only
candidate for that iteration, so it would have fed the next sweep plan.
`autocal.py` already had a warm-start-from-a-known-plan helper
(`_warm_start_seeds_from_plan`, used by the sparse-recovery path), but it was
never applied to the ordinary per-iteration fit.

## The change (E05)

`autocal/autocal.py`, inside the full-auto loop only:

- `_resolve_run_settings(run)` was factored out of the `for run in runs:` body
  (pure extraction, no behaviour change).
- `_execute_run_variant(run, *, warm_seeds=None)` runs the cold plan unchanged,
  or, when `warm_seeds` is given, the same plan under the id `f"{run_id}.warm"`
  with a distinct `path_tag` and `initial_guess` / `initial_radii_mm` /
  `initial_buildup_factor` taken from `_warm_start_seeds_from_plan`.
- After the cold runs, if the stop file was not requested, the iteration sources
  seeds from the newest history candidate's plan (falling back to `best_plan`)
  and appends one such warm variant.

The warm result is an ordinary entry in `run_results`, so it competes through
the existing `valid_runs` / `_sort_key` selection. It can only win on score, it
collects no data, it does not touch planning, stopping or the CLI surface. Cost
is one extra fit of the same dataset per iteration.

## Measurements

### Controlled same-data replay of the reference dataset

`autocal.py --sim --no-collect` over the unmodified 9-sweep reference dataset,
base (`/tmp/autocal_base.py`) vs H6, log and stage artifacts in
`offline/base-replay.log`, `offline/warm-replay.log`, `offline/stages-base_1/`,
`offline/stages-warm_1/`.

Per-iteration anchor RMSE / fit cost:

| iter | base | H6 cold | H6 warm |
| --- | --- | --- | --- |
| 1 | 8.001 / 0.1835 | 8.001 / 0.1835 | - |
| 2 | 1457.827 / 100.0 | 1457.827 / 100.0 | **3.248** / 0.2247 |
| 3 | 3.248 / 0.2247 | 3.248 / 0.2247 | 3.248 / 0.2247 |
| 4 | 2.826 / 0.1928 | 2.826 / 0.1928 | 3.249 / 0.2247 |
| 5 | 3.881 / 0.3629 | 3.881 / 0.3629 | **2.600** / 0.2289 |
| 6 | 2.733 / 0.2078 | 2.733 / 0.2078 | **2.426** / 0.2450 |
| 7 | 2.528 / 0.2359 | 2.528 / 0.2359 | 2.434 / 0.2557 |
| **applied** | **2.623** | | **2.552** |

Offline fit CPU: 147.8 s (base) -> 274.4 s (H6), 1.86x. Offline replays contain
no collection, so this ratio is the worst case; see below for the live split.

Note that the warm variants win iterations 5 and 6 on score while they are not
uniformly better than the cold fit (iteration 4's warm result is worse than the
cold one). The benefit is that a good candidate stays in the pool, not that the
seed is always right.

### Live HP3 run `c75c027e59c142ae84bfa4997880cfe0`

`events` recording, `sweep_points=20` (40 data points per sweep), accepted with
`stop_reason=patience-or-threshold` after 7 iterations: wall 394.6 s, 5798 s
simulated, 2 899 212 physics steps, 360 collected points, applied
`members=[2,3,4,5] spread_mm=0.2173` -> **2.614 mm** vs the reference's
**2.828 mm**. Fingerprint of the variant: `default.warm` appears 27 times in
`sweeps.full_auto.log`, and warm runs were selected at iterations 4, 5 and 6.

Caveat: this run used twice the reference's points per sweep, so it is *not* a
same-data comparison and does not by itself show a speed or accuracy win.

The honest live signal is the iteration-4 divergence: cold fit `cost=100.0`
(1209.9 mm), warm variant 2.619 mm, warm selected. Fitting accounted for 48.6 s
of the 394.6 s wall (12%), of which 8.3 s was warm-variant fitting, so the added
fit work is roughly 2% of a live run's wall time when collection dominates.

### Matched live A/B at the reference's data volume (E05b, E05c)

Both jobs: HP3, unlayered, `base_radii 30`, `buildup 0`, `find-radii off`,
`events` recording, `sweep_points=10`, `max_steps=20`, `wall_timeout_s=3600`,
lbfgsb/fast, 4 restarts, 400 iterations. Both produced **9 sweeps / 180 points
with exactly the reference's sweep layout** (fixed anchors
`[(0,3),(1,3),(2,3),(1,3),(2,3),(0,3),(2,3),(1,3),(0,3)]`, drives
`[1,0,0,0,0,1,0,0,1]`), i.e. the same data volume as
`no-buildup-reference-003`.

| run | variant | iters | applied RMSE | consensus | wall | fit CPU |
| --- | --- | --- | --- | --- | --- | --- |
| `no-buildup-reference-003` | older revision, browser JS | 7 | 2.828 mm | - | 1128 s | - |
| `d7a7507f...` | base | 7 | 2.706 mm | members `[2,3,4,5]`, spread 2.204 | 581.3 s | 60.2 s |
| `aa8435df...` | H6 | 7 | **2.440 mm** | members `[6,7]`, spread 0.7008 | 178.8 s | 15.6 s |

Per-iteration anchor RMSE (mm) makes the causal chain visible:

| iter | base cold | H6 cold | H6 warm |
| --- | --- | --- | --- |
| 1 | 7.379 | 7.379 | - |
| 2 | 3.088 | 3.088 | 3.088 |
| 3 | 3.088 | 3.088 | 3.088 |
| 4 | 3.088 | 3.088 | 3.089 |
| 5 | 2.385 | 2.385 | 3.088 |
| 6 | 2.877 | 2.877 | **2.452** |
| 7 | 3.169 | 3.169 | **2.444** |

The cold columns are numerically identical, which both validates the change as
purely additive and rules out any chance of the warm path perturbing the cold
fit. The base run's cold fit stalls at 3.088 mm for iterations 2-4, and its last
two cold fits (2.877, 3.169 mm) are worse than the H6 warm fits that replaced
them in the pool (2.452, 2.444 mm). The final consensus then averaged the two
best-and-newest candidates in the H6 run and the stalled mid-run candidates in
the base run, which is what turns into 2.440 vs 2.706 mm.

The wall-time columns are **not** comparable: host load average fell from ~57 to
~10 between the two jobs, and the *same* first fit took 10.9 s under load and
1.7 s at lower load. Fit CPU totals differ for the same reason. Compare only
in-process `fit_seconds` within a job, or the offline replay, where warm fits
cost 0.3-0.8x a cold fit and therefore add roughly +25% to +67% to fit CPU. Since
fit CPU is ~10% of a live run's wall time, the added work is on the order of 2-8%
of wall time.

### Rejected alternative for the live comparison

The first `sweep_points=10` job `d7a7507f...` was intended as the H6 half but ran
the **base** algorithm: `offline/xworld/matrix.py` swaps `autocal.py` between
variants and raced the job start. This is why the table above lists it as the
base control. Confirmed by the absence of any `default.warm` line at its
iteration 2. The matched H6 half was then run as `aa8435df...` with nothing else
touching the tree, and it shows 24 `default.warm` lines and warm-selected
iterations.

Fingerprint warning that cost time here: the job manifest's
`autocal_source_sha256` does **not** cover `autocal/autocal.py`
(`autocal/headless_sim.py:source_hash` hashes only `.js/.mjs/.rs/.c` files under
`autocal/control`, `scripts`, `src/js`, `src/**/app`, `src/wasm/**`,
`integrations`). Use `default.warm` occurrences in the run log, or
`source_sha256["autocal/autocal.py"]` inside the stage artifacts, to identify
which algorithm a run used.

## Rejected and untested hypotheses

- Fewer points per sweep (E02, rejected): 180 points -> 2.62 mm, 108 points ->
  3.46/3.53 mm, and 8 points/sweep degenerates below the `min_points=10` guard.
- Cable elasticity (E03, rejected): `--flex` moves the answer 2.62 -> 2.53 mm and
  the reference dataset carries no `assumed_tension_*`, so the correction is inert.
- Encoder drift on fixed lines (E03, rejected as an explanation): fixed-line
  encoder deltas are exactly constant within each sweep (range 0.000 mm).
- Nuisance per-line offsets / unknown home pose (E03, rejected): free offsets
  only reduce the truth-anchor residual from 0.651 to 0.489 mm rms, and the joint
  fit then walks to 107 mm RMSE. The raw objective is too flat to support extra
  freedom.
- Wider final consensus (E04, inconclusive/not supported): all-candidate 2.60 mm,
  closest-pair 2.62 mm, median 2.58-2.79 mm; the attractive 2.41 mm is a post-hoc
  subset choice. Not implemented.
- Cutting collected sweeps (E05, rejected): cumulative-sweep ablation on the
  reference dataset gives 3 sweeps 8.00 mm, 4 sweeps 1457.8 mm, 5 sweeps 3.25 mm,
  6 sweeps 2.83 mm, 7 sweeps 3.88 mm, 8 sweeps 2.73 mm, 9 sweeps 2.53 mm.
  Accuracy is non-monotone in sweep count and the last sweeps still help, so the
  "collect less data" direction found no safe reduction. The autocal patience
  rule already stops after 3 non-improving attempts.
- Force-selection sensitivity (H7, not tested) and the "many plausible physical
  worlds" robustness programme (not tested): the available fixture datasets are
  shipped-regime recordings (buildup 0.636619, global radii search) and give
  meaningless absolute error (485-2822 mm) when replayed with
  `--buildup-factor 0 --find-radii off`, so they are not usable proxies for the
  task regime. Building matched-world fixtures is the obvious next investment.
- Shipped-regime regression matrix (E06, neutral): with
  `autocal/tools/regress_calibration_logs.py` (outside the task regime, so this
  is a guard rail rather than evidence for the change) the
  `sum_true_err_total_delta` is -128.631 for base and -125.521 for H6 against the
  stored reference logs. Base leaves every `fit_score_ui` unchanged; H6 changes
  them on `sixteenth_hp3_dataset_even_more_pressure` (3.054 -> 1.299, better) and
  `fifth_hp3_dataset` (6.952 -> 25.100, worse), and moves
  `true_err_anchors` by +2.6 on `seventh_hp3_dataset` and +0.5 on
  `sixteenth_...`. H6 does not regress the older regime materially.
- Cross-world fixture matrix (E07, unusable): the warm half produced no
  `stdout.log` and no `results.json`, so it is incomplete; and the absolute
  errors make it uninformative regardless.

## Validation

- `autocal/tests` full suite: **368 passed, 3 failed, 2 deselected** in 64 s. All
  three failures are pre-existing and unrelated: `test_filter_pass_spool_fit.py`
  and `test_legacy_zero_buildup.py` need the missing vendored file
  `autocal/auto-calibration-simulation-for-hangprinter/util.py`.
- Targeted: `test_autocal_no_collect.py test_autocal_score_ui.py
  test_history_selection.py test_predictive_validation.py` -> 55 passed. The
  `test_autocal_no_collect.py` stubs were extended so the faked planner answers
  the warm-start call as well as the cold one.

## Limitations

- Simulated and synthetic data only. No hardware measurements were taken, and no
  claim about real machines is made.
- One world family (single HP3 scene, one true anchor set, no line layering).
  Robustness across realistic parameter variation is *not* established.
- Wall-clock numbers are unusable: host load average was 95-106 on 20 cores for
  the whole session, so live/offline timings are indicative only. The fit-CPU
  ratio comes from in-process `fit_seconds`, which is less load-sensitive but
  still not a clean benchmark.
- Each live variant was run once. The matched H6@10 vs base@10 gap (2.706 ->
  2.440 mm) and the offline gap (2.623 -> 2.552 mm) are single trials; the
  non-monotone iteration sequences show run-to-run variation of this order, so
  the size of the gain is not tightly estimated even though its direction is
  consistent across three independent comparisons.
- The live H6@10 win partly flows through the final consensus-member choice
  rather than through the fit alone. H6 improves the candidate pool; whether the
  existing selection then exploits it is a second, separate mechanism.

## Reproducing

```
# controlled same-data replay (base vs H6)
cp /tmp/autocal_base.py autocal/autocal.py     # or /tmp/autocal_h6.py
MPLCONFIGDIR=/tmp/mpl .venv/bin/python autocal/autocal.py --sim --no-collect \
  --machine-type hangprinter_3 --buildup-factor 0 --find-radii off \
  --dataset offline/ref003.json

# live HP3 full-auto (one-click-equivalent, supervised)
# via MCP: start_autocal(machine='hp3', layering=False, base_radii_mm=[30],
#   optimize/find flags off, optimizer={mode:'fast', solver:'lbfgsb',
#   solve_iterations:400, solve_restarts:4},
#   budgets={max_steps:20, sweep_points:10, wall_timeout_s:3600},
#   recording_mode='events')
# Then check the variant actually used with:
#   grep -c default.warm <job>/sweeps.full_auto.log
```

Matched live pair recorded here: base
[native/autocal/d7a7507f...](native/autocal/d7a7507f7b60486da3c0d626be819afd/sweeps.full_auto.log)
(2.706 mm) and H6
[native/autocal/aa8435df...](native/autocal/aa8435dfa54c4c28a8f040c43fa2ebce/sweeps.full_auto.log)
(2.440 mm), both 9 sweeps / 180 points on the same HP3 world.

Artifacts: `offline/` (replays, emulation, audits), `native/autocal/<job>/`
(live runs), `offline/regress-*.txt` (shipped-regime regressions).

## References

- [Research notebook](research.md): budgets, hypothesis table, accepted steering.
- [Searchable abstract](abstract.md): title, topics, tested hypotheses.
- `autocal/README.md`, `autocal/optimization_algorithm_overview`,
  `autocal/objective_functions_overview`,
  `autocal/README_elliptical_feature_calibration.md`: description of the current
  algorithm and objectives.
- [Reference run to beat](../../../extended-autocal/no-buildup-reference-003/):
  supplied HP3 baseline, 2.828 mm applied anchor RMSE (recomputed here).
- `autocal/tools/regress_calibration_logs.py` `SIM_3D_TRUE_ANCHORS`: ground truth
  used for every RMSE in this report.
- `autocal/autocal.py`, `autocal/_autocal_common.py`, `autocal/fit_stage.py`,
  `autocal/history_selection.py`, `autocal/headless_sim.py` at revision
  `1404c5ac15bd5d887b9c7becaefe7f4d9f289af1`: implementation studied and changed.
- [Prior session on settling and score-vs-geometry](../2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/report.md):
  earlier HP3 findings reused here (settling cost, thinning rejection, a
  default-force HP3 job accepting at 360 points / 2.604 mm).
- [Recording-cost budget](../../../../research/autocal-recording-performance.md):
  `none`/`events`/`sampled`/`full` recording workload, used to keep replay
  experiments cheap.
