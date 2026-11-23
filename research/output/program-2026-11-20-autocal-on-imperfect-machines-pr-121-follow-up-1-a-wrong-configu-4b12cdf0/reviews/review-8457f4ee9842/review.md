# Review: robust sweep jackknife (`fold_spread_mm`), candidate e292f73e

**Verdict: accept with caveats.** The change fixes a real, reproduced false-"poor" problem on HP5/HP5r under free low z. It does not remove the ability to flag failures that come from noise spread across every sweep. But three statements in the submission and in the new docstring overstate it:

- "Still calls every case off by more than 5 mm poor" holds only for every-sweep noise. With one bad sweep it is false: 2 of 22 injected >5 mm failures read "usable" under the robust spread, against 0 of 22 under the classical one.
- The Spearman gain (0.60 → 0.67) is not significant. A cluster bootstrap 95% interval is [−0.01, +0.18].
- "No change in selection" is untested on patience runs. On replay, 1 of 84 distinct corpus runs changes its applied fit for the worse (2.94 → 4.62 mm).

None of these is a blocker for a reporting statistic. The docstring and catalog claims should be narrowed before merge (see "Required changes").

## Integrity and setup

- `./hp-sim5-research-program --resume … --verify reviews/review-8457f4ee9842` → `verified: true`, candidate e292f73e2ab2913bc36343f141fb93817b360d9f, tree 422af6a2.
- The candidate was checked out detached in the reviewer worktree, with .venv, node_modules, RRF/build and the cable_solver target symlinked.
- `pytest autocal/tests/test_anchor_uncertainty.py`: 10 passed.
- This is not a physics change: there are no solver, Rust or wasm edits. The physics checks do not apply, so none were run.
- No simulations were run. Everything below is offline refitting, with at most 3 processes at a time.

Reviewer evidence is in [`review-evidence/`](review-evidence/):

- scripts: `replay_folds.py`, `replay_decisions.py`, `one_bad_sweep.py`
- outputs: `folds.jsonl`, `decisions.txt`/`.json`, `bad10.json`, `bad20.json`, `e11_repro.json`

## What was reproduced and what was only inspected

| Check | Kind | Result |
|---|---|---|
| E11 rows: new_hp5_seed1, new_hp5r_seed3, benchmark HP3:4, noisy 0.5 mm HP4 | **Reproduced** (submission script, candidate code) | Identical to the submission: e.g. 0.769 / 6.769 / 1.145 and 1.828 / 22.90 / 2.93 (err / J0 / J1, mm) |
| Live `robust_hp5_seed1/2` console uncertainty matches offline J1 (1.145, 2.495) | Inspected | Consistent, so the offline harness agrees with production on these runs |
| Small-n behaviour of the estimator | **Reproduced** (Monte Carlo, below) | Breakdown as derived |
| Single-bad-sweep injection, 4 reference machines × every sweep × 10 and 20 mm | **New test** | Two misses, below |
| Corpus replay of patience ranking and `newest_trusted_index`, J0 vs J1 | **New test**, 86 runs (84 distinct) | 1 worse selection, 2 patience stops removed, 2 added |
| Callers | Inspected | Below |

## Callers (inspection)

`fold_spread_mm` is reached only through `sweep_jackknife_mm` ← `plan_anchor_uncertainty_mm` (autocal.py:1222). The ellipsoid path uses its own `row_jackknife_mm` (max shift) and is unaffected. The value feeds three things:

1. **Console label** (`uncertainty_label`, autocal.py:316).
2. **`newest_trusted_index`** (autocal.py:850). It skips a newest fit whose value exceeds `max(3 mm, 10 × min over history)`. This ratio guard is scale-sensitive. The robust spread gives lower minima in good iterations, so the limit gets tighter (see corpus finding 1).
3. **Patience** (`stopping.history_rank_position`). An iteration "improves" only if it has the lowest value so far. This depends only on the ordering, so any reshuffle of near-equal values can move the stop. `good-fit` uses covariance `max_std_mm` and `stable-estimate` uses anchor step, so neither depends on this value.

## Q1. Does the robust spread hide real failures?

**By construction, yes, for any failure that shows up in one fold only.** A single contaminated sweep produces exactly one deviant leave-one-out fold: the fold that drops it, which is the *correct* estimate. With n folds, the median absolute deviation ignores up to ⌈n/2⌉−1 deviant folds. Monte Carlo (folds N(0, 0.3 mm), k folds offset 20 mm; median of 2000 draws):

| n | k=1 | k=2 | k=3 | k=4 |
|---|---|---|---|---|
| 3 | J0 13.3 / J1 0.61 | J1 0.62 | – | – |
| 4 | J1 0.59 | **J1 25.5** | J1 0.60 | – |
| 5 | J1 0.67 | J1 1.43 | J1 1.43 | – |
| 7 | J1 0.77 | J1 1.12 | J1 2.13 | J1 2.11 |
| 10 | J1 0.91 | J1 1.14 | J1 1.55 | J1 2.46 |

So two loose folds out of five, typical for HP5, are hidden completely. HP3/HP4 with 7–10 sweeps hide up to 3–4.

**Does this bite in practice?** I injected a localized disturbance: the held low anchor moves radially by 10 or 20 mm for one sweep's ranges only. Delta resynthesis came from the session's `lowz_common.resynthesise`, applied to every sweep of the four benchmark_923c2baa references in turn (56 cases).

- On HP5/HP5r (5–6 folds) the corruption usually disturbs *all* folds, because each sweep matters to the whole fit. J1 stayed "poor" on every case above 3.5 mm except one.
- Robust wins: J0 false-"poor" at error ≤ 1.5 mm was 6 cases, J1 1 case. Example: HP3 sweep 5 at 20 mm: err 1.15, J0 9.35, J1 0.39.
- **Misses** (`bad10.json`, `bad20.json`):

| Case | Sole holder? | err | J0 (label) | J1 (label) |
|---|---|---|---|---|
| HP3, sweep 8 corrupted 10 mm | no (C held in 5 sweeps) | 5.06 | 5.23 poor | **2.63 usable** |
| HP3, sweep 8 corrupted 20 mm | no | 10.06 | 10.10 poor | 4.84 poor (but < 5 mm on a > 10 mm failure) |
| HP5r, sweep 0 corrupted 20 mm | **yes** (only sweep holding A) | 5.71 | 5.95 poor | **2.60 usable** |

The HP5r row is exactly the scenario in the key questions: a single sweep carries the only constraint on a coordinate. Here the fold that drops it is the only one that sees the problem, and J1 discards it. The HP3 row shows the same blindness without a sole-holder sweep: one late, influential sweep is corrupted, and only its fold (deviation 4.9 / 9.5 mm vs ≤ 2.8 mm elsewhere) reveals it.

Over the injected >5 mm cases: J0 never failed to call "poor" (0/22), J1 failed twice (2/22). J1 also reported less than half the true error twice. **The prior property "every run off by > 10 mm reports > 5 mm" does not survive** (10.06 mm → 4.84 mm). It survives in the submission's 41 cases only because none of them is a localized failure.

Trade-off: the robust spread converts identifiability-driven false alarms (one fold loses a constraint) into passes. It also converts contamination-driven true alarms (one fold is the only clean one) into passes. The two cases look identical to a median.

## Q2. Fit selection and patience on existing runs

Corpus: every `*.full_auto_log.jsonl` with a `.headless/manifest.json` under `output/` and `research/output`. That gives 253 runs. 217 are sweep (non-ellipsoid) runs, and 88 are distinct after deduplication by dataset prefix, truth and iteration counts. 86 were replayed, 84 distinct after hashing.

For each iteration, the dataset prefix was taken from the console "Collected sweep nr" lines. It was refitted with the candidate two-stage pose fit (as in E11), all folds were stored, and J0/J1 were computed from the *same* folds. The candidate's `history_rank_position` patience counter (with the run's logged patience) and `newest_trusted_index` were then replayed at the actual end and at each replayed patience stop. Underconstrained/admissible flags were taken as admissible.

Differences (`decisions.txt`):

1. **Selection change, worse.** `output/extra-sensor-production/pass3/reduced` (HP5r, ended on patience, 13 sweeps).
   - J1 minimum was 0.72 mm (iteration 5), so the limit was 7.2 mm. Newest-fit J1 was 7.45 mm, so the newest fit was rejected as "diverged" and iteration 10 was applied.
   - Error: **4.62 mm instead of 2.94 mm**.
   - J0: minimum 1.09, limit 10.9, newest 5.46, so the newest fit was kept.
   - Cause: robust spread lowers good-iteration minima, tightening the 10× ratio guard, while on that iteration it is *higher* than J0 (spread evenly over 13 folds).
2. **Patience stops removed under J1.** `hp4-final` (P=3): J0 stops at iteration 5 (applied 3.97 mm); J1 keeps collecting to 6 (4.25 mm). `backfill/f0efe6cc21bb` HP3 (P=8): J0 stops at 11, J1 does not; same applied fit.
3. **Patience stops added under J1.** Two HP4 backfill runs whose harness fits are unusable (see limitations). They have no consequence.

Overall: 3 of 84 distinct runs change a meaningful decision, 1 of them for the worse by 1.7 mm. Patience fires within run length equally often (5 vs 5). There are no HP5 (non-reduced) patience runs in the corpus, so the machine where the change matters most has no patience coverage.

## Q3. Is the 41-case evaluation sound?

- **Duplicates and dependence.**
  - 24 of the 41 cases derive from 4 benchmark datasets: clean, disturbed, 0.2 mm noise and 0.5 mm noise variants, plus 8 ":4" prefix truncations of those same datasets.
  - Clustering truncations with their parent leaves 33 clusters.
  - Paired cluster bootstrap of Spearman(J1) − Spearman(J0): median +0.08, **95% interval [−0.01, +0.18]**. The improvement is not established.
- **The > 5 mm set is 10 cases, 6 independent, all iid range noise** (noisy_0p2mm/0p5mm and their truncations). Noise in every sweep moves every fold, which is the one failure class a median cannot miss. The set therefore cannot test the property claimed for it. There are no live or disturbed cases above 5 mm.
- **Spearman is carried by the noise cases.** On the 25 "natural" cases (live, clean and disturbed references; error 0.36–1.59 mm), both statistics are uninformative: Spearman J0 −0.40, J1 −0.16.
- **Where it really helps.** Median J/err for error ≤ 3 mm, J0 → J1: HP5 3.24 → 2.28, HP5r 1.81 → 1.07, HP3 0.54 → 0.54, HP4 0.76 → 0.52. The false-"poor" fix on HP5/HP5r is real and reproduced. HP4 under-reporting gets worse, though that machine's error is the known shared D.z bias that no jackknife sees.
- **Independent corpus** (84 distinct replayed runs, final fits): Spearman J0 0.65, J1 0.57.
  - HP3 0.82 / 0.82, HP4 0.57 / 0.52, HP5 0.64 / 0.71, HP5r 0.64 / 0.61.
  - Among final fits with err > 3 mm, "not poor": J0 20, J1 22.
  - "Poor" at ≤ 1.5 mm: 0 / 0, because this corpus is mostly old equal-z data.
  - So the 2026-11-18 "+0.64" property survives for J0 and is slightly *lower* for J1 outside HP5. Treat these corpus numbers with care (limitations).

## Q4. Small fold counts (3–5)

- **n < 3**: returns None (MIN_SWEEPS = 3), unchanged.
- **n = 3**: MAD = min(|a−m|, |b−m|), the distance from the median fold to its *nearer* neighbour. One outlier among three is ignored entirely. The estimator is very noisy: for clean normal folds J1/J0 has a 5–95% range of **0.09–1.74**, so it can read near zero by chance. It is well defined but uninformative. A 3-sweep "good" label from J1 means little.
- **n = 4**: the median averages the middle two. It hides 1 outlier, or 3 that agree; 2 outliers are exposed (25 mm in the table above). This is a discontinuous dependence on the count.
- **n = 5**: hides 2 loose folds. Clean-fold J1/J0 5–95%: 0.28–1.64.
- MAD = 0 is possible when ≥ half the folds coincide in a coordinate. Taking the max over coordinates makes that harmless unless every coordinate collapses.
- The unit test checks the normal-consistency constant at n = 400 and one loose fold at n = 5. It does not cover the 3–5 fold regime where the label is used, the n = 3 noise, or the 2-of-5 case.

## Strongest alternative explanation, and an alternative fix

The submission's diagnosis is sound (E10): the inflated HP5 values come from one fold that loses a low anchor's only height/radius constraint. That is an *identifiability* defect of the fold, not an outlier in the data.

A median cannot tell it apart from a contaminated sweep whose fold is the only clean one. A targeted fix keeps the classical spread and treats only the rank-deficient folds differently. Examples:

- drop or flag folds whose refit loses a held anchor entirely, a condition known from `fixed_anchors` before fitting;
- or report "unknown/limited" for coordinates held by a single unit;
- or have the planner hold every low anchor in ≥ 2 units.

This would keep the 2 misses above while still removing the HP5 false alarms.

## Required changes (for "accept")

1. Narrow the docstring in `autocal/anchor_uncertainty.py` and the catalog line:
   - "still calls every case off by more than 5 mm poor" was shown only for noise spread over every sweep;
   - a single bad sweep (especially one solely holding an anchor) can read "usable";
   - drop or qualify the Spearman improvement claim (not significant).
2. State in the report's limitations that the robust spread is blind to single-fold contamination, and cite the HP3 sweep-8 and HP5r sweep-0 injections.
3. Note the `newest_trusted_index` interaction: the 10× ratio guard is now tighter in practice. Either accept the pass3/reduced regression knowingly or revisit the ratio.

## Additional test most likely to change this judgment

Run live HP5 and HP5r with a real one-sweep disturbance (a line slip or encoder mismatch during a single sweep, ideally the sweep that solely holds a low anchor), with several seeds, and include a patience-ending configuration.

- If J1 calls those runs "usable/good" while the error is above 3–5 mm, the change should be reworked into the identifiability-aware variant above.
- If the fit's robust loss absorbs such disturbances (error stays low, as it did for most HP3 injections), the caveats here are mostly theoretical and the change is a clear net gain.

## Limitations of this review

- **Injection model.** The single-sweep anchor-move injection is synthetic (delta resynthesis). It is not a simulated slip.
- **Harness fits differ from production.** The corpus replay uses the E11 harness fit on dataset prefixes, not each run's production plan (filtering, spool fits, `dataset_for_estimation`), and treats every iteration as admissible. Some backfill runs give ~1–4 m harness errors: their truth or geometry does not match `firmware_initial`, or the blind seed fails on the prefix. These inflate the >10 mm list and make the corpus Spearman values indicative only. Both statistics flagged those runs (> 200 mm).
- **Truth source.** Truth was `input.json` where present, otherwise manifest `firmware_initial` M669, as in the 2026-11-18 study.
