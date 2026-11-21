# Uneven low-anchor heights: free every low z, fix the gauge at the top anchor

Investigation 2026-11-20/21, branch `t3/uneven-low-anchor-heights`. The evidence comes in three kinds:

- **Synthetic/offline:** refits of the committed `benchmark_923c2baa_*` reference sweeps after delta resynthesis.
- **Information analysis:** the Jacobian of the point-fit residuals.
- **Live:** headless native-Rust HP3 and HP5 simulations with RRF firmware, with low anchors moved in the scene. The firmware config still states equal heights.

There is no hardware evidence. Radius variation, line buildup and collection patterns are out of scope.

Part A diagnoses the old equal-height fit. Part B follows the user's decision, made mid-investigation, on the frame convention:
- origin at home;
- top anchor straight above home (top x = y = 0);
- A.x = 0;
- every low-anchor z free.

The decision implements that convention in code and measures it.

## Summary

**Code change (Part B)**
- One Hangprinter gauge is left (`theoretical_ellipse._ANCHOR_OPT_MODE_HANGPRINTER_VERTICAL_TOP`). The reduced vector is A.y, A.z, the other low anchors' xyz, and top z: 3n − 3 parameters.
- It is shared by the point fit, ellipse fit, ellipsoid fit, rough bootstrap, pose planning and the JAX ellipse objective.
- The shared and fixed low-z modes, `--low-anchor-z`, `_low_anchor_z` and all their plumbing are deleted: 33 files, +174/−482 lines.
- `pytest autocal/tests tests/python` passes (708 tests after the review follow-up); no physics code changed.

**Which six constraints pin the gauge** (§B1)
- **Rotation (3):** A.x = 0, top x = 0, top y = 0.
- **Translation (3):** the home pose is the origin, because every range is |a − p| − |a| plus a home offset. Only the last n − d lines carry fitted offsets, so the first d = 3 lines' zero offsets pin the translation.
- **Check:** the Jacobian over all 3n − 3 anchor parameters plus n − d offsets has no null direction. Smallest singular values are 0.10 (HP3), 0.08 (HP4) and 0.03 (HP5/HP5r), against 3.9e-9 for the two tilt directions when low z was free under the old A.x-only gauge.

**Accuracy against truth in the convention frame** (§B2, E6). One low anchor (A) moved by h, same datasets, old fit against new fit:

| Machine | Old fit, h = 0 → ±20 mm | New fit, h from −20 to +20 mm |
| --- | --- | --- |
| HP3 | 0.94 → 13.2 / 14.4 | 0.97–1.02 |
| HP4 | 0.69 → 13.6 / 13.3 | 0.69–0.70 |
| HP5 | 0.86 → 38.7 / 40.0 | 1.00–1.06 |
| HP5r | 1.32 → 45.1 / 48.5 | 1.56–1.63 |

- **Cost at equal heights:** HP3 +0.05 mm, HP4 0.00, HP5 +0.17 mm, HP5r +0.27 mm.
- **Live** (§B3):
  - **HP5 with A raised 5 mm failed outright under the old code** (`unreliable-rough-bootstrap`). It completes under the new code with 0.78 mm applied error; HP5 base is 0.69 mm (old code 1.04 mm).
  - **HP3 with one or two low anchors off by 10–20 mm** gives 0.89–1.42 mm under the new code (4 cases), against 7.5–13.4 mm under the old. HP3 base is 1.35 mm, against 1.17 mm.

**D.z / stiffness coupling under the new gauge** (§B4, E7)
- **Unchanged.** On HP3/HP4 the new gauge is an exact reparameterisation of the old one: same 9 parameters, condition 83–103.
- D.z still trades against a global stretch-correction scale k: corr 0.99 / 1.00 with k free, ∂D.z/∂k = 1.03 mm (HP3) and 1.77 mm (HP4) per unit scale, D.z std 0.81 / 2.67 mm.
- On HP5/HP5r the extra parameter barely changes it: corr 0.01 / 0.43, ∂D.z/∂k 0.02 / 8.3 mm, max coordinate std 0.50 → 0.54 mm and 0.49 → 0.60 mm with k prior σ = 0.2.
- Freeing low z neither creates nor cures the D.z/compliance degeneracy studied in this program's other investigation.

**What Part A established** (still true, now moot for printing)
- **HP3/HP4:** under the old equal-z fit a wrong low height is an exact gauge. The fit returned truth rigidly tilted, with error 0.68 × |h|, while rms and jackknife stayed identical, so no range diagnostic can see it.
- **HP5/HP5r:** equal z was a real wrong constraint. Error was 2.0–2.4 × |h|, and the jackknife flagged it on HP5 (0.66 → 5.8 mm at ±2 mm). An F test of equal z against a free warp detected ≥ 1–2 mm.

**After review (§B6–B8)**
- **HP3 equal heights (1.17 → 1.34 mm) is a fit effect, not collection divergence.** The free low z carry the ≈ −0.9 mm stretch bias unevenly (B.z −1.18 mm). It is not systematic: over 6 noise seeds, old 1.01 vs new 0.96 mm.
- **"Poor quality" was typical on HP5 under the free gauge.**
  - Of 9 new-code HP5/HP5r runs at equal heights (true error 0.36–1.12 mm), 4 printed "poor" (jackknife 3.3–6.8 mm).
  - Cause: one leave-one-sweep-out fold that loses a low anchor's only height/radius constraint.
  - **Fix:** the classical jackknife, except that a fold which *alone* loses a coordinate's constraint is left out of that coordinate's spread. Which folds lose what is read from the poses' information, not from the fold estimates. The affected coordinates are printed as resting on a single sweep.
  - **Test results:**
    - (a) false "poor" falls from 4 of 9 to 1 of 9 sub-mm HP5/HP5r runs;
    - (b) all 56 single-sweep corruptions keep the classical label, 0 of 22 above 5 mm missed;
    - (c) no patience stop or applied fit changes in the 84-run corpus replay.
  - **Rejected:** a median spread, tried first. Review 8457f4ee9842 showed it hides the one clean fold when a single sweep is corrupted, calling real 5–6 mm failures "usable" (2 of 22), and that it worsened one applied fit (2.94 → 4.62 mm).
- **HP4 live with A raised 10 mm:** 1.04 mm, "good", jackknife 0.76 mm. HP4 base is 0.87 mm, and old-code offline was ≈ 6.6 mm at h = 10. So the HP4 equal-strain home model works with uneven heights.
- **Top line off plumb by 50 mm** (26 mrad) moves the anchors by ≤ 0.09 mm through the stretch model's gravity direction. That is negligible.

## Part A — the old equal-height fit

### A1. Mechanism

Every range is a length change from home, so rotating all anchors about home changes no range. Three low anchors always span a plane. Any HP3/HP4 with uneven low heights is therefore congruent to an equal-height machine, tilted by θ ≈ h / 2850 mm (0.2° per 10 mm).

The old fit fixed the gauge with A.x = 0 and equal low z. It returned that tilted copy, with error θ·|a|, about 0.67 h at the top anchor and mostly horizontal.

With four low anchors, the height pattern orthogonal to {1, x, y} (the warp) cannot be produced by a rotation. Forcing it to zero distorts the shape.

### A2. Size of the error (E1: refits; E4: live)

Full tables: [`results/tables.md`](results/tables.md). All values in mm.

| h (A) | HP3 err / rotation-aligned / rms / jackknife | HP4 err / rotation-aligned | HP5 err / jackknife | HP5r err / jackknife |
| --- | --- | --- | --- | --- |
| 0 | 0.94 / 0.92 / 0.032 / 0.94 | 0.69 / 0.69 | 0.86 / 0.66 | 1.32 / 1.17 |
| +2 | 2.24 / 0.92 / 0.032 / 0.94 | 1.23 / 0.69 | 3.79 / 5.83 | 4.86 / 1.81 |
| +10 | 7.62 / 0.93 / 0.032 / 0.95 | 6.61 / 0.69 | 19.7 / 29.8 | 24.1 / 10.0 |
| +20 | 14.4 / 0.95 / 0.032 / 0.95 | 13.3 / 0.68 | 40.0 / 60.0 | 48.5 / 20.6 |

**Live HP3, old code** (8 runs). Applied error was:

| Case | Applied error |
| --- | --- |
| B +5 | 4.3 |
| A −10 / A +10 | 7.0 / 7.5 |
| C −20 | 13.4 |
| A +30 | 21.2 |
| A +10, B −10 | 11.5 |
| B +20, C +20 | 13.3 |

- The applied anchors equalled the predicted levelled truth to 0.95–1.80 mm (base run 1.17 mm).
- Jackknife was 0.39–1.14 mm (base 0.70 mm) and rms 0.024–0.027 mm (base 0.022 mm), so neither flagged the error.

### A3. Identifiability with free low z under the old A.x-only gauge (E2)

- **No range information about tilt.** Two exact null directions (roll and pitch about home) appear on every machine.
- **Gravity is a weak reference.** Gravity, through the stretch model's equilibrium tensions, gives a tilt std of only 22 mrad (HP3), 69–88 mrad (HP4) and 45–106 mrad (HP5/HP5r), with W = 0.081 kg. A 10 mm height error is 3.5 mrad.
- **Force balance is not a practical substitute.** It would need unbiased per-line tensions to about 10 mN (analytic estimate, not simulated).

Part B's convention makes this moot: tilt is defined by the top anchor, not by gravity.

### A4. HP5 model test (E3)

A free-warp refit with home-offset corrections kept intrinsic error flat:
- 1.0 mm on HP5 and 1.6 mm on HP5r for h from −20 to +20 mm;
- equal z gave 3.7–45 mm.

An F test of equal z against free warp with F > 20 flagged the wrong height:
- **False alarms:** 0 of 10 equal-height datasets.
- **HP5:** every ±1 and ±2 mm case.
- **HP5r:** +2 mm always; B at −2 mm only 1 of 4 times, because of a −0.2 to −0.5 mm baseline warp in the simulated HP5r.

Part B supersedes this, since the warp is always free there and the equal-height cost is small.

## Part B — the user's convention, implemented

### B1. Gauge and parameterisation

[`autocal/theoretical_ellipse.py`](../../../../autocal/theoretical_ellipse.py):
- `canonicalize_anchor_gauge` rotates about home so the top anchor is on +z, then turns about z so A is on −y. The existing slideprinter/HP xy routine handles that second step, including its mirror rule.
- `anchors_matrix_to_opt_vec` returns `[A.y, A.z, B.xyz, C.xyz, (D.xyz), top.z]`; `anchor_opt_vec_to_matrix` inverts it.
- Bounds come from the same per-machine boxes; the low-z and top-z half-spaces are unchanged.
- The JAX ellipse objective ([`ellipse_objective_jax._anchors_from_opt_vec`](../../../../autocal/slideprinter_2d/ellipse_objective_jax.py)) mirrors the layout. Tests check both against each other on uneven anchors.

Why exactly six gauge constraints:
- **Rotation (3).** Rotations about home are the only exact symmetry of length-change data, and the three coordinate constraints remove them. A direction (top) is pinned by 2, and one turn about it by A.x.
- **Translation (3).** A translation t changes every reference length |a_i + t|.
  - Only the last n − d lines have free home offsets. The first d lines' reference lengths therefore pin t: 3 conditions for 3 unknowns, generically regular.
  - The HP4 equal-strain home model sets the ABC offsets from compliance rather than fitting them, so it adds no freedom.
- **Numerical check (E7).** No null direction remains; the smallest singular values are as in the summary.

### B2. Accuracy in the convention frame (E6, refits)

Scripts: [`e6_gauge_compare.py`](scripts/e6_gauge_compare.py), run with the old code (main checkout at the branch base, read-only) and the new code on identical datasets. Truth has the top anchor at x = y = 0, so it already is in the convention frame. Results: `results/e6_{old,new}_<machine>.json`.

| Machine | Old err at h = 0 / +2 / +10 / +20 | New err at h = 0 / +2 / +10 / +20 | New jackknife | New low-z error at h = 0 (A, B, C, D) |
| --- | --- | --- | --- | --- |
| HP3 | 0.94 / 2.24 / 7.62 / 14.4 | 0.99 / 0.99 / 1.00 / 1.02 | 0.94–0.95 | −0.68, −0.99, −0.76 |
| HP4 | 0.69 / 1.23 / 6.61 / 13.3 | 0.69 / 0.69 / 0.69 / 0.69 | 0.55 | −0.07, +0.20, +0.19 |
| HP5 | 0.86 / 3.79 / 19.7 / 40.0 | 1.03 / 1.03 / 1.01 / 1.00 | 1.08–1.10 | −0.02, +0.28, +0.22, −0.25 |
| HP5r | 1.32 / 4.86 / 24.1 / 48.5 | 1.59 / 1.59 / 1.58 / 1.56 | 1.12–1.17 | −0.37, −0.06, −0.39, −0.30 |

All values in mm.

- **Negative h** behaves the same (see the JSON).
- **New rms equals old rms at h = 0** (HP3 0.032, HP4 0.024, HP5 0.030, HP5r 0.027 mm). It stays there at every h.
- **The HP3 low anchors sit 0.7–1.0 mm low.** That was already the case in the old fit (−0.81 mm shared), so it is the known stretch/D.z bias, not the gauge. The benchmark harness's convention-frame baseline (HP3 0.71–1.68 mm over 15 runs, dominated by a −0.5 to −1.2 mm common z shift) agrees.

### B3. Live confirmation (E4 on the new code)

Same runner, same scene edits; applied firmware anchors against truth. See `runs/new_*/summary.json`.

| Run | Old code: applied err, outcome | New code: applied err, outcome | Jackknife old → new |
| --- | --- | --- | --- |
| HP3 base | 1.17 mm, 7 sweeps, 4587 s | 1.35 mm, 7 sweeps, 4585 s | 0.70 → 0.68 mm |
| HP3 A +10 | 7.53 mm, 8 sweeps, 4909 s | 0.95 mm, 10 sweeps, 5687 s | 1.02 → 0.54 mm |
| HP3 C −20 | 13.44 mm, 8 sweeps, 5044 s | 0.89 mm, 8 sweeps, 5045 s | 0.48 → 0.46 mm |
| HP3 A +10, B −10 | 11.52 mm, 7 sweeps, 4728 s | 1.25 mm, 7 sweeps, 4723 s | 0.46 → 0.46 mm |
| HP3 B +20, C +20 | 13.30 mm, 8 sweeps, 4952 s | 1.42 mm, 9 sweeps, 5329 s | 0.43 → 0.42 mm |
| HP5 base | 1.04 mm, good-fit (5 sweeps, 3778 s) | 0.69 mm, good-fit (5 sweeps, 3781 s) | 1.62 → 3.28 mm |
| HP5 A +5 | **failed**: `unreliable-rough-bootstrap` | 0.78 mm, good-fit (5 sweeps, 3793 s) | — → 2.37 mm |

Every new-code run stopped normally: `stable-estimate` on HP3, `good-fit` on HP5. The full table is [`results/live_new.md`](results/live_new.md).

- **HP3:** uneven cases give 0.89–1.42 mm and the base 1.35 mm, against 7.5–13.4 mm under the old code.
- **The remaining error is mostly D.z.** The refit D.z error equals the worst coordinate in 4 of 5 HP3 runs (−0.57 to −1.42 mm). That is the known stretch bias.
- **The HP3 equal-height difference (1.17 → 1.35 mm) comes from the fit, not from the collection.** The independent review found this and I reproduced it (E8, [`results/e8_*.json`](results/)). Each code gives the same answer on either live HP3 base dataset:

  | Live HP3 base dataset | Old fit | New fit |
  | --- | --- | --- |
  | `base_0` (collected by old code) | 1.170 mm, low z −0.97 × 3 | 1.338 mm, low z −0.89 / −1.18 / −0.83 |
  | `new_base_0` (collected by new code) | 1.176 mm | 1.343 mm |

  The old fit spread the ≈ −0.9 mm stretch/D.z bias over one shared z. The free low z carry it unevenly, with B.z at −1.18 mm.
- **It is not a systematic loss.** Over 6 range-noise seeds (0.02 mm) on the benchmark dataset at equal heights, HP3 averages 1.01 mm old against 0.96 mm new (ranges 0.94–1.13 and 0.90–1.02), and HP4 0.67 against 0.67 mm (`results/e6_*_noise.json`). The equal-height cost on HP3/HP4 is ±0.2 mm and depends on the dataset: the bias is redistributed, the mean does not shift.
- **HP5 base** is 0.35 mm better with the new code. That is one run and within the ±0.5 mm single-run knife-edge.
- **Two HP3 runs collected more:** A+10 took 10 sweeps and 5687 s against 8 / 4909 s, and B+C+20 took 9 sweeps against 8. Fits steer planning, so those collections diverge. This is one run each, not evidence of a systematic time cost.

Why the old HP5 A+5 run failed:
- The equal-z rough bootstrap fitted rms 0.079 mm, against 0.028 mm at h = 0.
- Its geometry then had no reachable equal-strain positions with positive tensions.
- So a 5 mm height difference on HP5 was a hard failure of the old code, not just an accuracy loss.

### B4. D.z / stiffness coupling (E7)

[`e7_conditioning.py`](scripts/e7_conditioning.py) computes a Jacobian at truth over:
- the code's reduced anchor vector;
- the home offsets (5 mm prior);
- a global stretch scale k, which multiplies the gravity-aware stretch correction of every range, forced on with the recorded sensor forces.

It is evaluated at the fit's noise scale (0.02 mm). Outputs: `results/e7_{old,new}.json`.

| Machine (gauge) | Anchor params | Condition | D.z std, k fixed / k prior σ = 0.2 / k free (mm) | corr(D.z, k), k free | ∂D.z/∂k (mm per unit) |
| --- | --- | --- | --- | --- | --- |
| HP3 old / new | 9 / 9 | 95 / 83 | 0.10 / 0.23 / 0.81, both | 0.992, both | 1.03, both |
| HP4 old / new | 9 / 9 | 103 / 93 | 0.22 / 0.41 / 2.67, both | 0.997, both | 1.77, both |
| HP5 old / new | 11 / 12 | 226 / 221 | 0.31 / 0.31 / 0.31, both | 0.02 / 0.01 | 0.05 / 0.02 |
| HP5r old / new | 11 / 12 | 235 / 257 | 0.33 / 0.36 / 0.36, both | 0.42 / 0.43 | 8.3, both |

- **HP3/HP4:** numbers are identical up to rounding, as an exact reparameterisation must give.
- **HP5/HP5r:** the warp parameter costs 0.04–0.11 mm of max coordinate std.
- **Benchmark truth is level here,** so this table measures conditioning, not bias. Bias was measured in B2.

### B5. Printing frame

- **The convention needs no level reference.** Firmware z is parallel to the top line at home, and builders can make that vertical with a plumb line. Tilt of the whole frame is, by the user's decision, not scored.
- **The first layer still needs bed levelling or probing.** That is unchanged and outside autocal's scope.
- **Raising low anchors during a tall print and recalibrating keeps the frame,** because the top anchor and home define it.

### B6. HP5/HP5r quality label: typical over-report, and a structure-aware jackknife

The review asked for 3–5 more live HP5 and HP5r runs at equal heights. The simulator is deterministic, so I made distinct equal-height machines:
- `seed=N` in [`e4_live.py`](scripts/e4_live.py) moves every low anchor except A by a seeded uniform ±20 mm in x and y;
- A moves only in y, so A.x stays 0;
- heights and the top anchor are unchanged;
- truth is the moved scene in the convention frame;
- the firmware config keeps the nominal anchors.

New code, before the fix (`runs/new_hp5*`):

| Run | Stop | Sweeps | Sim s | Applied err | Jackknife | Label | Low anchors held per sweep |
| --- | --- | --- | --- | --- | --- | --- | --- |
| HP5 base | good-fit | 5 | 3781 | 0.69 | 3.28 | poor | A B C C D |
| HP5 seed 1 | good-fit | 5 | 3703 | 0.77 | 6.77 | poor | A B C C A |
| HP5 seed 2 | good-fit | 5 | 3674 | 0.79 | 5.14 | poor | A B C C A |
| HP5 seed 3 | good-fit | 5 | 3686 | 0.91 | 1.45 | usable | A B C A D |
| HP5 seed 4 | good-fit | 5 | 3714 | 0.84 | 2.71 | usable | A B C C D |
| HP5r base | good-fit | 6 | 4684 | 1.12 | 0.89 | good | A B C D A C |
| HP5r seed 1 | good-fit | 5 | 3979 | 0.36 | 1.37 | usable | A B C C D |
| HP5r seed 2 | good-fit | 5 | 3995 | 0.87 | 1.57 | usable | A B C A D |
| HP5r seed 3 | good-fit | 5 | 3959 | 1.01 | 4.27 | poor | A B C C D |

Errors and jackknife in mm. The HP5 base label was not captured by that older runner version; the reviewer read "poor" from its console.

- **"Poor" is typical on HP5** (3 of 5) and occasional on HP5r (1 of 4). Every run stopped normally on `good-fit`. The over-report touches only the label, not accuracy or stopping.
- **The cause is one fold** (E10, [`results/e10_jackknife_parts.json`](results/e10_jackknife_parts.json)). Leaving out one of five sweeps can remove the only constraint on one anchor's radius/height pair, or on top-z height diversity. That fold then moves a coordinate by 4–9 mm while the other folds move it by ≤ 2.6 mm.
  - HP5 base: leaving out sweep 0, the only sweep holding A, moves A.y by 3.9 mm.
  - HP5 seed 1: leaving out the second C sweep moves the top z by 8.75 mm.
  - Equal z used to tie each low anchor's height to the others, which hid this.
- **The fold is not wrong.** That leave-one-out fit really is 3–9 mm off. With 5 folds, though, one fold dominates the classical root-mean-square spread.

**Rejected: a median spread** (commit e292f73e, reverted).
- **What it was:** √(n−1) · 1.4826 · median |fold − median fold| over the same folds. On my 41 cases (E11) it removed most false "poor" labels.
- **Why it failed:** independent review 8457f4ee9842 showed it is blind by construction to a failure that shows in one fold. A single corrupted sweep produces exactly one deviant fold: the one that leaves the bad sweep out, which is the clean estimate. The median discards it.
  - In the reviewer's 56 single-sweep injections, it called 2 of 22 failures above 5 mm "usable": HP3 sweep 8 (5.06 mm error, 2.63) and HP5r sweep 0, the sole holder of A (5.71 mm error, 2.60).
  - A 10 mm HP3 failure read 4.84.
  - Its lower minima tightened the 10× guard in `newest_trusted_index`. On the patience-ending corpus run `extra-sensor-production/pass3/reduced` the applied fit worsened from 2.94 to 4.62 mm.
  - The Spearman gain was not significant (cluster bootstrap 95% interval [−0.01, +0.18]).
- **Where my E11 claim came from:** the "> 5 mm still poor" result held only because none of the 41 cases was a localized failure.

**Adopted: structure-aware classical spread** ([`autocal/anchor_uncertainty.py`](../../../../autocal/anchor_uncertainty.py): `sweep_jackknife`, `jackknife_spread_mm`, `fold_information_ratios`).
- **Which folds lost a coordinate:** taken from the poses, not from the fold estimates.
  - Rigid per-point range information at the full fit's poses (`pose_planning.point_informations`), with the fit's home-offset prior, gives each fold an information-implied standard deviation for every anchor coordinate.
  - Ratio = fold std / full-data std. Leaving out an average one of n units gives about √(n/(n−1)).
  - On the recorded runs, ordinary folds mostly stay at 1.2–3.7. A fold that loses a coordinate's only constraint reaches 4.3–13.7. For example, HP5 seed 1 drops the second C sweep, and the top z ratio is 8.75; that is the cell that moved 8.75 mm in E10.
- **Rule:** if *exactly one* fold has ratio > 3 for a coordinate, and at least 3 folds remain, that coordinate's classical spread is computed without that fold. Every other coordinate, including one that several folds lose (weak everywhere), keeps the classical spread over all folds.
- **What it cannot do:** it cannot tell a sole-holder sweep's corruption from its structural loss, any more than any statistic over these folds can. So it does not pretend. The affected coordinates are returned and printed after the uncertainty line, e.g. "; A.y, I.z rest on a single sweep: the refit without it lost them, so their spread comes from the other refits and that sweep is not cross-checked". They are also logged as `single_unit_coordinates`.
- **Unchanged:** thresholds, `newest_trusted_index` and patience.

**A rejected first structural variant:**
- **What it did:** excluded *every* flagged fold, reporting 0 when fewer than 2 folds remained.
- **What went wrong:** it changed 15 corpus decisions, 6 for the worse. Early 3–4-sweep fits lose coordinates in many folds, so their values fell toward 0. That treated "unmeasured" as "certain", won patience ranking and tightened the 10× guard.
- Restricting exclusion to a single losing fold removed all of these changes.

Evaluation (E12, [`results/e12_eval.json`](results/e12_eval.json); folds and ratios stored in `results/e12_*.json`). The reviewer's injections were regenerated with their recipe: the held low anchor moves radially by 10 or 20 mm for one sweep's ranges. The regenerated errors equal theirs. The corpus folds are the reviewer's own (`folds.jsonl`); only the information ratios were added.

| Test | Classical (before) | Median (e292f73e, rejected) | Structure-aware (adopted) |
| --- | --- | --- | --- |
| (a) false "poor" on 9 sub-mm HP5/HP5r live runs | 4 / 9 | 0 / 9 | **1 / 9** |
| (b) 56 single-sweep injections: "poor" lost relative to classical | — | 2 | **0** |
| (b) injected failures > 5 mm not called "poor" | 0 / 22 | 2 / 22 | **0 / 22** |
| (b) "poor" on injected cases ≤ 1.5 mm | 6 | 1 | 6 |
| (c) 84-run corpus: patience-stop or applied-fit changes | — | 3 (1 worse, 2.94 → 4.62 mm) | **0** |

The structure-aware values for (a) (mm; true error in brackets):

| Machine | Base | Seed 1 | Seed 2 | Seed 3 | Seed 4 |
| --- | --- | --- | --- | --- | --- |
| HP5 | 3.28 → 2.22 [0.68] | 6.77 → 2.39 [0.77] | 5.14 → 2.50 [0.79] | 1.45 → 1.42 [0.78] | 2.71 → 2.63 [0.84] |
| HP5r | 0.89 → 0.89 [1.12] | 1.37 → 0.67 [0.36] | 1.57 → 1.57 [0.87] | 4.27 → 4.27 [1.01] | — |

- **The remaining "poor":** HP5r seed 3's worst coordinate is D.x. Two folds lose it (ratios 3.29 and 4.29, the two C sweeps), so it is jointly carried, not single-sweep supported. Letting up to 2 folds drop also passed (a)–(c) and cleared it (E12, `E12_MAX_LOST=2`). I kept the single-fold rule because it is what "only constraint" means and it is less tuned to these 9 runs.
- **The two review misses** now read 5.23 (HP3 sweep 8, 5.06 mm error) and 5.95 (HP5r sweep 0, 5.71 mm error), both "poor", identical to classical. In both, no fold lost a constraint, so nothing was excluded.
- **Production matches the research helper exactly** on all of (a) and on the two misses ([`results/e12_production_check.json`](results/e12_production_check.json)). HP3/HP4 live runs and HP5r base have no single-sweep coordinates and keep their classical value.
- **What it does not do:** it does not reduce false "poor" on injected cases ≤ 1.5 mm (6 = classical). Those come from corrupted sweeps whose robust fit absorbed the corruption; the folds really do disagree there.
- **Live behaviour:** stopping (`good-fit`, `stable-estimate`) never reads this value. The robust_* live runs (median) showed identical collections with the label alone changing. The adopted rule changes no corpus decision, so I made no new live runs.

### B7. HP4 live with an uneven low anchor

| Run | Stop | Sweeps | Sim s | Applied err | Jackknife | Label | Refit low-z error (A, B, C) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| HP4 base (new code) | stable-estimate | 7 | 4926 | 0.87 | 0.42 | good | — |
| HP4 A +10 mm (new code) | stable-estimate | 7 | 4895 | 1.04 | 0.76 | good | +0.28, −0.01, −0.09 |

Errors in mm. The old fit gives ≈ 6.6 mm at h = 10 on HP4 (E6 offline). With the new fit, the U = 2 equal-strain home model (`_hp4_home_strain`) works with uneven heights: the raised anchor's z is recovered to 0.3 mm.

### B8. Top line off plumb (offline, E9)

Under the convention, z runs along the top line, and the stretch model puts gravity along −z. A top anchor δ off plumb at height H tilts real gravity by θ = δ / H in that frame. Only fits that use the stretch model are affected (HP5/HP5r, HP4 equal strain; the HP3 fit has none).

[`e9_off_plumb.py`](scripts/e9_off_plumb.py) refits the same data with the model's gravity tilted by θ. Results ([`results/e9_off_plumb.json`](results/e9_off_plumb.json)), anchor movement against the plumb fit:

| Dataset | δ = 10 mm (5.3 mrad) | 25 mm (13 mrad) | 50 mm (26 mrad) |
| --- | --- | --- | --- |
| Benchmark HP5 | 0.009–0.018 | 0.022–0.044 | 0.044–0.088 |
| Benchmark HP5r | 0.003–0.005 | 0.008–0.014 | 0.015–0.027 |
| Live HP5 base | 0.008–0.012 | 0.021–0.030 | 0.042–0.060 |

All values in mm; the range is over tilt azimuth (0° and 90°).

The movement is linear in δ and at most 0.09 mm at 50 mm, so an off-plumb top line costs no meaningful accuracy through this assumption. This tests only the fit's gravity assumption. A real off-plumb machine also has a correspondingly different physical stretch, which this replay keeps.

## Interface changes for the benchmark harness

- **CLI:** `--low-anchor-z` is removed from `autocal.py`, `--headless-sim` runs and the `slideprinter_2d.calibrate` ellipse sub-command. Passing it is now an argparse error.
- **Logs:** the full-auto run-settings dict (`base_solver`, copied into each run's settings in the full-auto log rows) no longer has a `low_anchor_z` key. No other summary or log field changed.
- **Applied M669:** low anchors may now have different z, and the top anchor's x and y are 0. Under the old code, low z were equal and top x/y were free.
- The fitted frame is the harness's convention frame, so `max_abs_frame_error_mm` and raw error coincide when truth's top anchor is vertical.
- **Anchor uncertainty** (the console line "Anchor uncertainty (mm, lower is better)", `anchor_uncertainty_mm` in the log rows, and the quality label) is the classical spread except for coordinates resting on a single sweep (§B6). The value is lower only where one fold alone lost a coordinate; the format is unchanged.
  - New: iteration log rows carry `single_unit_coordinates` (e.g. `["A.y", "I.z"]`), and the console summary prints them when present.
- Internal API: `low_anchor_z` is gone from every fitting, planning and parameterisation function. Datasets carrying `_low_anchor_z` are no longer read.

## Limitations and what would change the conclusions

- **Simulation only.** The benchmark truth's top anchor is exactly above home, so frame error equals raw error. A tilted top line on a real machine shifts the frame by design, not by error.
- **One deterministic run per live case.** The HP5/HP5r "seeds" are distinct equal-height machines (±20 mm xy anchor moves), not repeated noise draws. The offline E6 noise seeds (6 per machine, equal heights) cover HP3/HP4 only.
- **The structure-aware jackknife** is tuned on 9 live HP5/HP5r runs, checked on 56 synthetic single-sweep corruptions and replayed on 84 corpus runs.
  - **Unvalidated cases:** live single-sweep failures (a slip or encoder mismatch during the sweep that alone holds a low anchor) were not simulated.
  - **What it cannot verify:** a coordinate resting on a single sweep is printed as not cross-checked.
  - **Threshold:** the ratio limit 3 is set from the observed gap (ordinary folds mostly ≤ 3.7, lost constraints ≥ 4.3); a single ordinary fold above 3 is excluded only for coordinates no other fold loses. Every limit from 2.5 to 4 gave the same (b) and (c).
  - **Calibration:** HP5 values still sit 2–3× above the true error.
- **Uneven heights live** were tested on HP3 (4 runs), HP4 (1) and HP5 (1), not on HP5r.
- **HP5r costs 0.27 mm at equal heights** with free warp (1.32 → 1.59 mm offline). A Gaussian prior on the warp (a few mm) could recover part of that. It was not tried, to keep the change small.
- **The level-frame comparison in Part A had a bug** (yaw sign), found and fixed during the investigation. Affected values were recomputed; rotation-aligned errors were never affected.
- **Delta resynthesis** keeps the recorded stretch for the original geometry. The live runs cover the geometry change for HP3 and HP5.

## Evidence

- Scripts: [`scripts/`](scripts/) (run from the repo root with `PYTHONPATH=.`; use the old checkout on `PYTHONPATH` for old-code runs):
  - `lowz_common.py`
  - `e1_equal_z_bias.py`
  - `e2_identifiability.py`
  - `e3_free_low_z.py`
  - `e4_live.py` / `queue_live.sh`
  - `e5_rescore_live.py`
  - `e6_gauge_compare.py`
  - `e7_conditioning.py`
  - `e8_cross_refit.py` (old/new fit on recorded live datasets)
  - `e9_off_plumb.py`
  - `e10_jackknife_parts.py` (per-fold decomposition)
  - `e11_uncertainty_estimators.py` (classical vs median spread against true error; superseded by E12)
  - `structural.py`, `e12_ratio_scan.py`, `e12_folds.py`, `e12_eval.py`, `e12_production_check.py` (structure-aware jackknife against (a)–(c))
  - `tables.py`, `live_new_table.py`
- Results: [`results/`](results/): `e1_*`, `e2_identifiability.json`, `e3_*`, `e6_{old,new}_*` (including `_noise`), `e7_{old,new}.json`, `e8_{old,new}.json`, `e9_off_plumb.json`, `e10_jackknife_parts.json`, `e11_{live,refs,summary}.json`, `e12_{live,inject10,inject20,corpus_0,corpus_1,eval,eval_maxlost2,production_check}.json`, `tables.md`, `live_new.md`.
- Live runs: `runs/<case>/`:
  - `input.json` (command, shifts, truth);
  - `input-scene.usda`;
  - `summary.json`;
  - `sweeps.json`;
  - the headless manifest with revisions and source hashes.
  - Old-code runs are unprefixed; new-code runs are prefixed `new_` (classical jackknife) and `robust_` (with the rejected median spread).
  - Review runs (`new_hp4_*`, `new_hp5*_seed*`, `new_hp5r_*`) were made after the review of `a7a8d179`, on that code. `robust_*` were made on the follow-up commit.
  - The raw headless event logs (`sweeps.headless/events.jsonl`, ~200 MB each, 3.1 GB total) are gitignored and exist only on the workstation that ran them. [`runs/raw_events_manifest.txt`](runs/raw_events_manifest.txt) lists their sizes and SHA-256. No conclusion here depends on them: every number comes from `sweeps.json`, `summary.json` and the manifests.
- Code: this branch. The physics and RRF builds came from the main checkout at `8a02b7d5`, whose src/RRF/autocal/integrations sources equal this branch's base `b249a1a5`. Old-code offline runs imported that checkout's `autocal/`.
