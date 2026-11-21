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
- `pytest autocal/tests tests/python` passes (707 tests); no physics code changed.

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

**Side effect to watch:** on live HP5 the sweep jackknife rose from 1.62 to 3.28 mm while true error fell from 1.04 to 0.69 mm. With 5 sweeps, leaving one out loosens a low anchor's height. The run still stopped normally (`good-fit`, 5 sweeps, same time), but the jackknife now over-reports on HP5.

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
- **Base runs:** the new code is 0.18 mm worse on HP3 and 0.35 mm better on HP5. Both are within the ±0.5 mm single-run knife-edge.
- **Two HP3 runs collected more:** A+10 took 10 sweeps and 5687 s against 8 / 4909 s, and B+C+20 took 9 sweeps against 8. Fits steer planning, so collections diverge. This is one run each, not evidence of a systematic time cost.

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

## Interface changes for the benchmark harness

- **CLI:** `--low-anchor-z` is removed from `autocal.py`, `--headless-sim` runs and the `slideprinter_2d.calibrate` ellipse sub-command. Passing it is now an argparse error.
- **Logs:** the full-auto run-settings dict (`base_solver`, copied into each run's settings in the full-auto log rows) no longer has a `low_anchor_z` key. No other summary or log field changed.
- **Applied M669:** low anchors may now have different z, and the top anchor's x and y are 0. Under the old code, low z were equal and top x/y were free.
- The fitted frame is the harness's convention frame, so `max_abs_frame_error_mm` and raw error coincide when truth's top anchor is vertical.
- Internal API: `low_anchor_z` is gone from every fitting, planning and parameterisation function. Datasets carrying `_low_anchor_z` are no longer read.

## Limitations and what would change the conclusions

- **Simulation only.** The benchmark truth's top anchor is exactly above home, so frame error equals raw error. A tilted top line on a real machine shifts the frame by design, not by error.
- **One deterministic run per live case.** The offline E6 grid covers several h values and machines but only the benchmark collection. Noise-seed variants of E6 were not rerun under the new code; Part A's noisy runs used the old code.
- **The HP5 jackknife inflation** (1.6 → 3.3 mm live, 0.66 → 1.09 mm offline) could make HP5 runs collect more before stopping, or change which fit is trusted (`newest_trusted_index`). Two live HP5 runs show no change in stop time. More runs, and HP4/HP5r live, would settle it.
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
  - `tables.py`
- Results: [`results/`](results/): `e1_*`, `e2_identifiability.json`, `e3_*`, `e6_{old,new}_*`, `e7_{old,new}.json`, `tables.md`, `live_new.md`.
- Live runs: `runs/<case>/`:
  - `input.json` (command, shifts, truth);
  - `input-scene.usda`;
  - `summary.json`;
  - `sweeps.json`;
  - the headless manifest with revisions and source hashes.
  - Old-code runs are unprefixed; new-code runs are prefixed `new_`.
  - The raw headless event logs (`sweeps.headless/events.jsonl`, ~200 MB each, 3.1 GB total) are gitignored and exist only on the workstation that ran them. [`runs/raw_events_manifest.txt`](runs/raw_events_manifest.txt) lists their sizes and SHA-256. No conclusion here depends on them: every number comes from `sweeps.json`, `summary.json` and the manifests.
- Code: this branch. The physics and RRF builds came from the main checkout at `8a02b7d5`, whose src/RRF/autocal/integrations sources equal this branch's base `b249a1a5`. Old-code offline runs imported that checkout's `autocal/`.
