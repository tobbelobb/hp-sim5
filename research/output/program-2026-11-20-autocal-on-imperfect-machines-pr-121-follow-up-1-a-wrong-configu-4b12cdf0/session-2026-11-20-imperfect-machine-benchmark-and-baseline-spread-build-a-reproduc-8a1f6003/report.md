# Imperfect-machine benchmark: default autocal on randomized HP3/HP5 machines

Investigation `session-2026-11-20-imperfect-machine-benchmark-and-baseline-spread-build-a-reproduc-8a1f6003`,
2026-11-20/21, branch `t3/imperfect-machine-benchmark` (new-fit comparison on
`t3/imperfect-machine-newfit` = this branch + `a7a8d179`).
**Simulation evidence only.** Headless native Rust physics with the RRF host firmware,
plus offline kinematic refits. Nothing here has been checked on hardware.

## Summary

- **Harness.** `python -m autocal.tools.imperfect_machine OUT --machine hangprinter_3 --seed N` runs one machine with one command (§2).
  - It runs the default autocal (flat circle sweeps + point fit, the CI benchmark settings) on a scene whose geometry and stiffness differ from config.g.
  - It writes `OUT/result.json` with truth vs fit.
  - On the unperturbed machine it reproduces the CI benchmark exactly: HP3 0.97 mm / 4586.7 s, HP5 1.04 mm / 3778 s.
- **HP3 spread with the old fit (shared low z), 10 random machines.** The machines have low-anchor dz ±30 mm, a top-anchor offset ≤50 mm and true EA 0.8–1.2× config.
  - In the agreed print frame (origin at home, z through D, A.x = 0), the max coordinate error is **1.11 ± 0.35 mm** (range 0.71–1.74 mm). The nominal machine gives 1.18 mm.
  - The per-coordinate sd is 0.13–0.43 mm (§4).
  - Almost all of it is a common downward z shift of every anchor, −0.87 ± 0.15 mm. This is the known home-reference shift.
  - After the best rigid alignment, the error is 0.40 ± 0.21 mm.
- **On three low anchors, uneven low heights are an exact gauge.** Line-length data cannot see them.
  - The shared-z fit returns the true machine rigidly rotated about home so that the low-anchor plane is level.
  - Pairwise anchor distances are exact, and the residual is zero on kinematic data (§3).
  - Raw scene-frame errors are therefore 8–28 mm, while print-frame errors stay at baseline.
  - No residual, jackknife or split score can flag this, and under the convention none needs to.
- **Attribution on HP3.**
  - **Stiffness** is the only perturbation with a measurable effect, and it is small. Softer true lines than config lower all fitted anchors: +0.76 to +1.15 mm of mean low z per unit (k−1), t = 2.1–5.6. That is ±0.15–0.23 mm over ±20%. The effect on D.z is not resolved (t ≤ 2, residual sd 0.34 mm).
  - **Low-anchor dz** has no effect beyond the gauge.
  - **Top offset** up to 47 mm has no effect (it shows up as a frame tilt).
  - The "stiffness → D.z" trade documented for stretch-corrected HP5 and sphere-mesh data does not show on HP3 sweeps. They are equal-strain and use no stretch correction.
- **HP5 (four low anchors) is different.** Uneven heights there are a real misfit for the shared-z fit. The new free-low-z fit removes it: 19.3 → 0.52 mm and 4.3 → 1.00 mm, both stopping at good-fit in ≈3780 s (§7).
  - Random seeds 0 and 3 give 19.3 and 4.3 mm, ran to max-steps (20 iterations, 8580–8635 s, 2.3× nominal) and still applied the result.
  - Those failures are flagged: rms 0.10–0.49 mm vs 0.03 nominal, jackknife 2.4–26.7 mm vs 1.6 mm.
  - Stiffness ±20% alone gives 1.09 / 0.58 mm (nominal 1.00 mm). A 41 mm top offset alone gives 0.25 mm.
- **Robustness finding: force auto-tune knife-edge.**
  - 1 of 10 random HP3 machines (seed 9) failed at bootstrap ("Measured sweep is too short…").
  - The edge-ramp plateau rule fired on two pre-breakaway stiction samples. Their gain was 0.50° against a 0.5° threshold. The 19 HP3 runs that passed had gains of only 0.56–0.69° there, so a nominal machine can hit it too.
  - The fix is commit `1ec95e5e`: the plateau rule now applies only after the previous sample has moved ≥5°. Rerun with the fix, seed 9 gives 0.84 mm.
  - Failure rate before the fix: 1/10 random HP3 machines (1/20 HP3 runs). After the fix: 0/10.
- **New fit (free low z, `a7a8d179`), paired with the same 15 HP3 machines.**
  - Paired difference in print-frame error: −0.04 ± 0.03 mm (se) over all pairs, and −0.06 ± 0.03 mm over the 10 random seeds. New is better in 6/10.
  - Simulated time: +48 ± 33 s. Stopping and the jackknife behave the same: same stop rule (`stable-estimate`), same sweep counts in 12/15 pairs, jackknife 0.40–0.86 vs 0.31–0.96 mm.
  - The reported base regression (1.17 → 1.35 mm) does **not** hold in the print frame: nominal is 1.18 vs 1.19 mm. Raw scene-frame error does rise, 0.97 → 1.19 mm, because the new fit puts D.xy at 0 by construction. This is a frame change, not an accuracy change.

## 1. Question and scope

The question: how much true anchor error does the default autocal leave on randomized imperfect machines, which perturbation drives which coordinate, and do the fit's own scores flag the bad cases?

The perturbations, all applied to the scene only (config.g stays catalogued and nominal):
- each low anchor raised or lowered;
- an off-centre top anchor;
- true line compliance differing from config.g's `M666 S`.

Out of scope: radius variation, line buildup and collection-pattern changes.

## 2. Harness (reusable)

`autocal/tools/imperfect_machine.py`, with tests in `autocal/tests/test_imperfect_machine.py`.

- **Perturbations, drawn per seed** (`numpy.default_rng(seed)`):
  - each low anchor gets dz ~ U(±`--low-dz-max-mm`, default 30);
  - the top anchor gets an XY offset with radius ~ U(0, `--top-offset-max-mm`, default 50) and a uniform direction;
  - every cable's `stiffnessPerUnitLength` is multiplied by k ~ U(1 ± `--stiffness-error`, default 0.2). The true compliance is then 1/k × config.
- **Anchor moves.** Each anchor moves rigidly with its spool and all of its line pivots (top-level `Spool<X>*` / `Anchor<X>*` prims), so guy lengths stay unchanged. Supported machines: all four benchmark Hangprinters (HP4/HP5r since `fc1c0cc4`, see §7b).
- **Overrides.** `--low-dz-mm a:b:c`, `--top-offset-mm x:y` and `--stiffness-scale k` fix one factor while the others keep their seed draws. This allows single-factor attribution on the same machine.
- **Run settings.** Runs use `benchmark_history.run_autocal`, which is the CI benchmark command (patience 8, the same step cap and thread settings), with the new `scene=` override.
- **`--rescore`** recomputes `result.json` from the logs. It works for both the old shared-z fit and the new free-low-z fit, because it reads only `pose_fit_anchors` and the summary `anchors`.
- **Contents of `result.json`:** truth, fit, and the following error columns, each as `<key>_error_mm`, `max_abs_<key>_error_mm` and `worst_<key>_coordinate`:
  - **frame** (primary): truth and fit each rotated about home so the top anchor is on +z and A has x = 0, y < 0. The true tilt is in `truth_frame_tilt_deg`.
  - **raw:** in the scene frame.
  - **gauge:** truth with its low-anchor plane levelled (the shared-z fit's own frame).
  - **rigid:** after the best rotation and translation.
  - It also holds the fit's scores per iteration: rms, chi2, jackknife `anchor_uncertainty_mm`, `max_std_mm`, and the split-fit difference.
- **Batches.** Up to two simulations run at a time with [`queue.sh`](evidence/scripts/queue.sh) and a case list ([`cases.txt`](evidence/scripts/cases.txt), [`newfit_cases.txt`](evidence/scripts/newfit_cases.txt)).
- **Validation.** The unperturbed case reproduces CI run 44794416329 to the second (HP3 0.97 mm / 4586.7 s; HP5 1.04 mm / 3778 s).

## 3. Uneven low anchors are a gauge on three low anchors (kinematic synthetic)

[`geom_synthetic.py`](evidence/scripts/geom_synthetic.py) rewrites the committed HP3 benchmark dataset `benchmark_923c2baa_hangprinter_3` as if the anchors were perturbed, then refits:
- held and drive records are kept, the pose is re-solved on the perturbed anchors, and the sensor records become the exact ranges;
- there is no stretch and no noise;
- the refit is the default two-stage point fit.

Results for seeds 0–9 ([`geom_synthetic_hangprinter_3.json`](evidence/results/geom_synthetic_hangprinter_3.json), [log](evidence/results/geom_synthetic_hangprinter_3.log)):

| Case | Raw max error | Error vs truth rotated to the fit's gauge | rms |
| --- | --- | --- | --- |
| low dz only / all | 7.3–27.9 mm | 0.000 mm (seed 8: 0.03 mm) | 0.000 mm (seed 8: 0.009 mm) |
| top offset only | 0.000 mm | 0.000 mm | 0.000 mm |

- Pairwise anchor distances and home distances match truth to ≤0.02 mm. The fit is a rigid rotation of truth about home.
- The tilt is that of the true low-anchor plane, 0.2–0.85°. It reaches D.xy as 1900 mm × tilt.
- Three points always define a plane, so on HP3/HP4 this holds exactly for any heights.
- With four low anchors (HP5) the plane cannot absorb arbitrary heights. The synthetic HP5 refit gives tens of mm and an rms 5–80× its baseline.
  - That HP5 synthetic is confounded: it lacks the stretch the HP5 fit corrects for, so its own nominal baseline is 1.8 mm, and one top-offset case fell into a wrong minimum.
  - The live HP5 runs in §6 are the evidence for HP5. The synthetic is not.

## 4. HP3 live, old fit (shared low z)

Full table: [`hp3_live_table.txt`](evidence/results/hp3_live_table.txt). Statistics: [`analysis.txt`](evidence/results/analysis.txt).

Column key:
- **dz:** true low-anchor shifts.
- **top:** |top offset|.
- **k:** true/configured EA.
- **frame:** print-frame max error. The z errors listed after it (A, B, C, then D) and **xy** (max \|xy\| error) are also in the print frame.
- **raw / rigid:** secondary error columns.
- **rms, jack:** the fit's scores.

| Case | dz (mm) | top | k | frame | low z A B C | D.z | xy | raw | rigid | rms | jack | sim s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| nominal | 0 0 0 | 0 | 1.000 | 1.18 | −0.89 −1.18 −0.83 | −0.57 | 0.55 | 0.97 | 0.39 | 0.0218 | 0.70 | 4587 |
| s0 | +8 −14 −28 | 0.8 | 1.165 | 0.71 | −0.58 −0.71 −0.71 | −0.62 | 0.37 | 19.91 | 0.29 | 0.0230 | 0.96 | 4776 |
| s1 | +1 +27 −21 | 47.4 | 0.969 | 1.10 | −0.85 −1.09 −1.01 | −1.10 | 0.56 | 27.78 | 0.37 | 0.0270 | 0.49 | 5038 |
| s2 | −14 −12 +19 | 4.6 | 1.091 | 0.82 | −0.62 −0.62 −0.82 | −0.57 | 0.37 | 22.04 | 0.28 | 0.0245 | 0.55 | 5126 |
| s3 | −25 −16 +18 | 29.1 | 0.973 | 1.10 | −1.10 −0.77 −0.98 | −0.79 | 0.38 | 26.52 | 0.32 | 0.0262 | 0.34 | 5014 |
| s4 | +27 +1 +29 | 4.0 | 0.951 | 1.15 | −1.15 −0.63 −1.15 | −0.89 | 0.60 | 16.96 | 0.44 | 0.0251 | 0.38 | 5089 |
| s5 | +18 +19 +1 | 14.3 | 0.953 | 1.68 | −0.75 −0.80 −0.92 | −1.68 | 0.68 | 10.83 | 0.66 | 0.0284 | 0.73 | 5075 |
| s6 | +2 −9 −8 | 18.7 | 1.053 | 0.93 | −0.76 −0.53 −0.78 | −0.93 | 0.78 | 7.97 | 0.65 | 0.0247 | 0.49 | 5014 |
| s7 | +8 +24 +17 | 11.3 | 1.149 | 1.09 | −0.84 −0.84 −0.72 | −1.09 | 0.24 | 8.67 | 0.21 | 0.0234 | 0.39 | 4664 |
| s8 | −10 +29 −11 | 39.4 | 0.956 | 1.74 | −0.70 −0.84 −0.91 | −1.74 | 0.71 | 27.36 | 0.70 | 0.0258 | 0.71 | 4729 |
| s9 (with fix) | +22 −13 +6 | 38.9 | 1.166 | 0.84 | −0.73 −0.84 −0.79 | −0.67 | 0.09 | 17.78 | 0.10 | 0.0230 | 0.63 | 4731 |
| s9 (before fix) | same | | | **failed at bootstrap** | | | | | | | | 550 |
| k0.8 | 0 | 0 | 0.800 | 1.09 | −1.08 −1.06 −1.09 | −1.01 | 0.33 | 1.08 | 0.33 | 0.0328 | 0.51 | 5429 |
| k0.9 | 0 | 0 | 0.900 | 1.18 | −1.14 −0.90 −0.92 | −1.18 | 0.33 | 1.18 | 0.33 | 0.0293 | 0.70 | 4912 |
| k1.1 | 0 | 0 | 1.100 | 1.05 | −0.60 −0.67 −0.66 | −1.05 | 0.66 | 1.05 | 0.56 | 0.0229 | 0.46 | 5029 |
| k1.2 | 0 | 0 | 1.200 | 0.88 | −0.68 −0.60 −0.70 | −0.88 | 0.22 | 0.88 | 0.17 | 0.0231 | 0.31 | 5145 |
| dz-s0 | as s0 | 0 | 1 | 0.97 | −0.75 −0.97 −0.80 | −0.26 | 0.23 | 20.09 | 0.44 | 0.0265 | 0.67 | 5413 |
| dz-s1 | as s1 | 0 | 1 | 0.91 | −0.88 −0.89 −0.89 | −0.91 | 0.38 | 27.92 | 0.33 | 0.0257 | 0.43 | 5044 |
| dz-s2 | as s2 | 0 | 1 | 1.27 | −0.64 −0.88 −0.90 | −1.27 | 0.65 | 22.15 | 0.45 | 0.0269 | 0.67 | 5372 |
| top-s1 | 0 | 47.4 | 1 | 1.02 | −0.91 −0.61 −0.74 | −1.02 | 0.30 | 1.03 | 0.26 | 0.0276 | 0.40 | 5126 |
| top-s8 | 0 | 39.4 | 1 | 1.04 | −0.65 −0.58 −0.84 | −1.04 | 0.61 | 1.05 | 0.50 | 0.0264 | 0.50 | 4725 |

**Per-coordinate spread over the 10 random machines (print frame, mm, mean ± sd):**

| A.y | A.z | B.x | B.y | B.z | C.x | C.y | C.z | D.z |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| +0.18 ± 0.43 | −0.81 ± 0.19 | −0.27 ± 0.32 | −0.16 ± 0.20 | −0.77 ± 0.16 | +0.04 ± 0.31 | 0.00 ± 0.16 | −0.88 ± 0.14 | −1.01 ± 0.41 |

A.x, D.x and D.y are zero by the convention.

**Attribution.** A linear fit of each error on (k−1), |top offset| and the low-dz range, over all 19 completed HP3 cases:
- **Mean low z:** +1.15 mm per unit (k−1), t = 5.6, residual sd 0.08 mm. Over the 10 random seeds alone: +0.76, t = 2.1.
- **D.z:** +0.88 / +2.5 per unit (k−1), t = 1.0 / 2.0. Not resolved against its 0.34 mm residual sd.
- **Top offset and dz range:** |t| ≤ 1.7 everywhere.
- **The single-factor runs agree:**
  - stiffness 0.8–1.2 alone gives 0.88–1.18 mm;
  - top offsets of 39–47 mm alone give 1.02–1.04 mm;
  - low dz alone gives 0.91–1.27 mm;
  - all within the ~0.3 mm run-to-run sd of the nominal-like cases.

**Do the scores flag bad cases?**
- Over the 10 random seeds, rms and chi2 rank-correlate with the print-frame error (ρ = 0.81) and the rigid error (ρ = 0.70). The jackknife (ρ = −0.13 / +0.13) and the split-fit difference (ρ = 0.04 / −0.32) do not.
- The rms range is narrow, 0.023–0.028 mm, and the nominal machine sits at 0.022 mm, so there is no usable threshold.
- Over all 19 cases, ρ(rms, frame) falls to 0.42, because k0.8 has the highest rms (0.033) at an ordinary error.
- No HP3 case is bad in the print frame (max 1.74 mm), and the 8–28 mm raw errors are invisible to every score, as §3 predicts.

## 5. Robustness: force auto-tune knife-edge (fixed in `1ec95e5e`)

`findEdgeForce` (`autocal/control/behaviors/force_tuning.mjs`) ramps the force by ×1.4 per step. It stopped with "travel stopped increasing" whenever two consecutive moved samples gained ≤ `minUsefulTravelDeg` (0.5°).

The first two ramp samples are still in stiction, at about 1.1° and 1.6°:

| Run | Gain over the first two ramp samples | Outcome |
| --- | --- | --- |
| seed 9 | 0.50° | edge = start force 0.405 N instead of 2.18 N; collection failed |
| passing runs with moved samples | 0.56–0.69° | normal |
| s4, top-s1 | first sample unmoved | rule never armed |

Per-run first two ramp samples are in the run logs (`edge-ramp 1/120`, `2/120`). In seed 9, the edge then collapsed to the start force, and the first sweep failed "Measured sweep is too short to wind the sensor in half a spindle turn" after 550 s.
- The margin is 0.06–0.19°, set by stiction rather than by the perturbations. Nominal machines can hit it.
- The fix: the plateau rule now requires the previous sample to have moved ≥5° (10 × `minUsefulTravelDeg`). Every ramp that did not hit this branch is unchanged.
- Tests: two new unit tests, including the seed 9 curve; `jest autocal/control` passes 182/182.
- Rerun with the fix, seed 9 completes at 0.84 mm.

**Failure rate on imperfect HP3:** 1/10 random machines before the fix, 0/10 after. Ten seeds cannot bound the residual rate below about 25% (95%).

## 6. HP5 live, old fit

[`hp5_live_table.txt`](evidence/results/hp5_live_table.txt):

| Case | dz A B C D (mm) | top | k | frame | rigid | rms | jack | stop | sim s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| nominal | 0 | 0 | 1.000 | 1.00 | 0.65 | 0.0303 | 1.62 | good-fit | 3778 |
| k0.8 | 0 | 0 | 0.800 | 1.09 | 0.72 | 0.0363 | 1.34 | good-fit | 3657 |
| k1.2 | 0 | 0 | 1.200 | 0.58 | 0.54 | 0.0264 | 1.36 | good-fit | 3897 |
| top-s0 | 0 | 40.7 | 1 | 0.25 | 0.21 | 0.0305 | 1.56 | good-fit | 3653 |
| s3 | −25 −16 +18 +5 | 4.7 | 0.992 | 4.30 | 3.78 | 0.0973 | 2.41 | max-steps | 8635 |
| s0 | +8 −14 −28 −29 | 40.7 | 1.043 | **19.26** | 19.08 | 0.4878 | 26.68 | max-steps | 8580 |

- On HP5, uneven low heights are a genuine misfit of the shared-z model. It is not removable by any rotation: the rigid error ≈ the frame error.
- Autocal notices it (it never reaches its stop criterion, rms rises 3–16×, the jackknife rises 1.5–16×), but it still applies the fit after 20 iterations at 2.3× the time.
- Stiffness ±20% and a 41 mm top offset are harmless on HP5 in these single runs.

HP5 with the new fit: see the end of §7.

## 7. New fit (free low z, `a7a8d179`), paired

The cases are re-run on `t3/imperfect-machine-newfit` (merge commit `d35ac09c`), with the same imperfections and the same harness. Files: [`hp3_newfit_table.txt`](evidence/results/hp3_newfit_table.txt), [`paired_old_vs_newfit.txt`](evidence/results/paired_old_vs_newfit.txt), [`paired.py`](evidence/scripts/paired.py).

| Case | frame old → new | rigid old → new | sim s old → new | jack old/new | sweeps old/new |
| --- | --- | --- | --- | --- | --- |
| nominal | 1.18 → 1.19 | 0.39 → 0.40 | 4587 → 4585 | 0.70/0.68 | 7/7 |
| s0 | 0.71 → 0.72 | 0.29 → 0.24 | 4776 → 4794 | 0.96/0.84 | 7/7 |
| s1 | 1.10 → 1.12 | 0.37 → 0.40 | 5038 → 5055 | 0.49/0.42 | 8/8 |
| s2 | 0.82 → 0.74 | 0.28 → 0.28 | 5126 → 5434 | 0.55/0.40 | 8/9 |
| s3 | 1.10 → 0.93 | 0.32 → 0.51 | 5014 → 5045 | 0.34/0.46 | 8/8 |
| s4 | 1.15 → 1.20 | 0.44 → 0.24 | 5089 → 5021 | 0.38/0.86 | 8/8 |
| s5 | 1.68 → 1.62 | 0.66 → 0.60 | 5075 → 5074 | 0.73/0.72 | 8/8 |
| s6 | 0.93 → 0.94 | 0.65 → 0.28 | 5014 → 5060 | 0.49/0.44 | 8/8 |
| s7 | 1.09 → 0.92 | 0.21 → 0.22 | 4664 → 4807 | 0.39/0.63 | 7/7 |
| s8 | 1.74 → 1.54 | 0.70 → 0.59 | 4729 → 4721 | 0.71/0.72 | 7/7 |
| s9 | 0.84 → 0.82 | 0.10 → 0.08 | 4731 → 4730 | 0.63/0.63 | 7/7 |
| k0.8 | 1.09 → 1.09 | 0.33 → 0.33 | 5429 → 5431 | 0.51/0.49 | 10/10 |
| k1.2 | 0.88 → 0.84 | 0.17 → 0.35 | 5145 → 5160 | 0.31/0.40 | 8/8 |
| dz-s0 | 0.97 → 0.79 | 0.44 → 0.34 | 5413 → 5481 | 0.67/0.53 | 9/9 |
| dz-s1 | 0.91 → 1.06 | 0.33 → 0.44 | 5044 → 4755 | 0.43/0.78 | 8/7 |

Paired new − old (frame / rigid / simulated s):

| Subset | Frame | Rigid | Simulated time | New better |
| --- | --- | --- | --- | --- |
| all 15 pairs | −0.043 ± 0.026 mm (se) | −0.026 ± 0.036 mm | +18 ± 31 s | 8/15 |
| 10 random seeds | −0.058 ± 0.029 mm | −0.059 ± 0.047 mm | +48 ± 33 s | 6/10 |

- **Verdict on HP3: the two fits are equivalent in the print frame.** There is at most a 0.06 mm improvement. This is the expected outcome given §3: on three low anchors, freeing low z only changes the gauge.
- Every stop is `stable-estimate`. The new fit changes sweep counts in 3/15 pairs (±1 sweep) and leaves jackknife levels in the same range.
- The "base regression 1.17 → 1.35 mm" from the uneven-heights researcher does not appear here: nominal is 1.18 vs 1.19 mm in the print frame, and 0.97 vs 1.19 mm raw. The raw increase is the frame change (D.xy forced to 0, the old fit had D at (−0.20, −0.08)), not a loss of accuracy. Their 1.35 mm may come from a different scorer or a different run; I did not reproduce it.

**HP5 with the new fit, same two random machines** ([`hp5_newfit_table.txt`](evidence/results/hp5_newfit_table.txt)):

| Case | frame old → new | rigid old → new | rms old → new | jack old → new | stop old → new | sim s old → new |
| --- | --- | --- | --- | --- | --- | --- |
| s0 | 19.26 → **0.52** | 19.08 → 0.39 | 0.488 → 0.030 | 26.7 → 2.21 | max-steps → good-fit | 8580 → 3778 |
| s3 | 4.30 → **1.00** | 3.78 → 0.72 | 0.097 → 0.030 | 2.41 → 1.89 | max-steps → good-fit | 8635 → 3782 |

- On four low anchors the free-low-z fit removes the failure.
- Both machines now converge like the nominal HP5 (1.00 mm, 3778 s, 5 sweeps), at the nominal rms.
- These are two machines with one run each: strong single-pair evidence of a large effect, not a spread.

## 7b. Baseline on every machine with the free-low-z fit (follow-up, 2026-11-21)

**Harness extension (`fc1c0cc4`).** An anchor now moves with every frame-side prim, i.e. every top-level `Spool/Anchor/Wheel/Pinhole/Corner/Attach<X>*` prim that is not a member of the effector rigid body.
- **HP4:** the low anchors move their spool and `Anchor*_top/_bottom`. D moves `SpoolD`, the six top wheels and `AttachD0–2`. The U=2 bottom wheels stay on the effector.
- **HP5r:** the low anchors move their spool, `Anchor*` prims and the pulley-reduction wheels and pinholes at the anchor. I moves `SpoolI`, the top and anchor-side bottom wheels and `AttachI0–3`.
- **Check:** every machine's effector members stay put. A test covers both reeved scenes.

**Validation, unperturbed, old fit as in CI** ([table](evidence/results/hp4_hp5r_validation_table.txt)):

| Machine | Raw max error | Simulated time | Stop | CI run 44794416329 |
| --- | --- | --- | --- | --- |
| HP4 | 0.73 mm | 4923.9 s | stable-estimate | 0.74 mm / 4922 s |
| HP5r | 0.93 mm | 4685.5 s | good-fit | 0.93 mm / 4685 s |

- Both match the local final runs (`fixsweeps-*`) to the step.
- HP4 reads 0.73 here against CI's 0.74 because CI scores the M669 rounded to 0.01 mm.

**Random machines on `t3/imperfect-machine-newfit` (`6580dd6e`, free low z)** ([table](evidence/results/allmachines_newfit_table.txt)):
- The imperfections are the same distributions as §2. A seed draws the same low dz values on every machine, plus D's dz on the five-anchor machines.
- HP3 is the 10 paired runs of §7. HP5 seeds 0 and 3 are from §7; seeds 1 and 2 are new.

| Machine | n | Print-frame max error | Range | Rigid | Common z | Simulated s | Stops |
| --- | --- | --- | --- | --- | --- | --- | --- |
| HP3 | 10 | 1.06 ± 0.31 mm | 0.72–1.62 | 0.34 ± 0.17 | −0.83 | 4721–5434 | 10 × stable-estimate |
| HP4 | 3 | 0.53 ± 0.04 mm | 0.49–0.57 | 0.40 ± 0.10 | +0.17 | 4537–5002 | 3 × stable-estimate |
| HP5 | 4 | 0.95 ± 0.34 mm | 0.52–1.36 | 0.69 ± 0.23 | +0.07 | 3592–3782 | 4 × good-fit |
| HP5r | 3 | 1.23 ± 0.04 mm | 1.18–1.26 | 0.89 ± 0.10 | +0.09 | 3833–4966 | 3 × good-fit |

Per-seed values (print frame, mm):

| Seed | HP4 frame (worst) | HP5r frame (worst) | HP5 frame (worst) |
| --- | --- | --- | --- |
| 0 | 0.57 (D.z +0.57) | 1.18 (I.z +1.18) | 0.52 (I.z −0.52) |
| 1 | 0.51 (C.x) | 1.24 (I.z +1.24) | 1.36 (C.y) |
| 2 | 0.49 (C.x) | 1.26 (B.x) | 0.91 (A.y) |
| 3 | — | — | 1.00 (I.z +1.00) |

- **No failures:** 20/20 runs completed and applied a fit on the new fit. There were no max-steps stops: HP3/HP4 stop at `stable-estimate` and HP5/HP5r at `good-fit`.
- **HP4** random machines (0.49–0.57 mm) are at or below the unperturbed CI value (0.73 mm). Unlike HP3, HP4 shows no common downward z shift.
- **HP5r** is the least accurate machine: 1.18–1.26 mm, against 0.92 mm unperturbed. Its top-anchor z is +1.2 mm in 2 of 3 seeds, which is the held/top-anchor z direction that the stretch and home-reference analyses tie to compliance.
- **Unperturbed HP4/HP5r on the new fit** (`6580dd6e`, [table](evidence/results/hp4_hp5r_nominal_newfit_table.txt)):

  | Machine | Old fit (CI) | New fit | Simulated s (new) | Stop (new) |
  | --- | --- | --- | --- | --- |
  | HP4 | 0.72 mm | **0.72 mm** (worst C.x) | 4926 | stable-estimate |
  | HP5r | 0.92 mm | **1.10 mm** (worst C.y; I.z +0.53) | 4684 | good-fit |

  - On HP4 the free-low-z fit is neutral when unperturbed.
  - On HP5r it costs 0.18 mm. So the HP5r random machines' 1.23 ± 0.04 mm is mostly that fit-change cost; the imperfections add only about 0.13 mm over the new nominal.
  - The HP5r top-anchor z error grows from +0.53 mm (nominal) to +1.18/+1.24 mm on two random seeds, which is still the coordinate to watch with the home-strain term.
- **Comparison caveat:** n = 3–4 per machine, one deterministic run each. The ±0.04 mm sd on HP4/HP5r is three samples, not a bound.

This is the per-machine baseline to compare the home-strain term against: same seeds, same commands, `--rescore` for scoring.

## 8. Strongest alternative explanations and limitations

- **One deterministic run per machine.** The ~0.3 mm run-to-run sd seen across near-nominal cases is the noise floor. Effects below it are inferred only from the regression over many machines (the stiffness coefficient), not from single pairs.
- **The common −0.87 mm z shift dominates the frame error.** It is the known home-reference bias (`research/topics/autocal-stretch.md`), and nothing here changes it. A home reference would cut HP3 frame errors to about the rigid column (0.40 ± 0.21 mm).
- **Stiffness is perturbed in the scene only.** The firmware's flex compensation and the collector both use the nominal `M666 S`. That is a realistic wrong-config.g case, but stiffness is uniform over all lines: there are no per-line differences.
- **Coverage.** HP5 rests on two random seeds with a single run each. HP4 and HP5r have three random seeds each, on the new fit only (§7b).
- **The print convention hides a true tilt by design.** A real bed levelled to gravity would see the 0.2–1.4° tilts in `truth_frame_tilt_deg` as first-layer error (about 3–12 mm over 0.5 m). The user decided that this is not calibration error.
- **The kinematic synthetic ignores forces.** It shows the gauge for length data. In the live sim, gravity breaks the symmetry physically, but the HP3 fit uses no forces, so the gauge carries over. The live raw-vs-frame columns confirm this.

**What would change the conclusions:**
- a fit that uses tensions or gravity (stretch correction on HP3), which could make low-anchor tilt observable;
- more seeds showing a stiffness effect on D.z;
- a residual auto-tune failure mode at other stiction margins.

## 9. Reproduction

```bash
# one machine (about 6 min wall on HP3, 1 simulation)
.venv/bin/python -m autocal.tools.imperfect_machine output/imperfect/hp3-s3 --machine hangprinter_3 --seed 3
# batch, two at a time, skipping finished names
research/.../evidence/scripts/queue.sh output/imperfect < research/.../evidence/scripts/cases.txt
# tables and statistics
.venv/bin/python research/.../evidence/scripts/summarize.py output/imperfect/hp3-*
.venv/bin/python research/.../evidence/scripts/analyze.py output/imperfect/hp3-s[0-9]
.venv/bin/python research/.../evidence/scripts/paired.py output/imperfect output/wt-newfit/output/imperfect
# kinematic synthetic (no simulator)
PYTHONPATH=$PWD .venv/bin/python research/.../evidence/scripts/geom_synthetic.py \
    autocal/data/references/benchmark_923c2baa_hangprinter_3.json hangprinter_3 --seeds 0-9
```

- **Committed evidence:** every run's `result.json` and `imperfection.json`, under [`evidence/runs/`](evidence/runs/).
- **Raw run directories** (datasets, logs, about 180 MB each) stay uncommitted in `output/imperfect/` and `output/wt-newfit/output/imperfect/` of this worktree. They are regenerable, because collection is deterministic given the commit and the seed.
- **A fresh worktree needs:** `.venv` and `node_modules` symlinked from the main checkout, and `RRF/build/rrf_simulator` and `src/wasm/cable_solver/target/native/physics.node*` copied from it.
