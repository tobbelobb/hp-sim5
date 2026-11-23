# Review: free every low-anchor z, gauge at the top anchor (candidate a7a8d179)

**Verdict: accept with caveats.**

The scientific claim holds. Uneven low-anchor heights no longer produce a gauge error, and the six gauge constraints are right on every machine. The code is clean, and the equal-z removal is complete. Two points need to be stated correctly in the report before merge:

- **The HP3 equal-height regression is real for that dataset; it is not collection noise.** It comes from the fit, and I reproduced it offline.
- **The HP5 jackknife rise depends on the collection.** It already shows up to the user as "Found parameters of poor quality" on a 0.69 mm calibration.

Neither caveat blocks the change. No physics code changed, so the physics checks do not apply.

## Integrity

- I ran `./hp-sim5-research-program --resume … --verify review-0e816f6d262c` from the main checkout. Result: `verified: true`, candidate a7a8d179d38c…, tree b72a5c4b….
- I checked out the candidate detached in my own worktree, with .venv, node_modules, RRF/build and cable_solver/target symlinked from the main checkout.

## Checks performed

| Check | Kind | Result |
|---|---|---|
| `.venv/bin/python -m pytest autocal/tests -q` | reproduction | **527 passed**, 34 deselected, 195 s |
| `npx jest autocal/control` | reproduction | **180 passed** (25 suites) |
| Diff review of `theoretical_ellipse.py`, the JAX objective, pose_fit, pose_planning, rough_bootstrap, anchor_seed, ellipsoid_autocal, anchor_uncertainty | inspection | Consistent layout `[A.y, A.z, B.xyz, C.xyz, (D.xyz), top.z]`, 3n−3 parameters, everywhere |
| Leftover-reference grep (`low_anchor_z`, `--low-anchor-z`, `_low_anchor_z`, `lowAnchorZ`, `*_LOW_Z`) over .py/.js/.mjs/.ts/.rs/.sh/.json | inspection | Two hits only: the changelog in `autocal-flags.md`, and `"low_anchor_z": null` inside the recorded settings of `autocal/data/references/hp4_no_buildup_Oct_16_2026_4x_substepping.collection.jsonl`. That file is a historical log and nothing reads that key. No JS/browser references. |
| Cross-refit: old code (b249a1a5) and new code, each on the old-collected and new-collected live base datasets | **reproduction (new)** | See Q2 and Q3 |
| E6 with 6 range-noise seeds (0.02 mm), h = 0, old and new code, HP3 and HP4 | **reproduction (new; the report says this was not done)** | See Q2 |

Scripts: `/tmp/claude-1000/-home-torbjorn--t3-worktrees-hp-sim5-t3-b0c8bd3f-05e564e02c197816/ccaf19de-aff5-451a-a820-40373c1a4c51/scratchpad/cross.py`. It reuses the session's `lowz_common.fit_default`, and the old code was extracted with `git archive b249a1a5 autocal`. The scratch outputs are not durable. The numbers are copied below.

## Q1. Are the six gauge constraints right and complete?

**Yes, on HP3, HP4, HP5 and HP5r.** I checked this by inspection. The submission's Jacobian check (E7) supports it, and I did not rerun that check.

- **Rotation (3): A.x, top.x, top.y.** Every range is |a−p|−|a| (`fit_point_positions`: absolute = |a| + offset + length). So rotations about home are the exact symmetry, and fixing a direction (2) plus a turn about it (1) removes them. `_rotate_top_vertical` is a correct Rodrigues rotation taking top to +z. It handles the antiparallel case, and it is the identity when top is already vertical. The xy step afterwards rotates about z and mirrors x, so top x = y = 0 survives. The mirror rule (B.x > 0) plus the per-machine sign bounds handle reflection.
- **Translation (3).** Free home offsets exist only for lines `n−d … n−1`: line D on HP3/HP4, and lines D and top on HP5/HP5r. The first three lines' reference lengths pin translation.
  - **HP4 U=2:** `_hp4_home_strain` sets offsets[:3] = strain·(|a| + Y/U). That is a deterministic function of the anchors and adds no free parameter, so translation stays pinned. The U=2 reeving enters only through that term and through stretch. The point model has a single effector attachment, so there are no per-line effector offsets that would break the rotation symmetry differently.
  - **HP5/HP5r:** 12 anchor parameters plus 2 offsets. `extra_sensor_ranges` are per-anchor line ranges, so they carry no extra frame dependence.
- **Subtlety, not a defect.** The stretch correction uses gravity along frame −z. On a real machine whose top line is not plumb, the stretch model is therefore evaluated about 3.5 mrad per 10 mm off. That is a μm-level effect, and under the user's convention it is by design. Benchmark truth always has a plumb top, so this is untested.
- **Pose planning.** `point_informations` builds its selection mapping in the canonical frame and applies it to directions in the input frame. The mapping is a constant selection matrix, so the result is a valid gauge fix in any frame. This is harmless, and it predates this change.

## Q2. Is the HP3 equal-height regression real or noise?

**It is real for that collection, and it comes from the fit, not from the collection diverging.** Offline it is not systematic. The report's explanation ("collections diverge… within the ±0.5 mm single-run knife-edge") is wrong in mechanism and should be corrected.

Cross-refit of the live HP3 base datasets (max per-anchor error, mm):

| Dataset | Old code | New code |
|---|---|---|
| `runs/base_0` (collected by old code) | 1.170 (low z −0.97 ×3) | **1.338** (low z −0.89 / −1.18 / −0.83) |
| `runs/new_base_0` (collected by new code) | 1.176 | 1.343 |

The two collections are interchangeable: each code gives the same answer on either dataset. The whole 0.17 mm difference is the fit. The ≈ −0.9 mm stretch/D.z bias that the old fit spread evenly over a shared z now lands unevenly: B.z goes to −1.18 mm.

Noise seeds on the benchmark dataset (E6, h = 0, 0.02 mm noise, 6 seeds) show the opposite sign:

| Machine | Old code, mean (range) | New code, mean (range) |
|---|---|---|
| HP3 | 1.01 (0.94–1.13) | **0.96** (0.90–1.02) |
| HP4 | 0.67 (0.48–1.03) | 0.67 (0.51–1.04) |

**Reading.** The equal-height cost on HP3/HP4 is ±0.2 mm and dataset-dependent: a variance/bias redistribution, not a mean shift. That is consistent with the user's decision. The report should say "reproduced by refit; the free low z distribute the shared stretch bias unevenly", not "collection divergence".

## Q3. Does the HP5 jackknife rise threaten stopping or fit selection?

**It threatens neither in the evidence. It does change a user-facing message.**

- **Stopping.** HP5 stops on `good-fit`. That test uses the covariance `max_std_mm` ≤ 1.0, rms, agreement and coverage. It does not use the jackknife (`stopping.py:122-130`). Patience uses the jackknife only through its *rank* among fits, and the jackknife fell monotonically in both runs: 16.07 → 3.28 and 29.97 → 2.37.
- **Fit selection.** `newest_trusted_index` rejects a fit only if its jackknife exceeds max(3 mm, 10 × the minimum so far). With histories of [16.07, 3.28] and [29.97, 2.37] nothing is rejected. A rejection would need an earlier jackknife ≤ 0.33 mm, which HP5 has not shown.
- **User-facing label.** `uncertainty_label(3.277)` = "poor". So the new HP5 base run printed **"Found parameters of poor quality"** for a 0.69 mm calibration. The old run printed "usable" for 1.04 mm. The submission does not mention this.
- **Collection dependence (new evidence).** The new code on the *old-collected* HP5 base data gives jackknife 0.98 mm (error 1.29 mm). On the new-collected data it gives 3.28 mm (error 0.68 mm). So the inflation is not intrinsic to the free warp: it appears with one particular 5-sweep collection that the new planner chose. With two runs there is no telling how common it is.
- **Not completed.** The old-code refit of the two HP5 datasets, including its jackknife, hit my 30-minute timeout and produced no output. The old-code HP5 numbers here are the submission's live values, not my reproduction.

**Consequence.** No accuracy or stopping risk is shown. However, the jackknife's documented calibration on HP5 (ratio 0.9–1.2 to true error, per `anchor_uncertainty.py`) no longer holds under this gauge. In the one inflated run it over-reports about 5×. The docstring's corpus statistics predate this gauge and should be marked as such or re-measured.

## Q4. Is the deletion of equal-z / fixed-z / `--low-anchor-z` complete and safe?

**Yes.**

- There are no leftover Python callers, and the tests pass with the old signatures removed.
- There are no JS/browser references, and jest passes.
- The only stored artefact with the key is a historical HP4 collection log (`"low_anchor_z": null`). Nothing parses it: the regression tests pass, and grep finds no reader.
- `_apply_low_anchor_z_constraint` and the `_low_anchor_z` dataset key are deleted with no readers left.
- `autocal-flags.md` records the removal. That is appropriate, not stale.
- Anyone who passes `--low-anchor-z` now gets an argparse error, which is the intended behaviour.

## Limitations of this review

- I ran no new live simulations, because the machine was shared. The live results are inspected, not reproduced; the cross-refits reproduce their fit stage.
- HP5r was checked by inspection only, and HP4/HP5r live with uneven heights remain untested, as the submission says.
- I did not rerun E7 (Jacobian singular values) or the uneven-h E6 grid. I relied on the submitted JSON, which is internally consistent with the code.

## Decisive missing test

Run **3–5 live HP5 and HP5r base runs (different seeds or start poses) under the new code**, and record the final jackknife, the quality label and the stop reason. This would show whether the "poor"-labelled, 5×-over-reporting jackknife is typical or a one-off. If it is typical, the fix is small: recalibrate `uncertainty_label` for HP5, or exclude the warp direction from the jackknife's max. If it is rare, a note suffices.

A secondary test: one live HP4 run with a 10 mm low-anchor offset, to confirm the HP4 equal-strain home model under uneven heights.

## Requested report edits (non-blocking)

1. In §B3, change the HP3 base explanation to "reproduced by refit of either collection (old 1.17 / new 1.34); the free low z distribute the shared stretch bias unevenly". Cite the noise-seed result (old 1.01 vs new 0.96 mean) as evidence that it is not systematic.
2. In §B3 and Limitations, say that the HP5 jackknife inflation now changes the console label to "poor" on a 0.69 mm fit, and that it depends on the collection (0.98 mm on the old collection with the new code).
