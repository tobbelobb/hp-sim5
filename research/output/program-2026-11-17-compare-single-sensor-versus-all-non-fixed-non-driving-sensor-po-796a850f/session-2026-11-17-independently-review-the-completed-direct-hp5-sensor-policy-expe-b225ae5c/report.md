# Independent review: HP5 direct single-sensor vs both-low-axes-as-sensors

**Status:** review complete; simulation only; no production changes, no full runs. **Date:** 2026-11-17.
**Reviewed evidence (read only):** `/home/torbjorn/repos/hp-sim5/research/output/program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f/session-2026-11-17-hp5-direct-compare-the-current-single-active-sensor-plus-complia-6af2e045/` (`report.md` sha256 `f114055c…7c31e1`, `results.json` sha256 `94d8062d…a0f7`).
**Scope:** audit units, force calibration, information statistics and the offset-free synthetic fit; replay one decisive short matched native-headless sweep at 0.05 N vs 0.30 N.

## Verdict

| Claim | Verdict |
|---|---|
| At 0.05 N the redundant C axis intermittently pays out ~1.2–1.3 mm of extra line; the state persists after a 6 s hold | **Accepted, reproduced exactly** |
| 0.30 N removes the length slack in this scene and is mechanically feasible | **Accepted, reproduced** |
| The cause is the torque-mode floor (cogging + Coulomb + stiction), not gravity or settling | **Accepted as the best explanation**; the quantitative "floor" wording needs correction |
| Force conversion is 0.06 N m/N and delivered force is cogging-biased by ~0.05–0.16 N | **Accepted in magnitude**; per-pose variation is larger than the report says |
| The extra range is already sampled and discarded, so it costs no acquisition time | **Accepted** (collector code) |
| Adding C gives an 11.5× smaller anchor σ-trace / 130× variance and ~40% lower synthetic RMS | **Arithmetically correct, but the interpretation needs correction**; the gain is pose-set dependent and not shown on the tested sweep |
| "The median direction improves only ~1.4×" | **Rejected**: 1.4× is the strongest (best-determined) mode; the median mode improves 6.2× in σ |
| A 0.05 mm constant C bias erases the synthetic gain, so the value is conditional on bias control | **Rejected for a production offset fit**: per-line home offsets absorb the constant bias and the gain returns. The real risk is pose-dependent slack |
| The experiment demonstrates usable extra-range information | **Not established**: on the 14 poses actually swept, the paired synthetic fit shows no gain |

The reliability result is solid and reproducible; the information result is a separate broad-pose synthetic analysis that this experiment does not test, and the bias conclusion is an artifact of the offset-free fit.

## 1. Matched native-headless replay

The published `evidence/sweep_experiment.py` cannot run: line 113 returns the undefined `home_geom` (`NameError`), and its mtime (12:19) is later than the runs (12:15), so the archive does not contain the exact run script. A one-line fix (`return s, None`) in `sweep_review.py` restores the run. The replay is **bit-for-bit identical** to the source runs.

| run | direction | 0.05 N slack poses | mean / max error (mm) | after 6 s (mm) | min segment force (N) | length jitter (mm) |
|---|---|---|---:|---:|---:|---:|
| `repro_down_0.05` | down | 1/7 | +0.161 / **+1.183** | +1.188 | 0.000 | 0.0113 |
| `repro_down_0.3` | down | 0/7 | +0.020 / +0.065 | +0.068 | 0.071 | 0.0019 |
| `repro_up_0.05` | up | 1/7 | +0.158 / **+1.296** | +1.293 | 0.000 | 0.0112 |
| `repro_up_0.3` | up | 0/7 | +0.024 / +0.075 | +0.071 | 0.067 | 0.0019 |

Replay vs source: max |Δ| of the per-pose C error = **0.000000 mm** in all four runs. The slack is a deterministic property of this world, not sampling noise.

Per-pose 0.05 N errors are `[+0.064, +0.451, −0.056, −0.192, −0.194, +1.183, −0.129]` (down) and `[+0.064, −0.010, +0.239, −0.180, −0.190, +1.296, −0.111]` (up). Only one mid-sweep pose crosses 0.5 mm, but the distribution is heavy-tailed and two more poses exceed 0.2 mm. "1/7 slack poses" is a threshold statement, not a description of how often the axis is untrustworthy.

**Tension is a stronger indicator than length error.** At 0.05 N, both of C's two lines are unloaded in 3/7 (down) and 4/7 (up) poses; at least one line is unloaded in 5/7 and 6/7; both lines are taut in only 2/7 and 1/7. At 0.20 N one line is unloaded in 2/7 poses; at 0.30 N all 7/7 poses are fully taut. The report's reliability conclusion is therefore stronger than its table suggests: the 0.05 N axis is mostly unloaded, not merely occasionally over-paid.

## 2. Units and force calibration

- **Geometry/units are correct.** HP5 low axes have 2 lines on a 30 mm spool, so `torque_per_newton = 2 × 0.03 = 0.06 N m/N`; 0.05 N = 0.003 N m. Scene: `stepper:holdingTorque = 1`, `radius = 0.03`, `numPolePairs = 50` (`public/usd_scenes/hp5.usda`). Engine defaults: Coulomb `0.002·holding`, stiction `0.003·holding`, cogging amplitude `0.01·holding` (`src/wasm/cable_solver/src/engine.rs:1606–1625`). The report's 0.012–0.015 N m worst-case sum and 0.2–0.25 N per line are arithmetic-correct.
- **"Torque floor" wording.** Cogging is position-dependent and can assist take-up; the true static worst case is Coulomb + stiction + cogging at an opposing phase (0.015 N m), while the mean friction floor is 0.005 N m. The 0.05 N command is below both, so the conclusion holds, but "the floor is 0.012–0.015 N m" should be "below the worst-case phase sum".
- **Delivered force.** Per-pose C axis means (taut, down): 1.0 N → 0.833–1.140 (mean 0.941); 0.3 N → 0.140–0.464 (mean 0.241); 0.2 N → 0.037–0.366; 0.05 N → 0.000–0.217. The ~±0.16 N swing at 0.3–1.0 N (and a 0–0.37 N range at 0.05–0.2 N) supports a force bias of the order of the 0.167 N cogging amplitude, not a length-scale error. The causal attribution to cogging is consistent with prior corpus evidence (the known 50-pole sinusoid explains 98–99% of HP3/HP4 sensor-force-error variance), but this experiment does not separate cogging from Coulomb/stiction.
- **The report's "0.04–0.12 N" at 0.05 N mixes scenes** (global means are 0.046/0.039 on the taut scene and 0.116/0.117 on the catalog scene) and does not cover the per-pose 0.217 N. Use per-scene, per-pose ranges.
- **"encoder std" is a misnomer.** It is the standard deviation of the physical cable-joint rest length across six samples, not encoder noise or quantization. At 0.3 N it is 0.0019 mm, far below any real encoder. The measured length is nevertheless a good encoder-equivalent proxy: it agrees with 30 mm/rad × spool angle (7.5 mm/rad for I) to ≤0.01 mm for A–D, including at 0.05 N where the line is unloaded. So the slack is visible in the encoder angle too; the experiment simply does not exercise the production encoder/quantization path.

## 3. Information statistics

The Fisher code, numbers and noise scaling reproduce. The interpretation does not.

| quantity | current (A,I fixed; B,D measured) | both sensors (+C) | ratio |
|---|---:|---:|---:|
| σ-trace (mm) | 8.150 | 0.709 | 11.50× smaller σ |
| squared σ-trace norm | — | — | 132.3× ("130× variance") |
| weakest information eigenvalue | 0.0178 | 3.204 | 180× info, **13.4× σ** |
| median information eigenvalue | 11.66 | 444.2 | 38.1× info, **6.17× σ** |
| strongest information eigenvalue | 6575.8 | 12887.5 | 1.96× info, **1.40× σ** |

Per-mode σ ratios, sorted: `1.40, 2.25, 2.88, 6.17, 6.37, 6.37, 9.55, 9.69, 13.42` (6/9 modes improve >5×, 8/9 >2×). The report's "median direction improves only ≈1.4×" is the **strongest** mode, not the median. The gain is concentrated in the weak modes, as the report says, but the typical mode improves ~6× in σ, not 1.4×.

The σ-trace ratio is scale-invariant (11.44–11.52 for noise 0.005–0.1 mm), so the 0.01 mm choice does not drive the ratio. It is a ratio of covariance norms, not a per-direction variance, and it diverges when any mode is unobservable. On the 14 poses actually swept it is meaningless: the current weakest information eigenvalue is `5.4e-15`, the σ-trace is `1.6e7 mm`, and the "ratio" is 9368×.

## 4. Synthetic anchor recovery and the offset-free limitation

The original 6-seed design is reproduced exactly (`evidence/stats_review.json`, `original_design`). Paired differences (both policies use the same poses, noise and initial guess within a seed):

| points | current (mm) | both sensors (mm) | paired difference | t (5 df) |
|---|---:|---:|---:|---:|
| 10 | 1.366 | 1.167 | −0.198 ± 0.062 | −3.2 |
| 20 | 1.471 | 0.890 | −0.581 ± 0.076 | −7.6 |
| 30 | 1.574 | 0.894 | −0.680 ± 0.087 | −7.9 |
| 20 + 0.05 mm C bias | 1.471 | 1.356 | −0.115 ± 0.123 | −0.9 |

So the gains are statistically real *in the model*: ~14% at 10 points, ~40% at 20, ~43% at 30. The report's "at 10 points the gain is only 1.37 → 1.17" is a significant 14% gain, not a null; the 20-point 40% has a 95% interval of roughly 26–53%. The point estimate is one 6-seed realization.

A matched, nested design (same 30 poses, same noise realization, first 10/20/30 points) gives smaller, noisier gains: 20 points 1.32→0.98 (−0.34 ± 0.17), 30 points 1.35→0.88 (−0.48 ± 0.26). The direction is robust; the magnitude depends on pose-set composition. The nonconvex fit is also sensitive to BLAS thread count: the same 6 seeds gave 20-point current 1.32 (default threads) vs 1.46 (one thread), while the alt stayed ~0.98. That is a ~0.1 mm methodological uncertainty, comparable to the 6-seed standard error. Report the gain as a range (roughly 0.2–0.7 mm, 15–45%), not as "40%".

**The offset-free bias test is the main modeling problem.** My offset-free 0.05 mm C-bias run reproduces the report: the gain collapses to 0.06 mm (matched) / 0.115 mm (original). But production now implements per-line home offsets. With one free constant offset per measured line:

| 20 points | current | both sensors | gain |
|---|---:|---:|---:|
| offset-free, no bias | 1.32 | 0.98 | 0.34 |
| offset-free, 0.05 mm C bias | 1.32 | 1.26 | 0.06 |
| with per-line offsets, no bias | 1.57 | 1.05 | 0.51 |
| with per-line offsets, 0.05 mm C bias | 1.60 | 1.01 | 0.59 |

The offsets cost accuracy for both policies (nuisance parameters), but they absorb the constant C bias and preserve the relative gain. (The offset rows come from runs at default and one-thread BLAS; the absolute values differ by <0.05 mm and the contrast is unchanged.) The constant-bias warning is therefore a property of the offset-free model, not of a production offset fit. The real threat is pose-dependent error: adding **+1.2 mm to one of 20 C poses** gives 1.49 mm (offset-free) / 1.42 mm (offsets), versus 1.01 mm unbiased. One slack pose erases the gain and makes the both-sensors fit worse than current on the offset-free comparison. This is the correct basis for the tautness gate.

Robustness checks:
- Fixed lines given 0.01 mm noise instead of exact: gain 0.53 mm (current 1.46, alt 0.93), so the exact-fixed-line assumption is not driving the gain.
- Initial-guess perturbation 10/50/100 mm: gain −0.45 to −0.68 mm; the near-truth initialization is not driving the result.
- Range noise 0.01 vs 0.05 mm (one thread): gain −0.47 vs −0.33 mm; the relative gain survives 5× noise.

## 5. The information claim is not tested by the sweep

The Monte Carlo uses a broad 3D pose box (x,y ∈ ±0.24 m, z ∈ 0.02–0.55 m). The experiment actually swept B by ~90 mm at one height. Feeding the 14 measured effector poses from `taut_down_0.3` + `taut_up_0.3` into the same paired synthetic fit:

- true spans: down 90.5 mm x, 2.5 mm y, 2.2 mm z; up 91.2 / 2.4 / 2.1 mm.
- synthetic fit: current **1.511 mm**, both sensors **1.537 mm**, paired difference **+0.027 ± 0.141 mm** → no gain.
- Fisher: weakest information `5.4e-15`, σ-trace ratio 9368× (dominated by an unobservable mode).

This does not refute the broad-pose result; it scopes it. The extra range may help a multi-sweep production pose set, but this experiment's own sweep geometry provides no evidence for it. A production-pose-set synthetic or a real 20-point recording at ≥0.3 N is needed before claiming information gain. The report should not present the 40% as a result of the sweep experiment.

## 6. Provenance and reproducibility

- **Broken archived script.** `evidence/sweep_experiment.py` sha256 `8e2b94a1…becfb6` raises `NameError: home_geom`; mtime 12:19 vs runs 12:15. The exact run script is not archived. `sweep_review.py` sha256 `d6b88252…39cacb` differs by one line and replays all four runs exactly.
- **Fast-reversal provenance gap.** `rec_*.json` manifests have `direction: null` and `drive_speed_rad_s: null`; the report's "≈19 rad/s" is not archived. The 7/15 slack / 3.35 mm stress result is directionally consistent but cannot be replayed from the preserved inputs.
- **Role test is not a clean axis test.** `altrole_*` changes fixed B, drive C, sensor A, redundant D. Driving C moves the effector in y (~90 mm), not x; the recorded `x_span_mm = 2.37` is not the sweep span. It demonstrates non-universality across a different sweep, not an axis-specific cause.
- **`results.json` "encoder_std"** is length jitter (see §2), and the `0.8 mm longitudinal model bias` is the five-anchor midpoint reduction, not a measured hardware error. Both are acknowledged in the report's limitations but not in the tables.
- No research runtime was started; all worlds were private temporary directories under the native Rust engine, artifact sha256 `1c137e60…ed418b`; scene `hp5_taut.usda` sha256 `944b66cc…35c9bf`; root and worktree at `c2450975d7a3dcf1e6bd90d60afc1e7c3acc51f6`.

## Conclusions for the program

1. **Keep production defaults unchanged.** The 0.05 N redundant axis is not a trustworthy sensor; the failure is deterministic and reproduced.
2. **If a policy trial is considered, use ≥0.3 N plus a tautness gate.** 0.2 N leaves one line unloaded in 2/7 poses and is not a safe margin. A single 1.2 mm slack pose reverses the synthetic gain.
3. **Do not quote 11.5× σ-trace or 130× variance as accuracy.** Report the paired synthetic gain with its interval and the per-mode spectrum. Correct "median 1.4×" to 6.2× and "weakest singular value 180×" to "180× in information, 13.4× in σ".
4. **Drop or rescope the constant-0.05 mm-bias warning.** Production home offsets absorb a constant per-line bias; the remaining risk is pose-dependent slack and geometric model bias, which the report already identifies.
5. **Do not claim information gain from this experiment.** It is a broad-pose synthetic result. The tested sweep gives no gain; a production-pose-set or real-data fit is required.
6. **Fix the archive.** Repair the `home_geom` line and record the fast-reversal speed/direction before citing the stress result.

## Limitations of this review

- One scene, one replay pair (down/up), 0.05 vs 0.30 N; no hardware, no production collector or fitter.
- Synthetic analyses are geometry-only; the offset, initialization, fixed-noise and slack variants are mine, not the production fitter.
- The nonconvex fit depends on BLAS threading at the ~0.1 mm level; MC magnitudes are ranges, not precision estimates.
- The 14-pose actual-sweep test is a scope check, not a production pose set; the production planner uses multiple sweeps and heights.

## Reproduce

From this `evidence/` directory, with `/home/torbjorn/repos/hp-sim5/.venv/bin/python`:

```bash
# exact replay (one-line fix of the published script's home_geom NameError)
python recompute.py --source <reviewed evidence dir>
python sweep_review.py --scene "$PWD/hp5_taut.usda" --policy 0.05,1.0 --steps 6 \
  --drive-step-rad 0.5 --drive-speed-rad-s 0.8 --settle 800 --samples 6 \
  --recover-steps 3000 --direction down --out "repro_down_0.05.json"
# repeat 0.3,1.0 and --direction up
python stats_review.py        # Fisher spectrum, paired MC, offsets, slack
python sweep_scope.py         # 14 actual sweep poses
```

Artifacts: [`recompute.json`](evidence/recompute.json), [`repro_*.json`](evidence/), [`paired_stats.json`](evidence/paired_stats.json), [`stats_review.json`](evidence/stats_review.json), [`sweep_scope.json`](evidence/sweep_scope.json), [`init_sensitivity.json`](evidence/init_sensitivity.json), [`noise_sensitivity.json`](evidence/noise_sensitivity.json), [`matched30_offset.json`](evidence/matched30_offset.json), [`force_table.json`](evidence/force_table.json), [`sweep_review.py`](evidence/sweep_review.py), [`sweep_scope.py`](evidence/sweep_scope.py).

Source evidence: [`report.md`](/home/torbjorn/repos/hp-sim5/research/output/program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f/session-2026-11-17-hp5-direct-compare-the-current-single-active-sensor-plus-complia-6af2e045/report.md), [`results.json`](/home/torbjorn/repos/hp-sim5/research/output/program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f/session-2026-11-17-hp5-direct-compare-the-current-single-active-sensor-plus-complia-6af2e045/evidence/results.json), and the prior [autocal stretch topic](/home/torbjorn/repos/hp-sim5/research/topics/autocal-stretch.md).
