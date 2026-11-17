# Independent review: reduced HP5 matched short-collection two-sensor evidence

Investigation: `session-2026-11-17-independently-review-reduced-hp5-matched-short-collection-eviden-748ceb19`
Reviewed: `session-2026-11-17-hp5-pulley-reduced-independently-compare-the-current-single-acti-f359cbd6` (revision `c2450975`, read-only).
Evidence type: **native-headless simulation and saved-data replay only**. No new physical run and no hardware evidence. The reviewed report's own simulation is deterministic, so the replays below test the fitter and bookkeeping, not physical variability.

## Bottom line

* The headline matched result is real and reproduces exactly on the saved dataset: with the same four-sweep budget, starts, optimizer and home-offset prior, adding the fifth measured range moves the translation-aligned maximum anchor error from **22.13 mm to 1.85 mm** on the two-sensor recording. The estimate is stable across six starts and three offset gauges; the four-range estimate is not.
* The conclusion "extra measured ranges genuinely help rough calibration" is **accepted for this simulation**, but the size is **about 7–15×**, not a single 9×, and the absolute 1.85 mm is still rough. The headline 1.85 mm rests on the extra range being accurate to well under 0.1 mm; a useful gain survives to about 0.2 mm, and the report's own 0.35–0.86 mm bias would leave only ~1.2–1.5×.
* Two substantive corrections are required. **(a)** Both control arms were fitted with the redundant line assumed at 5 N although it was at `force_low`; the reported arm-A four-range baseline of 3.8 mm is actually **9.87 mm**, and the claimed disagreement with the production fitter disappears. **(b)** The claimed −0.35…−0.86 mm "extra range bias" is the residual against a pose inferred from three ranges, not a bias in the extra range; the extra range is consistent with the nominal anchors to **0.04 mm RMS**. Taken literally, the reported bias would have degraded the fit to 15–18 mm, contradicting the 1.85 mm headline.

## Verdict table

| Claim in the reviewed report | Verdict | Evidence |
|---|---|---|
| Compliant axis frozen over 79–140 mm of inferred motion | **Accepted** | Per sub-sweep measured travel ≤0.001 mm against 78.6–139.7 mm geometric travel; 38.4–46.7 mm extra paid-out line. `summary-checks.json`, `analysis.json` |
| 5 N measurement-only pretension tracks the motion | **Accepted** | Tracking slope 0.995–1.000; measured range 78.6–139.3 mm. (Preload is 2.06 N, not 5 N; reserve moves are at 5 N.) |
| Five-range 1.85 mm vs same-recording four-range 22.1 mm | **Accepted** | Independent replay 22.04 mm (single start) / 22.13 mm (two starts) vs 1.85 mm; CV folds 18.6–52.8 mm vs 2.0–2.9 mm. `review-fits.json`, `cv-replay.json` |
| "Baseline current four-range 3.8 mm" | **Rejected** | The control arm's redundant line was at 0.0516 N, but the analysis assumed 5 N. Correct force gives **9.87 mm**. `force-mode-check.json` |
| "Production fitter 11.1 mm vs my 3.8 mm; both stable basins" | **Corrected** | With the correct force the same fitter gives 9.9 mm, close to the production fitter's 11.1 mm. The 3.8 mm was a force-model error, not a basin. |
| +6.7 % simulation time; no usable wall-time comparison | **Accepted** | 1018.34 s / 954.05 s = 1.0674; wall times 122.9 s vs 219.7 s, reversed by host load. |
| Global idle 1 N shrinks coverage about 4× | **Corrected** | Per-sweep ratios 4.07, 4.25, 5.14, 4.25; say **4.1–5.1×**. |
| Extra-range bias ≈0.35–0.86 mm | **Rejected as a range bias** | At nominal anchors the five-line redundant residual is +0.009…+0.016 mm mean (max 0.033 mm) per sweep; all five lines 0.015–0.019 mm RMS. The reported offset is the three-range pose error. `bias-diagnostic.json` |
| 16.4 → 2.3 mm σ is ~9× | **Corrected** | Same-dataset B σ is 15.56 → 2.30 = **6.8×**; 16.4 mm is arm A's σ, and mixing it with B's 2.30 gives 7.1×. The 9× is the perturbation median, which is 18.32 → **2.11** = 8.7× in the saved file. |
| Canonical sensor "doubled" the residual (0.041 → 0.017 mm) | **Corrected** | Direction is reversed and the numbers are not in the saved evidence. Physical sensor is better: five-line truth RMS 0.038 mm vs canonical 0.091 mm (2.4×); four-line 0.030 vs 0.088 mm (2.9×). `review-fits.json` |
| Fixed-line holding error bounds anchor motion | **Corrected** | Reported 0.0015 mm is a median; max absolute fixed-line error is 0.0026 mm (axis 0) and 0.0157 mm (axis 4) on arm B. It bounds encoder following error, not frame/anchor elasticity; anchors are rigid in this scene. |
| Held-out validation has whole-sweep folds, no point leakage | **Accepted** | `leave_one_sweep_out` removes one complete sweep per fold; replay reproduces the saved fold errors. Caveats: offsets are learned on all data, and the five-line held-out residual includes an extra equation. |
| "Truth" is inferred geometry, not a measured physics pose | **Accepted as a limitation** | Pose is trilaterated from three loaded lengths and nominal anchors; no effector pose trace was saved. Anchor error against nominal `M669` is still a valid input-truth comparison; tracking and "encoder − geometry" are not physical-pose measurements. |
| No cogging ablation | **Accepted as a limitation** | The mechanism is consistent with the separate direct-scene torque-floor evidence, but not ablated here. |
| Two known-torque constraints and reversed sensor roles handled | **Accepted with a wording fix** | 3 equilibrium + 2 known-force rows for 5 tensions is correct (already true in the four-line fit); the force row must use `source_sensor_anchor`. `l_drive`/`l_sensor` are canonical columns, not physically remapped; the static solve is otherwise role-agnostic. |

## 1. What reproduces

I re-ran the decisive saved-data analysis with the root `.venv` interpreter and a re-implementation of the fitter that reproduces the saved fits to <0.5 %:

* Two-sensor recording, production offsets, physical sensor: four-line **22.044 mm** (single start) / **22.129 mm** (two starts) and five-line **1.853 / 1.851 mm**. The saved `analysis.json` values are 22.129 and 1.851 mm.
* Six-start sensitivity: five-line 1.851–1.853 mm; four-line 22.04–22.25 mm. This part of the report is exactly reproducible.
* Leave-one-sweep-out (four whole-sweep folds) reproduces the saved held-out anchor errors: four-line 52.8, 18.6, 19.7, 45.4 mm; five-line 2.11, 2.67, 2.86, 2.03 mm. No point-level leakage.
* Tracking, coverage and timing: arm A encoder travel ~0 mm over 78.6–139.7 mm; arm B slope 0.995–1.000; coverage differences ≤0.37 mm; simulated-time ratio 1.0674.
* Arm A's five-line fit is genuinely invalid: 3127 mm anchor translation and 73.1 mm truth residual. The frozen range is not rescued by the fitter.

## 2. Force handling: the control arms were mis-modelled

`collect_run.py:111` records

```python
redundant_force_mode=(args.redundant_force_mode
                      or ("sensor" if "prototype" in str(collector) else "passive"))
```

The control collector is named `prototype-control`, so the substring test labels **both control arms "sensor"**, although their redundant line was at `force_low` (0.0516 N on `ctl-1`, 1.0 N on `ctl-low1p0`). The saved `invocation.json` and `manifest.json` carry the wrong label.

The data themselves identify the correct model: the consistency residual at the nominal anchors prefers the actual force for every arm.

| Arm | Actual redundant force | Four-line truth RMS, labelled mode | Four-line truth RMS, actual mode |
|---|---:|---:|---:|
| A `ctl-1` | 0.0516 N | 0.0691 mm ("sensor") | **0.0242 mm** ("passive") |
| B `twosensor-1` | 4.9914 N | **0.0295 mm** ("sensor") | 0.1060 mm ("passive") |
| C `ctl-low1p0` | 1.0 N | 0.0337 mm ("sensor") | **0.0216 mm** ("passive") |

Consequences:

* Arm A's production-gauge four-range baseline changes **3.8 → 9.87 mm** (rms 0.0125 mm). The correct fit is already present in the saved `analysis.json` under the misleading name `production_loader_with_pretension` (9.872 mm). The report's "3.8 vs production fitter 11.1, two stable basins" is an artifact: 9.9 vs 11.1 mm is ordinary fitter/model disagreement (the production fitter uses its rigid path here because `is_equal_strain` is false).
* Arm C's numbers change **60.8 → 48.8 mm** (four-line) and **14.3 → 21.2 mm** (five-line). The direction of the conclusion is unchanged; the five-line fit is still 11.4× worse than arm B.
* Arm A's five-line truth residual is essentially unchanged (73.27 → 73.08 mm), so "invalid" stands.
* Arm B is unaffected: its label "sensor" is correct. The headline 22.1 → 1.85 mm comparison is therefore **not** contaminated by this defect.

This is a bookkeeping defect, not a data defect. It does not overturn the main comparison, but it invalidates the arm-A supporting baseline and the "production fitter disagrees" narrative.

## 3. The claimed extra-range bias is a pose-inference artifact

The reviewed report's Result 1 table compares the redundant encoder with a pose trilaterated from the two fixed lines and the drive, using nominal anchors. For arm B this gives an encoder-minus-geometry residual of −0.81…−0.35 mm, which Result 2 then calls a "constant −0.35…−0.86 mm offset per sub-sweep (home/routing offset plus ~0.4 mm friction hysteresis)".

That residual is dominated by the weakly determined three-range pose, which is precisely the error the extra range is meant to remove. Checking the extra range against the nominal anchors with the full five-line static solve:

* Per-sweep mean residual on the redundant line: +0.009, +0.012, −0.003, +0.016 mm; max absolute 0.033 mm.
* All five lines: 0.015–0.019 mm RMS.
* Five-line truth RMS: 0.0384 mm. A +0.4 mm injected bias would raise it to 0.249 mm and the fit error to 18.0 mm.

So the saved data are consistent with a **sub-0.1 mm** extra-range error, not 0.35–0.86 mm. The ~0.4 mm direction-to-direction difference in the tracking table is likewise a pose-inference effect, not a demonstrated friction hysteresis. If the reported bias were real, the report's own fitter would give about 13–18 mm, not 1.85 mm.

The correct—and more useful—statement is the opposite risk: **the gain is fragile to extra-range bias.** Injecting a constant bias into the redundant range gives:

| injected bias | 0 | −0.1 / +0.1 mm | −0.2 / +0.2 mm | −0.4 / +0.4 mm | −0.8 / +0.8 mm |
|---|---:|---:|---:|---:|---:|
| five-line anchor error | 1.85 mm | 3.4 / 5.9 mm | 7.4 / 9.9 mm | 14.9 / 18.0 mm | 30.7 / 34.5 mm |

The gain therefore requires the extra range's absolute length scale to be accurate to roughly **0.05–0.1 mm**: at 0.1 mm it is still about 4–6× better than the 22 mm four-range fit, but no longer close to 1.85 mm. The simulation satisfies the requirement; hardware has not been shown to.

## 4. Is the extra range genuinely informative?

Yes, for this dataset. Three independent checks:

1. **Matched within-recording comparison.** Same recording, same poses, same starts, same optimizer, production offsets: 22.13 → 1.85 mm. The only change is whether the redundant length is an observation. The five-line fit's rms is *higher* (0.0241 vs 0.0152 mm), so the extra range is constraining the solution rather than being ignored.
2. **Placebo with destroyed correspondence.** Permuting the redundant ranges across points within each axis (three seeds) destroys the fit: **3107–3114 mm**. The gain is not generic regularisation from "more equations".
3. **Gauge robustness.** The five-line estimate is 1.47 mm (no home offsets), 1.85 mm (production offsets on lines 3–4), 1.85 mm (all five line offsets). The four-line estimate is 11.18, 22.13, 28.06 mm for the same three gauges. The extra range removes a weak direction; the four-range number is gauge-dependent.

That last point is the honest qualification: **22.1 mm is one defensible four-range baseline, not the four-range baseline.** Matched-geometry ratios are 7.6× (no offsets), 12.0× (production offsets), 15.1× (all offsets). The report's "9×" is therefore a single statistic, not a property of the method. The saved perturbation median is 18.32 → **2.11 mm (8.7×)**, and the linearised σ is 15.56 → 2.30 mm (6.8×). Quote a range, e.g. "about 7–15× at this budget", and keep the absolute caveat.

The geometric placebo I tried—replacing the extra range with the value predicted by the three-range pose—gave 17.0 mm, i.e. worse than the measured extra range. This is not evidence against the range; it confirms that the three-range pose is the biased quantity, so a "perfect" range derived from it is not perfect. It is a useful negative control that I did not use for the verdict.

## 5. Coverage and the 1 N idle-force tradeoff

* Two-sensor measurement-only pretension leaves coverage essentially unchanged: 874.43 → 874.17, 586.17 → 585.90, 638.01 → 637.64, 586.17 → 585.91 mm (≤0.37 mm, ≤0.06 %).
* Raising the global idle force to 1 N collapses it to 214.78, 138.05, 124.08, 138.05 mm: **4.07×, 4.25×, 5.14×, 4.25×**, not a uniform 4×. The third sweep is the worst.
* With the actual 1 N force, the fits are 48.8 mm (four-line) and 21.2 mm (five-line); the qualitative "raising idle force is not a substitute" conclusion survives, with corrected numbers.
* Mechanism, corrected: the 1 N idle force acts during the main pullout. The sensor payout captured at the pullout endpoint falls from 579 mm (A and B) to 193 mm (C), and the accepted drive interval is limited by that reserve. The reserve probes in arm B are themselves at 5 N, so "reserve acceptance runs against a pulling line" cannot be the whole cause. This is an inference from the saved `spool_reserves` metadata, not an ablation.

## 6. Fitter and statistics audit

* **Fitter.** `ext_fit.py` is a faithful copy of the production residual structure. The 3 equilibrium + 2 known-force rows for 5 tensions are correct. `source_sensor_anchor` is required: the collector swaps physical drive/sensor roles in the reversed sub-sweep and records the physical sensor, while `l_drive`/`l_sensor` remain the canonical columns. Using the canonical sensor worsens the five-line truth RMS from 0.038 to 0.091 mm. The report's recommendation is right; its sentence and numbers are wrong.
* **Home-offset gauge.** `fit_home_offsets` frees only `n-d = 2` offsets, on lines 3 and 4. The redundant axis is 2, 3, 0, 1 across the four sweeps, so three of four extra ranges get no offset of their own. The report notes this. My all-line-offset fit leaves the five-line result unchanged (1.85 mm), so this limitation does not drive the gain; but it does show how much the four-line result depends on the gauge.
* **Cross-validation.** Folds are whole sweeps, so there is no point leakage. Two real caveats: offsets are learned once on all four sweeps (including the held-out one), and the five-line fold residual includes the extra equation while the four-line one does not. The held-out **anchor errors** (2.0–2.9 vs 18.6–52.8 mm) are the comparable evidence; the held-out point-residual column should not be read as an apples-to-apples improvement.
* **Noise study.** `noise_sensitivity.py` perturbs `l_drive`, `l_sensor` and `mu`, but **not** the fixed-line lengths, despite the report's "perturbing every measured length by σ = 0.02 mm". It uses one start per seed and production offsets, so its four-line median inherits the offset-gauge inflation. Treat it as a basin/spread proxy, not a noise model. Its saved five-line values are 1.97, 1.23, 1.56, 2.25, 2.40, 2.98 mm (median **2.11**); the report's 1.97 mm is the first seed, not the median.
* **σ.** `max_anchor_std_mm` is a finite-difference linearised covariance from the anchor block only, with σ = max(0.02, 3·robust_scale). It is a local indicator, not a confidence interval, and it omits offset uncertainty. Its relative message is consistent with the fits; its absolute values should not be quoted as uncertainties.
* **Fixed-line error.** The reported "≤0.0015 mm" is a median. Max absolute fixed-line following error on arm B is 0.0026 mm (axis 0) and 0.0157 mm (axis 4). This is still negligible against 1.85 mm, but it is an encoder-loop error, not a bound on elastic or frame deformation. In this scene the anchors are rigid, so the simulation cannot exhibit that failure mode at all.
* **Force/flex bound.** Between arms A and B at matched drive setpoints, the sensor-length change differs by 0.26–0.31 mm on the three sweeps whose setpoints match to ≤0.03 mm, consistent with the 0.25 mm modelled 5 N stretch. The third sweep's setpoints differ by up to 1.9 mm, so its 1.6 mm difference is a pose mismatch, not a force effect. This supports the "5 N does not move the geometry" bound only inside the simulation's rigid-anchor model.
* **Truth.** Anchor error is measured against the nominal firmware `M669`, which is the simulation's data-generating anchor set; that part is sound. The "geometry" used in Result 1, the "encoder − geometry" offset and the fixed-line loaded error are model-relative. The saved logs contain no effector pose trace, so the inferred pose cannot be checked against a measured physics pose without a new run. The report's Limitations say this; Results 1–2 should say it too.

## 7. Prototype diff and force sequence

The prototype diff is small and does what the report says, with two wording corrections:

* `prototype-control` = production collector plus per-sweep fixed-target groups; no force changes.
* `prototype` = the above plus adding the redundant line to `extraSensors` for preload (`2·force_mid = 2.06 N`), measurement (`sensorForce = 4.9914 N`) and spool-reserve moves (5 N). The **main pullout** keeps the redundant line at `forceLow`; idle keeps `forceLow`.
* The report's "4.9914 N during preload + measurement only" should be "2.06 N during preload, 4.99 N during measurement and reserve moves; `forceLow` during the main pullout and idle". The operational claim—measurement-only pretension, pullout/idle force unchanged—is right.
* The 4-line fit already uses two known-force rows (sensor + compliant redundant line), so the extra range adds a *length equation*, not a second torque constraint. The real loader change is the redundant line's force value plus the `source_sensor_anchor` mapping.

Production files were not edited. The control/prototype collectors are experiment copies with rewritten import URLs.

## 8. Usefulness and production recommendation

The evidence supports a **conditional, gated experiment**, not a production default change:

* Do not use the compliant axis at `forceLow`; it is a stalled slack accumulator and a naive five-line fit is dragged metres away.
* If the extra range is pursued, it needs: measurement-only pretension at the sensor force; a tautness/validity gate on the redundant encoder's sweep travel; a force-aware loader that knows the redundant line's commanded force and uses `source_sensor_anchor`; and a two-known-torque static solve.
* Expect roughly **7–15× lower rough-budget anchor error at this four-sweep budget** in simulation, not a calibration certificate: the five-line absolute error is still 1.5–1.9 mm, and the magnitude is fragile to extra-range bias (7–10 mm at 0.2 mm, 15–18 mm at 0.4 mm).
* The cost is +6.7 % simulated collection time and no coverage loss. Wall time is not a discriminator here.
* The separate direct-HP5 investigation found a much more modest synthetic gain (1.47 → 0.89 mm at 20 points) and fragility at 0.05 mm bias. The scenes and budgets differ, but the disagreement in magnitude is a reason not to generalise the pulley-reduced factor to production.

What would change this conclusion: a hardware measurement of the extra range's absolute bias and tautness at the proposed force; a measurement-only pretension arm at 1–2 N with the same protocol; a production-sized budget where the four-range fit becomes well-conditioned; or a cogging/torque-floor ablation in this scene.

## 9. Limitations of this review

* Simulation and saved-data replay only; no hardware validity.
* I did not start a private physics runtime or a new collection. The saved logs contain no effector pose trace, so the inferred-pose critique cannot be resolved without a new run; I judged that unnecessary for the main conclusion.
* My re-implementation is independent but mirrors the reviewed residual/optimizer structure; it reproduces the saved fits to <0.5 %. It is not a different fitting algorithm.
* The bias-injection and offset-gauge variants are hypothetical perturbations. They bound sensitivity, not hardware error.
* I did not re-verify the `_prelim` force sweep or the two repeat collections point-by-point; the repeats are identical in the saved analysis, and the main conclusions do not depend on them.

## 10. Reproduce

All commands use `/home/torbjorn/repos/hp-sim5/.venv/bin/python` and read the reviewed evidence read-only from
`/home/torbjorn/repos/hp-sim5/research/output/program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f/session-2026-11-17-hp5-pulley-reduced-independently-compare-the-current-single-acti-f359cbd6/`.

```bash
cd research/output/program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f/session-2026-11-17-independently-review-reduced-hp5-matched-short-collection-eviden-748ceb19
PY=/home/torbjorn/repos/hp-sim5/.venv/bin/python
$PY evidence/review_fit.py            # matched fits, sensor roles, offset gauges, bias/placebo
$PY evidence/force_mode_check.py      # labelled vs actual redundant force
$PY evidence/bias_diagnostic.py       # extra-range residual at nominal anchors
$PY evidence/summary_checks.py        # coverage, timing, fixed errors, ratios
```

Saved outputs: [review-fits.json](review-fits.json), [force-mode-check.json](evidence/force-mode-check.json), [bias-diagnostic.json](evidence/bias-diagnostic.json), [summary-checks.json](evidence/summary-checks.json), [cv-replay.json](evidence/cv-replay.json). Helpers: [review_fit.py](evidence/review_fit.py), [force_mode_check.py](evidence/force_mode_check.py), [bias_diagnostic.py](evidence/bias_diagnostic.py), [summary_checks.py](evidence/summary_checks.py).

## 11. Isolation and cleanup

No production source, configuration, scene or shared build was edited. No control port, service or world was started or stopped; no full autocal run was performed. The only shared-tree effect was Python regenerating the disposable `evidence/__pycache__/*.pyc` of the reviewed session when importing its read-only analysis modules; `__pycache__` is git-ignored and no evidence or source file changed. All new artifacts are in this session directory. Nothing is committed or pushed.
