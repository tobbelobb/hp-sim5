# Review: compliance observability (fixed-pose force-step probe), candidate 6c9cd463

**Assessment: accept with caveats.** The probe's k estimate is real, reproducible and essentially unbiased in simulation. The headline D.z/max-error improvement on HP3, however, comes **almost entirely from adding the home-strain term, not from the measured k**. The claim as worded ("feeding probe k … worst-coordinate error 1.07→0.44 mm") over-attributes the gain to the probe and needs rewording before it enters the corpus.

## Integrity

- `--verify` on this review directory: `verified: true`, revision `6c9cd4635cd1c9eca7d78b8303d267c48d6e9784`, tree `cb1972ec…`.
- I inspected the candidate in worktree `/home/torbjorn/.t3/worktrees/hp-sim5/t3-cc036ed6-1b973479bb12c26c` at that commit. The scripts ran read-only from there. All outputs went to my scratchpad.

## Checks performed

Inspection is marked (I); reproduction is marked (R).

1. **(R) Offline refits reproduce exactly.** I gunzipped all ten committed `evidence/runs/*/sweeps.json.gz` and ran the committed `refit.py` on each. k̂ and all four variants' D.z match the submitted `*_refit.json` to 0.0 for all nine runs.
2. **(R) No selection of favourable runs.** The local output also holds two interrupted datasets that the evidence does not carry: `r1s080` and `r2s120.interrupted`, whose sweeps.json differs in bytes from `r2s120`. Refitting both gives results identical to their kept reruns: same k̂, and D.z equal to 1e-3 in every variant. I found no other runs from the probe campaign.
3. **(R) Attribution: home-strain term vs measured k.** Means over the nine runs in the convention frame (mm), recomputed from the refits:

   | Variant | mean abs(D.z) | mean abs(low z) (worst) | mean max-coordinate (worst) | mean coordinate rms |
   |---|---|---|---|---|
   | today | 0.842 | 0.908 (1.306) | 1.065 (1.464) | 0.543 |
   | config S + home strain | 0.266 | 0.168 (0.310) | **0.381** (0.702) | 0.191 |
   | probe k̂ + home strain | 0.272 | **0.060** (0.109) | 0.441 (0.696) | 0.177 |
   | oracle k | 0.253 | 0.053 (0.107) | 0.436 (0.695) | 0.174 |

   - Going from today to config removes about 95% of the D.z and max-error improvement.
   - Going from config to probe changes only the k-proportional common z shift: the gap to oracle shrinks from 0.17 to 0.03 sd, and mean abs(low z) falls from 0.17 to 0.06.
   - Going from config to probe does **not** improve abs(D.z), and it slightly worsens the mean max error. On max error the probe beats config in 2 of 9 runs and loses in 3.
   - So the measured k is causally responsible for the low-z improvement, and the refits show the dose-response: config−oracle D.z against (1 − k_true) has slope 0.90 mm with residual sd 0.010 mm. The measured k is **not** responsible for the 1.07→0.44 figure.
4. **(R) "D.z within 0.041 mm of oracle" is near-tautological.** The home-strain term is a translation with slope ≈0.9 mm per unit k, so ±4% in k is about ±0.04 mm by construction. The figure restates k precision. It is not independent evidence about D.z accuracy against truth. Against truth, D.z is dominated by the −0.35…+0.65 mm shape error, which no variant touches.
5. **(R) Is the probe's k biased by the anchor/pose fit?** I recomputed k̂ with four anchor sets: today's fit, the oracle fit, the iterated probe fit, and the **true anchors**. k̂ changes by less than 5e-4 relative in every run. The residual k error (−4.3…+4.1%) is therefore probe-model versus sim physics, not fit error.
6. **(R) Is the probe's k biased by ±0.36 N cogging?** I took the 146 non-zero steps (F0 = 0.75–2.76 N, mostly 2–2.8 N) and split k per step into the 2× level (dF = F0) and the 3× level (dF = 2F0).
   - **What each hypothesis predicts.** An additive, level-independent force error δ (cogging-like) gives an error ∝ δ/dF. That would make the 2× error about twice the 3× error, only weakly correlated between the two levels. A model or scale error gives the same relative error at both levels.
   - **Observed:**
     - 2× level: −2.5 ± 8.3%;
     - 3× level: −1.7 ± 6.5%;
     - **correlation 0.91**;
     - 2× minus 3×: −0.76 ± 3.6% (se 0.3%).
   - **Conclusion.** The per-step scatter is overwhelmingly proportional, not additive. That matches the report's pose-model attribution: rotation, routing, pinhole friction. The additive part implied by the 2×−3× gap is about δ ≈ −0.035 N, which is ≤1% bias on k. Cogging-induced bias in k̂ is therefore small at these force levels.
   - **What this does not show.** It cannot rule out a *proportional* force-realisation error, such as tension-proportional friction, which would masquerade as k. That is bounded only empirically: the per-run k̂/k_true − 1 is −1.3 ± 3.0% (se 1.0%, n = 9), not significant.
7. **(I) Repeatability does not discriminate.** The "six repeated probes agree to 0.001" argument does not show that the scatter is model error rather than cogging. The sim is deterministic, so cogging at a fixed motor phase also repeats exactly. Check 6 is the discriminating test, and it supports the report's conclusion.
8. **(I) Code.**
   - `sweep_data_collection.mjs::probeCompliance` is env-gated and returns `null` when `HP_SIM5_COMPLIANCE_PROBE` is unset, so the default behaviour is unchanged. `benchmark_history.run_autocal` only adds the variable to its allow-list.
   - No Rust or physics code changed, so the project physics checks are not required for this candidate.
   - `autocal/tests/test_imperfect_machine.py`: 4 passed (R).
   - Minor: the probe does not itself restore 1×. It relies on the last listed multiple being 1. With `HP_SIM5_COMPLIANCE_PROBE=2,3` the sensor would stay at 3×.
   - Minor: level 0 uses the sweep point's `mu`. The other levels use a fresh 16-sample mean, so the averaging differs. The 0.015 mm return-to-1× hysteresis shows this is negligible.
   - `refit.py` monkeypatches `pose_fit._hp4_home_strain`. `ellipse_fit` imports it by name, so the patch would *not* reach the ellipsoid fitter. That is harmless here, because `synth.production_fit` uses only `fit_pose_least_squares`. Any reuse of the patch for the recommended ellipsoid or HP5r experiment must wire k̂ explicitly.

## Consequential gaps and limitations

- **Over-attributed headline.** It should read roughly as follows: "ABC home strain with any reasonable S removes HP3's ~0.9 mm common z shift (max 1.07→0.38 mm). The probe's k̂ additionally removes the residual 0.9 mm × (k−1) common shift (low z ±0.31→±0.11 mm), but does not change D.z or max error at ±20% EA."
- **The "does not perturb the run" control is weak.** s100 (with probe) and s100np (without) take different trajectories. Today's D.z is −1.03 vs −0.57 mm, and the config/oracle D.z is −0.09 vs +0.36 mm (my refit of s100np). The max-error agreement (1.09 vs 1.18) is within the single-run knife edge. It shows the probe is not harmful *on average* only in the weak n = 1 sense.
- **"±4%" is a range over n = 9, not a bound.** With sd 3.0%, a 95% prediction interval is closer to ±7%. On HP3 that is about ±0.06 mm of z, which is still negligible.
- **The probe's value is untested where it matters.** HP3's slope is about 0.9 mm per unit k, so even configured S is within about 0.25 mm at ±20%. The case for measuring k rests on HP5r (8.4 mm per unit) and the ellipsoid shape (5–10 mm per unit), and those were not run live. The slope scan's mismatch emulation (rescaling compliance on matched-EA data) is only first-order.
- **Offline-only fit change.** Live selection and stopping used today's fit. Since the HP3 term is a pure translation, I expect little interaction, but that is untested.
- **The sim's lines match the probe model exactly** (linear EA/L). On hardware, creep and nonlinearity at 2–8 N against 1–3 N measurement tension are unknown.

## Scientific value vs code readiness

- **Scientific value: high.** The mechanism is clean: HP3's common z shift is slack-ABC home stretch, shown by exact replay. The cross-machine slope map is useful. A cheap, well-conditioned observable for k exists. The negative results should be kept:
  - multi-tension sweeps are biased (k̂ ≈ 1.5 for a true 1.25) at 2× cost;
  - the full per-point stretch model forced on HP3 does no better than home strain;
  - configured S already captures most of the HP3 gain.
- **Code readiness: research-grade only.** It is an env-var gate, k̂ is not stored, the fit change exists only as a monkeypatch, and the collector has no test. That is appropriate for this submission, but it is not mergeable as a feature.

## Decisive missing test

Run the probe live on **HP5 pulley-reduced sweeps (or the ellipsoid shape) at EA ×0.8 and ×1.2**, with k̂ feeding `dataset_stretch`, compared against configured S on the same datasets. If k̂ still lands within about 5% there, and it removes the ±1.7 mm D.z trade that configured S leaves, the probe's value is established. If it does not, the HP3 result reduces to "use the home-strain term", and the probe is not needed on HP3.

A secondary, cheaper test would separate proportional friction from true compliance in k̂: an exact replay of one probe run, to read the realised strand tensions at each probe level.

## Evidence

- Submission files: `files/57-report.md`, `files/04…12-*_refit.json`, `files/44-probe.py`, `files/49-refit.py`.
- My reproductions are in `/tmp/claude-1000/-home-torbjorn-repos-hp-sim5/c0757736-73bb-431a-937e-b76d5cb84bbc/scratchpad/`:
  - `runs/*.refit.json`, including `r1s080` and `r2s120.interrupted`;
  - `ksens.py`, the anchor-sensitivity check for k̂.

  They are ephemeral. Every number above can be regenerated from the committed datasets with the committed `refit.py`.
