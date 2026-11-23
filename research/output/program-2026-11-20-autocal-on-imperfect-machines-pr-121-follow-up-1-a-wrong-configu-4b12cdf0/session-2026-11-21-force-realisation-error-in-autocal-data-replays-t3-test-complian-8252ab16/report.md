# Force realisation error in autocal data: torque ripple, hysteresis, and a joint force-model + compliance fit

Status: complete, 2026-11-21. Simulation only. Branch `t3/autocal-force-realisation`.

**Reviewed** (review-338b0f4b33a8: accept with caveats). This version corrects three points:
- **Unobservability.** The claim that constant per-motor offsets/scales cannot be identified was wrong (section 3).
- **Circularity.** The headline 0.16 mm uses the generator's exact 50/turn basis.
- **Baseline.** Gains are attributed against plain joint k, not the live configured-k fit.

It adds the review's decisive test: a blind period scan plus per-motor offset/scale terms, on non-50/turn data with a 0.5 N offset on one motor (section 5b). HP3e is done. HP5r is **not done** (stopped at wrap-up).

## Summary

- **Cause (simulator).** At a settled sweep point, realised spool tension = commanded + 0.333 N · sin(50 θ), with θ the motor's encoder angle.
  - Residual sd is 0.02–0.05 N. Scale error is ≤ 0.2% and offset ≤ 0.006 N.
  - This is exactly the sim's torque-mode cogging: 1% of holding torque / spool radius, at the pole-pair frequency. Every other torque term vanishes at rest. That rules out hysteresis, load-dependent friction, torque-constant error and saturation.
  - At the effector the error is U × that.
  - The replay's "2.5% short steps" and "−0.53 N base level" are this ripple at fixed rotor angles (section 1).
- **Mostly variance, not bias.** Sweep-point phases look random, yet the fit is so ill-conditioned in top z that the ripple moves top z by about 1 mm, differently on each run.
  - The oracle map at true k takes max error from 1.00 ± 0.46 to 0.26 ± 0.07 mm on the HP3 ellipsoid (HP3e). On HP5r it goes from 0.80 ± 0.25 to 0.51 ± 0.11 mm (6 runs each).
  - On hp3e-s1-ea0.8 the map gives top z +0.30 mm against the replay's realised-force +0.33 mm (section 2).
- **What the sweep data can identify** (corrected after review, section 3).
  - **Identifiable.** Ripple in encoder angle, an offset that flips with the direction of motion, and, once the ripple is modelled, a constant offset or scale on one motor. With the oracle ripple map, +0.5 N on one motor raises the rms 14–16× on HP3e and about 2× on HP5r (review).
  - **Degenerate with k.** A common scale is exactly compliance. Joint k absorbs it, harmlessly for the geometry.
    - On HP3e meshes the commanded force is constant within a run, so a common offset, and each motor's offset vs scale, are degenerate too.
  - **Earlier claim withdrawn.** Section 3's first scan made the "unobservable" claim. It tested common perturbations at fixed k on ripple-uncorrected data, where the ripple floor hid them. Its 3–13 mm figures are fixed-k numbers.
  - **The real risk.** An unmodelled per-motor offset leaks into k (−13…−19%) and top z (+1.5…+2.7 mm per 0.5 N on one HP3e motor; review).
- **Mitigation: fit the realised force jointly, then k from the rms.** Per-motor force terms are added to the production fit: sin/cos at harmonics of the encoder angle, optionally a direction-dependent offset. With them profiled out, the rms becomes sharply k-sensitive, so k is fitted too. Configured stiffness and the probe are not used (section 4).
  - **Oracle basis.** These section-4/5 numbers fit the generator's exact period and form (50/turn sin/cos). Only amplitude and phase are free.
    - The 2×/4× cases change amplitude only.
    - The "unknown" case fits exactly the injected harmonic set.
    - The hysteresis regressor is the generator itself.
    - The blind test (section 5b) removes these crutches.
  - **HP3 ellipsoid, 11 runs (6 replayed, 5 new seeds).** Max error 0.16 ± 0.06 mm (worst 0.27), k +0.7 ± 0.3%.
    - The fair attribution of the force terms is against plain joint k on the same data: 0.81 ± 0.31 → 0.16 mm. Commanded at true k → oracle map at true k is 1.01 → 0.25.
    - The live configured-k fit (2.22 ± 1.43 mm) also carries the ±20% stiffness error.
  - **HP5r, 6 of 11 available runs (2 replayed, 4 new seeds).** 0.74 ± 0.21 mm (worst 0.93), k +3.8 ± 1.4%.
    - Live: 1.86 ± 0.63 mm. Plain joint k: 1.49 ± 0.61 mm.
    - HP5r keeps a +0.4–0.9 mm top-z bias that is neither compliance nor force. Adding the missing reeving to M666 Y fixes k to +0.5–3% but not top z (section 4.2).
- **Blind test (review follow-up, section 5b; HP3e only).**
  - **Setup.**
    - A per-motor period scan over 1–400/turn with leave-one-sweep-out cross-validation.
    - Per-motor offset, scale and direction terms as contrasts (the common part left to k), fitted jointly with the anchors and k.
    - Data: 3 live HP3e seeds with scene cogging at **137/turn** (no 50/turn ripple at all), 3 seeds with the sim's 50/turn ripple, and the same 3 with an extra 200/turn ripple at random phases. Each case adds +0.5 N on motor B.
  - **Periods.** The scan finds 137/turn on every motor of every live run, at 0.332–0.337 N against 0.333 N true.
  - **Offsets come back.** B sits at +0.33…+0.45 N relative to the motor mean (true +0.33), the others at −0.06…−0.39 (true −0.17). Without the extra 200/turn ripple the recovery is exact to ±0.01 N.
  - **Max error on the 7 held-out cases (seed 5 was used to develop the method):** 0.17 mean, range 0.03–0.49 mm, 6 of 7 under 0.3 mm.
    - Plain joint k on the same cases: 3.8 mm mean. Commanded forces at true k: 1.05 mm.
  - **Controls.** Without an offset, the full model costs nothing (0.06–0.14 mm).
  - **k.** It absorbs the offset's common part and lands −15…+27% off. This is harmless for geometry here.
  - **HP5r blind test not done** (stopped at wrap-up).
- **Larger and unknown force errors (manager request, section 5).**
  - **2× and 4× ripple.**
    - Emulated offline: joint fit 0.16 ± 0.07 mm and 0.31 ± 0.09 mm (5 runs each). Commanded forces at true k give 1.92 / 3.81 mm; plain joint k diverges.
    - Live with 2× scene cogging (HP3e, 2 runs): 0.16 ± 0.01 mm, against 4.63 / 4.38 mm live.
  - **Unknown ripple pattern** (0.33 / 0.33 / 0.67 N at 50 / 100 / 200 per rev, random phases, fitted at those three harmonics): 0.66 ± 0.30 mm (worst 1.07).
  - **Hysteresis-like ±0.2 N direction offset.**
    - It breaks the ripple-only joint fit: 1.56 ± 0.34 mm, k −27%.
    - Two-sided data kept as separate points is worse: 3.31 mm.
    - Fitting a per-motor direction offset jointly recovers 0.19–0.24 N against 0.2 N true, and gives 0.32 ± 0.22 mm (worst 0.64, 3 seeds; k +2.4 ± 6.8%). That term costs nothing (0.19 mm) when there is no hysteresis.
  - **HP5r collection fails at 2× cogging** (2/2 live). Force auto-tune reads the larger ripple as stiction and doubles the start force (1.03 → 2.02 N). Held lines then slip until the step budget runs out.
- **Home-stretch correction.** My HP4-style emulation on HP5r (2.5–3.5 mm) is a double count of HP5r's existing stretch model (review). The production default is gated to HP3/HP4 (t3/default-home-stretch-fit). This regression is **not** a property of the default; scoring with the real default was not done (section 6).
- **Time.** No collection time is added. Joint k replaces the 2×/3× probe and saves its 2.6–3.6% of simulated time. The cost is fit CPU, done here by finite-difference wrapping of the production fit:
  - HP3e: about 450 refits, ~2.5 min on 4 cores.
  - HP5r: about 700 refits, 25–120 min. One HP5r scan (seed 8) hit the 2 h job limit twice and is not reported.
  - Inside the least squares it would be 2–3 parameters per force motor.
- **Hardware** (corrected). The sim has only the ripple. It lacks static friction and hysteresis, torque-constant and radius errors, and pulley friction.
  - Fit the force model blind: periodic terms at scanned periods, per-motor offset/scale contrasts, and direction terms where a line moves both ways. A common scale and offset go to k.
  - In the sim, with a blind period, this recovers a 0.5 N single-motor offset with max error ≤ 0.3 mm in 6 of 7 held-out HP3e cases.
  - External force calibration (a known mass, read from both directions) then becomes a **cross-check**. It is not the only way to see per-motor bias. It remains the way to fix the common (absolute) force scale, which sweeps fold into k (section 7).

## 1. Cause in the simulator

[`cogging_regress.py`](evidence/scripts/cogging_regress.py) regresses realised − commanded spool tension at every replayed full encoder read.
- **Realised:** the sum of the spool-side segment forces from the compliance-probe study's exact replays ([inputs and checksums](evidence/results/replay/README.md)).
- **Commanded:** |torqueNm| / 0.03 m from the recorded `SetTorqueMode` events.
- **Reads used:** torque held ≥ 1000 steps (2 s) and rotor still.

| run | axis | n | err sd N | offset N | amp N at 50/rev (sin, cos) | scale | residual sd N |
| --- | --- | --- | --- | --- | --- | --- | --- |
| hp3e-s1-ea0.8 | A | 456 | 0.224 | +0.005 | 0.334 (0.334, 0.000) | −0.2% | 0.024 |
| | B | 494 | 0.212 | +0.005 | 0.333 (0.333, −0.007) | −0.1% | 0.023 |
| | C | 580 | 0.217 | +0.005 | 0.333 (0.333, −0.002) | −0.2% | 0.020 |
| hp5r-s1-ea1.2 | A | 145 | 0.238 | +0.006 | 0.342 | −0.2% | 0.054 |
| | B | 195 | 0.223 | +0.002 | 0.332 | −0.1% | 0.018 |
| | C | 227 | 0.220 | −0.001 | 0.334 | −0.0% | 0.055 |
| | D | 243 | 0.224 | +0.001 | 0.335 | −0.0% | 0.018 |

([HP3e](evidence/results/replay/hp3e-s1-ea0.8.jsonl.cogging.json), [HP5r](evidence/results/replay/hp5r-s1-ea1.2.jsonl.cogging.json).)

**Model check** (`motors()`, `src/wasm/cable_solver/src/engine.rs`). The torque-mode drive is:
- `command·droop`;
- `−(holding/maxSpeed)·ω`;
- windage ∝ ω|ω|;
- `−(coulomb + stiction·e^{−|ω|})·ω/(|ω| + 10⁻³)`;
- `−0.01·holding·sin(numPolePairs·θ)`.

At a static equilibrium (ω → 0) every term but the last vanishes, which accounts for each candidate cause:
- **Friction.** Even the "stiction" term goes to zero at rest, so the sim has no hysteresis: a level reproduces exactly on return.
- **Torque-constant error.** None. Firmware converts force to torque with the configured `R30`, `Q0` and `U`, and layering is off in every run, so the radius is exact.
- **Saturation.** Not active at these levels.
- **Pulleys.** Frictionless (prior replay).

The ripple period is 2π r / 50 = 3.77 mm of spool line. Sweep points are tens of mm apart, so their phases look random.

Dead code: `hp-sim-3d/app/torqueModeSystem.js` duplicates this torque model in JS. Only its own unit test uses it. It is left for the manager to delete (not in scope here).

## 2. How much of the fit error the ripple causes

[`cog_refit.py`](evidence/scripts/cog_refit.py) sets every force-mode line's force to commanded + U · 0.333 · sin(50 θ_enc). This is the oracle map, and it bounds any ripple compensation: a perfect dither, an anticogging table, or a fitted map. Fits use the production fitters through the prior study's `refit2.fit_at`.

On the prior study's 12 runs ([summary](evidence/results/offline/summary_cog.txt)):

| machine | variant | max error mm (worst) | top z mm |
| --- | --- | --- | --- |
| HP3e (6) | commanded, true k | 1.00 ± 0.46 (1.82) | +0.82 ± 0.66 |
| | ripple map, true k | **0.26 ± 0.07 (0.35)** | +0.26 ± 0.07 |
| | map with wrong sign, true k | 1.85 ± 0.82 (3.29) | +1.40 ± 1.30 |
| | commanded / map, configured k | 2.50 ± 1.68 / 2.75 ± 1.09 | |
| HP5r (6) | commanded, true k | 0.80 ± 0.25 (1.12) | +0.45 ± 0.64 |
| | ripple map, true k | **0.51 ± 0.11 (0.67)** | +0.45 ± 0.21 |
| | commanded, configured k | 2.60 ± 1.27 (5.35) | |

**Readings.**
- **The ripple is the whole realised-force effect.** The map reproduces the replay's realised-force refit.
- **It is mostly spread.** It takes the top-z sd from 0.66 to 0.07 mm on HP3e.
- **HP5r keeps a +0.45 mm mean.** The map does not touch it.
- **Compliance still has to be fixed.** At a ±20% stiffness error the force map alone does not help.

## 3. What the sweep rms can and cannot identify

[`force_param_scan.py`](evidence/scripts/force_param_scan.py) profiles the production fit rms at true k over a multiplier s on three perturbations: the ripple map, a common offset, and a common scale ([results](evidence/results/offline/), `*.scan_*.json`):

| run | ripple s = 0 → 1: rms mm (top z) | offset −1 / 0 / +1 N: rms (top z) | scale −10 / 0 / +10%: rms (top z) |
| --- | --- | --- | --- |
| hp3e-s1-ea0.8 | 0.0495 (+1.33) → **0.0025** (+0.30) | 0.0597 (+7.0) / 0.0495 (+1.3) / 0.0568 (−4.5) | 0.0505 (+2.7) / 0.0495 / 0.0494 (+0.0) |
| hp3e-s2-ea1.2 | 0.0302 (+1.01) → **0.0013** (+0.16) | 0.0390 (+4.6) / 0.0302 / 0.0370 (−2.8) | 0.0308 / 0.0302 / 0.0302 (+1.7 / +1.0 / +0.3) |
| hp5r-s1-ea1.2 | 0.0278 (+1.12) → **0.0029** (+0.54) | 0.0281 / 0.0278 / 0.0289 (+1.9 / +1.1 / +0.3) | 0.0279 / 0.0278 / 0.0279 (+0.4 / +1.1 / +1.9) |
| hp5r-s2-ea0.8 | 0.0357 (−0.89) → **0.0018** (+0.05) | 0.0357–0.0359 (+1.1 / −0.9 / −2.9) | 0.0365 / 0.0357 / 0.0362 (−3.0 / −0.9 / +1.2) |

**Readings.**
- **A force error that varies from point to point is identifiable.** The anchors cannot absorb it, so the rms has a sharp minimum at the truth.
- **Common offset/scale at fixed k, ripple uncorrected: flat.** The rms changes by at most 20%, often under 1%, while top z moves several mm.
- **Withdrawn after review: "a per-motor offset or scale is unidentifiable".** The scan above perturbs all motors at once, at fixed true k, on data still carrying the 0.05 mm ripple floor. Review-338b0f4b33a8 repeated it per motor with the oracle ripple map applied:
  - +0.5 N on one motor: rms 0.0025 → 0.034–0.040 mm (14–16×) on HP3e, 1.8–2.2× on HP5r.
  - +10% on one motor: 6–8× on HP3e, 2.4–4× on HP5r.
  - A common +10% with joint k returns exactly to the base rms and base top z: it is compliance.
  - So per-motor terms are visible. Unmodelled, they leak into k and top z: +1.5…+2.7 mm per 0.5 N on one motor.
  - Section 5b fits them.

## 4. Joint fit of the force terms, then of compliance

### 4.1 Method

[`joint_cog.py`](evidence/scripts/joint_cog.py) adds to every force-mode line's effector force

  U_a Σ_h (A_ah sin(h θ_a) + B_ah cos(h θ_a))

- θ_a is that motor's encoder angle, already in every data point.
- h defaults to 50/rev, the motor's electrical cycles per turn (M666 J200 / 4).
- h = 0 instead means A_a · (direction of that line's motion since the previous point) + B_a.

The fit rms² is quadratic in these parameters. Each step fits that quadratic from 1 + 2d + d(d−1)/2 production refits and takes the min-norm minimum, so directions the rms cannot see are left alone. Nothing reads the sim's ripple amplitude or phase.

[`joint_k_cog.py`](evidence/scripts/joint_k_cog.py) does this at every k on a fixed grid from 0.75 to 1.35 (step 0.05, no truth used). It takes k at the parabola vertex of the profiled rms, refines the force terms there, and scores the anchors.
- Optional `PERTURB` / `HOMESTRAIN` ([`perturb.py`](evidence/scripts/perturb.py)) add synthetic force errors or the emulated home-stretch term.
- `JOBS` spreads the grid over processes. This was checked identical to serial.

**Single run at fixed k** ([hp3e-s1-ea0.8](evidence/results/offline/hp3e-s1-ea0.8.joint50.json)).
- At true k the fitted amplitudes are 0.331 / 0.336 / 0.339 N, and top z goes 1.33 → 0.30 mm, matching the oracle.
- At configured k (25% off) the map is biased (0.41–0.48 N) and the rms stays 7× higher. That is the k signal.

### 4.2 Results on imperfect machines

Print-frame max |error| / top z, mm, against scene truth:
- **live:** the live run's own production fit at configured k.
- **cmd@true:** commanded forces at true k.
- **map@true:** oracle ripple map at true k.
- **jk:** joint rms k, commanded forces.
- **jk+cog:** joint k + fitted 50/rev ripple.

The new seeds come from the imperfect-machine harness: random low dz, top offset, true EA within ±20%, probe enabled. "offline" are the earlier study's EA × 0.8 / × 1.2 runs. From [`table.py`](evidence/scripts/table.py).

| set | run | k_true | live | cmd@true | map@true | jk (k/k_true) | jk+cog (k/k_true) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| offline | hp3e-s1-ea0.8 | 1.250 | 4.42 / +4.42 | 1.33 / +1.33 | 0.30 / +0.30 | 0.58 / +0.58 (1.048) | 0.18 / +0.18 (1.008) |
| offline | hp3e-s1-ea1.2 | 0.833 | 1.24 / −1.24 | 0.43 / +0.36 | 0.26 / +0.26 | 0.45 / +0.06 (1.035) | 0.19 / +0.19 (1.007) |
| offline | hp3e-s2-ea0.8 | 1.250 | 4.80 / +4.80 | 1.82 / +1.82 | 0.35 / +0.35 | 1.32 / +1.32 (1.032) | 0.27 / +0.27 (1.004) |
| offline | hp3e-s2-ea1.2 | 0.833 | 0.69 / −0.69 | 1.01 / +1.01 | 0.16 / +0.16 | 0.67 / +0.67 (1.042) | 0.05 / +0.05 (1.012) |
| offline | hp3e-s3-ea0.8 | 1.250 | 3.02 / +3.02 | 0.80 / −0.19 | 0.29 / +0.29 | 0.81 / −0.64 (1.028) | 0.14 / +0.14 (1.009) |
| offline | hp3e-s3-ea1.2 | 0.833 | 0.83 / −0.83 | 0.63 / +0.63 | 0.17 / +0.17 | 0.33 / +0.33 (1.041) | 0.10 / +0.10 (1.010) |
| live | hp3e-s5 | 1.049 | 2.56 / +2.56 | 2.03 / +2.03 | 0.25 / +0.25 | 1.18 / +1.18 (1.071) | 0.20 / +0.20 (1.004) |
| live | hp3e-s6 | 0.950 | 0.30 / +0.10 | 0.74 / +0.74 | 0.20 / +0.20 | 0.83 / +0.83 (0.994) | 0.14 / +0.14 (1.003) |
| live | hp3e-s7 | 0.870 | 2.26 / −2.26 | 0.79 / −0.79 | 0.21 / +0.21 | 1.27 / −1.27 (1.048) | 0.10 / +0.10 (1.011) |
| live | hp3e-s8 | 1.046 | 1.39 / +1.39 | 0.87 / +0.87 | 0.27 / +0.27 | 0.83 / +0.83 (1.004) | 0.23 / +0.23 (1.004) |
| live | hp3e-s10 | 1.170 | 2.94 / +2.94 | 0.69 / +0.69 | 0.33 / +0.33 | 0.68 / −0.68 (1.088) | 0.19 / +0.19 (1.008) |
| offline | hp5r-s1-ea1.2 | 0.833 | 2.55 / +2.55 | 1.12 / +1.12 | 0.54 / +0.54 | 1.03 / +1.03 (0.987) | 0.88 / +0.88 (1.051) |
| offline | hp5r-s2-ea1.2 | 0.833 | 2.42 / +2.42 | 0.75 / +0.75 | 0.31 / +0.31 | 1.86 / +1.86 (1.133) | 0.49 / +0.49 (1.022) |
| live | hp5r-s5 | 1.038 | 0.99 / +0.17 | 0.77 / +0.59 | 0.26 / +0.26 | 1.96 / −1.64 (0.802) | 0.41 / +0.41 (1.014) |
| live | hp5r-s6 | 0.935 | 1.02 / +0.53 | 0.75 / +0.00 | 0.47 / +0.47 | 0.80 / −0.08 (0.989) | 0.80 / +0.80 (1.043) |
| live | hp5r-s7 | 1.247 | 2.26 / −2.26 | 0.62 / −0.09 | 0.43 / +0.43 | 2.39 / +2.39 (1.229) | 0.93 / +0.93 (1.044) |
| live | hp5r-s9 | 0.874 | 1.90 / +1.90 | 0.78 / +0.78 | 0.53 / +0.53 | 0.90 / −0.14 (0.880) | 0.92 / +0.92 (1.051) |

| machine | n | live | cmd@true | map@true | jk | jk+cog | k/k_true jk | k/k_true jk+cog |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| HP3e | 11 | 2.22 ± 1.43 (4.80) | 1.01 ± 0.48 (2.03) | 0.25 ± 0.06 (0.35) | 0.81 ± 0.31 (1.32) | **0.16 ± 0.06 (0.27)** | +3.9 ± 2.5% | **+0.7 ± 0.3%** |
| HP5r | 6 | 1.86 ± 0.63 (2.55) | 0.80 ± 0.15 (1.12) | 0.42 ± 0.11 (0.54) | 1.49 ± 0.61 (2.39) | **0.74 ± 0.21 (0.93)** | +0.3 ± 14.3% | +3.8 ± 1.4% |

Not refitted: hp5r-s8 (live 0.91 mm; joint scan stopped twice at the 2 h job limit) and the 4 other replayed HP5r runs (stopped to free CPU for section 5).

**Readings.**
- **HP3e.** The joint fit is within a factor of two of the oracle map at true k, without knowing k or the ripple. It beats every variant on every one of the 11 runs.
- **HP5r.** The joint fit roughly halves the live error and removes plain joint k's large misses (k 0.80 and 1.23). But k is biased by +3.8 ± 1.4%, and top z stays +0.4 to +0.9 mm.
  - **Missing reeving.** With the 85.5 mm effector-side reeving added to M666 Y ([yfix](evidence/results/yfix/)), k comes to +0.5 / +3.2 / +1.9% but max error is 0.46 / 0.85 / 0.85 mm (seeds 5 / 6 / 7). So the reeving explains part of the k bias but not the top z.
  - **The remainder.** HP5r's rms after the ripple fit (0.0024–0.0029 mm on seeds 6, 7 and 9, against 0.0006–0.0011 on HP3e) shows a further unmodelled effect. The compliance-probe study's candidates are effector rotation at the high pose and unmatched points. It is not a force or compliance error. Investigating it is out of scope here.

### 4.3 Robustness to the ripple model

On hp3e-s1-ea0.8 at true k:
- **Unknown harmonics.** Fitting 50, 100 and 200 per rev together (18 parameters) recovers 0.333 N at 50 and ≤ 0.008 N elsewhere. Top z is +0.32 mm, against +0.30 with 50 alone ([multi](evidence/results/offline/hp3e-s1-ea0.8.joint_multi.json)).
- **Wrong harmonic alone.** 200/rev lowers the rms only 5% (0.0495 → 0.0468 mm) and makes top z worse (+1.83 mm) ([h200](evidence/results/offline/hp3e-s1-ea0.8.joint_h200.json)).
- **The rms says whether the force model is right.** It drops 20× with the right one.

## 5. Larger and unknown force errors

The manager asked for robustness to force errors larger than the sim's, and to unknown ones.

**Method.**
- **Offline:** [`perturb.py`](evidence/scripts/perturb.py) makes the recorded force of every force-mode line wrong by an extra amount. The realised physics is unchanged, so realised − recorded = sim ripple + extra.
- **Live:** a new harness option `--cogging-scale` scales each stepper's cogging torque in the scene.
  - It needed a two-line scene-loader change: `stepper:coggingTorque` and `coggingFreq` were already engine inputs, but the USD builder ignored them.
  - Defaults are unchanged. There is no physics change, and tests were added.

Results are from [`robust_table.py`](evidence/scripts/robust_table.py), HP3e on live seeds 5–8 and 10 ([results](evidence/results/robust/), [live 2×](evidence/results/cog2/)). Values are max |error| mm, mean ± sd (worst):

| case | n | cmd@true | joint k (plain) | joint k + force terms | k/k_true |
| --- | --- | --- | --- | --- | --- |
| baseline (sim ripple) | 5 | 1.02 ± 0.50 (2.03) | 0.96 ± 0.22 (1.27) | 0.17 ± 0.04 (0.23) | +0.6 ± 0.3% |
| ripple × 2 (offline) | 5 | 1.92 ± 0.97 (3.82) | 8.68 ± 2.33 (11.16) | **0.16 ± 0.07 (0.23)** | +0.7 ± 0.6% |
| ripple × 2 (**live**, scene cogging 0.02 Nm) | 2 | 2.75 ± 0.82 (3.57) | 5.84 ± 4.33 (10.18) | **0.16 ± 0.01 (0.18)** | +1.6 ± 1.0% |
| ripple × 4 (offline) | 5 | 3.81 ± 1.88 (7.45) | diverges | **0.31 ± 0.09 (0.41)** | +1.0 ± 2.3% |
| unknown: +0.33 N at 100/rev, +0.67 N at 200/rev, random phases; fit 50,100,200 | 3 | 2.54 ± 1.22 (3.81) | diverges | 0.66 ± 0.30 (1.07) | +1.2 ± 5.2% |
| hysteresis ±0.2 N (direction of motion), ripple-only fit | 3 | 1.66 ± 0.96 (3.01) | 1.22 ± 0.37 (1.62) | 1.56 ± 0.34 (1.91) | **−26.9 ± 3.9%** |
| hysteresis, two-sided data kept separate | 3 | 1.23 ± 0.49 (1.92) | 3.07 ± 1.43 (4.32) | 3.31 ± 0.28 (3.66) | −28.0 ± 1.0% |
| hysteresis, fit ripple + direction offset (h = 50, 0) | 3 | 1.66 ± 0.96 (3.01) | 1.22 ± 0.37 (1.62) | **0.32 ± 0.22 (0.64)** | +2.4 ± 6.8% |
| no hysteresis, fit ripple + direction offset | 1 | 2.03 | 1.18 | 0.19 | −0.4% |

HP5r: ripple × 2 offline on seed 5 gives 0.60 mm (k +3.1%), against 0.41 mm at 1×. Live HP5r at 2× could not be scored (below).

**Readings.**
- **Amplitude.** The joint fit is insensitive to the ripple's size. The live 2× runs agree with the offline emulation: hp3e-s5 0.18 vs 0.21 mm; live commanded 3.57 vs emulated 3.82 mm.
  - Without the force terms, larger errors make plain joint k diverge (k 0.3× or negative). That is worse than configured k.
- **Unknown pattern.** It costs accuracy (0.36–1.07 mm) but still beats every alternative by 3–4×. A generic harmonic set (here 50, 100, 200) works without knowing which is present.
- **Hysteresis.**
  - **It is the dangerous case.** An error that depends on the direction of approach and is not modelled biases k by about −27%.
  - **Two-sided approach averaged per point** cancels a symmetric offset to first order. Its emulation is the baseline row, at about 2× point dwell (one extra settled sample per point, ≥ 0.4 s plus settle).
  - **Keeping both directions as separate points does not work.** The ±offset sets an rms floor that flattens the k profile.
  - **Fitting the direction offset jointly mostly works.** It recovers 0.19–0.24 N on lines that move both ways, at no collection cost. Seeds 5 and 7 give 0.19 / 0.14 mm. On seed 8 the direction term partly trades with A's ripple (0.21 N instead of 0.33 N), giving 0.64 mm and k +12%. It is harmless when there is no hysteresis.
  - **Limit.** A line that moves only one way in the data has its direction term degenerate with its offset, which stays unknown (section 3).
- **HP5r collection at 2× cogging fails (2/2 live; [notes](evidence/runs/hp5r-cog2-s5/collection_notes.txt)).**
  - Force auto-tune reads the larger ripple as stiction and picks start/edge forces 2.02 / 5.55 N, against 1.03 / 3.96 N at 1×.
  - The held lines then slip (7 and 12 slip recoveries). Seed 5 ran out of the step budget and seed 6 exhausted slip recovery.
  - HP3e collected fine at 2×.
  - This is a collection-robustness issue for any motor with large ripple or breakaway torque, independent of the fit.
- **Phase-averaging dither.** Not run live. Its best case is the oracle map row of section 2. It needs pose dithers of P/2 ≈ 1.9 mm of sensor line per point, and it does not address hysteresis.

## 5b. Blind test: unknown period, per-motor offset (review follow-up)

**Method.** [`blind_force_fit.py`](evidence/scripts/blind_force_fit.py) is given no ripple period, phase or amplitude.
- **Linearisation.** The production residual (ellipsoid misfit, stretch-corrected) is linearised in every point's force on every force-mode line. A point's force only moves its own residuals, so one finite difference per motor suffices. The anchors and free home offsets are nuisance columns.
- **Per-motor terms.**
  - Up to 3 harmonics, each chosen greedily from 1–400/turn by leave-one-sweep-out squared prediction error and kept while it cuts that error ≥ 5%, then pruned if no longer worth 5%.
  - Offset and scale **contrasts** (sum to zero across motors; the common part trades with k, and for HP3e's constant-force meshes the common offset is exactly k·F).
  - A direction term on lines that move both ways.
  - Ridge prior: 1 N against 0.001 mm residual noise.
- **Profiling over k.** At every grid k the harmonics are reselected and the terms refitted (linearise / apply / production refit, twice). k is the vertex of the production rms. The final model is reselected and refitted at that k.
- **Variants.** "ripple": harmonics only. "full": all terms.
- **Development history.** The method was developed on hp3e-s5 (offB, r200offB), with two fixes during development:
  - free common offsets let k drift to the grid edge (`results/blind_v1_free_offsets/`);
  - selecting harmonics once at the plain-jk start diverged with two ripples (`results/blind_dev_v2_select_once/`).

  The other cases are held out. Results: [`results/blind/`](evidence/results/blind/), [summary](evidence/results/blind/summary.md).

**Data.**
- **Live:** HP3e seeds 5, 7, 8 with `--cogging-freq 137` (new harness option). The scene's torque-mode cogging is at 137/turn and there is no 50/turn ripple at all. Live production fit: 1.49 / 2.06 / 0.80 mm.
- **Offline:** HP3e seeds 5, 7, 8 (sim 50/turn ripple), with and without an extra 0.5 N 200/turn ripple at random phases.
- **Injection:** every case adds +0.5 N (effector) to motor B's realised force ("offB"). The `none` cases are the controls.

Max |error| / top z mm (k/k_true):

| run | case | cmd@true | plain jk | blind ripple + jk | blind full + jk | harmonics found (full) | constant terms rel. to mean, N (true: A −0.17, B +0.33, C −0.17) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| hp3e-f137-s5 | offB | 1.33 | 2.85 (0.90) | 0.64 (1.06) | 0.49 (1.02) | 137 on all; + 2 tiny each | −0.17, +0.34, −0.17 |
| hp3e-f137-s7 | offB | 0.78 | 2.65 (0.80) | 1.49 (0.86) | **0.04** (1.10) | 137 on all | −0.17, +0.34, −0.17 |
| hp3e-f137-s8 | offB | 1.02 | 3.89 (0.79) | 1.27 (0.99) | **0.03** (1.13) | 137 on all | −0.16, +0.33, −0.17 |
| hp3e-s7 | offB | 0.68 | 0.71 (0.91) | 0.55 (0.78) | **0.14** (0.95) | 50 on all | −0.17, +0.33, −0.16 |
| hp3e-s8 | offB | 1.79 | 3.21 (0.88) | 2.68 (1.00) | **0.16** (1.07) | 50 on all | −0.17, +0.34, −0.17 |
| hp3e-s7 | r200offB | 0.71 | 5.92 (0.36) | 0.42 (0.96) | **0.19** (0.85) | 50 on all; 200 on A, B | −0.22, +0.42, −0.20 |
| hp3e-s8 | r200offB | 1.03 | 7.28 (0.40) | 2.76 (0.91) | **0.12** (1.27) | 50 on all; 200 on B | −0.06, +0.45, −0.39 |
| *dev* hp3e-s5 | offB | 2.77 | 4.65 (0.83) | 1.34 (0.83) | 0.16 (1.09) | 50 on all | −0.17, +0.34, −0.17 |
| *dev* hp3e-s5 | r200offB | 1.21 | 8.57 (0.46) | 1.39 (0.75) | 0.17 (1.01) | 50 on all; 200 on A | −0.13, +0.39, −0.26 |
| control hp3e-f137-s5 | none | 0.77 | 0.60 | 0.12 | 0.14 (1.01) | 137 on all | ±0.01 |
| control hp3e-f137-s7 | none | 0.45 | 0.20 | 0.07 | 0.06 (1.01) | 137 on all | ±0.01 |
| control hp3e-f137-s8 | none | 0.66 | 0.66 | 0.12 | 0.12 (1.01) | 137 on all | ±0.01 |

The oracle bound (true 137/turn map, no offset, true k) on the three live runs is 0.25 / 0.21 / 0.23 mm.

**Readings.**
- **The period is found blind.** 137/turn is selected first on every motor of every live run, with amplitude 0.332–0.337 N (true 0.333). Extra selected harmonics carry ≤ 0.005 N and are harmless.
  - The 200/turn injection is found on 1–2 of 3 motors. Its fitted amplitudes are not reliable (up to 3× the true 0.5 N on a motor where it is found), yet the geometry stays good.
- **Offsets come back.** The single-motor 0.5 N offset is recovered in its identifiable (contrast) form in all 9 cases: exactly (±0.01 N) without the extra ripple, within 0.2 N with it.
- **Top z under 0.3 mm in 6 of 7 held-out cases.** Mean 0.17 mm, worst 0.49 mm (f137-s5, with oracle 0.25). That compares with plain joint k at 3.8 mm mean and commanded forces at true k at 1.05 mm.
  - The ripple-only variant fails with an offset present (0.42–2.76 mm). The offset terms are what make it work.
- **k is not recovered when an offset is present** (−15…+27%). The common part of the offset (0.17 N on ~2.2 N) is absorbed by k. On constant-force meshes that is an exact degeneracy and harmless for geometry here.
- **Controls.** With no offset, the full model equals the ripple-only fit and returns offsets ≤ 0.01 N.
- **Not done.**
  - **HP5r:** the blind refits of hp5r-s5, hp5r-s6 and a live hp5r 137/turn run (live 1.05 mm) were stopped unfinished at wrap-up. Earlier attempts crashed when a wild trial correction made a force negative; it is now clipped at 0.05 N.
  - Realistic stepper detent with several electrical harmonics at once was not run live; only the 137 + 50 + 200 combinations here were.

## 6. With the home-stretch correction

The commissioned default (integration branch 338ded23) is not implemented yet. I emulate it as an HP4-style term (`HOMESTRAIN=1`, [`perturb.home_strain`](evidence/scripts/perturb.py)): low-line home offsets k · T_strand / S · (|anchor| + Y/U), with the fit's own k.
- **HP3e.** Not applicable: the ellipsoid meshes record no `strand_tension_n`.
- **HP5r** ([results](evidence/results/robust/), `*.home.json`):

| run | cmd@true without / with | jk+cog without / with |
| --- | --- | --- |
| hp5r-s5 | 0.77 / 2.62 | 0.41 / 2.48 |
| hp5r-s6 | 0.75 / 1.90 | 0.80 / 2.63 |
| hp5r-s7 | 0.62 / 2.86 | 0.93 / 3.50 |

**Reading.** On HP5r the emulated term worsens every variant by about 1.2–2.6 mm (3 seeds).
- **Cause.** Per the review, this is a double count: HP5r's sweep fit already carries the stretch model, and the production home-strain term is gated to HP3/HP4 (t3/default-home-stretch-fit).
- **Status.** Do not cite this regression as a property of the default.
- **Not done.** Scoring the mitigations with the real default term. On HP3e the term cannot apply: the ellipsoid meshes record no strand target.

## 7. Which parts are physical

| effect | sim | real hardware | autocal consequence |
| --- | --- | --- | --- |
| torque ripple periodic in rotor angle | one harmonic, 1% of holding at 50/rev | yes. Stepper detent (often 200/rev), BLDC cogging at LCM(slots, poles), current-loop ripple; ~1–5%, possibly larger. ODrive ships anticogging calibration. | identifiable; fit jointly at a generic harmonic set (sections 4–5) |
| static friction / hysteresis | none: the friction term → 0 at rest | yes: bearings, seals, pinholes, line on spool | breaks k unless modelled. Fit a direction offset (needs both directions in the data) or average two-sided per point |
| torque-constant / current-sense error | none | 5–10% per motor plausible, temperature dependent | per-motor scale/offset: visible once the ripple is modelled; fit as contrasts (section 5b). The common scale is k. Absolute force needs an external reference |
| spool radius / buildup | exact (`Q0`, layering off) | buildup changes the radius with wound length | slowly varying scale; partly fitted by the spool model, otherwise as above |
| pulley/pinhole friction | none | yes | load- and direction-dependent offsets |
| breakaway seen by force auto-tune | ripple only | ripple + stiction | larger breakaway → higher forces → slips (HP5r at 2×) |

**Verdict.**
- **What generalises.** The mechanism removed here is physical: repeatable force error as a function of encoder angle and direction. The fix generalises, because it fits force terms against recorded variables rather than the sim's pattern, and it held at 2×, 4× and with unknown harmonics.
- **What does not.** The sim has no constant per-motor force biases of its own; they were only injected (section 5b).
- **Hardware recommendation** (corrected). Fit a blind force model:
  - scanned periods, per-motor offset/scale contrasts, direction terms;
  - jointly with k.

  Keep a one-time reference calibration (known mass, both directions) as a cross-check of the per-motor terms. It is also the only source of the absolute force scale, which sweeps cannot separate from compliance. This report's earlier statement that sweeps cannot see per-motor bias was wrong (corrected by the review).

## 8. Limitations and what would change the conclusion

- **Fit-side diagnostic, not ready code.** The finite-difference wrapper is slow for HP5r. Inside the least squares the terms cost almost nothing, but that integration is not done.
- **Assumptions.** The method assumes the ripple is a fixed function of encoder angle within a run: stable encoder zero, no load or temperature dependence. The sim satisfies this by construction.
- **Hysteresis model.** Hysteresis was emulated as a clean ±offset keyed on the motion between sweep points. Real friction can depend on dwell, speed and load.
- **HP3 default sweeps not covered.** They use equal-strain collection with a rigid fit (`dataset_stretch` returns None), so forces never enter the fit.
- **Done / not done** (at wrap-up).
  - **Done:** cause; oracle and joint fits (HP3e 11, HP5r 6 of 11 runs); robustness; blind period + offset test on HP3e (9 cases + 3 controls).
  - **Not done:** HP5r blind test (stopped); scoring with the real home-stretch default; integration of the force terms into the production least squares; hp5r-s8 joint scan (time limit).
- **HP3e seed 9 failed before any fit.** Size-tune measured 0.26 mm of travel, then "Measured sweep is too short to wind the sensor in half a spindle turn". This is unrelated to forces ([run](evidence/runs/hp3e-s9/)).
- **What would change the conclusion.**
  - On hardware, if the rms does not fall to a noise floor with the blind model, the force error has structure the model lacks: dwell, load, or temperature dependence.
  - An HP5r blind test (not done) could still fail. HP5r's sweeps vary the force along a profile, which should help, but its +0.4–0.9 mm residual is unexplained.
  - HP5r's remaining +0.4–0.9 mm top z could change the HP5r ranking once it is explained.

## 9. Reproduction

```bash
# live runs (two lanes at most); --cogging-scale is new in this branch
evidence/scripts/lane.sh output/fr hp3e hangprinter_3 "5 6 7 8 9 10" --ellipsoid-shape
evidence/scripts/lane.sh output/fr hp5r hangprinter_5_pulley_reduced "5 6 7 8 9"
evidence/scripts/lane.sh output/fr hp3e-cog2 hangprinter_3 "5 7" --ellipsoid-shape --cogging-scale 2
evidence/scripts/lane.sh output/fr hp5r-cog2 hangprinter_5_pulley_reduced "5 6" --cogging-scale 2
evidence/scripts/lane.sh output/fr hp3e-f137 hangprinter_3 "5 7 8" --ellipsoid-shape --cogging-freq 137
# blind period scan + per-motor offset/scale/direction contrasts + joint k (JOBS=4 over the k grid)
evidence/scripts/blind_lane.sh evidence/results/blind "output/fr/hp3e-f137-s5 ..." offB '[{"offset":0.5,"anchor":1}]'
.venv/bin/python evidence/scripts/blind_table.py evidence/results/blind/*.json
# cause (needs the prior replay dumps; see evidence/results/replay/README.md)
.venv/bin/python evidence/scripts/cogging_regress.py EVENTS.jsonl REPLAY.jsonl 0.03 50 1000
# refits (RUN = dir with imperfection.json, result.json, sweeps.json[.gz])
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/cog_refit.py RUN [amp]
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/force_param_scan.py RUN {cog|offset|scale}
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/joint_cog.py RUN [50|50,100,200]
JOBS=4 PYTHONPATH=$PWD .venv/bin/python evidence/scripts/joint_k_cog.py RUN [50|50,0|50,100,200]   # YFIX=1, HOMESTRAIN=1, PERTURB='[...]'
evidence/scripts/robust_lane.sh OUT "RUNS" NAME 'PERTURB_JSON' [HARMONICS] [HOMESTRAIN]
.venv/bin/python evidence/scripts/table.py evidence/results/offline evidence/results/live
.venv/bin/python evidence/scripts/robust_table.py evidence/results/robust evidence/results/live evidence/results/cog2
```

- **Inputs.** The prior 12 runs are in the compliance-probe study's `evidence/runs/`. The new runs (imperfection, result, manifest, gzipped sweeps, stdout) are in [`evidence/runs/`](evidence/runs/).
- **Raw events.** The `events.jsonl` files (0.15–0.39 GB each) are local only, in `output/fr/`, with SHA-256 and sizes in [`runs/raw_files.txt`](evidence/runs/raw_files.txt).
- **Code changes in this branch.** All are small, with tests:
  - `imperfect_machine --cogging-scale` and `--cogging-freq`;
  - the scene loader reads `stepper:coggingTorque` / `coggingFreq`.

  There are no physics or default changes. Revision: fdb977ea plus this branch's commit.
