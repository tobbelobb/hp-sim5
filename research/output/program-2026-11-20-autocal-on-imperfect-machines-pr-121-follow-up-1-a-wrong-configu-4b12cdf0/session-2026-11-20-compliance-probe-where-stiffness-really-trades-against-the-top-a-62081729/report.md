# Compliance probe where stiffness really trades against the top anchor

Investigation, 2026-11-21, branch `t3/test-compliance-probe-stiffness`.

**Base.** The base is `t3/top-anchor-compliance-observability` (6c9cd463: the probe, the harness and the refits) merged with `t3/uneven-low-anchor-heights` (a7a8d179: every low z free, gauge top x = y = 0, A.x = 0, home at the origin). Both merged cleanly in code. Only research docs conflicted. Every fit below is the merged, free-low-z fit.

**Evidence type.** Simulation only: headless native physics with the RRF host firmware, offline refits of the recorded datasets, and (section 7) exact replays of two runs that read the realised strand tensions and segment lengths. There is no hardware data.

**Errors.** Errors are fit minus scene truth in the print convention frame (`imperfect_machine.convention_frame`: top anchor on the z axis, A.x = 0, origin at home), in mm. "Top z" is the top-anchor (D on HP3, I on HP5r) z error. "max" is the largest coordinate error. k = configured EA / true EA, the compliance scale.

## Summary

1. **The probe is needed where the trade is strong.** On both strong cases, the configured-stiffness fit leaves 2.5–2.6 mm mean and up to 5.4 mm worst error at EA ×0.8/×1.2. Feeding the probe's k into the fit's compliance roughly halves the mean and the worst case:

   | Case (n = 6: 3 seeds × EA 0.8, 1.2) | k error, probe | max error, configured S | max error, probe k | max error, true scene k |
   | --- | --- | --- | --- | --- |
   | HP5r default sweeps | **+4.1 ± 2.0 %** | 2.60 ± 1.39 (worst 5.35) | **1.01 ± 0.32 (worst 1.61)** | 0.80 ± 0.27 (1.12) |
   | HP3 ellipsoid shape | **−4.0 ± 1.9 %** | 2.50 ± 1.84 (worst 4.80) | **1.40 ± 0.77 (worst 2.72)** | 1.00 ± 0.51 (1.82) |

   Mean ± sample sd, worst in brackets. The probe's max error is below configured S's in 11 of 12 runs. The exception is HP3e s2 EA×1.2, where configured S happens to sit near zero error (0.69 against 1.31 mm).
2. **The probe's k is biased by about 4%, with opposite signs on the two machines.** It is consistent over seeds (sd 2%) and appears with true anchors too, so it is not fit error. The exact replay (point 6) shows it is force realisation at the probe levels, plus a 2% line-model omission on HP5r.
   - D.z moves 9–12 mm per unit k on HP3e and 4–16 mm (mostly 7–10) on HP5r.
   - So the bias costs 0.2–0.5 mm of D.z against the true-k fit.
3. **The joint k fit from the rms needs no extra data, but it is unreliable on HP5r.**
   - **HP5r.** k is off by +2.5 ± 6.2%. Five runs land within −2.7…+6.6%, but one is +13%. Max error is 1.05 ± 0.41 (worst 1.86 mm), the same mean as the probe with a worse tail. The rms changes only 1–3% over k 0.75–1.35, so its minimum is shallow and depends on the seed.
   - **HP3e.** k is +3.8 ± 0.7%, and the max error is the best of all variants at 0.69 ± 0.35 mm. Its mean is better than the true-k fit's, because the ellipsoid fit has a D.z bias at true k that a slightly larger k cancels (point 5).
4. **Time cost.** The probe takes 128–146 s per HP5r run (3.0–3.4% of 3.9–4.4 ks simulated) and 167–230 s per HP3e run (2.5–3.7% of 6.1–6.8 ks). It adds 18–32 force steps. The joint fit costs no collection time and about a minute of CPU.
5. **What remains at the true k.** At the scene's true EA, top z is +0.45 ± 0.70 mm (HP5r) and +0.82 ± 0.72 mm (HP3e). The k that would zero top z is 0.93 ± 0.06 × true on HP5r and 1.07 ± 0.05 × true on HP3e. The probe errs in the opposite direction on both machines; section 7 explains why (point 6).
6. **Exact replay: the ±4% probe bias is force realisation, not the line model or the sim's compliance** (section 7, two replayed runs).
   - **HP3e.** The sensor line's stretch per realised tension matches the M666 model to 0.1%. Substituting the realised tensions takes the probe from −6.7% to −0.2% (median per step). The realised force steps fall 2.5% short of the command (median), with ±0.2 N per level.
   - **HP5r.** The probe's 1× level, the sweep's last point, is realised 0.53 ± 0.13 N below the command, while 2× and 3× come out 0.2–0.3 N above. So the steps look 4–9% too big. Realised tensions take the probe from +9.0% to +2.4%.
   - **The HP5r line-model part.** The remaining +2% is a line-model omission: M666's `U·d + Y` leaves out the 85.5 mm of effector-side reeving per path (pinhole → wheel → wheel → pinhole).
   - **Ruled out.** There is no pulley or pinhole friction: tension is identical in every segment of a path. Uneven sharing between parallel paths (10–65% spread from effector roll) does not change the line's compliance.
   - **The sign contradiction.** The top-z-zeroing k is not a compliance measurement either. With realised forces in the fit, the true-k top-z error falls from 1.33 to 0.33 mm (HP3e) and from 1.12 to 0.51 mm (HP5r). The zeroing k moves from 1.085 to 1.017 and from 0.84 to 0.92 × true.
   - **Why the signs differ.** The fit's error at true k is positive on both machines. The D.z-vs-k slope has opposite signs (−12 and +9 mm/k), so zeroing it needs k > 1 on HP3e and k < 1 on HP5r. The probe's bias comes from force realisation at its own levels, which has its own sign per machine. The two have no common sign; they are two outlets of one cause.
7. **Small corrections, tested offline on the 12 runs** (section 7.4).
   - **HP5r reeving added to Y.** The probe's k error goes from +4.1 ± 2.0% to +2.1 ± 1.9%. Anchors barely change (max 1.01 → 0.99 mm mean).
   - **k from the 2×→3× step only.** This drops the sweep-point level. HP5r: +0.5 ± 3.6%, worst max 1.61 → 1.27 mm, mean unchanged. HP3e: no help (−4.1 ± 3.4%).
   - **What this leaves.** No line-model correction removes the HP3e bias, because it is not in the line model. Only better force realisation (or measuring realised tension) can.
8. **Is the probe unnecessary?** Not on HP5r sweeps or the ellipsoid shape. Without it or a joint fit, a ±20% stiffness error costs 2–5 mm. On HP3 sweeps the probe stays unneeded (prior review). HP4, HP5 and the HP5r ellipsoid shape were not run live.

## 1. Question and comparisons

The prior investigation showed by offline slope scans that a wrong configured stiffness moves the top-anchor z strongly only on HP5r default sweeps (8.4 mm per unit k) and in the ellipsoid shape (5–10 mm). This investigation asks four things:
1. Does the opt-in fixed-pose compliance probe (`HP_SIM5_COMPLIANCE_PROBE=2,3,1`) recover k live there?
2. Does feeding that k into the fit remove the top-anchor z error?
3. Does a joint k fit from the sweep data alone (the fit rms) do the same?

**Runs.**
- **HP5r default sweeps.** Seeds 1, 2 and 4 at true EA × 0.8 and × 1.2. Seed 3 failed, see section 4.
- **HP3 ellipsoid shape.** Seeds 1, 2 and 3 at true EA × 0.8 and × 1.2.
- **Perturbations.** Each seed also carries the harness's random geometry: low dz ±30 mm and top offset ≤ 50 mm.
- **Config.** config.g stays nominal (M666 S = 20000).
- **Parallelism.** At most two simulations ran at a time.

**Refits.** [`refit2.py`](evidence/scripts/refit2.py) refits each recorded dataset with that shape's production fitter:
- the sweep autocal's two-stage point fit; or
- `ellipsoid_autocal.fit`, the ellipsoid model.

The fit is also started from the live run's final anchors. Every config-derived compliance (`FlexModel.from_m666`, HP4 home strain) is multiplied by k. There are four variants:
- **config**: k = 1, which is today's default. It reproduces the live result in all 12 runs.
- **probe**: k from [`probe.py`](evidence/scripts/probe.py), evaluated on the config fit's anchors.
- **joint**: k at the minimum of a parabola through the rms grid (0.75–1.35, step 0.05).
- **oracle**: k = 1/(scene EA scale).

**Estimator change.** HP5r sweeps hold three lines (2 fixed + drive). The sensor and one extra force-mode line run at constant force, so the force-step estimator now:
- solves equilibrium over the held lines only, and
- adds each constant-force line as a k-free length equation for the effector shift (least squares).

On HP3 it reproduces the earlier k to 1e-9.

## 2. Results

Per-run values are in [`results/live/*_refit.json`](evidence/results/live/). The table below comes from [`table2.py`](evidence/scripts/table2.py) → [`live_summary.md`](evidence/results/live_summary.md). Variants are listed in the order config / probe / joint / oracle.

| Run | k true | k probe | k joint | top z | max | probe s / sim s |
|---|---|---|---|---|---|---|
| hp3e-s1-ea0.8 | 1.250 | 1.206 (−3.5%) | 1.310 (+4.8%) | +4.42 / +1.87 / +0.58 / +1.33 | 4.42 / 1.87 / 0.58 / 1.33 | 192 / 6626 (2.9%) |
| hp3e-s1-ea1.2 | 0.833 | 0.779 (−6.6%) | 0.862 (+3.5%) | −1.24 / +0.94 / +0.06 / +0.36 | 1.24 / 0.94 / 0.45 / 0.43 | 230 / 6232 (3.7%) |
| hp3e-s2-ea0.8 | 1.250 | 1.176 (−5.9%) | 1.290 (+3.2%) | +4.80 / +2.72 / +1.33 / +1.82 | 4.80 / 2.72 / 1.33 / 1.82 | 192 / 6637 (2.9%) |
| hp3e-s2-ea1.2 | 0.833 | 0.804 (−3.6%) | 0.868 (+4.2%) | −0.69 / +1.31 / +0.67 / +1.01 | 0.69 / 1.31 / 0.67 / 1.01 | 182 / 6075 (3.0%) |
| hp3e-s3-ea0.8 | 1.250 | 1.224 (−2.1%) | 1.285 (+2.8%) | +3.02 / +0.14 / −0.64 / −0.19 | 3.02 / 0.79 / 0.81 / 0.80 | 169 / 6770 (2.5%) |
| hp3e-s3-ea1.2 | 0.833 | 0.816 (−2.1%) | 0.868 (+4.1%) | −0.83 / +0.77 / +0.33 / +0.63 | 0.83 / 0.77 / 0.33 / 0.63 | 167 / 6553 (2.5%) |
| hp5r-s1-ea0.8 | 1.250 | 1.272 (+1.8%) | 1.226 (−1.9%) | −1.14 / +0.89 / +0.54 / +0.72 | 1.74 / 0.89 / 0.73 / 0.72 | 133 / 3892 (3.4%) |
| hp5r-s1-ea1.2 | 0.833 | 0.891 (+6.9%) | 0.823 (−1.3%) | +2.55 / +1.61 / +1.03 / +1.12 | 2.55 / 1.61 / 1.03 / 1.12 | 130 / 4144 (3.1%) |
| hp5r-s2-ea0.8 | 1.250 | 1.273 (+1.8%) | 1.265 (+1.2%) | −4.97 / −0.51 / −0.64 / −0.89 | 5.35 / 0.89 / 0.96 / 1.09 | 140 / 4443 (3.2%) |
| hp5r-s2-ea1.2 | 0.833 | 0.870 (+4.4%) | 0.944 (+13.3%) | +2.42 / +1.12 / +1.86 / +0.75 | 2.42 / 1.12 / 1.86 / 0.75 | 128 / 4200 (3.0%) |
| hp5r-s4-ea0.8 | 1.250 | 1.306 (+4.5%) | 1.216 (−2.7%) | −0.30 / +0.79 / +0.47 / +0.59 | 1.70 / 0.79 / 0.84 / 0.72 | 146 / 4244 (3.4%) |
| hp5r-s4-ea1.2 | 0.833 | 0.875 (+5.0%) | 0.888 (+6.6%) | +1.87 / +0.77 / +0.88 / +0.39 | 1.87 / 0.77 / 0.88 / 0.39 | 131 / 4193 (3.1%) |

Spread over seeds and both EA, n = 6 per case:

| Case | Variant | top z mean ± sd | abs(top z) mean (worst) | max mean ± sd (worst) |
|---|---|---|---|---|
| HP5r | config | +0.07 ± 2.90 | 2.21 (4.97) | 2.60 ± 1.39 (5.35) |
| HP5r | probe | +0.78 ± 0.71 | 0.95 (1.61) | 1.01 ± 0.32 (1.61) |
| HP5r | joint | +0.69 ± 0.82 | 0.90 (1.86) | 1.05 ± 0.41 (1.86) |
| HP5r | oracle | +0.45 ± 0.70 | 0.74 (1.12) | 0.80 ± 0.27 (1.12) |
| HP3e | config | +1.58 ± 2.81 | 2.50 (4.80) | 2.50 ± 1.84 (4.80) |
| HP3e | probe | +1.29 ± 0.91 | 1.29 (2.72) | 1.40 ± 0.77 (2.72) |
| HP3e | joint | +0.39 ± 0.66 | 0.60 (1.33) | 0.69 ± 0.35 (1.33) |
| HP3e | oracle | +0.82 ± 0.72 | 0.89 (1.82) | 1.00 ± 0.51 (1.82) |

**Readings.**
- **Where the error sits.** On HP3e, configured S's error is almost entirely top z, with a sign set by EA: ×0.8 gives D.z +3 to +5 mm, and low z stays within ±0.5 mm in every variant. On HP5r, low z moves as well: −0.5 to −2.4 mm at EA ×0.8 and +0.4 to +0.7 mm at ×1.2, while top z moves −0.3 to −5 mm. With the probe's k every HP5r low z is within ±0.33 mm (per-run JSON).
- **Both remedies shrink the spread.** With probe or joint k, the sd of top z over seeds falls from about 2.9 mm to 0.7–0.9 mm.
- **Probe against joint.** On HP5r the probe has the better worst case (1.61 against 1.86 mm). On HP3e the joint fit is better in mean (0.69 against 1.40 mm).

**Offline slope scan on the merged base** ([`slope_scan_freez.jsonl`](evidence/results/slope_scan_freez.jsonl), nominal datasets from `e022c095`):
- HP5r sweeps: 8.42 mm per unit k, rms minimum at k ≈ 1.03–1.05, unchanged by the free-low-z fit;
- HP3 ellipsoid: −10.5 mm/k;
- HP5r ellipsoid: −4.9 mm/k.

The live per-run slopes ([`zero_k.txt`](evidence/results/zero_k.txt)) are −8.8 to −12.8 mm/k on HP3e and +3.5 to +16.2 mm/k on HP5r.

## 3. Probe precision and structure

[`probe_structure2.py`](evidence/scripts/probe_structure2.py) → [`probe_structure.txt`](evidence/results/probe_structure.txt) recomputes each step's k with the **true anchors**.

- **HP3e.** The per-step median is 0.93–0.97 × true in all six runs, with every sensor line low. There is no clear trend with height. The 2× and 3× levels agree (corr 0.45–0.91), so the error is proportional, not additive: cogging would make the 2× level about twice as wrong as the 3× level.
- **HP5r.** The median is 1.01–1.09 × true. The scatter is larger: the 2× and 3× levels correlate only −0.04…0.84, and a few high poses read +20–30%.
- **The bias is not fit error.** The true anchors give the same bias as the fitted ones.
- **Against HP3 sweeps.** The earlier HP3 sweeps gave −1.3 ± 3.0% (n = 9). So the probe's model error depends on the machine and pose set, not only on noise.

## 4. Robustness note: HP5r seed 3 fails the rough bootstrap

`hp5r-s3` stops with `unreliable-rough-bootstrap` at about 1500 s simulated at both EA ×0.8 and ×1.2. Its perturbation is low dz −24.9 / −15.8 / +18.1 / +4.9 mm, with the top offset (−4.3, 1.9) mm.
- **Not the probe.** The same seed fails identically without the probe ([`runs/hp5r-s3-ea1.2-noprobe`](evidence/runs/hp5r-s3-ea1.2-noprobe/result.json)).
- **Deterministic.** A probe rerun fails at the same simulated time.

The failure is an autocal robustness problem on perturbed HP5r geometry, outside this question. Seed 4 replaced seed 3.

## 5. Alternative explanations and limitations

- **The "oracle" is exact for the sim's compliance; the commanded forces are not.** The earlier HP3 replay reported an effective EA of 17.8–19.8 kN against 20 kN. Here, with realised tensions and segment lengths, the HP3 sensor line's stretch matches the model at the scene EA to 0.1%. HP5r's matches it to +2%, which is the missing effector-side reeving. What separates the fits from the oracle is force realisation (section 7); the line model accounts only for HP5r's 2%. The 2026-11-18 regression most likely absorbed tension or geometry errors into EA, and this replay supersedes it for these machines.
- **The joint fit's success on HP3e is partly the ellipsoid's own bias.** At true k, the ellipsoid fit leaves +0.8 mm of top z on HP3 (+2.6 mm on the nominal dataset). The replay shows this bias is mostly force realisation in the sweep data (1.33 → 0.33 mm on hp3e-s1-ea0.8 with realised forces). The rms minimum sits at a slightly larger k that happens to cancel part of it. That is fortunate for this sim, but it is not evidence that the rms measures true compliance. On HP5r the same rms curve is too shallow: one run in six is 13% off.
- **Small n.** There are three seeds per EA per case, and single runs are knife-edge (±0.5 mm). The probe's advantage over configured S (2.6 → 1.0 and 2.5 → 1.4 mm mean) is large against that, but the probe-against-joint ranking is not established.
- **Offline wiring only.** The probe's k and the joint k enter only offline refits. Live selection and stopping used configured S. For the ellipsoid, the live run also stops on anchor stability with configured S, so a live-k run could stop at a different time.
- **Sim lines are linear EA/L.** On hardware, creep, nonlinearity, and the fact that the probe tests 2–7 N while measurement runs at 1–3 N are untested.
- **Not covered:** HP4, HP5, the HP5r ellipsoid shape (offline slope −4.9 mm/k) and EA ×0.9/×1.1.
- **Queue interruptions.** Two runs, `hp5r-s2-ea0.8` and `hp3e-s3-ea0.8`, were killed when a turn ended and were rerun from scratch. Their partial directories hold no dataset and are not used.

## 6. Recommendations

1. **HP5r default sweeps.** Measure k, with the probe at about 3% time or the joint rms fit at no time cost, and feed it to `dataset_stretch`'s compliance multiplier. The probe has the better worst case. A cheap robust choice is to run the joint fit and fall back to (or average with) the probe when they disagree by more than about 5%. That rule is untested.
2. **Ellipsoid shape.** Measured k removes the 2–5 mm stiffness error. The remaining ±1 mm is the ellipsoid fit's own top-z bias, mostly force realisation in the sweep data (section 7.3), which no k choice removes reliably. The ellipsoid stays opt-in.
3. **Before productionising.** The ±4% probe bias is force realisation (section 7). It will not go away through the line model, and on hardware it will depend on the motors.
   - **Options.** Measure the realised step (a load cell or current sense), average the force over motor phase at each level (the existing phase-averaged-detent lead), or step from a level the probe itself sets rather than the sweep's last point. The 2×→3× variant helps HP5r only.
   - **HP5r reeving.** Add the effector-side reeving length to the HP5r M666 Y (+85.5 mm) in config.g. That is a config-level fix for both the probe and the flex model. It was tested offline only, and the anchors barely change.
   - **Reduced probe.** A reduced probe (first sweeps, one level) was ±9% on HP3. That would give about 1 mm of top z on HP3e/HP5r, which is too coarse here.
4. **The bigger lever for top z is force realisation in the sweep data itself.** With realised forces the fit's true-k top-z error drops 0.6–1.0 mm on both replayed runs, more than any k choice gains.
5. **HP5r seed-3 bootstrap failure.** Hand it to the bootstrap or robustness work.

## 7. Exact replay attribution of the probe bias

### 7.1 Method

[`replay_probe.mjs`](evidence/scripts/replay_probe.mjs) replays a run's recorded `events.jsonl` in the native physics, as the 2026-11-18 replay did. At every full encoder read it dumps:
- the effector pose;
- for every cable joint, its constraint force, rest length and attachment distance.

It must sit one directory below the repo root (`output/`). Replays were run for two runs, at two simulations at most:
- `hp3e-s1-ea0.8`: probe −3.5% overall, 4.6 min wall.
- `hp5r-s1-ea1.2`: probe +6.9%, the largest HP5r bias, 7.3 min wall.

**Matching levels to reads.** Each probe level is matched to the reads within 0.005° (every axis) of its mean encoder angles. 48 of 48 and 20 of 20 steps matched.

**Per line.** Effector-side force F is the sum of the long-strand tensions. Stretch s and geometric length g are the per-path sums of (distance − rest) and distance, divided by U. The replay's length check |Δm − Δ(g − s)| is 0.05 mm median on HP3e (max 0.11) and ≤ 0.05 mm on HP5r.

Scripts:
- [`probe_replay_attrib.py`](evidence/scripts/probe_replay_attrib.py): k with true pieces substituted;
- [`level_forces.py`](evidence/scripts/level_forces.py): realised force per probe level;
- [`point_forces.py`](evidence/scripts/point_forces.py): realised force at every sweep point;
- [`strand_share.py`](evidence/scripts/strand_share.py): sharing between paths and within a path;
- [`realised_refit.py`](evidence/scripts/realised_refit.py): fit with the realised forces.

Outputs are in [`results/replay/`](evidence/results/replay/).

### 7.2 Where the probe's error comes from

Medians over steps of k̂ / k_true:

| Estimator | hp3e-s1-ea0.8 | hp5r-s1-ea1.2 |
| --- | --- | --- |
| probe as run (commanded force, point-mass equilibrium) | 0.933 | 1.090 |
| realised sensor and held-line tensions (extra force line held at dT = 0) | 0.998 | 1.021 |
| all realised tensions, incl. the extra force line | 0.998 | 1.024 |
| true geometry (replay Δg of the sensor line), commanded force | 0.973 | 1.064 |
| true geometry and realised sensor tension: the sensor line's own compliance against the model | **0.999** | **1.021** |
| realised sensor step / commanded step | 0.975 | 1.042 |

**Readings.**
- **Line model.** The sensor line's stretch per realised tension matches `(U d + Y)/(S U² N)` at the scene EA to 0.1% on HP3. On HP5r it is +2.1%, which matches the effector-side reeving the model omits: per path, pinhole → wheel → wheel → pinhole is 85.512 mm, and (2·1.92 + 0.429 + 0.0855)/(2·1.92 + 0.429) = 1.020.
- **Friction.** Pulleys and pinholes are frictionless in this sim: within a path every segment carries the same tension, with spread exactly 0.
- **Parallel paths.** They share the load unevenly. The median spread is 9–10% on the HP3 low lines, 39% on D, 3% on the HP5r low lines and 65% on HP5r's I, from effector roll. Parallel paths of equal length still add up to the same line compliance, and the replay confirms that the uneven sharing does not bias k.
- **Force realisation is the bias.**
  - **HP3e.** The realised steps are 2.5% short of the command (median). Across levels, realised ≈ 0.988 × commanded + 0.01 N, with ±0.19–0.24 N per level ([levels](evidence/results/replay/hp3e-s1-ea0.8.levels.json)).
  - **HP5r.** The probe's base level, the sweep's last point, is realised −0.53 ± 0.13 N below command. Returning to 1× reproduces exactly −0.53 N, so it is not hysteresis. 2× and 3× land +0.27 and +0.19 N above. The steps therefore look 4–9% too big ([levels](evidence/results/replay/hp5r-s1-ea1.2.levels.json)).
  - **The force-mode extra line is not constant.** Its realised tension moves +0.2…+0.6 N at low poses and about −1 N at z ≈ 988 mm.
- **High poses.** At the one high HP5r pose (z ≈ 988 mm), effector rotation of 0.01° adds 0.02–0.04 mm of geometry the point model lacks. True tensions there still leave +10%, and true geometry brings it to +0.5…+5.5%.
- **Sweep points.** At the sweep points themselves, realisation is unbiased on average: HP3e +0.014 N (sd 0.21–0.23), HP5r +0.04 N (sd 0.45–0.48) ([HP3e](evidence/results/replay/hp3e-s1-ea0.8_point_forces.txt), [HP5r](evidence/results/replay/hp5r-s1-ea1.2_point_forces.txt)). HP5r's larger scatter matches its 4 effector strands per motor (U·N = 4): a spool-side cogging error is multiplied by 4 at the effector.

### 7.3 The sign contradiction

The top-z-zeroing k is a property of the fit's other errors, not of compliance. Replacing every force-mode line's commanded force by the replay's realised force ([`realised_refit`](evidence/results/replay/)):

| Run | top z at true k: commanded / realised forces | zeroing k / k_true: commanded / realised |
| --- | --- | --- |
| hp3e-s1-ea0.8 (120/120 points matched) | +1.33 / **+0.33** mm | 1.085 / **1.017** |
| hp5r-s1-ea1.2 (95/100 points matched) | +1.12 / **+0.51** mm | 0.843 / **0.922** |

- **Why the zeroing k points opposite ways.** The fit's error at true k is positive on both machines. The slopes are −12.5 mm/k (ellipsoid) and +8.6 mm/k (HP5r sweeps), so zeroing it needs k above true on one and below on the other.
- **Why the probe's bias has its own sign.** It comes from force realisation at its own levels: HP3e steps are realised short, and HP5r's base level is realised low.
- **No contradiction.** Both are the same physical cause, commanded ≠ realised tension in the force-mode motors, entering two different estimators. Their signs need not agree.
- **What HP5r's remaining 0.5 mm is not.** It is not the line model (the reeving fix moves the zeroing k the other way). The unmatched points and the high-pose rotation are candidates.

### 7.4 Corrections tested offline on all 12 runs

[`refit3.py`](evidence/scripts/refit3.py) → [`results/corrections/`](evidence/results/corrections/). Mean ± sd over 6 runs per case:

| Case | Variant | k error | max error mm (worst) | top z mm |
| --- | --- | --- | --- | --- |
| HP3e | probe | −4.0 ± 1.9% | 1.40 ± 0.77 (2.72) | +1.29 ± 0.91 |
| HP3e | probe, 2×→3× step only | −4.1 ± 3.4% | 1.39 ± 1.04 (3.40) | +1.36 ± 1.07 |
| HP3e | oracle | 0 | 1.00 ± 0.51 (1.82) | +0.82 ± 0.72 |
| HP5r | probe | +4.1 ± 2.0% | 1.01 ± 0.32 (1.61) | +0.78 ± 0.71 |
| HP5r | probe + reeving in Y | +2.1 ± 1.9% | 0.99 ± 0.32 (1.60) | +0.74 ± 0.72 |
| HP5r | 2×→3× step only | +0.5 ± 3.6% | 1.01 ± 0.20 (1.27) | +0.49 ± 0.90 |
| HP5r | 2×→3× + reeving in Y | −1.3 ± 3.6% | 1.00 ± 0.22 (1.32) | +0.45 ± 0.91 |
| HP5r | oracle / oracle + reeving in Y | 0 | 0.80 ± 0.27 / 0.84 ± 0.25 | +0.45 / +0.59 |

**Readings.**
- **Reeving in Y.** It removes exactly the 2% line-model part. It is a correct description of the scene's reeving, but it does not improve anchors, because the fit's error is dominated by force realisation.
- **2×→3× step.** It removes the HP5r base-level offset on average, at nearly twice the k scatter, and does nothing for HP3e.
- **Verdict.** Neither correction is worth adopting on its own. The decisive fix is force realisation, which is outside the line model.

### 7.5 Replay files (local, ignored, regenerable)

[`replay_files.txt`](evidence/results/replay/replay_files.txt) lists size and SHA-256 for the two replay dumps (32 MB and 63 MB, `output/replay/*.jsonl`) and their source `events.jsonl` (394 MB and 153 MB, `output/probe/<run>/sweeps.headless/`) in this worktree. They are not on durable storage. The committed `sweeps.json.gz` cannot regenerate them, so the events files are the only copies. Every number above is in the committed summaries.

## 8. Reproduction

```bash
# harness (this branch): HP5r is now supported; --ellipsoid-shape selects the ellipsoid autocal
HP_SIM5_COMPLIANCE_PROBE=2,3,1 .venv/bin/python -m autocal.tools.imperfect_machine OUT --machine hangprinter_5_pulley_reduced --seed 1 --stiffness-scale 0.8
HP_SIM5_COMPLIANCE_PROBE=2,3,1 .venv/bin/python -m autocal.tools.imperfect_machine OUT --machine hangprinter_3 --seed 1 --stiffness-scale 0.8 --ellipsoid-shape
# queue used (one lane; two lanes at most):
evidence/scripts/queue.sh output/probe hp5r hangprinter_5_pulley_reduced
# exact replay (from output/, ~5-7 min each) and attribution
cp evidence/scripts/replay_probe.mjs output/ && (cd output && node replay_probe.mjs probe/RUN/sweeps.headless/scene.usda probe/RUN/sweeps.headless/events.jsonl replay/RUN.jsonl false)
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/probe_replay_attrib.py output/probe/RUN output/replay/RUN.jsonl
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/realised_refit.py output/probe/RUN output/replay/RUN.jsonl
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/refit3.py output/probe/RUN
# refits and tables
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/refit2.py OUT > OUT_refit.json
.venv/bin/python evidence/scripts/table2.py evidence/results/live
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/probe_structure2.py OUT...
.venv/bin/python evidence/scripts/zero_k.py evidence/results/live
# offline slope scan on the nominal datasets (local, /home/torbjorn/repos/hp-sim5/output/fix*)
PYTHONPATH=$PWD .venv/bin/python evidence/scripts/slope_scan.py hangprinter_3 <output>/fixellipsoid-hangprinter_3/sweeps.json ellipsoid 0.8 0.9 1.0 1.1 1.2
```

- **Committed inputs.** Each live run's `imperfection.json`, rescored `result.json`, headless `manifest.json` and gzipped final `sweeps.json` are under [`evidence/runs/`](evidence/runs/). Refits read `sweeps.json`, so gunzip into a run directory first.
- **Revision.** The runs used revision 97a3b8bd plus this branch's uncommitted harness change, which is committed with this report.
- **Code changes.**
  - `imperfect_machine` supports HP5r and moves the frame-side wheel and attach prims, selected as those more than 1 m from the origin. It also gains `--ellipsoid-shape`.
  - `benchmark_history.run_autocal` takes `extra_args`.
  - There is a new unit test for HP5r prim selection.
  - There are no physics changes.
