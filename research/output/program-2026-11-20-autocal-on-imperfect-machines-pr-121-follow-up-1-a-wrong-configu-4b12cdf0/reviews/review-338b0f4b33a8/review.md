# Review: force realisation in autocal data (candidate 1e2413f3)

Reviewer verdict: **accept with caveats**. The ripple cause and the joint ripple + k fit hold in the sim. The hardware-facing unobservability claim **does not hold as stated** and must be rewritten before it guides anything.

- Submission hashes were verified with `--verify` (candidate 1e2413f3, tree 8cddafd3).
- Reviewer scratch (experiment script and outputs):
  `/tmp/claude-1000/-home-torbjorn--t3-worktrees-hp-sim5-t3-ac051df9-d1bd3594cb81f4a4/b644d92b-cea2-45f1-a5fc-7352ac5652dc/scratchpad/exp/`
  - `offset_exp.py`
  - `hp3e-s1.jsonl`, `hp3e-s1-nocog.jsonl`, `hp5r-s1.jsonl`
  - `hp3e-s6.jointk.json`

  The scratchpad is temporary. The numbers below are the record.

## What I reproduced vs inspected

| item | how | result |
| --- | --- | --- |
| Oracle ripple map at true k, hp3e-s1-ea0.8 | reproduced (production fitter via `refit2.fit_at`) | top z +0.302 mm, rms 0.00248 (report: +0.30, 0.0025) |
| Common +10% scale / +0.5 N offset, ripple uncorrected, true k | reproduced | rms 0.04942 / 0.05072 vs 0.04948 base (report's flat scan reproduced) |
| Per-motor offset / scale with ripple corrected | **new** experiment, HP3e and HP5r | contradicts "unobservable" (below) |
| Joint k + 50/rev ripple fit, hp3e-s6 | re-run of `joint_k_cog.py` (JOBS=1) | see "Joint fit re-run" below |
| Cause regression (0.333 N sin 50θ) | inspected against `engine.rs:1610-1637` | consistent. At rest, friction ∝ ω/(\|ω\|+1e-3) → 0 and only cogging survives |
| Code: `--cogging-scale`, loader | inspected + tests run | correct; no default change (below) |
| 2×/4×, unknown, hysteresis, HP5r tables | inspected (result JSONs, `perturb.py`) | not re-run |

## 1. Circularity (Q1)

**The headline 0.16 mm is an oracle-model result.**
- The fitted basis is exactly the sim's generator: sin/cos at h = 50 of the encoder angle. The 50 comes from M666 J200/4, and the cogging law in `engine.rs` uses `numPolePairs`.
- Amplitude and phase are free, so nothing is read from the sim's ripple. But the period and the functional form are known.
- The 2× and 4× offline cases add the *same-phase* sin(50θ) (`perturb.py`, "same phase as the sim's at h = 50"). They test amplitude only.
- The live 2× runs change only the amplitude.

**The "honest" tests are only partly honest.**
- *Unknown pattern (0.66 ± 0.30 mm).* The fitted harmonic set {50, 100, 200} equals the injected set. Only the phases and the 100/200 amplitudes are unknown, so this is not a period-unknown test.
  - In hp3e-s10.unknown, the fit leaves rms at 0.0049 mm (≈4× the 1× floor) and k at +8%.
  - The 50/rev amplitude comes back at 0.30 rather than 0.33 N.
  - So 18 parameters with 2 quadratic-model iterations do not fully converge. Part of the 0.66 mm is the optimiser, not identifiability.
- *Hysteresis + direction offset (0.32 ± 0.22 mm).* The injected offset is keyed on `point_directions()`, the same function the fitter's h = 0 regressor uses. The regressor is therefore the generator. Real hysteresis depends on dwell, speed, load and approach distance (the report says so in §8).
  - n = 3; seed 8 gives 0.64 mm with k +12%.

**A hardware-transferable version.**
- Per motor, run a periodogram of the post-fit residual sensitivity against the encoder angle over integer h ≈ 1…400. Since rms² is quadratic in A and B, this is a linear least-squares scan. Then keep only harmonics above a false-alarm threshold.
  - With hundreds of points per motor and sweep spacing ≫ the ripple period, the phases are effectively random for every h.
  - The report's h = 200-alone result (rms −5%) against the true h = 50 (rms −20×) suggests the right period stands out by a wide margin. So it is very likely identifiable without knowing the period. *This is an inference; not tested.*
- Regularise with a ridge on the amplitudes, plus cross-validation over sweeps (the jackknife already exists).
- Add a direction term only where a line moves both ways.
- The test most likely to change the judgment is the joint fit with the period found by a blind h-scan, on data with a non-50 ripple and realistic stepper detent (2nd/4th electrical harmonics).

## 2. Fit controls (Q2)

- **Same data, same fitter.**
  - Commanded and realised/fitted variants use the same `sweeps.json` and the same production fitters through `refit2.fit_at`. That comparison is fair.
  - The "live" baseline is the live run's own `result.json` at configured k. It is the production fit on the same data, but it does not use the wrapper's extra start point.
- **Attribution.**
  - The fair force-term attribution is jk 0.81 → jk+cog 0.16 mm on HP3e, or cmd@true 1.01 → map@true 0.25.
  - The abstract's "live 2.22 mm" mixes in the ±20% configured-k error, which joint k alone already halves.
  - The abstract should say so.
- **Run selection.**
  - HP3e: seed 9 is a collection failure unrelated to forces (inspected stdout notes). That is acceptable.
  - HP5r: 6 of 12 possible runs are scored.
    - Seed 8 (live 0.91 mm) hit the 2 h limit twice.
    - Four replayed runs were stopped for CPU.
    - The selection is not by result, but n = 6 with a ±0.21 mm sd is thin. The 0.74 mm should carry "6 of 11 available runs".
- **Robustness tables.** These use HP3e live seeds 5–8 and 10. The hysteresis and unknown cases use only 3 of them (no s6), and the fitdir control uses only 1.

## 3. Unobservability of constant offset/scale (Q3) — claim not supported as stated

The report tests a **common** offset/scale (all motors at once), at **fixed true k**, on **ripple-uncorrected** data. In that regime the 0.05 mm ripple rms floor hides everything. I reproduced the flat scan, and then repeated it the way the claim is used: per-motor, with the oracle ripple map applied, and with joint k.

hp3e-s1-ea0.8 (k_true 1.25), oracle map applied. Each cell is rms mm / top z mm:

| perturbation | at true k | joint k (grid ±15%, parabola) |
| --- | --- | --- |
| none | 0.00248 / +0.30 | 0.00238 / +0.21 (k 1.257) |
| motor A +0.5 N | 0.0395 / −0.13 (max 0.54) | 0.0357 / +2.71 (k 1.048) |
| motor B +0.5 N | 0.0404 / −1.43 | 0.0341 / +2.47 (k 1.011) |
| motor C +0.5 N | 0.0337 / −1.25 | 0.0307 / +1.47 (k 1.088) |
| motor A +10% | 0.0189 / +0.05 | 0.0182 / +0.90 (k 1.190) |
| motor B +10% | 0.0181 / −0.35 | 0.0167 / +0.89 (k 1.166) |
| motor C +10% | 0.0143 / −0.34 | 0.0138 / +0.39 (k 1.201) |
| all +0.5 N | 0.0155 / −3.06 | 0.0072 / −0.36 (k 1.100) |
| all +10% | 0.0085 / −1.21 | **0.00238 / +0.30 (k 1.152 = 1.25/1.085)** |

hp5r-s1-ea1.2, oracle map applied, true k. Base rms is 0.00292 mm and top z +0.54 mm.
- Per-motor +0.5 N: rms 0.0051–0.0065 (1.8–2.2×), top z −0.00…+0.91.
- Per-motor +10%: rms 0.0071–0.0119 (2.4–4.1×), top z +0.17…+1.51.
- All +0.5 N: rms 0.0052.
- All +10%: rms 0.0025 (flat), top z +1.22.

Without the map (HP3e, true k), per-motor +0.5 N still raises the rms 20–30% (0.0495 → 0.059–0.065). A common +10% stays flat (0.0494), which is the report's result.

**Readings.**
- **Per-motor offsets and scales are observable once the ripple is modelled.** The rms rises 6–16× on HP3e and 2–4× on HP5r for 0.5 N / 10%. "Not identifiable … a joint per-motor offset/scale parameter cannot work" is not supported. The flat scans were masked by the ripple floor and pooled across motors.
- **A common scale is degenerate with k, not with top z.** Joint k absorbs it exactly on HP3e (rms back to base, top z unchanged). It is harmless for geometry whenever k is fitted. That is the opposite of "7–13 mm per 10%".
  - That figure is a fixed-k number, and k is not fixed in the proposed method.
  - On HP5r the common scale is rms-flat at fixed k. Joint k with a common scale was not run there.
- **A common offset is mostly absorbed by k.** It leaves 3× rms and −0.36 mm.
- **The real hardware risk is different from the report's.**
  - Per-motor offsets/scales are visible but unmodelled in the current joint fit.
  - Joint k then mis-absorbs them: k −13 to −19% and top z +1.5 to +2.7 mm per 0.5 N on one motor.
  - The "rms didn't drop to the floor" signal would show this.
  - The mitigation to test is adding per-motor offset/scale terms, with one gauge fixed or with k. The report rules this out without evidence.
- **External force calibration** (§7) remains a sound recommendation. Its stated justification ("sweep data cannot supply it") is wrong.

## 4. HP5r residual and home-stretch emulation (Q4)

- **The +0.4–0.9 mm top-z bias is real, and it is not a force or compliance effect.**
  - The oracle map at true k already leaves top z +0.45 ± 0.21 mm (inspected `summary_cog.txt`). I reproduced +0.54 mm on hp5r-s1.
  - The post-map rms is ≈3× HP3e's.
  - The explanation (effector rotation, unmatched points) is a hypothesis, untested. The yfix result shows that part of the k bias is reeving.
  - Plausibly explained as "something else", not explained.
- **The home-stretch emulation is probably a double count, and in any case is not the commissioned default.**
  - Production `_hp4_home_strain` is gated to `hangprinter_4` with equal-strain data (`pose_fit.py:200-220`).
  - The emulation applies it to HP5r, whose sweep fit already carries a stretch model through FlexModel.
  - The 2.5–3.5 mm regression should not be cited as a property of the default. The report flags this correctly.

## 5. Code changes (Q5)

- **Loader.** `machineEntityBuilders.js:254-258` copies `stepper:coggingTorque` and `coggingFreq` only when they are authored.
  - The Rust engine treats the default (NaN) as 0.01·holding at `numPolePairs` (`engine.rs:1629-1635`). The JS adapter passes `?? NaN` (`rust_physics_engine.js:118`).
  - `git grep` finds no scene, `.usd*` or catalogue file that sets either attribute.
  - Default sim behaviour, the CI benchmark and the physics tests are therefore unchanged.
- **Harness.** `scaled_cogging` inserts `coggingTorque = 0.01·holding·scale` after every `custom double stepper:holdingTorque` line.
  - It raises if there are none. It requires the `custom` qualifier, which current scenes use.
  - `cogging_scale` is recorded in `imperfection.json`.
- **Tests run on the candidate.**
  - `jest machineScenePipeline` 14/14.
  - `pytest autocal/tests/test_imperfect_machine.py` 6/6.
  - HP4 physics `node --test` (logo stability, stability, pretension) 11/11.
  - No Rust changed, so there was no rebuild.
- **Code readiness.** Fine to merge as a harness flag. There is no production fit change. The joint fit is an FD wrapper, not ready code (the report says so).
- **Dead code.** `hp-sim-3d/app/torqueModeSystem.js` is a JS duplicate of the torque law, used only by its own test. Per CLAUDE.md, delete it in a follow-up.

## Joint fit re-run (hp3e-s6)

**Not reproduced.**
- The single-process run (JOBS=1, to respect the 2-simulation limit) hit the background time limit before it printed anything. I did not restart it.
- The submitted result (`files/29-hp3e-s6.jointk.json`) was inspected only:
  - jk+cog: k/k_true 1.0034, top z +0.145 mm, rms 0.00121;
  - fitted amplitudes 0.334 / 0.332 / 0.334 N;
  - plain jk: 0.83 mm.
- These match the table and are consistent with my oracle-map reproduction on hp3e-s1 (true k: +0.30 mm; joint k with the map: +0.21 mm, k 1.257).
- So the joint *k* step is reproduced with the oracle map. The joint *ripple-parameter* step is not reproduced.

## Assessment

- **Scientific value: high.**
  - It finds the cause of the sim's force-realisation error.
  - It shows that the ripple is variance in top z, not bias.
  - It shows that a periodic force term fitted jointly with k recovers ≈ the oracle accuracy on HP3e.
  - The negative results are worth preserving:
    - plain joint k diverges under larger ripple;
    - unmodelled hysteresis gives k −27%;
    - two-sided points kept separate are worse;
    - HP5r collection fails at 2× cogging (auto-tune reads ripple as stiction);
    - the wrong single harmonic barely moves the rms.
- **Caveats required before this enters the corpus:**
  1. State that the 0.16 mm and the 2×/4× results use the generator's exact period and form (oracle basis). The "unknown" case fits the injected harmonic set, and the hysteresis regressor is the generator.
  2. Replace the unobservability claim with what the data show:
     - a common scale is degenerate with k and harmless under joint k;
     - per-motor offsets/scales are visible in the rms once the ripple is modelled, and are currently mis-absorbed into k, giving 1.5–2.7 mm per 0.5 N on one HP3e motor.
     - The 3–13 mm figures are fixed-k, common-perturbation numbers.
  3. Attribute the improvement against jk / cmd@true, not against live at configured k.
  4. HP5r 0.74 mm is 6 of 11 available runs, and its residual bias is unexplained.
- **Test most likely to change the judgment:** a blind period scan plus per-motor offset/scale terms, on data with a non-50/rev ripple and an injected per-motor offset (one motor at 0.5 N), across 3+ HP3e seeds.
  - If the period is found and the offsets are recovered with top z < 0.3 mm, the method transfers and external calibration becomes a cross-check.
  - If not, the hardware recommendation stands.
