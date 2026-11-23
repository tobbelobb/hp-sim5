# Autocal on imperfect machines (PR #121 follow-up). (1) A wrong configured line stiffness trades off almost perfectly against top anchor D.z with the current collection routine and force tuning; find force/compliance-tuning data that reveals the real compliance so D.z is recovered even when config.g stiffness is wrong. (2) The default fit assumes all low anchors share one height; real machines differ slightly and the current residual does not catch one wrong low-anchor z. First benchmark on randomized imperfect machines (uneven anchor heights, off-centre top anchor, ±20% compliance error), several seeds, report spread. Only then compare collection patterns. Radius variation and line buildup are out of scope. Small experiments first, then HP3, then HP4, HP5, HP5r. Use Rerun recordings sparingly.

## Current understanding

Prior work (PR #121, merged):
- The held anchor's z is exactly degenerate with the scale of the free lines' stretch correction: about 8.5 mm per unit scale, with the rms flat. That correction is 10–20% off per machine.
- So the compliance and tension model, not the collection plan, limits accuracy.
- The point fit assumes the low anchors share one z.
- 2026-11-20 (benchmark harness b77ec4aa, branch t3/imperfect-machine-benchmark): on 3-low-anchor machines (HP3/HP4), uneven low z is an exact gauge for length-only data.
  - The fit returns the true machine rigidly rotated so that the low plane is level. Pairwise distances are within 0.02 mm.
  - Raw errors reach 28 mm; with the rigid motion removed they are 0.66 mm (HP3 seed 0).
  - So the residual cannot see it. Only gravity or force data, or a 4th low anchor, can. The open question is whether tilt matters for printing.

- 2026-11-21, HP3 baseline (harness 9fcec8c3, 15 of 18 live runs). Perturbations: ±30 mm low dz, ≤50 mm top offset, stiffness k 0.8–1.2.
  - Convention-frame error is 0.71–1.68 mm, against 1.18 mm nominal.
  - It is dominated by a common z shift of all anchors (−0.5 to −1.2 mm), i.e. the home reference. Rigid-aligned error is 0.17–0.66 mm.
  - No perturbation stands out on its own. Stiffness ±20% alone gives 1.09 / 0.88 mm.
  - So on HP3 default sweeps, the stiffness→D.z trade-off is weak. Investigation 2 is asked where it is actually large.

- 2026-11-21, investigation 2 done (branch t3/top-anchor-compliance-observability, 6c9cd463). Under review: review-78e5d2a30d57.
  - Wrong stiffness moves D.z strongly only on HP5r sweeps (8.4 mm per unit k) and in the ellipsoid shape (5–10 mm per unit).
  - HP3's common z shift is home-reference stretch: low lines slack at the encoder zero. Removing the true stretch takes HP3 from 0.97 to 0.14 mm.
  - The opt-in probe steps the force to 2×/3× at fixed poses. It estimates k to −1.3 ± 3.0% (9 HP3 runs) for about 3% extra time.
  - With probe k in the home-stretch term (offline refit), D.z is within 0.04 mm of the true-stiffness fit, and the worst coordinate improves from 1.07 to 0.44 mm on average.
  - Remaining floor: ±0.36 N cogging gives −0.35 to +0.65 mm D.z.

- 2026-11-21, investigation 3 done (branch t3/uneven-low-anchor-heights, a7a8d179). Under review: review-0e816f6d262c.
  - Every low z is now free, with the gauge at top x = y = 0, A.x = 0 and home at the origin. The equal-z and fixed-z paths are deleted.
  - Old-fit error from a low anchor h mm off: 0.68·h on HP3/HP4, a pure tilt that the rms can't see; 2–2.4·h on HP5/HP5r, where equal z was a real constraint.
  - New fit is flat in h on all machines.
  - Live HP3 uneven cases: 0.89–1.42 mm, against 7.5–13.4 mm before. HP5 base: 0.69 mm, against 1.04 before.
  - Open: HP3 equal-height base 1.35 vs 1.17 mm (one run). HP5 jackknife rose 1.62 → 3.28 mm.
  - The D.z/stiffness coupling is unchanged; that is investigation 2's job.

- 2026-11-21, review-78e5d2a30d57: accepted with caveats.
  - About 95% of the HP3 gain comes from adding the home-strain term with the configured S (max error 1.07 → 0.38 mm).
  - The probe's k only removes the leftover common z shift, about 0.9 mm × (k − 1). It is not needed on HP3.
  - The probe's k is unbiased (cogging bias ≤1%; 95% interval about ±7%).
  - The probe still has to be tested where the trade is strong: HP5r sweeps and the ellipsoid shape.

- 2026-11-21, review-0e816f6d262c: free low z accepted with caveats.
  - The gauge is complete on all machines, and the deletion is clean.
  - The HP3 base change (1.17 → 1.34 mm) is caused by the fit redistributing the stretch bias, not by noise or different data. Across noise seeds it is 1.01 vs 0.96 mm, so there is no systematic loss.
  - The HP5 jackknife does not affect stopping or fit selection, but it labels a 0.69 mm result "poor".
  - Follow-up sent to investigation 3: 3–5 live HP5/HP5r base runs, one HP4 run with an uneven anchor, and report fixes.

- 2026-11-21, investigation 1 done (t3/imperfect-machine-benchmark a0b27d7d; newfit comparison on t3/imperfect-machine-newfit d35ac09c).
  - HP3, old fit, 10 random machines: print-frame max error 1.11 ± 0.35 mm, with D.z the worst at −1.01 ± 0.41.
  - Common z shift −0.87 ± 0.15 mm; after rigid alignment 0.40 ± 0.21.
  - Only stiffness has a measurable effect on HP3: 0.15–0.23 mm common shift over ±20%.
  - Scores don't flag anything on HP3, and there is nothing bad to flag.
  - HP5, old fit, with uneven heights: 19.3 and 4.3 mm, run to max steps, flagged by rms and jackknife.
  - New fit, paired on 15 HP3 machines: −0.04 ± 0.03 mm (no regression in the print frame).
  - New fit on HP5: 19.26 → 0.52 and 4.30 → 1.00 mm, at nominal time.
  - Force auto-tune knife-edge fixed (1ec95e5e).
  - Follow-up running: harness for HP4/HP5r, plus a baseline on every machine with the new fit.

- 2026-11-21, harness now covers HP4/HP5r (t3/imperfect-machine-benchmark 175b19e1; newfit 6580dd6e).
  - Unperturbed runs reproduce CI: HP4 0.73 mm, HP5r 0.93 mm.
  - Free-low-z fit, random machines, print-frame max error:

    | Machine | Runs | Error |
    | --- | --- | --- |
    | HP3 | 10 | 1.06 ± 0.31 mm |
    | HP4 | 3 | 0.53 ± 0.04 mm |
    | HP5 | 4 | 0.95 ± 0.34 mm |
    | HP5r | 3 | 1.23 ± 0.04 mm (top z +1.2 mm in 2 of 3) |

  - 20/20 completed.
  - New-fit nominal (c0554092): HP4 0.72 mm (unchanged); HP5r 0.92 → 1.10 mm.
  - So freeing low z costs HP5r 0.18 mm, and its top z is +0.53 nominal and +1.2 mm on random seeds. HP5r's top z is the target for the home-strain term.

- 2026-11-21, investigation 4 (t3/test-compliance-probe-stiffness 3a5b166b, on top of free low z). 12 live runs, true EA ×0.8/×1.2.
  - Configured-k error: HP5r 2.60 ± 1.39 mm, HP3 ellipsoid 2.50 ± 1.84 mm, almost all in top z.
  - With probe k: 1.01 ± 0.32 and 1.40 ± 0.77 mm. The probe wins in 11 of 12 runs, at about 3% extra time.
  - The probe's k is biased: +4% on HP5r, −4% on the ellipsoid, with an unexplained sign contradiction. A replay is running to find the cause.
  - A joint k fit from the rms alone works on average on HP5r but missed by 13% once.
  - HP5r seed 3 fails the rough bootstrap deterministically: a separate robustness bug.

- 2026-11-21, investigation 3 follow-up (e292f73e).
  - HP3 explanation corrected.
  - HP5 "poor" labels came from one dominant leave-one-sweep-out fold. A robust median-based spread fixes this: false "poor" on ≤1.5 mm fits drops from 6 to 2 of 26, with no >5 mm fit missed. Under review: review-8457f4ee9842.
  - HP4 with A raised 10 mm: 1.04 mm live.
  - Top line 50 mm off plumb: ≤0.09 mm.
  - Delivery note: branches commit hundreds of raw run files (a7a8d179 has 958, e292f73e 671). Strip them before any PR.

- 2026-11-21, probe replay (7bfd016c): the probe's ±4% k bias is force realisation (produced vs commanded tension), not compliance.
  - HP3 ellipsoid: force steps come out about 2.5% short.
  - HP5r: the sweep's last point is produced 0.53 N below its command.
  - The HP5r M666 Y leaves out 85.5 mm of effector reeving, a 2% effect that barely matters.
  - With the produced forces, the top-z error at true k drops by 0.6–1.0 mm. Force realisation in the sweep data is the bigger lever for top z.
  - Open: is the sim's force-realisation error representative of hardware? Mitigations must not exploit sim-specific cogging.

- 2026-11-21, review-8457f4ee9842: the robust (median) jackknife is accepted with caveats, but in effect rejected.
  - It hides single-bad-sweep failures: 2 of 22 cases above 5 mm are labelled "usable".
  - It worsens one corpus patience run from 2.94 to 4.62 mm.
  - Rework sent to investigation 3: keep the classical spread, and handle folds that remove a coordinate's only constraint structurally.

- 2026-11-21, investigation 5 done (t3/rough-bootstrap-robustness d09716bc; fix 9b2941ee).
  - HP5r seed 3 failed because a force-tune ramp creep sample with a −0.4° gain fired the plateau rule. Force max came out at 1.44 N, so the rough sweeps never left home.
  - Fix: a flat or reversing sample counts as one of the two declines the rule already requires.
  - Failure rate on random machines: 1/23 before, 0/23 after. 16/16 comparable runs are bit-identical.
  - Residual: two flat samples in a row.

- 2026-11-21, structure-aware jackknife (059a5e27) replaces the median.
  - Classical spread, but a fold that alone removes a coordinate's constraint is left out of that coordinate's spread.
  - False "poor" labels: 4/9 → 1/9. Injected failures caught: 0 lost. Corpus decisions changed: 0.
- 2026-11-21, integration: research/autocal-imperfect-machines dff509b8 merges the harness, probe, free low z, the jackknife change and both force-tune fixes.
  - pytest autocal/tests: 538 passed. jest autocal/control: 183 passed.

- 2026-11-21, investigation 6 (t3/autocal-force-realisation 1e2413f3). Under review: review-338b0f4b33a8.
  - The sim's force error is pure cogging: 0.333 N·sin(50θ).
  - A joint offline fit of ripple, direction offset, anchors and k gives HP3 ellipsoid 0.16 mm and HP5r 0.74 mm, and holds at 2–4× ripple.
  - A constant per-motor force offset or scale is unobservable from sweep data but moves top z 3–13 mm, so hardware needs an external force calibration.
  - HP5r collection fails at 2× cogging, because force auto-tune reads the ripple as stiction. Sent to investigation 5.
  - Their HP4-style home-stretch emulation made HP5r worse (possible double count). Investigation 7 warned.

- 2026-11-21, investigation 7 (t3/default-home-stretch-fit 2b086a3c, merged into the integration branch).
  - The home-strain term is now gated on "d+1 lines with equal-strain sweeps", so it covers HP3 and HP4.
  - HP3, 10 paired seeds: 1.06 ± 0.31 → 0.44 ± 0.10 mm, better on 10/10. Nominal: 1.19 → 0.60 mm.
  - HP4, HP5 and HP5r are byte-identical.
  - On HP5/HP5r, forcing the term double-counts (1.10 → 2.43 mm), because their per-point stretch model already covers the effect. HP5r's +1.2 mm top z is not a missing home strain.
  - A ±20% stiffness error costs about ±0.1 mm of common z.
  - Tooling gotcha: RRF fails on config paths of about 150 characters or more, so worktrees need short paths.

- 2026-11-21, review-338b0f4b33a8: force-realisation fit accepted with caveats.
  - The 0.16 mm uses the sim's exact 50/turn law, so it is circular. The fair gain over joint k alone is 0.81 → 0.16 mm.
  - Per-motor force offsets and scales ARE visible once the ripple is modelled, and a common scale is absorbed by k.
  - The real risk is an unmodelled per-motor offset leaking into k and top z (+1.5 to +2.7 mm).
  - The scene-loader change leaves defaults unchanged.
  - Decisive test sent: a blind period scan with per-motor offset and scale on non-50/turn data.

## Human decisions (continued)

- 2026-11-21: the home-stretch correction with configured stiffness may become the default (HP3, then HP4, HP5, HP5r). Stored benchmark numbers may change.
- 2026-11-21: real motors may have even larger force errors than the sim's ±0.2–0.5 N. Force-realisation mitigations must hold up to larger errors.

## Status at wrap-up (2026-11-21)

Simulation only. The integration branch merged every investigation. The deliverable is a cleaned single commit off main, with raw run trees and review diffs dropped; see `RAW_DATA.md` in this directory.

**In the default code now**
- **Free low-anchor z with the top-anchor gauge** (top x = y = 0). It fixes uneven heights on every machine: HP5 went from 4–19 mm to 0.5–1.0 mm.
- **Structure-aware jackknife.** It removes the false "poor" labels on HP5.
- **Force-tune knife-edge fixes:** 1ec95e5e and 9b2941ee.
- **Home-strain term on HP3** (HP4 already had it). HP3 on 10 random machines: 1.06 → 0.44 mm.
- **Imperfect-machine harness** for all four machines, with `--cogging-scale` and `--cogging-freq`.
- **Equal-strain tension cap** (cb086259). It is UNVERIFIED live at 2×/3× cogging; the 1× force profiles are byte-identical.

**Opt-in or offline only**
- **Compliance probe** (`HP_SIM5_COMPLIANCE_PROBE`). Its k is biased by force realisation. It is needed only on HP5r sweeps and the ellipsoid shape.
- **Joint force-realisation fit** (offline wrapper). Blind period scan plus per-motor offset/scale, fitted with k: HP3 ellipsoid 0.17 mm on 7 held-out cases. Not in the production fit, and not tested on HP5r.

**Per-machine state (print-frame max error, random imperfect machines)**

| Machine | Error |
| --- | --- |
| HP3 | 0.44 ± 0.10 mm |
| HP4 | 0.53 ± 0.04 mm |
| HP5 | 0.95 ± 0.34 mm |
| HP5r | 1.23 ± 0.04 mm (top z +1.2 mm; not home strain, force or stiffness) |

## PR #122 follow-up

- The tension cap cleared the profiled bootstrap failure on both paired HP5r seeds at 3× cogging. It lowers planned load and shortens the supported arc. The experiments measure their combined effect.
- HP5/HP5r now have a fallback stop for noisy but stable fits, with a range RMS ceiling, geometry checks, and a sweep jackknife. On the archived 2× HP5r seed 6 run it would have stopped at 8 sweeps instead of 23 at the same 1.54 mm error.
- Profiled slip recovery reduces positioning and pullout forces, preserves the measurement profile, and replans after bounded recovery fails during refinement. [High-cogging report](validation/high-cogging/report.md).
- The failed CI integration test exited in collector startup. The runtime now allocates distinct ports in one batch and CI retains the collector and firmware logs on failure. The original failure did not reproduce locally.

## Force and geometry results

- The HP3 home-strain fit now offers `--fit-force-ripple`. The guarded fit improved all eleven archived HP3 simulations and detected 137 periods per turn in a fresh 2× run. HP5r offset and scale contrasts produced regressions and negative sensor tensions in mixed-error controls. [Force-fit evidence](session-2026-11-21-production-force-realisation-fitting-reuse-blind-period-scan-and-b5e5ea67/report.md).
- HP5r's positive residual top height shift comes mainly from the effector rotating the low-line attachments. Holding attitude at home in the matched forward controls reduces +0.35–0.59 mm to +0.04–0.07 mm. The observed ranges leave general effector attitude underdetermined, so this establishes a cause in simulation. [Rotation and collection comparison](session-2026-11-21-isolate-the-remaining-hp5r-top-anchor-height-bias-with-controlle-76b38f06/report.md).
- The archived collection comparison gives different orderings on different machines. The nominal HP5r sweep is both faster and more accurate than the nominal ellipsoid. The paired imperfect HP5r runs test sensitivity to wrong configured EA. [Paired refits](session-2026-11-21-isolate-the-remaining-hp5r-top-anchor-height-bias-with-controlle-76b38f06/report.md).
- Hardware validation needs an independent force scale. The earlier program identified a known mass in both directions as a cross-check.

## Human decisions

- **2026-11-21: frame convention.** Rigid tilt does not matter for printing. The frame is "origin directly below the top/D anchor": Dx = Dy = 0, A.x = 0, and the origin at home. It applies on all machines.
  - The equal-low-z constraint is dropped, and every low z is fitted jointly.
  - Score errors in this frame. Investigation 3 is redirected to implement it; investigation 1 adds the convention-frame error.
