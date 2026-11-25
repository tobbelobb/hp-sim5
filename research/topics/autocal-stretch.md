# Autocal stretch and identifiability

The [original HP3 investigation](../output/autocal_research_with_Fable5p2/report.md) identifies smooth elastic bias hidden by tiny residuals: substituting geometric lengths removes the simulated error. Height diversity and more cable-solver iterations alone did not remove it. Direct-collector experiments with weighted per-strand force targets and a home-length offset gave promising shape accuracy, but those used different data and an independent fitter, not the whole firmware/autocal pipeline.

The [production hardening program](../output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/report.md) now implements default home offsets, position-information sweep ranking, force profiles, checked rough collection and separate remeasurement for HP3/HP4. It fixes profile identity, candidate-offset prediction and restart/model-purity defects. Rough gates assess local observability and convergence, not correlated mechanical bias. They can accept rough estimates about 25 mm from truth, which may be adequate for force planning but is not a precision calibration certificate.

The [HP4 D.z investigation](../output/session-2026-11-18-hp4-autocal-dz-bias/report.md) shows that HP4's persistent ~4.6 mm D.z error is not stretch. The three D lines share one spool, and their chiral reeving makes the effector roll about 0.2°/m off-centre. One of each ABC parallel pair then goes slack in about half the poses, and the encoder follows only the taut line. Feeding the fit that taut-line geometry with zero stretch reproduces about −7 mm D.z, and no stiffness scaling removes it. Keeping the per-strand equal-strain target independent of mechanical advantage (`/ min(N)` instead of `/ min(N·U)`) keeps the pairs taut. In single runs it reduces HP4 from 4.57 to 1.52 mm and HP5 pulley-reduced from 2.57 to 1.34 mm. HP4's remainder is mostly the common home-reference z shift that HP3 also shows.

Production-path accuracy remains uneven. The [expanded-workspace collector](../output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/validation/workspace-expansion/report.md) reaches actual sampled heights near 1.73 m and returns 0.91 mm worst coordinate error / 0.63 mm translation-aligned on nominal HP3; its largest 3D anchor-distance error is 1.07 mm. Half-stiffness HP3 gives 1.63 / 0.895 mm, improved from the preceding 4.60 / 2.16 mm trial. Nominal HP4 remains at 4.65 / 2.99 mm; worn HP4 gives 2.33 / 0.818 mm after safely rejecting and replanning three high-pose configurations. These are individual simulations with changed collection costs and protocols, not isolated interventions or robustness bounds. The earlier [variant and noise evaluation](../output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/session-2026-11-15-evaluate-the-hardened-candidate-on-full-hp3-hp4-compliance-and-b-3c7b0091/report.md) preserves the preceding failures and counterexamples.

On one exact multi-height HP3 recording, fitting offsets reduced ellipse error from 14.79 to 1.70 mm and independent joint-range error from 12.24 to 1.58 mm. On old single-height data, fitting the offset instead made error much worse. Filtering also has mixed effects and can lower cost while worsening geometry. Independent least squares retains substantial errors on problematic recordings; optimizer replacement alone has not solved the physical/model bias. Seeded corruption of recorded lengths is now tested, but full simulated encoder noise, uncertainty distributions and hardware remain open.

The [reference-to-production crossover](../output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/next_steps.md) reproduces the early best recording: original fitter 0.801/0.120 mm absolute/aligned, current point fitter 0.814/0.179 mm. On the earlier narrow full recording, the original fitter instead gives 1.475/1.312 mm. That older recording covers approximately 40–423 mm reconstructed height under a ~390 mm planner budget, while the direct reference spans 0–1200 mm. Removing upper poses worsens aligned fits; count-matched wider subsets fare better, though XY/force composition also changes. Progressive exploration now removes that permanent budget using successful probes, preserves initial half-spindle-turn sensor reserve, and tests local coupling before reclaiming later reserve. Failed motion aborts; resolved unmeasurability records a boundary and replans without faking convergence.

Geometric-length substitution on the new wide nominal recordings removes local point-fit errors. The simulator's known 50-pole motor cogging sinusoid explains 98–99% of held-out HP3 sensor-force-error variance and over 99% on HP4, with amplitudes near the predicted 0.333/0.667 N. This identifies a force discrepancy, not its causal fraction of calibration error. Correct reeving-aware force profiles remove a separate software error, but commanded forces still differ from actual tensions. A matched phase-averaging/dither and same-axis opposite-motion experiment is the next protocol test; actual physics forces and scene-known parameters must remain diagnostics, not production inputs.

A common terminal-validation pilot scored frozen candidates on one new, unfitted sweep. It worsened nominal HP3, left soft HP3 unchanged and modestly improved HP4; prediction cost and anchor accuracy still disagreed. The live option was withdrawn after independent review, preserving offline replay and negative evidence in the production program.

The [home-reference investigation](../output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/session-2026-11-15-determine-and-implement-physically-justified-final-home-shift-co-fc8f5233/report.md) finds that fitted offsets and nominal stiffness alone do not identify absolute home. RRF and Klipper already have an absolute flex reference term. A guarded correction helper requires independently established compatible extension evidence; ordinary collection supplies none. Its algebra is tested, but correlated wrong evidence can pass consistency checks. A reliable physical home-reference protocol remains needed, and translation correction alone cannot remove the observed shape errors.

The [broader-layout pilot](../output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/session-2026-11-15-review-production-integration-of-pose-fit-information-planning-a-84abd4be/report.md) shows why HP3 assumptions cannot simply be enabled for four-high or cube corners. Four-high needs gravity-supported nonnegative equilibrium and branch/gauge handling; cube corners needs all fixed constraints, active/slack sets and multidimensional tension allocation. Their new-method rollout remains a staged synthetic and simulator plan. No hardware behavior is established.

The [HP5 redundant-sensor program](../output/program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f/report.md)
now implements measurement-only spare-line pretension, per-point reversible
coupling, two-load static stretch and extra-range home offsets. Two simulated
continuations apply approximately 1 mm worst-coordinate anchors. Same-recording
full-data refits improve direct HP5 (aligned 3.11 → 0.70 mm) but worsen reduced
HP5 (1.27 → 2.88 mm). Coupling does not remove smooth mechanical bias; the good
selected reduced estimate must not hide its worse final refit. Earlier reviewed
short reduced recordings favor five ranges, while direct narrow physical sweeps
did not establish a gain. Neither universal accuracy nor full-loop speed benefit
is established. Rough bootstrap and historical datasets keep the old policy.

The HP5 [prefix audit](../output/program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f/production-validation/prefix-audit/report.md)
clarifies time-to-quality: direct reaches ≤2 mm worst-coordinate error after four
profiled sweeps using extra ranges versus none through twelve when they are
withheld on the same recording; reduced reaches it after four either way. Later
reduced fits worsen more often. The production fitter uses the extra data, but
candidate forecasts omit the fifth range, history validation remains ellipse
based, and rough bootstrap does not collect it. Speed benefits are therefore
not fully exploited or measured as a controlled end-to-end wall-time result.

The [integrated HP5 implementation](../output/program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f/production-validation/integrated-pipeline/report.md)
(`68260871`) closes those pipeline gaps: sensing starts in the first sweep,
force-aware bootstrap is reused, forecasts include the fifth range, covariance
uses the force-aware residual, and held-out validation keeps each model's own
offsets. HP5 applies the best validated model instead of mixing geometries and
offsets through consensus. Two good agreeing fits can stop collection. Fresh
stock simulations used six total sweeps and three refinement fits; final replay
applied 1.16/1.92 mm worst-coordinate anchors. This is limited simulation
validation, with model bias and hardware behaviour still open.

The [ellipsoid autocal investigation](../output/session-2026-11-20-ellipsoid-autocal/report.md) (2026-11-20, simulation) locates the remaining ~1–2 mm error of sphere-mesh collection in the held anchor's height. That height is exactly degenerate with the scale of the *free* lines' stretch correction.
- Rescaling that correction moves the held anchor's z about 8.5 mm per unit scale while the fit rms stays constant to 0.0002 mm.
- Zero error needs the free lines ×0.9 on HP4 and about ×1.19 on HP5. On the sweep benchmark, HP5 pulley-reduced also improves, from 1.30 to 0.72 mm, with free lines ×1.1.
- Scaling the held line itself barely matters.

The residual, the jackknife and the covariance cannot detect this error. The tension estimate (static equilibrium from commanded forces), the flex-model compliance (`autocal/flex.py`: `S`, `U`, `N`, `Y`) or slack parallel pairs must be made right physically. More or wider data will not fix it. The scene stiffness (`stiffnessPerUnitLength = 20000`) matches M666 `S`. The guy lengths, the MA power, and per-strand tension under roll are unchecked.

The [compliance-observability investigation](../output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-20-compliance-observability-for-the-top-anchor-height-with-todays-c-4f2510fd/report.md) (2026-11-20/21, simulation) settles where configured stiffness matters.
- **Where the trade is large.** Offline slope scans on nominal datasets put the stiffness→D.z trade at 8.4 mm per unit compliance scale on HP5 pulley-reduced sweeps and 5–10 mm in the ellipsoid shape. HP3 sweeps use no compliance; HP4's home-strain term shifts all z together (1.5 mm/k); HP5 sweeps leave D.z alone.
- **HP3's common z shift is stretch.** HP3's ~1 mm common z shift is ABC stretch relative to the slack encoder origin, 0.93 mm per unit k at 1.11 N/strand, and is invisible to the rms. An exact replay shows that removing the true stretch takes HP3 from 0.97 to 0.14 mm.
- **A force step measures compliance.** A fixed-pose force step (hold the other lines, step the sensor force 2×/3×) measures the compliance scale to ±4% in ~150 s of simulated time. With it, the HP4-style home-strain term puts HP3 D.z within 0.04 mm of the true-compliance fit at EA ×0.8–1.2.
- **Remaining error and open work.** What remains is the commanded-vs-realised tension error (cogging), not compliance. Live tests on HP4 remain open, as does hardware.

The [home-stretch default](../output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-21-make-the-home-stretch-correction-the-default-autocal-fit-the-com-01a484d6/report.md) (2026-11-21, simulation, live paired) makes the HP4 home-strain term default on HP3. This supersedes "HP3 sweeps use no compliance" above.
- **HP3.** Print-frame max error falls from 1.06 ± 0.31 to 0.44 ± 0.10 mm on 10 seeds, better in all 10. Configured S costs about 0.54 mm per unit k.
- **HP5/HP5r.** The per-point flex model already contains the slack-at-encoder-zero assumption. Adding the term double-counts it, doubling the error at identical rms.

The [compliance probe live test](../output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-20-compliance-probe-where-stiffness-really-trades-against-the-top-a-62081729/report.md) (2026-11-21, simulation, 12 runs on the free-low-z fit) covers the two strong cases.
- **Measured k is worth it.** At EA ×0.8/×1.2, configured S leaves mean max error 2.6 mm on HP5r sweeps and 2.5 mm on the HP3 ellipsoid (worst 5.4 / 4.8). The probe's k gives 1.0 / 1.4 mm (worst 1.6 / 2.7) for about 3% of collection time.
- **The probe is biased.** The bias is +4.1 ± 2.0% on HP5r and −4.0 ± 1.9% on HP3e, against the scene EA. It shows with the true anchors too. An exact replay attributes it to force realisation, realised ≠ commanded tension at the probe levels. HP3e's steps are realised 2.5% short. HP5r's base level, the sweep's last point, is realised 0.53 N low.
  - **The line model.** At the scene EA the HP3 sensor line's stretch per realised tension matches M666 to 0.1%. On HP5r it is +2%, because `U·d + Y` omits 85.5 mm of effector-side reeving per path.
  - **Ruled out.** There is no pinhole or wheel friction in the sim. Uneven parallel-path sharing from roll is 10–65%, but it does not bias k.
  - **The k that zeroes top z (0.93× / 1.07×) is not compliance either.** With realised forces in the fit, the true-k top-z error falls from 1.33 to 0.33 mm (HP3e) and from 1.12 to 0.51 mm (HP5r). The opposite signs come from the opposite D.z-vs-k slopes.
  - **What this supersedes.** It supersedes the 2026-11-18 "effective EA 17.8–19.8 kN" estimate for these machines.
- **The rms is not flat with live mismatch.** A joint rms fit of k matches the probe in mean on HP5r but misses by 13% in one run in six. On HP3e it scores best only because it also cancels the ellipsoid's own top-z bias.

The [uneven low-anchor heights investigation](../output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-20-uneven-low-anchor-heights-the-default-point-fit-assumes-all-low-8b2f2abb/report.md) changes the anchor gauge on every Hangprinter. Length changes from home leave exactly one symmetry, a rotation about home.
- **Old convention:** the fit fixed that rotation with A.x = 0 and one shared low z. On HP3/HP4 a wrong low height then became an invisible tilt (error 0.68 × h). On HP5 it was a real wrong constraint (2–2.4 × h).
- **New convention:** top x = y = 0 and A.x = 0, with translation pinned by home and the first d lines' zero offsets. Every low z is fitted, and simulated error stays flat in h. The common HP3 low-z shift of −0.7 to −1.0 mm is the existing stretch/D.z bias, not the gauge.
- **Unchanged:** the D.z–stretch-scale trade-off (corr 0.99 on HP3/HP4) does not change with the gauge.
- **Jackknife:** with free low z, one HP5 fold that loses a coordinate's only constraint inflated the classical jackknife 2–9×. The jackknife stays classical but leaves such a fold out of that coordinate alone, and reports the coordinate as not cross-checked. A median spread was rejected: it also hides the one clean fold when a single sweep is corrupted.

The [imperfect-machine benchmark](../output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-20-imperfect-machine-benchmark-and-baseline-spread-build-a-reproduc-8a1f6003/report.md) (2026-11-21, simulation) ran the default autocal on randomized machines: low anchors ±30 mm, top anchor off-centre ≤50 mm, true EA 0.8–1.2× config.g.
- **HP3, 10 random machines.** Max error in the print frame (origin at home, z through D, A.x = 0) is 1.11 ± 0.35 mm. Most of it is the common home-reference z shift (−0.87 ± 0.15 mm); the rigid-aligned error is 0.40 ± 0.21 mm.
- **Uneven low heights on three low anchors are an exact rigid tilt for length-only data.** Raw scene-frame errors are 8–28 mm, but no score can see them, and none needs to under the print convention.
- **Stiffness** shifts the mean low z by about +0.15–0.23 mm over ±20%. Its effect on D.z is unresolved on HP3 sweeps.
- **HP5 (four low anchors).** Uneven heights broke the shared-z fit: 4–19 mm, flagged by rms and the jackknife. Freeing the low z values (`a7a8d179`) restored 0.5–1.0 mm.
- **Force auto-tune knife-edge.** The plateau rule fired on stiction samples and failed 1/10 machines; it was fixed in `1ec95e5e`.

The [rough-bootstrap robustness investigation](../output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-21-rough-bootstrap-robustness-on-imperfect-machines-hp5-pulley-redu-20f0ffb5/report.md) (2026-11-21, simulation, 29 live runs) found a second firing of the same one-sample plateau rule on HP5r seed 3. This one came after breakaway: HP5r always creeps about 140° at 1.44 and 2.02 N before running free at 2.83 N.
- **Effect.** Force max was set at 1.44 N, so size tuning moved 0.25 mm and the rough bootstrap failed.
- **Fix.** The single-sample rule is gone, and the 1ec95e5e stiction guard with it. A flat or reversing sample now counts toward the existing two-decline rule (`9b2941ee`).
- **Results.** HP5r 10/10 random seeds and HP5 8/8 pass, and the comparable runs are bit-identical. It is not a stretch effect. It is noted here because both investigations were found through this benchmark.
- **Open.** Two flat samples in a row after breakaway would still give a near-breakaway force max. A size-tune travel backstop is not implemented.


The [force realisation investigation](../output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-21-force-realisation-error-in-autocal-data-replays-t3-test-complian-8252ab16/report.md) (2026-11-21, simulation) closes the "commanded ≠ realised tension" lead from the compliance probe.
- **Cause.** At rest the sim's torque-mode error is exactly the cogging term: 0.333 N · sin(50 θ) per spool, residual 0.02–0.05 N. It has no friction, hysteresis or scale error. It is the whole of the replay's realised-force improvement, and it acts as per-run top-z spread (sd 0.66 → 0.07 mm on HP3e with the oracle map).
- **Identifiability** (corrected by review-338b0f4b33a8).
  - The sweep rms identifies a force error that varies with recorded variables: encoder-angle ripple (rms 20× lower at the truth) and an offset that flips with motion direction.
  - Once the ripple is modelled, a per-motor constant offset/scale is visible too (+0.5 N on one motor: rms 14–16× HP3e, 2× HP5r).
  - A common scale is degenerate with k. On constant-force meshes so is a common offset.
  - Unmodelled, a single-motor offset leaks into k and top z (+1.5–2.7 mm per 0.5 N).
  - **Blind test.** A per-motor period scan (1–400/turn, sweep CV) plus offset/scale contrasts recovers a 137/turn live ripple and a 0.5 N offset: HP3e 0.17 mm mean on 7 held-out cases (6/7 < 0.3). The HP5r blind test is not done.
  - **Hardware.** External force calibration is a cross-check, and the only source of absolute force scale.
- **Joint fit.** Fitting the force terms jointly makes the rms sharply k-sensitive. Joint k + force terms then gives 0.16 ± 0.06 mm on 11 HP3 ellipsoid runs (k +0.7 ± 0.3%; live 2.22 mm) and 0.74 ± 0.21 mm on 6 HP5r runs (k +3.8%). It needs neither the probe nor configured stiffness.
- **Robustness.** The fit holds at 2× (live too) and 4× ripple and degrades gracefully with unknown harmonics (0.66 mm). With a ±0.2 N direction offset it works only if the direction term is fitted. Two-sided data kept as separate points is worse.
- **Open.**
  - HP5r keeps +0.4–0.9 mm top z that is neither force nor compliance; the M666 Y reeving fixes only k.
  - HP5r collection fails at 2× cogging, because force auto-tune reads ripple as stiction.
  - The HP4-style home-stretch emulation on HP5r was a double count. The production term is gated to HP3/HP4.


The [HP5 high-cogging follow-up](../output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/validation/high-cogging/report.md)
verified the cap during profiled bootstrap at 3× and added a stable-fit stop for
well-constrained noisy measurements. The original 2× seed 6 recording reaches
the same 1.54 mm maximum error after 8 of its 23 sweeps.

The [guarded force fit](../output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-21-production-force-realisation-fitting-reuse-blind-period-scan-and-b5e5ea67/report.md) adds an HP3 opt-in on top of
home-strain correction. Eleven archived simulations improve from 0.45 to 0.15 mm
mean maximum anchor error. The additional cost is 1.56 s median CPU time per fit
stage. Mixed force errors and negative inferred tensions limit the HP5r joint-fit
result to research evidence.

The [HP5r rotation replay and paired collection study](../output/program-2026-11-20-autocal-on-imperfect-machines-pr-121-follow-up-1-a-wrong-configu-4b12cdf0/session-2026-11-21-isolate-the-remaining-hp5r-top-anchor-height-bias-with-controlle-76b38f06/report.md)
uses verified encoder replays to explain the remaining positive height error.
The low-line attachments rotate with the effector. Freezing their attitude in
the geometric controls removes most of the positive top-height shift. General
effector attitude is underdetermined by the existing ranges. In the archived
nominal HP5r pair the sweep workflow gives 1.06 mm after 4,686 simulated seconds
and the ellipsoid gives 2.94 mm after 6,495 simulated seconds, with the same
point objective.

The [DaRUS spool-physics handoff](../output/session-copacabana-planea/autocal-handoff.md) compares the six native CI scenarios before and after strained drum storage and radial flattening became available. All six ran with both effects disabled in that historical comparison. Subsequent hp-sim-3d builds use strained storage on elastic spools by default. For a future enabled run, it proposes varying winding tension independently of payout tension before estimating anchor geometry.
