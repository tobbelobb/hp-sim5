# HP4 autocal wobble: XPBD damping displacement evaluated per solver visit

Objective: make headless HP4 autocal settle and collect at the default
`--physics-substeps 1`, so settling checks stop misfiring and `force_max`
selection and timeouts stop being driven by a wobbling effector. Secondary
target: the user's `--physics-substeps 2 --speedup 25` command against
`output/extended-autocal/no-buildup-reference-010/sweeps.json` (that dataset is
absent in this worktree, so a session-local dataset path is used).

Constraint: simulation research only; no hardware, no calibration-accuracy
claim from successful collection. Use `.venv/bin/python`; keep source changes
small; do not commit. Recording budget per
`research/autocal-recording-performance.md`: `none` for numeric sweeps,
`events` for the long autocal runs, sampled/full only for physics evidence.
One long autocal run at a time; bounded tests otherwise.

Accepted steering: reproduce the wobble in an RRD with a repeatable procedure,
explain the PBD equation ordering that permits it, implement a PBD-best-practice
fix mirrored across backends, and only then validate with headless autocal.
Later steering: also check what earlier sessions and commits already ruled out.

Previous research (checked): HP4 effector rocking (session
`2026-10-16-hp4-effector-instability`: one 2 ms step sustains rocking; four
0.5 ms steps settle), substep options (`2026-10-16-hp4-substep-options`:
settling passes at one step but after ~472 simulated seconds), single-step
autocal (`2026-10-16-hp4-one-step-autocal`: force tuning completes, size tuning
times out; the effector still travelled ~2 mm/s during the pull), and the PBD
hotspot work (`2026-10-13-pbd-constraint-hotspot`). Commits `55632cdc`
(open-loop missed steps), `6dfd772c` (motor load once per constraint),
`145d9cde` (coupled rotor dynamics), `966ead90`, `63b847c8`. None of them
identified per-visit damping displacement as the energy source; all of them
documented the slow-travel symptom that still blocks the present run.

## Hypotheses and outcomes

- H1 (supported): the compliant cable damping term recomputes
  `jDx = grad(C) . (x_current - x_prev)` from partially solved positions on
  every Gauss-Seidel visit. HP4 uses two solver iterations per path,
  `damping = 5`, `compliance = 1/20000`, `dt = 2 ms`, hence `gamma = 0.125` and
  `alphaTilde = 12.5`. The second visit's `jDx` contains the first visit's own
  correction, so the damping term feeds on its own output and adds energy; the
  effector settles into a ~24 ms limit cycle at one substep. Freezing `jDx` to
  its first-visit value per step keeps the `(1 + gamma)` denominator and the
  intended damping impulse per visit, and removes the energy source.
  Evidence: E05, E06, E07, E08, E09, E13, E14, E15.
- H2 (supported as the masking mechanism): more substeps shrink `dt`, shrink
  the per-visit correction and therefore the spurious term, so the legacy
  scheme happens to settle at 2 or 4 substeps. This explains why four substeps
  worked once. Evidence: E01, E07, E12.
- H3 (rejected): gravity, slack cables or non-taut friction states cause the
  wobble. Gravity-off still wobbled and all 51 joints stayed taut.
  Evidence: E05, E06.
- H4 (rejected as sufficient): earlier motor-load and quaternion/compliance
  fixes explain it. They fixed real defects but the one-step case still failed.
  Evidence: previous session reports, E01 baseline.
- H5 (rejected as stated): the damping fix alone makes a full one-substep
  autocal run complete. Not established: collection still fails, but in a
  different way (see H6). Evidence: sub1-s1, sub2-s1, sub1-fixedforce.
- H6 (supported): the remaining one-substep failure is a distinct monotone
  post-move drift, not the 36 Hz wobble. Encoder replies during the failing
  settle are monotone (about 14-15 deg/s, no oscillation) with no G-code in
  flight; the same slow travel was recorded before this fix by
  `2026-10-16-hp4-one-step-autocal` (~2 mm/s during the pull).
  Evidence: sub1-fixedforce `events.jsonl`, prior session.
- H7 (supported): the drift is timestep-controlled rather than a force-tuning
  defect: four substeps complete the same fixed-force bootstrap and fit where
  one and two substeps time out. Evidence: sub4-fixedforce.
- H8 (rejected): more Gauss-Seidel iterations at one substep substitute for
  substeps. Eight iterations failed at the same first sweep point and ran
  about 2.5x slower. Evidence: sub1-iter8.

## Experiment IDs and status

- E01 checkpoint/substep baseline. HP4 loaded checkpoint, body entity 58;
  one-substep start angular speed > 1 rad/s; four-substep control settles.
- E02 iteration trace: per-visit multiplier/error trace.
- E03 ablations: iteration count, backend and damping scale.
- E04 force diagnostics during the wobble.
- E05 damping sweep: larger damping made the late-window wobble grow
  monotonically; gravity-off and all-taut checks rejected H3.
- E06 jDx decomposition: late mean `|C|` 3.8e-3 m versus mean `|jDx|` 1.2e-2 m;
  spin and rotational-gradient terms dominate.
- E07 numeric matrix (`experiments/E07-matrix/matrix-fixed-vs-legacy.json`):
  sub1 legacy zRange 2.1633e-3 m, maxOmega 1.7674 rad/s; sub1 fixed zRange
  3.28e-14 m and zero omega; sub2 legacy and fixed both settle; 51 taut joints
  in every case.
- E08 cable forces: legacy late force oscillates 0-10.203 N (mean 2.042 N);
  fixed holds near 2.2577 N.
- E09 native RRD pair (`experiments/E09-rrd/`): first attempt used the native
  backend in both arms and settled in both (preserved as
  `*-first-attempt/`); the retained pair uses JS legacy (8.8 MB, all-time z
  range 0.01147 m, late 0.00407 m, late max omega 3.63 rad/s) against native
  fixed (5.9 MB, late 6.5e-14 m, omega 0).
- E12 timing (`experiments/E12-timing/timing.jsonl`): native sub1 ~2586
  steps/s, sub2 ~1326, sub4 ~662; JS fixed ~204 and JS legacy ~202-205.
- E13 same-backend JS A/B RRD (`experiments/E13-rrd-js-pair/`,
  `STEPS=2000 STRIDE=2 BACKEND=js ./run_pair.sh`): both arms
  `physicsEngine: javascript`, only `solverMode` differs. Legacy late z range
  4.068e-3 m, late max omega 3.633 rad/s, dominant period ~24 ms (about 36-42
  Hz), late max cable force 18.0-46.2 N. Fixed late z range 7.21e-14 m,
  omega 0, force steady at 20.6 N. This is the primary reproducible wobble RRD.
- E14 visit trace (`experiments/E14-visit-trace/visit_trace.mjs`,
  `visit-trace-summary.json`): 709 joint-visit pairs; `jDx` changes between
  visits in 34.6 %, median relative change 0.651, p90 3.25, max 419; fixed
  exactly 0. Joint (1,6) at step 595: legacy 0.010719 -> 0.011767 m, fixed
  0.014312 m on both visits.
- E15 damping splitting (`experiments/E15-damping-splitting/decay.mjs`,
  `decay.json`): legacy keeps a residual limit cycle (late z range 9.07e-4 m,
  late max omega 1.02 rad/s); baseline (frozen displacement, damp on every
  visit) settles (5.3e-14 m, 0); damping only on the first visit
  (`oncePerStepDamping`, experimental and since reverted) under-damps
  (1.59e-3 m, 0.634 rad/s). Re-applying the frozen displacement per visit is
  therefore both necessary and sufficient; the mode no longer exists in the
  tree, so this arm is historical evidence only.
- sub1-s1: one-substep headless autocal, `--speedup 1`, events. Force tuning
  reproduced the reference probe bracket (probe 13 test 0.5669 N) but travel
  kept rising to the 20 N cap, so no knee was found and `force_max` fell back
  to 1 N. Bootstrap then completed three sweeps / 60 points, with pullouts
  taking up to 472 simulated seconds. Stopped during active-learning sweep 1.
  Partial evidence only.
- sub2-s1: two-substep headless autocal, `--speedup 1`, events. Knee found at
  6.5739 N (d_max 3586.22 deg); size tune 590.238 mm; then
  `Timed out waiting for encoder stability after 30000ms` at the first sweep
  point. Manifest status failed, 1547.55 simulated s, 467.2 s wall.
- sub1-fixedforce: one-substep, `--no-auto-tune-force --force-low 0.0223
  --force-mid 0.4455 --force-max 6.5739`. Size tune 590.238 mm (reference
  590.418 mm), then the same 30 s settling timeout 21 simulated s into the
  first sweep point. Manifest failed, 515.42 simulated s, 109.5 s wall.
  Encoder trace: A pinned at 2254.500 deg, B -1040.68 -> -1504.08 deg and C
  -583.15 -> -94.90 deg at a near-constant 14-15 deg/s, with no G-code
  between 485.4 s and 515.4 s. This is drift, not wobble.
- sub4-fixedforce: four-substep control with identical forces. Size tune
  541.759 mm, three bootstrap sweeps (60 points), a fourth active-learning
  sweep, then a completed fit: "Found parameters of ideal quality", score
  0.5295, exit 0.
- sub1-iter8: one substep with `cablePath:solverIterations` temporarily raised
  from 2 to 8 in `hp4_rigid_body.usda` (restored afterwards). Size tune
  576.685 mm, then the same 30 s timeout at the first sweep point; 485.18
  simulated s and 269.2 s wall versus 109.5 s wall at two iterations.

## Budgets

Movement: long autocal runs bounded by `--max-steps 1` (bootstrap only).
Compute: numeric A/B matrices are a few thousand steps; RRD pairs are 2000
steps at stride 2; E12 timing is 500 steps per arm. Recording: `events` for
autocal runs (0 MB RRD per the recording table is for `none`; `events` costs
about 3.4 MB per 200k steps), full/sampled only where physics inspection was
needed (E09/E13 used recorder events and numeric traces).

## Next decision

The sub4 control completes end to end and the iter8 control fails, so the
remaining defect is timestep-controlled but is not a constraint-convergence
problem. Next: a bounded numeric reproduction of the post-move coast at
substeps 1/2/4 with effector pose and per-motor commanded vs actual position
recorded, to decide between (a) closed-loop step catch-up after a large move
and (b) residual cable-network motion that only substeps resolve. Only then
choose between a velocity-level/kinematic treatment and an explicit substep
default.
