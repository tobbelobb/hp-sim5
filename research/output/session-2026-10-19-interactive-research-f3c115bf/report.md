# HP4 autocal wobble: one XPBD damping displacement per step (per-visit re-evaluation is an energy source)

Status: wobble root cause fixed and reproduced in RRDs. Full one-substep
autocal completion is still blocked by a separate post-move drift.

## Abstract

HP4 autocal at the default `--physics-substeps 1` did not settle: the effector
limit-cycled, the settling checks timed out and force selection moved. We
reproduced the wobble in same-backend A/B Rerun recordings of a loaded HP4
checkpoint: the JavaScript legacy arm keeps a late-window effector Z range of
4.07 mm and a peak angular speed of 3.63 rad/s at about a 24 ms period, with
cable force swinging 18.0-46.2 N, while the fixed arm reaches 7.2e-14 m and
zero angular speed with steady force. Root cause: the XPBD damping term
re-evaluated `grad(C)·(x - x_prev)` on every Gauss-Seidel visit, so the second
of HP4's two visits consumed its own correction; at dt = 2 ms with
`damping = 5` and `compliance = 1/20000` (`gamma = 0.125`, `alphaTilde = 12.5`)
that injects rather than removes energy. Freezing the displacement at its first
visit per step, mirrored in the JavaScript, Rust/native/Wasm, Python and Warp
solvers, settles the state. The fix does not yet make autocal complete at one
substep: collection now fails on a monotone post-move drift (about 14-15 deg/s
of encoder payout with no G-code in flight), which four substeps do not show.
All evidence is simulated.

## Reproducing the wobble

`experiments/E13-rrd-js-pair/run_pair.sh` recreates it from scratch:

```
cd output/research/sessions/2026-10-19-interactive-research-f3c115bf/experiments/E13-rrd-js-pair
STEPS=2000 STRIDE=2 BACKEND=js ./run_pair.sh
```

The script loads the E01 HP4 checkpoint into a headless JavaScript world, runs
2000 steps at dt = 2 ms with `physicsSubsteps: 1` and `solverMode:
legacyDamping`, then repeats with `solverMode: baseline` (the fix). Both arms
report `physicsEngine: javascript`; only the solver mode differs. Each arm
writes `wobble-<mode>-js.json` (numeric trace) and a Rerun `.rrd`
(about 9 MB) in `legacy/` and `fixed/`. `rerun legacy/*.rrd fixed/*.rrd` shows
the rocking transform and the encoder curves; the numeric summary is enough to
regress on:

| Arm | late z range | late max omega | late cable force |
| --- | --- | --- | --- |
| legacy | 4.068e-3 m | 3.633 rad/s | 18.0-46.2 N |
| fixed | 7.21e-14 m | 0 | 20.6 N steady |

`experiments/E09-rrd/record_wobble.mjs` is an earlier cross-backend version
(JS legacy versus native fixed). Its first attempt accidentally used the native
backend in both arms and is preserved as `*-first-attempt/`; two arms that agree
on the backend are required.

## What in the solver allows it

Order of operations in one physics step (JavaScript `CableJoint`/`PBDCableConstraintSolver`,
mirrored in Rust and Python):

1. record previous poses `x_prev`;
2. `motors(false)`: advance motor targets for the step;
3. integrate velocities to trial positions `x_trial`;
4. `sync(false)`: write rigid-body and cable attachments;
5. merge/split and friction;
6. Gauss-Seidel constraint loop, `solverIterations` visits per path;
7. `sync(true)`: write solved poses back;
8. velocities from `(x - x_prev)/dt`;
9. `motors(true)`, then encoder reads.

The compliant cable constraint is XPBD (Macklin et al. 2016, eqs. 19-26):

```
C   = |p_b - p_a| - restLength
dC  = grad(C) · (x - x_prev)                    <- damping displacement
delta = (-C - alphaTilde * lambda + gamma * dC) / (1 + gamma) * grad(C) ...
```

with `alphaTilde = compliance/dt^2` and `gamma = compliance * damping / dt`.
For HP4 one path carries `compliance = 5e-5 m/N`, `damping = 5`, `dt = 2e-3 s`,
so `gamma = 0.125` and `alphaTilde = 12.5`.

The error term `C` must be re-evaluated each visit: that is what Gauss-Seidel
iterates on. The damping term must not be. `dC` is a *velocity* quantity, fixed
for the step. Re-deriving it from the partially solved positions makes visit 2
measure visit 1's correction as if it were additional velocity, and the damping
impulse then pushes in the direction of the residual motion instead of opposing
it. Two visits per path at `gamma = 0.125` are enough to keep the loaded HP4
network oscillating. E14 measures this directly: across 709 joint-visit pairs
the legacy `jDx` changes in 34.6 % of visits (median relative change 0.65,
p90 3.25, max 419); with the fix it changes in none. E05 shows the wobble
growing with the damping coefficient, which is the signature of a
sign-carrying feedback error rather than under-damping, and it survives with
gravity disabled and all 51 joints taut.

This is also what the PBD literature recommends. Macklin et al. (2016, sec. 5)
treat damping as a separate velocity-level update; Small Steps (Macklin et al.
2019, sec. 4.1 and 6) exploits that substeps make the velocity error small and
warns that damping applied inside the projection pass is iteration-order
dependent; PBDBodies (Müller et al. 2020, sec. 3.6) likewise keeps damping out
of the position projection. Freezing the step displacement is the minimal
version of that separation that fits the existing single-pass architecture.

## The fix

One cached value per joint per solve call, consumed on every visit:

- `src/js/cable_joints_3d/cable_joints_core.js`: `jdxCache` keyed by joint; the
  first visit stores `jDx`, later visits reuse it. `solverMode:
  'legacyDamping'` keeps the old behaviour purely for A/B experiments, and an
  opt-in `solverTrace` resource records per-visit rows (off by default).
- `src/wasm/cable_solver/src/{lib.rs,engine.rs}`: `damping_disp: Vec<f64>`
  initialised to NaN per solve; first visit writes, later visits read.
- `src/python/cable_joints_3d/pbd_cable_constraint_solver.py`:
  `damping_displacements` dict keyed by `(path, joint)`.
- `src/python/cable_joints_3d/warp_solver.py`: `Joint.damping_disp` reset to
  NaN per solve, 14 lines.
- `public/wasm/cable_solver.wasm` rebuilt from the Rust sources.

The `(1 + gamma)` denominator and the multiplier update are untouched, so the
only change is *when* the damping displacement is sampled. E15 checks the other
obvious variant, damping only on the first visit: it under-damps (late z range
1.59e-3 m, omega 0.634 rad/s), because XPBD's `gamma` is derived per visit.
Frozen displacement with damping on every visit is the consistent choice.

## Evidence

| ID | What was measured | Result |
| --- | --- | --- |
| E05 | damping sweep, gravity off, taut check | wobble grows with damping; gravity-off still wobbles; 51/51 taut |
| E06 | late-window decomposition | mean `|C|` 3.8e-3 m, mean `|jDx|` 1.2e-2 m; spin/rotation dominate |
| E07 | substep x mode matrix | sub1 legacy 2.1633e-3 m / 1.767 rad/s; sub1 fixed 3.28e-14 m / 0; sub2 both settle |
| E08 | cable force histories | legacy 0-10.203 N (mean 2.042); fixed 2.2577 N |
| E09 | cross-backend RRD pair | JS legacy 8.8 MB, late 4.07e-3 m / 3.63 rad/s; native fixed 5.9 MB, 6.5e-14 m / 0 |
| E12 | throughput | native sub1 2586 steps/s, sub2 1326, sub4 662; JS ~204 either mode |
| E13 | same-backend JS A/B RRD | legacy 4.068e-3 m / 3.633 rad/s / ~24 ms / 18-46 N; fixed 7.21e-14 m / 0 / 20.6 N |
| E14 | per-visit `jDx` trace | 34.6 % of 709 visit pairs change (median 0.65, p90 3.25); fixed 0 % |
| E15 | damping splitting | legacy residual limit cycle; baseline settles; first-visit-only under-damps |

Numeric artifacts are under `experiments/`; the RRDs are in `E09-rrd/` and
`E13-rrd-js-pair/`.

## Tests

- `npx jest tests/js/cable_joints_3d`: 32 suites / 188 tests pass.
- `npx jest autocal/control`: 123 pass.
- `node --test tests/benchmark/hp4_stability.test.mjs`: 3/3 pass (rerun after
  the last source change). `hp4_open_loop_substeps.test.mjs`: 4/4.
- Python: 398 passed, 5 skipped, 3 deselected; differential parity 176 pass.
- The only fixture edit is the `constraintForce*` tolerance in
  `tests/fixtures/python_3d_parity/cable_solver_bodies.json`, 1e-9 -> 5e-8,
  because the float summation order inside a joint changed.
- Native/Wasm benchmark suites were not rerun after the final deadline-only
  edits (those edits do not touch physics).

## Autocal at one substep

Headless runs use `scripts/autocal_headless.mjs`, which advances
`ceil(seconds * speed / dt)` steps. Headless `--speedup 1` is the faithful
equivalent of the browser's `--speedup 25` window; headless `--speedup 25` is
not, and previous partial evidence produced that way (`autocal-sub1d`) is
treated as non-equivalent.

| Run | Substeps | Result |
| --- | --- | --- |
| `autocal-sub1-s1` | 1 | force bracket matched, no knee before the 20 N cap, fallback 1 N; bootstrap 3 sweeps / 60 points completed with pullouts up to 472 simulated s; stopped in active learning. Partial. |
| `autocal-sub2-s1` | 2 | knee 6.5739 N (d_max 3586.22 deg); size tune 590.238 mm; then encoder-stability timeout at the first sweep point. Failed, 1547.55 simulated s. |
| `autocal-sub1-fixedforce` | 1 | fixed forces 0.0223/0.4455/6.5739 N; size tune 590.238 mm; same 30 s timeout 21 simulated s into the first sweep point. Failed, 515.42 simulated s. |
| `autocal-sub4-fixedforce` | 4 | same fixed forces; size tune 541.759 mm; three bootstrap sweeps (60 points), a fourth sweep, then a completed fit ("ideal quality", score 0.5295, exit 0). Control. |
| `autocal-sub1-iter8` | 1 | `solverIterations` temporarily 8 (scene restored afterwards): size tune 576.685 mm, same 30 s timeout, 2.5x slower (269.2 s versus 109.5 s wall). Rejects convergence as a substitute for substeps. |

The failing settle is not the wobble. In `autocal-sub1-fixedforce` the encoder
replies between 485.4 s and 515.4 s are monotone: A pinned at 2254.500 deg, B
-1040.68 -> -1504.08 deg and C -583.15 -> -94.90 deg, i.e. a near-constant
14-15 deg/s payout, and no G-code is sent in that interval. A 36 Hz rocking
mode would show alternating sign; this is a slow drift that outlives the 30 s
settling budget. The previous session documented the same symptom at one step
(~2 mm/s during the pull), so the fix did not introduce it. Four substeps then complete the
entire pipeline — three bootstrap sweeps, a fourth active-learning sweep and a
fit reported as "ideal quality" with score 0.5295 — where one and two substeps
time out. That makes the drift timestep-controlled and puts it in the same
"stiff cables at dt = 2 ms" family as the original rocking.

Raising `cablePath:solverIterations` from 2 to 8 at one substep does not
substitute for substeps: it fails at the same first sweep point and runs about
2.5x slower (269.2 s versus 109.5 s wall; the scene file was restored
afterwards). The drift is therefore not a constraint-convergence deficit, which
argues against "just iterate more" and for a genuine small-step or
velocity-level treatment. The next experiment is a bounded post-move coast at
1/2/4 substeps recording effector pose and per-motor commanded versus actual
position, to separate closed-loop step catch-up from residual cable motion.

Related: the low-force force-tuning excursions also differ from the stored
browser reference (probe 13 at 0.5669 N: 24.76 deg here versus 1.81 deg there),
even though size tuning matches (590.238 versus 590.418 mm). The probes are
matched-duration excursions sampled through different clocks, so this is
recorded as an open force-tuning question, separate from the damping fix.

## Separate bug fixed: wall-clock deadlines for simulated waits

`runForceTrial` and `waitForStableEncoders` used fixed wall deadlines for waits
expressed in simulated time. At speedup 25 a 25 s simulated window covers
625 s of physics, so a healthy but slow engine hit
`Force trial exceeded its wall-clock deadline`. Both now keep the configured
budget as a floor and extend it in proportion to observed clock progress
(`autocal/control/primitives/simulation_deadline.mjs`), with a hard cap.

## Limitations

- Simulated evidence only: no hardware, and no calibration-accuracy claim.
- The wobble fix is validated on the loaded HP4 checkpoint and on one-substep
  free settling. It is not validated on the full closed-loop autocal workload
  because that workload is still blocked by the drift above.
- `oncePerStepDamping` (E15) no longer exists in the tree; that arm is
  historical numeric evidence.
- `solverMode: legacyDamping` and `solverTrace` are experiment scaffolding kept
  deliberately so the A/B recording stays reproducible; the default path is
  the fix.
- Timestep-dependent numerical damping remains an accuracy concern at one
  substep, as noted by the earlier substep session.

## References

- Macklin, Müller, Chentanez, Kim, *XPBD: Position-Based Simulation of
  Compliant Constrained Dynamics*, MIG 2016, DOI 10.1145/2994258.2994272, sec. 5
  eqs. 19-26 (compliant constraint, `alphaTilde`, `gamma`, separate velocity
  update). Supports the claim that the damping term is a step-level velocity
  quantity.
- Macklin, Storandt, Lu, *Small Steps in Physics Simulation*, SIGGRAPH 2019,
  DOI 10.1145/3345768.3355926, sec. 4.1, 6, 7 (substepping, iteration-order
  dependence of dissipative terms, velocity error caveat). Supports the
  substep-masking explanation.
- Müller, Macklin, Chentanez, Jeschke, Kim, *Detailed Rigid Body Simulation
  with Extended Position Based Dynamics*, SCA 2020, sec. 3.6 (damping kept out
  of the position projection). Supports the fix's placement.
- Code revisions consulted: `966ead90`, `55632cdc` (HP4 open-loop missed
  steps), `6dfd772c` (motor load once per constraint), `145d9cde` (coupled
  rotor dynamics), `63b847c8`.
- Prior sessions: [effector rocking](../2026-10-16-hp4-effector-instability/report.md),
  [substep options](../2026-10-16-hp4-substep-options/report.md),
  [one-step autocal](../2026-10-16-hp4-one-step-autocal/report.md),
  [PBD hotspot](../2026-10-13-pbd-constraint-hotspot/report.md).
- Reference log: `autocal/data/references/hp4_no_buildup_Oct_16_2026_4x_substepping.collection.log`
  (knee 9.2035 N, d_max 3492.240 deg, size tune 590.418 mm).
- Local artifacts: [E13 A/B RRD](experiments/E13-rrd-js-pair/),
  [E14 trace](experiments/E14-visit-trace/visit-trace-summary.json),
  [E15 decay](experiments/E15-damping-splitting/decay.json),
  [E07 matrix](experiments/E07-matrix/matrix-fixed-vs-legacy.json),
  [autocal runs](autocal-sub1-fixedforce/sweeps.full_auto.log).
