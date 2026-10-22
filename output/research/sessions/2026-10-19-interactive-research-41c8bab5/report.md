# HP4 one-substep autocal wobble: XPBD damping feedback and acceptance status

Status: in progress (fix verified in replay; full autocal acceptance incomplete).

## Abstract

Loaded HP4 autocal wobbled with the default one 2 ms physics step, disturbing
settling and force selection. We froze a production checkpoint, replayed it in
the native Rust solver, and recorded the effector in Rerun. With two XPBD
Gauss-Seidel passes the effector entered a deterministic bounded limit cycle
(6.27 mm late span, 1.79 rad/s). The cause was order-dependent damping:
each joint's velocity-dependent displacement was recomputed from poses already
changed by earlier visits, so damping differentiated its own correction and
injected energy; velocity reconstruction then fed the biased pose into the next
step. Caching each joint's first-visit displacement per substep removed the
cycle at the default one 2 ms step and at explicit two substeps (late span
about 1e-10 to 1e-9 mm, zero angular speed). Exact requested substeps replaced
implicit 1 ms promotion. The full headless default-one-substep autocal still
has not completed: the clean run was externally interrupted at 210,000 steps
after entering sweep collection, and earlier runs hit a now-fixed deadline bug.
This is simulated evidence, not hardware validation.

## Question and scope

The user's symptom was an HP4 end effector that appeared to wobble when it
should settle, leading autocal to pick a wrong `force_max` or time out. A
one-off run with `--physics-substeps 4` completed collection once, but the
default one substep did not. The immediate question was whether this is a
numerical PBD/XPBD artifact, what equation order creates it, and whether a
small best-practice fix can make the default cadence settle.

The eventual target is a headless run similar to:

```bash
.venv/bin/python autocal/autocal.py --sim --machine-type hangprinter_4 \
  --physics-substeps 2 --speedup 25 \
  --dataset output/extended-autocal/no-buildup-reference-010/sweeps.json
```

That dataset was absent. Acceptance therefore used a three-sweep unlayered HP4
seed, `bootstrap-first3/sweeps.json`, with the same 25x speedup and an explicit
bounded `--max-steps 3`. No result here should be read as hardware calibration
accuracy.

## Reproduction

The reproduction is a frozen numerical checkpoint, not a live autocal run:

- Fixture: `tests/fixtures/hp4_loaded_stability.json`, SHA-256
  `3d0ce9e30b2eaf35ff315b1102f9a911965b4b96a1625e998399503c7e725341`.
- Frozen at replay step 584000 of
  `hangprinter-20261016T092144.564660Z-35df9491.rrd`.
- Scene: `public/usd_scenes/hp4_rigid_body.usda`, unlayered, closed-loop
  motors enabled.
- Procedure: one `world.update(dt)` restores saved numeric regions; select
  solver iterations; advance 10000 steps at `dt=0.002`; compute metrics over
  steps 7500-10000.
- Native Rust artifact for the final runs:
  `99c4f129fbf7949b795d3f2f4074dea02ec3e879c782d95da402584748566c60`.
- Source revision: branch `research/research-8a582c8d`, HEAD `3c30e6b9`.

Baseline command:

```bash
SESSION=output/research/sessions/2026-10-19-interactive-research-41c8bab5
WOBBLE_CONFIG='{"solverIterations":2,"legacyScheduler":true,"label":"before-s1-i2"}' \
node "$SESSION/record_hp4_wobble.mjs" 1 "$SESSION/repro-legacy-check"
```

Fixed command:

```bash
WOBBLE_CONFIG='{"solverIterations":2,"label":"s1-i2"}' \
node "$SESSION/record_hp4_wobble.mjs" 1 "$SESSION/final-cache"
```

The runner writes `wobble-<label>.metrics.json` and
`wobble-<label>.samples.csv`; when a second argument is supplied it also opens
a live recorder. `make_wobble_rrd.py` combines the baseline and fixed traces
into one compact Rerun recording containing late effector path, deviation from
the late-window mean and angular speed.

## What the PBD engine evaluates, in order

One external step with `physicsSubsteps=1` and `dt=0.002` runs the systems in
this order:

```text
sync_shared -> save previous poses -> motors -> integrate -> body sync
-> layering/topology -> friction -> constraints -> overcorrections
-> final sync -> velocities(dt) -> encoders
```

The constraint stage runs the authored number of Gauss-Seidel passes over the
cable paths, alternating forward and reverse order. XPBD multipliers persist
across passes. For one joint with endpoints `a` and `b`, the solver evaluates:

```text
d = ta + ra + spin_a + tb + rb + spin_b
D = grad(C) M^-1 grad(C)^T
alpha_tilde = compliance / dt^2
gamma = compliance * damping / dt
delta_lambda = (-C - alpha_tilde * lambda_prev + gamma * d)
               / ((1 + gamma) * D + alpha_tilde)
lambda_next = min(0, lambda_prev + delta_lambda)
```

where `ta`, `ra`, `spin_a` are translational, rotational and spin
displacements of endpoint `a`, and likewise for `b`. `lambda` is the XPBD
multiplier; `alpha_tilde` is the compliance in multiplier units; `gamma`
scales the velocity-dependent damping term. The multiplier update is a
projection onto the non-positive half-line, appropriate for a cable that can
only pull.

After the last pass, `velocities(dt)` reconstructs each body's velocity from
the final pose and the saved previous pose. Encoders are then advanced. The
final pose is therefore not re-integrated; it becomes the next step's
beginning state through that reconstructed velocity.

## Why the cycle forms

The bug was not convergence failure. It was an equation-order bug.

On the first Gauss-Seidel visit, `d` is a genuine beginning-of-substep
displacement. On a later visit to the same joint, the legacy code recomputed
`d` from the poses that the earlier visits had already changed. The damping
term therefore differentiated the solver's own correction. With two passes,
the second pass sees a displacement caused by the first pass and adds another
damped correction in the same direction. Damping stops dissipating and starts
injecting energy. The final pose is biased, `velocities(dt)` turns that bias
into a residual velocity, and the next external step integrates it. The result
is a bounded limit cycle rather than a divergence.

This explains the observed signature:

- one pass is stable (`7.4e-10 mm`, zero angular speed) because every joint is
  visited once, so there is no earlier-visit displacement to feed on;
- two passes oscillate (`6.272 mm`, `1.787 rad/s`);
- the amplitude is non-monotone in iteration count (i3 `0.067 mm`, i4
  `0.362 mm`, i64 `0.001065 mm`), which is a traversal-order signature, not a
  simple insufficient-convergence signature;
- removing motor pseudo-stiffness does not change the cycle;
- damping magnitude and rotor coupling change the amplitude but do not remove
  the two-pass effect.

The per-iteration trace at step 7500 makes the non-contraction visible:
iteration 0 displaced the effector `0.379063 mm`; iteration 1 displaced it
another `0.284514 mm` (`0.751x`); the net after both passes was `0.151241 mm`
(`0.399x`). The projection is not monotone, and the net displacement is not
small compared with the per-pass displacement. Late position autocorrelation
peaks at lag 341 steps (`0.682 s`, correlation `0.866`), with energy spread
across roughly 19-45 Hz.

## Fix

Cache each joint's first-visit damping displacement `d` once per substep,
reset the cache at the start of every substep, and reuse the cached value for
later visits while the multiplier continues to converge. The multiplier still
accumulates and the static solution still improves; only the
velocity-dependent term is frozen to the beginning-of-step state, which is the
XPBD intent. This is a Gauss-Seidel damping practice, not a stiffness or
friction parameter change.

Implemented in:

- native Rust: `src/wasm/cable_solver/src/engine.rs`,
  `src/wasm/cable_solver/src/lib.rs`, rebuilt addon;
- JS: `src/js/cable_joints_3d/cable_joints_core.js`;
- Python: `src/python/cable_joints_3d/pbd_cable_constraint_solver.py`;
- Warp: `src/python/cable_joints_3d/warp_solver.py`;
- rebuilt `public/wasm/cable_solver.wasm`.

Substep policy: requested and effective substeps are exact. The earlier
implicit promotion of a loaded one-substep world to two 1 ms steps was removed
(`hp-sim-3d/app/simulationSystems.js`,
`src/python/cable_joints_3d/machine_simulation.py`). `physicsSubstepsEffective`
is recorded in the headless status and recorder context
(`scripts/autocal_headless.mjs`).

The deadline correction is separate: headless advances are chunked to at most
500 ms (`autocal/control/primitives/headless_bridge.mjs`), force trials project
a deadline from the observed simulation-clock rate, keep a 600 s hard cap for
simulation and abort if the simulation clock stops advancing
(`autocal/control/primitives/simulation_deadline.mjs`,
`force_tuning.mjs`, `uncalibrated_actions.mjs`,
`sweep_data_collection.mjs`).

## Evidence

### Fixture replay

Late metrics are over steps 7500-10000 of a 10000-step replay.

| Setup | Late 3D corner-proxy diagonal | Late max angular speed |
| --- | ---: | ---: |
| Legacy `1 x 2 ms`, i1 | `7.407949e-10 mm` | `0 rad/s` |
| Legacy `1 x 2 ms`, i2 | `6.272107 mm` | `1.78668 rad/s` |
| Cache `1 x 2 ms`, i2 (default) | `1.19275e-10 mm` | `0` |
| Cache `2 x 1 ms`, i2 | `1.38641e-9 mm` | `0` |
| Cache `1 x 2 ms`, i1 | `7.40795e-10 mm` | `0` |
| Cache `1 x 2 ms`, i3 | `0.512348 mm` | `0.472138 rad/s` |
| Cache `1 x 2 ms`, i4 | `3.82642e-10 mm` | `0` |
| Cache `1 x 2 ms`, i8 | `0.00276242 mm` | `0.00690564 rad/s` |

The i3 residual is the remaining first-visit traversal-order limitation. It is
not the pre-fix 6.27 mm cycle, but it shows that caching does not make the
solver order-independent.

Global-snapshot control (one beginning-of-substep snapshot instead of a
first-visit cache) was worse:

| Setup | Late diagonal | Late max omega |
| --- | ---: | ---: |
| Snapshot i2 | `0.120463 mm` | `0.122418 rad/s` |
| Snapshot i4 | `0.124975 mm` | `0.177678 rad/s` |
| Snapshot i8 | `0.268322 mm` | `0.152968 rad/s` |

### Recordings

- Baseline: `wobble-baseline.rrd`, SHA-256
  `ff27609f93d4c1dbfc23d7af1fc0a15c88a3feffbd82c8cf70f64da90462736c`.
- Fixed: `wobble-damping-cache.rrd`, SHA-256
  `229663f079530af0fd020a95fbeb3c5f08b2f9548af10d0ccb6a165f7314af9a`.
- Screenshot: `wobble-damping-cache-view.png`.
- Metrics: `baseline-i1-i2/`, `final-cache/`, `recheck/`, `snapshot/`,
  `policy-matrix/`, `iteration-matrix/`, `dt-matrix/`, `sweeps/`.

### Acceptance attempts

Command shape (headless):

```text
autocal/autocal.py --sim --headless-sim --machine-type hangprinter_4 \
  --speedup 25 --dataset <job>/sweeps.json \
  --max-steps 3 --recording-mode events
```

Seed: `bootstrap-first3/sweeps.json`, SHA-256
`92b69ecdc03e7857d44f3e9dd859383dd7f41af83a9c18fcdd1091eeaa76d88c`
(three sweeps, unlayered, tuned force set).

| Job | Status | Last step | Requested/effective | Outcome |
| --- | --- | ---: | --- | --- |
| `acceptance-default-s1-clean-r2` | running when launcher restarted | 210000 | `1/1` | interrupted, no physics error; iteration 1/3, collecting sweep |
| `acceptance-cache-bootstrap-default-s1-r1` | stopped | 2031342 | `1/1` | contaminated partial (force probe 5/120) |
| `acceptance-seeded-default-s1` | failed | 2156841 | `1/2` (old promotion) | collector exit 1, pre-fix deadline |
| `acceptance-seeded-explicit-s2` | failed | 2156841 | `2/2` | collector exit 1, pre-fix deadline |
| `acceptance-default-25x` | failed | 8716124 | `1/2` (old promotion) | collector exit 1 |
| `acceptance-default-25x-r2` | failed | 4906442 | `1/2` (old promotion) | collector exit 1 |
| `acceptance-explicit-s2-25x` | running when stopped | 4774000 | `2/2` | stopped, no final manifest |
| `acceptance-explicit-s2-25x-r2` | failed | 4906442 | `2/2` | collector exit 1 |
| `acceptance-reference3-hp4-25x-s2` | running when stopped | 3096000 | `2/2` | stopped, no final manifest |
| `acceptance-reference-hp4-25x-s2` | complete | 0 | `1/1` | zero-step smoke only; not acceptance evidence |

The clean default run is the closest to success: it started with requested and
effective substeps `1`, reused the seed's force set, and entered sweep
collection without a physics or timeout error. It did not finish because the
launcher restarted and killed its isolated processes. It also did not re-test
`force_max` selection because it reused tuning.

No completed end-to-end HP4 autocal run exists in this session. The user's
question "did autocal run?" therefore has a strict answer: the CLI was
exercised headlessly for 210,000 physical steps and reached live collection,
but a full bounded run has not completed, and the target dataset was absent.

## Validation and tool checks

- `cargo check`: passed.
- `npm run build:native -- --no-pgo`: passed.
- `npm run build:wasm`: passed.
- Python physics/differential suites: `180 passed`.
- `autocal/tests/test_headless_sim.py`: `37 passed, 2 deselected`.
- Focused Jest suites: `43 passed`.
- `node --test tests/benchmark/hp4_stability.test.mjs`: `2 passed`
  (`4511 ms` default one-substep case, `25775 ms` opt-in substeps).
- `scripts/research_socket_probe.py`: passed.
- `research_preflight`: collector, firmware, recorder and capabilities
  passed; attached-world settings reported a source mismatch because the
  attached world predates the dirty-tree build. This is a tool check, not
  calibration accuracy.

## Limitations

- The stability evidence is a frozen simulated replay, not hardware.
- The first-visit cache is still Gauss-Seidel traversal-order dependent.
  Production HP4 uses a fixed order and `i2`; i3 shows a small residual.
  A true Jacobi/SSOR formulation and separate velocity damping were not tested.
- The full default-one-substep autocal acceptance is incomplete. The bounded
  run was interrupted externally, and no final manifest exists.
- The clean run reused a pre-tuned force set, so `force_max` selection under
  default substeps is not yet demonstrated.
- The target dataset `output/extended-autocal/no-buildup-reference-010/sweeps.json`
  was absent and was not fabricated.
- Earlier multi-hour acceptance jobs that failed with collector exit 1 are
  preserved as failed trials; several predate the deadline fix and one is
  explicitly contaminated.

## Prior art

Upstream `main` commit `bf9f5399` (`Fix HP4 XPBD damping feedback and simulation
waits`, equivalent to `7f7248d5`) already contains essentially the same damping
cache and deadline ideas. This branch adds the exact-substep policy, broader
cross-backend tests, the frozen fixture, RRD evidence and acceptance
instrumentation. The damping cache itself is not claimed as novel.

## Next steps

1. Re-run the clean default one-substep headless acceptance in a new directory
   and wait for `status=complete`, `stop_reason=max-steps`,
   `recording_drained=true`, requested/effective `1/1`.
2. Run the same seed with explicit `--physics-substeps 2`.
3. Run one acceptance without reusing force tuning, to exercise `force_max`
   selection with the fix.
4. When the target dataset exists, repeat the two-substep 25x headless run
   against it.

## References

- [XPBD paper notes](../../../../ai_docs/XPBD/XPBD.md); Müller, Macklin, Chentanez,
  "XPBD: Position-Based Simulation of Compliant Constrained Dynamics", 2016,
  DOI `10.1145/2994258.2994272`. Used for the multiplier/compliance/damping
  equations and the invariant that velocity-dependent terms belong to the
  beginning-of-step state.
- [Small Steps paper notes](../../../../ai_docs/Smallsteps/Smallsteps.md); Macklin et al.,
  "Small Steps in Physics Simulation", 2019, DOI `10.1145/3309486.3340247`.
  Used for the substep/stability trade-off and the decision to make substeps
  explicit.
- [PBD Bodies paper notes](../../../../ai_docs/PBDBodies/PBDBodies.md); Müller et al.,
  "Detailed Rigid Body Simulation with Extended Position Based Dynamics",
  2020, DOI `10.1111/cgf.14105`. Used for the rigid-body XPBD formulation.
- Upstream commit `bf9f5399` in this repository: prior damping feedback and
  simulation-wait correction.
- Prior sessions:
  [HP4 effector instability](../2026-10-16-hp4-effector-instability/report.md),
  [HP4 substep options](../2026-10-16-hp4-substep-options/report.md),
  [HP4 one-step autocal](../2026-10-16-hp4-one-step-autocal/report.md),
  [HP4 load bookkeeping](../2026-10-16-hp4-load-bookkeeping/report.md),
  [HP4 roller propagation](../2026-10-16-hp4-roller-propagation/report.md).
- Local evidence: [baseline RRD](wobble-baseline.rrd),
  [fixed RRD](wobble-damping-cache.rrd),
  [fixture](../../../../tests/fixtures/hp4_loaded_stability.json),
  [research notebook](research.md).
