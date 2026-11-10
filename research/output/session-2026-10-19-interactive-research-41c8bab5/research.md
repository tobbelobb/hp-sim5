# HP4 one-substep wobble: XPBD damping feedback, fix and acceptance status

## Objective

Reproduce and explain the loaded-HP4 effector wobble that prevents autocal from
settling with the default one 2 ms physics step, make it visible in a
reproducible Rerun recording, test a PBD/XPBD best-practice fix, and finish a
headless HP4 full-auto run. The user's eventual target command is the
two-substep, 25x run against
`output/extended-autocal/no-buildup-reference-010/sweeps.json`.

## Accepted steering and constraints

- Reproduce the symptom in an `.rrd` before changing physics, using a procedure
  that can be rerun after the fix.
- Search existing tools, previous sessions and commit history before choosing
  the reproduction.
- Explain which PBD equations are evaluated in what order, using the XPBD,
  Small Steps and PBD-bodies references in `ai_docs/`.
- Prefer a PBD best-practice correction over a parameter workaround.
- Keep source changes small; use `.venv/bin/python`; run acceptance headless.
- Preserve failed/contaminated trials; distinguish simulated evidence from
  hardware and successful tool checks from calibration accuracy.
- No hardware action, commit or branch was requested.
- The dataset path named by the user was absent. Acceptance therefore uses the
  three-sweep `bootstrap-first3/sweeps.json` seed with the same machine,
  unlayered geometry, speedup and substep setting. Do not fabricate the absent
  dataset.

## Tools searched and selected

- `research/README.md`, `research/AGENTS.md`, `More_on_hp-sim5_MCP.md`,
  `More_on_browser_based_work.md`, `Advanced_research_example_autocal.md`.
- Prior sessions: HP4 effector instability, substep options, one-step autocal,
  load bookkeeping, roller propagation (links in `report.md`).
- Commit history: `63b847c8`, `145d9cde`, `55632cdc`, `6dfd772c`, `a36315b1`,
  `11044351`, `76afde01`, `9a632f82`, `3c30e6b9`, and upstream
  `bf9f5399`.
- Reproduction selected: freeze one numerical checkpoint from an existing
  loaded production recording and replay it in the native Rust solver. This is
  deterministic, much faster than a full autocal, and records the effector pose
  directly. Browser UI was not needed; the native fixture run is the evidence
  source and Rerun is the viewer.

## Reproduction procedure

Fixture: `tests/fixtures/hp4_loaded_stability.json`, SHA-256
`3d0ce9e30b2eaf35ff315b1102f9a911965b4b96a1625e998399503c7e725341`,
frozen from replay step 584000 of
`hangprinter-20261016T092144.564660Z-35df9491.rrd`.
Scene: unlayered HP4 rigid-body scene `public/usd_scenes/hp4_rigid_body.usda`,
closed-loop motors enabled. After one `world.update(dt)`, restore all saved
numeric regions, select the solver iteration count, then advance at
`dt=0.002` for 10000 steps. Late metrics use steps 7500-10000.

Before-fix command:

```bash
SESSION=output/research/sessions/2026-10-19-interactive-research-41c8bab5
WOBBLE_CONFIG='{"solverIterations":2,"legacyScheduler":true,"label":"before-s1-i2"}' \
node "$SESSION/record_hp4_wobble.mjs" 1 "$SESSION/repro-legacy-check"
```

After-fix command:

```bash
WOBBLE_CONFIG='{"solverIterations":2,"label":"s1-i2"}' \
node "$SESSION/record_hp4_wobble.mjs" 1 "$SESSION/final-cache"
```

The runner emits `wobble-<label>.metrics.json`, `wobble-<label>.samples.csv`
and, when requested with a second argument, a live `.rrd`.
`make_wobble_rrd.py` combines baseline and fixed traces into one compact Rerun
recording with late effector path, deviation from the late-window mean and
angular speed.

## Engine order and the failure mode

Per external step (one 2 ms step when `physicsSubsteps=1`):

```text
sync_shared -> save previous poses -> motors -> integrate -> body sync
-> layering/topology -> friction -> constraints -> overcorrections
-> final sync -> velocities(dt) -> encoders
```

Constraint passes alternate forward and reverse cable order. XPBD multipliers
persist across passes. Per joint the solver evaluates:

```text
d = ta + ra + spin_a + tb + rb + spin_b
D = grad(C) M^-1 grad(C)^T
alpha_tilde = compliance / dt^2
gamma = compliance * damping / dt
delta_lambda = (-C - alpha_tilde * lambda_prev + gamma * d)
               / ((1 + gamma) * D + alpha_tilde)
lambda_next = min(0, lambda_prev + delta_lambda)
```

The legacy solvers recomputed `d` on every Gauss-Seidel visit from the poses
that earlier visits had already changed. On the second pass the damping term
therefore differentiated its own correction. That makes damping an energy
source rather than a dissipator, leaves a biased final pose, and
`velocities(dt)` reconstructs a residual velocity from that biased pose. The
next external step integrates the residual and the cycle repeats. This explains
why one pass is stable while two passes oscillate, and why the amplitude is
non-monotone in iteration count.

Fix: cache each joint's first-visit damping displacement `d` per substep, reset
the cache every substep, and reuse the cached value while the multiplier keeps
converging. The multiplier still accumulates and the static solution still
improves; the velocity-dependent term is evaluated from the beginning-of-step
state, as XPBD intends. Implemented in native Rust, JS, Python, Warp and the
rebuilt Wasm artifact.

Substep policy: requested and effective substeps are now exact. The earlier
implicit promotion of a loaded one-substep world to two 1 ms steps was removed.
With the damping cache, one external 2 ms step with two iterations is stable
without that workaround.

## Experiment log and outcomes

### E01 - causal baseline and visible symptom

Native Rust artifact used for the final fixture runs:
`99c4f129fbf7949b795d3f2f4074dea02ec3e879c782d95da402584748566c60`.
One external 2 ms step.

| Setup | Late 3D corner-proxy diagonal | Late max angular speed |
| --- | ---: | ---: |
| Legacy `1 x 2 ms`, 1 iteration | `7.407949e-10 mm` | `0 rad/s` |
| Legacy `1 x 2 ms`, 2 iterations | `6.272107 mm` | `1.78668 rad/s` |

The two-iteration case is a deterministic, bounded, quasi-periodic limit cycle.
It persists for 20000 steps. RRD: `wobble-baseline.rrd`, SHA-256
`ff27609f93d4c1dbfc23d7af1fc0a15c88a3feffbd82c8cf70f64da90462736c`.
Evidence: `baseline-i1-i2/wobble-s1_i1.metrics.json`,
`baseline-i1-i2/wobble-s1_i2.metrics.json`. Outcome: supported.

### E02 - substep and iteration policy matrix (pre-fix)

Same fixture, 20000 steps.

| Policy | Late diagonal | Late max omega |
| --- | ---: | ---: |
| `1 x 2 ms`, i1 | `8.597e-10 mm` | `0` |
| `1 x 2 ms`, i2 | `6.2721 mm` | `1.7867 rad/s` |
| `1 x 2 ms`, i64 | `0.001065 mm` | `0.00265 rad/s` |
| `2 x 1 ms`, i1 | `2.050e-9 mm` | `0` |
| `2 x 1 ms`, i2 | `9.764e-10 mm` | `0` |
| `4 x 0.5 ms`, i1 | `1.716e-9 mm` | `0` |
| `4 x 0.5 ms`, i2 | `4.369e-9 mm` | `0` |
| `8 x 0.25 ms`, i1 | `3.753e-9 mm` | `0` |
| `8 x 0.25 ms`, i2 | `1.395e-8 mm` | `0` |

Evidence: `policy-matrix/s1_i*_20k.json`, `s2_i*_20k.json`,
`s4_i*_20k.json`, `s8_i*_20k.json`; also `dt-matrix/`.
Outcome: smaller PBD steps remove the cycle; one 2 ms step does not, before the
fix.

### E03 - parameter perturbations (pre-fix)

| Change | Late diagonal | Late max omega |
| --- | ---: | ---: |
| Cable damping 0 | `0.334 mm` | `0.0317 rad/s` |
| Cable damping 1 | `1.853 mm` | `0.475 rad/s` |
| Cable damping 5 (stock) | `6.272 mm` | `1.787 rad/s` |
| Cable damping 10 | `6.279 mm` | `1.763 rad/s` |
| Cable damping 20 | `6.921 mm` | `1.883 rad/s` |
| Motor stiffness 5000 | `7.125e-10 mm` | `0` |
| Motor stiffness 10000 | `3.543e-10 mm` | `0` |
| Motor stiffness 20000 (stock) | `6.272 mm` | `1.787 rad/s` |
| Motor stiffness 50000 | `3.359 mm` | `0.924 rad/s` |
| Motor stiffness 100000 | `2.060 mm` | `0.505 rad/s` |
| Motor stiffness 1000000 | `0.229 mm` | `0.0548 rad/s` |
| Increased rotor damping | `8.079 mm` | `3.913 rad/s` |
| Damping sign flip, i1 | `11.361 mm` | `1.418 rad/s` |
| Damping sign flip, i2 | `170.637 mm` | `2.196 rad/s` |
| Remove motor pseudo-stiffness | reproduced baseline `6.272 mm` | `1.787 rad/s` |

Evidence: `sweeps/param-matrix.jsonl`, `candidate-rotor-damping/`,
`no-pseudo-stiffness/`, `policy-matrix/sign_*`.
Outcome: pseudo-stiffness is not the sole cause; damping magnitude and rotor
coupling change the amplitude but do not remove the two-pass order effect.

### E04 - per-iteration geometry trace

Instrumented native trace at step 7500 (`trace-i2.stderr`; trace build artifact
`8f3c3b2c...`, not a production artifact):

- iteration 0 displaced the effector `0.379063 mm`;
- iteration 1 (reverse order) displaced it another `0.284514 mm`
  (`0.751 x` iteration 0);
- net pose change after both passes was `0.151241 mm`
  (`0.399 x` iteration 0).

The projection does not contract monotonically. Velocity reconstruction from
the biased final pose injects residual velocity every external step. Position
autocorrelation of the late window peaks at lag 341 steps (`0.682 s`,
correlation `0.866`); spectral energy is spread across roughly 19-45 Hz.
Evidence: `joint-trace/`, `new-trace-i2.stderr`,
`baseline-i1-i2/wobble-s1_i2.samples.csv`.

### E05 - damping-cache fix, fixture verification

Same fixture, default one external 2 ms step unless stated.

| Setup | Late diagonal | Late max omega |
| --- | ---: | ---: |
| Cache `1 x 2 ms`, i2 (default) | `1.19275e-10 mm` | `0` |
| Cache `2 x 1 ms`, i2 | `1.38641e-9 mm` | `0` |
| Cache `1 x 2 ms`, i1 | `7.40795e-10 mm` | `0` |
| Cache `1 x 2 ms`, i3 | `0.512348 mm` | `0.472138 rad/s` |
| Cache `1 x 2 ms`, i4 | `3.82642e-10 mm` | `0` |
| Cache `1 x 2 ms`, i8 | `0.00276242 mm` | `0.00690564 rad/s` |

The i3 residual is the remaining first-visit traversal-order limitation, not a
return to the pre-fix limit cycle. Fixed RRD: `wobble-damping-cache.rrd`,
SHA-256
`229663f079530af0fd020a95fbeb3c5f08b2f9548af10d0ccb6a165f7314af9a`.
Screenshot: `wobble-damping-cache-view.png`. Metrics: `final-cache/`,
`recheck/`. Outcome: supported.

### E06 - global-snapshot control

A prototype that evaluated damping from one global beginning-of-substep
snapshot was worse than the first-visit cache:

| Setup | Late diagonal | Late max omega |
| --- | ---: | ---: |
| Snapshot `1 x 2 ms`, i2 | `0.120463 mm` | `0.122418 rad/s` |
| Snapshot `1 x 2 ms`, i4 | `0.124975 mm` | `0.177678 rad/s` |
| Snapshot `1 x 2 ms`, i8 | `0.268322 mm` | `0.152968 rad/s` |

Evidence: `snapshot/`. Outcome: rejected as a replacement for the first-visit
cache.

### E07 - force-trial deadline correction

The loaded native run at nominal 25x spends more wall time than the old fixed
120 s deadline per 30 s trial because effective solver throughput is far below
25x during force tuning. The old fixed deadline aborted valid trials. The new
code projects a deadline from the observed simulation-clock rate, keeps a hard
600 s cap for simulation runs, and separately aborts if the simulation clock
stops advancing. Hardware timing retains the original fixed wall deadline.
Headless advances are chunked to at most 500 ms to stay below the HTTP client's
300 s body timeout. Files: `autocal/control/primitives/simulation_deadline.mjs`,
`force_tuning.mjs`, `uncalibrated_actions.mjs`, `sweep_data_collection.mjs`,
`headless_bridge.mjs`.

Failed trials preserved: `autocal-default/` (`Force trial exceeded its
wall-clock deadline`), `acceptance-default/` (`Physics step budget exhausted`
at 600000 steps), `acceptance-seeded-default-s1/` and
`acceptance-seeded-explicit-s2/` (collector exit 1, pre-fix deadline).
Outcome: supported as a timing fix; it does not by itself prove calibration
accuracy.

### E08 - headless acceptance attempts

Command shape:

```text
autocal/autocal.py --sim --headless-sim --machine-type hangprinter_4 \
  --speedup 25 --dataset <session>/<job>/sweeps.json \
  --max-steps 3 --recording-mode events
```

The seed is `bootstrap-first3/sweeps.json`, SHA-256
`92b69ecdc03e7857d44f3e9dd859383dd7f41af83a9c18fcdd1091eeaa76d88c`
(three sweeps, unlayered, tuned force set). Requested/effective substeps are
recorded in `sweeps.headless/clock.json`.

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

The clean default run reused the seed's force tuning, so it did not re-test
`force_max` selection. No complete end-to-end default-one-substep autocal run
exists yet. This is the main limitation of the session.

### E09 - validation

- `cargo check` passed.
- `npm run build:native -- --no-pgo` and `npm run build:wasm` passed.
- Python physics/differential suites: `180 passed`.
- `autocal/tests/test_headless_sim.py`: `37 passed, 2 deselected`.
- Focused Jest suites: `43 passed`.
- `node --test tests/benchmark/hp4_stability.test.mjs`: `2 passed` on the
  final tree (`4511 ms` default one-substep case, `25775 ms` opt-in substeps).
- `scripts/research_socket_probe.py`: passed.
- `research_preflight`: collector/firmware/recorder/capabilities passed;
  attached-world settings blocked by source mismatch, expected for the
  autonomous CLI.

## Prior art and novelty

Upstream `main` commit `bf9f5399` (`Fix HP4 XPBD damping feedback and
simulation waits`, equivalent to `7f7248d5`) already contains essentially the
same damping cache and deadline ideas. This branch's contribution is the exact
substep policy, broader cross-backend tests, the frozen replay fixture and RRD
evidence, and the acceptance instrumentation. Do not claim the damping cache
itself as novel.

## Next decision

1. Finish a clean default one-substep headless acceptance with a fresh
   `--max-steps 3` run and a final `manifest.json`/`clock.json`
   (`status=complete`, `stop_reason=max-steps`, `recording_drained=true`).
2. Run the same seed with explicit `--physics-substeps 2` in a new directory.
3. Run at least one acceptance that does not reuse force tuning, to exercise
   `force_max` selection with the fix.
4. When the user's target dataset exists, repeat the two-substep 25x headless
   run against it.

Until those finish, the end-to-end autocal claim remains untested, not
supported.
