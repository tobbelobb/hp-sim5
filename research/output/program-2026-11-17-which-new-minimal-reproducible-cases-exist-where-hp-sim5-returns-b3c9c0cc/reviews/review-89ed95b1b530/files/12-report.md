# Cross-backend and state-lifecycle correctness (Wasm vs native)

Revision `c2450975` (main), Node 24.14.1 / V8 13.6, i7-12700K, Linux x64. Native
artifact `physics.node` sha256 `1c137e60…418b` (release, `x86_64-unknown-linux-gnu`,
no PGO); Wasm artifact `cable_solver.wasm` sha256 `db64c87a…1a78` (222,218 bytes).
Exact versions in [environment.json](environment.json).

## Question

In one Node process: (1) are two headless worlds independent, (2) does a
reset/reconfigure reproduce a newly constructed world, (3) is the same input
bit-identical across repeats and process restarts, and (4) how far apart are
native and Wasm on the same scene and step sequence — is the difference
"small rounding" or qualitative (topology, encoder counts, sign, divergence)?

## Method

`hp-sim-3d/app/headlessWorld.js` is the production scene-construction path
(bake → parse → entity plan → `applyEntityPlan` → production system
registration: `RemoteSpoolSystem` → `RustPhysicsSystem` → `ExtruderSystem`).
All scripts use it, and compare state with [lib.mjs](lib.mjs), which serializes
*every* component field of *every* entity in ascending entity-ID order plus every
scalar/vector/map resource (host objects such as the backend, performance
monitor, recorder and canvas are excluded). "Exactly equal" below means the
serialized entry lists are identical number-for-number (`Object.is`); NaN slots
that are NaN on both sides are skipped, ±Infinity must match.

Motion comes from the bundled production RRF/CAN schedules
(`public/RRF_CAN_commands/Hangprinter_logo6_hp{3,4}_*.can`) decoded to one
`Move` per 2 ms step exactly as `tests/benchmark/hp4_logo_commands.mjs` does
([logoCommands.mjs](logoCommands.mjs)). One synthetic schedule is also used
(sinusoidal Moves, no randomness) for the determinism repeats.

Two backends are exercised everywhere: the native Node-API addon and
`WebAssembly.Instance` of the checked-in Wasm module (the same instance path the
browser uses).

## Finding 1 — worlds are independent (negative result)

Command: `node repro-independence.mjs` (0.3 s, exit 0).

Three worlds from the same HP3 text are identical at construction (1338
serialized entries). After stepping only A for 50 steps (including the cable
topology transition at step 1), B is still **exactly** equal to a never-stepped
fresh world C (1338 entries, zero differences). Stepping B and C afterwards
keeps all three exactly equal (1388 entries). The same holds when an HP4 world is
stepped while two HP3 worlds are alive, when two worlds are stepped interleaved,
when one backend is disposed, and for both native and Wasm.

Why this is expected, and what could have broken it: `NativePhysicsEngine` has a
module-level `modules` map, but it caches only the addon handle + build hashes
(`src/js/cable_joints_3d/native_physics_engine.js:6`); each `create()` allocates a
separate Rust world. `WasmPhysicsEngine` constructs a new `WebAssembly.Instance`
per engine, so each world has its own linear memory
(`wasm_physics_engine.js:16`). The Rust crate has no `static`/`thread_local`
physics state (`grep` over `src/wasm/cable_solver/src/*.rs` finds none); all state
is inside the `Box<PhysicsWorld>` returned by `engine_create`. The only
module-level JavaScript binding in the physics path is `defaultModule`
(`wasm_physics_engine.js:5`), which is a module handle, not per-world state.

This is a negative result: no shared view, cached buffer or default world leaks
between live worlds, including with cross-scene lifetimes and disposal.

## Finding 2 — reset/rebuild reproduces a fresh world in production order (negative)

Command: `node repro-reset.mjs` (0.6 s, exit 0).

Reset in the app is `world.clear()` followed by re-applying the scene plan
(`setupScene` with `append=false`). On both backends, for HP3:

- a world stepped 137 steps, then rebuilt, is exactly equal to a fresh unstepped
  world (1316 entity fields);
- after 500 further steps it is exactly equal to a fresh world's 500-step
  trajectory (1365 fields), repeatedly for 3 rebuild cycles;
- a backend object reused after an HP4 world reproduces a fresh HP3 world;
- re-baking and rebuilding from unlayered→layered and layered→unlayered, in the
  app's order, exactly reproduces the corresponding fresh world;
- `setSolverIterations(8)→null` and layering off→on before any step leaves both
  the initial state and a 200-step trajectory untouched.

The only resources that differ after a rebuild are the intentional lifecycle
counters `sceneGeneration` and `cableHybridTransitionStep` (the latter only
numbers trace events; `grep` shows no physics read). No entity leak, no
monotone accumulation over repeated resets, no stale path/joint state.

## Finding 2b — applying a scene plan is not self-contained (genuine, latent)

Command: `node repro-plan-ambient-flag.mjs` (exit **1**).

A `CablePathComponent`'s initial hybrid-knot angle is derived from `path.stored`
using the **ambient** resource at apply time:

```js
// src/js/cable_joints_3d/cable_joints_core.js:1199
const pathHalfWidth = layeringEnabled(world) ? (path.cableHalfWidth ?? 0.0) : 0.0;
const thetaAtState = Math.abs(_storedToThetaSigned(path.stored[endpointIndex] ?? 0.0,
  baseRadius, pathHalfWidth, rampLength));
```

The plan that produced `stored` (baked with `cablePathHalfWidthOverride` and
`deriveAll`) is not consulted. So the identical layered plan (HP3 cable half
width 0.001 m), applied to two worlds that end in the identical configuration
(`enableLayering = true`), produces different initial physical state depending
only on whether the flag was true or false *at the moment of application*:

| | apply-then-flag (wrong order) | flag-then-apply (production) |
| --- | ---: | ---: |
| `e0.HybridKnotAngleComponent.pathAngles.35` | −1.1299124550083235 rad | −2.933692419514937 rad |
| body position difference after 500 steps | **7.199e-2 m** | 0 m |
| body position difference after 2000 steps | **7.044e-2 m** | — |

Error magnitude: 1.804 rad in the knot angle and ~7 cm in body position; it does
not decay. This is not a rounding artifact and not an intended simplification: the
same plan and same final resource value should define one world. It is a genuine
invariant violation in `applyEntityPlan` semantics, but it is **not reachable from
the shipped UI**: `featureFlagsController.setLineLayeringEnabledState` sets the
resource first and only then calls `onLineLayeringChanged` →
`runtime.reset` → `rebuildScene` → `refreshBakedStage` (which bakes with the
new flag) → `setupScene`
(`hp-sim-3d/app/featureFlagsController.js:67-87`,
`hp-sim-3d/app/commandJobController.js:161-172`,
`hp-sim-3d/app/machineSceneController.js:214-226,362-372`). A plain reset keeps
the flag unchanged. `createHeadlessWorld` is also safe because it bakes the
unlayered variant with half width 0 and layered with the default resource value
(true). The bug is reachable by any API caller that caches/reuses a plan, changes
layering between bake and apply, or appends a layered scene to a world whose flag
is off. `node probe-layering-order.mjs` records both the production-order pass
and this measured failure.

## Finding 3 — determinism is exact (negative result)

Command: `node repro-determinism.mjs` (5.0 s, exit 0).

1500 commanded steps (2 substeps, sinusoidal Moves) per backend and machine:
three fresh worlds in one process and two fresh OS processes produce the same
state digest.

| backend/machine | in-process ×3 | two process restarts |
| --- | --- | --- |
| native / HP3 | `8ef96148cd0a9f40` | `8ef96148cd0a9f40` |
| native / HP4 | `3e8ecce1d4576181` | `3e8ecce1d4576181` |
| wasm / HP3 | `f203cc790ad128ff` | `f203cc790ad128ff` |
| wasm / HP4 | `8be5cce007cbfe18` | `8be5cce007cbfe18` |

Native and Wasm digests differ from each other (expected); each is internally
exact. No wall-clock, random or iteration-order dependence was found.

## Finding 4 — Wasm/native difference: small per-step, large transient bursts

Command: `node repro-parity.mjs` (~28 s, exit 0; [results.json](results.json),
console log [repro-parity.log](repro-parity.log)); long run:
`node run-long-hp4.mjs` (≈43 s, writes [long-hp4-100k.json](long-hp4-100k.json)).

All cases start from **bit-identical prepared records** (max diff `0`, same record
lengths 2720 for HP3 and 9440 for HP4) and run identical commands. Every case has
identical topology versions (1), identical cable joint counts (18 HP3 / 51 HP4),
identical missed-step counters (all 0) and no non-finite state. The 20 k cases
have no quaternion anti-podal representation differences; the 100 k run records
30 sampled orientations as q vs −q (the same orientation — the angle metric below
uses |q·q′|).

| case | steps | max body position diff | max encoder diff | max quaternion angle | end position / encoder |
| --- | ---: | ---: | ---: | ---: | ---: |
| hp3 settle, layered | 2 000 | 8.2e-15 m | 2.5e-13 rad | 0 | 4.5e-15 m / 1.1e-17 rad |
| hp3 logo, layering off | 20 000 | 8.5e-14 m | 5.8e-13 rad | 4.2e-8 rad | 6.3e-14 m / 1.5e-14 rad |
| hp4 logo, layered | 20 000 | **6.90e-4 m** | **8.25e-2 rad** | 8.25e-2 rad | 3.4e-9 m / 6.7e-7 rad |
| hp4 logo, closed loop | 20 000 | **4.18e-4 m** | **5.32e-2 rad** | 5.31e-2 rad | 4.0e-8 m / 1.1e-5 rad |
| hp4 logo, layered | 100 000 | **1.238e-3 m** | **1.425e-1 rad** | 1.426e-1 rad | 1.1e-7 m / 1.45e-5 rad |

The HP4 "max" is not a slow drift: it is a burst during fast logo transients
(step 22 300 in the 100 k run; 18 400 and 13 200 in the 20 k runs). 34–36 of the
~200 sampled steps exceed 1e-4 rad, always followed by a return to 1e-7…1e-9 m.
End-of-run differences are 1e-9…1e-7 m. HP3 has no such bursts in the cases measured
(≤2.7e-13 m and ≤5.8e-13 rad over 100 k steps of the unlayered HP3 logo
schedule, [long-hp3-off-100k.json](long-hp3-off-100k.json)).

The bursts are the trajectory's own sensitivity, not a backend equation
difference:

- **One-step fork** (`oneStepFork` in [parity.mjs](parity.mjs)): copy the native
  entity *and* path records into Wasm before every step, so feedback cannot
  accumulate. Over 500 steps the remaining per-step difference is at most
  1.6e-10 in the entity record (on constraint-force slots whose scale is ~1e2 N,
  i.e. ~1e-12 relative), 1.9e-12 in path records and 3.8e-13 rad on encoders
  (HP3: 1.0e-12 / 2.4e-12 / 1.3e-15). That is the expected size of libm/rounding
  differences.
- **Perturbation scaling** (HP4 logo, native control vs native twins perturbed by
  1e-15, 1e-12 and 1e-9 m/s on the fastest body at step 1): at step 300 all three
  twins are already 1.8–2.6e-9 rad apart — the perturbation size no longer
  matters; by step 500 all three are 3.8e-3…1.2e-2 rad apart. The burst saturates
  at a system-determined size regardless of how the tiny difference was seeded.
- **Growth rate**: during the step 200→500 burst onset the wasm-vs-native encoder
  difference grows with λ = 0.0772 per 2 ms step (36 s⁻¹, doubling every 9.0 steps); the native twins grow at λ = 0.0728–0.0775 (doubling 9.0–9.5 steps).
  The cross-backend curve follows one of the native perturbation curves. Over the
  whole 100 k run, maxima do not grow monotonically (0.1425 rad at step 22 300,
  0.0899 at 52 300, 0.0825 at 18 400), so there is no sustained positive
  Lyapunov exponent over this window; the large errors are finite-time bursts
  that the driven motion re-converges.
- A native twin perturbed by only 1e-12 reproduces the cross-backend maxima:
  0.1427 rad / 1.189 mm vs 0.1425 rad / 1.238 mm over 100 k steps.

**Classification.** The per-step operator difference is small rounding
(≤1.7e-10 absolute, ~1e-12 relative, ≤4e-13 rad encoder), and any sufficiently
small difference — including one inside a single backend — is amplified by the
HP4 fast-transient dynamics to 0.05–0.14 rad and 0.4–1.2 mm before re-converging.
So this is a *documented limitation* ("small rounding differences are expected";
"Long-run numerical drift and equivalent full-autocal outcomes are not CI acceptance criteria",
`src/wasm/cable_solver/README.md`, `research/rust-native-engine.md:70`), not a
qualitative backend inconsistency. The word "small" is misleading for
*instantaneous* observations during those transients: 0.14 rad is 8.2°, and
1.2 mm is above typical print-feature scales. It is not a sign flip, not a
different topology, not different encoder counts, and not divergence.

## Strongest alternative explanations

- **Finding 2b could be "unsupported usage".** The resource is arguably part of
  the application context, and the app always sets it before re-baking, so the
  invariant may be intentionally implicit. Against that: the same plan and the
  same final resource value yield a 7 cm different physical world, and nothing in
  `buildEntityPlan`/`applyEntityPlan` documents the ordering requirement or
  carries the bake layering setting. Calling it latent rather than user-visible
  is the honest middle position.
- **Finding 4 could be a real accuracy defect rather than chaos.** If the Rust
  trig implementations differed by more than rounding, the one-step fork would
  show it: re-synchronizing every step leaves ≤4e-13 rad on encoders and ~1e-10
  in stiff force slots. The perturbation series also shows the burst size is
  independent of the seed magnitude (1e-15…1e-9), and the cross-backend growth
  rate matches the native-twin growth rate. A systematic multi-ULP libm bias at a
  scale larger than 1e-12 relative is not supported by these measurements; a
  residual systematic difference below that scale cannot be excluded.
- **The hp3 "no divergence" result could be hiding a topology difference.** It is
  not: topology version and joint counts are compared every step, and both stay
  equal (1, 18) for 100 k steps.

## Negative results

- No shared state between live worlds, in either backend (including interleaved
  stepping, mixed scenes and disposal).
- No reset/rebuild leak in the production ordering (full entity arrays, 3 cycles,
  backend reuse, layering reconfigure, flag cycling).
- No nondeterminism in-process or across process restarts.
- No qualitative Wasm/native difference: no topology or joint-count divergence,
  no missed-step/encoder-count divergence, no non-finite state, no orientation
  difference hidden by a quaternion sign change (30 anti-podal *representations*
  in the 100 k run, same physical orientation), no monotone divergence.

## Limitations

- All evidence is simulation. Nothing here validates hardware or calibration.
- Scenes are the bundled HP3/HP4 designs and their bundled CAN logo schedules;
  HP5, cubecorners, autocal traces, browser engines and the DOM/render path were
  not exercised. Wasm ran inside Node's V8, not a browser.
- Peak differences are sampled every 50–100 steps; true peaks between samples may
  be higher. The 100 k run is a single run per backend.
- The perturbed twin uses one field/entity/step (velocity x of the fastest rigid
  body at step 1). Other seeds could behave differently, although the
  perturbation-magnitude scan makes a magnitude-dependent explanation unlikely.
- Independence and reset are judged on observable ECS + resource state.
  Non-enumerable or module-internal state that never manifests in observables
  would not be detected.
- Cross-process determinism ran on one host with one Node/V8 build; cross-host or
  cross-CPU bit-identity is not claimed.
- `probe-layering-order.mjs` reproduces the app's ordering by calling the same
  functions, not by driving the browser UI.

## Reproduction

Run from this directory (each ≤60 s; no rebuilds, no source changes):

```bash
node repro-independence.mjs          # exit 0
node repro-reset.mjs                 # exit 0
node repro-determinism.mjs           # exit 0
node repro-parity.mjs                # exit 0, writes results.json (26 s)
node run-long-hp4.mjs                # exit 0, writes long-hp4-100k.json (43 s)
node repro-plan-ambient-flag.mjs     # exit 1 (Finding 2b)
node probe-layering-order.mjs        # exit 0, prints both orderings
```

Evidence: [results.json](results.json) (all parity cases, forks, perturbation
series), [long-hp4-100k.json](long-hp4-100k.json),
[long-hp3-off-100k.json](long-hp3-off-100k.json), the `*.log` files next to each
script, [environment.json](environment.json), shared harness
[lib.mjs](lib.mjs), parity measurement [parity.mjs](parity.mjs), command decoder
[logoCommands.mjs](logoCommands.mjs).
