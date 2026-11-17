# Probing the Rust cable/physics core for wrong results

**Question.** Does the Rust cable/physics core (`src/wasm/cable_solver/src`, plus the
JS/native binding in `src/js/cable_joints_3d/rust_physics_engine.js` and
`native_physics_engine.js`) return genuinely wrong results for (1) degenerate but
in-domain cable geometry, (2) non-finite propagation, (3) tension sign/magnitude on
taut vs slack lines, (4) constraint residuals where a closed form exists, or
(5) world isolation and reset?

**Answer in one line.** On this revision (c2450975, native artifact
`1c137e6…`) the closed-form and degenerate-geometry cases I could construct are
numerically exact, and worlds are isolated and deterministic. I found two
state-poisoning bugs: a single non-finite *input* coordinate silently and
permanently NaNs the whole cable network while reporting a healthy 0 N (default
flags), and the opt-in layering `split` pass writes NaN into the persistent path
records when a rolling link has no finite radius (an input the always-on
attachment kernel explicitly tolerates, but a feature the app leaves disabled).

Everything below is numerical evidence about the simulator. No hardware claim is
made.

---

## Method

* **Harness.** `lib.mjs` builds persistent entity records (160 f64) and path records
  (`9 + 5·(n+1) + 16·n` f64) exactly as `src/wasm/cable_solver/tests/common/mod.rs`
  and `rust_physics_engine.js` do, then calls the same native entry points the app
  uses (`addon.create(...)`, `step(dt)`, `path(i)`, `options()`). One script per case;
  each is < 1 s.
* **Flags.** Unit-level worlds start from the crate-test defaults: gravity
  `-9.81 ŷ`, attach updates on, rest-length clamp on, layering off. Layering /
  merge / split / hybrid are switched per case.
* **Closed-form references.** Atwood hairpin `a = g(m₁−m₂)/(m₁+m₂)`,
  `T = m₁(g−a)`; hanging statics `Δ = mgL/EA`, `T = mg`; spool winding
  `Δy = r·Δθ`; rigid residual `|d − rest| = 0`.
* **Invariants.** Finite pose/quaternion/ω and path fields; tension ≥ 0; material
  conservation `Σ rest + Σ stored = totalRestLength`; bit-identical behaviour of
  independent worlds; fuzz over 3000 randomised minimal worlds.
* **Repro discipline.** The two bug scripts exit **1** with an assertion message
  when the bug is present. The two negative-result scripts exit **0** only when
  every invariant holds.

Commands (run from this directory):

```
node repro_nan_poisoning.mjs          # exit 1 (bug present)
node repro_split_nan_radius.mjs       # exit 1 (bug present)
node check_degenerate_ok.mjs          # exit 0
node check_production_scenes.mjs 300  # exit 0
```

Captured output is in `evidence/results_*.txt`; measured values are also in
`results.json`.

---

## Finding 1 — one non-finite input coordinate poisons the whole cable system, silently and permanently

**Repro:** `repro_nan_poisoning.mjs` (exits 1; `node repro_nan_poisoning.mjs inf`
does the same with `Infinity`).

World: a fixed roller (r = 0.05 m, no spin) with two 1 kg masses in a hairpin — a
valid configuration that settles to `y₁ = −1.000009810 m`, `T = 9.810000 N`.
Then one mass's `position.x` is set to NaN (exactly what a bad scene write or a
JS-side divide-by-zero leaves behind) and the world is stepped once.

| quantity | expected | actual after one step |
|---|---|---|
| bodies with non-finite pose | only the bad body (or none) | **2 of 3**; the fixed roller's quaternion (4 slots) and ω (3 slots) are NaN and the bad mass's position/prev-position/velocity are NaN |
| path segment rest lengths | unchanged, 1.0 m | **NaN, NaN** (both segments, including the segment that does not touch the bad body) |
| stored wrap arc | unchanged, 0.157… m | **NaN** |
| `constraintLambda` / `constraintForceMagnitude` | NaN or an error | **0 / 0 N** — no signal at all |

Repairing the input (`position.x = −0.050000001`) and stepping again leaves
**3 of 3** bodies and both rest lengths NaN. The world is unrecoverable without a
rebuild.

**Error magnitude and tolerance.** Every pose/velocity/path value is expected to be
finite; 2→3 bodies and 3 path fields are not. The reported cable force is 0 N, so
nothing in the telemetry flags the failure.

**Why this is a bug, not an intended simplification.** The core is written to
contain non-finite values elsewhere: `load()` refuses non-finite torque,
`update_attachments` skips non-finite rest/radius, `servo`/`encoders`/`diagnostics`
check `is_finite`, and the Node bridge rejects non-finite `dt`. Here the NaN
survives into the correction and material-transport passes. Consistent with the
measurements: `lambda()` clamps its step with Rust `f64::min`
(`src/wasm/cable_solver/src/lib.rs:810,817`), and `0.0f64.min(NaN) == 0.0`, so the
reported force is exactly 0 while `correct()` (`lib.rs:867`) computes
`0 · NaN = NaN` and writes it into the body; `transport`/`turn_wheel` and
`update_attachments` then write NaN into the path. The damage is silent *and*
spreads to bodies that never held the bad value.

**Strongest alternative explanation.** Garbage-in-garbage-out: a NaN position is
an invalid input that the JS adapter should have prevented, and the native bridge
does not promise input validation. I still count it as a bug because (a) the NaN
spreads to unrelated bodies and path fields, (b) the adapter mirrors these buffers
straight into ECS components, so one bad write corrupts the whole session's state
and telemetry, (c) the reported force is a plausible-looking 0 N instead of NaN,
and (d) the core demonstrably tries to contain non-finite values everywhere else.

---

## Finding 2 — `layeringSplitJoints` writes NaN path records for a rolling link without a finite radius

**Repro:** `repro_split_nan_radius.mjs` (exits 1).

Setup: a straight span from a fixed point (0,0) to a mass (1,0), declared as two
`rolling` links, but neither endpoint has a `RadiusComponent` (the adapter writes
`radius = NaN`). A bystander roller (r = 0.05 m) at (0.3, 0) is crossed by the
span. Flags: `enableLayering = layeringSplitJoints = true` (merge/hybrid off).

| | split off (control) | split on |
|---|---|---|
| segments | 1 | 2 |
| stored arcs (m) | [0, 0] | **[NaN, NaN, NaN]** |
| rest lengths (m) | [1] | **[NaN, NaN]** |
| poses finite | [true, true, true] | **[false, true, false]** |
| mass y (m) | −0.00053955 | **NaN** |

**Error magnitude and tolerance.** All 5 path fields (3 stored arcs + 2 rest
lengths) and 2 of 3 bodies are non-finite; expected zero non-finite values. Every
following step stays NaN.

**Why this is a bug, not a simplification.** Missing radii are a supported input in
the always-on geometry path: `geometry()` packs `radii[side].0.is_finite()` as a
flag (`src/wasm/cable_solver/src/engine.rs:487`) and `rolling_attachment` degrades
to the centre, while `update_attachments` guards with `radius.is_finite()`
(`engine.rs:928`) and `capstan` treats `radius ≤ EPS` as zero wrap
(`engine.rs:2356`). `split()` (`engine.rs:1283`) has no such guard: it feeds the
NaN radius to `sphere_tangents`/`arc` (`engine.rs:1319,1342`) and writes the NaN
tangent points, wrap angle and rest lengths back into the persistent records, from
where the always-on constraint solve picks them up.

**Strongest alternative explanation.** The input is malformed and unreachable in
production: the scene baker computes rolling stored lengths from
`RadiusComponent` and would throw, and the app sends
`layeringSplitJoints = false`. I verified this on hp4 through the real adapter —
after one step `options[3..8] = [1, 1, 1, 0, 0, 1]`, i.e. layering and hybrid on,
merge/split off. So this is a latent bug in a disabled feature, not a
mis-simulation of a shipped scene. I report it as a real but dormant defect
because the core accepts the input elsewhere and destroys the world instead of
ignoring the link.

Supporting sweep: 0/3000 randomised worlds with domain-valid link types failed
(`node explorations/explore11.mjs`); 195/3000 worlds containing a rolling link on
a radius-less entity failed the same way with split enabled
(`MALFORMED=1 node explorations/explore11.mjs`).

---

## Finding 3 (limitation) — negative rest length becomes 1e-6 m and 19.6 kN, while `totalRestLength` keeps −1

`rest = −1` m for a 1 kg hanging mass: after one step the stored rest length is
`1.000e-6 m`, the reported tension is **19607.827 N**, and the mass sits at
−0.9804 m. The path's `totalRestLength` field stays **−1** while the segments sum
to **+1e-6 m** (the clamp creates 1 µm of material).

This is the `layeringClampJointRestLength` feature (on by default) sanitising an
invalid input, so I do not count it as a bug — but it is a silent repair with an
absurd force, and the `totalRestLength` telemetry becomes inconsistent with the
segment sum. No valid scene can construct a negative rest length.

---

## Finding 4 (reporting quirk) — a zero-length segment in a taut merged row reports tension with a zero force vector

Duplicate path points (`anchor A`, `anchor B` at the same point, rest 0) merged
with a taut span to a 1 kg mass: both segments report
`constraintForceMagnitude = 9.810 N`, but the zero-length segment's
`constraintForce` is `(0, 0, 0)` — it has no direction, so the vector cannot carry
that load. Dynamics and material bookkeeping are correct and stable for 4000
steps; only the scalar is misleading on that degenerate segment.

---

## Negative results

Every item below was checked against a closed form or an invariant and passed.

| case | reference | measured | error / tolerance |
|---|---|---|---|
| Atwood hairpin, r = 0.05, 0.02, 1e-3, 1e-9, **0** | a = g/3 = 3.270000000 m/s² | 3.270000000 | ≤ 2.4e-10 (tol 1e-6) |
| same, tension | T = 13.080000 N | 13.080000 | ≤ 4e-10 (tol 1.3e-2) |
| hairpin mirrored (heavy side left vs right) | identical | identical | 1.1e-13 m (tol 1e-12) |
| hanging mass, EA = 20 kN, m = 10 kg | Δ = mgL/EA = 4.905e-3 m, T = 98.1 N | 4.929224e-3 m, 98.099975 N | +0.49 % (tol 2 %), 2.5e-5 N |
| slack line (rest 1.05 > span 1.0) | T = 0 | exactly 0 | exact |
| spool payout, r = 0.05 / 0.02, ±Δθ | Δy = r·Δθ, Δstored = r·Δθ | matches | 0.05 % (tol 1 %), < 1e-12 m |
| rigid 1-segment residual (stiffness = ∞) | 0 | exactly 0 after 1 iteration | exact |
| rigid 2-segment chain residual after 8 iterations | → 0 | 6.54e-7 m | Gauss–Seidel, expected |
| tension convention | λ < 0 for tension, force on entityB | λ = −9.81e-5, F = (0, +98.099975, 0) N | as designed |
| coincident anchors, rest 0.5 / 0 | T = 0, no NaN | T = 0, rest 0.5 / lifted to 1e-6 | exact / by clamp |
| zero-rest tether | mass stays at anchor | max offset 1.006e-6 m over 1000 steps | tol 5e-6 m |
| duplicate path points | stable, T = mg | T = 9.81 N for 4000 steps, all finite | exact |
| anchor exactly on the spool surface | Δy = r·Δθ | 0.05 % | tol 1 % |
| two worlds | no shared state | bit-identical (`Object.is`) while the other runs 100 extra steps | exact |
| two identical worlds | deterministic | bit-identical after 200 steps | exact |
| dispose + rebuild | fresh first step | all 56 path fields identical | exact |
| hp3, hp4, hp4_worn_bearings, hp5, slideprinter_single_pinholes × layering on/off, 300 steps | finite, T ≥ 0, material conserved | 0 non-finite, 0 negative T, max relative material drift 1.12e-14 | tol 1e-9 |
| 3000 randomised minimal worlds (valid links, layering/hybrid/split on) | finite, T ≥ 0, material conserved | 0 failures | — |
| uncommanded hp3/hp4 for 12 s | motors hold, no creep | max displacement saturates at 9.31e-5 m (hp3) / 8.97e-6 m (hp4) after 2 s and is constant to the last digit for 12 s | no drift |

The world-lifecycle checks (isolation, determinism, reset) therefore close angle 5
with no finding: two worlds share no state, and rebuilding reproduces a fresh
world exactly.

---

## Limitations / what could change the conclusion

* The native ABI is exercised directly. The ECS/JS adapter path is covered only by
  the production-scene runs, which use the real `NativePhysicsEngine`.
* Finding 1 requires an externally injected non-finite value. I did **not** find a
  finite input in the tested ranges that produces a non-finite output (3000-world
  fuzz over positions, radii, masses, rest lengths, stiffness, damping, iteration
  counts and hybrid/layering flags). If the project's position is "the adapter
  must validate inputs", Finding 1 is a robustness bug rather than a physics bug —
  but the silent 0 N report and the unrecoverable spread are still a defect.
* Finding 2 is unreachable with the app's flags (`layeringSplitJoints = false`)
  and its input is not producible by the scene baker. It is a latent bug in a
  dormant feature.
* Constraint residuals were only measured for one- and two-segment rigid chains;
  long chains need more iterations, which is documented XPBD behaviour.
* Motor/spool torque control (servo, missed steps, worn bearings) was not audited
  beyond the production-scene invariants.
* A slack segment inside a taut merged row is pulled with the row's tension in the
  same step that its material is redistributed (measured 773 N on both ends for an
  initial 0.7 m stretch). That follows the documented row model ("material moves
  freely inside a row"), so I do not count it as a bug; but a badly initialised
  slack span produces a one-step impulse rather than a gradual take-up.
* No hardware evidence is involved anywhere: these are statements about the
  simulator, not about a real Hangprinter.

## Other observation (not reproduced as a bug)

`rust_physics_engine.js:278` sends `options[15] = world.getResource('dt') ?? -1`.
The Rust core uses any non-NaN `options[15]` as the solve timestep, so a missing
`dt` resource (−1) leaves positions constrained but reports every cable force as
exactly **0 N** (measured: 0 N vs 155.8 N, with the mass still held at
y = −1.000000). Not reachable through `createHeadlessWorld`, which always sets
`dt`, but it is another silent-zero path.

## Evidence index

| file | purpose |
|---|---|
| `lib.mjs` | entity/path builders + native addon loader |
| `repro_nan_poisoning.mjs` | Finding 1 (exit 1 on bug) |
| `repro_split_nan_radius.mjs` | Finding 2 (exit 1 on bug) |
| `check_degenerate_ok.mjs` | negative-result suite, closed-form + degenerate cases (exit 0 = all pass) |
| `check_production_scenes.mjs` | negative-result suite over five real scenes (exit 0 = all pass) |
| `explorations/` | supporting scripts (fuzz, geometry sweeps, diagnostics) |
| `evidence/results_*.txt` | captured outputs of the four canonical scripts |
| `evidence/trial18.json` | dumped randomised configuration that exposed the split NaN |
| `results.json` | all measured numbers, classifications and alternative explanations |
