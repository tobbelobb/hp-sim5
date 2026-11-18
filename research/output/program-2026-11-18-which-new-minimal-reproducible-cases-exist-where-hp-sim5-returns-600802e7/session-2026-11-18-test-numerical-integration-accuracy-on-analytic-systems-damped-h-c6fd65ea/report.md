# Numerical integration accuracy on analytic systems (hp-sim5)

Session: `session-2026-11-18-test-numerical-integration-accuracy-on-analytic-systems-damped-h-c6fd65ea`
Program: `program-2026-11-18-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-600802e7`
Date: 2026-11-18. Checkout: `74ecdc95`. Node 26.13.0, Linux x64.
Engine under test: checked-in `public/wasm/cable_solver.wasm` (ABI 2), driven through
the production `createHeadlessWorld` pipeline. No native addon, no build, no Python path.

**Post-rebase addendum (2026-11-18).** Re-run on `e4785901`. The new strict USD loader
ignores xform ops that are not declared in `xformOpOrder`, so the synthetic prims in
`common.mjs` now declare `uniform token[] xformOpOrder = ["xformOp:translate"]` and use
3-component translations. Results are unchanged (+33.33 %/+38.96 % Atwood; 14.19 deg/
27.25 deg gyroscopic). "No integration defect" in the verdict below refers to time
integration; the passive-wheel-inertia and gyroscopic-term gaps are separate
model-fidelity findings.

## Verdict

**No new numerical-integration defect.** The solver is a first-order
semi-implicit/PBD integrator, exactly as `hp-sim-3d/README.md` describes, and
every analytic system matches its closed form at the documented defaults to the
error expected from that scheme. There is no divergence, energy gain, or
instability on the tested range.

Two **latent model-fidelity gaps** were found and quantified (these are not
time-integration errors):

1. A passive (non-motor) wheel's authored `physics:inertiaTensor` is ignored.
   An Atwood machine with a finite-inertia pulley accelerates as if the pulley
   were massless: +33.3 % (hoop-like pulley), +39.0 % (the inertia of the four
   HP4 D-line idlers) vs the textbook closed form. The crate test documents
   this as a "negligible inertia" idealization, but the shipped HP4 wheels are
   not negligible: each 2 g / 6.5 mm wheel has `I/r² = 0.0118 kg`, 5.9x its own
   mass, and the effector is 0.081 kg.
2. A torque-free asymmetric rigid body holds `omega` constant in the world
   frame instead of integrating Euler's equations. After 0.5 s an asymmetric
   dumbbell is 14.19° from the exact solution and its angular-momentum
   direction has rotated 27.25° with zero torque. All shipped rigid bodies are
   symmetric tops (or spherical), so this is latent in the catalog.

One minor latent API inconsistency: `physicsSubsteps` is silently ignored
unless the world contains a `RigidBodyComponent` (and a cable path). A valid
point-mass cable scene gets identical trajectories at 1 and 16 substeps.

The earlier program's "Atwood hairpin `a=g/3` to 2.4e-10" is correct only for
its fixed, non-rotating roller. It does not establish Atwood-with-inertia
accuracy; see §5.

## Method

* One process, one checked-in Wasm module, minimal authored USDA scenes.
* Defaults read from the scenes, not assumed:
  `hp4` dt 0.002 s, 1 substep, 10 solver iterations; `hp3` 1/1;
  `cubecorners` 1/5; `slideprinter` 1/1 (matches `machines/catalog.json` and
  the README).
* Closed forms:
  * free mass: `x = -g t²/2`, `v = -g t`;
  * cable spring/damper: a span of current length `x` has compliance `x/EA`
    in the engine (`Row.alpha`), so static equilibrium is
    `x_eq = L/(1 - mg/EA)` and the tangent stiffness at equilibrium is
    `k = EA·L/x_eq²`; damping is `c = B/x_eq`. The response is compared to the
    linearized damped oscillator. A constant-`k = EA/L_rest` reference is
    *not* the engine's model; it shifts the soft-case period by ~1 % here
    (and gave the earlier, misleading ~3 % when the test amplitude also
    touched slack).
  * pendulum: `T = 2π√(L/g)(1 + θ0²/16 + 11θ0⁴/3072)`, energy `½v² + g y`.
  * Atwood: `a = (m1-m2)g / (m1 + m2 + I/r²)`; massless limit `I/r² = 0`.
  * free rigid body: Euler equations with the aggregate tensor `I0`,
    `L = R I0 Rᵀω` constant, integrated by RK4 at `h = 2e-5` s (checked against
    `h/2`: identical to shown precision).
* Periods are measured from interpolated zero crossings on every simulation
  step (not every 5th), so interpolation bias is < 1e-4 of a period.
* Tolerances are stated per row. The harness asserts the expected first-order
  behavior; the two repros assert the textbook closed forms.

## 1. Free mass, constant acceleration

A lone 1 kg `Wheel` body, `v0 = 0`, no cable (`engine.rs:684`, `integrate`:
`v += g dt; x += v dt`). Semi-implicit Euler predicts
`v_n = -g n dt` exactly and `x_n = -½ g n(n+1) dt²`, i.e. a position error of
`-½ g t dt` evaluated at `t = n dt`.

| dt (ms) | steps to t=0.12 s | measured x (m) | exact x (m) | error (mm) | velocity error |
|---|---|---|---|---|---|
| 8 | 15 | -0.0753408 | -0.070632 | -4.7088 | 2.2e-16 |
| 4 | 30 | -0.0729864 | -0.070632 | -2.3544 | 3.8e-15 |
| 2 | 60 | -0.0718092 | -0.070632 | -1.1772 | -1.0e-14 |
| 1 | 120 | -0.0712206 | -0.070632 | -0.5886 | 3.8e-14 |
| 0.5 | 240 | -0.0709263 | -0.070632 | -0.2943 | 3.8e-14 |

(The table's x column is rounded; the raw values are in `evidence.json`.) The
measured error equals `-½ g t dt` to <1e-9 m at every dt, and the fitted order
is **1.000000** over a 16x dt range. This is the textbook semi-implicit Euler
result, not a defect.

`physicsSubsteps` 2/4/8/16 at a fixed 2 ms tick reduces the position error
exactly as 1/n: -0.981, -0.490, -0.245, -0.123, -0.061 mm at t=0.1 s. The
velocity stays exact.

**Substep gate (minor):** with a slack cable path but no `RigidBodyComponent`
in the world, substeps 1 and 16 give bit-identical trajectories at t=0.1 s
(`y = -0.05003099999999997` both). `hp-sim-3d/app/simulationSystems.js:19-22`
gates the count on
`query([RigidBodyComponent]).length && query([CablePathComponent]).length`. All
shipped scenes have a rigid body, so this is latent; a valid point-mass cable
scene cannot use the setting.

## 2. Damped harmonic oscillator (cable spring + damper)

One mass hangs from one compliant span; `A` is the initial displacement below
the equilibrium, chosen well under the static stretch so the cable never goes
slack (`minSlack > 0` checked).

**Soft, visible case** (`m=1 kg, L=1 m, EA=1000 N, B=6.3246 N·s`, ζ=0.1):

| dt (ms) | period error vs closed form | max response error (A=1 mm) |
|---|---|---|
| 8 | +3.50 % | 0.285 mm |
| 2 | **+0.745 %** | **0.101 mm** |
| 0.5 | +0.164 % | 0.028 mm |

**Production-like stiff cable** (effector-scale `m=0.081 kg, L=2 m,
EA=20000 N, B=5 N·s`, ζ=0.044, axial mode ≈56 Hz):

| substeps (2 ms tick) | period error | max response error (A=10 µm) |
|---|---|---|
| 1 (default) | **+16.03 %** | 6.78 µm |
| 4 | +1.67 % | 3.82 µm |
| 16 | +0.252 % | 1.46 µm |

Solver iterations 1, 4, 10, 64 are **bit-identical** for a single-row cable
(`maxAbsErr` equal to 1e-12): a single span with one dynamic endpoint is fully
projected in the first iteration. Only substeps improve this mode.

Interpretation: the stiff number is a genuine sampling limit (ω dt ≈ 0.7,
~9 steps per period at 2 ms), not a bug. The documented control (substeps)
reduces it monotonically, and no configuration diverged.

## 3. Pendulum / small angle

A 1 kg point mass on a 1 m, `EA=1e7` cable, `θ0=0.05 rad`, zero initial
velocity. Exact finite-amplitude period 2.006380 s.

| dt (ms) | measured period (s) | error | energy loss over 4.1 s |
|---|---|---|---|
| 8 | 2.00676 | +0.0187 % | 3.388 mJ |
| 2 (default) | 2.00640 | **+0.00075 %** | **0.958 mJ** |
| 0.5 | 2.00638 | -0.00002 % | 0.254 mJ |

The period error shrinks steeply with dt (×25 and ×34 per 4x dt over these
small values) and is <1e-5 relative at the default. The energy loss is first-order in dt (8 ms / 0.5 ms
ratio 13.3, close to 16) and exactly the behavior documented in the Rust
test comments; the cable's radial stretch is 1e-6 m (EA=1e7). Solver
iterations 1 vs 64 are identical. At 2 ms, 16 substeps cut the energy loss
from 0.260 mJ to 0.0254 mJ over 1.05 s (10.2x), again as documented.

## 4. Solver iterations and guide count (controls)

* **Two-cable V**, 1 kg at the apex of two 20 kN cables, 600 steps to settle.
  Static residual vs the converged position: 1 iteration **8.28 µm**,
  2 → 0.370 µm, 4 → 0.74 nm; 8 and 64 agree to <1e-12 m. The documented per-path
  iteration setting is wired and converges; it is simply irrelevant to a
  single-row cable.
* **Guide count:** 0, 1 or 3 fixed frictionless guides on the same straight
  span produce trajectories differing by at most `6e-15 m` from the 0-guide
  run. Multi-segment row handling does not change the dynamics.

## 5. Atwood with inertia (secondary finding)

Hairpin over a fixed wheel, `m1=2 kg`, `m2=1 kg`, cable `EA=1e7`, 8
iterations, 2 ms, acceleration fitted over 0.05–0.3 s. Closed form
`a = (m1-m2)g/(m1+m2+I/r²)`.

| pulley | I/r² (kg) | expected with inertia | measured | error |
|---|---|---|---|---|
| I = 0 | 0 | 3.2700 | 3.2700 | 0 |
| hoop-like, I=2.5e-3, r=0.05 | 1 | 2.4525 | **3.2700** | **+33.33 %** |
| I=1e-2, r=0.05 | 4 | 1.4014 | **3.2700** | **+133 %** |
| one shipped HP4 wheel (I=5e-7, r=6.5 mm), loads 81/40.5 g | 0.01183 | 2.9798 | **3.2700** | **+9.74 %** |
| four HP4 D-line wheels (4×5e-7), same loads | 0.04734 | 2.3532 | **3.2700** | **+38.96 %** |

The measured value is always the massless-pulley `g/3`; the declared inertia
is dead data for a passive wheel. `engine.rs:2274` (`turn_wheel`) sets the wheel
angle from material travel with no inertial reaction, and
`engine.rs:2376` (`passive_wheel`) plus `capstan()` treat non-motor rolling
wheels as ideal. The crate test
`free_wheel_atwood_turns_with_line` states the assumption explicitly, so this
is a **deliberate idealization**, but it is not documented in the user-facing
README and it is quantitatively wrong for the shipped HP4 wheels: 24 wheels,
each `I/r² = 0.0118 kg` (5.9x its own mass), 2 per A/B/C path and 4 per D
path, against an 0.081 kg effector. The effective moving mass along a line is
understated by ~29 % (A/B/C) to ~58 % (D) if those idlers spin with the line.
No shipped trajectory error is claimed; this is a latent fidelity gap, and the
reproducer is a valid authored scene.

Repro: `repro_atwood_wheel_inertia.mjs` (exit 1).

## 6. Freely rotating rigid body (secondary finding)

A free rigid body with no gravity, aggregate tensor
`I0 = diag(0.002, 0.502, 0.502)` kg m² (a dumbbell; `Ixx` is the symmetry
axis), seeded with an angular velocity, 0.5 s at 2 ms.

| initial ω (rad/s) | orientation error vs exact Euler solution | angular-momentum direction change |
|---|---|---|
| (2, 0, 0) — symmetry axis | 0 | 0 |
| (1, 2, 0.5) | **0.24762 rad (14.19°)** | **0.47569 rad (27.25°)** |
| (3, 1, -2) | **0.76227 rad (43.68°)** | **1.39612 rad (80.00°)** |

The engine's orientation matches a constant-world-frame-ω rotation to 0
(the control), and exactly the same trajectories are produced by
`exp(ωt)`, but the exact torque-free solution precesses. The engine conserves
`|L|` and energy (because it rotates about a fixed ω) but not the *direction*
of `L`, which must be constant with zero torque. This is the missing
`ω × (Iω)` gyroscopic term; it is a known PBD simplification, not an
integration-order error.

Reachability: all shipped rigid bodies are symmetric tops about z
(`Ixx = Iyy`, `Izz ≈ 2 Ixx`: hp3 6.48e-3/1.296e-2, hp4 6.638e-3/1.321e-2,
hp5 5.063e-3/1.013e-2) or spherical (cubecorners 1.728e-2/1.728e-2), and
their rotation in the settled presets is small. For ω along z the omission
is exactly zero. It becomes visible only with an authored asymmetric body or
a large off-axis ω; latent for the catalog.

Repro: `repro_gyroscopic_free_body.mjs` (exit 1).

## Negative results (important)

* No divergence, no non-finite state, no energy gain on any tested system at
  dt = 0.5–8 ms, EA = 1e3–1e12 N, B = 0–1e9 N·s (the latter is a stress sweep
  outside production; it stays finite). At EA ≥ 1e9 N with dt = 2 ms the stiff
  mode is under-resolved: a 1 mm initial displacement produces a 12 mm
  excursion (the cable goes slack) instead of a 1 mm oscillation. That is the
  expected sampling limit, and production uses EA = 2e4 N.
* The first-order free-mass error is exactly the documented integrator
  behavior, not a defect. Full converged values are in `evidence.json`.
* Solver iterations are correctly wired but have no effect on a single cable
  row; this is a property of the constraint, not a broken setting.
* Frictionless guide count is dynamically neutral to 6e-15 m.
* The pendulum period is accurate to 7.5e-6 relative at the default dt; the
  only first-order loss is the documented XPBD energy dissipation.
* The earlier "Atwood closed form passes" result used `spool:false` with the
  helper's default `inertia = 0`, i.e. a fixed non-rotating roller. Its
  `a=g/3` is the massless-pulley limit; the prior claim should not be read as
  validating authored pulley inertia.

## Limitations

* Simulation evidence only; no hardware validation.
* Minimal authored scenes exercise the same engine and defaults as the app,
  but they are not shipped trajectories. The production reachability of the
  two model gaps is inferred from the shipped inertia/radius data, not from a
  measured print/regression.
* The Wasm artifact is the checked-in build; source and artifact could differ
  (the prior program found a stale native addon). No native/Python rebuild or
  parity run was performed, per the resource rules.
* The oscillator reference is a linearization at the current-span equilibrium.
  The engine's `EA/L_current` law makes the spring weakly nonlinear; a
  constant-`k = EA/L_rest` reference is not appropriate.
* Period extraction uses zero crossings with every-step sampling; the
  interpolation bias is below 1e-4 of a period. The 8 ms soft-oscillator
  periods still scatter by ~3 % because only ~25 samples cover a period.
* The free-mass substep test adds an unrelated dummy rigid body and a slack
  cable solely to satisfy the substep gate; the body under test is coupled to
  neither.

## Reproduce

From `/home/torbjorn/repos/hp-sim5`:

```bash
node research/output/program-2026-11-18-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-600802e7/session-2026-11-18-test-numerical-integration-accuracy-on-analytic-systems-damped-h-c6fd65ea/probe_integration_accuracy.mjs
node research/output/program-2026-11-18-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-600802e7/session-2026-11-18-test-numerical-integration-accuracy-on-analytic-systems-damped-h-c6fd65ea/repro_atwood_wheel_inertia.mjs
node research/output/program-2026-11-18-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-600802e7/session-2026-11-18-test-numerical-integration-accuracy-on-analytic-systems-damped-h-c6fd65ea/repro_gyroscopic_free_body.mjs
```

The first exits 0 after 19 passing checks and writes `evidence.json`; the two
repros exit 1 with the expected-vs-measured message. Measured resource use:
probe 2.4 s wall / 132 MB RSS; each repro <1 s / <100 MB. `free -h` before and
after stayed at ~1.3 GiB available and ~0.7 GiB free swap; no build was run.

## Evidence

* `evidence.json` — all raw rows, defaults, checks, summary.
* `common.mjs` — scene builders, closed forms, RK4 reference.
* `probe_integration_accuracy.mjs` — the 19-check harness (exit 0).
* `repro_atwood_wheel_inertia.mjs` — exit 1, +33.3 % / +39.0 %.
* `repro_gyroscopic_free_body.mjs` — exit 1, 14.19° / 27.25°.
