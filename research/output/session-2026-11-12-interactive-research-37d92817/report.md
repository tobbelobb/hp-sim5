# Stable 1 Nm (and any-torque) stepper motors at the 2 ms step

Base revision `50e826bf` plus the uncommitted 0.5 → 1.0 Nm change to
`public/usd_scenes/hp4_rigid_body.usda`, `public/usd_scenes/hp4_pinhole_gate.usda`
and `tests/fixtures/hp4_loaded_stability.json` (four motor torque values + scene
hash) from conversation `30f989dd-e916-4dcd-a8d2-230355b9cf9e`.

## Question

Doubling holding torque to 1.0 Nm broke two gates:

- `hp4_stability` at 1 substep. The closed-loop HP4 kept a 125 Hz, 25.6 µm limit cycle.
- `motor_drag` for open-loop HP4. Motor C slipped 696 cycles, and the effector came back to its start.

Is this a PID-tuning problem that a retune (closed-form or iterative) can fix,
so that any holding torque just works?

## Answer

**No retune was needed.** The gains derived from the motor
(kp = T·npp, kd = 2√(kp·J), ki = kp/1 s) give a stable linear loop at every
torque and timestep tried. Both failures were numerical artifacts of the
saturation and detent nonlinearities sampled at 2 ms, and two small changes fix
them. The tuning logic was already automatic, so changing the holding torque
needs no other edits. A new test runs the loaded HP4 at 0.5 and 4 Nm.

## Closed loop: the servo's current-limit test

### Hypotheses and experiments

1. **Torque scan on the loaded fixture** (`scratchbook/loaded_settle.mjs`, 1
   substep, old code):
   - 0.6, 0.7, 0.8, 0.9 and 1.2 Nm settle.
   - 1.0, 1.5 and 2.0 Nm lock into the same 125 Hz cycle, which is a period of 4 steps.

   The pattern is not monotonic, so this is a stable rest state coexisting with a
   stable limit cycle, not a linear instability.
2. **Linear analysis** (`scratchbook/linmap.py`): finite-difference Jacobian of
   the unsaturated step map (servo + implicit rotor + XPBD line + light effector).
   - Spectral radius is 0.998 for 0.25–4 Nm at dt = 0.5–2 ms, and that slowest pole is the 1 s integrator.
   - The fast poles have magnitude about 0.72 or less.
   - Conclusion: the PID gains are fine.
3. **One-motor nonlinear model** (`scratchbook/toy.py`) copying `servo()` and the
   `J + K·dt²` spool inertia in the cable solve:
   - It reproduces kick-triggered cycles once kp·dt²/J ≳ 4.
   - At 0.5 Nm, kp·dt²/J = 2, which is why the hand tuning settled. At 1 Nm it is 4.
   - Turning off the cable-side release makes no difference, so the servo's own saturation test is the cause.

### Mechanism

`servo()` tested the **explicit** command `|kp·e + I + kd·ė| ≥ T`:

- **Below the limit:** the step is implicit, with K = kp + kd/dt. The rotor behaves like
  inertia J + K·dt², which is 9·J at 1 Nm and 2 ms.
- **Above the limit:** K drops to 0. The rotor gets the full ±T on its bare inertia J,
  a 9× larger velocity kick. That jumps it across its whole linear band
  (±1/npp rad) in one step and drives a relay oscillation between ±T.

### Fix (`src/wasm/cable_solver/src/engine.rs` `servo()`, `lib.rs` `load()`)

The limit is now tested on the torque the servo would **end the implicit step
with**, assuming the lines keep last step's torque τ_line on the spool:

    u_end = (cmd − K·c·τ_line) / (1 + K·c),   c = dt²·(1/J + housing⁻¹)
    saturated ⇔ |u_end| > T  →  drive ±T, K = 0;  else  drive cmd, K

- The rotor's velocity change is now continuous where the limit engages.
- In steady load, u_end equals the held torque.
- `load()` now also accumulates the line torque `e[59]` for position-mode spools. Its
  observation flag and stiffness/damping coefficients stay torque-mode only, so the JS
  `torqueModeCableLoad*` maps are unchanged.

An intermediate version tested only the motor-alone realized torque
`|cmd|/(1+K·c)` and ignored τ_line. It fixed the cycle but made HP4 servos refuse
to yield to a hand drag (8 mm instead of > 0.2 m). Each HP4 spool feeds two line
rows, and the cable solve's release check is per row, so the total spool torque
went over T without any single row tripping it. Including τ_line fixed that.

## Open loop: detent aliasing

The previous agent flagged motor C's 696-cycle slip as "suspicious". A substep
sweep of the HP4 hand drag (`scratchbook/drag_probe.mjs`, old code) showed it
is numerical:

| Holding torque | 1 substep | 4 substeps | 16 substeps |
|---|---|---|---|
| 0.5 Nm | 127 | 135 | 134 |
| 1.0 Nm | 696 | 111 | 108 |
| 2.0 Nm | runaway: A/B/C slip 309/959/858, effector ends 0.70 m away, still moving | 97 | 96 |

Cause: a slipping rotor sweeps a large part of a detent cycle (2π/50) per 2 ms step,
and the once-per-step sample of `−T·sin(npp·err)` aliases into a net drive. Fix:
average the detent torque and its stiffness over the phase φ₀ → φ₀ + npp·ω·dt
swept this step. The torque becomes `−T·(cos φ₀ − cos φ₁)/Δφ` and the stiffness
`T·npp·(sin φ₁ − sin φ₀)/Δφ`. Both reduce exactly to the old law as Δφ → 0.

After the fix, at 1 substep:

| Holding torque | Slip cycles | Effector after release |
|---|---|---|
| 0.5 Nm | 123 | 1.6 mm from start |
| 1.0 Nm | 137 | 1.0 mm from start |
| 2.0 Nm | 106 | 1.1 mm from start |

There is no runaway, and the effector stays displaced as the test expects.

## Results (after both fixes, HP4 at 1.0 Nm)

| Check | Before | After |
|---|---|---|
| Loaded fixture, 1 substep, 0.25–4 Nm | cycles at 1.0/1.5/2.0 Nm | all settle, z p2p ≤ 0.001 µm |
| `hp4_stability` 1/2/4 substeps | 1 substep fails | pass |
| New 0.5 and 4 Nm stability tests | 4 Nm fails on old code | pass |
| Closed-loop HP4 drag, 0.25–4 Nm | — | returns within 7 µm, 0 missed steps |
| Open-loop HP4 drag | fails (696 cycles, effector back) | pass |
| HP4 Squares contour RMS: closed unlayered / layered | 0.387 / 0.247 mm | 0.387 / 0.248 mm |
| HP4 Squares contour RMS: open unlayered / layered | 0.356 / 0.250 mm | 0.357 / 0.251 mm |
| HP4 Squares slips | 0 | 0 |

Test suites:

- `node --test tests/benchmark/*.test.mjs`: 64 pass, 0 fail.
- `cargo test --release`: ok.
- `npx jest`: 303/303.
- `pytest tests/python`: 200 passed.

The Squares replay (`scratchbook/squares_replay.mjs`, `contour.py`, copied from
session `ecc2ae53`) shows ordinary printing is unchanged. The fixes only act at
the current limit or while a rotor is slipping.

## Observed but not addressed

The slideprinter is still moving 0.2–2 mm per step at the end of the
closed-loop drag. This is the same with the original code, and the fix reduces
it at 2 Nm (1.7 → 0.17 mm/step). It's a separate issue.

## Reproduce

- **Torque sweep on the loaded fixture:** `node scratchbook/loaded_settle.mjs <substeps> <steps> <torque>`
- **Drag probe:** `node scratchbook/drag_probe.mjs <scene> <closed 0|1> <torque> [substeps]`
- **Models:** `.venv/bin/python scratchbook/toy.py`, `.venv/bin/python scratchbook/linmap.py`
- **Squares:** `node scratchbook/squares_replay.mjs fix_cl_unl 400 1 0`, then
  `python contour.py fix_cl_unl.json`. The 22–30 MB telemetry was deleted; it is
  regenerable with these commands.
- **Engine diff:** `scratchbook/servo_fix.patch`

## Next steps

- Other machine scenes (HP3, cubecorners, slideprinters) are still at 0.5 Nm. With
  these fixes they can be raised with no other changes.
- The cable solve's per-row release could check the total spool torque across
  rows. Then the motor-side τ_line estimate would no longer be needed to make
  multi-row spools yield.
