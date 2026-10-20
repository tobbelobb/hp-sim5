# The experimental motor update: origin, derivation and timestep dependence

Status: explanatory research report. The alternative motor formula remains experimental.

Integration correction: the separate rolling-slip removal was withdrawn after
it caused explosive HP4 Print Logo motion. The faster formula was never merged.
Historical no-slip trials below remain research evidence, not validation of a
production solver. See the [stability correction](hp4-logo-stability.md).

## Abstract

The experimental motor update compensates a spring load evaluated at a predicted rotor pose. Its local derivation removes an extra timestep-dependent resistance term. Saved HP4 simulations show both promise and substantial integration error: at 30 seconds, the alternative reaches Y=633.7 mm with one substep and 572.4 mm with sixteen, while X changes from 675.5 to 3.9 mm. The coarse trajectory is therefore unreliable. Similar fine-step endpoints do not establish matching time histories or physical accuracy. These simulated results predate the new motor-feedback fix; no hardware reference was used.

“Faster” means more motion per **simulated second under identical torque commands**, not reduced CPU work. The measurements are coordinates at fixed times, not total distance travelled.

## Origin and local theory

The [October 19 investigation](../output/research/sessions/2026-10-19-interactive-research-8d7a82a2/report.md) introduced this correction, then withheld it after poor calibration fits. The [October 20 study](../output/research/sessions/2026-10-20-interactive-research-62300644/report.md) revisited its mechanics. This formula is not from the Cable Joints paper.

Let `h` be the physics substep duration, `I` the rotor inertia, `w` angular velocity, `K` reflected cable stiffness, `D` reflected cable damping, `T` the drive torque, and `L` the recorded cable torque. The code forms `S = L + D*w`, intended to remove the old damping contribution from `L` and leave a spring-like load.

The current update and the alternative differ by one numerator term:

```text
Q = I + h*D + h²*K

current:     w_next = (I*w + h*(T + S)) / Q
alternative: w_next = (I*w + h*(T + S) + h²*K*w) / Q
```

Motor drag, speed droop, friction and cogging remain in `T`.

Here is the rationale in a **single linear spring/rotor model**. Predict the angle using the old velocity:

```text
theta_pred = theta_old + h*w
S = -K*(theta_pred - theta_equilibrium)
```

If the new velocity corrects that prediction, then

```text
theta_next = theta_pred + h*(w_next - w)
spring_torque_next = S - h*K*(w_next - w)
```

Insert this spring torque and implicit damping into angular momentum balance:

```text
I*(w_next - w)/h = T + S - h*K*(w_next - w) - D*w_next
```

Rearranging gives the alternative formula above. It is the backward-Euler result for this local model when the load has already been evaluated at the predicted angle.

The current formula instead satisfies

```text
I*(w_next - w)/h = T + S - D*w_next - h*K*w_next
```

For a steady velocity, its last term behaves like an additional angular damping coefficient **`h*K`**. The alternative replaces that term with `-h*K*(w_next-w)`, which vanishes at steady velocity. For the measured two-active-branch HP4 value `K=36 Nm/rad`, `h*K` is 0.072 Nms/rad at one 2 ms substep and 0.0045 Nms/rad at sixteen substeps. This is the proposed mechanism for numerical resistance.

## Why that derivation is not a proof of the full implementation

The actual update follows motor/body prediction, cable geometry updates, constraint projection, and velocity reconstruction. Its input velocity already includes projection effects. `L` comes from XPBD multipliers, while `K`/`D` are local coefficients rather than an exact reduced model of all reeving.

The derivation therefore assumes more than the implementation guarantees. Pulley inertia, carrier rotation, slack transitions and subsequent motor integration can cause further error. The compensating term is a plausible local correction, not proof of a correct complete algorithm. Calibration residuals alone cannot resolve this.

## What the matched measurements establish

Native Rust, zero buildup, 30 mm spools, closed loop, 2 ms outer steps. At two seconds, A receives -0.000334089 Nm and B/C each -0.037733 Nm; D stays held. Both formulas remove rolling slip but use **old first-visit feedback**. These are historical results, not measurements of the newly committed feedback fix.

| Substeps | `h`, ms | Current Y at 10 s, mm | Alternative Y at 10 s, mm | Current Y at 30 s, mm | Alternative Y at 30 s, mm | Alternative X at 30 s, mm |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 2 | 19.61 | 612.45 | 66.30 | 633.67 | 675.47 |
| 4 | 0.5 | 107.10 | 480.58 | 322.78 | 602.15 | 73.58 |
| 16 | 0.125 | 261.57 | 488.37 | 548.79 | 572.41 | 3.94 |

The original Y endpoint increases with refinement while the alternative decreases. Opposite biases are possible because their discretization errors differ. Similar **30-second endpoints** can also reflect approaching equilibrium: at sixteen substeps, the 10-second Y difference is still 227 mm. Sixteen substeps is not an established continuum reference; the 64-substep request was rejected before physics ran.

Combining accumulated feedback with the alternative reduced one-substep X at ten seconds from 565.9 to 93.0 mm in JS; Y changed from 612.5 to 594.8 mm. This still falls short of a reliable coarse-step trajectory. The alternative motor formula is not included in the three requested fixes.

## Outcomes

- **Supported, E19/E20/E26 versus E29/E30/E31:** the formulations differ strongly at coarse timesteps and have more similar 30-second endpoints at finer timesteps.
- **Supported locally by algebra:** compensating a spring load evaluated at a predicted pose removes the `h*K*w` steady-motion term under the stated assumptions.
- **Inconclusive:** the alternative correctly integrates the complete HP4 mechanism. No exact trajectory or hardware reference was used.
- **Proposed, not tested:** compare complete time histories after the three fixes against an independent linear rotor/cable reference, then the reeved machine. Check energy, torque balance and timestep refinement before calibration fitting.

## References

- [October 19 report](../output/research/sessions/2026-10-19-interactive-research-8d7a82a2/report.md) and [original experimental patch](../output/research/sessions/2026-10-19-interactive-research-8d7a82a2/rejected-motor-correction.patch): origin and prior calibration outcomes.
- [October 20 experiment manifest](../output/research/sessions/2026-10-20-interactive-research-62300644/provenance.json), [motion summary](../output/research/sessions/2026-10-20-interactive-research-62300644/motion-summary.json), [E35 combined trial](../output/research/sessions/2026-10-20-interactive-research-62300644/E35.json): measured values above. E19/E20/E26 use the original motor; E29/E30/E31 use the archived alternative native artifact. Inputs derive from revision `c6f0b5a1bb501100706ecdc5e47294b74dac5ab8` plus the recorded variants.
- [Current motor source](../hp-sim-3d/app/torqueModeSystem.js) and [experimental native source](../output/research/sessions/2026-10-20-interactive-research-62300644/motor-no-slip-engine.rs): exact implemented formulas. The local derivation in this report is an interpretation of this code.
- Müller, Chentanez, Jeschke and Macklin (2018), *Cable Joints*, [DOI 10.1111/cgf.13507](https://doi.org/10.1111/cgf.13507): the rolling/pinhole model; it does not supply this motor update.
- Macklin, Müller and Chentanez (2016), *XPBD*, [DOI 10.1145/2994258.2994272](https://doi.org/10.1145/2994258.2994272): constraint projection and force estimation, used to explain the distinction between the local rotor model and the complete solver.
