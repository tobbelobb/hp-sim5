# Stateful two-port pinhole gates

Select **Hangprinter v4 (pinhole gates, experimental)** in Machines instead of
the original HP4. The scene is `public/usd_scenes/hp4_pinhole_gate.usda`;
`node scripts/create_hp4_gate_scene.mjs` regenerates it from the original scene.
Native Rust, browser Wasm and JavaScript support the attachment. The independent
Python backend explicitly rejects it.

## Mechanical model

`pinhole_gate` replaces the ABC effector's pinhole-wheel-wheel-pinhole sequence
with six condensed gate units. Each unit has distinct entry and exit ports and
no internal CableJoints. This reduces the HP4 ABC topology from 51 to 33 cable
joints while retaining unilateral cable behaviour on both sides.

The hidden reeving has one material-flow coordinate `q`. Entry and exit use
separate unilateral constraints with rest lengths biased by `q`; `q` is held
fixed during the position solve and advanced from the final entry/exit
constraint multipliers. The resulting hidden flow law is

```text
Tin - Tout = M qdd + B qdot + Fc sign(qdot)
```

where `M` is the summed bearing inertia reflected through the cable radius, `B`
is viscous drag and `Fc` is dry friction. Static friction and a non-reversal
condition hold `q` when the tension difference cannot overcome the breakaway
force. This preserves small differences between entering and exiting CableJoints
without recreating the long internal constraint chain.

The HP4 values are **provisional synthetic values**, not hardware calibration:
`M = 0.023668639053254437 kg`, `B = 0.9467455621301777 Ns/m`,
`Fc = 4e-5 N`, and radius `0.0065 m`. `M` is computed from the two original
bearing inertia tensors; the friction values are placeholders for later
measurement.

The JS, Native Rust and Wasm implementations use the same state law. The
native/Wasm path ABI is version 3 and its path header includes gate link, flow,
flow velocity, inertia, damping and Coulomb force.

## Visualization

`PinholeGateComponent.visualRoute` / USD `pinholeGate:visualRoute` is a local
polyline independent of the physical constraint solve. The twelve hidden
bearings are rendered as cylinders, not spheres. Their rotation is driven by
actual cable transport through the gate and scaled by
`pinholeGate:rotationScale`.

Replaying the original HP4 shows the top and bottom bearings counter-rotating.
The gate therefore uses top scale `1.0` and bottom scale `-0.338`. The bottom
magnitude is a provisional visual compromise: the original ratio varied by
branch and trial (about `-0.11` to `-0.39` in the 200- and 1000-step probes).
The animation is render-only and does not add physical rotor DOFs.

## Checks

`npm run test:physics` rebuilds Native Rust and Wasm and runs the production HP4
Print Logo stability checks plus loaded-state settling at one, two and four
substeps. The gate-specific checks cover the state law, six gate units, twelve
visual cylinders, transport-driven wheel angles and Native/Wasm/JS parity.

At the final source state, all 11 physics-gate tests passed. Short HP4 prefix
evidence found a nonzero gate tension difference (maximum `0.0781 N`, mean
absolute `0.0090 N` over 200 steps), while the isolated summed-friction probe
showed breakaway and a `0.0157 N` peak difference. A matched 1000-step native
trial took `146.3 ms` versus `386.0 ms` for the original model (`2.64x`) with a
final-Z difference of `0.000184 m`.

These are short, simulated structural and performance checks. They do not
establish reduced numerical damping, hardware-calibrated realism, long-print
accuracy or full-print success. Browser appearance has not been inspected.
