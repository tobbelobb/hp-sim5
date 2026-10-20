# Two-port pinhole gates

Select **Hangprinter v4 (pinhole gates, experimental)** in Machines instead of
the original HP4. The scene is `public/usd_scenes/hp4_pinhole_gate.usda`;
`node scripts/create_hp4_gate_scene.mjs` regenerates it from the original scene.
Native Rust, browser Wasm and JavaScript support it. The independent Python
backend explicitly rejects this new attachment for now.

## Mechanical model

`pinhole_gate` connects two distinct ports mounted on the same rigid body.
The entry prim declares `pinholeGate:exit`; the path's gate entry in
`cablePath:stored` preserves the hidden internal cable length. There are no
internal CableJoints. Connected frictionless spans are condensed into **one
unilateral length constraint with one common tension**. Carrier translation
and rotation Jacobians are summed before applying inverse mass, retaining
cross terms and the separate port lever arms. The same solve includes the
external spool, holding-torque release and signed torque feedback once.

The first model has ideal massless flow and zero internal friction. Interior
frictional guides and more than one carrier plus one external rotor are
unsupported. This is not a finite-inertia bearing model.

For gate paths, `cablePath:stiffness` and `cablePath:damping` describe the combined
constraint. The HP4 variant uses 20000/6 N/m and 5/6 Ns/m to preserve six identical
original Kelvin–Voigt spans in series. Tune those explicit parameters and port
geometry when measurements become available. Material partition is reconstructed
without changing total free cable length; it is not another tension solver.

The variant reduces HP4 cable joints from 51 to 33 and constraint projections
from 102 to 42 per two-iteration substep. D reeving is unchanged. This reduces
solver work structurally; no measured overall speedup or reduction in timestep
bias is claimed from the short checks.

## Visualization

`PinholeGateComponent.visualRoute` / USD `pinholeGate:visualRoute` contains a local
polyline drawn independently of physics. `pinholeGate:visible` hides it. Original
ABC bearing masses remain in the carrier, but their independent physical rotor
DOFs are removed. Their visual route introduces no constraints. Flow inertia,
resistance and animated visual bearing rotation remain future extensions.

## Checks

`npm run test:physics` includes the gate checks automatically. Four analytic
JS/Rust checks verify combined inverse mass, equal tension and symmetric/asymmetric
port torque. Three fresh one-second logo checks cover native, Wasm and JS and
compare sampled positions within 1 micrometre. Existing baseline logo and
loaded-state tests also run. These establish a bounded implementation check,
not hardware accuracy, long-print stability or calibrated damping.
