# Spool-On-Surface Energy Analysis (XPBD Cable Joints)

## Scope

This note analyzes energy behavior for a single hybrid spool (ball with stored cable)
rolling on a flat surface while connected to a static attachment point by one cable joint.
The immediate target is the sudden fling/jump failure mode.

## Reference Energy Model

Let:

- mass `m`
- raw spool radius `R`
- inertia `I`
- gravity vector `g`
- spring-like cable stiffness `k`

Energy terms used in the debug scene:

- `K_trans = 0.5 * m * |v|^2`
- `K_rot = 0.5 * I * omega^2`
- `U_gravity = -m * g dot x`
- `U_cable ~= 0.5 * k * max(0, L_current - L_rest)^2` per joint
- `E_total = K_trans + K_rot + U_gravity + U_cable`

For ideal rolling contact, the instantaneous contact point is not slipping; the normal
constraint should not inject large positive energy spikes by itself.

## Mapping To This Codebase

The update path is split into systems:

1. prediction (`GravitySystem`, `MovementSystem`, `AngularMovementSystem`)
2. geometry update (`CableAttachmentUpdateSystem` and topology/pinch systems)
3. position solve (`PBDCableConstraintSolver`, contact projections)
4. velocity reconstruction (`PBDVelocityUpdateSystem`, `PBDAngularVelocityUpdateSystem`)
5. velocity contact impulses (`BallBorderOrFlipperVelocityContactSystem`)

Because the solve is split, energy drift can appear when a frame has both:

- a large positional correction (`delta_lambda` source), and
- a follow-up velocity impulse (normal/tangential impulse) from the corrected state.

That is exactly where sudden fling events usually originate.

## Minimal Repro Scene

Use `examples/js/flipper/spool_energy_debug.html`.

Scene definition:

- one dynamic hybrid ball (spool endpoint)
- one static attachment point
- one cable joint/path
- flat border floor (no flippers, no bumpers)

Recorded per step:

- position/velocity/angular velocity
- `K_trans`, `K_rot`, `U_gravity`, `U_cable`, `E_total`, `DeltaE`
- stored wrap length (`stored[0]`)
- effective bottom collision radius
- max border `delta_lambda` for the ball

## How To Capture Data

1. Open the scene and reproduce a jump/fling.
2. In console, run `dumpSpoolEnergyTrace()`.
3. Copy JSON between:
   - `SPOOL_ENERGY_TRACE_JSON_START`
   - `SPOOL_ENERGY_TRACE_JSON_END`

Optional configuration before reset:

`setSpoolEnergyConfig({ initialStoredTurns: 1.5, initialLinearSpeed: 0.20, cableStiffness: 1200 })`

## Interpretation Checklist

For each fling window, check whether all are true in the same 1-3 frame interval:

- large positive `DeltaE`
- abrupt change in effective bottom collision radius
- abrupt jump in border `delta_lambda`

If yes, the jump is solver-coupling/contact-entry dominated rather than a physically
smooth transfer from winding geometry.
