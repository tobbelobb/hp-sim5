# hp-sim-3d Implementation Notes

hp-sim-3d is the 3D Hangprinter app built on the shared JavaScript cable-joints
XPBD/PBD code in `src/js/cable_joints_3d/`. Its current physics structure is
intentionally pragmatic: it follows the broad Muller-style PBD update order, but
keeps the rigid body and spool model simpler than a fully general rigid-body
joint solver.

## XPBD Step Order

`app/setupScene.js` registers systems in the order that `World.update(dt)` runs
them. In local simulation mode the frame order is:

1. Save previous final position and orientation.
2. Apply non-constraint state changes, including remote spool input and the
   custom stepper motor model.
3. Apply forces and integrate predicted linear and angular poses.
4. Sync rigid-body members from their parent body pose.
5. Rebuild cable attachment geometry and slack state.
6. Solve position-level cable constraints and over-correction constraints.
7. Apply post-solve cable friction.
8. Derive final linear and angular velocities from the corrected poses.
9. Update extruder and encoder bookkeeping.

This matches the usual PBD/XPBD shape:

```txt
save previous pose -> integrate velocities -> integrate pose
-> solve constraints -> update velocities
```

There is not yet a single global "for each substep" loop inside
`setupScene.js`. The runner calls `world.update(dt)` repeatedly with a fixed
simulation timestep. If future constraints become softer or more coupled,
explicit substeps around the whole physics sequence would be the natural way to
make stiffness and stability easier to reason about.

## Rigid Bodies and Members

The current 3D rigid-body model uses:

```txt
RigidBodyComponent
RigidBodyMemberComponent
RigidBodySyncSystem
```

`RigidBodyComponent` is the simulated parent body. Members are authored local
frames that are kinematically synced from that parent. This is close to "a rigid
body with attachment frames", which is useful for Hangprinter geometry, but it
is not the same as representing every attached object as an independent rigid
body connected by XPBD fixed joints.

Cable constraints can still redirect endpoint reaction to the parent body, and
this gives useful body-level response for the current app. The tradeoff is that
member attachment stiffness is effectively hard-coded by sync/projection logic
rather than controlled by joint compliance and solver iterations.

## Spool Rotors

A motor housing should be treated conceptually as fixed to the Hangprinter body.
The spool rotor is different: physically it has one rotational degree of freedom
relative to the housing/body.

The current implementation keeps this simple:

- `SpoolStateComponent` stores the local spool axis and reference orientation.
- `StepperMotorSystem` applies motor torque along the spool axis.
- Rigid-body-member spools integrate their free twist directly in member-local
  orientation.
- `RigidBodySyncSystem` recomposes the member world orientation from the parent
  body and member local orientation.
- `constrainSpoolOrientation()` discards any swing and keeps only twist around
  the spool axis.
- `constrainSpoolAngularVelocity()` projects angular velocity onto that same
  axis.

In other words, the spool is not currently a generic XPBD hinge joint. Off-axis
rotor orientation and off-axis angular velocity are clamped away. This is cheap,
stable, and good enough for idealized control and cable winding behavior, but it
can remove angular energy without a corresponding physical bearing reaction.

`StepperMotorSystem` does apply a custom equal/opposite angular-velocity
reaction to the parent body along the motor axis. That gives useful motor
reaction behavior for hp-sim-3d, but it is still a special path rather than a
general hinge/velocity-motor constraint.

## Inertia Approximation

The shared 3D ECS currently reuses `MomentOfInertiaComponent` as a scalar
effective inertia. Some code estimates effective inertia about a requested axis,
but hp-sim-3d does not yet carry a full 3x3 body inertia tensor through the
solver.

This is sufficient for the current app assumptions:

- spools mostly rotate about a known local axis,
- many objects are close enough to isotropic or axis-symmetric for the visual
  and control use cases,
- cable and motor reactions mainly need plausible body response, not full
  arbitrary 3D rigid-body fidelity.

It is not a fully general Muller-style 3D rigid-body model. A general model
would use world-space inverse inertia tensors when distributing angular
corrections:

```txt
delta_theta ~= inverse_inertia_world * constraint_angular_gradient
```

That becomes important for anisotropic bodies, off-axis cable side loads,
bearing reactions, and constraints that should share correction between two
independent rigid bodies.

## Future Hinge-Motor Direction

If hp-sim-3d later needs more physical spool/bearing behavior, the target model
should be:

```txt
body <-> motor housing: fixed attachment
motor housing/body <-> spool rotor: hinge joint + angular motor
```

A proper XPBD hinge would keep the rotor attachment point fixed to the body,
align the body and rotor hinge axes, and leave twist around that axis free. When
the rotor receives off-axis rotation or side load, the solver would correct both
body and rotor according to inverse mass and inverse inertia. With compliance it
could model bearing softness; with velocity-level damping it could model bearing
losses; with stored constraint multipliers it could expose bearing reaction
force/torque.

That future work likely requires:

- a rotor entity separate from the body/housing,
- fixed and hinge joint constraints between rigid bodies,
- full or improved inertia handling,
- substeps or more solver iterations for stable stiffness,
- motor/servo constraints or velocity-level angular motor damping.

The current projection path should remain useful as a simple/debug mode even if
a full hinge path is added later.
