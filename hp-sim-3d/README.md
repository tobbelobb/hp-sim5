# hp-sim-3d Implementation Notes

hp-sim-3d is the 3D Hangprinter application built on the shared JavaScript
cable-joints PBD/XPBD code in `src/js/cable_joints_3d/`. Machine geometry and
physics properties come from USDA scenes; application controllers turn print
commands into motor input and render the resulting ECS world with Three.js.

The simulation is intentionally specialized for Hangprinter cable paths,
rigid assemblies, and driven spools. It is not a general-purpose rigid-body
joint engine.

## Runtime and Scene Construction

`app/hp-sim-3d.js` boots the application assembled by `app/appBootstrap.js`.
The bootstrap is a composition root for controllers that own machine loading,
print jobs, workers, feature flags, quality checks, inspection tools, and view
state.

`app/setupScene.js` is a small scene-loading coordinator. Scene construction is
split into these phases:

1. `app/scene/machineSceneSpec.js` normalizes the supplied USDA stage, reads the
   selected scene root, and validates the machine specification.
2. `app/scene/machineScenePipeline.js` discovers scene prims and builds an
   entity plan using the feature-specific builders in `app/scene/`.
3. `app/scene/entityPlanApplier.js` applies resources, entities, and post-apply
   hooks to the ECS world.
4. `app/sceneSystems.js` registers the input and simulation systems once, in
   execution order.

`app/machineSceneController.js` owns catalog and uploaded machines, bakes cable
scene data before loading, and can append multiple namespaced machines to the
same world. The first machine supplies global physics resources such as gravity
and the fixed timestep derived from USDA `timeCodesPerSecond`.

The renderer is stored as the `renderSystem` world resource rather than as a
simulation system. `app/runner.js` advances physics and invokes rendering
separately, so ASAP playback can simulate several steps between rendered
frames.

## Simulation Step Order

`World.update(dt)` runs systems in the order registered by
`app/sceneSystems.js`. For an active local simulation step, the order is:

1. Process pointer/input state.
2. Save previous final positions and orientations.
3. Consume queued spool commands and run position-mode stepper motors.
4. Apply gravity, then integrate predicted linear and angular poses.
5. Sync rigid-body members from their parent bodies and enforce spool-axis
   projections.
6. Rebuild cable attachment geometry and cache attachment data.
7. Redistribute adjacent cable-segment rest lengths in
   `CableFrictionSystem` before constraint solving.
8. Solve cable constraints using each path's `solverIterations`, then resolve
   cable over-corrections.
9. Derive linear and angular velocities from the corrected poses.
10. Run torque-mode motors using cable loads recorded by the cable solver.
11. Update extrusion, encoders, and missed-step diagnostics.

The central position solve still has the usual PBD/XPBD shape:

```txt
save previous pose -> integrate pose -> solve constraints -> update velocities
```

Torque mode is a specialized velocity update after that sequence. Its angular
velocity change affects subsequent simulation steps.

There is no global substep loop around the complete system sequence. The runner
repeatedly calls `world.update(dt)` with the fixed scene timestep, either from a
real-time accumulator or in time-budgeted ASAP batches. Cable paths can request
different solver iteration counts through `cablePath:solverIterations`, while
friction work per step is scaled with `dt`.

## Cable Paths

A `CablePathComponent` joins ordered cable segments and stores link types,
winding direction, stored length, stiffness/compliance, damping, cable width,
and its solver iteration count.

Before the position solve, `CableFrictionSystem` redistributes rest length
between adjacent segments. With no active friction barrier it tends toward
equal PBD extension; at pinholes and non-free rolling links it limits the
tension ratio using the configured coefficient of friction and wrap angle.
Cable attachment updating also handles rolling geometry, hybrid link state,
and optional line layering.

The cable solver maps ordinary rigid-body-member endpoints to the parent body,
applies translational and tensor-based angular corrections, and treats supported
spool endpoints as a special one-axis rotational degree of freedom. For
torque-mode spools it also records cable load torque and, where available,
effective stiffness and damping for `TorqueModeSystem`.

## Rigid Bodies and Members

The rigid assembly model uses:

```txt
RigidBodyComponent
RigidBodyMemberComponent
RigidBodySyncSystem
```

`app/scene/rigidBodyBuilder.js` turns an authored rigid-body group into one
simulated parent entity. It computes aggregate mass, center of mass, linear
velocity, and inertia tensor from the members. Member inertia tensors are
rotated into the aggregate frame and combined with parallel-axis terms.

The original entities become zero-mass, non-gravity-affected members with
authored local positions and orientations. `RigidBodySyncSystem` derives their
world transforms and velocities from the parent pose. Cable and distance
constraint helpers normally redirect a member endpoint's reaction to that
parent body.

This is equivalent to a rigid body with attachment frames, not a collection of
independent bodies connected by XPBD fixed joints. Member attachment is
therefore hard and kinematic; it has no per-joint compliance or solver
iteration setting.

## Spools and Motors

A physical motor housing is fixed to its Hangprinter body, while its spool rotor
has one rotational degree of freedom. hp-sim-3d represents that behavior with a
specialized member model:

- `SpoolStateComponent` stores the spool axis and reference orientation.
- `StepperMotorSystem` handles position-commanded motors. Open-loop mode uses
  holding and damping torque; closed-loop mode directly projects to the target
  twist.
- `TorqueModeSystem` handles torque-commanded motors after the cable solve and
  includes the solver's cable load.
- Rigid-body-member spools integrate their free twist in member-local space.
- `RigidBodySyncSystem` recomposes the world orientation from the parent body
  and local member orientation.
- `constrainSpoolOrientation()` removes swing, and
  `constrainSpoolAngularVelocity()` removes off-axis angular velocity.
- Motor paths apply a custom equal-and-opposite rotation or angular impulse to
  the parent body when a spool is a rigid-body member.

The spool is therefore not a generic XPBD hinge. Direct projection is cheap and
stable, but removing off-axis motion is not the same as solving a bearing
constraint and can discard angular energy without exposing the corresponding
bearing reaction.

## Inertia Tensors

The 3D ECS uses a full 3x3 inertia tensor. `MomentOfInertiaComponent` stores the
local tensor and its inverse; scalar constructor input remains supported as an
isotropic tensor. `src/js/cable_joints_3d/inertia_tensor.js` supplies world-space
tensor transforms, inverse-inertia application, axis-effective inertia, and
parallel-axis aggregation.

Cable constraints, distance constraints, over-correction resolution, collision
response, and motor/body reaction paths use this tensor logic. Spool drive and
spool-axis constraints intentionally reduce it to effective inertia about the
allowed axis, because the current rotor model exposes only that one degree of
freedom.

## Remaining Hinge-Joint Gap

If bearing reactions or off-axis rotor dynamics become important, the next
model should be:

```txt
body <-> motor housing: fixed attachment
motor housing/body <-> spool rotor: hinge joint + angular motor
```

That requires a rotor entity independent of the body, fixed/hinge constraints
between bodies, and a motor constraint that shares corrections through the
existing inverse-inertia tensors. Compliance, damping, and stored constraint
multipliers could then model bearing softness and losses and expose bearing
reaction forces or torques. More solver iterations or whole-step substeps may
also be needed for a stiff hinge. The current one-axis projection remains a
useful simple mode.
