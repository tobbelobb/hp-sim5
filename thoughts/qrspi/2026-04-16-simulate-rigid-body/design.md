# Design Discussion

## Current State

The 3D runtime is entity-centric today. Scene loading turns authored spool, pinhole, cable-joint, cable-path, distance-joint, and rigid-group prims into separate ECS entities or metadata over separate ECS entities, not a single compound body object. `hp-sim-3d/app/setupScene.js:332-739`

For `slideprinter_hexagon.usda`, the effector is authored as three spool prims and six pinhole prims, plus one `RigidGroup`, and six cable paths built from twelve cable joints. `public/usd_scenes/slideprinter_hexagon.usda:13-405`

At load time, the rigid group becomes a separate ECS entity with `RigidGroupComponent`, `MachineTagComponent`, and `RenderableComponent`, but not body-state components such as position, velocity, orientation, angular velocity, mass, or inertia. `hp-sim-3d/app/setupScene.js:529-579`

The group members remain ordinary body entities with their own per-entity state. Spools get position, velocity, mass, orientation, angular velocity, scalar moment of inertia, previous-final caches, stepper state, spool state, and render state. Pinholes get their own position, velocity, mass, render state, and optional rotational state depending on authored data. `hp-sim-3d/app/setupScene.js:378-452`, `hp-sim-3d/app/setupScene.js:464-525`

`RigidGroupSystem` enforces rigidity by operating on member entities directly. It computes a mass-weighted center of mass, stores rest-local offsets, performs all-pairs positional correction using member inverse masses, estimates group rotation from current member positions, and applies the orientation delta back onto each member entity individually. `src/js/cable_joints_3d/commonSystems.js:349-450`

`PBDCableConstraintSolver` is also member-centric. It traces cable paths through cable joints, computes world attachment points from per-entity position and orientation, forms translational and angular gradients per endpoint, reads inverse mass and scalar inverse inertia from each endpoint entity, and writes position/orientation corrections directly to those endpoint entities. `src/js/cable_joints_3d/cable_joints_core.js:531-637`, `src/js/cable_joints_3d/cable_joints_core.js:1338-1499`

The current frame order compounds corrections onto the same member state: cache previous state, predict motion, update cable attachments, solve cable constraints, then run rigid-group corrections, then reconstruct velocities from the start/end member states. `hp-sim-3d/app/setupScene.js:774-817`

This means the slideprinter hexagon currently stacks two rigidity mechanisms on the same assembly: `RigidGroup`, and cable constraints.

Velocity and angular velocity reconstruction are likewise per entity. `PrevFinalPos*` and `PrevFinalOrientation*` cache member state, and `PBDVelocityUpdateSystem` / `PBDAngularVelocityUpdateSystem` derive velocities from each member’s start/end pose instead of any aggregate body state. `src/js/cable_joints_3d/commonSystems.js:480-566`

The inertia abstraction is also not yet suitable for general 3D rigid bodies. The 3D ECS explicitly notes that `MomentOfInertiaComponent` is still a scalar 2D-style inertia and that a full `3x3` tensor would be needed for general 3D rigid bodies or rigid groups. `src/js/cable_joints_3d/ecs.js:9-18`


## Desired End State

Rigid groups should be simulated as true rigid bodies, not as overlays on independent member bodies.

A rigid group should have authoritative body state:
- position / center of mass
- orientation
- linear velocity
- angular velocity
- mass
- full body-space inertia tensor and world-space inverse inertia
- previous-final caches for body pose

Members should become geometric attachments of the body, not independent dynamic bodies during solve. Their world transforms should be derived from the rigid body pose plus fixed local offsets.

Cable constraints that attach to a member of a rigid group should resolve against the owning rigid body directly. The constraint should still use the member-local attachment point, but the solver should apply translational and rotational corrections to the rigid body pose, not to a standalone member entity.

Frame integration should follow the rigid-body XPBD model rather than the current member-overlay model: cache body pose, predict body motion, solve constraints against body state, then reconstruct body velocities from body start/end pose.

The driving example for phase 1 is the slideprinter hexagon, but the design should generalize to other rigid groups in the repo.

Correctness should be verifiable by these outcomes:
- the hexagon moves as one rigid body under cable pulls
- no sponge-like internal deformation appears under load
- cable pulls produce coherent body translation and rotation
- reconstructed velocities reflect whole-body motion instead of fake per-member internal motion

## Patterns to Follow

Follow the existing scene-loading pattern where authored USD prims are translated into ECS/runtime objects in `setupScene()`. Extend that path rather than introducing a second loader path. `hp-sim-3d/app/setupScene.js:319-739`

Follow the existing pattern that cable paths and cable joints are authored data first, then converted to runtime components in scene setup. The redesign should preserve authored cable topology and only change which runtime body owns each endpoint. `hp-sim-3d/app/setupScene.js:623-686`, `src/js/cable_joints_3d/cable_joints_core.js:36-113`

Follow the non-linear projected Gauss-Seidel / XPBD style already present in the codebase: constraints should work with the latest state and update the solved state immediately. This is already the core pattern in the cable solver and is compatible with the chosen PBDBodies direction. `src/js/cable_joints_3d/cable_joints_core.js:1410-1438`, `src/js/cable_joints_3d/cable_joints_core.js:1453-1499`, `ai_docs/PBDBodies/PBDBodies.md`.

Follow the current separation between authored topology and runtime caches. Attachment points may remain cached and recomputed from current state, but the current state must become body state for rigid groups, not member state. `src/js/cable_joints_3d/cable_joints_core.js:531-637`, `src/js/cable_joints_3d/cable_joints_core.js:1346-1360`

Follow the current previous/final cache pattern conceptually, but move it to rigid-body state for rigid groups. The design should keep the start-of-step / end-of-step reconstruction idea while changing the owner of that state. `hp-sim-3d/app/setupScene.js:774-813`, `src/js/cable_joints_3d/commonSystems.js:480-566`

Do **not** follow the current pattern where `RigidGroupComponent` is only overlay metadata while member entities stay authoritative for dynamics. That pattern is the core mismatch with the target behavior. `src/js/cable_joints/ecs.js:177-189`, `hp-sim-3d/app/setupScene.js:561-579`

Do **not** follow the current pattern where cable solving and rigid-group solving both write onto the same member entities in sequence. That shared mutable member-state handoff could be a source of amplified impulses and energy injection. `hp-sim-3d/app/setupScene.js:788-813`, `src/js/cable_joints_3d/cable_joints_core.js:1410-1438`, `src/js/cable_joints_3d/commonSystems.js:399-450`

Do **not** preserve the scalar-only inertia abstraction for rigid groups. The 3D ECS already flags it as insufficient for general 3D rigid bodies and rigid groups. `src/js/cable_joints_3d/ecs.js:9-18`

## Design Decisions

1. **Rigid body authority**: Introduce a true compound rigid body for each runtime rigid group. The body entity owns pose, velocities, mass, and inertia; members become attached geometry/attachment handles.
2. **Cable ownership**: Resolve cable endpoints to the owning rigid body when an endpoint lies on a rigid-group member. Constraint gradients still use the endpoint’s local offset, but corrections apply to body translation and rotation.
3. **Internal rigidity**: Remove internal `DistancePhysicsJoint`s from `slideprinter_hexagon.usda`, and treat same-body distance constraints as out of scope for runtime solving.
4. **Inertia model**: Introduce full tensor inertia now. Store body-space inertia and derive world-space inverse inertia per step/substep.
5. **Velocity reconstruction**: Reconstruct linear and angular velocity at the rigid-body level from body previous/final pose, then derive member transforms from the solved body pose.
6. **System order**: Replace the current “cable solve on members, then rigid-group patch-up” pattern with direct solving on rigid bodies. `RigidGroupSystem` should stop being a positional repair pass for fully rigid groups and instead become setup/maintenance logic or disappear for those groups.
7. **Genericity**: Build generic compound-body groundwork now, even though the hexagon is the phase-1 proving case.
8. **Simplicity over compatibility**: Prefer simpler code and clearer ownership boundaries over backward compatibility with member-centric behavior.
9. **XPBD stepping**: Keep XPBD compliance and favor small-step solving rather than large-step multi-iteration compensation.

## What We're NOT Doing

We are not trying to preserve backward-compatible member-level dynamics semantics for rigid groups.

We are not implementing hinge joints, spherical joints, frictional contact handling, or velocity-level contact solves as part of this task.

We are not trying to solve every soft/rigid coupling problem in the repo at once. Phase 1 only needs generic rigid-body groundwork plus the cable-to-rigid-body path required by rigid groups.

We are not leaving scalar inertia in place for rigid groups and calling the result “done”.

We are not relying on a final post-projection step to make a soft member cluster “look rigid”.

We are not treating `machine:rigidGroup` on the extruder prim as a required dependency for phase 1, because the current runtime effect of that relationship is unconfirmed. `hp-sim-3d/app/setupScene.js:688-739`

## Open Risks

The largest risk is migration breadth: many systems currently assume bodies are individual ECS entities with directly mutable position/orientation state. `hp-sim-3d/app/setupScene.js:774-817`, `src/js/cable_joints_3d/commonSystems.js:480-566`

Introducing full tensor inertia will touch setup, solve, and velocity reconstruction at once. This is the right direction, but it increases first-pass implementation cost.

Some existing code may rely implicitly on member entities carrying orientation or spool-state updates directly. Those responsibilities need explicit re-homing or derivation from body pose.

The pinhole/spool authored data is asymmetric today: spools carry angular data and inertia tensors, pinholes often do not because they're modelled as points, not bodies. The new rigid-body builder will need a clear rule for composing a body from heterogeneous member prims. `public/usd_scenes/slideprinter_hexagon.usda:14-33`, `public/usd_scenes/slideprinter_hexagon.usda:50-74`, `public/usd_scenes/slideprinter_hexagon.usda:76-94`, `public/usd_scenes/slideprinter_hexagon.usda:112-136`, `public/usd_scenes/slideprinter_hexagon.usda:138-156`, `public/usd_scenes/slideprinter_hexagon.usda:174-198`

The current repo may still contain other rigid-group use cases that implicitly depend on the overlay behavior. Those scenes will need review once the authoritative-body model lands.
