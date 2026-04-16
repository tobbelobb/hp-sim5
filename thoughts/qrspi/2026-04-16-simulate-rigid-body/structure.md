# Structure Outline

## Approach
Replace the current rigid-group overlay with an authoritative compound-body path in small end-to-end slices: first load and render a real rigid body, then solve cables against that body, then move stepping/velocity reconstruction to body state, and finally remove the old double-correction path. One thin horizontal seam is unavoidable: the body/tensor abstractions must exist before later slices can consume them, so Phase 1 establishes that seam **through** `setupScene()` and runtime rendering rather than as a detached refactor.

## Phase 1: Authoritative compound-body loading
This phase makes `setupScene()` create a true rigid-body entity for each rigid group and turns members into attached geometry/attachment handles. The scene should still load and render normally, but ownership is now explicit and future solver work has a stable target.

**Files**: `src/js/cable_joints_3d/ecs.js`, `hp-sim-3d/app/setupScene.js`, `src/js/cable_joints_3d/commonSystems.js`, `tests/js/hp-sim-3d/setupScene3d.test.js`, `tests/js/cable_joints_3d/rigidGroupSystem.test.js` (or replacement body-setup test)

**Key changes**:
- `class RigidBodyComponent { memberEntities: number[]; localFrames: Map<number, LocalFrame>; }` — new
- `class InertiaTensorComponent { bodyInertia: Matrix3; bodyInvInertia: Matrix3; worldInvInertia: Matrix3; }` — new
- `class RigidAttachmentComponent { bodyEntity: number; localPos: Vector3; localOrientation: Quaternion; }` — new
- `buildRigidBodyFromGroup(world: World, groupPrim: UsdPrim, memberEntities: number[]): number` — new
- `syncAttachedMembersFromBody(world: World): void` — new/modified maintenance pass

**Verify**: `npx jest tests/js/hp-sim-3d/setupScene3d.test.js`; manually run `npx vite`, open the 3D app, load `slideprinter_hexagon`, and confirm the scene still loads with the same rest pose and no visible member drift.

---

## Phase 2: Cable endpoints resolve to owning rigid bodies
This phase changes the solver path end to end: cable joints still use authored topology and member-local attachment points, but endpoint ownership resolves to the rigid body when the endpoint lies on a rigid-group member. The hexagon should now move as a single body under cable pull.

**Files**: `src/js/cable_joints_3d/cable_joints_core.js`, `hp-sim-3d/app/setupScene.js`, `src/js/cable_joints_3d/ecs.js`, `tests/js/cable_joints_3d/cableRigidBodySolver.test.js` (new), `tests/js/hp-sim-3d/setupScene3d.test.js`

**Key changes**:
- `type BodyRef = { bodyEntity: number; localPoint: Vector3; localOrientation?: Quaternion }` — new
- `resolveBodyRef(world: World, entityId: number, localPoint?: Vector3): BodyRef` — new
- `computeWorldAttachment(world: World, bodyRef: BodyRef): Vector3` — modified ownership boundary
- `applyBodyCorrection(world: World, bodyEntity: number, deltaPos: Vector3, deltaQuat: Quaternion): void` — new
- `PBDCableConstraintSolver.update(world: World, dt: number): void` — modified to read/write body state rather than member state for grouped endpoints

**Verify**: `npx jest tests/js/cable_joints_3d/cableRigidBodySolver.test.js tests/js/hp-sim-3d/setupScene3d.test.js`; manually run `npx vite`, drive the hexagon, and confirm it translates/rotates coherently with no sponge-like internal deformation.

---

## Phase 3: Body-level stepping, caches, and velocity reconstruction
This phase moves prediction and reconstruction to authoritative body state. Previous/final caches, motion integration, and velocity/angular-velocity reconstruction happen once per rigid body; member transforms are derived after solve.

**Files**: `src/js/cable_joints_3d/commonSystems.js`, `src/js/cable_joints_3d/ecs.js`, `hp-sim-3d/app/setupScene.js`, `hp-sim-3d/app/hangprinter_spools.js` (if spool orientation readers need body-derived transforms), `tests/js/cable_joints_3d/bodyVelocityUpdate.test.js` (new)

**Key changes**:
- `class PrevFinalBodyPoseComponent { pos: Vector3; orientation: Quaternion; }` — new
- `predictRigidBodyMotion(world: World, dt: number): void` — new/modified integration path
- `reconstructRigidBodyVelocity(world: World, dt: number): void` — new
- `reconstructRigidBodyAngularVelocity(world: World, dt: number): void` — new
- `deriveMemberPose(bodyPose: Pose3, attachment: RigidAttachmentComponent): Pose3` — new helper boundary

**Verify**: `npx jest tests/js/cable_joints_3d/bodyVelocityUpdate.test.js tests/js/cable_joints_3d/cableRigidBodySolver.test.js`; manually check in the 3D app that reported/observed motion matches whole-body translation and rotation rather than fake per-member internal motion.

---

## Phase 4: Remove double-correction paths and stabilize migrated scenes
This phase retires the old overlay behavior for migrated rigid groups: no `RigidGroupSystem` positional patch-up on authoritative groups, and no redundant internal fixed-length constraints between members of the same rigid body. That should eliminate the current stacked-rigidity/energy-injection path.

**Files**: `hp-sim-3d/app/setupScene.js`, `src/js/cable_joints_3d/commonSystems.js`, `src/js/cable_joints_3d/cable_joints_core.js`, `tests/js/cable_joints_3d/rigidGroupSystem.test.js`, `tests/js/hp-sim-3d/setupScene3d.test.js`

**Key changes**:
- `isAuthoritativeRigidBody(entityId: number): boolean` — new helper
- `shouldCreateDistanceConstraint(entityA: number, entityB: number): boolean` — modified to skip same-body internals
- `RigidGroupSystem.update(world: World, dt: number): void` — removed, narrowed to non-migrated legacy groups, or reduced to setup-only logic
- `registerSimulationSystems(world: World): void` — modified ordering so body solve replaces "cable solve on members, then rigid-group repair"

**Verify**: `npx jest`; manually run `npx vite`, stress the hexagon under sustained pulls, and confirm stable motion without double-stiffening, visible internal jitter, or correction explosions.

## Testing Checkpoints
After Phase 1: `setupScene()` produces a body-owned rigid-group runtime model, and the scene still renders correctly at rest.

After Phase 2: cable forces act on rigid bodies directly, and `slideprinter_hexagon` moves as one rigid object under pull.

After Phase 3: start/end-of-step caches and reconstructed velocities are body-level, with member poses derived from body pose.

After Phase 4: the old overlay correction path is gone (or isolated to legacy scenes), redundant same-body constraints are skipped, and the migrated scene is stable under repeated solves.
