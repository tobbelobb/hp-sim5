# Implementation Plan

## Overview
Convert rigid groups from a post-solve member overlay into authoritative compound rigid bodies with body-owned pose, velocity, mass, and full inertia, while keeping authored cable topology and the current USD loading path intact. The target runtime behavior is that grouped assemblies such as `slideprinter_hexagon` move as one rigid body under cable pull, with member poses derived from body state instead of repaired after the fact.

## Phase 1: Authoritative compound-body loading

### Changes

#### 1. 3D ECS body/attachment primitives
**File**: `src/js/cable_joints_3d/ecs.js`
**Action**: modify

Add the missing 3D rigid-body primitives directly in the 3D ECS module so later phases can import them without introducing a separate utility file.

```js
export class Matrix3 {
  constructor(elements = null) {
    this.elements = elements
      ? [...elements]
      : [1, 0, 0,
         0, 1, 0,
         0, 0, 1];
  }

  clone() { return new Matrix3(this.elements); }
  setIdentity() { /* set diagonal = 1 */ return this; }
  add(other) { /* elementwise */ return this; }
  scale(s) { /* elementwise */ return this; }
  multiplyVector(v) { /* row-major 3x3 * Vector3 */ }
  transpose() { /* return new Matrix3 */ }
  multiplyMatrix(other) { /* return new Matrix3 */ }
  inverseSymmetricOrZero() { /* robust inverse for positive semidefinite body inertia */ }

  static zero() { return new Matrix3([0,0,0, 0,0,0, 0,0,0]); }
  static identity() { return new Matrix3(); }
  static fromOuterProduct(v) {
    return new Matrix3([
      v.x * v.x, v.x * v.y, v.x * v.z,
      v.y * v.x, v.y * v.y, v.y * v.z,
      v.z * v.x, v.z * v.y, v.z * v.z,
    ]);
  }
}

export class RigidBodyComponent {
  constructor(memberEntities = [], localFrames = new Map()) {
    this.memberEntities = memberEntities;
    this.localFrames = localFrames; // Map<memberEntityId, LocalFrame>
  }
}

export class InertiaTensorComponent {
  constructor(bodyInertia = Matrix3.identity()) {
    this.bodyInertia = bodyInertia.clone();
    this.bodyInvInertia = bodyInertia.clone().inverseSymmetricOrZero();
    this.worldInvInertia = this.bodyInvInertia.clone();
  }
}

export class RigidAttachmentComponent {
  constructor(bodyEntity, localPos, localOrientation = new Quaternion()) {
    this.bodyEntity = bodyEntity;
    this.localPos = localPos.clone();
    this.localOrientation = localOrientation.clone().normalize();
  }
}
```

Implementation notes:
- Keep the existing re-exports from `src/js/cable_joints/ecs.js`; do **not** edit the 2D ECS file.
- Keep `MomentOfInertiaComponent` for non-compound legacy entities; `InertiaTensorComponent` is additive, not a replacement in this phase.
- Use a plain `{ localPos, localOrientation, preserveSpoolTwist }` value in `RigidBodyComponent.localFrames` so later phases can reconstruct member pose without re-reading USD data.

#### 2. Runtime rigid-body construction during scene load
**File**: `hp-sim-3d/app/setupScene.js`
**Action**: modify

Upgrade the current rigid-group creation path so the group entity itself becomes the authoritative body entity instead of a metadata-only overlay. Keep the authored member entities for rendering, cable-path topology, spool state, and attachment handles.

Add imports:

```js
import {
  PositionComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  GravityAffectedComponent,
  EncoderComponent,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  PrevFinalOrientationComponent,
  PrevFinalPosComponent,
  RestitutionComponent,
  CoefficientOfFrictionComponent,
  SimulationErrorStateComponent,
  RenderableComponent,
  DistanceConstraintComponent,
  RigidGroupComponent,
  MachineTagComponent,
  RigidBodyComponent,
  InertiaTensorComponent,
  RigidAttachmentComponent,
  Matrix3,
} from "../../src/js/cable_joints_3d/ecs.js";
```

Add helper functions near the existing USD attribute readers:

```js
function matrixFromUsdInertiaTensor(rawTensor) {
  if (!Array.isArray(rawTensor) || rawTensor.length < 3) {
    return Matrix3.zero();
  }
  return new Matrix3([
    Number(rawTensor[0]?.[0] ?? 0.0), Number(rawTensor[0]?.[1] ?? 0.0), Number(rawTensor[0]?.[2] ?? 0.0),
    Number(rawTensor[1]?.[0] ?? 0.0), Number(rawTensor[1]?.[1] ?? 0.0), Number(rawTensor[1]?.[2] ?? 0.0),
    Number(rawTensor[2]?.[0] ?? 0.0), Number(rawTensor[2]?.[1] ?? 0.0), Number(rawTensor[2]?.[2] ?? 0.0),
  ]);
}

function rotationMatrixFromQuaternion(quaternion) {
  // Return Matrix3 R from unit quaternion.
}

function rotateBodyTensorToWorld(bodyTensor, orientation) {
  const R = rotationMatrixFromQuaternion(orientation);
  return R.multiplyMatrix(bodyTensor).multiplyMatrix(R.transpose());
}

function parallelAxisTerm(mass, offset) {
  const r2 = offset.lengthSq();
  return Matrix3.identity().scale(mass * r2)
    .add(Matrix3.fromOuterProduct(offset).scale(-mass));
}

function buildRigidBodyFromGroup(world, groupPrim, memberEntities, machineId, palette) {
  // 1. Compute COM from positive-mass members; fall back to average position if all masses are static/nonpositive.
  // 2. Create/upgrade the group entity.
  // 3. Seed body Position/Orientation/Velocity/AngularVelocity/Mass/InertiaTensor.
  // 4. Add RigidAttachmentComponent to each member and store local frames on the body.
  // 5. Keep RigidGroupComponent only for renderSegments + legacy metadata.
  // 6. Return the group/body entity id.
}
```

Inside `buildRigidBodyFromGroup(...)`, use this mass-properties pattern:

```js
const bodyOrientation = new Quaternion(); // identity in Phase 1
const bodyPosition = computedCom;
const localFrames = new Map();
let totalMass = 0.0;
let bodyInertia = Matrix3.zero();

for (const memberEntity of memberEntities) {
  const pos = world.getComponent(memberEntity, PositionComponent)?.pos?.clone();
  const memberOrientation = world.getComponent(memberEntity, OrientationComponent)?.quaternion?.clone() || new Quaternion();
  const memberMass = world.getComponent(memberEntity, MassComponent)?.mass ?? 0.0;
  const localPos = pos.clone().subtract(bodyPosition);
  const localOrientation = memberOrientation.clone();

  localFrames.set(memberEntity, {
    localPos,
    localOrientation,
    preserveSpoolTwist: world.hasComponent(memberEntity, SpoolTagComponent),
  });
  world.addComponent(memberEntity, new RigidAttachmentComponent(groupEnt, localPos, localOrientation));

  if (memberMass > 0.0) {
    totalMass += memberMass;

    const memberTensor = world.hasComponent(memberEntity, MomentOfInertiaComponent)
      ? new Matrix3([
          world.getComponent(memberEntity, MomentOfInertiaComponent).inertia, 0, 0,
          0, world.getComponent(memberEntity, MomentOfInertiaComponent).inertia, 0,
          0, 0, world.getComponent(memberEntity, MomentOfInertiaComponent).inertia,
        ])
      : Matrix3.zero();

    const rotatedMemberTensor = rotateBodyTensorToWorld(memberTensor, localOrientation);
    bodyInertia.add(rotatedMemberTensor).add(parallelAxisTerm(memberMass, localPos));
  }
}
```

Attach these authoritative components to the rigid-group entity:

```js
world.addComponent(groupEnt, new RigidBodyComponent(memberEntities, localFrames));
world.addComponent(groupEnt, new PositionComponent(bodyPosition.x, bodyPosition.y, bodyPosition.z));
world.addComponent(groupEnt, new OrientationComponent(0, 0, 0, 1));
world.addComponent(groupEnt, new VelocityComponent(avgVel.x, avgVel.y, avgVel.z));
world.addComponent(groupEnt, new AngularVelocityComponent(0, 0, 0));
world.addComponent(groupEnt, new MassComponent(totalMass > 0.0 ? totalMass : -1.0));
world.addComponent(groupEnt, new InertiaTensorComponent(bodyInertia));
```

Important constraints for this phase:
- Do **not** remove the member entities.
- Do **not** change cable-joint topology yet.
- Do **not** remove the existing member components yet; later phases still consume them.
- Do call `syncAttachedMembersFromBody(world)` once at the end of scene construction, immediately before the final `renderSystem.update(world, 0)` call.

#### 3. Body-to-member maintenance pass
**File**: `src/js/cable_joints_3d/commonSystems.js`
**Action**: modify

Add a body-driven pose derivation helper and an exported sync pass. This is the Phase 1 seam that later solver and velocity changes will consume.

Add imports:

```js
import {
  EncoderComponent,
  OrientationComponent,
  PrevFinalOrientationComponent,
  AngularVelocityComponent,
  VelocityComponent,
  GravityAffectedComponent,
  PositionComponent,
  PrevFinalPosComponent,
  MassComponent,
  MomentOfInertiaComponent,
  DistanceConstraintComponent,
  RigidGroupComponent,
  RigidBodyComponent,
  RigidAttachmentComponent,
  InertiaTensorComponent,
} from './ecs.js';

import {
  SpoolStateComponent,
  rotateSpoolReferenceOrientation,
  getSpoolRotationAngle,
  getSpoolWorldAxis,
  decomposeSpoolOrientation,
  composeSpoolOrientation,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';
```

Add helpers:

```js
export function deriveMemberPose(bodyPose, attachment) {
  const rotatedLocalPos = bodyPose.orientation.transformVector(attachment.localPos);
  const worldPos = bodyPose.pos.clone().add(rotatedLocalPos);
  const worldOrientation = bodyPose.orientation.clone()
    .multiplyQuaternions(bodyPose.orientation, attachment.localOrientation)
    .normalize();
  return { pos: worldPos, orientation: worldOrientation };
}

export function syncAttachedMembersFromBody(world) {
  const bodyEntities = world.query([RigidBodyComponent, PositionComponent, OrientationComponent]);
  for (const bodyEntity of bodyEntities) {
    const body = world.getComponent(bodyEntity, RigidBodyComponent);
    const bodyPos = world.getComponent(bodyEntity, PositionComponent).pos;
    const bodyOrientation = world.getComponent(bodyEntity, OrientationComponent).quaternion;

    for (const memberEntity of body.memberEntities) {
      const attachment = world.getComponent(memberEntity, RigidAttachmentComponent);
      if (!attachment || attachment.bodyEntity !== bodyEntity) continue;

      const nextPose = deriveMemberPose(
        { pos: bodyPos, orientation: bodyOrientation },
        attachment,
      );

      const posComp = world.getComponent(memberEntity, PositionComponent);
      if (posComp) posComp.pos.set(nextPose.pos);

      const orientationComp = world.getComponent(memberEntity, OrientationComponent);
      if (!orientationComp) continue;

      const spoolState = world.getComponent(memberEntity, SpoolStateComponent);
      if (!spoolState) {
        orientationComp.quaternion.set(nextPose.orientation);
        continue;
      }

      const { swing, angle } = decomposeSpoolOrientation(spoolState, orientationComp.quaternion);
      spoolState.referenceOrientation.set(nextPose.orientation);
      orientationComp.quaternion.set(composeSpoolOrientation(spoolState, swing, angle));
    }
  }
}
```

Also add a tiny system wrapper so later phases can insert the sync pass in the frame order without duplicating logic:

```js
export class SyncAttachedMembersFromBodySystem {
  runInPause = false;
  update(world, _dt) {
    syncAttachedMembersFromBody(world);
  }
}
```

Do **not** change `RigidGroupSystem` behavior yet in Phase 1.

#### 4. Scene setup test coverage for authoritative body creation
**File**: `tests/js/hp-sim-3d/setupScene3d.test.js`
**Action**: modify

Extend the current mocked USD stage so one test covers a real rigid-group import.

Add mocked rigid-group data:

```js
usdStage.getChildren.mockImplementation((prim) => {
  if (prim?.path === '/World/SlideprinterScene') {
    return [
      spoolPrim('SpoolA', [0.2, 0.3, 0.0]),
      pinholePrim('PinholeAL', [0.1, 0.2, 0.003]),
      rigidGroupPrim('SpoolTriangle'),
    ];
  }
  return [];
});

usdStage.getRelationship.mockImplementation((prim, rel) => {
  if (prim?.path === '/World/SlideprinterScene/SpoolTriangle' && rel === 'rigidGroup:members') {
    return [
      '/World/SlideprinterScene/SpoolA',
      '/World/SlideprinterScene/PinholeAL',
    ];
  }
  return [];
});
```

Add assertions:

```js
const rigidBodies = world.query([RigidBodyComponent, InertiaTensorComponent]);
expect(rigidBodies).toHaveLength(1);

const bodyEntity = rigidBodies[0];
const body = world.getComponent(bodyEntity, RigidBodyComponent);
expect(body.memberEntities).toEqual(expect.arrayContaining([spoolEntity, pinholeEntity]));

const spoolAttachment = world.getComponent(spoolEntity, RigidAttachmentComponent);
expect(spoolAttachment.bodyEntity).toBe(bodyEntity);
expect(world.getComponent(bodyEntity, MassComponent).mass).toBeGreaterThan(0.0);
```

Keep the existing render-system assertions intact.

#### 5. Body-setup regression test for member sync
**File**: `tests/js/cable_joints_3d/rigidGroupSystem.test.js`
**Action**: modify

Retain the file path, but repurpose its first test to cover the new body-setup seam instead of the old overlay-only assumption.

Add a test like this:

```js
test('syncAttachedMembersFromBody derives member pose from the body pose', () => {
  const world = new World();
  const body = world.createEntity();
  const member = world.createEntity();

  world.addComponent(body, new PositionComponent(1, 2, 3));
  world.addComponent(body, new OrientationComponent(0, 0, 0, 1));
  world.addComponent(body, new RigidBodyComponent([member], new Map()));

  world.addComponent(member, new PositionComponent());
  world.addComponent(member, new OrientationComponent());
  world.addComponent(member, new RigidAttachmentComponent(
    body,
    new Vector3(0.5, 0.0, 0.0),
    new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 2),
  ));

  syncAttachedMembersFromBody(world);

  expect(world.getComponent(member, PositionComponent).pos.x).toBeCloseTo(1.5, 8);
  expect(world.getComponent(member, PositionComponent).pos.y).toBeCloseTo(2.0, 8);
});
```

Keep the existing spool-orientation compatibility coverage, but make it run through `syncAttachedMembersFromBody(...)` instead of `RigidGroupSystem.update(...)`.

### Verification
#### Automated
- [ ] `npx jest tests/js/hp-sim-3d/setupScene3d.test.js`
- [ ] `npx jest tests/js/cable_joints_3d/rigidGroupSystem.test.js`

#### Manual
- [ ] `npx vite`
- [ ] Open the 3D app, load `slideprinter_hexagon`, and confirm the scene still renders in the same rest pose.
- [ ] Confirm the rigid-group line rendering still appears and member meshes do not visibly drift away from the hexagon at rest.

---

## Phase 2: Cable endpoints resolve to owning rigid bodies

### Changes

#### 1. Body-reference ownership boundary in the cable solver
**File**: `src/js/cable_joints_3d/cable_joints_core.js`
**Action**: modify

Keep authored cable topology member-centric, but make the solver resolve each endpoint to its owning body before computing mass/inertia and applying corrections.

Add a documented body-ref shape near the top of the file:

```js
/**
 * @typedef {Object} BodyRef
 * @property {number} bodyEntity
 * @property {Vector3} localPoint
 * @property {Quaternion | null} localOrientation
 */
```

Replace `_computeWorldAttachment(...)` with a body-ref version and add a resolver:

```js
export function resolveBodyRef(world, entityId, localPoint = new Vector3(0, 0, 0)) {
  const attachment = world.getComponent(entityId, RigidAttachmentComponent);
  if (!attachment) {
    return {
      bodyEntity: entityId,
      localPoint: localPoint.clone(),
      localOrientation: null,
    };
  }

  const rotatedLocal = attachment.localOrientation.transformVector(localPoint);
  return {
    bodyEntity: attachment.bodyEntity,
    localPoint: attachment.localPos.clone().add(rotatedLocal),
    localOrientation: attachment.localOrientation.clone(),
  };
}

export function computeWorldAttachment(world, bodyRef) {
  const posComp = world.getComponent(bodyRef.bodyEntity, PositionComponent);
  const orientComp = world.getComponent(bodyRef.bodyEntity, OrientationComponent);
  if (!posComp) return bodyRef.localPoint.clone();
  if (!orientComp) return posComp.pos.clone().add(bodyRef.localPoint);
  return posComp.pos.clone().add(
    orientComp.quaternion.transformVector(bodyRef.localPoint),
  );
}
```

Add rigid-body correction application:

```js
export function applyBodyCorrection(world, bodyEntity, deltaPos, deltaQuat) {
  const posComp = world.getComponent(bodyEntity, PositionComponent);
  if (posComp && deltaPos) {
    posComp.pos.add(deltaPos);
  }

  const orientComp = world.getComponent(bodyEntity, OrientationComponent);
  if (orientComp && deltaQuat) {
    orientComp.quaternion.multiplyQuaternions(deltaQuat, orientComp.quaternion).normalize();
  }

  const inertia = world.getComponent(bodyEntity, InertiaTensorComponent);
  if (inertia && orientComp) {
    inertia.worldInvInertia = rotateBodyTensorToWorld(
      inertia.bodyInvInertia,
      orientComp.quaternion,
    );
  }
}
```

Update the solver snapshot and solve path:

```js
const jointRefs = new Map();
for (const jointId of path.jointEntities) {
  const joint = world.getComponent(jointId, CableJointComponent);
  jointRefs.set(jointId, {
    refA: resolveBodyRef(world, joint.entityA, computeLocal(joint.entityA, joint.attachmentPointA_world)),
    refB: resolveBodyRef(world, joint.entityB, computeLocal(joint.entityB, joint.attachmentPointB_world)),
  });
}
```

In the inner solver loop:

```js
const { refA, refB } = jointRefs.get(jointId);
if (refA.bodyEntity === refB.bodyEntity) {
  continue; // Internal same-body cable span should not self-correct the body.
}

const pA = computeWorldAttachment(world, refA);
const pB = computeWorldAttachment(world, refB);
```

Replace the scalar angular denominator with a tensor-aware one for rigid bodies, while preserving the old scalar path for standalone entities:

```js
function inverseMass(world, entityId) {
  const mass = world.getComponent(entityId, MassComponent)?.mass ?? 0.0;
  return mass > 0.0 ? 1.0 / mass : 0.0;
}

function inverseAngularMetric(world, entityId, gradAng) {
  const bodyTensor = world.getComponent(entityId, InertiaTensorComponent);
  if (bodyTensor) {
    const worldInv = bodyTensor.worldInvInertia ?? bodyTensor.bodyInvInertia;
    const rotatedGrad = worldInv.multiplyVector(gradAng);
    return {
      metric: gradAng.dot(rotatedGrad),
      rotationStep: rotatedGrad,
    };
  }

  const scalarMoi = world.getComponent(entityId, MomentOfInertiaComponent)?.invInertia ?? 0.0;
  return {
    metric: scalarMoi * gradAng.lengthSq(),
    rotationStep: gradAng.clone().scale(scalarMoi),
  };
}
```

Then apply the body correction with rotation-vector-to-quaternion conversion:

```js
const angA = inverseAngularMetric(world, refA.bodyEntity, gradAngA);
const angB = inverseAngularMetric(world, refB.bodyEntity, gradAngB);
const denom = invMassA + angA.metric + invMassB + angB.metric + ((compliance ?? 0.0) / (dt * dt));
const lambda = -constraintError / denom;

applyBodyCorrection(
  world,
  refA.bodyEntity,
  gradPosA.clone().scale(-invMassA * lambda),
  deltaQuatFromRotationVector(angA.rotationStep.clone().scale(-lambda)),
);
applyBodyCorrection(
  world,
  refB.bodyEntity,
  gradPosB.clone().scale(-invMassB * lambda),
  deltaQuatFromRotationVector(angB.rotationStep.clone().scale(-lambda)),
);
```

Do **not** rewrite `CableAttachmentUpdateSystem`; it should keep tracing authored member entities.

#### 2. Scene system order for body-driven endpoint motion
**File**: `hp-sim-3d/app/setupScene.js`
**Action**: modify

Insert the new sync pass so member entities reflect body pose both before cable geometry updates and after body corrections.

In the local-simulation registration block, import and register the system wrapper:

```js
import {
  PrevFinalPosSystem,
  PrevFinalOrientationSystem,
  EncoderUpdateSystem,
  GravitySystem,
  MovementSystem,
  AngularMovementSystem,
  PBDVelocityUpdateSystem,
  PBDAngularVelocityUpdateSystem,
  XPBDDistanceConstraintSystem,
  RigidGroupSystem,
  SyncAttachedMembersFromBodySystem,
} from '../../src/js/cable_joints_3d/commonSystems.js';
```

Insert it in two places:

```js
world.registerSystem(new GravitySystem());
world.registerSystem(new MovementSystem());
world.registerSystem(new AngularMovementSystem());
world.registerSystem(new SyncAttachedMembersFromBodySystem());

world.registerSystem(new CableAttachmentUpdateSystem());
world.registerSystem(new CableAttachmentCacheSystem());
world.registerSystem(new CableSlackSystem());

world.registerSystem(new PBDCableConstraintSolver());
world.registerSystem(new PBDResolveCableOverCorrections());
world.registerSystem(new SyncAttachedMembersFromBodySystem());
```

Leave `RigidGroupSystem` in place for now; Phase 4 removes the old double-correction path.

#### 3. Shared 3D ECS typings used by the solver
**File**: `src/js/cable_joints_3d/ecs.js`
**Action**: modify

Add any small exports needed by the solver without adding a new file, for example:

```js
export function isAttachedToRigidBody(world, entityId) {
  return world.hasComponent(entityId, RigidAttachmentComponent);
}
```

This is optional glue only; keep the main ownership logic in `cable_joints_core.js`.

#### 4. Direct solver regression test for rigid-body cable pull
**File**: `tests/js/cable_joints_3d/cableRigidBodySolver.test.js`
**Action**: create

Create a focused rigid-body cable test with no USD loader involved.

Test fixture outline:

```js
const world = new World();
const body = makeRigidBody(world, {
  pos: new Vector3(0, 0, 0),
  mass: 2.0,
  inertia: diagonalTensor(0.25, 0.25, 0.5),
});
const memberLeft = attachMember(world, body, new Vector3(-0.5, 0, 0));
const memberRight = attachMember(world, body, new Vector3(0.5, 0, 0));
const anchor = makeStaticPoint(world, new Vector3(0, 1.0, 0));

const joint = makeCableJoint(world, anchor, memberRight, restLengthShorterThanCurrentSpan);
const path = makeCablePath(world, [joint]);

new PBDCableConstraintSolver().update(world, 1 / 120);
syncAttachedMembersFromBody(world);

expect(world.getComponent(body, PositionComponent).pos.y).toBeGreaterThan(0.0);
expect(memberSeparationStillRigid(world, memberLeft, memberRight)).toBe(true);
```

Assertions to include:
- body position changed,
- body orientation changed when the cable line of action is off-center,
- member world distance still equals the attachment rest distance after syncing,
- attached member entities themselves were **not** the mass/inertia owners used by the solver.

#### 5. Loader regression for body-owned endpoints
**File**: `tests/js/hp-sim-3d/setupScene3d.test.js`
**Action**: modify

Add one assertion block that proves a cable-joint endpoint on a rigid-group member resolves to a body-owned endpoint during solve.

Simplest path:
- mock a rigid body member plus one cable joint on that member,
- call `setupScene(...)`,
- run one `PBDCableConstraintSolver().update(world, dt)` step,
- assert the rigid body entity moved while the member motion still matches body-derived pose after sync.

### Verification
#### Automated
- [ ] `npx jest tests/js/cable_joints_3d/cableRigidBodySolver.test.js tests/js/hp-sim-3d/setupScene3d.test.js`

#### Manual
- [ ] `npx vite`
- [ ] Load `slideprinter_hexagon` and drive one spool/axis enough to tension a cable.
- [ ] Confirm the hexagon translates and rotates as one coherent rigid object.
- [ ] Confirm there is no visible sponge-like internal deformation between spools and pinholes.

---

## Phase 3: Body-level stepping, caches, and velocity reconstruction

### Changes

#### 1. Body-level previous/final cache component
**File**: `src/js/cable_joints_3d/ecs.js`
**Action**: modify

Add the body-pose cache component used for whole-body velocity reconstruction.

```js
export class PrevFinalBodyPoseComponent {
  constructor(pos = new Vector3(), orientation = new Quaternion()) {
    this.pos = pos.clone();
    this.orientation = orientation.clone().normalize();
  }
}
```

Keep the existing `PrevFinalPosComponent` and `PrevFinalOrientationComponent` for standalone, non-attached entities.

#### 2. Body prediction, derived member pose, and body velocity reconstruction
**File**: `src/js/cable_joints_3d/commonSystems.js`
**Action**: modify

Add body-level cache/prediction/reconstruction helpers and make the existing member-level systems skip attached members.

Add helpers:

```js
function isAttachedMember(world, entityId) {
  return world.hasComponent(entityId, RigidAttachmentComponent);
}

function isAuthoritativeBody(world, entityId) {
  return world.hasComponent(entityId, RigidBodyComponent);
}

export function predictRigidBodyMotion(world, dt) {
  const bodies = world.query([RigidBodyComponent, PositionComponent, VelocityComponent]);
  for (const bodyId of bodies) {
    const pos = world.getComponent(bodyId, PositionComponent).pos;
    const vel = world.getComponent(bodyId, VelocityComponent).vel;
    pos.add(vel, dt);

    const orientation = world.getComponent(bodyId, OrientationComponent)?.quaternion;
    const omega = world.getComponent(bodyId, AngularVelocityComponent)?.omega;
    if (orientation && omega && omega.lengthSq() > 1e-12) {
      const angle = omega.length() * dt;
      const axis = omega.clone().normalize();
      const dq = new Quaternion().setFromAxisAngle(axis, angle);
      orientation.multiplyQuaternions(dq, orientation).normalize();
    }

    const inertia = world.getComponent(bodyId, InertiaTensorComponent);
    if (inertia && orientation) {
      inertia.worldInvInertia = rotateBodyTensorToWorld(inertia.bodyInvInertia, orientation);
    }
  }
}

export function reconstructRigidBodyVelocity(world, dt) {
  const bodies = world.query([RigidBodyComponent, PositionComponent, VelocityComponent, PrevFinalBodyPoseComponent]);
  for (const bodyId of bodies) {
    const pos = world.getComponent(bodyId, PositionComponent).pos;
    const prev = world.getComponent(bodyId, PrevFinalBodyPoseComponent).pos;
    world.getComponent(bodyId, VelocityComponent).vel
      .subtractVectors(pos, prev)
      .scale(1.0 / dt);
  }
}

export function reconstructRigidBodyAngularVelocity(world, dt) {
  const bodies = world.query([RigidBodyComponent, OrientationComponent, AngularVelocityComponent, PrevFinalBodyPoseComponent]);
  for (const bodyId of bodies) {
    const qCurr = world.getComponent(bodyId, OrientationComponent).quaternion;
    const qPrev = world.getComponent(bodyId, PrevFinalBodyPoseComponent).orientation;
    const qDelta = qCurr.clone().multiplyQuaternions(qCurr, qPrev.clone().conjugate().normalize()).normalize();
    // Same quaternion-delta-to-omega math as the existing PBDAngularVelocityUpdateSystem.
  }
}
```

Add system wrappers:

```js
export class PrevFinalBodyPoseSystem {
  runInPause = false;
  update(world, _dt) {
    const bodies = world.query([RigidBodyComponent, PositionComponent, OrientationComponent, PrevFinalBodyPoseComponent]);
    for (const bodyId of bodies) {
      const cache = world.getComponent(bodyId, PrevFinalBodyPoseComponent);
      cache.pos.set(world.getComponent(bodyId, PositionComponent).pos);
      cache.orientation.set(world.getComponent(bodyId, OrientationComponent).quaternion);
    }
  }
}

export class RigidBodyPredictionSystem {
  runInPause = false;
  update(world, dt) { predictRigidBodyMotion(world, dt); }
}

export class RigidBodyVelocityUpdateSystem {
  runInPause = false;
  update(world, dt) { reconstructRigidBodyVelocity(world, dt); }
}

export class RigidBodyAngularVelocityUpdateSystem {
  runInPause = false;
  update(world, dt) { reconstructRigidBodyAngularVelocity(world, dt); }
}
```

Modify the existing systems as follows:
- `MovementSystem`: skip entities that have `RigidAttachmentComponent`; keep handling standalone entities only.
- `AngularMovementSystem`: same skip rule.
- `PrevFinalPosSystem` / `PrevFinalOrientationSystem`: skip attached members and rigid bodies that now have `PrevFinalBodyPoseComponent`.
- `PBDVelocityUpdateSystem` / `PBDAngularVelocityUpdateSystem`: skip attached members and rigid bodies; they remain the standalone-body fallback path.

Extend `syncAttachedMembersFromBody(world)` so it also derives member velocity and angular velocity after body reconstruction:

```js
const bodyVel = world.getComponent(bodyEntity, VelocityComponent)?.vel || new Vector3();
const bodyOmega = world.getComponent(bodyEntity, AngularVelocityComponent)?.omega || new Vector3();
const rWorld = nextPose.pos.clone().subtract(bodyPos);

const memberVel = bodyVel.clone().add(bodyOmega.clone().cross(rWorld));
world.getComponent(memberEntity, VelocityComponent)?.vel?.set(memberVel);

const memberOmega = bodyOmega.clone();
// For spools, add the preserved twist-rate around the current spool world axis if one is present.
world.getComponent(memberEntity, AngularVelocityComponent)?.omega?.set(memberOmega);
```

#### 3. Seed body pose caches during scene load and switch stepping order to body authority
**File**: `hp-sim-3d/app/setupScene.js`
**Action**: modify

When creating a rigid body in `buildRigidBodyFromGroup(...)`, add the new cache component:

```js
world.addComponent(
  groupEnt,
  new PrevFinalBodyPoseComponent(bodyPosition.clone(), new Quaternion()),
);
```

Update imports and local simulation registration order:

```js
import {
  PrevFinalPosSystem,
  PrevFinalOrientationSystem,
  PrevFinalBodyPoseSystem,
  EncoderUpdateSystem,
  GravitySystem,
  MovementSystem,
  AngularMovementSystem,
  RigidBodyPredictionSystem,
  PBDVelocityUpdateSystem,
  PBDAngularVelocityUpdateSystem,
  RigidBodyVelocityUpdateSystem,
  RigidBodyAngularVelocityUpdateSystem,
  XPBDDistanceConstraintSystem,
  RigidGroupSystem,
  SyncAttachedMembersFromBodySystem,
} from '../../src/js/cable_joints_3d/commonSystems.js';
```

Replace the middle of the system order with this sequence:

```js
world.registerSystem(new PrevFinalPosSystem());
world.registerSystem(new PrevFinalOrientationSystem());
world.registerSystem(new PrevFinalBodyPoseSystem());

world.registerSystem(new RemoteSpoolSystem());
world.registerSystem(new StepperMotorSystem());

world.registerSystem(new GravitySystem());
world.registerSystem(new MovementSystem());
world.registerSystem(new AngularMovementSystem());
world.registerSystem(new RigidBodyPredictionSystem());
world.registerSystem(new SyncAttachedMembersFromBodySystem());

world.registerSystem(new CableAttachmentUpdateSystem());
world.registerSystem(new CableAttachmentCacheSystem());
world.registerSystem(new CableSlackSystem());
world.registerSystem(new PBDCableConstraintSolver());
world.registerSystem(new PBDResolveCableOverCorrections());
world.registerSystem(new SyncAttachedMembersFromBodySystem());

world.registerSystem(new RigidGroupSystem()); // legacy-only by Phase 4
world.registerSystem(new SpoolAxisConstraintSystem());
world.registerSystem(new CableFrictionSystem());

world.registerSystem(new PBDVelocityUpdateSystem());
world.registerSystem(new PBDAngularVelocityUpdateSystem());
world.registerSystem(new RigidBodyVelocityUpdateSystem());
world.registerSystem(new RigidBodyAngularVelocityUpdateSystem());
world.registerSystem(new SyncAttachedMembersFromBodySystem());
```

#### 4. Spool helpers for body-derived reference orientation
**File**: `hp-sim-3d/app/hangprinter_spools.js`
**Action**: modify

Add a small helper used by `syncAttachedMembersFromBody(...)` so body motion preserves spool twist cleanly instead of reimplementing that composition logic inline.

```js
export function composeAttachedSpoolOrientation(baseReferenceOrientation, spoolState, currentOrientation) {
  const { swing, angle } = decomposeSpoolOrientation(spoolState, currentOrientation);
  spoolState.referenceOrientation.set(baseReferenceOrientation);
  return composeSpoolOrientation(spoolState, swing, angle);
}
```

Then `syncAttachedMembersFromBody(...)` should call that helper for attached spool members instead of mutating the reference orientation and recomposing manually.

#### 5. Body-level velocity regression test
**File**: `tests/js/cable_joints_3d/bodyVelocityUpdate.test.js`
**Action**: create

Create a direct regression for whole-body caches and reconstructed velocities.

Test outline:

```js
test('reconstructs rigid-body linear and angular velocity from body pose delta', () => {
  const world = new World();
  const body = makeRigidBodyWithCache(world, {
    pos: new Vector3(0, 0, 0),
    orientation: new Quaternion(),
  });

  world.getComponent(body, PrevFinalBodyPoseComponent).pos.set(new Vector3(0, 0, 0));
  world.getComponent(body, PrevFinalBodyPoseComponent).orientation.set(new Quaternion());
  world.getComponent(body, PositionComponent).pos.set(new Vector3(0.1, -0.2, 0.0));
  world.getComponent(body, OrientationComponent).quaternion
    .set(new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 6));

  reconstructRigidBodyVelocity(world, 0.5);
  reconstructRigidBodyAngularVelocity(world, 0.5);

  expect(world.getComponent(body, VelocityComponent).vel.x).toBeCloseTo(0.2, 8);
  expect(world.getComponent(body, VelocityComponent).vel.y).toBeCloseTo(-0.4, 8);
  expect(world.getComponent(body, AngularVelocityComponent).omega.z).toBeCloseTo((Math.PI / 6) / 0.5, 8);
});
```

Add one attached-member assertion proving that after `syncAttachedMembersFromBody(world)`, the member pose is derived from the body pose and not from any independent member cache.

### Verification
#### Automated
- [ ] `npx jest tests/js/cable_joints_3d/bodyVelocityUpdate.test.js tests/js/cable_joints_3d/cableRigidBodySolver.test.js`

#### Manual
- [ ] `npx vite`
- [ ] Load the 3D app and move the hexagon under cable pull.
- [ ] Confirm observed motion is whole-body translation/rotation, not fake per-member internal motion.
- [ ] Confirm spool orientation still follows body tilt while preserving visible spool spin around its own axis.

---

## Phase 4: Remove double-correction paths and stabilize migrated scenes

### Changes

#### 1. Final scene registration/order cleanup and same-body constraint skipping
**File**: `hp-sim-3d/app/setupScene.js`
**Action**: modify

Extract the system-registration block into a helper so the final ordering is explicit and testable.

```js
function registerSimulationSystems(world, { canvas, pauseBtn, isRemote }) {
  // contains the final system order for local mode
}
```

Use it in place of the inline `if (world.systems.length === 0) { ... }` block.

Add helper logic near distance-joint creation:

```js
function isAuthoritativeRigidBody(world, entityId) {
  return world.hasComponent(entityId, RigidBodyComponent);
}

function bodyOwnerForConstraint(world, entityId) {
  return world.getComponent(entityId, RigidAttachmentComponent)?.bodyEntity ?? entityId;
}

function shouldCreateDistanceConstraint(world, entityA, entityB) {
  return bodyOwnerForConstraint(world, entityA) !== bodyOwnerForConstraint(world, entityB);
}
```

Use it in the `DistancePhysicsJoint` loader path:

```js
if (Math.abs(minDistance - maxDistance) < 1e-6) {
  if (shouldCreateDistanceConstraint(world, entityA, entityB)) {
    // create constraint entity
  }
}
```

Final local system order should no longer rely on `RigidGroupSystem` to repair migrated bodies after cable solve.

#### 2. Disable legacy rigid-group repair for authoritative groups
**File**: `src/js/cable_joints_3d/commonSystems.js`
**Action**: modify

Narrow `RigidGroupSystem` to legacy non-migrated groups only.

At the top of `RigidGroupSystem.update(...)`, skip any group entity that already owns a rigid body:

```js
for (const groupId of groupEntities) {
  if (world.hasComponent(groupId, RigidBodyComponent)) {
    continue;
  }
  const group = world.getComponent(groupId, RigidGroupComponent);
  // existing legacy path remains unchanged below
}
```

Leave the class in place so older scenes that still only use `RigidGroupComponent` keep working.

Also remove any remaining assumptions in helper functions that every rigid group is member-authoritative.

#### 3. Solver-side same-body protection and final ownership cleanup
**File**: `src/js/cable_joints_3d/cable_joints_core.js`
**Action**: modify

Keep the Phase 2 body-ref path, but make the skip of internal same-body corrections explicit and permanent.

Add a fast guard in the inner loop before denominator assembly:

```js
if (refA.bodyEntity === refB.bodyEntity) {
  continue;
}
```

And keep `CableJointComponent` authored-entity semantics unchanged so path tracing still operates on members.

Do **not** add any new post-solve member repair in this phase.

#### 4. Legacy-system regression coverage and migrated-scene stability coverage
**File**: `tests/js/cable_joints_3d/rigidGroupSystem.test.js`
**Action**: modify

Split the file into two expectations:
- legacy `RigidGroupComponent` without `RigidBodyComponent` still gets repaired,
- authoritative rigid bodies are skipped by `RigidGroupSystem`.

Example authoritative-body skip test:

```js
test('RigidGroupSystem ignores authoritative rigid bodies', () => {
  const world = new World();
  const groupId = makeAuthoritativeRigidGroup(world);
  const before = snapshotMemberPositions(world, groupId);

  new RigidGroupSystem().update(world, 1 / 120);

  expect(snapshotMemberPositions(world, groupId)).toEqual(before);
});
```

**File**: `tests/js/hp-sim-3d/setupScene3d.test.js`
**Action**: modify

Add final loader/system-order assertions:

```js
const systemNames = world.systems.map((system) => system.constructor.name);
expect(systemNames).toContain('SyncAttachedMembersFromBodySystem');
expect(systemNames).toContain('RigidBodyPredictionSystem');
expect(systemNames).toContain('RigidBodyVelocityUpdateSystem');
expect(systemNames).toContain('RigidBodyAngularVelocityUpdateSystem');
```

Also assert that no fixed-length `DistanceConstraintComponent` is created for two members owned by the same authoritative body.

### Verification
#### Automated
- [ ] `npx jest`

#### Manual
- [ ] `npx vite`
- [ ] Load `slideprinter_hexagon` and apply sustained cable pulls / repeated drive inputs.
- [ ] Confirm there is no visible double-stiffening, internal jitter, or correction explosion.
- [ ] Confirm the hexagon stays rigid without any post-solve member patch-up.

---

## Notes for the implementing agent

- Treat `CableJointComponent.entityA/entityB` as the authored geometry endpoints throughout all four phases. The solver ownership boundary changes, not the authored path topology.
- The rigid-group entity should become the authoritative body entity. Do **not** create a second parallel “body object” beside the rigid-group entity unless a test proves the shared-entity approach is impossible.
- Preserve spool twist when syncing body pose to spool members. That is the one place where a member keeps a local rotational DOF even after the body becomes authoritative.
- Keep `MomentOfInertiaComponent` in place for standalone entities and legacy paths. Only body-owned rigid groups should use `InertiaTensorComponent`.
- When updating tests, prefer direct component assertions over brittle entity-id assumptions.
