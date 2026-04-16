# Implementation Plan

## Overview
Convert runtime `RigidGroup` handling from a member-repair overlay into an authoritative rigid-body path with body-owned pose, velocity, mass, and 3×3 inertia, while preserving the existing USD scene-loading path and authored cable topology. The target runtime behavior is that grouped assemblies such as `slideprinter_hexagon` move as one rigid body under cable pull, with cable compliance solved through XPBD lambda state on the body rather than by correcting member entities and repairing them afterward.

## Phase 1: Authoritative compound-body loading

### Changes

#### 1. 3D ECS rigid-body primitives and tensor math
**File**: `src/js/cable_joints_3d/ecs.js`
**Action**: modify

Add the rigid-body and tensor primitives needed by later phases directly in the 3D ECS module.

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
  set(other) { this.elements.splice(0, 9, ...other.elements); return this; }
  add(other) { for (let i = 0; i < 9; i += 1) this.elements[i] += other.elements[i]; return this; }
  scale(s) { for (let i = 0; i < 9; i += 1) this.elements[i] *= s; return this; }
  transpose() {
    const e = this.elements;
    return new Matrix3([e[0], e[3], e[6], e[1], e[4], e[7], e[2], e[5], e[8]]);
  }
  multiplyMatrix(other) {
    const a = this.elements;
    const b = other.elements;
    return new Matrix3([
      a[0]*b[0] + a[1]*b[3] + a[2]*b[6], a[0]*b[1] + a[1]*b[4] + a[2]*b[7], a[0]*b[2] + a[1]*b[5] + a[2]*b[8],
      a[3]*b[0] + a[4]*b[3] + a[5]*b[6], a[3]*b[1] + a[4]*b[4] + a[5]*b[7], a[3]*b[2] + a[4]*b[5] + a[5]*b[8],
      a[6]*b[0] + a[7]*b[3] + a[8]*b[6], a[6]*b[1] + a[7]*b[4] + a[8]*b[7], a[6]*b[2] + a[7]*b[5] + a[8]*b[8],
    ]);
  }
  multiplyVector(v) {
    const e = this.elements;
    return new Vector3(
      e[0] * v.x + e[1] * v.y + e[2] * v.z,
      e[3] * v.x + e[4] * v.y + e[5] * v.z,
      e[6] * v.x + e[7] * v.y + e[8] * v.z,
    );
  }
  inverseOrZero() {
    const e = this.elements;
    const c00 = e[4]*e[8] - e[5]*e[7];
    const c01 = e[2]*e[7] - e[1]*e[8];
    const c02 = e[1]*e[5] - e[2]*e[4];
    const c10 = e[5]*e[6] - e[3]*e[8];
    const c11 = e[0]*e[8] - e[2]*e[6];
    const c12 = e[2]*e[3] - e[0]*e[5];
    const c20 = e[3]*e[7] - e[4]*e[6];
    const c21 = e[1]*e[6] - e[0]*e[7];
    const c22 = e[0]*e[4] - e[1]*e[3];
    const det = e[0]*c00 + e[1]*c10 + e[2]*c20;
    if (!Number.isFinite(det) || Math.abs(det) < 1e-12) {
      return Matrix3.zero();
    }
    const invDet = 1.0 / det;
    return new Matrix3([
      c00*invDet, c01*invDet, c02*invDet,
      c10*invDet, c11*invDet, c12*invDet,
      c20*invDet, c21*invDet, c22*invDet,
    ]);
  }

  static zero() { return new Matrix3([0,0,0, 0,0,0, 0,0,0]); }
  static identity() { return new Matrix3(); }
  static diagonal(x, y, z) { return new Matrix3([x,0,0, 0,y,0, 0,0,z]); }
  static fromOuterProduct(v) {
    return new Matrix3([
      v.x*v.x, v.x*v.y, v.x*v.z,
      v.y*v.x, v.y*v.y, v.y*v.z,
      v.z*v.x, v.z*v.y, v.z*v.z,
    ]);
  }
}

export class RigidBodyComponent {
  constructor(memberEntities = [], localFrames = new Map()) {
    this.memberEntities = [...memberEntities];
    this.memberLookup = new Set(memberEntities);
    this.localFrames = localFrames;
  }
}

export class InertiaTensorComponent {
  constructor(bodyInertia = Matrix3.identity()) {
    this.bodyInertia = bodyInertia.clone();
    this.bodyInvInertia = bodyInertia.clone().inverseOrZero();
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

export class PrevFinalBodyPoseComponent {
  constructor(pos = new Vector3(), orientation = new Quaternion()) {
    this.pos = pos.clone();
    this.orientation = orientation.clone().normalize();
  }
}
```

Implementation notes:
- Keep `MomentOfInertiaComponent` for non-compound legacy entities.
- `InertiaTensorComponent` is authoritative for rigid bodies created from `RigidGroup`.
- Do not touch `src/js/cable_joints/ecs.js`.

#### 2. Build a true body entity for each rigid group during scene load
**File**: `hp-sim-3d/app/setupScene.js`
**Action**: modify

Introduce a true rigid-body build path for rigid groups. The body entity owns mass properties and pose. Member entities remain for rendering, authored topology, spool state, and attachment handles.

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
  PrevFinalBodyPoseComponent,
  Matrix3,
} from '../../src/js/cable_joints_3d/ecs.js';
```

Add helpers near the existing USD readers:

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

function rotationMatrixFromQuaternion(q) {
  const xx = q.x * q.x;
  const yy = q.y * q.y;
  const zz = q.z * q.z;
  const xy = q.x * q.y;
  const xz = q.x * q.z;
  const yz = q.y * q.z;
  const wx = q.w * q.x;
  const wy = q.w * q.y;
  const wz = q.w * q.z;
  return new Matrix3([
    1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy),
    2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx),
    2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy),
  ]);
}

function rotateTensor(tensor, orientation) {
  const R = rotationMatrixFromQuaternion(orientation);
  return R.multiplyMatrix(tensor).multiplyMatrix(R.transpose());
}

function parallelAxisTerm(mass, offset) {
  const r2 = offset.lengthSq();
  return Matrix3.identity().scale(mass * r2)
    .add(Matrix3.fromOuterProduct(offset).scale(-mass));
}
```

While creating member entities, collect authored rigid-body source data in a map keyed by entity id:

```js
const authoredBodyData = new Map();

// For each spool or pinhole/attachment entity:
authoredBodyData.set(ent, {
  primPath: prim.path,
  mass,
  position: pos.clone(),
  orientation: initialOrientation.clone(),
  inertiaTensor: matrixFromUsdInertiaTensor(getAttribute(prim, 'physics:inertiaTensor')),
});
```

Replace the rigid-group build block with a helper that creates authoritative body state:

```js
function buildRigidBodyFromGroup(world, groupEnt, memberEntities, authoredBodyData) {
  let totalMass = 0.0;
  const com = new Vector3();
  const avgVel = new Vector3();
  let dynamicCount = 0;

  for (const memberEntity of memberEntities) {
    const authored = authoredBodyData.get(memberEntity);
    const mass = authored?.mass ?? 0.0;
    if (!(mass > 0.0)) continue;
    com.add(authored.position, mass);
    totalMass += mass;
  }

  if (totalMass > 0.0) {
    com.scale(1.0 / totalMass);
  } else {
    for (const memberEntity of memberEntities) {
      const pos = world.getComponent(memberEntity, PositionComponent)?.pos;
      if (!pos) continue;
      com.add(pos);
      dynamicCount += 1;
    }
    if (dynamicCount > 0) {
      com.scale(1.0 / dynamicCount);
    }
  }

  const localFrames = new Map();
  let bodyInertia = Matrix3.zero();

  for (const memberEntity of memberEntities) {
    const authored = authoredBodyData.get(memberEntity);
    const pos = authored?.position?.clone() || world.getComponent(memberEntity, PositionComponent)?.pos?.clone();
    const orientation = authored?.orientation?.clone() || world.getComponent(memberEntity, OrientationComponent)?.quaternion?.clone() || new Quaternion();
    const mass = authored?.mass ?? 0.0;
    const localPos = pos.clone().subtract(com);
    const localOrientation = orientation.clone();

    localFrames.set(memberEntity, { localPos, localOrientation });
    world.addComponent(memberEntity, new RigidAttachmentComponent(groupEnt, localPos, localOrientation));

    if (mass > 0.0) {
      const memberTensorBody = authored?.inertiaTensor?.clone() || Matrix3.zero();
      const memberTensorInGroupFrame = rotateTensor(memberTensorBody, localOrientation);
      bodyInertia.add(memberTensorInGroupFrame).add(parallelAxisTerm(mass, localPos));
    }
  }

  world.addComponent(groupEnt, new RigidBodyComponent(memberEntities, localFrames));
  world.addComponent(groupEnt, new PositionComponent(com.x, com.y, com.z));
  world.addComponent(groupEnt, new OrientationComponent(0, 0, 0, 1));
  world.addComponent(groupEnt, new VelocityComponent(avgVel.x, avgVel.y, avgVel.z));
  world.addComponent(groupEnt, new AngularVelocityComponent(0.0, 0.0, 0.0));
  world.addComponent(groupEnt, new MassComponent(totalMass > 0.0 ? totalMass : -1.0));
  world.addComponent(groupEnt, new InertiaTensorComponent(bodyInertia));
  world.addComponent(groupEnt, new PrevFinalBodyPoseComponent(com, new Quaternion()));
}
```

Replace the current rigid-group entity block with:

```js
const groupEnt = world.createEntity();
world.addComponent(groupEnt, new MachineTagComponent(machineId));
world.addComponent(groupEnt, new RigidGroupComponent(memberEntities, stiffness, renderSegments));
world.addComponent(groupEnt, new RenderableComponent('line', palette?.rigidGroup ?? palette?.distanceConstraint ?? '#55ff88'));
buildRigidBodyFromGroup(world, groupEnt, memberEntities, authoredBodyData);
```

At the end of `setupScene()`, call member sync before the initial render:

```js
syncAttachedMembersFromBody(world);
```

Constraints for this phase:
- Do not remove member entities.
- Do not reroute cable joints yet.
- Do not remove member components yet.

#### 3. Body-driven member sync seam
**File**: `src/js/cable_joints_3d/commonSystems.js`
**Action**: modify

Add helpers to derive member pose from body pose and push body-owned transforms back to attached members.

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
  PrevFinalBodyPoseComponent,
} from './ecs.js';
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

export function updateWorldInvInertia(world, bodyEntity) {
  const tensor = world.getComponent(bodyEntity, InertiaTensorComponent);
  const orientation = world.getComponent(bodyEntity, OrientationComponent)?.quaternion;
  if (!tensor || !orientation) {
    return;
  }
  const R = rotationMatrixFromQuaternion(orientation);
  tensor.worldInvInertia = R.multiplyMatrix(tensor.bodyInvInertia).multiplyMatrix(R.transpose());
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

      const nextPose = deriveMemberPose({ pos: bodyPos, orientation: bodyOrientation }, attachment);
      const posComp = world.getComponent(memberEntity, PositionComponent);
      if (posComp) posComp.pos.set(nextPose.pos);

      const orientationComp = world.getComponent(memberEntity, OrientationComponent);
      if (orientationComp) {
        orientationComp.quaternion.set(nextPose.orientation);
      }
    }
  }
}

export class SyncAttachedMembersFromBodySystem {
  runInPause = false;
  update(world, _dt) {
    syncAttachedMembersFromBody(world);
  }
}
```

Do not narrow `RigidGroupSystem` yet.

#### 4. Scene-setup test coverage for authoritative body creation
**File**: `tests/js/hp-sim-3d/setupScene3d.test.js`
**Action**: modify

Extend the USD stage mocks so one test loads a rigid group with at least one spool and one pinhole member, then verify authoritative body creation.

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

const tensor = world.getComponent(bodyEntity, InertiaTensorComponent);
expect(tensor.bodyInertia.elements.some((value) => Math.abs(value) > 0.0)).toBe(true);
```

#### 5. Body-setup regression test for derived member pose
**File**: `tests/js/cable_joints_3d/rigidGroupSystem.test.js`
**Action**: modify

Repurpose the setup seam coverage so it verifies body-driven member derivation.

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

### Verification
#### Automated
- [ ] `npx jest tests/js/hp-sim-3d/setupScene3d.test.js`
- [ ] `npx jest tests/js/cable_joints_3d/rigidGroupSystem.test.js`

#### Manual
- [ ] `npx vite`
- [ ] Open the 3D app, load `slideprinter_hexagon`, and confirm the scene still renders in the same rest pose.
- [ ] Confirm the rigid-group line rendering still appears and member meshes do not drift away from the hexagon at rest.

---

## Phase 2: Cable endpoints resolve to owning rigid bodies

### Changes

#### 1. Normalize attachment ownership for body-based cable solving
**File**: `src/js/cable_joints_3d/ecs.js`
**Action**: modify

Tighten `RigidBodyComponent` and `RigidAttachmentComponent` invariants used by the solver.

```js
export class RigidBodyComponent {
  constructor(memberEntities = [], localFrames = new Map()) {
    this.memberEntities = [...memberEntities];
    this.memberLookup = new Set(memberEntities);
    this.localFrames = localFrames;
  }

  hasMember(entityId) {
    return this.memberLookup.has(entityId);
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

#### 2. XPBD cable ownership boundary and body corrections
**File**: `src/js/cable_joints_3d/cable_joints_core.js`
**Action**: modify

Keep authored cable topology member-centric, but resolve each endpoint to its owning body before mass/inertia lookup and correction application. Also convert compliant cable solves to explicit XPBD lambda state.

Add a documented body-ref shape near the top of the file:

```js
/**
 * @typedef {Object} BodyRef
 * @property {number} bodyEntity
 * @property {Vector3} localPoint
 * @property {Quaternion | null} localOrientation
 */
```

Extend `CableJointComponent` with persistent XPBD state:

```js
export class CableJointComponent {
  constructor(entityA, entityB, restLength, attachmentPointA_world, attachmentPointB_world) {
    this.entityA = entityA;
    this.entityB = entityB;
    this.restLength = restLength;
    this.attachmentPointA_world = attachmentPointA_world.clone();
    this.attachmentPointB_world = attachmentPointB_world.clone();
    this.lambda = 0.0;
  }
}
```

Reset joint lambdas when authored rest geometry is rebuilt:

```js
joint.lambda = 0.0;
```

Add rigid-body helpers:

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

  const rotatedLocalPoint = attachment.localOrientation.transformVector(localPoint);
  return {
    bodyEntity: attachment.bodyEntity,
    localPoint: attachment.localPos.clone().add(rotatedLocalPoint),
    localOrientation: attachment.localOrientation.clone(),
  };
}

export function computeWorldAttachment(world, bodyRef) {
  const bodyPos = world.getComponent(bodyRef.bodyEntity, PositionComponent)?.pos;
  const bodyOrientation = world.getComponent(bodyRef.bodyEntity, OrientationComponent)?.quaternion;
  if (!bodyPos) return bodyRef.localPoint.clone();
  if (!bodyOrientation) return bodyPos.clone().add(bodyRef.localPoint);
  return bodyPos.clone().add(bodyOrientation.transformVector(bodyRef.localPoint));
}

function linearizedDeltaQuaternion(rotationVector, orientation) {
  const omegaQuat = new Quaternion(rotationVector.x, rotationVector.y, rotationVector.z, 0.0);
  const delta = new Quaternion().multiplyQuaternions(omegaQuat, orientation).scale(0.5);
  return new Quaternion(
    orientation.x + delta.x,
    orientation.y + delta.y,
    orientation.z + delta.z,
    orientation.w + delta.w,
  ).normalize();
}

export function applyBodyCorrection(world, bodyEntity, deltaPos, deltaRotationVector) {
  const posComp = world.getComponent(bodyEntity, PositionComponent);
  if (posComp && deltaPos) {
    posComp.pos.add(deltaPos);
  }

  const orientationComp = world.getComponent(bodyEntity, OrientationComponent);
  if (orientationComp && deltaRotationVector) {
    orientationComp.quaternion.set(
      linearizedDeltaQuaternion(deltaRotationVector, orientationComp.quaternion),
    );
  }

  updateWorldInvInertia(world, bodyEntity);
}
```

Use tensor-aware generalized inverse mass:

```js
function inverseMass(world, entityId) {
  const mass = world.getComponent(entityId, MassComponent)?.mass ?? 0.0;
  return mass > 0.0 ? 1.0 / mass : 0.0;
}

function inverseAngularMetric(world, entityId, gradAng) {
  const tensor = world.getComponent(entityId, InertiaTensorComponent);
  if (tensor) {
    const rotated = tensor.worldInvInertia.multiplyVector(gradAng);
    return { metric: gradAng.dot(rotated), rotatedGrad: rotated };
  }

  const scalar = world.getComponent(entityId, MomentOfInertiaComponent)?.invInertia ?? 0.0;
  return { metric: scalar * gradAng.lengthSq(), rotatedGrad: gradAng.clone().scale(scalar) };
}
```

Replace the core solve rule with XPBD lambda accumulation:

```js
function solveCableConstraint(world, joint, path, refA, refB) {
  const pA = computeWorldAttachment(world, refA);
  const pB = computeWorldAttachment(world, refB);
  const diff = pB.clone().subtract(pA);
  const length = diff.length();
  if (length <= EPSILON) return;

  const C = length - joint.restLength;
  if (C <= EPSILON) return;

  const n = diff.clone().scale(1.0 / length);
  const bodyPosA = world.getComponent(refA.bodyEntity, PositionComponent)?.pos;
  const bodyPosB = world.getComponent(refB.bodyEntity, PositionComponent)?.pos;
  const rA = pA.clone().subtract(bodyPosA);
  const rB = pB.clone().subtract(bodyPosB);
  const gradAngA = rA.clone().cross(n);
  const gradAngB = rB.clone().cross(n.clone().scale(-1.0));

  const wA = inverseMass(world, refA.bodyEntity);
  const wB = inverseMass(world, refB.bodyEntity);
  const angA = inverseAngularMetric(world, refA.bodyEntity, gradAngA);
  const angB = inverseAngularMetric(world, refB.bodyEntity, gradAngB);
  const alphaTilde = path.compliance / (world.getResource('dt') * world.getResource('dt'));
  const denom = wA + wB + angA.metric + angB.metric + alphaTilde;
  if (denom <= EPSILON) return;

  const deltaLambda = (-C - alphaTilde * joint.lambda) / denom;
  joint.lambda += deltaLambda;
  const impulse = n.clone().scale(deltaLambda);

  if (wA > 0.0 || angA.metric > 0.0) {
    applyBodyCorrection(
      world,
      refA.bodyEntity,
      impulse.clone().scale(wA),
      angA.rotatedGrad.clone().scale(deltaLambda),
    );
  }
  if (wB > 0.0 || angB.metric > 0.0) {
    applyBodyCorrection(
      world,
      refB.bodyEntity,
      impulse.clone().scale(-wB),
      angB.rotatedGrad.clone().scale(-deltaLambda),
    );
  }
}
```

Update `PBDCableConstraintSolver.update(...)` so the solver snapshots member-local attachment points once, resolves them to body refs, skips same-body endpoint pairs, and calls `solveCableConstraint(...)`.

#### 3. Keep setupScene cable loading unchanged except for compatibility hooks
**File**: `hp-sim-3d/app/setupScene.js`
**Action**: modify

Do not change authored cable topology in setup. Only add the imports needed for the body sync path and make sure the post-solve sync system is registered after cable solving and before render-visible state consumers.

Replace the current rigid-group sync point in system registration with:

```js
world.registerSystem(new PBDCableConstraintSolver());
world.registerSystem(new PBDResolveCableOverCorrections());
world.registerSystem(new SyncAttachedMembersFromBodySystem());
world.registerSystem(new RigidGroupSystem());
world.registerSystem(new SpoolAxisConstraintSystem());
```

`RigidGroupSystem` still remains in place in this phase, but member visuals must now be refreshed from body state before later systems consume member transforms.

#### 4. Cable-rigid-body solver coverage
**File**: `tests/js/cable_joints_3d/cableRigidBodySolver.test.js`
**Action**: create

Add a new unit test that checks cable correction on a body rather than on a member entity.

```js
test('cable correction updates the owning rigid body and accumulates lambda', () => {
  const world = new World();
  world.setResource('dt', 1 / 120);

  const body = world.createEntity();
  const anchor = world.createEntity();
  const member = world.createEntity();
  const jointEntity = world.createEntity();
  const pathEntity = world.createEntity();

  world.addComponent(body, new PositionComponent(0, 0, 0));
  world.addComponent(body, new OrientationComponent(0, 0, 0, 1));
  world.addComponent(body, new VelocityComponent(0, 0, 0));
  world.addComponent(body, new AngularVelocityComponent(0, 0, 0));
  world.addComponent(body, new MassComponent(2.0));
  world.addComponent(body, new InertiaTensorComponent(Matrix3.diagonal(1.0, 1.0, 1.0)));
  world.addComponent(body, new RigidBodyComponent([member], new Map()));

  world.addComponent(member, new PositionComponent(0, 0, 0));
  world.addComponent(member, new OrientationComponent(0, 0, 0, 1));
  world.addComponent(member, new RigidAttachmentComponent(body, new Vector3(0.5, 0, 0), new Quaternion()));

  world.addComponent(anchor, new PositionComponent(2.0, 0, 0));
  world.addComponent(anchor, new MassComponent(-1.0));

  world.addComponent(jointEntity, new CableJointComponent(
    member,
    anchor,
    1.0,
    new Vector3(0.5, 0, 0),
    new Vector3(2.0, 0, 0),
  ));
  world.addComponent(pathEntity, new CablePathComponent(world, [jointEntity], ['attachment', 'attachment'], [false, false], 1000.0));

  new PBDCableConstraintSolver().update(world, world.getResource('dt'));

  const joint = world.getComponent(jointEntity, CableJointComponent);
  expect(joint.lambda).not.toBe(0.0);
  expect(world.getComponent(body, PositionComponent).pos.x).toBeGreaterThan(0.0);
});
```

#### 5. setupScene regression for body-owned endpoints
**File**: `tests/js/hp-sim-3d/setupScene3d.test.js`
**Action**: modify

Add a regression that verifies cable joints authored on rigid-group members still load, and that members have `RigidAttachmentComponent` linking them to the group body.

```js
expect(world.getComponent(spoolEntity, RigidAttachmentComponent)?.bodyEntity).toBe(bodyEntity);
expect(world.query([CablePathComponent]).length).toBeGreaterThan(0);
expect(world.query([CableJointComponent]).length).toBeGreaterThan(0);
```

### Verification
#### Automated
- [ ] `npx jest tests/js/cable_joints_3d/cableRigidBodySolver.test.js tests/js/hp-sim-3d/setupScene3d.test.js`

#### Manual
- [ ] `npx vite`
- [ ] Load `slideprinter_hexagon` in the 3D app.
- [ ] Drive the hexagon and confirm that cable pull causes coherent whole-body translation and rotation.
- [ ] Confirm there is no visible sponge-like motion between rigid-group members while the body is under pull.

---

## Phase 3: Body-level stepping, caches, and velocity reconstruction

### Changes

#### 1. Body pose caches and tensor-aware body motion helpers
**File**: `src/js/cable_joints_3d/ecs.js`
**Action**: modify

`PrevFinalBodyPoseComponent` was added in Phase 1. In this phase it becomes authoritative for body-level cache and reconstruction.

No new schema beyond Phase 1 is needed here; only ensure all rigid-body queries use `PrevFinalBodyPoseComponent` rather than member caches.

#### 2. Predict and reconstruct on body state, with linearized quaternion updates
**File**: `src/js/cable_joints_3d/commonSystems.js`
**Action**: modify

Add body-level cache, motion, and reconstruction helpers matching the PBDBodies rigid-body state layout.

```js
export function cacheRigidBodyPose(world) {
  const bodies = world.query([RigidBodyComponent, PositionComponent, OrientationComponent, PrevFinalBodyPoseComponent]);
  for (const bodyEntity of bodies) {
    const pos = world.getComponent(bodyEntity, PositionComponent).pos;
    const orientation = world.getComponent(bodyEntity, OrientationComponent).quaternion;
    const prev = world.getComponent(bodyEntity, PrevFinalBodyPoseComponent);
    prev.pos.set(pos);
    prev.orientation.set(orientation);
  }
}

function integrateOrientationLinearized(quaternion, omega, dt) {
  const omegaQuat = new Quaternion(omega.x, omega.y, omega.z, 0.0);
  const delta = new Quaternion().multiplyQuaternions(omegaQuat, quaternion).scale(0.5 * dt);
  return new Quaternion(
    quaternion.x + delta.x,
    quaternion.y + delta.y,
    quaternion.z + delta.z,
    quaternion.w + delta.w,
  ).normalize();
}

export function predictRigidBodyMotion(world, dt) {
  const gravity = world.getResource('gravity');
  const grabbed = world.getResource('grabbedBall');
  const bodies = world.query([RigidBodyComponent, PositionComponent, OrientationComponent, VelocityComponent, AngularVelocityComponent, MassComponent]);

  for (const bodyEntity of bodies) {
    if (bodyEntity === grabbed) continue;
    const mass = world.getComponent(bodyEntity, MassComponent).mass;
    if (!(mass > 0.0)) continue;

    const pos = world.getComponent(bodyEntity, PositionComponent).pos;
    const orientation = world.getComponent(bodyEntity, OrientationComponent).quaternion;
    const vel = world.getComponent(bodyEntity, VelocityComponent).vel;
    const omega = world.getComponent(bodyEntity, AngularVelocityComponent).omega;

    if (gravity) {
      vel.add(gravity, dt);
    }
    pos.add(vel, dt);
    orientation.set(integrateOrientationLinearized(orientation, omega, dt));
    updateWorldInvInertia(world, bodyEntity);
  }
}

export function reconstructRigidBodyVelocity(world, dt) {
  if (!(dt > 0.0)) return;
  const bodies = world.query([RigidBodyComponent, PositionComponent, VelocityComponent, PrevFinalBodyPoseComponent, MassComponent]);
  for (const bodyEntity of bodies) {
    const mass = world.getComponent(bodyEntity, MassComponent).mass;
    if (!(mass > 0.0)) continue;
    const pos = world.getComponent(bodyEntity, PositionComponent).pos;
    const vel = world.getComponent(bodyEntity, VelocityComponent).vel;
    const prev = world.getComponent(bodyEntity, PrevFinalBodyPoseComponent);
    vel.subtractVectors(pos, prev.pos).scale(1.0 / dt);
  }
}

export function reconstructRigidBodyAngularVelocity(world, dt) {
  if (!(dt > 0.0)) return;
  const bodies = world.query([RigidBodyComponent, OrientationComponent, AngularVelocityComponent, PrevFinalBodyPoseComponent, InertiaTensorComponent]);
  for (const bodyEntity of bodies) {
    const orientation = world.getComponent(bodyEntity, OrientationComponent).quaternion;
    const prev = world.getComponent(bodyEntity, PrevFinalBodyPoseComponent);
    const omega = world.getComponent(bodyEntity, AngularVelocityComponent).omega;
    const qDelta = orientation.clone().multiplyQuaternions(
      orientation,
      prev.orientation.clone().conjugate().normalize(),
    ).normalize();

    if (qDelta.w < 0.0) {
      qDelta.x *= -1.0;
      qDelta.y *= -1.0;
      qDelta.z *= -1.0;
      qDelta.w *= -1.0;
    }

    omega.x = (2.0 / dt) * qDelta.x;
    omega.y = (2.0 / dt) * qDelta.y;
    omega.z = (2.0 / dt) * qDelta.z;
  }
}
```

Add small wrapper systems that call these helpers:

```js
export class PrevFinalBodyPoseSystem { runInPause = false; update(world, _dt) { cacheRigidBodyPose(world); } }
export class PredictRigidBodyMotionSystem { runInPause = false; update(world, dt) { predictRigidBodyMotion(world, dt); } }
export class RigidBodyVelocityUpdateSystem { runInPause = false; update(world, dt) { reconstructRigidBodyVelocity(world, dt); } }
export class RigidBodyAngularVelocityUpdateSystem { runInPause = false; update(world, dt) { reconstructRigidBodyAngularVelocity(world, dt); } }
```

Keep `deriveMemberPose(...)` and `syncAttachedMembersFromBody(...)` as the only path that updates member transforms after body solve.

#### 3. Switch setupScene system registration to body-owned motion
**File**: `hp-sim-3d/app/setupScene.js`
**Action**: modify

Register body-level cache and motion systems for body-owned rigid groups. Leave legacy systems in place for non-rigid-group standalone entities.

Replace the body-relevant registration block with:

```js
world.registerSystem(new PrevFinalPosSystem());
world.registerSystem(new PrevFinalOrientationSystem());
world.registerSystem(new PrevFinalBodyPoseSystem());

world.registerSystem(new RemoteSpoolSystem());
world.registerSystem(new StepperMotorSystem());

world.registerSystem(new GravitySystem());
world.registerSystem(new MovementSystem());
world.registerSystem(new AngularMovementSystem());
world.registerSystem(new PredictRigidBodyMotionSystem());

world.registerSystem(new CableAttachmentUpdateSystem());
world.registerSystem(new CableAttachmentCacheSystem());
world.registerSystem(new CableSlackSystem());
world.registerSystem(new PBDCableConstraintSolver());
world.registerSystem(new PBDResolveCableOverCorrections());
world.registerSystem(new SyncAttachedMembersFromBodySystem());
world.registerSystem(new RigidGroupSystem());
world.registerSystem(new SpoolAxisConstraintSystem());
world.registerSystem(new CableFrictionSystem());
world.registerSystem(new PBDVelocityUpdateSystem());
world.registerSystem(new PBDAngularVelocityUpdateSystem());
world.registerSystem(new RigidBodyVelocityUpdateSystem());
world.registerSystem(new RigidBodyAngularVelocityUpdateSystem());
world.registerSystem(new ExtruderSystem());
world.registerSystem(new EncoderUpdateSystem());
```

Body-owned rigid groups must now derive their visible member pose from the body before `ExtruderSystem` and `EncoderUpdateSystem` consume those transforms.

#### 4. Spool orientation readers consume body-derived member pose only
**File**: `hp-sim-3d/app/hangprinter_spools.js`
**Action**: modify if needed

Only change this file if a reader assumes member orientation is directly simulated rather than derived. The desired end state is that spool utilities continue to operate on member orientation, but that orientation is now body-derived.

If a safeguard is needed, add a no-op compatibility helper:

```js
export function getResolvedSpoolOrientation(world, entityId) {
  return world.getComponent(entityId, OrientationComponent)?.quaternion ?? new Quaternion();
}
```

Do not change spool math unless a direct-state assumption breaks tests.

#### 5. Body-velocity regression coverage
**File**: `tests/js/cable_joints_3d/bodyVelocityUpdate.test.js`
**Action**: create

Add a focused regression test for body-level cache and reconstruction.

```js
test('reconstructs rigid body linear and angular velocity from body pose delta', () => {
  const world = new World();
  const body = world.createEntity();
  const dt = 1 / 120;

  world.addComponent(body, new RigidBodyComponent([], new Map()));
  world.addComponent(body, new PositionComponent(0, 0, 0));
  world.addComponent(body, new OrientationComponent(0, 0, 0, 1));
  world.addComponent(body, new VelocityComponent(0, 0, 0));
  world.addComponent(body, new AngularVelocityComponent(0, 0, 0));
  world.addComponent(body, new MassComponent(1.0));
  world.addComponent(body, new InertiaTensorComponent(Matrix3.diagonal(1, 1, 1)));
  world.addComponent(body, new PrevFinalBodyPoseComponent(new Vector3(0, 0, 0), new Quaternion()));

  cacheRigidBodyPose(world);
  world.getComponent(body, PositionComponent).pos.set(new Vector3(0.12, 0.0, 0.0));
  world.getComponent(body, OrientationComponent).quaternion.set(
    new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), 0.1),
  );

  reconstructRigidBodyVelocity(world, dt);
  reconstructRigidBodyAngularVelocity(world, dt);

  expect(world.getComponent(body, VelocityComponent).vel.x).toBeCloseTo(0.12 / dt, 6);
  expect(world.getComponent(body, AngularVelocityComponent).omega.z).toBeGreaterThan(0.0);
});
```

### Verification
#### Automated
- [ ] `npx jest tests/js/cable_joints_3d/bodyVelocityUpdate.test.js tests/js/cable_joints_3d/cableRigidBodySolver.test.js`

#### Manual
- [ ] `npx vite`
- [ ] In the 3D app, move the hexagon and confirm reported/visible motion follows whole-body translation and rotation.
- [ ] Confirm member pose remains coherent after motion and cable correction.

---

## Phase 4: Remove double-correction paths and stabilize migrated scenes

### Changes

#### 1. Skip same-body internal distance constraints at setup
**File**: `hp-sim-3d/app/setupScene.js`
**Action**: modify

Keep the setup-time skip for authored fixed-length constraints that connect two members of the same authoritative rigid body.

Replace the inline skip with a named helper:

```js
function shouldCreateDistanceConstraint(world, entityA, entityB) {
  const attachmentA = world.getComponent(entityA, RigidAttachmentComponent);
  const attachmentB = world.getComponent(entityB, RigidAttachmentComponent);
  if (attachmentA && attachmentB && attachmentA.bodyEntity === attachmentB.bodyEntity) {
    return false;
  }
  return true;
}
```

Use it in the `DistancePhysicsJoint` import path:

```js
if (shouldCreateDistanceConstraint(world, entityA, entityB)) {
  const constraintEntity = world.createEntity();
  world.addComponent(constraintEntity, new MachineTagComponent(machineId));
  world.addComponent(constraintEntity, new DistanceConstraintComponent(entityA, entityB, restLength, 0.0));
  world.addComponent(constraintEntity, new RenderableComponent('line', distanceColor));
}
```

#### 2. Narrow RigidGroupSystem to non-authoritative legacy groups only
**File**: `src/js/cable_joints_3d/commonSystems.js`
**Action**: modify

Add a helper:

```js
export function isAuthoritativeRigidBody(world, entityId) {
  return world.hasComponent(entityId, RigidBodyComponent)
    && world.hasComponent(entityId, InertiaTensorComponent)
    && world.hasComponent(entityId, PrevFinalBodyPoseComponent);
}
```

Guard `RigidGroupSystem.update(...)` so authoritative bodies are skipped:

```js
if (isAuthoritativeRigidBody(world, groupId)) {
  continue;
}
```

After this phase, `RigidGroupSystem` remains only as a fallback path for scenes not yet migrated to the body-owned model.

#### 3. Skip same-body cable self-correction and finalize body-owned solve order
**File**: `src/js/cable_joints_3d/cable_joints_core.js`
**Action**: modify

Keep the body-ref same-body skip from Phase 2 and make it explicit in the final implementation.

```js
if (refA.bodyEntity === refB.bodyEntity) {
  continue;
}
```

Do not apply any cable correction directly to member `PositionComponent` or `OrientationComponent` when an endpoint resolves to a rigid body.

#### 4. Regression coverage for the migrated path
**File**: `tests/js/cable_joints_3d/rigidGroupSystem.test.js`
**Action**: modify

Replace overlay-specific expectations with migrated-path expectations.

```js
test('authoritative rigid-body groups are ignored by RigidGroupSystem', () => {
  const world = new World();
  const body = world.createEntity();
  world.addComponent(body, new RigidGroupComponent([], 1.0));
  world.addComponent(body, new RigidBodyComponent([], new Map()));
  world.addComponent(body, new InertiaTensorComponent(Matrix3.diagonal(1, 1, 1)));
  world.addComponent(body, new PrevFinalBodyPoseComponent(new Vector3(), new Quaternion()));

  expect(() => new RigidGroupSystem().update(world, 1 / 120)).not.toThrow();
});
```

#### 5. End-to-end setupScene stability coverage
**File**: `tests/js/hp-sim-3d/setupScene3d.test.js`
**Action**: modify

Add a final regression that verifies a rigid-group member pair no longer creates a same-body `DistanceConstraintComponent`.

```js
const constraints = world.query([DistanceConstraintComponent]);
expect(constraints).toHaveLength(0);
```

### Verification
#### Automated
- [ ] `npx jest`

#### Manual
- [ ] `npx vite`
- [ ] Load `slideprinter_hexagon` and stress it with sustained cable pull.
- [ ] Confirm motion remains stable with no visible internal jitter, no double-stiffening, and no repair-style snapping between members.
- [ ] Confirm cable pull still produces coherent body translation and rotation after the legacy same-body correction paths are removed from the migrated scene.
