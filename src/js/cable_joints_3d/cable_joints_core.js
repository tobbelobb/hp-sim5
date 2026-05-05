import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';

import {
  tangentFromPointToSphere,
  tangentFromSphereToPoint,
  tangentFromSphereToSphere,
  signedArcLengthOnWheel,
  lineSegmentSphereIntersection,
  rightOfPlane
} from './geometry3.js';

import {
  PositionComponent,
  PrevFinalPosComponent,
  RadiusComponent,
  MassComponent,
  OrientationComponent,
  PrevFinalOrientationComponent,
  RigidBodyMemberComponent,
  HybridKnotAngleComponent,
  MomentOfInertiaComponent,
  MachineTagComponent,
  layeringEnabled
} from './ecs.js';
import {
  computeLocalAttachment,
  computeWorldAttachment,
  getEntityWorldOrientation,
  getEntityWorldPosition,
  getRigidBodyEntityForMember,
  resolveRigidBodySolverEndpoint,
  updateRigidBodyMemberLocalOrientation,
} from './rigid_bodies.js';
import { StepperMotorComponent } from '../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { isStepperClosedLoopEnabled } from '../../../hp-sim-3d/app/hangprinter_runtime.js';
import { SpoolStateComponent } from '../../../hp-sim-3d/app/hangprinter_spools.js';

export class CableLinkComponent {
  constructor(
    x = 0,
    y = 0,
    z = 0,
    orientation = null,
    planeNormal = null,
    planeNormalLocal = null
  ) {
    this.prevCableAttachmentTimePos = new Vector3(x, y, z);
    this.prevCableAttachmentTimeOrientation = orientation ? orientation.clone() : new Quaternion();
    this.prevCableAttachmentTimeLocalOrientation = orientation ? orientation.clone() : new Quaternion();
    this.cablePlaneNormal = planeNormal ? planeNormal.clone() : new Vector3(0, 0, 1);
    this.cablePlaneNormalLocal = planeNormalLocal ? planeNormalLocal.clone().normalize() : null;
  }
}

// Represents a single segment constraint between two entities
export class CableJointComponent {
  constructor(entityA, entityB, restLength, attachmentPointA_world, attachmentPointB_world) {
    this.entityA = entityA;
    this.entityB = entityB;
    this.restLength = restLength; // dn - the dynamic maximum length
    this.attachmentPointA_world = attachmentPointA_world.clone();
    this.attachmentPointB_world = attachmentPointB_world.clone();
    this.constraintLambda = 0.0;
    this.constraintForce = new Vector3(0, 0, 0);
    this.constraintForceMagnitude = 0.0;
    this.transferredConstraintForceMagnitude = 0.0;
  }

  static fromWorld(entityA, entityB, restLength, attachmentPointA_world, attachmentPointB_world) {
    return new CableJointComponent(
      entityA,
      entityB,
      restLength,
      attachmentPointA_world,
      attachmentPointB_world
    );
  }

  static fromLocal(world, entityA, entityB, restLength, attachmentPointA_local, attachmentPointB_local) {
    return new CableJointComponent(
      entityA,
      entityB,
      restLength,
      computeWorldAttachment(world, entityA, attachmentPointA_local),
      computeWorldAttachment(world, entityB, attachmentPointB_local)
    );
  }
}

// Connects individual cable joints into a cable path
export class CablePathComponent {
  constructor(
    world,
    jointEntities = [],
    linkTypes = [],
    cw = [],
    spring_constant = 1e6,
    stored = null,
    cableHalfWidth = 0.0,
    damping = 0.0,
    solverIterations = 1
  ) {
    this.totalRestLength = 0.0;
    this.jointEntities = jointEntities; // Ordered list of CableJoint entity IDs
    this.linkTypes = linkTypes; // Ordered. linkTypes.length === jointEntities.length + 1
    this.cw = cw; // Ordered. cw.length === linkTypes.length
    this.spring_constant = spring_constant;
    this.compliance = 1.0 / spring_constant;
    this.damping = Number.isFinite(damping) ? Math.max(0.0, damping) : 0.0;
    this.solverIterations = Number.isFinite(solverIterations)
      ? Math.max(1, Math.floor(solverIterations))
      : 1;
    this.stored = new Array(cw.length).fill(0.0); // Ordered. stored.length === cw.length
    this.cableHalfWidth = Number.isFinite(cableHalfWidth) ? Math.max(0.0, cableHalfWidth) : 0.0;

    for (const jointId of jointEntities) {
      const joint = world.getComponent(jointId, CableJointComponent);
      this.totalRestLength += joint.restLength;
    }
    for (let i = 0; i < jointEntities.length - 1; i++) {
      const jointId_i = jointEntities[i];
      const jointId_i_plus_1 = jointEntities[i + 1];
      const joint_i = world.getComponent(jointId_i, CableJointComponent);
      const joint_i_plus_1 = world.getComponent(jointId_i_plus_1, CableJointComponent);
      const linkId = joint_i.entityB;
      const linkId2 = joint_i_plus_1.entityA;
      if (linkId !== linkId2) {
        console.warn("CablePathComponent constructor: Links don't match up. There's something wrong with this cable path.");
        return;
      }
      const isRolling = linkTypes[i + 1] === 'rolling';
      if (isRolling) {
        const center = getEntityWorldPosition(world, linkId);
        const baseRadius = world.getComponent(linkId, RadiusComponent).radius;
        const radius = baseRadius + (layeringEnabled(world) ? this.cableHalfWidth : 0.0);
        const isCw = cw[i + 1];

        const initialStoredLength = signedArcLengthOnWheel(
          joint_i.attachmentPointB_world,
          joint_i_plus_1.attachmentPointA_world,
          center,
          radius,
          isCw,
          _getPlaneNormal(world, linkId),
          true
        );
        this.stored[i + 1] = initialStoredLength;
        this.totalRestLength += initialStoredLength;
      }
    }

    if (stored !== null) {
      for (let i = 0; i < this.stored.length; i++) {
        if (stored[i] !== null) {
          this.totalRestLength -= this.stored[i];
          this.totalRestLength += stored[i];
          this.stored[i] = stored[i];
        }
      }
    }

    const lastLinkIndex = this.linkTypes.length - 1;
    if (lastLinkIndex >= 0) {
      _ensureHybridKnotAngleComponentForEndpoint(world, this, 0);
      if (lastLinkIndex !== 0) {
        _ensureHybridKnotAngleComponentForEndpoint(world, this, lastLinkIndex);
      }
    }
  }
}

export const linecolor1 = '#FFFF00';

const EPSILON = 1e-9;
const MIN_JOINT_REST_LENGTH = 1e-6;
const KNOT_SPAN = Math.PI / 30.0;
const CABLE_DEBUG_PREFIX = '[CableJointsDebug]';
const CABLE_HYBRID_TRACE_PREFIX = '[CableHybridTrace]';

const DEFAULT_PLANE_NORMAL = new Vector3(0, 0, 1);
const DEFAULT_CLOSED_LOOP_STEPPER_CONSTRAINT_STIFFNESS = 1e8;
const DEFAULT_OPEN_LOOP_STEPPER_CONSTRAINT_STIFFNESS_SCALE = 1.0;

function _debugCable(world, message) {
  if (world?.getResource('cableDebugLogs') === true) {
    console.debug(`${CABLE_DEBUG_PREFIX} ${message}`);
  }
}

function _readFiniteResource(world, key, fallback) {
  const value = world?.getResource?.(key);
  return Number.isFinite(value) ? value : fallback;
}

function _vecSnapshot(v) {
  if (!v) {
    return null;
  }
  return { x: v.x, y: v.y, z: v.z };
}

function _hybridTransitionTraceEnabled(world, step) {
  if (world?.getResource?.('cableHybridTransitionTrace') !== true) {
    return false;
  }
  const minStep = _readFiniteResource(world, 'cableHybridTransitionTraceStepMin', -Infinity);
  const maxStep = _readFiniteResource(world, 'cableHybridTransitionTraceStepMax', Infinity);
  const resolvedStep = Number.isFinite(step)
    ? Math.floor(step)
    : Math.floor(_readFiniteResource(world, 'cableHybridTransitionStep', 0));
  return resolvedStep >= minStep && resolvedStep <= maxStep;
}

function _recordHybridTransitionTrace(world, step, event) {
  if (!_hybridTransitionTraceEnabled(world, step)) {
    return;
  }

  let traceBuffer = world.getResource('cableHybridTransitionTraceBuffer');
  if (!Array.isArray(traceBuffer)) {
    traceBuffer = [];
    world.setResource('cableHybridTransitionTraceBuffer', traceBuffer);
  }

  const maxTraceEvents = Math.max(
    1,
    Math.floor(_readFiniteResource(world, 'cableHybridTransitionTraceLimit', 256))
  );
  if (traceBuffer.length >= maxTraceEvents) {
    world.setResource('cableHybridTransitionTraceTruncated', true);
    return;
  }

  const entry = {
    step: Number.isFinite(step) ? Math.floor(step) : Math.floor(_readFiniteResource(world, 'cableHybridTransitionStep', 0)),
    ...event
  };
  traceBuffer.push(entry);

  if (world.getResource('cableHybridTransitionTraceConsole') === true) {
    console.debug(`${CABLE_HYBRID_TRACE_PREFIX} ${JSON.stringify(entry)}`);
  }
}

function _cableEventTraceEnabled(world, step) {
  if (world?.getResource?.('cableEventTrace') !== true) {
    return false;
  }
  const minStep = _readFiniteResource(world, 'cableEventTraceStepMin', -Infinity);
  const maxStep = _readFiniteResource(world, 'cableEventTraceStepMax', Infinity);
  const resolvedStep = Number.isFinite(step)
    ? Math.floor(step)
    : Math.floor(_readFiniteResource(world, 'cableHybridTransitionStep', 0));
  return resolvedStep >= minStep && resolvedStep <= maxStep;
}

function _recordCableEventTrace(world, step, event) {
  if (!_cableEventTraceEnabled(world, step)) {
    return;
  }

  let traceBuffer = world.getResource('cableEventTraceBuffer');
  if (!Array.isArray(traceBuffer)) {
    traceBuffer = [];
    world.setResource('cableEventTraceBuffer', traceBuffer);
  }

  const maxTraceEvents = Math.max(
    1,
    Math.floor(_readFiniteResource(world, 'cableEventTraceLimit', 4096))
  );
  if (traceBuffer.length >= maxTraceEvents) {
    world.setResource('cableEventTraceTruncated', true);
    return;
  }

  const entry = {
    step: Number.isFinite(step) ? Math.floor(step) : Math.floor(_readFiniteResource(world, 'cableHybridTransitionStep', 0)),
    ...event
  };
  traceBuffer.push(entry);

  if (world.getResource('cableEventTraceConsole') === true) {
    console.debug(`[CableEventTrace] ${JSON.stringify(entry)}`);
  }
}

function _recordCableStepSummary(world, step, phase) {
  if (!_cableEventTraceEnabled(world, step)) {
    return;
  }
  const jointEntities = world.query([CableJointComponent]);
  const pathEntities = world.query([CablePathComponent]);

  let minRestLength = Infinity;
  let maxRestLength = -Infinity;
  let tinyRestCount = 0;
  let negativeRestCount = 0;
  let nonFiniteRestCount = 0;
  for (const jointId of jointEntities) {
    const joint = world.getComponent(jointId, CableJointComponent);
    const rest = joint?.restLength;
    if (!Number.isFinite(rest)) {
      nonFiniteRestCount += 1;
      continue;
    }
    if (rest < minRestLength) minRestLength = rest;
    if (rest > maxRestLength) maxRestLength = rest;
    if (rest < MIN_JOINT_REST_LENGTH) tinyRestCount += 1;
    if (rest < 0.0) negativeRestCount += 1;
  }
  if (jointEntities.length < 1) {
    minRestLength = 0.0;
    maxRestLength = 0.0;
  }

  let minStored = Infinity;
  let maxStored = -Infinity;
  let negativeStoredCount = 0;
  let nonFiniteStoredCount = 0;
  let maxPathJoints = 0;
  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    maxPathJoints = Math.max(maxPathJoints, path?.jointEntities?.length ?? 0);
    for (const stored of path?.stored ?? []) {
      if (!Number.isFinite(stored)) {
        nonFiniteStoredCount += 1;
        continue;
      }
      if (stored < minStored) minStored = stored;
      if (stored > maxStored) maxStored = stored;
      if (stored < 0.0) negativeStoredCount += 1;
    }
  }
  if (pathEntities.length < 1) {
    minStored = 0.0;
    maxStored = 0.0;
  }
  if (!Number.isFinite(minStored)) minStored = 0.0;
  if (!Number.isFinite(maxStored)) maxStored = 0.0;

  _recordCableEventTrace(world, step, {
    type: 'summary',
    phase,
    pathCount: pathEntities.length,
    jointCount: jointEntities.length,
    maxPathJoints,
    minRestLength,
    maxRestLength,
    tinyRestCount,
    negativeRestCount,
    nonFiniteRestCount,
    minStored,
    maxStored,
    negativeStoredCount,
    nonFiniteStoredCount
  });
}

function getMachineId(world, entityId) {
  if (entityId == null) {
    return '';
  }
  const tag = world.getComponent(entityId, MachineTagComponent);
  return tag ? tag.id : '';
}

function ensureMachineTag(world, entityId, machineId) {
  if (!entityId) {
    return;
  }
  const existing = world.getComponent(entityId, MachineTagComponent);
  if (existing) {
    return;
  }
  world.addComponent(entityId, new MachineTagComponent(machineId));
}

function _getPlaneNormal(world, entityId) {
  const linkComp = world.getComponent(entityId, CableLinkComponent);
  if (linkComp?.cablePlaneNormalLocal) {
    const localAxis = linkComp.cablePlaneNormalLocal.clone();
    const orientation = getEntityWorldOrientation(world, entityId);
    if (orientation && typeof orientation.transformVector === 'function') {
      const worldAxis = orientation.transformVector(localAxis);
      if (worldAxis.lengthSq() > EPSILON) {
        return worldAxis.normalize();
      }
    }
    if (localAxis.lengthSq() > EPSILON) {
      return localAxis.normalize();
    }
  }
  if (linkComp && linkComp.cablePlaneNormal) {
    return linkComp.cablePlaneNormal;
  }
  return DEFAULT_PLANE_NORMAL;
}

function _localTwistAngle(quaternion, axisLocal) {
  if (!quaternion) {
    return 0.0;
  }
  const normalizedAxis = axisLocal?.clone?.().normalize?.() || DEFAULT_PLANE_NORMAL.clone();
  const projectionScale =
    (quaternion.x * normalizedAxis.x)
    + (quaternion.y * normalizedAxis.y)
    + (quaternion.z * normalizedAxis.z);
  const twist = new Quaternion(
    normalizedAxis.x * projectionScale,
    normalizedAxis.y * projectionScale,
    normalizedAxis.z * projectionScale,
    quaternion.w,
  );
  if (twist.lengthSq() <= EPSILON) {
    return 0.0;
  }
  twist.normalize();
  const signedSinHalf =
    (twist.x * normalizedAxis.x)
    + (twist.y * normalizedAxis.y)
    + (twist.z * normalizedAxis.z);
  return _normalizeAngleSigned(2.0 * Math.atan2(signedSinHalf, twist.w));
}

function _deltaAngleAroundAxis(prevQuat, currQuat, axis, axisIsLocal = false) {
  if (!prevQuat || !currQuat) {
    return 0.0;
  }
  if (axisIsLocal) {
    const invPrev = prevQuat.clone().conjugate().normalize();
    const qRelLocal = new Quaternion().multiplyQuaternions(invPrev, currQuat).normalize();
    return _localTwistAngle(qRelLocal, axis);
  }
  const invPrev = prevQuat.clone().conjugate().normalize();
  const qRel = new Quaternion().multiplyQuaternions(currQuat, invPrev).normalize();
  const w = Math.max(-1.0, Math.min(1.0, qRel.w));
  let angle = 2.0 * Math.acos(w);
  if (angle < EPSILON) {
    return 0.0;
  }
  const sinHalf = Math.sqrt(Math.max(0.0, 1.0 - w * w));
  let axisRel;
  if (sinHalf < EPSILON) {
    axisRel = axis.clone().normalize();
  } else {
    axisRel = new Vector3(qRel.x / sinHalf, qRel.y / sinHalf, qRel.z / sinHalf);
  }
  const sign = Math.sign(axisRel.dot(axis));
  if (sign === 0.0) {
    return angle;
  }
  if (angle > Math.PI) {
    angle -= 2.0 * Math.PI;
  }
  return angle * sign;
}

function _rotationVectorBetween(prevQuat, currQuat) {
  if (!prevQuat || !currQuat) {
    return new Vector3(0.0, 0.0, 0.0);
  }

  const qDelta = new Quaternion()
    .multiplyQuaternions(
      currQuat,
      prevQuat.clone().conjugate().normalize(),
    )
    .normalize();

  if (qDelta.w < 0.0) {
    qDelta.x *= -1.0;
    qDelta.y *= -1.0;
    qDelta.z *= -1.0;
    qDelta.w *= -1.0;
  }

  const w = Math.max(-1.0, Math.min(1.0, qDelta.w));
  const angle = 2.0 * Math.acos(w);
  if (angle <= EPSILON) {
    return new Vector3(0.0, 0.0, 0.0);
  }

  const sinHalf = Math.sqrt(Math.max(0.0, 1.0 - (w * w)));
  if (sinHalf <= EPSILON) {
    return new Vector3(0.0, 0.0, 0.0);
  }

  return new Vector3(
    qDelta.x / sinHalf,
    qDelta.y / sinHalf,
    qDelta.z / sinHalf,
  ).scale(angle);
}

function _rotateAroundAxis(point, center, axis, angle, cw) {
  const axisNorm = axis.clone().normalize();
  const signedAngle = cw ? angle : -angle;
  return point.clone().subtract(center).rotateAroundAxis(axisNorm, signedAngle).add(center);
}

function _projectPointToPlane(point, planePoint, planeNormal) {
  if (!point || !planePoint || !planeNormal || planeNormal.lengthSq() <= EPSILON) {
    return point ? point.clone() : null;
  }
  const normal = planeNormal.clone().normalize();
  return point.clone().subtract(normal, point.clone().subtract(planePoint).dot(normal));
}

function _effectiveCW(path, linkIndex, travellingFromCircle) {
  if (linkIndex === 0 && travellingFromCircle) {
    return !path.cw[linkIndex];
  }
  return path.cw[linkIndex];
}

function _deriveRollingSideAttachment(bodyPos, counterpartPos, radius, planeNormal, cw, pointIsFirst) {
  if (!bodyPos || !counterpartPos || radius === undefined) {
    return bodyPos ? bodyPos.clone() : null;
  }
  const projectedCounterpart = _projectPointToPlane(counterpartPos, bodyPos, planeNormal);
  return pointIsFirst
    ? tangentFromSphereToPoint(projectedCounterpart, bodyPos, radius, planeNormal, cw).a_sphere
    : tangentFromPointToSphere(projectedCounterpart, bodyPos, radius, planeNormal, cw).a_sphere;
}

function _planesAreParallel(normalA, normalB) {
  if (!normalA || !normalB || normalA.lengthSq() <= EPSILON || normalB.lengthSq() <= EPSILON) {
    return false;
  }
  return Math.abs(Math.abs(normalA.clone().normalize().dot(normalB.clone().normalize())) - 1.0) <= 1e-6;
}

function _resourceBool(world, key, fallback = true) {
  const value = world?.getResource?.(key);
  return typeof value === 'boolean' ? value : fallback;
}

function _featureFlag(world, key, fallback = true) {
  return _resourceBool(world, key, fallback);
}

function _normalizeAngleSigned(angle) {
  let wrapped = angle;
  const twoPi = 2.0 * Math.PI;
  while (wrapped > Math.PI) wrapped -= twoPi;
  while (wrapped < -Math.PI) wrapped += twoPi;
  return wrapped;
}

function _unwrapAngleNear(reference, wrappedValue) {
  let value = wrappedValue;
  const twoPi = 2.0 * Math.PI;
  while (value - reference > Math.PI) value -= twoPi;
  while (value - reference < -Math.PI) value += twoPi;
  return value;
}

function _planeBasisForAxis(axis) {
  const n = (axis ?? DEFAULT_PLANE_NORMAL).clone();
  if (n.lengthSq() <= EPSILON) {
    return {
      n: DEFAULT_PLANE_NORMAL.clone(),
      u: new Vector3(1, 0, 0),
      v: new Vector3(0, 1, 0)
    };
  }
  n.normalize();
  let reference = Math.abs(n.x) < 0.9 ? new Vector3(1, 0, 0) : new Vector3(0, 1, 0);
  const nDotRef = n.dot(reference);
  let u = reference.clone().subtract(n, nDotRef);
  if (u.lengthSq() <= EPSILON) {
    reference = new Vector3(0, 0, 1);
    const altDot = n.dot(reference);
    u = reference.clone().subtract(n, altDot);
  }
  u.normalize();
  const v = n.cross(u);
  return { n, u, v };
}

function _orientationAngleAroundAxis(quaternion, axis) {
  if (!quaternion) {
    return 0.0;
  }
  const basis = _planeBasisForAxis(axis);
  const rotatedU = quaternion.transformVector(basis.u);
  const projected = rotatedU.clone().subtract(basis.n, rotatedU.dot(basis.n));
  if (projected.lengthSq() <= EPSILON) {
    return 0.0;
  }
  projected.normalize();
  return Math.atan2(projected.dot(basis.v), projected.dot(basis.u));
}

function _currentLocalOrientationForEntity(world, entityId, fallbackQuaternion = null) {
  const member = world.getComponent(entityId, RigidBodyMemberComponent);
  return member?.localOrientation || fallbackQuaternion;
}

function _previousLocalOrientationForLink(linkComp, fallbackQuaternion = null) {
  return linkComp?.prevCableAttachmentTimeLocalOrientation || fallbackQuaternion;
}

function _sameRigidBodyMemberPair(world, entityA, entityB) {
  const memberA = world.getComponent(entityA, RigidBodyMemberComponent);
  const memberB = world.getComponent(entityB, RigidBodyMemberComponent);
  return Boolean(memberA && memberB && memberA.bodyEntity === memberB.bodyEntity);
}

function _orientationFrameForEndpoint(
  world,
  entityId,
  counterpartEntityId,
  currentWorldQuaternion,
  previousWorldQuaternion,
  linkComp,
) {
  // Onboard spans should ignore host body rotation. External spans should
  // include the mounted part's full world rotation against the outside cable.
  if (_sameRigidBodyMemberPair(world, entityId, counterpartEntityId)) {
    return {
      current: _currentLocalOrientationForEntity(world, entityId, currentWorldQuaternion),
      previous: _previousLocalOrientationForLink(linkComp, previousWorldQuaternion),
    };
  }
  return {
    current: currentWorldQuaternion,
    previous: previousWorldQuaternion,
  };
}

function _bodyOrientationFromMemberFrame(worldQuaternion, localQuaternion) {
  if (!worldQuaternion || !localQuaternion) {
    return null;
  }
  return new Quaternion()
    .multiplyQuaternions(
      worldQuaternion,
      localQuaternion.clone().conjugate().normalize(),
    )
    .normalize();
}

function _arcFrameForEndpoint(
  world,
  entityId,
  counterpartEntityId,
  previousVector,
  currentVector,
  planeNormal,
  previousWorldQuaternion,
  currentWorldQuaternion,
  previousFrameQuaternion,
  currentFrameQuaternion,
) {
  if (!_sameRigidBodyMemberPair(world, entityId, counterpartEntityId)) {
    return {
      previousVector,
      currentVector,
      planeNormal,
    };
  }

  const previousBodyOrientation = _bodyOrientationFromMemberFrame(
    previousWorldQuaternion,
    previousFrameQuaternion,
  );
  const currentBodyOrientation = _bodyOrientationFromMemberFrame(
    currentWorldQuaternion,
    currentFrameQuaternion,
  );
  if (!previousBodyOrientation || !currentBodyOrientation) {
    return {
      previousVector,
      currentVector,
      planeNormal,
    };
  }

  const previousBodyInverse = previousBodyOrientation.clone().conjugate().normalize();
  const currentBodyInverse = currentBodyOrientation.clone().conjugate().normalize();
  const framePlaneNormal = currentBodyInverse.transformVector(planeNormal.clone());
  if (framePlaneNormal.lengthSq() > EPSILON) {
    framePlaneNormal.normalize();
  }

  return {
    previousVector: previousBodyInverse.transformVector(previousVector),
    currentVector: currentBodyInverse.transformVector(currentVector),
    planeNormal: framePlaneNormal,
  };
}

export function _orientationAngleForEntity(world, entityId, quaternion, localQuaternion = null) {
  const linkComp = world.getComponent(entityId, CableLinkComponent);
  if (linkComp?.cablePlaneNormalLocal) {
    return _localTwistAngle(localQuaternion || quaternion, linkComp.cablePlaneNormalLocal);
  }
  return _orientationAngleAroundAxis(quaternion, _getPlaneNormal(world, entityId));
}

export function _deltaAngleForEntity(
  world,
  entityId,
  prevQuat,
  currQuat,
  prevLocalQuat = null,
  currLocalQuat = null,
) {
  const linkComp = world.getComponent(entityId, CableLinkComponent);
  if (linkComp?.cablePlaneNormalLocal) {
    return _deltaAngleAroundAxis(
      prevLocalQuat || prevQuat,
      currLocalQuat || currQuat,
      linkComp.cablePlaneNormalLocal,
      true,
    );
  }
  return _deltaAngleAroundAxis(prevQuat, currQuat, _getPlaneNormal(world, entityId));
}

function _attachmentRelativeOrientationForEntity(
  world,
  entityId,
  center,
  attachmentPoint,
  quaternion = null,
  orientationAngle = null,
  localQuaternion = null,
) {
  if (!center || !attachmentPoint) {
    return null;
  }

  const linkComp = world.getComponent(entityId, CableLinkComponent);
  if (linkComp?.cablePlaneNormalLocal) {
    const orientationQuat = quaternion ?? getEntityWorldOrientation(world, entityId);
    if (!orientationQuat) {
      return null;
    }
    const relWorld = attachmentPoint.clone().subtract(center);
    const invOrientation = orientationQuat.clone().conjugate().normalize();
    const relLocal = invOrientation.transformVector(relWorld);
    const basisLocal = _planeBasisForAxis(linkComp.cablePlaneNormalLocal);
    return Math.atan2(relLocal.dot(basisLocal.v), relLocal.dot(basisLocal.u));
  }

  const basis = _planeBasisForAxis(_getPlaneNormal(world, entityId));
  const relWorld = attachmentPoint.clone().subtract(center);
  const attachmentAngleWorld = Math.atan2(relWorld.dot(basis.v), relWorld.dot(basis.u));
  const resolvedOrientationAngle = Number.isFinite(orientationAngle)
    ? orientationAngle
    : _orientationAngleForEntity(
      world,
      entityId,
      quaternion ?? getEntityWorldOrientation(world, entityId),
      localQuaternion,
    );
  return _normalizeAngleSigned(attachmentAngleWorld - resolvedOrientationAngle);
}

function _applyAxisAngleDelta(quaternion, axis, deltaAngle) {
  if (!quaternion || !Number.isFinite(deltaAngle) || Math.abs(deltaAngle) <= EPSILON) {
    return;
  }
  const normalizedAxis = (axis ?? DEFAULT_PLANE_NORMAL).clone();
  if (normalizedAxis.lengthSq() <= EPSILON) {
    return;
  }
  normalizedAxis.normalize();
  const dq = new Quaternion().setFromAxisAngle(normalizedAxis, deltaAngle);
  quaternion.multiplyQuaternions(dq, quaternion).normalize();
}

function _syncRigidBodyMemberLocalOrientationIfNeeded(world, entityId) {
  if (!world.getComponent(entityId, RigidBodyMemberComponent)) {
    return;
  }
  updateRigidBodyMemberLocalOrientation(world, entityId);
}

function _applyEntityAxisAngleDelta(world, entityId, quaternion, axis, deltaAngle) {
  _applyAxisAngleDelta(quaternion, axis, deltaAngle);
  const orientationComp = world.getComponent(entityId, OrientationComponent);
  if (!orientationComp?.quaternion) {
    return;
  }
  _applyAxisAngleDelta(orientationComp.quaternion, axis, deltaAngle);
  _syncRigidBodyMemberLocalOrientationIfNeeded(world, entityId);
}

function _hybridTransitionArcThreshold(path) {
  const halfWidth = Number.isFinite(path?.cableHalfWidth) ? Math.max(0.0, path.cableHalfWidth) : 0.0;
  if (!(halfWidth > 1e-9)) {
    return 1e-6;
  }
  return Math.max(1e-6, 0.25 * halfWidth);
}

function _layerWrapParams(r0, dr, rampLength, layerIndex) {
  const twoPi = 2.0 * Math.PI;
  const rn = r0 + dr * layerIndex;

  let dPhiRamp = 0.0;
  if (rampLength > EPSILON) {
    dPhiRamp = rampLength / (rn + 0.5 * dr);
    if (dPhiRamp > twoPi) dPhiRamp = twoPi;
    if (dPhiRamp < 0.0) dPhiRamp = 0.0;
  }

  const phiConst = twoPi - dPhiRamp;
  const Lconst = rn * phiConst;
  const Lwrap = Lconst + dPhiRamp * (rn + 0.5 * dr);
  return { rn, dPhiRamp, phiConst, Lconst, Lwrap };
}

function _storedInWrapAtPhi(phi, dr, wrap) {
  const twoPi = 2.0 * Math.PI;
  const phiClamped = Math.max(0.0, Math.min(twoPi, phi));
  if (!(wrap.dPhiRamp > EPSILON) || phiClamped <= wrap.phiConst + EPSILON) {
    return wrap.rn * Math.min(wrap.phiConst, phiClamped);
  }

  const x = phiClamped - wrap.phiConst;
  const a = dr / (2.0 * wrap.dPhiRamp);
  return wrap.Lconst + wrap.rn * x + a * x * x;
}

function _thetaToStoredLength(theta, baseRadius, halfWidth, rampLength) {
  if (!Number.isFinite(theta) || Math.abs(theta) <= EPSILON) {
    return 0.0;
  }
  if (theta < 0.0) {
    return -_thetaToStoredLength(-theta, baseRadius, halfWidth, rampLength);
  }

  const r0 = baseRadius + halfWidth;
  const dr = 2.0 * halfWidth;
  if (!(r0 > EPSILON) || !(dr > EPSILON)) {
    const linearRadius = Number.isFinite(baseRadius) ? Math.max(baseRadius, 0.0) : 0.0;
    return linearRadius * theta;
  }

  const LrampTarget = Math.max(0.0, rampLength ?? 0.0);
  const twoPi = 2.0 * Math.PI;
  const MAX_LAYERS = 2048;

  let remainingTheta = theta;
  let stored = 0.0;
  let n = 0;
  while (remainingTheta > twoPi + EPSILON && n < MAX_LAYERS) {
    const wrap = _layerWrapParams(r0, dr, LrampTarget, n);
    stored += wrap.Lwrap;
    remainingTheta -= twoPi;
    n += 1;
  }

  if (n >= MAX_LAYERS) {
    const rn = r0 + dr * MAX_LAYERS;
    return stored + rn * remainingTheta;
  }

  const wrap = _layerWrapParams(r0, dr, LrampTarget, n);
  stored += _storedInWrapAtPhi(remainingTheta, dr, wrap);
  return stored;
}

function _storedToRadiusAndTheta(storedLength, baseRadius, halfWidth, rampLength) {
  const stored = Math.max(0.0, storedLength ?? 0.0);
  const twoPi = 2.0 * Math.PI;
  const r0 = baseRadius + halfWidth;
  const dr = 2.0 * halfWidth;
  const LrampTarget = Math.max(0.0, rampLength ?? 0.0);

  if (!(r0 > EPSILON) || !(dr > EPSILON)) {
    const linearRadius = Number.isFinite(baseRadius) ? Math.max(baseRadius, 0.0) : 0.0;
    const theta = linearRadius > EPSILON ? (stored / linearRadius) : 0.0;
    return { radius: linearRadius, theta, layer: 0, phi: theta, inRamp: false };
  }

  let s = stored;
  let thetaBase = 0.0;
  let n = 0;
  const MAX_LAYERS = 2048;

  while (n < MAX_LAYERS) {
    const wrap = _layerWrapParams(r0, dr, LrampTarget, n);
    if (s > wrap.Lwrap + EPSILON) {
      s -= wrap.Lwrap;
      thetaBase += twoPi;
      n += 1;
      continue;
    }

    if (s <= wrap.Lconst + EPSILON || !(wrap.dPhiRamp > EPSILON)) {
      const phi = wrap.rn > EPSILON ? Math.min(wrap.phiConst, s / wrap.rn) : 0.0;
      return { radius: wrap.rn, theta: thetaBase + phi, layer: n, phi, inRamp: false };
    }

    const sRamp = Math.max(0.0, s - wrap.Lconst);
    const a = dr / (2.0 * wrap.dPhiRamp);
    const b = wrap.rn;
    const disc = b * b + 4.0 * a * sRamp;
    const x = (-b + Math.sqrt(Math.max(0.0, disc))) / (2.0 * a);
    const xClamped = Math.max(0.0, Math.min(wrap.dPhiRamp, x));
    const alpha = xClamped / wrap.dPhiRamp;
    const radius = wrap.rn + dr * alpha;
    const phi = wrap.phiConst + xClamped;
    return { radius, theta: thetaBase + phi, layer: n, phi, inRamp: true };
  }

  const rn = r0 + dr * MAX_LAYERS;
  return { radius: rn, theta: thetaBase, layer: MAX_LAYERS, phi: 0.0, inRamp: false };
}

function _storedToThetaSigned(storedLength, baseRadius, halfWidth, rampLength) {
  if (!Number.isFinite(storedLength) || Math.abs(storedLength) <= EPSILON) {
    return 0.0;
  }
  if (storedLength < 0.0) {
    return -_storedToRadiusAndTheta(-storedLength, baseRadius, halfWidth, rampLength).theta;
  }
  return _storedToRadiusAndTheta(storedLength, baseRadius, halfWidth, rampLength).theta;
}

function _hybridStoredDeltaFromRotation(thetaBefore, deltaAngle, cw, baseRadius, halfWidth, radiusFallback) {
  const signedDeltaTheta = (cw ? 1.0 : -1.0) * deltaAngle;
  if (!Number.isFinite(signedDeltaTheta) || Math.abs(signedDeltaTheta) <= EPSILON) {
    return 0.0;
  }

  if (Number.isFinite(baseRadius) && Number.isFinite(halfWidth) && halfWidth > EPSILON) {
    const rampLength = Math.max(0.0, baseRadius) * KNOT_SPAN;
    const thetaStart = Number.isFinite(thetaBefore) ? thetaBefore : 0.0;
    const thetaEnd = thetaStart + signedDeltaTheta;
    const storedStart = _thetaToStoredLength(thetaStart, baseRadius, halfWidth, rampLength);
    const storedEnd = _thetaToStoredLength(thetaEnd, baseRadius, halfWidth, rampLength);
    return storedEnd - storedStart;
  }

  const fallbackRadius = Number.isFinite(radiusFallback)
    ? radiusFallback
    : (Number.isFinite(baseRadius) ? baseRadius : 0.0);
  return signedDeltaTheta * fallbackRadius;
}

function _hybridAngleCorrectionFromStoredShift(
  thetaBefore,
  deltaAngle,
  cw,
  baseRadius,
  halfWidth,
  storedShift,
  radiusFallback
) {
  if (!Number.isFinite(storedShift) || Math.abs(storedShift) <= EPSILON) {
    return 0.0;
  }

  const signedDirection = cw ? 1.0 : -1.0;
  const signedDeltaTheta = signedDirection * deltaAngle;
  if (!Number.isFinite(signedDeltaTheta)) {
    return 0.0;
  }

  if (Number.isFinite(baseRadius) && Number.isFinite(halfWidth) && halfWidth > EPSILON) {
    const rampLength = Math.max(0.0, baseRadius) * KNOT_SPAN;
    const thetaStart = Number.isFinite(thetaBefore) ? thetaBefore : 0.0;
    const storedAfterCurrent = _thetaToStoredLength(thetaStart + signedDeltaTheta, baseRadius, halfWidth, rampLength);
    const storedAfterTarget = storedAfterCurrent + storedShift;
    const thetaAfterTarget = _storedToThetaSigned(storedAfterTarget, baseRadius, halfWidth, rampLength);
    if (!Number.isFinite(thetaAfterTarget)) {
      return 0.0;
    }
    const deltaAngleTarget = (thetaAfterTarget - thetaStart) / signedDirection;
    if (!Number.isFinite(deltaAngleTarget)) {
      return 0.0;
    }
    return deltaAngleTarget - deltaAngle;
  }

  const fallbackRadius = Number.isFinite(radiusFallback)
    ? radiusFallback
    : (Number.isFinite(baseRadius) ? baseRadius : 0.0);
  if (!(Math.abs(fallbackRadius) > EPSILON)) {
    return 0.0;
  }
  return storedShift / (signedDirection * fallbackRadius);
}

function _effectiveRollingRadius(world, path, linkIndex, baseRadius) {
  if (!layeringEnabled(world) || !Number.isFinite(baseRadius) || !path || !Array.isArray(path.linkTypes) || !Array.isArray(path.stored)) {
    return { radius: baseRadius, theta: 0.0 };
  }

  const halfWidth = path.cableHalfWidth ?? 0.0;
  if (!(halfWidth > EPSILON)) {
    return { radius: baseRadius, theta: 0.0 };
  }

  const effectiveRadius = baseRadius + halfWidth;
  const isEndpoint = linkIndex === 0 || linkIndex === path.linkTypes.length - 1;
  if (!isEndpoint || !_isHybrid(path.linkTypes[linkIndex])) {
    return { radius: effectiveRadius, theta: 0.0 };
  }

  const stored = Math.max(0.0, path.stored[linkIndex] ?? 0.0);
  if (!(stored > EPSILON)) {
    return { radius: effectiveRadius, theta: 0.0 };
  }

  const { radius, theta } = _storedToRadiusAndTheta(stored, baseRadius, halfWidth, baseRadius * KNOT_SPAN);
  return { radius: Math.max(effectiveRadius, radius), theta };
}

function _pathLinkIndicesForEntity(world, path, entityId) {
  if (!world || !path || entityId === undefined || entityId === null) {
    return [];
  }
  if (!Array.isArray(path.linkTypes) || !Array.isArray(path.jointEntities)) {
    return [];
  }
  if (path.linkTypes.length < 1 || path.jointEntities.length < 1) {
    return [];
  }

  const indices = [];
  const firstJoint = world.getComponent(path.jointEntities[0], CableJointComponent);
  if (firstJoint && firstJoint.entityA === entityId) {
    indices.push(0);
  }

  for (let i = 1; i < path.linkTypes.length - 1; i++) {
    const leftJoint = world.getComponent(path.jointEntities[i - 1], CableJointComponent);
    const rightJoint = world.getComponent(path.jointEntities[i], CableJointComponent);
    if (!leftJoint || !rightJoint) {
      continue;
    }
    if (leftJoint.entityB === entityId && rightJoint.entityA === entityId) {
      indices.push(i);
    }
  }

  const lastJoint = world.getComponent(path.jointEntities[path.jointEntities.length - 1], CableJointComponent);
  const lastIndex = path.linkTypes.length - 1;
  if (lastJoint && lastJoint.entityB === entityId) {
    indices.push(lastIndex);
  }
  return indices;
}

function _effectivePathRadiusForEntity(world, path, entityId, preferredLinkIndex = null) {
  const baseRadius = world.getComponent(entityId, RadiusComponent)?.radius;
  if (!Number.isFinite(baseRadius) || baseRadius <= EPSILON) {
    return baseRadius;
  }

  let effectiveRadius = baseRadius;
  if (Number.isInteger(preferredLinkIndex) && preferredLinkIndex >= 0 && preferredLinkIndex < path.linkTypes.length) {
    effectiveRadius = Math.max(
      effectiveRadius,
      _effectiveRollingRadius(world, path, preferredLinkIndex, baseRadius).radius
    );
  }

  const candidateIndices = _pathLinkIndicesForEntity(world, path, entityId);
  for (const linkIndex of candidateIndices) {
    effectiveRadius = Math.max(
      effectiveRadius,
      _effectiveRollingRadius(world, path, linkIndex, baseRadius).radius
    );
  }
  return effectiveRadius;
}

function _knotPathKey(pathId) {
  if (pathId === null || pathId === undefined) {
    return '__default__';
  }
  return String(pathId);
}

function _hybridKnotAngleForPath(knotComp, pathId) {
  if (!knotComp) {
    return null;
  }
  if (pathId !== null && pathId !== undefined) {
    const pathAngles = knotComp.pathAngles;
    const key = _knotPathKey(pathId);
    if (pathAngles && typeof pathAngles === 'object' && !Array.isArray(pathAngles)) {
      const value = pathAngles[key];
      if (Number.isFinite(value)) {
        return value;
      }
    }
    return null;
  }
  return Number.isFinite(knotComp.angle) ? knotComp.angle : null;
}

function _setHybridKnotAngleForPath(knotComp, pathId, angle) {
  if (!knotComp || !Number.isFinite(angle)) {
    return;
  }
  if (!knotComp.pathAngles || typeof knotComp.pathAngles !== 'object' || Array.isArray(knotComp.pathAngles)) {
    knotComp.pathAngles = {};
  }
  knotComp.pathAngles[_knotPathKey(pathId)] = angle;
  knotComp.angle = angle;
}

function _copyDefaultKnotAngleToPathIfUnambiguous(knotComp, pathId) {
  if (!knotComp || pathId === null || pathId === undefined) {
    return false;
  }
  const pathAngles = knotComp.pathAngles;
  if (!pathAngles || typeof pathAngles !== 'object' || Array.isArray(pathAngles)) {
    return false;
  }
  const defaultAngle = pathAngles[_knotPathKey(null)];
  if (!Number.isFinite(defaultAngle)) {
    return false;
  }
  let nonDefaultPathCount = 0;
  for (const key in pathAngles) {
    if (key === _knotPathKey(null)) {
      continue;
    }
    if (Number.isFinite(pathAngles[key])) {
      nonDefaultPathCount += 1;
    }
  }
  if (nonDefaultPathCount > 0) {
    return false;
  }
  _setHybridKnotAngleForPath(knotComp, pathId, defaultAngle);
  return true;
}

function _ensureHybridKnotAngleComponentForEndpoint(world, path, endpointIndex, pathId = null, options = null) {
  if (!world || !path || !Array.isArray(path.linkTypes) || !Array.isArray(path.jointEntities)) {
    return;
  }
  if (endpointIndex !== 0 && endpointIndex !== path.linkTypes.length - 1) {
    return;
  }
  if (path.linkTypes[endpointIndex] !== 'hybrid' || !(path.jointEntities.length > 0)) {
    return;
  }

  let joint = null;
  let entityId = null;
  let attachmentPoint = null;
  if (endpointIndex === 0) {
    joint = world.getComponent(path.jointEntities[0], CableJointComponent);
    if (joint) {
      entityId = joint.entityA;
      attachmentPoint = joint.attachmentPointA_world;
    }
  } else {
    joint = world.getComponent(path.jointEntities[path.jointEntities.length - 1], CableJointComponent);
    if (joint) {
      entityId = joint.entityB;
      attachmentPoint = joint.attachmentPointB_world;
    }
  }

  const createIfMissing = options && typeof options.createIfMissing === 'boolean'
    ? options.createIfMissing
    : true;
  const resolvedAttachmentPoint = options?.attachmentPoint ?? attachmentPoint;
  if (entityId === null || entityId === undefined || !resolvedAttachmentPoint) {
    return;
  }

  let knotComp = world.getComponent(entityId, HybridKnotAngleComponent);
  if (_hybridKnotAngleForPath(knotComp, pathId) !== null) {
    return;
  }
  if (_copyDefaultKnotAngleToPathIfUnambiguous(knotComp, pathId)) {
    return;
  }
  if (!knotComp && createIfMissing === false) {
    return;
  }

  const center = options?.center ?? getEntityWorldPosition(world, entityId);
  if (!center) {
    return;
  }

  const baseRadius = world.getComponent(entityId, RadiusComponent)?.radius;
  const pathHalfWidth = layeringEnabled(world) ? (path.cableHalfWidth ?? 0.0) : 0.0;
  const rampLength = Math.max(0.0, baseRadius ?? 0.0) * KNOT_SPAN;
  const thetaAtState = Math.abs(
    _storedToThetaSigned(path.stored[endpointIndex] ?? 0.0, baseRadius, pathHalfWidth, rampLength)
  );
  const cwAtState = _effectiveCW(path, endpointIndex, endpointIndex === 0);
  const thetaSignAtState = endpointIndex === 0
    ? (cwAtState ? -1.0 : 1.0)
    : (cwAtState ? 1.0 : -1.0);
  const thetaSignedAtState = thetaSignAtState * thetaAtState;
  const currentWorldOrientation = options?.quaternion ?? getEntityWorldOrientation(world, entityId);
  const currentLocalOrientation = options?.localQuaternion
    ?? _currentLocalOrientationForEntity(world, entityId, currentWorldOrientation);
  const orientation = Number.isFinite(options?.orientation)
    ? options.orientation
    : _orientationAngleForEntity(world, entityId, currentWorldOrientation, currentLocalOrientation);
  const relAttachmentAngle = _attachmentRelativeOrientationForEntity(
    world,
    entityId,
    center,
    resolvedAttachmentPoint,
    currentWorldOrientation,
    orientation,
    currentLocalOrientation,
  );
  if (!Number.isFinite(relAttachmentAngle)) {
    return;
  }
  const knotAngle = _normalizeAngleSigned(relAttachmentAngle - thetaSignedAtState);
  if (!Number.isFinite(knotAngle)) {
    return;
  }

  if (!knotComp) {
    knotComp = new HybridKnotAngleComponent(knotAngle);
    world.addComponent(entityId, knotComp);
  }
  _setHybridKnotAngleForPath(knotComp, pathId, knotAngle);
}

function _clearDebugPoints(world) {
  const debugPoints = world.getResource('debugRenderPoints');
  if (debugPoints) {
      for (const key in debugPoints) {
          delete debugPoints[key];
      }
  }
}

function _isAttachment(value) {
  return value === 'attachment' || value === 'hybrid-attachment' || value === 'pinhole';
}

function _isRolling(value) {
  return value === 'rolling' || value === 'hybrid';
}

function _isHybrid(value) {
  return value === 'hybrid' || value === 'hybrid-attachment';
}

function _hasAxisOnlyCableSpinDof(world, entityId) {
  const spoolState = world.getComponent(entityId, SpoolStateComponent);
  const linkComp = world.getComponent(entityId, CableLinkComponent);
  return Boolean(spoolState && linkComp?.cablePlaneNormalLocal);
}

function _positionStepperConstraintStiffness(world, stepper) {
  if (!stepper || stepper.torqueMode) {
    return 0.0;
  }

  if (isStepperClosedLoopEnabled(world, stepper)) {
    return Math.max(
      0.0,
      _readFiniteResource(
        world,
        'closedLoopStepperConstraintStiffness',
        DEFAULT_CLOSED_LOOP_STEPPER_CONSTRAINT_STIFFNESS,
      ),
    );
  }

  const holdingTorque = Number.isFinite(stepper.holdingTorque)
    ? Math.max(0.0, stepper.holdingTorque)
    : 0.0;
  const polePairs = Number.isFinite(stepper.numPolePairs)
    ? Math.max(1.0, Math.abs(stepper.numPolePairs))
    : 1.0;
  const scale = Math.max(
    0.0,
    _readFiniteResource(
      world,
      'openLoopStepperConstraintStiffnessScale',
      DEFAULT_OPEN_LOOP_STEPPER_CONSTRAINT_STIFFNESS_SCALE,
    ),
  );

  return holdingTorque * polePairs * scale;
}

function _effectiveMotorizedSpinInvInertia(world, entityId, invInertia, dt) {
  if (!(invInertia > EPSILON)) {
    return 0.0;
  }

  const stepper = world.getComponent(entityId, StepperMotorComponent);
  const stiffness = _positionStepperConstraintStiffness(world, stepper);
  if (!(stiffness > EPSILON) || !(Number.isFinite(dt) && dt > EPSILON)) {
    return invInertia;
  }

  const inertia = world.getComponent(entityId, MomentOfInertiaComponent)?.inertia;
  const effectiveInertia = (
    Number.isFinite(inertia) && inertia > EPSILON
      ? inertia
      : (1.0 / invInertia)
  ) + (stiffness * dt * dt);
  return effectiveInertia > EPSILON ? (1.0 / effectiveInertia) : invInertia;
}

function _linkAllowsCableSpin(world, path, linkIndex, entityId) {
  if (!_hasAxisOnlyCableSpinDof(world, entityId)) {
    return false;
  }
  const linkType = path?.linkTypes?.[linkIndex];
  if (_isHybrid(linkType) || linkType === 'rolling') {
    return true;
  }
  return linkType !== undefined;
}

function _isSpinBackdrivableThroughPinhole(linkType) {
  return linkType === 'rolling' || _isHybrid(linkType);
}

export function calculateAttachmentPoints(world, joint, path, i, radiusA, radiusB) {
  const A = i;
  const B = i + 1;

  const entityA = joint.entityA;
  const entityB = joint.entityB;

  const linkAComp = world.getComponent(entityA, CableLinkComponent);

  const posA = getEntityWorldPosition(world, entityA);
  const attachmentA_previous = joint.attachmentPointA_world;
  const prevPosA = linkAComp?.prevCableAttachmentTimePos;
  const planeNormalA = _getPlaneNormal(world, entityA);
  const currentQuatA = getEntityWorldOrientation(world, entityA);
  const prevQuatA = linkAComp?.prevCableAttachmentTimeOrientation;
  const frameQuatA = _orientationFrameForEndpoint(
    world,
    entityA,
    entityB,
    currentQuatA,
    prevQuatA,
    linkAComp,
  );
  const deltaAngleA = _deltaAngleForEntity(
    world,
    entityA,
    prevQuatA,
    currentQuatA,
    frameQuatA.previous,
    frameQuatA.current,
  );

  const cwA = _effectiveCW(path, A, true);
  const rollingLinkA = _isRolling(path.linkTypes[A]);
  const isHybridA = _isHybrid(path.linkTypes[A]);

  const pADiffFromTranslation = (posA && prevPosA) ? posA.clone().subtract(prevPosA) : null;
  let pADiffFromRotation = new Vector3(0, 0, 0);
  if (prevPosA) {
    const rotated = attachmentA_previous.clone().subtract(prevPosA).rotateAroundAxis(planeNormalA, deltaAngleA).add(prevPosA);
    pADiffFromRotation = rotated.clone().subtract(attachmentA_previous);
  }

  const linkBComp = world.getComponent(entityB, CableLinkComponent);

  const posB = getEntityWorldPosition(world, entityB);
  const attachmentB_previous = joint.attachmentPointB_world;
  const prevPosB = linkBComp?.prevCableAttachmentTimePos;
  const planeNormalB = _getPlaneNormal(world, entityB);
  const currentQuatB = getEntityWorldOrientation(world, entityB);
  const prevQuatB = linkBComp?.prevCableAttachmentTimeOrientation;
  const frameQuatB = _orientationFrameForEndpoint(
    world,
    entityB,
    entityA,
    currentQuatB,
    prevQuatB,
    linkBComp,
  );
  const deltaAngleB = _deltaAngleForEntity(
    world,
    entityB,
    prevQuatB,
    currentQuatB,
    frameQuatB.previous,
    frameQuatB.current,
  );

  const cwB = _effectiveCW(path, B, false);
  const rollingLinkB = _isRolling(path.linkTypes[B]);
  const isHybridB = _isHybrid(path.linkTypes[B]);

  const pBDiffFromTranslation = (posB && prevPosB) ? posB.clone().subtract(prevPosB) : null;
  let pBDiffFromRotation = new Vector3(0, 0, 0);
  if (prevPosB) {
    const rotated = attachmentB_previous.clone().subtract(prevPosB).rotateAroundAxis(planeNormalB, deltaAngleB).add(prevPosB);
    pBDiffFromRotation = rotated.clone().subtract(attachmentB_previous);
  }

  let attachmentA_current = posA ? posA.clone() : null;
  let attachmentB_current = posB ? posB.clone() : null;

  const rotatedHybridA = path.linkTypes[A] === 'hybrid-attachment' && Math.abs(deltaAngleA) > EPSILON;
  const rotatedHybridB = path.linkTypes[B] === 'hybrid-attachment' && Math.abs(deltaAngleB) > EPSILON;
  const useCommonRollingTangent = (
    rollingLinkA &&
    rollingLinkB &&
    _planesAreParallel(planeNormalA, planeNormalB) &&
    !rotatedHybridA &&
    !rotatedHybridB
  );

  if (useCommonRollingTangent) {
    if (posA && posB && radiusA !== undefined && radiusB !== undefined) {
      const tangents = tangentFromSphereToSphere(posA, radiusA, cwA, posB, radiusB, cwB, planeNormalA);
      attachmentA_current = tangents.a_sphere;
      attachmentB_current = tangents.b_sphere;
    }
  } else {
    if ((rotatedHybridA || !rollingLinkA) && isHybridA && pADiffFromTranslation) {
      attachmentA_current = attachmentA_previous.clone().add(pADiffFromTranslation).add(pADiffFromRotation);
    }

    if ((rotatedHybridB || !rollingLinkB) && isHybridB && pBDiffFromTranslation) {
      attachmentB_current = attachmentB_previous.clone().add(pBDiffFromTranslation).add(pBDiffFromRotation);
    }

    if (rollingLinkA && !rotatedHybridA) {
      attachmentA_current = _deriveRollingSideAttachment(posA, attachmentB_current, radiusA, planeNormalA, cwA, true);
    }

    if (rollingLinkB && !rotatedHybridB) {
      attachmentB_current = _deriveRollingSideAttachment(posB, attachmentA_current, radiusB, planeNormalB, cwB, false);
    }
  }

  return { attachmentA_current, attachmentB_current };
}

function _isNonCenterAttachment(point, center) {
  if (!point || !center) {
    return false;
  }
  const distance = point.distanceTo(center);
  if (!Number.isFinite(distance)) {
    return false;
  }
  return Math.abs(distance) >= 1e-4;
}

export function _updateAttachmentPoints(world) {
  const logDiffA = false;
  const logDiffB = false;
  const allowRestLengthClamp = true;
  const enforceKnotPhase = true;
  const step = Math.floor(_readFiniteResource(world, 'cableHybridTransitionStep', 0));
  const pathEntities = world.query([CablePathComponent]);

  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    const pathHalfWidth = layeringEnabled(world) ? (path.cableHalfWidth ?? 0.0) : 0.0;

    for (let i = 0; i < path.jointEntities.length; i++) {
      const jointId = path.jointEntities[i];
      const joint = world.getComponent(jointId, CableJointComponent);

      const attachmentA_previous = joint.attachmentPointA_world.clone();
      const attachmentB_previous = joint.attachmentPointB_world.clone();

      const A = i;
      const B = i + 1;
      const entityA = joint.entityA;
      const entityB = joint.entityB;

      const linkAComp = world.getComponent(entityA, CableLinkComponent);
      const posA = getEntityWorldPosition(world, entityA);
      const prevPosA = linkAComp?.prevCableAttachmentTimePos;
      const baseRadiusA = world.getComponent(entityA, RadiusComponent)?.radius;
      const { radius: radiusA, theta: thetaA } = _effectiveRollingRadius(world, path, A, baseRadiusA);
      const planeNormalA = _getPlaneNormal(world, entityA);
      const currentQuatA = getEntityWorldOrientation(world, entityA);
      const prevQuatA = linkAComp?.prevCableAttachmentTimeOrientation;
      const frameQuatA = _orientationFrameForEndpoint(
        world,
        entityA,
        entityB,
        currentQuatA,
        prevQuatA,
        linkAComp,
      );
      const angleA = _orientationAngleForEntity(world, entityA, currentQuatA, frameQuatA.current);
      const prevAngleA = _orientationAngleForEntity(world, entityA, prevQuatA, frameQuatA.previous);
      const deltaAngleA = _deltaAngleForEntity(
        world,
        entityA,
        prevQuatA,
        currentQuatA,
        frameQuatA.previous,
        frameQuatA.current,
      );
      const cwA = _effectiveCW(path, A, true);
      const rollingLinkA = _isRolling(path.linkTypes[A]);
      const isHybridA = _isHybrid(path.linkTypes[A]);
      // Rolling links are non-slip by link type. Material friction only belongs
      // to the secondary CableFrictionSystem tension-transfer pass.
      const transfersRollingRotationA = rollingLinkA && !isHybridA;

      const linkBComp = world.getComponent(entityB, CableLinkComponent);
      const posB = getEntityWorldPosition(world, entityB);
      const prevPosB = linkBComp?.prevCableAttachmentTimePos;
      const baseRadiusB = world.getComponent(entityB, RadiusComponent)?.radius;
      const { radius: radiusB, theta: thetaB } = _effectiveRollingRadius(world, path, B, baseRadiusB);
      const planeNormalB = _getPlaneNormal(world, entityB);
      const currentQuatB = getEntityWorldOrientation(world, entityB);
      const prevQuatB = linkBComp?.prevCableAttachmentTimeOrientation;
      const frameQuatB = _orientationFrameForEndpoint(
        world,
        entityB,
        entityA,
        currentQuatB,
        prevQuatB,
        linkBComp,
      );
      const angleB = _orientationAngleForEntity(world, entityB, currentQuatB, frameQuatB.current);
      const prevAngleB = _orientationAngleForEntity(world, entityB, prevQuatB, frameQuatB.previous);
      const deltaAngleB = _deltaAngleForEntity(
        world,
        entityB,
        prevQuatB,
        currentQuatB,
        frameQuatB.previous,
        frameQuatB.current,
      );
      const cwB = _effectiveCW(path, B, false);
      const rollingLinkB = _isRolling(path.linkTypes[B]);
      const isHybridB = _isHybrid(path.linkTypes[B]);
      const transfersRollingRotationB = rollingLinkB && !isHybridB;

      let { attachmentA_current, attachmentB_current } = calculateAttachmentPoints(
        world,
        joint,
        path,
        i,
        radiusA,
        radiusB
      );

      if (isHybridA && i === 0) {
        _ensureHybridKnotAngleComponentForEndpoint(world, path, A, pathId, {
          attachmentPoint: attachmentA_previous,
          center: prevPosA,
          orientation: prevAngleA,
          quaternion: prevQuatA,
          createIfMissing: false
        });
      }
      if (isHybridB && i === path.jointEntities.length - 1) {
        _ensureHybridKnotAngleComponentForEndpoint(world, path, B, pathId, {
          attachmentPoint: attachmentB_previous,
          center: prevPosB,
          orientation: prevAngleB,
          quaternion: prevQuatB,
          createIfMissing: false
        });
      }

      const knotCompA = world.getComponent(entityA, HybridKnotAngleComponent);
      const knotAngleA = _hybridKnotAngleForPath(knotCompA, pathId);
      const knotCompB = world.getComponent(entityB, HybridKnotAngleComponent);
      const knotAngleB = _hybridKnotAngleForPath(knotCompB, pathId);

      if (
        logDiffA &&
        i === 0 &&
        isHybridA &&
        knotAngleA !== null &&
        posA &&
        attachmentA_current &&
        Number.isFinite(thetaA)
      ) {
        const attachmentRelOrientationA = _attachmentRelativeOrientationForEntity(
          world,
          entityA,
          posA,
          attachmentA_current,
          currentQuatA,
          angleA,
          frameQuatA.current,
        );
        const thetaSignedA = (cwA ? -1.0 : 1.0) * thetaA;
        const knotAngleFromAttachmentA = _normalizeAngleSigned(attachmentRelOrientationA - thetaSignedA);
        const diffA = _normalizeAngleSigned(knotAngleFromAttachmentA - knotAngleA);
        console.log(`diffA: ${diffA}`);
      }

      if (
        logDiffB &&
        i === path.jointEntities.length - 1 &&
        isHybridB &&
        knotAngleB !== null &&
        posB &&
        attachmentB_current &&
        Number.isFinite(thetaB)
      ) {
        const attachmentRelOrientationB = _attachmentRelativeOrientationForEntity(
          world,
          entityB,
          posB,
          attachmentB_current,
          currentQuatB,
          angleB,
          frameQuatB.current,
        );
        const thetaSignedB = (cwB ? 1.0 : -1.0) * thetaB;
        const knotAngleFromAttachmentB = _normalizeAngleSigned(attachmentRelOrientationB - thetaSignedB);
        const diffB = _normalizeAngleSigned(knotAngleFromAttachmentB - knotAngleB);
        console.log(`diffB: ${diffB}`);
      }

      let sA = 0.0;
      let sB = 0.0;
      const restBefore = joint.restLength;
      const storedA_before = path.stored[A] ?? 0.0;
      const storedB_before = path.stored[B] ?? 0.0;

      if (
        rollingLinkA &&
        attachmentA_previous &&
        attachmentA_current &&
        prevPosA &&
        posA &&
        radiusA !== undefined &&
        _isNonCenterAttachment(attachmentA_previous, prevPosA, radiusA)
      ) {
        const arcFrameA = _arcFrameForEndpoint(
          world,
          entityA,
          entityB,
          attachmentA_previous.clone().subtract(prevPosA),
          attachmentA_current.clone().subtract(posA),
          planeNormalA,
          prevQuatA,
          currentQuatA,
          frameQuatA.previous,
          frameQuatA.current,
        );
        sA = signedArcLengthOnWheel(
          arcFrameA.previousVector,
          arcFrameA.currentVector,
          new Vector3(0.0, 0.0, 0.0),
          radiusA,
          cwA,
          arcFrameA.planeNormal
        );
        if (isHybridA) {
          sA += _hybridStoredDeltaFromRotation(
            thetaA,
            deltaAngleA,
            cwA,
            baseRadiusA,
            pathHalfWidth,
            radiusA
          );
        } else if (transfersRollingRotationA) {
          sA += (cwA ? deltaAngleA * radiusA : -deltaAngleA * radiusA);
        }
      }

      if (
        rollingLinkB &&
        attachmentB_previous &&
        attachmentB_current &&
        prevPosB &&
        posB &&
        radiusB !== undefined &&
        _isNonCenterAttachment(attachmentB_previous, prevPosB, radiusB)
      ) {
        const arcFrameB = _arcFrameForEndpoint(
          world,
          entityB,
          entityA,
          attachmentB_previous.clone().subtract(prevPosB),
          attachmentB_current.clone().subtract(posB),
          planeNormalB,
          prevQuatB,
          currentQuatB,
          frameQuatB.previous,
          frameQuatB.current,
        );
        sB = signedArcLengthOnWheel(
          arcFrameB.previousVector,
          arcFrameB.currentVector,
          new Vector3(0.0, 0.0, 0.0),
          radiusB,
          cwB,
          arcFrameB.planeNormal
        );
        if (isHybridB) {
          sB += _hybridStoredDeltaFromRotation(
            thetaB,
            deltaAngleB,
            cwB,
            baseRadiusB,
            pathHalfWidth,
            radiusB
          );
        } else if (transfersRollingRotationB) {
          sB += (cwB ? deltaAngleB * radiusB : -deltaAngleB * radiusB);
        }
      }

      const sA_raw = sA;
      const sB_raw = sB;
      let sAEffective = sA;
      let sBEffective = sB;
      let clampApplied = false;
      const clampEnabled = allowRestLengthClamp && _featureFlag(world, 'layeringClampJointRestLength', true);
      if (clampEnabled && Number.isFinite(restBefore)) {
        const unclampedRest = restBefore - sAEffective + sBEffective;
        if (unclampedRest < MIN_JOINT_REST_LENGTH) {
          const requiredLift = MIN_JOINT_REST_LENGTH - unclampedRest;
          const decreaseFromA = Math.max(0.0, sAEffective);
          const decreaseFromB = Math.max(0.0, -sBEffective);
          const totalDecrease = decreaseFromA + decreaseFromB;
          if (totalDecrease > EPSILON) {
            const shiftA = requiredLift * (decreaseFromA / totalDecrease);
            const shiftB = requiredLift - shiftA;
            sAEffective -= shiftA;
            sBEffective += shiftB;

            if ((isHybridB || transfersRollingRotationB) && currentQuatB) {
              if (isHybridB) {
                _applyEntityAxisAngleDelta(
                  world,
                  entityB,
                  currentQuatB,
                  planeNormalB,
                  _hybridAngleCorrectionFromStoredShift(
                    thetaB,
                    deltaAngleB,
                    cwB,
                    baseRadiusB,
                    pathHalfWidth,
                    shiftB,
                    radiusB
                  )
                );
              } else if (Math.abs((cwB ? 1.0 : -1.0) * radiusB) > EPSILON) {
                _applyEntityAxisAngleDelta(
                  world,
                  entityB,
                  currentQuatB,
                  planeNormalB,
                  shiftB / ((cwB ? 1.0 : -1.0) * radiusB),
                );
              }
            }

            if ((isHybridA || transfersRollingRotationA) && currentQuatA) {
              if (isHybridA) {
                _applyEntityAxisAngleDelta(
                  world,
                  entityA,
                  currentQuatA,
                  planeNormalA,
                  _hybridAngleCorrectionFromStoredShift(
                    thetaA,
                    deltaAngleA,
                    cwA,
                    baseRadiusA,
                    pathHalfWidth,
                    -shiftA,
                    radiusA
                  )
                );
              } else if (Math.abs((cwA ? 1.0 : -1.0) * radiusA) > EPSILON) {
                _applyEntityAxisAngleDelta(
                  world,
                  entityA,
                  currentQuatA,
                  planeNormalA,
                  -shiftA / ((cwA ? 1.0 : -1.0) * radiusA),
                );
              }
            }
          } else {
            sBEffective += requiredLift;
            if ((isHybridB || transfersRollingRotationB) && currentQuatB) {
              if (isHybridB) {
                _applyEntityAxisAngleDelta(
                  world,
                  entityB,
                  currentQuatB,
                  planeNormalB,
                  _hybridAngleCorrectionFromStoredShift(
                    thetaB,
                    deltaAngleB,
                    cwB,
                    baseRadiusB,
                    pathHalfWidth,
                    requiredLift,
                    radiusB
                  )
                );
              } else if (Math.abs((cwB ? 1.0 : -1.0) * radiusB) > EPSILON) {
                _applyEntityAxisAngleDelta(
                  world,
                  entityB,
                  currentQuatB,
                  planeNormalB,
                  requiredLift / ((cwB ? 1.0 : -1.0) * radiusB),
                );
              }
            }
          }

          const stillLowRest = restBefore - sAEffective + sBEffective;
          if (stillLowRest + EPSILON < MIN_JOINT_REST_LENGTH) {
            console.warn('Still low rest');
          }
          clampApplied = true;

          _recordCableEventTrace(world, step, {
            type: 'rest-length-clamp',
            stage: 'updateAttachmentPoints',
            pathId,
            jointId,
            jointIndex: i,
            entityA,
            entityB,
            linkTypeA: path.linkTypes[A],
            linkTypeB: path.linkTypes[B],
            restBefore,
            unclampedRest,
            clampedRest: restBefore - sAEffective + sBEffective,
            sA_raw,
            sB_raw,
            sA_effective: sAEffective,
            sB_effective: sBEffective
          });
        }
      }

      path.stored[A] += sAEffective;
      joint.restLength -= sAEffective;
      path.stored[B] -= sBEffective;
      joint.restLength += sBEffective;

      if (enforceKnotPhase && isHybridA && i === 0 && Number.isFinite(baseRadiusA)) {
        if (knotAngleA !== null && posA && attachmentA_current) {
          const rampLengthA = Math.max(0.0, baseRadiusA) * KNOT_SPAN;
          let thetaCurrentA = Math.abs(
            _storedToThetaSigned(path.stored[A] ?? 0.0, baseRadiusA, pathHalfWidth, rampLengthA)
          );
          if (!Number.isFinite(thetaCurrentA)) {
            thetaCurrentA = _effectiveRollingRadius(world, path, A, baseRadiusA).theta;
          }
          const thetaSignA = cwA ? -1.0 : 1.0;
          const attachmentRelOrientationA = _attachmentRelativeOrientationForEntity(
            world,
            entityA,
            posA,
            attachmentA_current,
            currentQuatA,
            angleA,
            frameQuatA.current,
          );
          const thetaSignedWrappedA = _normalizeAngleSigned(attachmentRelOrientationA - knotAngleA);
          const thetaSignedCurrentA = thetaSignA * thetaCurrentA;
          const thetaSignedTargetA = _unwrapAngleNear(thetaSignedCurrentA, thetaSignedWrappedA);
          const thetaTargetA = thetaSignA * thetaSignedTargetA;
          const storedTargetA = _thetaToStoredLength(thetaTargetA, baseRadiusA, pathHalfWidth, rampLengthA);
          if (Number.isFinite(storedTargetA)) {
            const storedBeforeProjectionA = path.stored[A] ?? 0.0;
            const storedDeltaA = storedTargetA - storedBeforeProjectionA;
            if (Math.abs(storedDeltaA) > 1e-3) {
              path.stored[A] = storedTargetA;
              joint.restLength -= storedDeltaA;
            }
          }
        }
      }

      if (enforceKnotPhase && isHybridB && i === path.jointEntities.length - 1 && Number.isFinite(baseRadiusB)) {
        if (knotAngleB !== null && posB && attachmentB_current) {
          const rampLengthB = Math.max(0.0, baseRadiusB) * KNOT_SPAN;
          let thetaCurrentB = Math.abs(
            _storedToThetaSigned(path.stored[B] ?? 0.0, baseRadiusB, pathHalfWidth, rampLengthB)
          );
          if (!Number.isFinite(thetaCurrentB)) {
            thetaCurrentB = _effectiveRollingRadius(world, path, B, baseRadiusB).theta;
          }
          const thetaSignB = cwB ? 1.0 : -1.0;
          const attachmentRelOrientationB = _attachmentRelativeOrientationForEntity(
            world,
            entityB,
            posB,
            attachmentB_current,
            currentQuatB,
            angleB,
            frameQuatB.current,
          );
          const thetaSignedWrappedB = _normalizeAngleSigned(attachmentRelOrientationB - knotAngleB);
          const thetaSignedCurrentB = thetaSignB * thetaCurrentB;
          const thetaSignedTargetB = _unwrapAngleNear(thetaSignedCurrentB, thetaSignedWrappedB);
          const thetaTargetB = thetaSignB * thetaSignedTargetB;
          const storedTargetB = _thetaToStoredLength(thetaTargetB, baseRadiusB, pathHalfWidth, rampLengthB);
          if (Number.isFinite(storedTargetB)) {
            const storedBeforeProjectionB = path.stored[B] ?? 0.0;
            const storedDeltaB = storedTargetB - storedBeforeProjectionB;
            if (Math.abs(storedDeltaB) > 1e-3) {
              path.stored[B] = storedTargetB;
              joint.restLength -= storedDeltaB;
            }
          }
        }
      }

      const restAfter = joint.restLength;
      if (
        !Number.isFinite(restAfter) ||
        restAfter < MIN_JOINT_REST_LENGTH
      ) {
        const centerDistance = (posA && posB) ? posA.distanceTo(posB) : null;
        const overlap = (
          Number.isFinite(radiusA) &&
          Number.isFinite(radiusB) &&
          Number.isFinite(centerDistance)
        )
          ? (radiusA + radiusB - centerDistance)
          : null;
        const bothEndsHybrid = _isHybrid(path.linkTypes[A]) && _isHybrid(path.linkTypes[B]);
        _recordCableEventTrace(world, step, {
          type: 'rest-length-anomaly',
          stage: 'updateAttachmentPoints',
          pathId,
          jointId,
          jointIndex: i,
          entityA,
          entityB,
          linkTypeA: path.linkTypes[A],
          linkTypeB: path.linkTypes[B],
          sameJointPath: path.jointEntities.length === 1,
          bothEndsHybrid,
          centerDistance,
          overlap,
          attachmentDistance: (attachmentA_current && attachmentB_current)
            ? attachmentA_current.distanceTo(attachmentB_current)
            : null,
          restBefore,
          restAfter,
          restDelta: restAfter - restBefore,
          clampApplied,
          sA_raw,
          sB_raw,
          sA_effective: sAEffective,
          sB_effective: sBEffective,
          storedA_before,
          storedB_before,
          storedA_after: path.stored[A] ?? 0.0,
          storedB_after: path.stored[B] ?? 0.0
        });
      }

      if (attachmentA_current) {
        joint.attachmentPointA_world.set(attachmentA_current);
      }
      if (attachmentB_current) {
        joint.attachmentPointB_world.set(attachmentB_current);
      }
    }
  }
}

export function _mergeJoints(world) {
  const step = Math.floor(_readFiniteResource(world, 'cableHybridTransitionStep', 0));
  const pathEntities = world.query([CablePathComponent]);
  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    if (path.jointEntities.length < 2) continue;
    const jointsInPath = path.jointEntities;
    let reRunMerge = true;
    while (reRunMerge) {
      reRunMerge = false;
      for (let i = 0; i < path.jointEntities.length - 1; i++) {
        if (path.linkTypes[i + 1] !== 'rolling') {
          continue;
        }
        const jointId_i = path.jointEntities[i];
        const jointId_i_plus_1 = path.jointEntities[i + 1];
        const joint_i = world.getComponent(jointId_i, CableJointComponent);
        const joint_i_plus_1 = world.getComponent(jointId_i_plus_1, CableJointComponent);
        const linkId = joint_i.entityB;
        const linkId2 = joint_i_plus_1.entityA;
        if (linkId !== linkId2) {
          console.warn("Merge loop saw disconnected cable path");
          continue;
        }
        if (!layeringEnabled(world) && joint_i.entityA === joint_i_plus_1.entityB) {
          continue;
        }
        if (path.stored[i + 1] < 0.0) {
          const storedLeftBefore = path.stored[i] ?? 0.0;
          const storedMiddleBefore = path.stored[i + 1] ?? 0.0;
          const storedRightBefore = path.stored[i + 2] ?? 0.0;
          const restLeftBefore = joint_i.restLength;
          const restRightBefore = joint_i_plus_1.restLength;
          const pathJointCountBefore = path.jointEntities.length;
          const pA1 = joint_i.attachmentPointA_world;
          const pB2 = joint_i_plus_1.attachmentPointB_world;
          const posA = getEntityWorldPosition(world, joint_i.entityA);
          const radiusA = _effectivePathRadiusForEntity(world, path, joint_i.entityA, i);
          const planeNormalA = _getPlaneNormal(world, joint_i.entityA);
          const cwA = _effectiveCW(path, i, true);
          const posB = getEntityWorldPosition(world, joint_i_plus_1.entityB);
          const radiusB = _effectivePathRadiusForEntity(world, path, joint_i_plus_1.entityB, i + 2);
          const planeNormalB = _getPlaneNormal(world, joint_i_plus_1.entityB);
          const cwB = path.cw[i + 2];

          joint_i.restLength += joint_i_plus_1.restLength + path.stored[i + 1];
          joint_i.entityB = joint_i_plus_1.entityB;
          const isAttachmentA = _isAttachment(path.linkTypes[i]);
          const isRollingA = _isRolling(path.linkTypes[i]);
          const isAttachmentB = _isAttachment(path.linkTypes[i + 2]);
          const isRollingB = _isRolling(path.linkTypes[i + 2]);

          let attachmentA_current = pA1;
          let attachmentB_current = pB2;
          if (isRollingA && isRollingB) {
            const tangents = tangentFromSphereToSphere(posA, radiusA, cwA, posB, radiusB, cwB, planeNormalA);
            attachmentA_current = tangents.a_sphere;
            attachmentB_current = tangents.b_sphere;
          } else if (isRollingA && isAttachmentB) {
            attachmentA_current = tangentFromSphereToPoint(pB2, posA, radiusA, planeNormalA, cwA).a_sphere;
          } else if (isAttachmentA && isRollingB) {
            attachmentB_current = tangentFromPointToSphere(pA1, posB, radiusB, planeNormalB, cwB).a_sphere;
          }

          let sA = 0.0;
          let sB = 0.0;
          if (isRollingA && isRollingB) {
            sA = signedArcLengthOnWheel(pA1, attachmentA_current, posA, radiusA, cwA, planeNormalA);
            sB = signedArcLengthOnWheel(pB2, attachmentB_current, posB, radiusB, cwB, planeNormalB);
          } else if (isRollingA && isAttachmentB) {
            sA = signedArcLengthOnWheel(pA1, attachmentA_current, posA, radiusA, cwA, planeNormalA);
          } else if (isAttachmentA && isRollingB) {
            sB = signedArcLengthOnWheel(pB2, attachmentB_current, posB, radiusB, cwB, planeNormalB);
          }

          path.stored[i] += sA;
          joint_i.restLength -= sA;
          path.stored[i + 2] -= sB;
          joint_i.restLength += sB;
          reRunMerge = path.stored[i] < 0.0 || path.stored[i + 2] < 0.0;

          _recordCableEventTrace(world, step, {
            type: 'merge',
            pathId,
            mergeIndex: i,
            jointKeptId: jointId_i,
            jointRemovedId: jointId_i_plus_1,
            pathJointCountBefore,
            pathJointCountAfter: pathJointCountBefore - 1,
            storedLeftBefore,
            storedMiddleBefore,
            storedRightBefore,
            storedLeftAfter: path.stored[i] ?? 0.0,
            storedRightAfter: path.stored[i + 2] ?? 0.0,
            restLeftBefore,
            restRightBefore,
            restAfter: joint_i.restLength,
            sA,
            sB,
            resultingEntityA: joint_i.entityA,
            resultingEntityB: joint_i.entityB,
            resultingTinyRest: Number.isFinite(joint_i.restLength) && joint_i.restLength < MIN_JOINT_REST_LENGTH,
            rerunMergeTriggered: reRunMerge
          });

          joint_i.attachmentPointA_world.set(attachmentA_current);
          joint_i.attachmentPointB_world.set(attachmentB_current);

          path.jointEntities.splice(i + 1, 1);
          path.stored.splice(i + 1, 1);
          path.cw.splice(i + 1, 1);
          path.linkTypes.splice(i + 1, 1);
          world.destroyEntity(jointId_i_plus_1);
        }
      }
    }
  }
}

export function _splitJoints(world) {
  const step = Math.floor(_readFiniteResource(world, 'cableHybridTransitionStep', 0));
  const potentialSplitters = world.query([PositionComponent, RadiusComponent, CableLinkComponent]);
  const pathEntities = world.query([CablePathComponent]);
  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    const pathMachine = getMachineId(world, pathId);
    if (path.jointEntities.length < 1) continue;
    for (let i = 0; i < path.jointEntities.length; i++) {
      const jointId = path.jointEntities[i];
      const joint = world.getComponent(jointId, CableJointComponent);

      const pA = joint.attachmentPointA_world;
      const pB = joint.attachmentPointB_world;
      for (const splitterId of potentialSplitters) {
        if (splitterId === joint.entityA || splitterId === joint.entityB) {
          continue;
        }
        if (getMachineId(world, splitterId) !== pathMachine) {
          continue;
        }
        const posSplitter = getEntityWorldPosition(world, splitterId);
        const radiusSplitter = _effectivePathRadiusForEntity(world, path, splitterId, null);
        if (lineSegmentSphereIntersection(pA, pB, posSplitter, radiusSplitter)) {
          const entityA = joint.entityA;
          const entityB = joint.entityB;
          const newJointId = world.createEntity();
          ensureMachineTag(world, newJointId, pathMachine);

          const posA = getEntityWorldPosition(world, entityA);
          const linkTypeA = path.linkTypes[i];
          const isAttachmentA = _isAttachment(linkTypeA);
          const isRollingA = _isRolling(linkTypeA);
          const radiusA = _effectivePathRadiusForEntity(world, path, entityA, i);
          const planeNormalA = _getPlaneNormal(world, entityA);
          const cwA = _effectiveCW(path, i, true);

          const posB = getEntityWorldPosition(world, entityB);
          const linkTypeB = path.linkTypes[i + 1];
          const isAttachmentB = _isAttachment(linkTypeB);
          const isRollingB = _isRolling(linkTypeB);
          const radiusB = _effectivePathRadiusForEntity(world, path, entityB, i + 1);
          const planeNormalB = _getPlaneNormal(world, entityB);
          const cwB = path.cw[i + 1];

          const prevPosSplitter = world.getComponent(splitterId, CableLinkComponent).prevCableAttachmentTimePos;
          const planeNormalSplitter = _getPlaneNormal(world, splitterId);
          const cw = rightOfPlane(prevPosSplitter, pA, pB, planeNormalSplitter);

          let newAttachmentPointAForJoint = null;
          let newAttachmentPointBForJoint = null;
          if (isRollingA) {
            const tangentAS = tangentFromSphereToSphere(posA, radiusA, cwA, posSplitter, radiusSplitter, cw, planeNormalA);
            newAttachmentPointAForJoint = tangentAS.a_sphere;
            newAttachmentPointBForJoint = tangentAS.b_sphere;
          } else if (isAttachmentA) {
            const tangentAS = tangentFromPointToSphere(pA, posSplitter, radiusSplitter, planeNormalSplitter, cw);
            newAttachmentPointAForJoint = tangentAS.a_attach;
            newAttachmentPointBForJoint = tangentAS.a_sphere;
          } else {
            console.warn(`Splitting cable joint coming from link type ${linkTypeA} is not supported.`);
            continue;
          }

          let attachmentPointAForNewJoint = null;
          let attachmentPointBForNewJoint = null;
          if (isRollingB) {
            const tangentSB = tangentFromSphereToSphere(posSplitter, radiusSplitter, cw, posB, radiusB, cwB, planeNormalB);
            attachmentPointAForNewJoint = tangentSB.a_sphere;
            attachmentPointBForNewJoint = tangentSB.b_sphere;
          } else if (isAttachmentB) {
            const tangentSB = tangentFromSphereToPoint(pB, posSplitter, radiusSplitter, planeNormalSplitter, cw);
            attachmentPointAForNewJoint = tangentSB.a_sphere;
            attachmentPointBForNewJoint = tangentSB.a_attach;
          } else {
            console.warn(`Splitting cable joint attached to ${linkTypeB} is not supported.`);
          }

          let sA = 0.0;
          let sB = 0.0;
          if (isRollingA) {
            sA = signedArcLengthOnWheel(pA, newAttachmentPointAForJoint, posA, radiusA, cwA, planeNormalA);
          }
          if (isRollingB) {
            sB = signedArcLengthOnWheel(pB, attachmentPointBForNewJoint, posB, radiusB, cwB, planeNormalB);
          }

          const s = signedArcLengthOnWheel(
              newAttachmentPointBForJoint,
              attachmentPointAForNewJoint,
              posSplitter,
              radiusSplitter,
              cw,
              planeNormalSplitter
          );
          if (s <= 0.0) {
              _recordCableEventTrace(world, step, {
                type: 'split-abort',
                reason: 'nonpositive-wrap',
                pathId,
                jointId,
                splitterId,
                splitIndex: i,
                s
              });
              console.warn(`Nothing wraps around splitter. Aborting split.`);
              continue;
          }
          if ((s + 1e-9) >= 2.0*Math.PI * radiusSplitter) {
              _recordCableEventTrace(world, step, {
                type: 'split-abort',
                reason: 'full-wrap',
                pathId,
                jointId,
                splitterId,
                splitIndex: i,
                s,
                radiusSplitter
              });
              console.warn(`Split resulted a full wrap around splitter. Aborting split.`);
              continue;
          }

          const dAS = newAttachmentPointAForJoint.clone().subtract(newAttachmentPointBForJoint).length();
          const dSB = attachmentPointAForNewJoint.clone().subtract(attachmentPointBForNewJoint).length();
          const originalRestLength = joint.restLength + sB - sA;
          const totalDist = dAS + dSB;
          const availableRestLength = originalRestLength - s;
          const storedA_before = path.stored[i] ?? 0.0;
          const storedB_before = path.stored[i + 1] ?? 0.0;
          const restBefore = joint.restLength;
          const pathJointCountBefore = path.jointEntities.length;

          let newRestLengthAS = 0;
          let newRestLengthSB = 0;
          if (totalDist > EPSILON) {
              if (availableRestLength < EPSILON) {
                  _recordCableEventTrace(world, step, {
                    type: 'split-abort',
                    reason: 'insufficient-rest',
                    pathId,
                    jointId,
                    splitterId,
                    splitIndex: i,
                    availableRestLength,
                    originalRestLength,
                    s,
                    sA,
                    sB
                  });
                  console.warn(`Split resulted in < ${EPSILON.toFixed(4)} available rest length (${availableRestLength.toFixed(4)}). Aborting split.`);
                  continue;
              }
              newRestLengthAS = availableRestLength * dAS / totalDist;
              newRestLengthSB = availableRestLength * dSB / totalDist;
          } else {
              _recordCableEventTrace(world, step, {
                type: 'split-abort',
                reason: 'degenerate-distances',
                pathId,
                jointId,
                splitterId,
                splitIndex: i,
                totalDist,
                dAS,
                dSB
              });
              console.warn("Split occurred with near-zero distance between new segments:", totalDist);
          }
          path.stored[i + 1] -= sB;
          joint.restLength += sB;
          path.jointEntities.splice(i + 1, 0, newJointId);
          path.cw.splice(i + 1, 0, cw);
          path.linkTypes.splice(i + 1, 0, 'rolling');
          joint.attachmentPointA_world.set(newAttachmentPointAForJoint);
          path.stored[i] += sA;
          joint.restLength -= sA;
          joint.entityB = splitterId;
          path.stored.splice(i + 1, 0, s);

          joint.restLength = newRestLengthAS;
          joint.attachmentPointB_world.set(newAttachmentPointBForJoint);

          world.addComponent(newJointId, new CableJointComponent(
              splitterId, entityB, newRestLengthSB, attachmentPointAForNewJoint, attachmentPointBForNewJoint));

          _recordCableEventTrace(world, step, {
            type: 'split',
            pathId,
            splitIndex: i,
            originalJointId: jointId,
            newJointId,
            splitterId,
            pathJointCountBefore,
            pathJointCountAfter: pathJointCountBefore + 1,
            originalRestLength,
            availableRestLength,
            restBefore,
            newRestLengthAS,
            newRestLengthSB,
            minNewRestLength: Math.min(newRestLengthAS, newRestLengthSB),
            storedA_before,
            storedB_before,
            storedA_after: path.stored[i] ?? 0.0,
            storedSplitter_after: path.stored[i + 1] ?? 0.0,
            storedB_after: path.stored[i + 2] ?? 0.0,
            sA,
            sB,
            s,
            dAS,
            dSB,
            totalDist,
            cw,
            cwA,
            cwB,
            tinyRestProduced:
              (Number.isFinite(newRestLengthAS) && newRestLengthAS < MIN_JOINT_REST_LENGTH) ||
              (Number.isFinite(newRestLengthSB) && newRestLengthSB < MIN_JOINT_REST_LENGTH)
          });
        }
      }
    }
  }
}

export function _updateHybridLinkStates(world, traceStep = null) {
  const pathEntities = world.query([CablePathComponent]);
  const step = Number.isFinite(traceStep)
    ? Math.floor(traceStep)
    : Math.floor(_readFiniteResource(world, 'cableHybridTransitionStep', 0));

  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    for (const i of [0, path.linkTypes.length - 1]) {
      if (path.linkTypes[i] === 'hybrid') {
        const transitionArcThreshold = _hybridTransitionArcThreshold(path);
        if (path.stored[i] < -transitionArcThreshold) {
          const endpointJointId = (i === 0 ? path.jointEntities[i] : path.jointEntities[i - 1]);
          const oldLinkType = path.linkTypes[i];
          const oldCw = path.cw[i];
          const oldStored = path.stored[i] ?? 0.0;
          path.linkTypes[i] = 'hybrid-attachment';
          const joint = (
            i === 0
              ? world.getComponent(path.jointEntities[i], CableJointComponent)
              : world.getComponent(path.jointEntities[i - 1], CableJointComponent)
          );
          const linkEntity = (i === 0 ? joint.entityA : joint.entityB);
          const pos = getEntityWorldPosition(world, linkEntity);
          const radius = _effectiveRollingRadius(
            world,
            path,
            i,
            world.getComponent(linkEntity, RadiusComponent)?.radius
          ).radius;
          const planeNormal = _getPlaneNormal(world, linkEntity);
          const oldRestLength = joint.restLength;
          const attachmentBefore = i === 0
            ? joint.attachmentPointA_world.clone()
            : joint.attachmentPointB_world.clone();
          const neighborAttachmentPoint = i === 0
            ? joint.attachmentPointB_world
            : joint.attachmentPointA_world;
          joint.restLength += oldStored;
          path.stored[i] = 0;
          const rotationApplied = Boolean(pos && Number.isFinite(radius) && radius > EPSILON);
          if (pos && Number.isFinite(radius) && radius > EPSILON) {
            const rotAng = -oldStored / radius;
            if (i === 0) {
              const rotated = _rotateAroundAxis(joint.attachmentPointA_world, pos, planeNormal, rotAng, path.cw[i]);
              joint.attachmentPointA_world.set(rotated);
            } else if (i === path.linkTypes.length - 1) {
              const rotated = _rotateAroundAxis(joint.attachmentPointB_world, pos, planeNormal, rotAng, path.cw[i]);
              joint.attachmentPointB_world.set(rotated);
            }
          }

          const attachmentAfter = i === 0
            ? joint.attachmentPointA_world.clone()
            : joint.attachmentPointB_world.clone();
          _recordHybridTransitionTrace(world, step, {
            transition: 'hybrid->hybrid-attachment',
            pathId,
            endpointIndex: i,
            jointId: endpointJointId,
            linkEntityId: linkEntity,
            neighborEntityId: i === 0 ? joint.entityB : joint.entityA,
            oldLinkType,
            newLinkType: path.linkTypes[i],
            oldCW: oldCw,
            newCW: path.cw[i],
            oldStored,
            newStored: path.stored[i] ?? 0.0,
            restLengthDelta: joint.restLength - oldRestLength,
            oldRestLength,
            newRestLength: joint.restLength,
            effectiveRadius: radius,
            rotationApplied,
            attachmentBefore: _vecSnapshot(attachmentBefore),
            attachmentAfter: _vecSnapshot(attachmentAfter),
            neighborAttachment: _vecSnapshot(neighborAttachmentPoint)
          });
          _recordCableEventTrace(world, step, {
            type: 'hybrid-transition',
            direction: 'hybrid->hybrid-attachment',
            pathId,
            endpointIndex: i,
            jointId: endpointJointId,
            linkEntityId: linkEntity,
            neighborEntityId: i === 0 ? joint.entityB : joint.entityA,
            oldStored,
            newStored: path.stored[i] ?? 0.0,
            oldRestLength,
            newRestLength: joint.restLength,
            restLengthDelta: joint.restLength - oldRestLength,
            oldCW: oldCw,
            newCW: path.cw[i],
            effectiveRadius: radius,
            transitionArcThreshold
          });
        } else if (path.stored[i] < 0.0) {
          const endpointJointId = (i === 0 ? path.jointEntities[i] : path.jointEntities[i - 1]);
          const joint = world.getComponent(endpointJointId, CableJointComponent);
          _recordCableEventTrace(world, step, {
            type: 'hybrid-transition-suppressed',
            direction: 'hybrid->hybrid-attachment',
            pathId,
            endpointIndex: i,
            jointId: endpointJointId,
            linkEntityId: i === 0 ? joint?.entityA : joint?.entityB,
            neighborEntityId: i === 0 ? joint?.entityB : joint?.entityA,
            stored: path.stored[i],
            threshold: transitionArcThreshold,
            reason: 'negative-stored-within-hysteresis'
          });
        }
      } else if (path.linkTypes[i] === 'hybrid-attachment') {
        let jointId, joint, entityId, attachmentPoint, neighborId, neighborAttachmentPoint;
        if (i === 0) {
          jointId = path.jointEntities[0];
          joint = world.getComponent(jointId, CableJointComponent);
          entityId = joint.entityA;
          attachmentPoint = joint.attachmentPointA_world;
          neighborId = joint.entityB;
          neighborAttachmentPoint = joint.attachmentPointB_world;
        } else if (i === path.linkTypes.length - 1) {
          jointId = path.jointEntities[path.jointEntities.length - 1];
          joint = world.getComponent(jointId, CableJointComponent);
          entityId = joint.entityB;
          attachmentPoint = joint.attachmentPointB_world;
          neighborId = joint.entityA;
          neighborAttachmentPoint = joint.attachmentPointA_world;
        }

        const C = getEntityWorldPosition(world, entityId);
        const R = _effectiveRollingRadius(
          world,
          path,
          i,
          world.getComponent(entityId, RadiusComponent)?.radius
        ).radius;
        const lastEndpointIndex = path.linkTypes.length - 1;
        const neighborLinkIndex = i === 0 ? 1 : Math.max(0, lastEndpointIndex - 1);
        const neighborPos = getEntityWorldPosition(world, neighborId);
        const neighborRadius = _effectiveRollingRadius(
          world,
          path,
          neighborLinkIndex,
          world.getComponent(neighborId, RadiusComponent)?.radius
        ).radius;
        const attachmentDistance = attachmentPoint?.distanceTo?.(neighborAttachmentPoint) ?? Infinity;
        const centerDistance = (
          C &&
          neighborPos &&
          Number.isFinite(C.x) &&
          Number.isFinite(C.y) &&
          Number.isFinite(C.z) &&
          Number.isFinite(neighborPos.x) &&
          Number.isFinite(neighborPos.y) &&
          Number.isFinite(neighborPos.z)
        )
          ? C.distanceTo(neighborPos)
          : Infinity;
        const centerOverlap = (
          Number.isFinite(centerDistance) &&
          Number.isFinite(R) &&
          Number.isFinite(neighborRadius)
        )
          ? (R + neighborRadius - centerDistance)
          : null;
        const sameJointPath = path.jointEntities.length === 1;
        const bothEndpointsHybridLike = (
          _isHybrid(path.linkTypes[0]) &&
          _isHybrid(path.linkTypes[lastEndpointIndex])
        );

        if (!C || !Number.isFinite(R) || R <= EPSILON) {
          _recordCableEventTrace(world, step, {
            type: 'hybrid-rub-check',
            pathId,
            endpointIndex: i,
            jointId,
            linkEntityId: entityId,
            neighborEntityId: neighborId,
            sameJointPath,
            bothEndpointsHybridLike,
            centerDistance,
            centerOverlap,
            attachmentDistance,
            reason: 'invalid-center-or-radius'
          });
          continue;
        }

        const planeNormal = _getPlaneNormal(world, entityId);
        const degenerateThreshold = Math.max(1e-6, 2.0 * (path.cableHalfWidth ?? 0.0) + 1e-6);
        if (attachmentDistance <= degenerateThreshold) {
          _recordCableEventTrace(world, step, {
            type: 'hybrid-rub-check',
            pathId,
            endpointIndex: i,
            jointId,
            linkEntityId: entityId,
            neighborEntityId: neighborId,
            sameJointPath,
            bothEndpointsHybridLike,
            centerDistance,
            centerOverlap,
            attachmentDistance,
            degenerateThreshold,
            reason: 'attachment-degenerate-skip'
          });
          continue;
        }

        const neighborAttachmentDistSq = neighborAttachmentPoint.distanceToSq(C);
        const neighborAttachmentInsideCircle = neighborAttachmentDistSq <= (R * R + 1e-9);
        const tanCW = tangentFromSphereToPoint(neighborAttachmentPoint, C, R, planeNormal, true).a_sphere;
        const tanCCW = tangentFromSphereToPoint(neighborAttachmentPoint, C, R, planeNormal, false).a_sphere;
        const tangentFinite = (
          tanCW &&
          tanCCW &&
          Number.isFinite(tanCW.x) &&
          Number.isFinite(tanCW.y) &&
          Number.isFinite(tanCW.z) &&
          Number.isFinite(tanCCW.x) &&
          Number.isFinite(tanCCW.y) &&
          Number.isFinite(tanCCW.z)
        );

        const crossedCW = signedArcLengthOnWheel(attachmentPoint, tanCW, C, R, true, planeNormal);
        const crossedCCW = signedArcLengthOnWheel(attachmentPoint, tanCCW, C, R, false, planeNormal);
        const distSqCW = attachmentPoint.distanceToSq(tanCW);
        const distSqCCW = attachmentPoint.distanceToSq(tanCCW);

        const transitionArcThreshold = _hybridTransitionArcThreshold(path);
        let newCW = null;
        let crossingTangent = null;
        let candidateStored = null;
        let noTransitionReason = 'no-transition';
        if (crossedCCW > 0.0 && distSqCCW < distSqCW) {
          newCW = true;
          crossingTangent = tanCCW;
          candidateStored = crossedCCW;
        } else if (crossedCW > 0.0 && distSqCW < distSqCCW) {
          newCW = false;
          crossingTangent = tanCW;
          candidateStored = crossedCW;
        }
        if (newCW !== null && !(candidateStored > transitionArcThreshold)) {
          noTransitionReason = 'candidate-below-hysteresis';
          newCW = null;
          crossingTangent = null;
          candidateStored = null;
        }

        if (newCW !== null) {
          const oldStored = path.stored[i] ?? 0.0;
          const oldLinkType = path.linkTypes[i];
          const oldCw = path.cw[i];
          const newStored = candidateStored ?? oldStored;
          const oldRestLength = joint.restLength;
          const attachmentBefore = attachmentPoint.clone();
          path.linkTypes[i] = 'hybrid';
          path.cw[i] = newCW;
          path.stored[i] = newStored;
          joint.restLength -= (newStored - oldStored);
          attachmentPoint.set(crossingTangent);
          _ensureHybridKnotAngleComponentForEndpoint(world, path, i, pathId, {
            attachmentPoint,
            createIfMissing: true
          });
          _recordHybridTransitionTrace(world, step, {
            transition: 'hybrid-attachment->hybrid',
            pathId,
            endpointIndex: i,
            jointId,
            linkEntityId: entityId,
            neighborEntityId: neighborId,
            oldLinkType,
            newLinkType: path.linkTypes[i],
            oldCW: oldCw,
            newCW: path.cw[i],
            oldStored,
            newStored: path.stored[i] ?? 0.0,
            restLengthDelta: joint.restLength - oldRestLength,
            oldRestLength,
            newRestLength: joint.restLength,
            effectiveRadius: R,
            crossedCW,
            crossedCCW,
            distSqCW,
            distSqCCW,
            candidateStored,
            selectedTangent: _vecSnapshot(crossingTangent),
            attachmentBefore: _vecSnapshot(attachmentBefore),
            attachmentAfter: _vecSnapshot(attachmentPoint),
            neighborAttachment: _vecSnapshot(neighborAttachmentPoint)
          });
          _recordCableEventTrace(world, step, {
            type: 'hybrid-transition',
            direction: 'hybrid-attachment->hybrid',
            pathId,
            endpointIndex: i,
            jointId,
            linkEntityId: entityId,
            neighborEntityId: neighborId,
            oldStored,
            newStored: path.stored[i] ?? 0.0,
            oldRestLength,
            newRestLength: joint.restLength,
            restLengthDelta: joint.restLength - oldRestLength,
            oldCW: oldCw,
            newCW: path.cw[i],
            effectiveRadius: R,
            crossedCW,
            crossedCCW,
            transitionArcThreshold
          });
          _recordCableEventTrace(world, step, {
            type: 'hybrid-rub-check',
            pathId,
            endpointIndex: i,
            jointId,
            linkEntityId: entityId,
            neighborEntityId: neighborId,
            sameJointPath,
            bothEndpointsHybridLike,
            centerDistance,
            centerOverlap,
            attachmentDistance,
            degenerateThreshold,
            neighborAttachmentInsideCircle,
            tangentFinite,
            crossedCW,
            crossedCCW,
            distSqCW,
            distSqCCW,
            reason: 'transition'
          });
        } else {
          _recordCableEventTrace(world, step, {
            type: 'hybrid-rub-check',
            pathId,
            endpointIndex: i,
            jointId,
            linkEntityId: entityId,
            neighborEntityId: neighborId,
            sameJointPath,
            bothEndpointsHybridLike,
            centerDistance,
            centerOverlap,
            attachmentDistance,
            degenerateThreshold,
            neighborAttachmentInsideCircle,
            tangentFinite,
            crossedCW,
            crossedCCW,
            distSqCW,
            distSqCCW,
            transitionArcThreshold,
            reason: noTransitionReason
          });
        }
      }
    }

    for (let linkIndex = 1; linkIndex < path.linkTypes.length - 1; linkIndex++) {
      if (!_isRolling(path.linkTypes[linkIndex])) {
        continue;
      }
      const leftJointId = path.jointEntities[linkIndex - 1];
      const rightJointId = path.jointEntities[linkIndex];
      const leftJoint = world.getComponent(leftJointId, CableJointComponent);
      const rightJoint = world.getComponent(rightJointId, CableJointComponent);
      if (!leftJoint || !rightJoint || leftJoint.entityB !== rightJoint.entityA) {
        continue;
      }
      const bodyId = leftJoint.entityB;
      const center = getEntityWorldPosition(world, bodyId);
      const radius = _effectiveRollingRadius(
        world,
        path,
        linkIndex,
        world.getComponent(bodyId, RadiusComponent)?.radius
      ).radius;
      if (!center || !Number.isFinite(radius) || radius <= EPSILON) {
        continue;
      }

      const cw = path.cw[linkIndex];
      const planeNormal = _getPlaneNormal(world, bodyId);
      const arc = signedArcLengthOnWheel(
        leftJoint.attachmentPointB_world,
        rightJoint.attachmentPointA_world,
        center,
        radius,
        cw,
        planeNormal,
        true
      );
      const altArc = signedArcLengthOnWheel(
        leftJoint.attachmentPointB_world,
        rightJoint.attachmentPointA_world,
        center,
        radius,
        !cw,
        planeNormal,
        true
      );
      const stored = path.stored[linkIndex] ?? 0.0;
      if (arc <= 1e-6 && altArc > arc + 1e-4 && stored > 1e-6) {
        _debugCable(
          world,
          `rolling-arc-mismatch path=${pathId} link=${linkIndex} body=${bodyId} ` +
          `cw=${cw} arc=${arc.toFixed(6)} altArc=${altArc.toFixed(6)} stored=${stored.toFixed(6)} ` +
          `leftJoint=${leftJointId} rightJoint=${rightJointId}`
        );
      }
    }
  }
}

export class CableAttachmentUpdateSystem {
  runInPause = false;

  constructor(mergeAndSplitFeature = true) {
    this.mergeAndSplitFeature = mergeAndSplitFeature;
  }

  update(world, _dt_unused) {
    const prevHybridStep = Math.floor(_readFiniteResource(world, 'cableHybridTransitionStep', 0));
    const nextHybridStep = prevHybridStep + 1;
    world.setResource('cableHybridTransitionStep', nextHybridStep);
    _recordCableStepSummary(world, nextHybridStep, 'begin');
    _clearDebugPoints(world);

    if (_featureFlag(world, 'layeringAttachmentUpdatePoints', true)) {
      _updateAttachmentPoints(world);
    }
    _recordCableStepSummary(world, nextHybridStep, 'afterAttachment');

    if (_featureFlag(world, 'layeringMergeJoints', this.mergeAndSplitFeature)) {
      _mergeJoints(world);
    }
    _recordCableStepSummary(world, nextHybridStep, 'afterMerge');

    if (_featureFlag(world, 'layeringSplitJoints', this.mergeAndSplitFeature)) {
      _splitJoints(world);
    }
    _recordCableStepSummary(world, nextHybridStep, 'afterSplit');

    if (_featureFlag(world, 'layeringHybridLinkStates', true)) {
      _updateHybridLinkStates(world, nextHybridStep);
    }
    _recordCableStepSummary(world, nextHybridStep, 'afterHybrid');
  }
}

export class PBDCableConstraintSolver {
  runInPause = false;

  update(world, _dt_unused) {
    const pathEntities = world.query([CablePathComponent]);
    const dt = world.getResource('dt');

    const jointLocals = new Map();
    const computeLocal = (entityId, worldPoint) => computeLocalAttachment(world, entityId, worldPoint);
    const getPrevPosition = (entityId) => world.getComponent(entityId, PrevFinalPosComponent)?.pos ?? null;
    const getPrevOrientation = (entityId) => world.getComponent(entityId, PrevFinalOrientationComponent)?.quaternion ?? null;
    const getCurrentOrientation = (entityId) => world.getComponent(entityId, OrientationComponent)?.quaternion ?? null;
    const getPrevLocalOrientation = (entityId) => {
      const member = world.getComponent(entityId, RigidBodyMemberComponent);
      if (!member) {
        return getPrevOrientation(entityId);
      }

      const prevBodyOrientation = getPrevOrientation(member.bodyEntity);
      const prevMemberOrientation = getPrevOrientation(entityId);
      if (!prevBodyOrientation || !prevMemberOrientation) {
        return null;
      }

      return new Quaternion()
        .multiplyQuaternions(
          prevBodyOrientation.clone().conjugate().normalize(),
          prevMemberOrientation,
        )
        .normalize();
    };
    const getCurrentLocalOrientation = (entityId) => {
      const member = world.getComponent(entityId, RigidBodyMemberComponent);
      if (member?.localOrientation) {
        return member.localOrientation;
      }
      return getCurrentOrientation(entityId);
    };
    const buildExternalMemberSpinSolveInfo = (entityId, pointWorld, gradPos) => {
      const linkComp = world.getComponent(entityId, CableLinkComponent);
      if (!linkComp?.cablePlaneNormalLocal || !pointWorld || !gradPos) {
        return null;
      }
      const orientationComp = world.getComponent(entityId, OrientationComponent);
      const moiComp = world.getComponent(entityId, MomentOfInertiaComponent);
      const centerWorld = getEntityWorldPosition(world, entityId);
      if (!orientationComp?.quaternion || !centerWorld || !(moiComp?.invInertia > EPSILON)) {
        return null;
      }

      const axisWorld = orientationComp.quaternion
        .transformVector(linkComp.cablePlaneNormalLocal.clone());
      if (axisWorld.lengthSq() <= EPSILON) {
        return null;
      }
      axisWorld.normalize();
      return {
        entityId,
        centerWorld,
        invInertia: _effectiveMotorizedSpinInvInertia(world, entityId, moiComp.invInertia, dt),
        axisWorld,
        pointWorld: pointWorld.clone(),
        gradPos: gradPos.clone(),
      };
    };
    const getExternalMemberSpinSolveInfo = (
      path,
      jointIndex,
      side,
      originalEntityId,
      mappedEndpoint,
      pointWorld,
      gradPos
    ) => {
      if (!mappedEndpoint) {
        return null;
      }

      const directLinkIndex = side === 'A' ? jointIndex : jointIndex + 1;
      if (_linkAllowsCableSpin(world, path, directLinkIndex, originalEntityId)) {
        const directInfo = buildExternalMemberSpinSolveInfo(originalEntityId, pointWorld, gradPos);
        if (directInfo) {
          return directInfo;
        }
      }

      if (mappedEndpoint.internalToBody || mappedEndpoint.entityId === originalEntityId) {
        return null;
      }

      if (!path || !Array.isArray(path.linkTypes) || !Array.isArray(path.jointEntities) || path.jointEntities.length < 2) {
        return null;
      }

      const pinholeLinkIndex = side === 'A' ? jointIndex : jointIndex + 1;
      if (path.linkTypes[pinholeLinkIndex] !== 'pinhole') {
        return null;
      }

      let internalJointIndex = null;
      let spinOnJointSide = null;
      if (
        side === 'A' &&
        pinholeLinkIndex > 0 &&
        _isSpinBackdrivableThroughPinhole(path.linkTypes[pinholeLinkIndex - 1])
      ) {
        internalJointIndex = jointIndex - 1;
        spinOnJointSide = 'A';
      } else if (
        side === 'B' &&
        pinholeLinkIndex < path.linkTypes.length - 1 &&
        _isSpinBackdrivableThroughPinhole(path.linkTypes[pinholeLinkIndex + 1])
      ) {
        internalJointIndex = jointIndex + 1;
        spinOnJointSide = 'B';
      }
      if (!(internalJointIndex >= 0 && internalJointIndex < path.jointEntities.length)) {
        return null;
      }

      const internalJointId = path.jointEntities[internalJointIndex];
      const internalJoint = world.getComponent(internalJointId, CableJointComponent);
      const internalLocals = jointLocals.get(internalJointId);
      if (!internalJoint || !internalLocals) {
        return null;
      }

      const spinEntityId = spinOnJointSide === 'A' ? internalJoint.entityA : internalJoint.entityB;
      const spinPointWorld = spinOnJointSide === 'A'
        ? computeWorldAttachment(world, spinEntityId, internalLocals.localA)
        : computeWorldAttachment(world, spinEntityId, internalLocals.localB);
      if (!spinPointWorld || !pointWorld) {
        return null;
      }

      const coupledGradPos = pointWorld.clone().subtract(spinPointWorld);
      if (coupledGradPos.lengthSq() <= EPSILON) {
        return null;
      }
      coupledGradPos.normalize();

      const info = buildExternalMemberSpinSolveInfo(spinEntityId, spinPointWorld, coupledGradPos);
      if (info) {
        info.transferredJointId = internalJointId;
        info.transferredJointIndex = internalJointIndex;
      }
      return info;
    };
    const recordTransferredForceMagnitude = (jointId, forceMagnitude) => {
      if (!(Number.isFinite(forceMagnitude) && forceMagnitude > 0.0)) {
        return;
      }
      const transferredJoint = world.getComponent(jointId, CableJointComponent);
      if (!transferredJoint) {
        return;
      }
      const previous = transferredJoint.transferredConstraintForceMagnitude ?? 0.0;
      transferredJoint.transferredConstraintForceMagnitude = Math.max(previous, forceMagnitude);
    };
    const recordPinholeNeighborForceTransfers = (path, jointIndex, forceMagnitude) => {
      if (!path || !Array.isArray(path.linkTypes) || !Array.isArray(path.jointEntities)) {
        return;
      }
      if (path.linkTypes[jointIndex] === 'pinhole' && jointIndex > 0) {
        recordTransferredForceMagnitude(path.jointEntities[jointIndex - 1], forceMagnitude);
      }
      if (
        path.linkTypes[jointIndex + 1] === 'pinhole' &&
        jointIndex + 1 < path.jointEntities.length
      ) {
        recordTransferredForceMagnitude(path.jointEntities[jointIndex + 1], forceMagnitude);
      }
    };

    const applyConstraint = (
      path,
      jointIndex,
      joint,
      entityA,
      entityB,
      pointA_world,
      pointB_world,
      gradPosA,
      gradPosB,
      constraintError,
      compliance,
      solveIteration
    ) => {
      if (constraintError <= EPSILON) {
        return;
      }
      const mappedA = resolveRigidBodySolverEndpoint(world, entityA, entityB, pointA_world);
      const mappedB = resolveRigidBodySolverEndpoint(world, entityB, entityA, pointB_world);

      const solverEntityA = mappedA.entityId;
      const solverEntityB = mappedB.entityId;
      const solverPointA = pointA_world;
      const solverPointB = pointB_world;
      const memberSpinA = getExternalMemberSpinSolveInfo(
        path,
        jointIndex,
        'A',
        entityA,
        mappedA,
        pointA_world,
        gradPosA, // gradPosA is a unit vector pointing from A to B
      );
      const memberSpinB = getExternalMemberSpinSolveInfo(
        path,
        jointIndex,
        'B',
        entityB,
        mappedB,
        pointB_world,
        gradPosB, // gradPosB is a unit vector pointing from B to A
      );
      const axisOnlySolverA = _hasAxisOnlyCableSpinDof(world, solverEntityA);
      const axisOnlySolverB = _hasAxisOnlyCableSpinDof(world, solverEntityB);

	    const massAComp = world.getComponent(solverEntityA, MassComponent);
	    const invMassA = (massAComp && massAComp.mass > 0) ? 1.0 / massAComp.mass : 0.0;
	    const moiAComp = world.getComponent(solverEntityA, MomentOfInertiaComponent);
	    const invInertiaA = (!axisOnlySolverA && moiAComp) ? moiAComp.invInertia : 0.0;

	    const massBComp = world.getComponent(solverEntityB, MassComponent);
	    const invMassB = (massBComp && massBComp.mass > 0) ? 1.0 / massBComp.mass : 0.0;
	    const moiBComp = world.getComponent(solverEntityB, MomentOfInertiaComponent);
	    const invInertiaB = (!axisOnlySolverB && moiBComp) ? moiBComp.invInertia : 0.0;

      const memberInvInertiaA = memberSpinA?.invInertia ?? 0.0;
      const memberInvInertiaB = memberSpinB?.invInertia ?? 0.0;

	    if (
	      invMassA + invMassB + invInertiaA + invInertiaB + memberInvInertiaA + memberInvInertiaB
	      <= EPSILON
	    ) {
	      return;
	    }

      const posAComp = world.getComponent(solverEntityA, PositionComponent);
      const posBComp = world.getComponent(solverEntityB, PositionComponent);
      if (!posAComp || !posBComp) {
        return;
      }

      const rA = new Vector3().subtractVectors(solverPointA, posAComp.pos);
      const rB = new Vector3().subtractVectors(solverPointB, posBComp.pos);
      const gradAngA = rA.cross(gradPosA);
      const gradAngB = rB.cross(gradPosB);

      const prevPosA = getPrevPosition(solverEntityA);
      const prevPosB = getPrevPosition(solverEntityB);
      const prevQuatA = getPrevOrientation(solverEntityA);
      const prevQuatB = getPrevOrientation(solverEntityB);
      const currQuatA = getCurrentOrientation(solverEntityA);
      const currQuatB = getCurrentOrientation(solverEntityB);

      const translationalDispA = prevPosA
        ? gradPosA.dot(posAComp.pos.clone().subtract(prevPosA))
        : 0.0;
      const translationalDispB = prevPosB
        ? gradPosB.dot(posBComp.pos.clone().subtract(prevPosB))
        : 0.0;
      const rotationalDispA = gradAngA.dot(_rotationVectorBetween(prevQuatA, currQuatA));
      const rotationalDispB = gradAngB.dot(_rotationVectorBetween(prevQuatB, currQuatB));

      const gradAngMemberA = memberSpinA
        ? new Vector3().subtractVectors(memberSpinA.pointWorld, memberSpinA.centerWorld).cross(memberSpinA.gradPos)
        : null;
      const gradAngMemberB = memberSpinB
        ? new Vector3().subtractVectors(memberSpinB.pointWorld, memberSpinB.centerWorld).cross(memberSpinB.gradPos)
        : null;
      const gradSpinA = (memberSpinA?.axisWorld && gradAngMemberA)
        ? gradAngMemberA.dot(memberSpinA.axisWorld)
        : 0.0;
      const gradSpinB = (memberSpinB?.axisWorld && gradAngMemberB)
        ? gradAngMemberB.dot(memberSpinB.axisWorld)
        : 0.0;

      const linkCompMemberA = memberSpinA
        ? world.getComponent(memberSpinA.entityId, CableLinkComponent)
        : null;
      const linkCompMemberB = memberSpinB
        ? world.getComponent(memberSpinB.entityId, CableLinkComponent)
        : null;
      const spinAngleDispA = (memberSpinA && linkCompMemberA?.cablePlaneNormalLocal)
        ? _deltaAngleAroundAxis(
          getPrevLocalOrientation(memberSpinA.entityId),
          getCurrentLocalOrientation(memberSpinA.entityId),
          linkCompMemberA.cablePlaneNormalLocal,
          true,
        )
        : 0.0;
      const spinAngleDispB = (memberSpinB && linkCompMemberB?.cablePlaneNormalLocal)
        ? _deltaAngleAroundAxis(
          getPrevLocalOrientation(memberSpinB.entityId),
          getCurrentLocalOrientation(memberSpinB.entityId),
          linkCompMemberB.cablePlaneNormalLocal,
          true,
        )
        : 0.0;
      const spinDispA = gradSpinA * spinAngleDispA;
      const spinDispB = gradSpinB * spinAngleDispB;

      let mechDenom = 0.0;
      mechDenom += invMassA * gradPosA.lengthSq();
      mechDenom += invInertiaA * gradAngA.lengthSq();
      if (memberInvInertiaA > 0.0 && Math.abs(gradSpinA) > EPSILON) {
        mechDenom += memberInvInertiaA * gradSpinA * gradSpinA;
      }
      mechDenom += invMassB * gradPosB.lengthSq();
      mechDenom += invInertiaB * gradAngB.lengthSq();
      if (memberInvInertiaB > 0.0 && Math.abs(gradSpinB) > EPSILON) {
        mechDenom += memberInvInertiaB * gradSpinB * gradSpinB;
      }

      const alphaTilde = (Number.isFinite(dt) && dt > EPSILON)
        ? (compliance ?? 0.0) / (dt * dt)
        : 0.0;
      const gamma = (Number.isFinite(dt) && dt > EPSILON)
        ? Math.max(0.0, (compliance ?? 0.0) * (path?.damping ?? 0.0) / dt)
        : 0.0;
      const jDx = translationalDispA + rotationalDispA + spinDispA + translationalDispB + rotationalDispB + spinDispB;
      const denom = ((1.0 + gamma) * mechDenom) + alphaTilde;

      if (denom <= EPSILON) {
        return;
      }
      //console.log(`gamma: ${gamma}`);

      const lambda = (-constraintError - (gamma * jDx)) / denom;
      const invDtSq = (Number.isFinite(dt) && dt > EPSILON) ? 1.0 / (dt * dt) : 0.0;
      if (!joint.constraintForce) {
        joint.constraintForce = new Vector3(0, 0, 0);
      }
      if (solveIteration === 0) {
        joint.constraintLambda = lambda;
        joint.constraintForce.set(gradPosA);
        joint.constraintForce.scale(lambda * invDtSq);
        joint.constraintForceMagnitude = Math.abs(lambda) * invDtSq;
        const transferredForceMagnitude = Math.abs(lambda) * invDtSq;
        recordPinholeNeighborForceTransfers(path, jointIndex, transferredForceMagnitude);
        if (memberSpinA?.transferredJointId !== undefined) {
          recordTransferredForceMagnitude(
            memberSpinA.transferredJointId,
            transferredForceMagnitude,
          );
        }
        if (
          memberSpinB?.transferredJointId !== undefined &&
          memberSpinB.transferredJointId !== memberSpinA?.transferredJointId
        ) {
          recordTransferredForceMagnitude(
            memberSpinB.transferredJointId,
            transferredForceMagnitude,
          );
        }
      }

      if (invMassA > 0.0) {
        const deltaPosA = gradPosA.clone().scale(-invMassA * lambda);
        posAComp.pos.add(deltaPosA);
      }
	    if (invInertiaA > 0.0) {
	      const deltaAngA = gradAngA.clone().scale(-invInertiaA * lambda);
	      const orientationAComp = world.getComponent(solverEntityA, OrientationComponent);
	      if (orientationAComp) {
          const angle = deltaAngA.length();
          if (angle > EPSILON) {
            const axis = deltaAngA.clone().scale(1.0 / angle);
            const dq = new Quaternion().setFromAxisAngle(axis, angle);
            orientationAComp.quaternion.multiplyQuaternions(dq, orientationAComp.quaternion).normalize();
            updateRigidBodyMemberLocalOrientation(world, solverEntityA);
	        }
	      }
	    }
      if (memberInvInertiaA > 0.0 && memberSpinA?.axisWorld && Math.abs(gradSpinA) > EPSILON) {
        const deltaAngleMemberA = -memberInvInertiaA * lambda * gradSpinA;
        const memberOrientationAComp = world.getComponent(memberSpinA.entityId, OrientationComponent);
        if (memberOrientationAComp) {
          if (Math.abs(deltaAngleMemberA) > EPSILON) {
            const dq = new Quaternion().setFromAxisAngle(memberSpinA.axisWorld, deltaAngleMemberA);
            memberOrientationAComp.quaternion.multiplyQuaternions(dq, memberOrientationAComp.quaternion).normalize();
            updateRigidBodyMemberLocalOrientation(world, memberSpinA.entityId);
          }
        }
      }

	    if (invMassB > 0.0) {
	      const deltaPosB = gradPosB.clone().scale(-invMassB * lambda);
	      posBComp.pos.add(deltaPosB);
      }
      if (invInertiaB > 0.0) {
        const deltaAngB = gradAngB.clone().scale(-invInertiaB * lambda);
        const orientationBComp = world.getComponent(solverEntityB, OrientationComponent);
        if (orientationBComp) {
          const angle = deltaAngB.length();
          if (angle > EPSILON) {
            const axis = deltaAngB.clone().scale(1.0 / angle);
            const dq = new Quaternion().setFromAxisAngle(axis, angle);
            orientationBComp.quaternion.multiplyQuaternions(dq, orientationBComp.quaternion).normalize();
            updateRigidBodyMemberLocalOrientation(world, solverEntityB);
	        }
	      }
	    }
      if (memberInvInertiaB > 0.0 && memberSpinB?.axisWorld && Math.abs(gradSpinB) > EPSILON) {
        const deltaAngleMemberB = -memberInvInertiaB * lambda * gradSpinB;
        const memberOrientationBComp = world.getComponent(memberSpinB.entityId, OrientationComponent);
        if (memberOrientationBComp) {
          if (Math.abs(deltaAngleMemberB) > EPSILON) {
            const dq = new Quaternion().setFromAxisAngle(memberSpinB.axisWorld, deltaAngleMemberB);
            memberOrientationBComp.quaternion.multiplyQuaternions(dq, memberOrientationBComp.quaternion).normalize();
            updateRigidBodyMemberLocalOrientation(world, memberSpinB.entityId);
          }
        }
      }
	  };

    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (path.jointEntities.length < 1) continue;
      for (const jointId of path.jointEntities) {
        const joint = world.getComponent(jointId, CableJointComponent);
        joint.constraintLambda = 0.0;
        if (!joint.constraintForce) {
          joint.constraintForce = new Vector3(0, 0, 0);
        } else {
          joint.constraintForce.x = 0.0;
          joint.constraintForce.y = 0.0;
          joint.constraintForce.z = 0.0;
        }
        joint.constraintForceMagnitude = 0.0;
        joint.transferredConstraintForceMagnitude = 0.0;
        jointLocals.set(jointId, {
          localA: computeLocal(joint.entityA, joint.attachmentPointA_world),
          localB: computeLocal(joint.entityB, joint.attachmentPointB_world)
        });
      }
    }
    let maxIterations = 1;
    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      maxIterations = Math.max(maxIterations, path?.solverIterations ?? 1);
    }

    for (let iter = 0; iter < maxIterations; iter++) {
      const isForward = (iter % 2 === 0);

      const startPath = isForward ? 0 : pathEntities.length - 1;
      const endPath = isForward ? pathEntities.length : -1;
      const stepPath = isForward ? 1 : -1;

      for (let p = startPath; p !== endPath; p += stepPath) {
        const pathId = pathEntities[p];
        const path = world.getComponent(pathId, CablePathComponent);
        if (path.jointEntities.length < 1) continue;
        if (iter >= (path.solverIterations ?? 1)) continue;

        const startJoint = isForward ? 0 : path.jointEntities.length - 1;
        const endJoint = isForward ? path.jointEntities.length : -1;
        const stepJoint = isForward ? 1 : -1;

        for (let j = startJoint; j !== endJoint; j += stepJoint) {
          const jointId = path.jointEntities[j];
          const joint = world.getComponent(jointId, CableJointComponent);
          const locals = jointLocals.get(jointId);

          const entityA = joint.entityA;
          const entityB = joint.entityB;

          const pA = computeWorldAttachment(world, entityA, locals.localA);
          const pB = computeWorldAttachment(world, entityB, locals.localB);

          const currentSegmentLength = pA.distanceTo(pB);
          let dir = null;
          if (currentSegmentLength > EPSILON) {
            dir = new Vector3().subtractVectors(pB, pA).scale(1.0 / currentSegmentLength);
          }
          if (!dir || dir.lengthSq() <= EPSILON) {
            continue;
          }

          const constraintError = currentSegmentLength - joint.restLength;
          if (constraintError <= EPSILON) {
            continue;
          }

          const gradPosA = dir.clone();
          const gradPosB = dir.clone().scale(-1.0);
          applyConstraint(
            path,
            j,
            joint,
            entityA,
            entityB,
            pA,
            pB,
            gradPosA,
            gradPosB,
            constraintError,
            path.compliance,
            iter
          );
        }
      }
    }
  }
}
