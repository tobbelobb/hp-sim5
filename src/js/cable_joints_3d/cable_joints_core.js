import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';

import {
  tangentFromPointToSphere,
  tangentFromSphereToPoint,
  signedArcLengthOnWheel,
} from './geometry3.js';

import {
  RadiusComponent,
  MassComponent,
  OrientationComponent,
  RigidBodyMemberComponent,
  HybridKnotAngleComponent,
  MomentOfInertiaComponent,
  MachineTagComponent,
  CoefficientOfFrictionComponent,
  layeringEnabled,
} from './ecs.js';
import {
  computeWorldAttachment,
  getEntityWorldOrientation,
  getEntityWorldPosition,
  updateRigidBodyMemberLocalOrientation,
} from './rigid_bodies.js';
import {
  hasAnyInverseInertia,
} from './inertia_tensor.js';
import {
  SpoolStateComponent,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';

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

// A condensed cable guide with independent, rigidly mounted ports. The hidden
// reeving is represented by one cable-flow coordinate q. visualRoute is
// body-local decoration and never adds physical constraints.
export class PinholeGateComponent {
  constructor(exitEntity, internalLength = 0) {
    this.exitEntity = exitEntity;
    this.internalLength = internalLength;
    // Equivalent translational quantities, summed over the hidden bearings:
    // M = sum(I/r^2), B = sum(b/r^2), Fc = sum(tau_c/r).
    this.effectiveInertia = 1e-6;
    this.damping = 0.0;
    this.coulombForce = 0.0;
    this.radius = 0.0065;
    this.flow = 0.0;
    this.flowVelocity = 0.0;
    this.visualRoute = [];
    this.visible = true;
  }
}

// Visual-only hidden bearing. It follows material transported past the carrier
// spool without adding a rotor DOF or a cable constraint.
export class GateWheelComponent {
  constructor(gateEntity, pathEntity, radius = 0.0065, axisLocal = null, rotationScale = 1.0, initialStored = 0.0) {
    this.gateEntity = gateEntity;
    this.pathEntity = pathEntity;
    this.radius = Number.isFinite(radius) && radius > 0.0 ? radius : 0.0065;
    this.axisLocal = axisLocal ? axisLocal.clone().normalize() : new Vector3(0, 0, 1);
    this.rotationScale = Number.isFinite(rotationScale) ? rotationScale : 1.0;
    this.initialStored = Number.isFinite(initialStored) ? initialStored : 0.0;
  }
}

// The hidden bearings follow material transported past the carrier spool, not
// the small differential gate state q. Positive transport unwinds stored cable.
export function gateWheelVisualAngle(wheel, path) {
  const stored = path?.stored?.[0];
  const initial = Number.isFinite(wheel?.initialStored) ? wheel.initialStored : stored;
  if (!Number.isFinite(stored) || !Number.isFinite(initial) || !(wheel.radius > 0.0)) return 0.0;
  return wheel.rotationScale * (initial - stored) / wheel.radius;
}

export function findGateLinkIndex(path) {
  const index = path.linkTypes.indexOf('pinhole_gate');
  if (index <= 0 || index >= path.linkTypes.length - 1) {
    throw new Error('pinhole_gate must be an interior cable path link');
  }
  return index;
}

export function getGateComponent(world, path, gateLinkIndex = findGateLinkIndex(path)) {
  const incoming = world.getComponent(path.jointEntities[gateLinkIndex - 1], CableJointComponent);
  return world.getComponent(incoming?.entityB, PinholeGateComponent);
}

export function advanceGateFlow(gate, tensionDifference, dt) {
  const inertia = gate.effectiveInertia;
  if (!(Number.isFinite(inertia) && inertia > 0.0)) throw new Error('pinhole_gate effectiveInertia must be positive');
  if (!(Number.isFinite(dt) && dt > 0.0)) return gate.flow;
  const damping = Math.max(0.0, gate.damping);
  const coulombForce = Math.max(0.0, gate.coulombForce);
  const drive = Number.isFinite(tensionDifference) ? tensionDifference : 0.0;
  const velocity = gate.flowVelocity;
  const moving = Math.abs(velocity) > 1e-9;
  // Static friction holds the reeving still until the port tension difference
  // exceeds the summed dry-bearing threshold. Once slipping, resistance
  // opposes the hidden flow, and drag is integrated implicitly.
  if (!moving && Math.abs(drive) <= coulombForce) {
    gate.flowVelocity = 0.0;
    return gate.flow;
  }
  const resistance = moving ? Math.sign(velocity) * coulombForce : Math.sign(drive) * coulombForce;
  let next = (velocity + dt * (drive - resistance) / inertia)
    / (1.0 + dt * damping / inertia);
  // Bearing friction can stop the flow within one step but cannot reverse it.
  if (moving && Math.sign(next) !== Math.sign(velocity)) next = 0.0;
  else if (!moving && Math.sign(next) !== Math.sign(drive)) next = 0.0;
  gate.flow += next * dt;
  gate.flowVelocity = next;
  return gate.flow;
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
      if (linkTypes[i + 1] === 'pinhole_gate') {
        const a = world.getComponent(linkId, RigidBodyMemberComponent)?.bodyEntity;
        const b = world.getComponent(linkId2, RigidBodyMemberComponent)?.bodyEntity;
        if (linkId !== linkId2 && (a === undefined || a !== b)) throw new Error('pinhole_gate ports must share one rigid body');
        const hidden = stored?.[i + 1] ?? 0;
        const separation = getEntityWorldPosition(world, linkId).distanceTo(getEntityWorldPosition(world, linkId2));
        if (!Number.isFinite(hidden) || hidden + 1e-9 < separation) throw new Error('pinhole_gate internal length must cover the port separation');
        world.addComponent(linkId, new PinholeGateComponent(linkId2, hidden));
        continue;
      }
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

    if (this.linkTypes.includes('pinhole_gate')) {
      const dynamic = new Set();
      for (const id of this.jointEntities) for (const entity of [world.getComponent(id, CableJointComponent).entityA, world.getComponent(id, CableJointComponent).entityB]) {
        const body = world.getComponent(entity, RigidBodyMemberComponent)?.bodyEntity ?? entity;
        if ((world.getComponent(body, MassComponent)?.mass ?? 0) > 0 || hasAnyInverseInertia(world.getComponent(body, MomentOfInertiaComponent)) || world.getComponent(entity, SpoolStateComponent)) dynamic.add(body);
      }
      if (dynamic.size > 2) throw new Error('pinhole_gate supports one carrier and one external rotor');
      if (this.linkTypes.slice(1, -1).some(t => t !== 'pinhole' && t !== 'pinhole_gate')) throw new Error('pinhole_gate requires frictionless interior guides');
      for (let i = 1; i < this.linkTypes.length - 1; i++) {
        const id = world.getComponent(this.jointEntities[i - 1], CableJointComponent).entityB;
        if ((world.getComponent(id, CoefficientOfFrictionComponent)?.mu ?? 0) !== 0) throw new Error('pinhole_gate does not support guide friction yet');
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
const DEFAULT_OPEN_LOOP_STEPPER_HOLDING_TORQUE_SCALE = 1.0;

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
  const sinHalf = Math.hypot(qDelta.x, qDelta.y, qDelta.z);
  const angle = 2.0 * Math.atan2(sinHalf, w);
  if (angle <= EPSILON) {
    return new Vector3(0.0, 0.0, 0.0);
  }

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

// Predict endpoint winding using the same layer/ramp model as the cable solver.
export function cableStoredLengthAfterRotation(world, path, linkIndex, entityId, deltaAngle) {
  const stored = path.stored[linkIndex] ?? 0.0;
  const baseRadius = world.getComponent(entityId, RadiusComponent)?.radius ?? 0.0;
  const halfWidth = layeringEnabled(world) ? (path.cableHalfWidth ?? 0.0) : 0.0;
  const endpointSign = linkIndex === 0 ? 1.0 : -1.0;
  if (path.linkTypes[linkIndex] !== 'hybrid') {
    return stored + endpointSign * (path.cw[linkIndex] ? 1.0 : -1.0)
      * deltaAngle * (baseRadius + halfWidth);
  }
  const theta = _storedToThetaSigned(stored, baseRadius, halfWidth, baseRadius * KNOT_SPAN);
  return stored + _hybridStoredDeltaFromRotation(
    theta, endpointSign * deltaAngle, path.cw[linkIndex], baseRadius, halfWidth, baseRadius + halfWidth,
  );
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
