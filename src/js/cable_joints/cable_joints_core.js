import Vector2 from './vector2.js';

import {
  tangentFromPointToCircle,
  tangentFromCircleToPoint,
  tangentFromCircleToCircle,
  signedArcLengthOnWheel,
  lineSegmentCircleIntersection,
  rightOfLine,
} from './geometry.js';

import {
  World,
  PositionComponent,
  PrevFinalPosComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  OrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
  RenderableComponent,
  MachineTagComponent
} from './ecs.js';

import {
  GravitySystem,
  MovementSystem,
} from './commonSystems.js';

export const linecolor1 = '#FFFF00';
const EPSILON = 1e-9;
const MIN_JOINT_REST_LENGTH = 1e-6;
const CABLE_DEBUG_PREFIX = '[CableJointsDebug]';
const CABLE_HYBRID_TRACE_PREFIX = '[CableHybridTrace]';

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
  return { x: v.x, y: v.y };
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
      nonFiniteRestCount++;
      continue;
    }
    if (rest < minRestLength) minRestLength = rest;
    if (rest > maxRestLength) maxRestLength = rest;
    if (rest < MIN_JOINT_REST_LENGTH) tinyRestCount++;
    if (rest < 0.0) negativeRestCount++;
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
        nonFiniteStoredCount++;
        continue;
      }
      if (stored < minStored) minStored = stored;
      if (stored > maxStored) maxStored = stored;
      if (stored < 0.0) negativeStoredCount++;
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

function _resourceBool(world, key, fallback = true) {
  const value = world?.getResource?.(key);
  return typeof value === 'boolean' ? value : fallback;
}

function _layeringEnabled(world) {
  return _resourceBool(world, 'enableLayering', true);
}

function _layeringFlag(world, key, fallback = true) {
  return _layeringEnabled(world) && _resourceBool(world, key, fallback);
}

function _featureFlag(world, key, fallback = true) {
  return _resourceBool(world, key, fallback);
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

function _computeWorldAttachment(world, entityId, localPoint) {
  if (!localPoint) {
    return null;
  }
  const localVec = localPoint.clone();
  if (!world) {
    return localVec;
  }

  const posComp = world.getComponent(entityId, PositionComponent);
  if (!posComp || !posComp.pos) {
    return localVec;
  }

  const orientationComp = world.getComponent(entityId, OrientationComponent);
  const angle = orientationComp?.angle ?? 0.0;
  const cos = Math.cos(angle);
  const sin = Math.sin(angle);

  const rotatedX = localVec.x * cos - localVec.y * sin;
  const rotatedY = localVec.x * sin + localVec.y * cos;

  return new Vector2(posComp.pos.x + rotatedX, posComp.pos.y + rotatedY);
}

function _computeLocalAttachment(worldPoint, centerPoint, angle) {
  if (!worldPoint || !centerPoint || !Number.isFinite(angle)) {
    return null;
  }
  const dx = worldPoint.x - centerPoint.x;
  const dy = worldPoint.y - centerPoint.y;
  const cos = Math.cos(-angle);
  const sin = Math.sin(-angle);
  return new Vector2(dx * cos - dy * sin, dx * sin + dy * cos);
}

export class CableLinkComponent {
  constructor(x = 0, y = 0, angle = 0.0) {
    this.prevCableAttachmentTimePos = new Vector2(x, y);
    this.prevCableAttachmentTimeAngle = angle;
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
  }

  static fromWorld(entityA, entityB, restLength, attachmentPointA_world, attachmentPointB_world) {
    return new CableJointComponent(entityA, entityB, restLength, attachmentPointA_world, attachmentPointB_world);
  }

  static fromLocal(world, entityA, entityB, restLength, attachmentPointA_local, attachmentPointB_local) {
    return new CableJointComponent(
      entityA,
      entityB,
      restLength,
      _computeWorldAttachment(world, entityA, attachmentPointA_local),
      _computeWorldAttachment(world, entityB, attachmentPointB_local)
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
    cableHalfWidth = 0.0
  ) {
    this.totalRestLength = 0.0;
    this.jointEntities = jointEntities; // Ordered list of CableJoint entity IDs
    this.linkTypes = linkTypes; // Ordered. linkTypes.length === jointEntities.length + 1
    this.cw = cw // Ordered. cw.length === linkTypes.length
    this.spring_constant = spring_constant;
    this.compliance = 1.0/spring_constant;
    this.stored = new Array(cw.length).fill(0.0); // Ordered. stored.length === cw.length
    this.cableHalfWidth = Number.isFinite(cableHalfWidth) ? Math.max(0.0, cableHalfWidth) : 0.0;

    for (const jointId of jointEntities) {
      const joint = world.getComponent(jointId, CableJointComponent);
      this.totalRestLength += joint.restLength;
    }
    for (let i = 0; i < jointEntities.length - 1; i++) { // Iterate over adjacent pairs
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
      // Assuming standard path structure A->B, A->B, check B_i == A_i+1
      if (isRolling) {
        const center = world.getComponent(linkId, PositionComponent).pos;
        const baseRadius = world.getComponent(linkId, RadiusComponent).radius;
        const useLayeredBaseRadius = _layeringFlag(world, 'layeringCableBaseRadius', true);
        const radius = baseRadius + (useLayeredBaseRadius ? this.cableHalfWidth : 0.0);
        const isCw = cw[i + 1];

        const initialStoredLength = signedArcLengthOnWheel(
            joint_i.attachmentPointB_world,
            joint_i_plus_1.attachmentPointA_world,
            center,
            radius,
            isCw,
            true
        );
        this.stored[i + 1] = initialStoredLength;
        this.totalRestLength += initialStoredLength;
      }
    }

    // If amount of stored line is supplied directly, use those values instead of the calculated ones
    if (stored !== null) {
      for (let i = 0; i < this.stored.length; i++) {
        // Allow the user to skip some values by sending null
        // eg [1.0, null, null, 5.0] only sets stored at endpoints.
        if (stored[i] !== null) {
          this.totalRestLength -= this.stored[i];
          this.totalRestLength += stored[i];
          this.stored[i] = stored[i];
        }
      }
    }
  }
}

function _effectiveCW(path, linkIndex, travellingFromCircle) {
  if (linkIndex === 0 && travellingFromCircle)
    return !path.cw[linkIndex];
  return path.cw[linkIndex];
}

function _effectiveRollingRadius(world, path, linkIndex, baseRadius) {
  if (!Number.isFinite(baseRadius) || baseRadius <= EPSILON) {
    return baseRadius;
  }
  if (!_layeringEnabled(world)) {
    return baseRadius;
  }
  if (!path || !Array.isArray(path.linkTypes) || !Array.isArray(path.stored)) {
    return baseRadius;
  }
  if (!_isRolling(path.linkTypes[linkIndex])) {
    return baseRadius;
  }

  const halfWidth = path.cableHalfWidth ?? 0.0;
  if (!(halfWidth > EPSILON)) {
    return baseRadius;
  }

  if (!_layeringFlag(world, 'layeringCableBaseRadius', true)) {
    return baseRadius;
  }

  const fullWidth = 2.0 * halfWidth;
  let effectiveRadius = baseRadius + halfWidth;

  if (!_layeringFlag(world, 'layeringCableStoredLayerRadius', true)) {
    return effectiveRadius;
  }

  // Layered winding is only modeled for hybrid endpoints.
  const isEndpoint = linkIndex === 0 || linkIndex === path.linkTypes.length - 1;
  if (!isEndpoint || !_isHybrid(path.linkTypes[linkIndex])) {
    return effectiveRadius;
  }

  const stored = Math.max(0.0, path.stored[linkIndex] ?? 0.0);
  if (!(stored > EPSILON)) {
    return effectiveRadius;
  }

  const decomposition = _decomposeStoredWrapLayers(stored, baseRadius, halfWidth);
  if (!decomposition) {
    return effectiveRadius;
  }
  if (decomposition.hasPartial) {
    return Math.max(effectiveRadius, decomposition.partialRadius);
  }

  const topLayerRadius = baseRadius + halfWidth + fullWidth * Math.max(0, decomposition.fullLayers - 1);
  return Math.max(effectiveRadius, topLayerRadius);
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
      _effectiveRollingRadius(world, path, preferredLinkIndex, baseRadius)
    );
  }

  const candidateIndices = _pathLinkIndicesForEntity(world, path, entityId);
  for (const linkIndex of candidateIndices) {
    effectiveRadius = Math.max(
      effectiveRadius,
      _effectiveRollingRadius(world, path, linkIndex, baseRadius)
    );
  }
  return effectiveRadius;
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

export function calculateAttachmentPoints(world, joint, path, i) {
  const A = i;
  const B = i + 1;

  const entityA = joint.entityA;
  const entityB = joint.entityB;

  // Get components for Entity A
  const posAComp = world.getComponent(entityA, PositionComponent);
  const radiusAComp = world.getComponent(entityA, RadiusComponent);
  const linkAComp = world.getComponent(entityA, CableLinkComponent);
  const orientationAComp = world.getComponent(entityA, OrientationComponent);

  const posA = posAComp?.pos;
  const attachmentA_previous = joint.attachmentPointA_world;
  const prevPosA = linkAComp?.prevCableAttachmentTimePos;
  const baseRadiusA = radiusAComp?.radius;
  const radiusA = _effectiveRollingRadius(world, path, A, baseRadiusA);
  const angleA = orientationAComp?.angle ?? 0.0;
  const prevAngleA = linkAComp?.prevCableAttachmentTimeAngle ?? 0.0;
  const deltaAngleA = angleA - prevAngleA;

  const cwA = _effectiveCW(path, A, true);
  const attachmentLinkA = _isAttachment(path.linkTypes[A]);
  const rollingLinkA = _isRolling(path.linkTypes[A]);
  const isHybridA = _isHybrid(path.linkTypes[A]);

  const pADiffFromTranslation = (posA && prevPosA) ? posA.clone().subtract(prevPosA) : null;
  const tempRotatedA = attachmentA_previous.clone();
  if (prevPosA) {
    tempRotatedA.rotate(deltaAngleA, prevPosA, true);
  }
  const pADiffFromRotation = tempRotatedA.clone().subtract(attachmentA_previous);

  // Get components for Entity B
  const posBComp = world.getComponent(entityB, PositionComponent);
  const radiusBComp = world.getComponent(entityB, RadiusComponent);
  const linkBComp = world.getComponent(entityB, CableLinkComponent);
  const orientationBComp = world.getComponent(entityB, OrientationComponent);

  const posB = posBComp?.pos;
  const attachmentB_previous = joint.attachmentPointB_world;
  const prevPosB = linkBComp?.prevCableAttachmentTimePos;
  const baseRadiusB = radiusBComp?.radius;
  const radiusB = _effectiveRollingRadius(world, path, B, baseRadiusB);
  const angleB = orientationBComp?.angle ?? 0.0;
  const prevAngleB = linkBComp?.prevCableAttachmentTimeAngle ?? 0.0;
  const deltaAngleB = angleB - prevAngleB;

  const cwB = _effectiveCW(path, B, false);
  const attachmentLinkB = _isAttachment(path.linkTypes[B]);
  const rollingLinkB = _isRolling(path.linkTypes[B]);
  const isHybridB = _isHybrid(path.linkTypes[B]);

  const pBDiffFromTranslation = (posB && prevPosB) ? posB.clone().subtract(prevPosB) : null;
  const tempRotatedB = attachmentB_previous.clone();
  if (prevPosB) {
    tempRotatedB.rotate(deltaAngleB, prevPosB, true);
  }
  const pBDiffFromRotation = tempRotatedB.clone().subtract(attachmentB_previous);

  let attachmentA_current = posA ? posA.clone() : null;
  let attachmentB_current = posB ? posB.clone() : null;

  if (attachmentLinkA && rollingLinkB) {
    if (isHybridA && pADiffFromTranslation) {
      attachmentA_current = attachmentA_previous.clone().add(pADiffFromTranslation).add(pADiffFromRotation);
    }
    if (attachmentA_current && posB && radiusB !== undefined) {
      attachmentB_current = tangentFromPointToCircle(attachmentA_current, posB, radiusB, cwB).a_circle;
    }
  } else if (rollingLinkA && attachmentLinkB) {
    if (isHybridB && pBDiffFromTranslation) {
      attachmentB_current = attachmentB_previous.clone().add(pBDiffFromTranslation).add(pBDiffFromRotation);
    }
    if (attachmentB_current && posA && radiusA !== undefined) {
      attachmentA_current = tangentFromCircleToPoint(attachmentB_current, posA, radiusA, cwA).a_circle;
    }
  } else if (rollingLinkA && rollingLinkB) {
    if (posA && posB && radiusA !== undefined && radiusB !== undefined) {
      const tangents = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
      attachmentA_current = tangents.a_circle;
      attachmentB_current = tangents.b_circle;
    }
  } else {
    if (isHybridA && pADiffFromTranslation) {
      attachmentA_current = attachmentA_previous.clone().add(pADiffFromTranslation).add(pADiffFromRotation);
    }
    if (isHybridB && pBDiffFromTranslation) {
      attachmentB_current = attachmentB_previous.clone().add(pBDiffFromTranslation).add(pBDiffFromRotation);
    }
  }

  return { attachmentA_current, attachmentB_current };
}

export function _updateAttachmentPoints(world) {
  const step = Math.floor(_readFiniteResource(world, 'cableHybridTransitionStep', 0));
  const pathEntities = world.query([CablePathComponent]);

  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);

    for (let i = 0; i < path.jointEntities.length; i++) {
      const jointId = path.jointEntities[i];
      const joint = world.getComponent(jointId, CableJointComponent);

      const attachmentA_previous = joint.attachmentPointA_world.clone();
      const attachmentB_previous = joint.attachmentPointB_world.clone();

      let { attachmentA_current, attachmentB_current } = calculateAttachmentPoints(world, joint, path, i);

      const A = i;
      const B = i + 1;
      const entityA = joint.entityA;
      const entityB = joint.entityB;

      // Get components for Entity A
      const posAComp = world.getComponent(entityA, PositionComponent);
      const radiusAComp = world.getComponent(entityA, RadiusComponent);
      const linkAComp = world.getComponent(entityA, CableLinkComponent);
      const orientationAComp = world.getComponent(entityA, OrientationComponent);
      const posA = posAComp?.pos;
      const prevPosA = linkAComp?.prevCableAttachmentTimePos;
      const baseRadiusA = radiusAComp?.radius;
      const radiusA = _effectiveRollingRadius(world, path, A, baseRadiusA);
      const angleA = orientationAComp?.angle ?? 0.0;
      const prevAngleA = linkAComp?.prevCableAttachmentTimeAngle ?? 0.0;
      const deltaAngleA = angleA - prevAngleA;
      const cwA = _effectiveCW(path, A, true);
      const rollingLinkA = _isRolling(path.linkTypes[A]);
      const isHybridA = _isHybrid(path.linkTypes[A]);
      const hasFrictionA = world.getComponent(entityA, CoefficientOfFrictionComponent);

      // Get components for Entity B
      const posBComp = world.getComponent(entityB, PositionComponent);
      const radiusBComp = world.getComponent(entityB, RadiusComponent);
      const linkBComp = world.getComponent(entityB, CableLinkComponent);
      const orientationBComp = world.getComponent(entityB, OrientationComponent);
      const posB = posBComp?.pos;
      const prevPosB = linkBComp?.prevCableAttachmentTimePos;
      const baseRadiusB = radiusBComp?.radius;
      const radiusB = _effectiveRollingRadius(world, path, B, baseRadiusB);
      const angleB = orientationBComp?.angle ?? 0.0;
      const prevAngleB = linkBComp?.prevCableAttachmentTimeAngle ?? 0.0;
      const deltaAngleB = angleB - prevAngleB;
      const cwB = _effectiveCW(path, B, false);
      const rollingLinkB = _isRolling(path.linkTypes[B]);
      const isHybridB = _isHybrid(path.linkTypes[B]);
      const hasFrictionB = world.getComponent(entityB, CoefficientOfFrictionComponent);

      let sA = 0;
      let sB = 0;
      const restBefore = joint.restLength;
      const storedA_before = path.stored[A] ?? 0.0;
      const storedB_before = path.stored[B] ?? 0.0;

      if (rollingLinkA && attachmentA_previous && attachmentA_current && prevPosA && posA && radiusA !== undefined) {
          sA = signedArcLengthOnWheel(
            attachmentA_previous.clone().subtract(prevPosA),
            attachmentA_current.clone().subtract(posA),
            new Vector2(0.0, 0.0),
            radiusA,
            cwA
          );
          if (isHybridA || hasFrictionA) {
            sA += (cwA ? deltaAngleA * radiusA : -deltaAngleA * radiusA);
          }
      }

      if (rollingLinkB && attachmentB_previous && attachmentB_current && prevPosB && posB && radiusB !== undefined) {
          sB = signedArcLengthOnWheel(
            attachmentB_previous.clone().subtract(prevPosB),
            attachmentB_current.clone().subtract(posB),
            new Vector2(0.0, 0.0),
            radiusB,
            cwB
          );
          if (isHybridB || hasFrictionB) {
            sB += (cwB ? deltaAngleB * radiusB : -deltaAngleB * radiusB);
          }
      }
      const sA_raw = sA;
      const sB_raw = sB;

      let sA_effective = sA;
      let sB_effective = sB;
      let clampApplied = false;
      let orientationCorrectionA = 0.0;
      let orientationCorrectionB = 0.0;
      const clampEnabled = _featureFlag(world, 'layeringClampJointRestLength', true);
      if (clampEnabled && Number.isFinite(restBefore)) {
        const unclampedRest = restBefore - sA_effective + sB_effective;
        if (unclampedRest < MIN_JOINT_REST_LENGTH) {
          const requiredLift = MIN_JOINT_REST_LENGTH - unclampedRest;
          const decreaseFromA = Math.max(0.0, sA_effective);
          const decreaseFromB = Math.max(0.0, -sB_effective);
          const totalDecrease = decreaseFromA + decreaseFromB;
          if (totalDecrease > EPSILON) {
            const shiftA = requiredLift * (decreaseFromA / totalDecrease);
            const shiftB = requiredLift - shiftA;
            sA_effective -= shiftA;
            sB_effective += shiftB;
          } else {
            sB_effective += requiredLift;
          }

          const stillLowRest = restBefore - sA_effective + sB_effective;
          if (stillLowRest < MIN_JOINT_REST_LENGTH) {
            sB_effective += (MIN_JOINT_REST_LENGTH - stillLowRest);
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
            clampedRest: restBefore - sA_effective + sB_effective,
            sA_raw,
            sB_raw,
            sA_effective,
            sB_effective
          });
        }
      }
      const blockedSA = sA_raw - sA_effective;
      const blockedSB = sB_effective - sB_raw;
      let localAttachmentDriftA = null;
      let localAttachmentDriftB = null;
      let localAttachmentBeforeA = null;
      let localAttachmentBeforeB = null;
      let localAttachmentAfterA = null;
      let localAttachmentAfterB = null;
      if (clampApplied) {
        const shouldProjectOrientationA = (
          rollingLinkA &&
          orientationAComp &&
          Number.isFinite(radiusA) &&
          radiusA > EPSILON &&
          Math.abs(blockedSA) > EPSILON &&
          (isHybridA || Boolean(hasFrictionA))
        );
        if (
          shouldProjectOrientationA
        ) {
          const angleBeforeProjection = orientationAComp.angle;
          const attachmentBeforeProjection = attachmentA_current ? attachmentA_current.clone() : null;
          orientationCorrectionA = (cwA ? -blockedSA : blockedSA) / radiusA;
          orientationAComp.angle += orientationCorrectionA;
          if (isHybridA && attachmentA_current && posA) {
            // Keep hybrid attachment-point world geometry coherent with the angle projection.
            attachmentA_current.rotate(orientationCorrectionA, posA, true);
            const localBefore = _computeLocalAttachment(attachmentBeforeProjection, posA, angleBeforeProjection);
            const localAfter = _computeLocalAttachment(attachmentA_current, posA, orientationAComp.angle);
            if (localBefore && localAfter) {
              localAttachmentBeforeA = _vecSnapshot(localBefore);
              localAttachmentAfterA = _vecSnapshot(localAfter);
              localAttachmentDriftA = localBefore.distanceTo(localAfter);
            }
          }
        }
        const shouldProjectOrientationB = (
          rollingLinkB &&
          orientationBComp &&
          Number.isFinite(radiusB) &&
          radiusB > EPSILON &&
          Math.abs(blockedSB) > EPSILON &&
          (isHybridB || Boolean(hasFrictionB))
        );
        if (
          shouldProjectOrientationB
        ) {
          const angleBeforeProjection = orientationBComp.angle;
          const attachmentBeforeProjection = attachmentB_current ? attachmentB_current.clone() : null;
          orientationCorrectionB = (cwB ? blockedSB : -blockedSB) / radiusB;
          orientationBComp.angle += orientationCorrectionB;
          if (isHybridB && attachmentB_current && posB) {
            // Keep hybrid attachment-point world geometry coherent with the angle projection.
            attachmentB_current.rotate(orientationCorrectionB, posB, true);
            const localBefore = _computeLocalAttachment(attachmentBeforeProjection, posB, angleBeforeProjection);
            const localAfter = _computeLocalAttachment(attachmentB_current, posB, orientationBComp.angle);
            if (localBefore && localAfter) {
              localAttachmentBeforeB = _vecSnapshot(localBefore);
              localAttachmentAfterB = _vecSnapshot(localAfter);
              localAttachmentDriftB = localBefore.distanceTo(localAfter);
            }
          }
        }
        if (Math.abs(orientationCorrectionA) > EPSILON || Math.abs(orientationCorrectionB) > EPSILON) {
          _recordCableEventTrace(world, step, {
            type: 'rest-length-orientation-projection',
            stage: 'updateAttachmentPoints',
            pathId,
            jointId,
            jointIndex: i,
            entityA,
            entityB,
            blockedSA,
            blockedSB,
            radiusA,
            radiusB,
            cwA,
            cwB,
            orientationCorrectionA,
            orientationCorrectionB,
            localAttachmentDriftA,
            localAttachmentDriftB,
            localAttachmentBeforeA,
            localAttachmentAfterA,
            localAttachmentBeforeB,
            localAttachmentAfterB
          });
        }
      }

      path.stored[A] += sA_effective;
      joint.restLength -= sA_effective;
      path.stored[B] -= sB_effective;
      joint.restLength += sB_effective;
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
          sA_effective,
          sB_effective,
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
    // A merge operation in itself might move attachment points in such a way
    // that an additional merge operation is required, or a single joint might
    // require a merge of both of its ends at the same time step. We re run merge
    // until all such cases are resolved for a given time step.
    let reRunMerge = true;
    while (reRunMerge) {
      reRunMerge = false;
      for (let i = 0; i < path.jointEntities.length - 1; i++) { // Iterate over adjacent pairs
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
        if (!_layeringEnabled(world) && joint_i.entityA === joint_i_plus_1.entityB) {
          // Legacy behavior: if cable wraps around a link and back to the same body,
          // do not merge this pair.
          continue;
        }
        if (path.stored[i + 1] < 0.0) {
          // console.log(`Merging joints ${jointId_i} and ${jointId_i_plus_1} (stored: ${path.stored[i + 1].toFixed(4)})`);

          // Calculate angle between the two segments, just for debug
          const pA1 = joint_i.attachmentPointA_world;
          const pB2 = joint_i_plus_1.attachmentPointB_world;
          const posA = world.getComponent(joint_i.entityA, PositionComponent).pos;
          const radiusA = _effectivePathRadiusForEntity(world, path, joint_i.entityA, i);
          const cwA = _effectiveCW(path, i, true);
          const posB = world.getComponent(joint_i_plus_1.entityB, PositionComponent).pos;
          const radiusB = _effectivePathRadiusForEntity(world, path, joint_i_plus_1.entityB, i + 2);
          const cwB = path.cw[i+2];

          const storedLeftBefore = path.stored[i] ?? 0.0;
          const storedMiddleBefore = path.stored[i + 1] ?? 0.0;
          const storedRightBefore = path.stored[i + 2] ?? 0.0;
          const restLeftBefore = joint_i.restLength;
          const restRightBefore = joint_i_plus_1.restLength;
          const pathJointCountBefore = path.jointEntities.length;

          joint_i.restLength += joint_i_plus_1.restLength + path.stored[i + 1];
          joint_i.entityB = joint_i_plus_1.entityB;
          const isAttachmentA = _isAttachment(path.linkTypes[i]);
          const isRollingA = _isRolling(path.linkTypes[i]);
          const isAttachmentB = _isAttachment(path.linkTypes[i+2]);
          const isRollingB = _isRolling(path.linkTypes[i+2]);

          let attachmentA_current = pA1;
          let attachmentB_current = pB2;
          if (isRollingA && isRollingB) {
            const tangents = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
            attachmentA_current = tangents.a_circle;
            attachmentB_current = tangents.b_circle;
          } else if (isRollingA && isAttachmentB) {
            attachmentA_current = tangentFromCircleToPoint(pB2, posA, radiusA, cwA).a_circle;
          } else if (isAttachmentA && isRollingB) {
            attachmentB_current = tangentFromPointToCircle(pA1, posB, radiusB, cwB).a_circle;
          }

          let sA = 0.0;
          let sB = 0.0;
          if (isRollingA && isRollingB) {
            sA = signedArcLengthOnWheel(pA1, attachmentA_current, posA, radiusA, cwA);
            sB = signedArcLengthOnWheel(pB2, attachmentB_current, posB, radiusB, cwB);
          } else if (isRollingA && isAttachmentB) {
            sA = signedArcLengthOnWheel(pA1, attachmentA_current, posA, radiusA, cwA);
          } else if (isAttachmentA && isRollingB) {
            sB = signedArcLengthOnWheel(pB2, attachmentB_current, posB, radiusB, cwB);
          }

          path.stored[i] += sA;
          joint_i.restLength -= sA;
          path.stored[i+2] -= sB;
          joint_i.restLength += sB;
          reRunMerge = path.stored[i] < 0.0 || path.stored[i+2] < 0.0;

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

          path.jointEntities.splice(i+1, 1);
          path.stored.splice(i+1, 1);
          path.cw.splice(i+1, 1);
          path.linkTypes.splice(i+1, 1);
          world.destroyEntity(jointId_i_plus_1);
        }
      }
    }
  }
}

export function _splitJoints(world) {
  const step = Math.floor(_readFiniteResource(world, 'cableHybridTransitionStep', 0));
  const qualityGuardEnabled = _featureFlag(world, 'layeringSplitQualityGuard', false);
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
        const posSplitter = world.getComponent(splitterId, PositionComponent).pos;
        const radiusSplitter = _effectivePathRadiusForEntity(world, path, splitterId, null);
        if (lineSegmentCircleIntersection(pA, pB, posSplitter, radiusSplitter)) {
          // console.log(`Splitting joint ${jointId} due to intersection with ${splitterId}`);
          const entityA = joint.entityA;
          const entityB = joint.entityB;
          const newJointId = world.createEntity();
          ensureMachineTag(world, newJointId, pathMachine);

          // Get components for Entity A
          const posA = world.getComponent(entityA, PositionComponent).pos;
          const linkTypeA = path.linkTypes[i];
          const isAttachmentA = _isAttachment(linkTypeA);
          const isRollingA = _isRolling(linkTypeA);
          const radiusA = _effectivePathRadiusForEntity(world, path, entityA, i);
          const cwA = _effectiveCW(path, i, true);

          // Get components for Entity B
          const posB = world.getComponent(entityB, PositionComponent).pos;
          const linkTypeB = path.linkTypes[i + 1];
          const isAttachmentB = _isAttachment(linkTypeB);
          const isRollingB = _isRolling(linkTypeB);
          const radiusB = _effectivePathRadiusForEntity(world, path, entityB, i + 1);
          const cwB = path.cw[i + 1];

          // Calculate components for new joint
          const prevPosSplitter = world.getComponent(splitterId, CableLinkComponent).prevCableAttachmentTimePos;
          const cw = rightOfLine(prevPosSplitter, pA, pB);

          let newAttachmentPointAForJoint = null;
          let newAttachmentPointBForJoint = null;
          if (isRollingA) {
            // Rolling -> Splitter
            const tangentAS = tangentFromCircleToCircle(posA, radiusA, cwA, posSplitter, radiusSplitter, cw);
            newAttachmentPointAForJoint = tangentAS.a_circle;
            newAttachmentPointBForJoint = tangentAS.b_circle;
          } else if (isAttachmentA) {
            // Attachment -> Splitter
            const tangentAS = tangentFromPointToCircle(pA, posSplitter, radiusSplitter, cw);
            newAttachmentPointAForJoint = tangentAS.a_attach;
            newAttachmentPointBForJoint = tangentAS.a_circle;
          } else {
            console.warn(`Splitting cable joint coming from link type ${linkTypeA} is not supported.`);
            continue;
          }

          let attachmentPointAForNewJoint = null;
          let attachmentPointBForNewJoint = null;
          if (isRollingB) {
            // Splitter -> Rolling
            const tangentSB = tangentFromCircleToCircle(posSplitter, radiusSplitter, cw, posB, radiusB, cwB);
            attachmentPointAForNewJoint = tangentSB.a_circle;
            attachmentPointBForNewJoint = tangentSB.b_circle;
          } else if (isAttachmentB) {
            // Splitter -> Attachment
            const tangentSB = tangentFromCircleToPoint(pB, posSplitter, radiusSplitter, cw);
            attachmentPointAForNewJoint = tangentSB.a_circle;
            attachmentPointBForNewJoint = tangentSB.a_attach;
          } else {
            console.warn(`Splitting cable joint attached to ${linkTypeB} is not supported.`);
          }

          let sA = 0.0;
          let sB = 0.0;
          if (isRollingA) {
            sA = signedArcLengthOnWheel(pA, newAttachmentPointAForJoint, posA, radiusA, cwA);
          }
          if (isRollingB) {
            sB = signedArcLengthOnWheel(pB, attachmentPointBForNewJoint, posB, radiusB, cwB);
          }

          // Calculate stored length 's' on the new splitter link
          const s = signedArcLengthOnWheel(
              newAttachmentPointBForJoint, // Point on splitter from A side
              attachmentPointAForNewJoint, // Point on splitter from B side
              posSplitter,
              radiusSplitter,
              cw
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

          // The line is slightly stretched due to the intersection with the new link (the splitterId Entity).
          // Distributes the discrepancy (length_error) to both sides of the new link such that tension remains constant on both sides of the new link.
          // The paper states:
          // "The rest length of the original joint is distributed among the new joints such that the tension on both sides are equal."
          // "The rest length is split such that the tension remains constant."
          // "tension (the current length divided by the rest length)"
          // Distribute original rest length (minus new stored length s)
          // between the two new segments to maintain equal tension.
          const dAS = newAttachmentPointAForJoint.clone().subtract(newAttachmentPointBForJoint).length();
          const dSB = attachmentPointAForNewJoint.clone().subtract(attachmentPointBForNewJoint).length();
          const originalRestLength = joint.restLength + sB - sA; // Store before modifying
          const totalDist = dAS + dSB;
          const availableRestLength = originalRestLength - s;
          const storedA_before = path.stored[i] ?? 0.0;
          const storedB_before = path.stored[i + 1] ?? 0.0;
          const restBefore = joint.restLength;
          const pathJointCountBefore = path.jointEntities.length;

          let newRestLengthAS = 0; // For original joint (entityA -> splitter)
          let newRestLengthSB = 0; // For new joint (splitter -> entityB)
          if (totalDist > 1e-9) {
              if (availableRestLength < 1e-9) {
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
                  console.warn(`Split resulted in < 1e-9 available rest length (${availableRestLength.toFixed(4)}). Aborting split.`);
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
          if (qualityGuardEnabled) {
            const halfWidth = Number.isFinite(path.cableHalfWidth) ? Math.max(0.0, path.cableHalfWidth) : 0.0;
            const qualityThreshold = Math.max(1e-3, 3.0 * halfWidth);
            const minNewRestLength = Math.min(newRestLengthAS, newRestLengthSB);
            const minNewSegmentLength = Math.min(dAS, dSB);
            if (
              !Number.isFinite(minNewRestLength) ||
              !Number.isFinite(minNewSegmentLength) ||
              minNewRestLength < qualityThreshold ||
              minNewSegmentLength < qualityThreshold
            ) {
              _recordCableEventTrace(world, step, {
                type: 'split-abort',
                reason: 'quality-guard',
                pathId,
                jointId,
                splitterId,
                splitIndex: i,
                qualityThreshold,
                minNewRestLength,
                minNewSegmentLength,
                newRestLengthAS,
                newRestLengthSB,
                dAS,
                dSB
              });
              continue;
            }
          }
          path.stored[i + 1] -= sB;
          joint.restLength += sB;
          path.jointEntities.splice(i + 1, 0, newJointId);
          path.cw.splice(i + 1, 0, cw);
          path.linkTypes.splice(i + 1, 0, 'rolling');
          joint.attachmentPointA_world.set(newAttachmentPointAForJoint);
          path.stored[i] += sA;
          joint.restLength -= sA;
          joint.entityB = splitterId; // Original joint now connects A -> Splitter
          path.stored.splice(i + 1, 0, s);

          // Update original joint (now entityA -> splitter)
          joint.restLength = newRestLengthAS;
          joint.attachmentPointB_world.set(newAttachmentPointBForJoint); // Update endpoint

          // Create the new joint (splitter -> entityB)
          world.addComponent(newJointId, new CableJointComponent(
              splitterId, entityB, newRestLengthSB, attachmentPointAForNewJoint, attachmentPointBForNewJoint));
          world.addComponent(newJointId, new RenderableComponent('line', linecolor1));

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

          // Some final debug logging
          const discrepancy = originalRestLength - s - newRestLengthAS - newRestLengthSB;
          const tensionAS = dAS/newRestLengthAS;
          const tensionSB = dSB/newRestLengthSB;
          // console.log(`Split: L_orig=${originalRestLength.toFixed(4)}, s=${s.toFixed(4)} -> L_AS=${newRestLengthAS.toFixed(4)} (d_AS=${dAS.toFixed(4)}), L_SB=${newRestLengthSB.toFixed(4)} (d_SB=${initialDistSB.toFixed(4)})`);
          // console.log(`Split stats: discrepancy=${discrepancy.toFixed(4)}, tensionAS=${tensionAS.toFixed(4)}, tensionSB=${tensionSB.toFixed(4)}`);
          if (discrepancy > 1.0) {
            console.warn("discrepancy > 1.0");
          }
          if (tensionAS > 1.5) {
            console.warn("tensionAS > 1.5");
          }
          if (tensionSB > 1.5) {
            console.warn("tensionSB > 1.5");
          }
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
        if (path.stored[i] < 0.0) {
          // console.log(`Switching joint ${path.jointEntities[i == 0 ? 0 : path.jointEntities.length - 1]} to hybrid-attachment`);
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
          const pos = world.getComponent(linkEntity, PositionComponent)?.pos;
          const radius = _effectiveRollingRadius(
            world,
            path,
            i,
            world.getComponent(linkEntity, RadiusComponent)?.radius
          );
          const oldRestLength = joint.restLength;
          const attachmentBefore = i === 0
            ? joint.attachmentPointA_world.clone()
            : joint.attachmentPointB_world.clone();
          const neighborAttachmentPoint = i === 0
            ? joint.attachmentPointB_world
            : joint.attachmentPointA_world;
          // We have "fed out negative line", undo that
          joint.restLength += oldStored;
          const rotationApplied = Boolean(pos && Number.isFinite(radius) && radius > EPSILON);
          if (pos && Number.isFinite(radius) && radius > EPSILON) {
            const rotAng = -oldStored / radius;
            if (i === 0) {
              joint.attachmentPointA_world.rotate(rotAng, pos, path.cw[i]);
            } else if (i === path.linkTypes.length - 1) {
              joint.attachmentPointB_world.rotate(rotAng, pos, path.cw[i]);
            }
          }
          path.stored[i] = 0;

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
            effectiveRadius: radius
          });
        }
      }
      else if (path.linkTypes[i] === 'hybrid-attachment') {
        let jointId, joint, entityId, attachmentPoint, neighborId, neighborAttachmentPoint;
        if (i === 0) {
          jointId = path.jointEntities[0];
          joint   = world.getComponent(jointId, CableJointComponent);
          entityId        = joint.entityA;
          attachmentPoint = joint.attachmentPointA_world;
          neighborId      = joint.entityB;
          neighborAttachmentPoint = joint.attachmentPointB_world;
        }
        else if (i === path.linkTypes.length - 1) {
          jointId = path.jointEntities[path.jointEntities.length - 1];
          joint   = world.getComponent(jointId, CableJointComponent);
          entityId        = joint.entityB;
          attachmentPoint = joint.attachmentPointB_world;
          neighborId      = joint.entityA;
          neighborAttachmentPoint = joint.attachmentPointA_world;
        }

        const C = world.getComponent(entityId, PositionComponent).pos;
        const R = _effectiveRollingRadius(
          world,
          path,
          i,
          world.getComponent(entityId, RadiusComponent)?.radius
        );
        const lastEndpointIndex = path.linkTypes.length - 1;
        const neighborLinkIndex = i === 0 ? 1 : Math.max(0, lastEndpointIndex - 1);
        const neighborPos = world.getComponent(neighborId, PositionComponent)?.pos;
        const neighborRadius = _effectiveRollingRadius(
          world,
          path,
          neighborLinkIndex,
          world.getComponent(neighborId, RadiusComponent)?.radius
        );
        const attachmentDistance = attachmentPoint?.distanceTo?.(neighborAttachmentPoint) ?? Infinity;
        const centerDistance = (
          C &&
          neighborPos &&
          Number.isFinite(C.x) &&
          Number.isFinite(C.y) &&
          Number.isFinite(neighborPos.x) &&
          Number.isFinite(neighborPos.y)
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

        // When the endpoint and neighbor attachment collapse to (near) the same
        // point, both CW/CCW tangent solutions become ill-conditioned.
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
        const tanCW  = tangentFromCircleToPoint(neighborAttachmentPoint, C, R, true).a_circle;
        const tanCCW = tangentFromCircleToPoint(neighborAttachmentPoint, C, R, false).a_circle;
        const tangentFinite = (
          tanCW &&
          tanCCW &&
          Number.isFinite(tanCW.x) &&
          Number.isFinite(tanCW.y) &&
          Number.isFinite(tanCCW.x) &&
          Number.isFinite(tanCCW.y)
        );

        const crossedCW  = signedArcLengthOnWheel(attachmentPoint, tanCW,  C, R, true);
        const crossedCCW  = signedArcLengthOnWheel(attachmentPoint, tanCCW,  C, R, false);
        const distSqCW = attachmentPoint.distanceToSq(tanCW);
        const distSqCCW = attachmentPoint.distanceToSq(tanCCW);

        let newCW = null;
        let crossingTangent = null;
        let candidateStored = null;
        if (crossedCCW > 0.0 && distSqCCW < distSqCW) {
            newCW = true;
            crossingTangent = tanCCW;
            candidateStored = crossedCCW;
        } else if (crossedCW > 0.0 && distSqCW < distSqCCW) {
            newCW = false;
            crossingTangent = tanCW;
            candidateStored = crossedCW;
        }

        if (newCW !== null) {
          const oldStored = path.stored[i] ?? 0.0;
          const oldLinkType = path.linkTypes[i];
          const oldCw = path.cw[i];
          const newStored = candidateStored ?? oldStored;
          const oldRestLength = joint.restLength;
          const attachmentBefore = attachmentPoint.clone();
          // console.log(`Switching joint ${jointId} to hybrid`);
          path.linkTypes[i] = 'hybrid';
          path.cw[i]        = newCW;
          path.stored[i] = newStored;
          joint.restLength -= (newStored - oldStored);
          attachmentPoint.set(crossingTangent);
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
            crossedCCW
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
            reason: 'no-transition'
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
      const center = world.getComponent(bodyId, PositionComponent)?.pos;
      const radius = _effectiveRollingRadius(
        world,
        path,
        linkIndex,
        world.getComponent(bodyId, RadiusComponent)?.radius
      );
      if (!center || !Number.isFinite(radius) || radius <= EPSILON) {
        continue;
      }

      const cw = path.cw[linkIndex];
      const arc = signedArcLengthOnWheel(
        leftJoint.attachmentPointB_world,
        rightJoint.attachmentPointA_world,
        center,
        radius,
        cw,
        true
      );
      const altArc = signedArcLengthOnWheel(
        leftJoint.attachmentPointB_world,
        rightJoint.attachmentPointA_world,
        center,
        radius,
        !cw,
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

function _decomposeStoredWrapLayers(storedLength, baseRadius, halfWidth) {
  const stored = Math.max(0.0, storedLength ?? 0.0);
  if (!(stored > EPSILON) || !(baseRadius > EPSILON) || !(halfWidth > EPSILON)) {
    return null;
  }

  const fullWidth = 2.0 * halfWidth;
  const firstLayerRadius = baseRadius + halfWidth;
  let remaining = stored;
  let fullLayers = 0;
  const MAX_LAYERS = 2048;
  while (fullLayers < MAX_LAYERS) {
    const layerRadius = firstLayerRadius + fullWidth * fullLayers;
    if (!(layerRadius > EPSILON)) {
      break;
    }
    const layerCircumference = 2.0 * Math.PI * layerRadius;
    if (remaining + EPSILON >= layerCircumference) {
      remaining -= layerCircumference;
      if (remaining < 0.0) {
        remaining = 0.0;
      }
      fullLayers++;
      continue;
    }

    const partialLength = remaining;
    return {
      fullLayers,
      partialLength,
      partialRadius: layerRadius,
      hasPartial: partialLength > EPSILON
    };
  }

  const fallbackRadius = firstLayerRadius + fullWidth * fullLayers;
  return {
    fullLayers,
    partialLength: 0.0,
    partialRadius: fallbackRadius,
    hasPartial: false
  };
}

export class CableAttachmentUpdateSystem {
  runInPause = false;

  update(world, _dt_unused) {
    const prevHybridStep = Math.floor(_readFiniteResource(world, 'cableHybridTransitionStep', 0));
    const nextHybridStep = prevHybridStep + 1;
    world.setResource('cableHybridTransitionStep', nextHybridStep);
    _recordCableStepSummary(world, nextHybridStep, 'begin');

    _clearDebugPoints(world);
    if (_featureFlag(world, 'layeringHybridLinkStates', true)) {
      _updateHybridLinkStates(world, nextHybridStep);
    }
    _recordCableStepSummary(world, nextHybridStep, 'afterHybrid');
    if (_featureFlag(world, 'layeringAttachmentUpdatePoints', true)) {
      _updateAttachmentPoints(world);
    }
    _recordCableStepSummary(world, nextHybridStep, 'afterAttachment');
    if (_featureFlag(world, 'layeringMergeJoints', true)) {
      _mergeJoints(world);
    }
    _recordCableStepSummary(world, nextHybridStep, 'afterMerge');
    if (_featureFlag(world, 'layeringSplitJoints', true)) {
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
    const ITERATIONS = 2; // 0: Forward, 1: Backward

    // Pre-calculate local offsets for all joints to ensure attachment points move with bodies
    // during the multi-iteration solve. This prevents "stale" world points from distorting geometry.
    const jointLocals = new Map();
    const computeLocal = (entityId, worldPoint) => {
      const posComp = world.getComponent(entityId, PositionComponent);
      const orientComp = world.getComponent(entityId, OrientationComponent);
      if (!posComp) return new Vector2(0, 0); // Should not happen for valid bodies

      const rel = worldPoint.clone().subtract(posComp.pos);
      if (orientComp) {
        // Rotate backwards by -angle to get local frame
        const c = Math.cos(-orientComp.angle);
        const s = Math.sin(-orientComp.angle);
        return new Vector2(rel.x * c - rel.y * s, rel.x * s + rel.y * c);
      }
      return rel;
    };

    const applyConstraint = (
      entityA,
      entityB,
      pointA_world,
      pointB_world,
      gradPosA,
      gradPosB,
      constraintError,
      compliance
    ) => {
      if (constraintError <= EPSILON) {
        return;
      }

      const massAComp = world.getComponent(entityA, MassComponent);
      const invMassA = (massAComp && massAComp.mass > 0) ? 1.0 / massAComp.mass : 0.0;
      const moiAComp = world.getComponent(entityA, MomentOfInertiaComponent);
      const invInertiaA = moiAComp ? moiAComp.invInertia : 0.0;

      const massBComp = world.getComponent(entityB, MassComponent);
      const invMassB = (massBComp && massBComp.mass > 0) ? 1.0 / massBComp.mass : 0.0;
      const moiBComp = world.getComponent(entityB, MomentOfInertiaComponent);
      const invInertiaB = moiBComp ? moiBComp.invInertia : 0.0;

      // If both entities are effectively immovable for this constraint, skip.
      if (invMassA + invMassB + invInertiaA + invInertiaB <= EPSILON) {
        return;
      }

      const posAComp = world.getComponent(entityA, PositionComponent);
      const posBComp = world.getComponent(entityB, PositionComponent);
      if (!posAComp || !posBComp) {
        return;
      }

      const rA = new Vector2().subtractVectors(pointA_world, posAComp.pos);
      const rB = new Vector2().subtractVectors(pointB_world, posBComp.pos);
      const gradAngA = rA.x * gradPosA.y - rA.y * gradPosA.x;
      const gradAngB = rB.x * gradPosB.y - rB.y * gradPosB.x;

      let denom = 0.0;
      denom += invMassA * gradPosA.lengthSq();
      denom += invInertiaA * gradAngA * gradAngA;
      denom += invMassB * gradPosB.lengthSq();
      denom += invInertiaB * gradAngB * gradAngB;
      if (Number.isFinite(dt) && dt > EPSILON) {
        denom += (compliance ?? 0.0) / (dt * dt);
      }

      if (denom <= EPSILON) {
        return;
      }

      const lambda = -constraintError / denom;

      if (invMassA > 0.0) {
        const deltaPosA = gradPosA.clone().scale(-invMassA * lambda);
        posAComp.pos.add(deltaPosA);
      }
      if (invInertiaA > 0.0) {
        const deltaAngA = -invInertiaA * lambda * gradAngA;
        const orientationAComp = world.getComponent(entityA, OrientationComponent);
        if (orientationAComp) {
          orientationAComp.angle += deltaAngA;
        }
      }

      if (invMassB > 0.0) {
        const deltaPosB = gradPosB.clone().scale(-invMassB * lambda);
        posBComp.pos.add(deltaPosB);
      }
      if (invInertiaB > 0.0) {
        const deltaAngB = -invInertiaB * lambda * gradAngB;
        const orientationBComp = world.getComponent(entityB, OrientationComponent);
        if (orientationBComp) {
          orientationBComp.angle += deltaAngB;
        }
      }
    };

    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (path.jointEntities.length < 1) continue;
      for (const jointId of path.jointEntities) {
        const joint = world.getComponent(jointId, CableJointComponent);
        jointLocals.set(jointId, {
          localA: computeLocal(joint.entityA, joint.attachmentPointA_world),
          localB: computeLocal(joint.entityB, joint.attachmentPointB_world)
        });
      }
    }

    for (let iter = 0; iter < ITERATIONS; iter++) {
      const isForward = (iter % 2 === 0);

      const startPath = isForward ? 0 : pathEntities.length - 1;
      const endPath = isForward ? pathEntities.length : -1;
      const stepPath = isForward ? 1 : -1;

      for (let p = startPath; p !== endPath; p += stepPath) {
        const pathId = pathEntities[p];
        const path = world.getComponent(pathId, CablePathComponent);
        if (path.jointEntities.length < 1) continue;

        const startJoint = isForward ? 0 : path.jointEntities.length - 1;
        const endJoint = isForward ? path.jointEntities.length : -1;
        const stepJoint = isForward ? 1 : -1;

        for (let j = startJoint; j !== endJoint; j += stepJoint) {
          const jointId = path.jointEntities[j];
          const joint = world.getComponent(jointId, CableJointComponent);
          const locals = jointLocals.get(jointId);

          const entityA = joint.entityA;
          const entityB = joint.entityB;

          const pA = _computeWorldAttachment(world, entityA, locals.localA);
          const pB = _computeWorldAttachment(world, entityB, locals.localB);

          const currentSegmentLength = pA.distanceTo(pB);
          let dir = null;
          if (currentSegmentLength > EPSILON) {
            dir = new Vector2().subtractVectors(pB, pA).scale(1.0 / currentSegmentLength);
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
            entityA,
            entityB,
            pA,
            pB,
            gradPosA,
            gradPosB,
            constraintError,
            path.compliance
          );
        } // End loop through joints in path

      } // End loop through paths
    } // End loop through iterations
  } // end update
} // end PBDCableConstraintSolver
