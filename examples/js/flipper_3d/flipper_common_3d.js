import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import {
  PositionComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
  PrevFinalPosComponent,
  OrientationComponent,
  PrevFinalOrientationComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
  layeringEnabled,
  BallTagComponent,
  ObstacleTagComponent,
  ObstaclePushComponent
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import {
  closestPointOnSegment,
  rightOfPlane
} from '../../../src/js/cable_joints_3d/geometry3.js';

export { BallTagComponent, ObstacleTagComponent, ObstaclePushComponent };

const DEFAULT_PLANE_NORMAL = new Vector3(0, 0, 1);

function normalizePlaneNormal(planeNormal) {
  const n = (planeNormal || DEFAULT_PLANE_NORMAL).clone();
  if (n.lengthSq() <= 1e-12) {
    n.set(DEFAULT_PLANE_NORMAL);
  } else {
    n.normalize();
  }
  return n;
}

function projectPointToPlane(point, planeNormal, planeOffset) {
  const signedDistance = point.dot(planeNormal) - planeOffset;
  point.subtract(planeNormal, signedDistance);
}

function projectVectorToPlane(vector, planeNormal) {
  const normalComponent = vector.dot(planeNormal);
  vector.subtract(planeNormal, normalComponent);
}

function buildPlaneBasis(planeNormal) {
  const n = planeNormal.clone();
  const eps = 1e-9;
  if (n.lengthSq() <= eps) {
    return {
      n: new Vector3(0, 0, 1),
      u: new Vector3(1, 0, 0),
      v: new Vector3(0, 1, 0)
    };
  }

  n.normalize();
  let reference = Math.abs(n.x) < 0.9 ? new Vector3(1, 0, 0) : new Vector3(0, 1, 0);
  const nDotRef = n.dot(reference);
  let u = reference.clone().subtract(n, nDotRef);
  if (u.lengthSq() <= eps) {
    reference = new Vector3(0, 0, 1);
    const nDotRef2 = n.dot(reference);
    u = reference.clone().subtract(n, nDotRef2);
  }
  u.normalize();
  const v = n.cross(u);
  return { n, u, v };
}

export class FlipperTagComponent {}

export class PlanarConstraintSystem3D {
  runInPause = false;

  constructor(planeNormal = DEFAULT_PLANE_NORMAL, planeOffset = 0.0) {
    this.basis = buildPlaneBasis(planeNormal);
    this.planeOffset = planeOffset;
    this._axisQuat = new Quaternion();
  }

  _projectPointToPlane(point) {
    const signedDistance = point.dot(this.basis.n) - this.planeOffset;
    point.subtract(this.basis.n, signedDistance);
  }

  _projectVectorToPlane(vector) {
    const normalComponent = vector.dot(this.basis.n);
    vector.subtract(this.basis.n, normalComponent);
  }

  _projectAngularVelocityToNormal(vector) {
    const scalar = vector.dot(this.basis.n);
    vector.x = this.basis.n.x * scalar;
    vector.y = this.basis.n.y * scalar;
    vector.z = this.basis.n.z * scalar;
  }

  _clampOrientationToPlaneAxis(quaternion) {
    const rotatedU = quaternion.transformVector(this.basis.u);
    this._projectVectorToPlane(rotatedU);

    if (rotatedU.lengthSq() <= 1e-12) {
      quaternion.set(this._axisQuat.setFromAxisAngle(this.basis.n, 0.0));
      return;
    }

    rotatedU.normalize();
    const angle = Math.atan2(rotatedU.dot(this.basis.v), rotatedU.dot(this.basis.u));
    quaternion.set(this._axisQuat.setFromAxisAngle(this.basis.n, angle));
  }

  update(world, _dt_unused) {
    for (const entityId of world.query([PositionComponent])) {
      const pos = world.getComponent(entityId, PositionComponent).pos;
      this._projectPointToPlane(pos);
    }

    for (const entityId of world.query([PrevFinalPosComponent])) {
      const prevPos = world.getComponent(entityId, PrevFinalPosComponent).pos;
      this._projectPointToPlane(prevPos);
    }

    for (const entityId of world.query([VelocityComponent])) {
      const vel = world.getComponent(entityId, VelocityComponent).vel;
      this._projectVectorToPlane(vel);
    }

    for (const entityId of world.query([AngularVelocityComponent])) {
      const omega = world.getComponent(entityId, AngularVelocityComponent).omega;
      this._projectAngularVelocityToNormal(omega);
    }

    for (const entityId of world.query([OrientationComponent])) {
      const quat = world.getComponent(entityId, OrientationComponent).quaternion;
      this._clampOrientationToPlaneAxis(quat);
    }

    for (const entityId of world.query([PrevFinalOrientationComponent])) {
      const prevQuat = world.getComponent(entityId, PrevFinalOrientationComponent).quaternion;
      this._clampOrientationToPlaneAxis(prevQuat);
    }
  }
}

export class FlipperStateComponent {
  constructor(length, restAngle, maxRotation, angularVelocity, planeNormal = DEFAULT_PLANE_NORMAL) {
    this.length = length;
    this.restAngle = restAngle;
    this.maxRotation = Math.abs(maxRotation);
    this.sign = Math.sign(maxRotation);
    this.angularVelocity = angularVelocity;
    this.planeNormal = planeNormal.clone().normalize();

    this.rotation = 0.0;
    this.currentAngularVelocity = 0.0;
    this.pressed = false;
  }
}

export class FlipperTipComponent {
  constructor(flipperEntityId) {
    this.flipperEntityId = flipperEntityId;
  }
}

export class BorderComponent {
  constructor(points = [], planeNormal = DEFAULT_PLANE_NORMAL) {
    this.points = points.map((p) => p.clone());
    this.planeNormal = planeNormal.clone().normalize();
  }
}

export class ScoredTagComponent {}

export class ScoreComponent {
  constructor(score = 0) {
    this.value = score;
  }
}

export class PauseStateComponent {
  constructor(paused = true) {
    this.paused = paused;
  }
}

export class CircleCamTag {}

export class OverlayRadiusComponent {
  constructor(radius = 0.0, mu = null) {
    this.radius = Number.isFinite(radius) ? Math.max(0.0, radius) : 0.0;
    this.mu = Number.isFinite(mu) ? Math.max(0.0, mu) : null;
  }
}

export class CircleSectorComponent {
  constructor(radius = 0.0, startAngle = 0.0, endAngle = 0.0, cw = false, mu = null) {
    this.radius = Number.isFinite(radius) ? Math.max(0.0, radius) : 0.0;
    this.startAngle = Number.isFinite(startAngle) ? startAngle : 0.0;
    this.endAngle = Number.isFinite(endAngle) ? endAngle : 0.0;
    this.cw = cw === true;
    this.mu = Number.isFinite(mu) ? Math.max(0.0, mu) : null;
  }
}

export class CircleSectorsComponent {
  constructor(sectors = []) {
    this.sectors = Array.isArray(sectors)
      ? sectors
        .filter((sector) => sector && Number.isFinite(sector.radius) && sector.radius > 1e-9)
        .map((sector) => ({
          radius: Math.max(0.0, Number(sector.radius)),
          startAngle: Number.isFinite(sector.startAngle) ? sector.startAngle : 0.0,
          endAngle: Number.isFinite(sector.endAngle) ? sector.endAngle : 0.0,
          cw: sector.cw === true,
          mu: Number.isFinite(sector.mu) ? Math.max(0.0, Number(sector.mu)) : null
        }))
      : [];
  }
}

function entityPlaneNormal(world, entityId) {
  const linkNormal = world.getComponent(entityId, CableLinkComponent)?.cablePlaneNormal;
  if (linkNormal) {
    return normalizePlaneNormal(linkNormal);
  }
  const flipperNormal = world.getComponent(entityId, FlipperStateComponent)?.planeNormal;
  if (flipperNormal) {
    return normalizePlaneNormal(flipperNormal);
  }
  return DEFAULT_PLANE_NORMAL.clone();
}

function angleOnPlane(point, center, planeNormal) {
  const activeBasis = buildPlaneBasis(planeNormal ?? DEFAULT_PLANE_NORMAL);
  const rel = point.clone().subtract(center);
  return Math.atan2(rel.dot(activeBasis.v), rel.dot(activeBasis.u));
}

function angleForDirection(direction, planeNormal) {
  const basis = buildPlaneBasis(planeNormal ?? DEFAULT_PLANE_NORMAL);
  return Math.atan2(direction.dot(basis.v), direction.dot(basis.u));
}

function offsetFromAngle(radius, angle, planeNormal) {
  const basis = buildPlaneBasis(planeNormal ?? DEFAULT_PLANE_NORMAL);
  return basis.u.clone().scale(Math.cos(angle) * radius).add(basis.v, Math.sin(angle) * radius);
}

function _normalizeAngle(angle) {
  if (!Number.isFinite(angle)) {
    return 0.0;
  }
  let value = angle;
  const twoPi = 2.0 * Math.PI;
  while (value <= -Math.PI) value += twoPi;
  while (value > Math.PI) value -= twoPi;
  return value;
}

function _ccwDiff(fromAngle, toAngle) {
  const twoPi = 2.0 * Math.PI;
  let diff = _normalizeAngle(toAngle - fromAngle);
  if (diff < 0.0) {
    diff += twoPi;
  }
  return diff;
}

function _isAngleInSector(angle, startAngle, endAngle, cw) {
  const a = _normalizeAngle(angle);
  const s = _normalizeAngle(startAngle);
  const e = _normalizeAngle(endAngle);
  if (cw) {
    return _isAngleInSector(-a, -s, -e, false);
  }
  const span = _ccwDiff(s, e);
  const rel = _ccwDiff(s, a);
  return rel <= span + 1e-9;
}

export function getRawCollisionRadius(world, entityId) {
  const radius = world.getComponent(entityId, RadiusComponent)?.radius;
  return Number.isFinite(radius) ? Math.max(0.0, radius) : 0.0;
}

function _frictionMu(world, entityId) {
  const mu = world.getComponent(entityId, CoefficientOfFrictionComponent)?.mu;
  return Number.isFinite(mu) ? Math.max(0.0, mu) : 0.0;
}

function _getBaseCollisionRadius(world, entityId) {
  let radius = getRawCollisionRadius(world, entityId);
  const overlayComp = world.getComponent(entityId, OverlayRadiusComponent);
  if (overlayComp && Number.isFinite(overlayComp.radius)) {
    radius = Math.max(radius, overlayComp.radius);
  }
  return radius;
}

function _getBaseCollisionSupport(world, entityId) {
  const rawRadius = getRawCollisionRadius(world, entityId);
  const baseMu = _frictionMu(world, entityId);
  const overlayComp = world.getComponent(entityId, OverlayRadiusComponent);
  if (overlayComp && Number.isFinite(overlayComp.radius) && overlayComp.radius > rawRadius + 1e-9) {
    const overlayMu = Number.isFinite(overlayComp.mu) ? Math.max(0.0, overlayComp.mu) : baseMu;
    return { radius: overlayComp.radius, mu: overlayMu };
  }
  return { radius: rawRadius, mu: baseMu };
}

export function getMaxCollisionRadius(world, entityId) {
  let radius = _getBaseCollisionRadius(world, entityId);
  const sectorsComp = world.getComponent(entityId, CircleSectorsComponent);
  if (sectorsComp && Array.isArray(sectorsComp.sectors)) {
    for (const sector of sectorsComp.sectors) {
      if (Number.isFinite(sector?.radius)) {
        radius = Math.max(radius, sector.radius);
      }
    }
  } else {
    const sectorComp = world.getComponent(entityId, CircleSectorComponent);
    if (sectorComp && Number.isFinite(sectorComp.radius)) {
      radius = Math.max(radius, sectorComp.radius);
    }
  }
  return radius;
}

function _normalizedDirection(vec, planeNormal, fallback = null) {
  const normal = planeNormal ?? DEFAULT_PLANE_NORMAL;
  const projected = vec ? vec.clone() : (fallback ? fallback.clone() : new Vector3(1.0, 0.0, 0.0));
  projectVectorToPlane(projected, normal);
  if (projected.lengthSq() <= 1e-12) {
    if (fallback) {
      const fallbackProjected = fallback.clone();
      projectVectorToPlane(fallbackProjected, normal);
      if (fallbackProjected.lengthSq() > 1e-12) {
        return fallbackProjected.normalize();
      }
    }
    const basis = buildPlaneBasis(normal);
    return basis.u.clone();
  }
  return projected.normalize();
}

function _entitySectorList(world, entityId) {
  const sectorsComp = world.getComponent(entityId, CircleSectorsComponent);
  if (sectorsComp && Array.isArray(sectorsComp.sectors) && sectorsComp.sectors.length > 0) {
    return sectorsComp.sectors;
  }
  const single = world.getComponent(entityId, CircleSectorComponent);
  if (single && Number.isFinite(single.radius) && single.radius > 1e-9) {
    return [single];
  }
  return [];
}

function _compositeSupportToward(world, entityId, directionTowardOther) {
  const center = world.getComponent(entityId, PositionComponent)?.pos;
  if (!center) {
    return null;
  }
  const planeNormal = entityPlaneNormal(world, entityId);
  const dir = _normalizedDirection(directionTowardOther, planeNormal);
  const baseSupport = _getBaseCollisionSupport(world, entityId);
  const baseRadius = baseSupport.radius;
  let projection = baseRadius;
  let offset = dir.clone().scale(baseRadius);
  let frictionMu = baseSupport.mu;
  const sectorList = _entitySectorList(world, entityId);
  const angle = angleForDirection(dir, planeNormal);
  for (const sector of sectorList) {
    if (!sector || !Number.isFinite(sector.radius) || !(sector.radius > baseRadius + 1e-9)) {
      continue;
    }
    if (_isAngleInSector(angle, sector.startAngle, sector.endAngle, sector.cw === true)) {
      if (sector.radius > projection + 1e-9) {
        projection = sector.radius;
        offset = dir.clone().scale(projection);
        frictionMu = Number.isFinite(sector.mu) ? Math.max(0.0, sector.mu) : baseSupport.mu;
      }
    }

    const cornerAngles = [sector.startAngle, sector.endAngle];
    for (const cornerAngle of cornerAngles) {
      const cornerOffset = offsetFromAngle(sector.radius, cornerAngle, planeNormal);
      const cornerProjection = cornerOffset.dot(dir);
      if (cornerProjection > projection + 1e-9) {
        projection = cornerProjection;
        offset = cornerOffset;
        frictionMu = Number.isFinite(sector.mu) ? Math.max(0.0, sector.mu) : baseSupport.mu;
      }
    }
  }

  return {
    center,
    baseRadius,
    projection,
    offset,
    friction: frictionMu,
    planeNormal
  };
}

function _collisionSupportToward(world, entityId, directionTowardOther, useSectorSupports) {
  if (useSectorSupports) {
    return _compositeSupportToward(world, entityId, directionTowardOther);
  }
  const center = world.getComponent(entityId, PositionComponent)?.pos;
  if (!center) {
    return null;
  }
  const planeNormal = entityPlaneNormal(world, entityId);
  const dir = _normalizedDirection(directionTowardOther, planeNormal);
  const baseSupport = _getBaseCollisionSupport(world, entityId);
  return {
    center,
    baseRadius: baseSupport.radius,
    projection: baseSupport.radius,
    offset: dir.clone().scale(baseSupport.radius),
    friction: baseSupport.mu,
    planeNormal
  };
}

function _pathLinkEntity(world, path, linkIndex) {
  if (!Array.isArray(path.jointEntities) || path.jointEntities.length < 1) {
    return null;
  }
  if (linkIndex === 0) {
    const joint = world.getComponent(path.jointEntities[0], CableJointComponent);
    return joint ? joint.entityA : null;
  }
  if (linkIndex === path.linkTypes.length - 1) {
    const joint = world.getComponent(path.jointEntities[path.jointEntities.length - 1], CableJointComponent);
    return joint ? joint.entityB : null;
  }
  const leftJoint = world.getComponent(path.jointEntities[linkIndex - 1], CableJointComponent);
  return leftJoint ? leftJoint.entityB : null;
}

function _pathLinkStartAttachment(world, path, linkIndex) {
  if (!Array.isArray(path.jointEntities) || path.jointEntities.length < 1) {
    return null;
  }
  if (linkIndex === 0) {
    return world.getComponent(path.jointEntities[0], CableJointComponent)?.attachmentPointA_world?.clone() ?? null;
  }
  if (linkIndex === path.linkTypes.length - 1) {
    return world.getComponent(path.jointEntities[path.jointEntities.length - 1], CableJointComponent)?.attachmentPointB_world?.clone() ?? null;
  }
  return world.getComponent(path.jointEntities[linkIndex - 1], CableJointComponent)?.attachmentPointB_world?.clone() ?? null;
}

function _decomposeStoredWrap(storedLength, firstLayerRadius, layerStep) {
  const stored = Math.max(0.0, storedLength ?? 0.0);
  if (!(firstLayerRadius > 1e-9) || !(layerStep > 1e-9)) {
    return null;
  }
  if (!(stored > 1e-9)) {
    return { fullLayers: 0, partialLength: 0, partialRadius: firstLayerRadius };
  }
  let remaining = stored;
  let fullLayers = 0;
  const maxLayers = 2048;
  while (fullLayers < maxLayers) {
    const layerRadius = firstLayerRadius + fullLayers * layerStep;
    const circumference = 2.0 * Math.PI * layerRadius;
    if (remaining + 1e-9 >= circumference) {
      remaining -= circumference;
      if (remaining < 0.0) remaining = 0.0;
      fullLayers += 1;
      continue;
    }
    return { fullLayers, partialLength: remaining, partialRadius: layerRadius };
  }
  return {
    fullLayers,
    partialLength: 0.0,
    partialRadius: firstLayerRadius + fullLayers * layerStep
  };
}

const LAYER_RADIUS_RAMP_ANGLE = (2.0 * Math.PI) / 5.0;

function _closingOverlayBlend(span) {
  const spanValue = Number.isFinite(span) ? Math.max(0.0, span) : 0.0;
  const remaining = Math.max(0.0, (2.0 * Math.PI) - spanValue);
  if (remaining >= LAYER_RADIUS_RAMP_ANGLE) {
    return 0.0;
  }
  return 1.0 - (remaining / LAYER_RADIUS_RAMP_ANGLE);
}

function _smoothedSectorRadius(rawRadius, decomposition, halfWidth, span) {
  if (!(halfWidth > 1e-9)) {
    return 0.0;
  }
  const fullLayers = Math.max(0, decomposition?.fullLayers ?? 0);
  const layerWidth = 2.0 * halfWidth;
  const rampStartRadius = rawRadius + layerWidth * fullLayers;
  const rampTargetRadius = rawRadius + layerWidth * (fullLayers + 1);
  if (!(rampTargetRadius > 1e-9)) {
    return 0.0;
  }
  const spanValue = Number.isFinite(span) ? Math.max(0.0, span) : 0.0;
  const rampAlpha = Math.max(0.0, Math.min(1.0, spanValue / LAYER_RADIUS_RAMP_ANGLE));
  return rampStartRadius + (rampTargetRadius - rampStartRadius) * rampAlpha;
}

export class OverlayRadiusAndCircleSectorSystem {
  runInPause = false;

  update(world, _dt_unused) {
    if (!layeringEnabled(world)) {
      for (const entityId of world.query([OverlayRadiusComponent])) {
        world.removeComponent(entityId, OverlayRadiusComponent);
      }
      for (const entityId of world.query([CircleSectorComponent])) {
        world.removeComponent(entityId, CircleSectorComponent);
      }
      for (const entityId of world.query([CircleSectorsComponent])) {
        world.removeComponent(entityId, CircleSectorsComponent);
      }
      for (const entityId of world.query([CircleCamTag])) {
        world.removeComponent(entityId, CircleCamTag);
      }
      return;
    }

    const overlayEnabled = world.getResource('layeringCollisionOverlayRadius') !== false;
    const overlayRampEnabled = world.getResource('layeringCollisionOverlayRamp') !== false;
    const sectorEnabled = world.getResource('layeringCollisionCircleSectors') !== false;
    const overlayByEntity = new Map();
    const sectorListByEntity = new Map();

    for (const pathId of world.query([CablePathComponent])) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (!path || !Array.isArray(path.linkTypes) || !Array.isArray(path.stored) || !Array.isArray(path.cw)) {
        continue;
      }
      const halfWidth = Number.isFinite(path.cableHalfWidth) ? Math.max(0.0, path.cableHalfWidth) : 0.0;
      if (!(halfWidth > 1e-9)) {
        continue;
      }
      const layerStep = 2.0 * halfWidth;

      for (let linkIndex = 0; linkIndex < path.linkTypes.length; linkIndex++) {
        const linkType = path.linkTypes[linkIndex];
        if (!(linkType === 'rolling' || linkType === 'hybrid' || linkType === 'hybrid-attachment')) {
          continue;
        }
        const stored = Math.max(0.0, path.stored[linkIndex] ?? 0.0);
        const entityId = _pathLinkEntity(world, path, linkIndex);
        if ((stored <= 1e-9 && linkType !== 'hybrid-attachment') || entityId === null || entityId === undefined) {
          continue;
        }
        const center = world.getComponent(entityId, PositionComponent)?.pos;
        const rawRadius = getRawCollisionRadius(world, entityId);
        if (!center || !(rawRadius > 1e-9)) {
          continue;
        }

        const firstLayerRadius = rawRadius + halfWidth;
        const decomposition = _decomposeStoredWrap(stored, firstLayerRadius, layerStep);
        if (!decomposition || (decomposition.fullLayers === 0 && decomposition.partialLength < 1e-9 && linkType !== 'hybrid-attachment')) {
          continue;
        }

        const baseOverlayRadius = rawRadius + layerStep * decomposition.fullLayers;
        let overlayEnvelopeRadius = baseOverlayRadius;
        if (overlayRampEnabled && decomposition.partialLength > 1e-9 && decomposition.partialRadius > 1e-9) {
          const span = decomposition.partialLength / decomposition.partialRadius;
          const closingBlend = _closingOverlayBlend(span);
          if (closingBlend > 1e-9) {
            overlayEnvelopeRadius = baseOverlayRadius + layerStep * closingBlend;
          }
        }

        if (overlayEnabled && overlayEnvelopeRadius > rawRadius + 1e-9) {
          const prev = overlayByEntity.get(entityId);
          const mu = _frictionMu(world, entityId);
          if (prev === undefined || overlayEnvelopeRadius > prev.radius + 1e-9) {
            overlayByEntity.set(entityId, { radius: overlayEnvelopeRadius, mu });
          }
        }

        if (sectorEnabled) {
          const startPoint = _pathLinkStartAttachment(world, path, linkIndex);
          if (!startPoint) {
            continue;
          }
          const planeNormal = entityPlaneNormal(world, entityId);
          const rel = startPoint.clone().subtract(center);
          if (rel.lengthSq() <= 1e-12) {
            continue;
          }
          const cw = Boolean(path.cw[linkIndex]);
          const knotSpan = Math.PI / 30.0;
          const startAngle = linkType === 'hybrid'
            ? angleOnPlane(startPoint, center, planeNormal) + (cw ? 1.0 : -1.0) * knotSpan * (rawRadius / decomposition.partialRadius)
            : angleOnPlane(startPoint, center, planeNormal);
          const spanBase = decomposition.partialLength / decomposition.partialRadius;
          const span = (decomposition.fullLayers === 0 && linkType === 'hybrid-attachment')
            ? Math.max(spanBase, knotSpan)
            : spanBase;
          const sectorRadius = (decomposition.fullLayers === 0 && linkType === 'hybrid-attachment')
            ? rawRadius + 2.0 * halfWidth
            : _smoothedSectorRadius(rawRadius, decomposition, halfWidth, span);
          if (!(sectorRadius > rawRadius + 1e-9)) {
            continue;
          }
          const endAngle = cw ? (startAngle - span) : (startAngle + span);
          const list = sectorListByEntity.get(entityId) ?? [];
          list.push({
            radius: sectorRadius,
            startAngle,
            endAngle,
            cw,
            mu: _frictionMu(world, entityId)
          });
          sectorListByEntity.set(entityId, list);
        }
      }
    }

    for (const [entityId, overlay] of overlayByEntity.entries()) {
      if (world.hasComponent(entityId, OverlayRadiusComponent)) {
        const comp = world.getComponent(entityId, OverlayRadiusComponent);
        comp.radius = overlay.radius;
        comp.mu = overlay.mu;
      } else {
        world.addComponent(entityId, new OverlayRadiusComponent(overlay.radius, overlay.mu));
      }
    }
    for (const entityId of world.query([OverlayRadiusComponent])) {
      if (!overlayByEntity.has(entityId)) {
        world.removeComponent(entityId, OverlayRadiusComponent);
      }
    }

    for (const [entityId, sectors] of sectorListByEntity.entries()) {
      const validSectors = Array.isArray(sectors)
        ? sectors.filter((sector) => sector && Number.isFinite(sector.radius) && sector.radius > 1e-9)
        : [];
      if (validSectors.length < 1) {
        continue;
      }
      const sector = validSectors.reduce((best, candidate) => (!best || candidate.radius > best.radius ? candidate : best), null);
      if (world.hasComponent(entityId, CircleSectorComponent)) {
        const comp = world.getComponent(entityId, CircleSectorComponent);
        comp.radius = sector.radius;
        comp.startAngle = sector.startAngle;
        comp.endAngle = sector.endAngle;
        comp.cw = sector.cw;
        comp.mu = sector.mu;
      } else {
        world.addComponent(entityId, new CircleSectorComponent(sector.radius, sector.startAngle, sector.endAngle, sector.cw, sector.mu));
      }

      if (world.hasComponent(entityId, CircleSectorsComponent)) {
        const comp = world.getComponent(entityId, CircleSectorsComponent);
        comp.sectors = validSectors.map((entry) => ({
          radius: entry.radius,
          startAngle: entry.startAngle,
          endAngle: entry.endAngle,
          cw: entry.cw === true,
          mu: Number.isFinite(entry.mu) ? Math.max(0.0, entry.mu) : null
        }));
      } else {
        world.addComponent(entityId, new CircleSectorsComponent(validSectors));
      }
    }
    for (const entityId of world.query([CircleSectorComponent])) {
      if (!sectorListByEntity.has(entityId)) {
        world.removeComponent(entityId, CircleSectorComponent);
      }
    }
    for (const entityId of world.query([CircleSectorsComponent])) {
      if (!sectorListByEntity.has(entityId)) {
        world.removeComponent(entityId, CircleSectorsComponent);
      }
    }

    const camEntities = new Set([...overlayByEntity.keys(), ...sectorListByEntity.keys()]);
    for (const entityId of camEntities) {
      if (!world.hasComponent(entityId, CircleCamTag)) {
        world.addComponent(entityId, new CircleCamTag());
      }
    }
    for (const entityId of world.query([CircleCamTag])) {
      if (!camEntities.has(entityId)) {
        world.removeComponent(entityId, CircleCamTag);
      }
    }
  }
}

export class InputSystem {
  runInPause = true;

  constructor(canvas, world, projectPointerToSim = null, grabSpringParams = { restLength: 0.1, springConstant: () => 10.0 }) {
    this.canvas = canvas;
    this.world = world;
    this.projectPointerToSim = projectPointerToSim;
    this.grabSpringParams = grabSpringParams;

    this.clicks = [];
    this.releases = [];
    this.eventLog = [];
    this.frame = 0;
    this.grabSpring = null;

    this.canvas.setAttribute('tabindex', '0');
    this.canvas.style.outline = 'none';
    this.canvas.focus();

    this.canvas.addEventListener('contextmenu', (event) => {
      event.preventDefault();
    });
    document.addEventListener('pointerdown', this.handlePointerDown.bind(this));
    document.addEventListener('pointerup', this.handlePointerUp.bind(this));
    this.canvas.addEventListener('pointermove', this.handlePointerMove.bind(this));
    window.addEventListener('keydown', this.handleKeydown.bind(this));
    window.addEventListener('keyup', this.handleKeyup.bind(this));
  }

  _shouldHandlePointerEvent(event) {
    if (this.grabSpring) {
      return true;
    }
    const target = event.target;
    return target === this.canvas || this.canvas.contains(target);
  }

  reset() {
    this.clicks.length = 0;
    this.releases.length = 0;
    this.eventLog.length = 0;
    this.frame = 0;
    this.grabSpring = null;
  }

  dumpDebugScenario() {
    const resStr = JSON.stringify(this.world.resources);
    const logEntries = this.eventLog.map((frame) => {
      const clicksStr = frame.clicks.map((p) => `new Vector3(${p.x}, ${p.y}, ${p.z})`).join(', ');
      const releasesStr = frame.releases.map((p) => `new Vector3(${p.x}, ${p.y}, ${p.z})`).join(', ');
      return `{ \"frame\": ${frame.frame}, \"clicks\": [${clicksStr}], \"releases\": [${releasesStr}] }`;
    }).join(', ');
    console.log('DEBUG_SCENARIO_DUMP', `{ \"resources\": ${resStr}, \"inputLog\": [${logEntries}] }`);
  }

  _pointerToSim(event) {
    if (typeof this.projectPointerToSim === 'function') {
      const projected = this.projectPointerToSim(event.clientX, event.clientY);
      if (projected) {
        return projected;
      }
    }

    const rect = this.canvas.getBoundingClientRect();
    const simHeight = this.world.getResource('simHeight') || 1.7;
    const scale = this.canvas.height / simHeight;
    const simX = (event.clientX - rect.left) / scale;
    const simY = (this.canvas.height - (event.clientY - rect.top)) / scale;
    return new Vector3(simX, simY, 0);
  }

  _queueFlipperEdgeInput(isLeft, isPress) {
    const flipperEntities = this.world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent]);
    if (flipperEntities.length < 2) {
      return;
    }

    const firstPos = this.world.getComponent(flipperEntities[0], PositionComponent).pos;
    const secondPos = this.world.getComponent(flipperEntities[1], PositionComponent).pos;

    let targetPos = null;
    if (firstPos.x < secondPos.x) {
      targetPos = isLeft ? firstPos : secondPos;
    } else {
      targetPos = isLeft ? secondPos : firstPos;
    }

    if (isPress) {
      this.clicks.push(targetPos.clone());
    } else {
      this.releases.push(targetPos.clone());
    }
  }

  handleKeyup(event) {
    if (event.key === 'ArrowLeft' || event.key === 'ArrowRight') {
      event.preventDefault();
    }
    if (event.key === 'ArrowLeft') {
      this._queueFlipperEdgeInput(true, false);
    }
    if (event.key === 'ArrowRight') {
      this._queueFlipperEdgeInput(false, false);
    }
  }

  handleKeydown(event) {
    if (event.code === 'Space') {
      this.dumpDebugScenario();
    }
    if (event.key === 'ArrowLeft' || event.key === 'ArrowRight') {
      event.preventDefault();
    }
    if (event.key === 'ArrowLeft') {
      this._queueFlipperEdgeInput(true, true);
    }
    if (event.key === 'ArrowRight') {
      this._queueFlipperEdgeInput(false, true);
    }
  }

  handlePointerDown(event) {
    const button = event.button;
    if (button !== undefined && button !== 0 && button !== 2) {
      return;
    }
    if (button === 2 && event.shiftKey) {
      return;
    }
    if (!this._shouldHandlePointerEvent(event)) {
      return;
    }
    event.preventDefault();
    const clickPos = this._pointerToSim(event);
    if (!clickPos) {
      return;
    }

    if (this.canvas.setPointerCapture) {
      try {
        this.canvas.setPointerCapture(event.pointerId);
      } catch (_err) {
        // Ignore capture failures.
      }
    }

    const cmOnScreen = 0.5;
    const dpi = 96;
    const pixelsPerCm = dpi / 2.54;
    const extraPixels = cmOnScreen * pixelsPerCm;
    const scale = this.world.getResource('cScale') || (this.canvas.height / (this.world.getResource('simHeight') || 1.7));
    const extraClickableRadius = extraPixels / Math.max(scale, 1e-9);

    let closestBall = null;
    let closestDistSq = Infinity;

    for (const ballId of this.world.query([BallTagComponent, PositionComponent, RadiusComponent])) {
      const pos = this.world.getComponent(ballId, PositionComponent).pos;
      const radius = getMaxCollisionRadius(this.world, ballId) + extraClickableRadius;
      const distSq = clickPos.clone().subtract(pos).lengthSq();
      if (distSq <= radius * radius && distSq < closestDistSq) {
        closestBall = ballId;
        closestDistSq = distSq;
      }
    }

    if (closestBall !== null) {
      const ptrE = this.world.createEntity();
      this.world.addComponent(ptrE, new PositionComponent(clickPos.x, clickPos.y, clickPos.z));
      this.world.addComponent(ptrE, new CableLinkComponent(clickPos.x, clickPos.y, clickPos.z));

      const ballPos = this.world.getComponent(closestBall, PositionComponent).pos.clone();
      const ballMass = this.world.getComponent(closestBall, MassComponent).mass;
      const ptrPos = clickPos.clone();

      const jointE = this.world.createEntity();
      this.world.addComponent(
        jointE,
        new CableJointComponent(
          closestBall,
          ptrE,
          this.grabSpringParams.restLength,
          ballPos,
          ptrPos
        )
      );

      const pathE = this.world.createEntity();
      this.world.addComponent(
        pathE,
        new CablePathComponent(
          this.world,
          [jointE],
          ['attachment', 'attachment'],
          [true],
          this.grabSpringParams.springConstant(ballMass)
        )
      );

      this.grabSpring = { ptrE, jointE, pathE, ballE: closestBall };
      this.world.setResource('grabbedBall', closestBall);

      const pauseState = this.world.getResource('pauseState');
      if (pauseState) {
        pauseState.paused = false;
      }
      const pauseBtn = document.getElementById('pauseBtn');
      if (pauseBtn) {
        pauseBtn.textContent = 'Pause';
      }

      return;
    }

    this.clicks.push(clickPos);
  }

  handlePointerUp(event) {
    const button = event.button;
    if (!this.grabSpring && button !== undefined && button !== 0 && button !== 2) {
      return;
    }
    if (!this.grabSpring && button === 2 && event.shiftKey) {
      return;
    }
    if (!this._shouldHandlePointerEvent(event)) {
      return;
    }
    event.preventDefault();

    if (this.canvas.releasePointerCapture) {
      try {
        this.canvas.releasePointerCapture(event.pointerId);
      } catch (_err) {
        // Ignore release failures.
      }
    }

    const releasePos = this._pointerToSim(event);
    if (!releasePos) {
      return;
    }

    if (this.grabSpring) {
      const { ptrE, jointE, pathE, ballE } = this.grabSpring;
      const velComp = this.world.getComponent(ballE, VelocityComponent);
      const posComp = this.world.getComponent(ballE, PositionComponent);
      const prevFinalPosComp = this.world.getComponent(ballE, PrevFinalPosComponent);

      const dt = this.world.getResource('dt');
      if (velComp && posComp && prevFinalPosComp && dt > 1e-9) {
        velComp.vel.set(posComp.pos.clone().subtract(prevFinalPosComp.pos).scale(1.0 / dt));
      }

      this.world.destroyEntity(pathE);
      this.world.destroyEntity(jointE);
      this.world.destroyEntity(ptrE);
      this.grabSpring = null;
      this.world.setResource('grabbedBall', null);
      return;
    }

    this.releases.push(releasePos);
  }

  handlePointerMove(event) {
    if (!this.grabSpring) {
      return;
    }
    event.preventDefault();

    const simPos = this._pointerToSim(event);
    if (!simPos) {
      return;
    }

    const ptrPos = this.world.getComponent(this.grabSpring.ptrE, PositionComponent)?.pos;
    if (ptrPos) {
      ptrPos.set(simPos);
    }
  }

  update(world, _dt_unused) {
    const clicksFrame = this.clicks.slice();
    const releasesFrame = this.releases.slice();

    if (clicksFrame.length > 0 || releasesFrame.length > 0) {
      this.eventLog.push({ frame: this.frame, clicks: clicksFrame, releases: releasesFrame });
    }
    this.frame += 1;

    if (clicksFrame.length > 0) {
      const clickPos = this.clicks.shift();
      const flipperEntities = world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent]);

      const borderEntities = world.query([BorderComponent]);
      if (borderEntities.length > 0) {
        const border = world.getComponent(borderEntities[0], BorderComponent);
        const borderPoints = border.points;
        const planeNormal = border.planeNormal || DEFAULT_PLANE_NORMAL;

        const rightClick =
          rightOfPlane(clickPos, borderPoints[0], borderPoints[1], planeNormal) &&
          rightOfPlane(clickPos, borderPoints[1], borderPoints[2], planeNormal);

        const leftClick =
          rightOfPlane(clickPos, borderPoints[5], borderPoints[6], planeNormal) &&
          rightOfPlane(clickPos, borderPoints[6], borderPoints[7], planeNormal);

        if (rightClick) {
          const flippers = world.query([FlipperStateComponent, PositionComponent]);
          const pos0 = world.getComponent(flippers[0], PositionComponent).pos;
          const pos1 = world.getComponent(flippers[1], PositionComponent).pos;
          if (pos0.x > pos1.x) {
            world.getComponent(flippers[0], FlipperStateComponent).pressed = true;
          } else {
            world.getComponent(flippers[1], FlipperStateComponent).pressed = true;
          }
        } else if (leftClick) {
          const flippers = world.query([FlipperStateComponent, PositionComponent]);
          const pos0 = world.getComponent(flippers[0], PositionComponent).pos;
          const pos1 = world.getComponent(flippers[1], PositionComponent).pos;
          if (pos0.x < pos1.x) {
            world.getComponent(flippers[0], FlipperStateComponent).pressed = true;
          } else {
            world.getComponent(flippers[1], FlipperStateComponent).pressed = true;
          }
        } else {
          for (const id of flipperEntities) {
            const pos = world.getComponent(id, PositionComponent).pos;
            const state = world.getComponent(id, FlipperStateComponent);
            if (clickPos.clone().subtract(pos).lengthSq() < state.length ** 2) {
              state.pressed = true;
            }
          }
        }
      }
    }

    if (this.releases.length > 0) {
      const releasePos = this.releases.shift();
      const flipperEntities = world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent]);

      let closestId = null;
      let minDistSq = Infinity;

      for (const id of flipperEntities) {
        const state = world.getComponent(id, FlipperStateComponent);
        if (!state.pressed) continue;

        const pos = world.getComponent(id, PositionComponent).pos;
        const d2 = releasePos.clone().subtract(pos).lengthSq();
        if (d2 < minDistSq) {
          minDistSq = d2;
          closestId = id;
        }
      }

      if (closestId !== null) {
        world.getComponent(closestId, FlipperStateComponent).pressed = false;
      }
    }
  }
}

export class InputReplaySystem {
  runInPause = false;

  constructor(inputLog, inputSystem) {
    this.inputLog = inputLog;
    this.currentIndex = 0;
    this.frame = 0;
    this.inputSystem = inputSystem;
  }

  update(_world, _dt_unused) {
    if (this.inputLog.length > 0) {
      if (this.inputSystem.frame === this.inputLog[0].frame) {
        const frame = this.inputLog.shift();
        this.inputSystem.clicks = frame.clicks.slice();
        this.inputSystem.releases = frame.releases.slice();
      }
    }
  }

  reset() {
    this.inputLog = [];
    this.currentIndex = 0;
    this.frame = 0;
  }
}

export class ScoreSystem {
  runInPause = false;

  update(world, _dt_unused) {
    const scoreEntity = world.query([ScoreComponent])[0];
    const scoreComp = scoreEntity !== undefined ? world.getComponent(scoreEntity, ScoreComponent) : null;
    if (!scoreComp) return;

    const scoredEntities = world.query([ScoredTagComponent]);
    for (const scoredId of scoredEntities) {
      world.removeComponent(scoredId, ScoredTagComponent);
      scoreComp.value += 1;
    }
  }
}

export class ScoreDisplaySystem {
  runInPause = true;

  constructor(elementId) {
    this.scoreElement = document.getElementById(elementId);
  }

  update(world, _dt_unused) {
    const scoreEntity = world.query([ScoreComponent])[0];
    if (scoreEntity !== undefined && this.scoreElement) {
      const scoreComp = world.getComponent(scoreEntity, ScoreComponent);
      this.scoreElement.textContent = scoreComp.value.toString();
    }
  }
}

export class FlipperMotionSystem {
  runInPause = true;

  update(world, dt) {
    const flipperEntities = world.query([FlipperStateComponent]);
    for (const entityId of flipperEntities) {
      const state = world.getComponent(entityId, FlipperStateComponent);

      const prevRotation = state.rotation;
      if (state.pressed) {
        state.rotation = Math.min(state.rotation + dt * state.angularVelocity, state.maxRotation);
      } else {
        state.rotation = Math.max(state.rotation - dt * state.angularVelocity, 0.0);
      }

      state.currentAngularVelocity = dt > 1e-6
        ? state.sign * (state.rotation - prevRotation) / dt
        : 0.0;
    }
  }
}

export class FlipperTipLinkSystem {
  runInPause = false;

  update(world, _dt_unused) {
    for (const tipId of world.query([FlipperTipComponent, PositionComponent])) {
      const tipComp = world.getComponent(tipId, FlipperTipComponent);
      const flipId = tipComp.flipperEntityId;
      const pivotPos = world.getComponent(flipId, PositionComponent).pos;
      const state = world.getComponent(flipId, FlipperStateComponent);

      const angle = state.restAngle + state.sign * state.rotation;
      const dir = new Vector3(Math.cos(angle), Math.sin(angle), 0);
      const tipPos = pivotPos.clone().add(dir, state.length);

      world.getComponent(tipId, PositionComponent).pos.set(tipPos);
    }
  }
}

function _borderCollisionNormal(point, closestPoint, edgeStart, edgeEnd, planeNormal) {
  const ballToClosest = point.clone().subtract(closestPoint);
  const edgeVec = edgeEnd.clone().subtract(edgeStart);
  const normal = planeNormal.clone().cross(edgeVec).normalize();

  let collisionNormal;
  if (ballToClosest.lengthSq() < 1e-9) {
    collisionNormal = normal.clone();
  } else {
    collisionNormal = ballToClosest.clone().normalize();
  }

  if (ballToClosest.dot(normal) < 0) {
    collisionNormal = normal.clone();
  }
  return collisionNormal;
}

function _applyPlanarOrientationDelta(world, entityId, deltaAngle, planeNormal) {
  const orientation = world.getComponent(entityId, OrientationComponent)?.quaternion;
  if (!orientation || !Number.isFinite(deltaAngle) || Math.abs(deltaAngle) <= 1e-12) {
    return;
  }
  const axis = normalizePlaneNormal(planeNormal);
  const dq = new Quaternion().setFromAxisAngle(axis, deltaAngle);
  orientation.multiplyQuaternions(dq, orientation).normalize();
}

function _resolveRigidContactSingle(world, entityId, contactOffset, normal, penetration) {
  if (!(penetration > 0.0)) {
    return 0.0;
  }
  const posComp = world.getComponent(entityId, PositionComponent);
  if (!posComp?.pos) {
    return 0.0;
  }

  const massComp = world.getComponent(entityId, MassComponent);
  const invMass = massComp && massComp.mass > 0.0 ? 1.0 / massComp.mass : 0.0;
  const invInertia = world.getComponent(entityId, MomentOfInertiaComponent)?.invInertia ?? 0.0;
  const planeNormal = entityPlaneNormal(world, entityId);
  const r = contactOffset?.clone?.() ?? new Vector3(0.0, 0.0, 0.0);
  const rn = r.cross(normal).dot(planeNormal);
  const denom = invMass + invInertia * rn * rn;
  if (denom <= 1e-12) {
    return 0.0;
  }

  const lambda = penetration / denom;
  if (invMass > 0.0) {
    posComp.pos.add(normal, invMass * lambda);
  }
  if (invInertia > 0.0) {
    _applyPlanarOrientationDelta(world, entityId, invInertia * rn * lambda, planeNormal);
  }
  return lambda;
}

function _resolveRigidContactPair(world, entityA, offsetA, entityB, offsetB, normalAB, penetration) {
  if (!(penetration > 0.0)) {
    return 0.0;
  }
  const posAComp = world.getComponent(entityA, PositionComponent);
  const posBComp = world.getComponent(entityB, PositionComponent);
  if (!posAComp?.pos || !posBComp?.pos) {
    return 0.0;
  }

  const invMassA = (world.getComponent(entityA, MassComponent)?.mass > 0.0) ? (1.0 / world.getComponent(entityA, MassComponent).mass) : 0.0;
  const invMassB = (world.getComponent(entityB, MassComponent)?.mass > 0.0) ? (1.0 / world.getComponent(entityB, MassComponent).mass) : 0.0;
  const invInertiaA = world.getComponent(entityA, MomentOfInertiaComponent)?.invInertia ?? 0.0;
  const invInertiaB = world.getComponent(entityB, MomentOfInertiaComponent)?.invInertia ?? 0.0;
  const planeNormalA = entityPlaneNormal(world, entityA);
  const planeNormalB = entityPlaneNormal(world, entityB);
  const rA = offsetA?.clone?.() ?? new Vector3(0.0, 0.0, 0.0);
  const rB = offsetB?.clone?.() ?? new Vector3(0.0, 0.0, 0.0);
  const rnA = rA.cross(normalAB).dot(planeNormalA);
  const rnB = rB.cross(normalAB).dot(planeNormalB);
  const denom = invMassA + invMassB + invInertiaA * rnA * rnA + invInertiaB * rnB * rnB;
  if (denom <= 1e-12) {
    return 0.0;
  }

  const lambda = penetration / denom;
  if (invMassA > 0.0) {
    posAComp.pos.add(normalAB, -invMassA * lambda);
  }
  if (invMassB > 0.0) {
    posBComp.pos.add(normalAB, invMassB * lambda);
  }
  if (invInertiaA > 0.0) {
    _applyPlanarOrientationDelta(world, entityA, -invInertiaA * rnA * lambda, planeNormalA);
  }
  if (invInertiaB > 0.0) {
    _applyPlanarOrientationDelta(world, entityB, invInertiaB * rnB * lambda, planeNormalB);
  }
  return lambda;
}

function _flipperTipFromState(flipperPos, flipperState) {
  const angle = flipperState.restAngle + flipperState.sign * flipperState.rotation;
  return flipperPos.clone().add(new Vector3(Math.cos(angle), Math.sin(angle), 0), flipperState.length);
}

function _projectToPlane2(point, basisOrigin, basis) {
  const rel = point.clone().subtract(basisOrigin);
  return { x: rel.dot(basis.u), y: rel.dot(basis.v) };
}

function _segmentsProperlyIntersect(a0, a1, b0, b1, planeNormal, eps = 1e-9) {
  const basis = buildPlaneBasis(planeNormal ?? DEFAULT_PLANE_NORMAL);
  const a02 = _projectToPlane2(a0, a0, basis);
  const a12 = _projectToPlane2(a1, a0, basis);
  const b02 = _projectToPlane2(b0, a0, basis);
  const b12 = _projectToPlane2(b1, a0, basis);

  const rX = a12.x - a02.x;
  const rY = a12.y - a02.y;
  const sX = b12.x - b02.x;
  const sY = b12.y - b02.y;
  const rCrossS = (rX * sY) - (rY * sX);
  if (Math.abs(rCrossS) <= eps) {
    return false;
  }

  const qpX = b02.x - a02.x;
  const qpY = b02.y - a02.y;
  const t = ((qpX * sY) - (qpY * sX)) / rCrossS;
  const u = ((qpX * rY) - (qpY * rX)) / rCrossS;
  if (!Number.isFinite(t) || !Number.isFinite(u)) {
    return false;
  }

  const rLen = Math.hypot(rX, rY);
  const sLen = Math.hypot(sX, sY);
  const endpointLinearMargin = 1e-4;
  const tMargin = Math.min(0.49, endpointLinearMargin / Math.max(rLen, 1e-9));
  const uMargin = Math.min(0.49, endpointLinearMargin / Math.max(sLen, 1e-9));
  return t > tMargin && t < (1.0 - tMargin) && u > uMargin && u < (1.0 - uMargin);
}

function _segmentEndpointAtCurrentPose(world, entityId, attachmentPointWorld) {
  if (!attachmentPointWorld) {
    return null;
  }
  const currentPos = world.getComponent(entityId, PositionComponent)?.pos;
  if (!currentPos) {
    return attachmentPointWorld.clone();
  }
  const currentQuat = world.getComponent(entityId, OrientationComponent)?.quaternion ?? new Quaternion();
  const linkComp = world.getComponent(entityId, CableLinkComponent);
  const cachedPos = linkComp?.prevCableAttachmentTimePos ?? currentPos;
  const cachedQuat = linkComp?.prevCableAttachmentTimeOrientation ?? currentQuat;
  const local = cachedQuat.clone().conjugate().normalize().transformVector(attachmentPointWorld.clone().subtract(cachedPos));
  return currentPos.clone().add(currentQuat.transformVector(local));
}

function _collectCableJointSegments(world) {
  const segmentByJointId = new Map();
  for (const pathId of world.query([CablePathComponent])) {
    const path = world.getComponent(pathId, CablePathComponent);
    const halfWidth = Number.isFinite(path?.cableHalfWidth) ? Math.max(0.0, path.cableHalfWidth) : 0.0;
    if (!(halfWidth > 1e-9)) {
      continue;
    }
    for (const jointId of path?.jointEntities ?? []) {
      const joint = world.getComponent(jointId, CableJointComponent);
      if (!joint) {
        continue;
      }
      const segmentA = _segmentEndpointAtCurrentPose(world, joint.entityA, joint.attachmentPointA_world);
      const segmentB = _segmentEndpointAtCurrentPose(world, joint.entityB, joint.attachmentPointB_world);
      if (!segmentA || !segmentB) {
        continue;
      }
      const prev = segmentByJointId.get(jointId);
      if (!prev || halfWidth > prev.halfWidth) {
        segmentByJointId.set(jointId, {
          halfWidth,
          a: segmentA.clone(),
          b: segmentB.clone(),
          entityA: joint.entityA,
          entityB: joint.entityB,
          planeNormal: entityPlaneNormal(world, joint.entityA)
        });
      }
    }
  }
  return Array.from(segmentByJointId.values());
}

function _crossingCableHalfWidth(cableSegments, centerA, centerB, pairEntityA, pairEntityB, planeNormal) {
  if (!Array.isArray(cableSegments) || cableSegments.length === 0 || pairEntityA === pairEntityB) {
    return 0.0;
  }
  let maxHalfWidth = 0.0;
  for (const segment of cableSegments) {
    if (!(segment?.halfWidth > 1e-9)) {
      continue;
    }
    if (
      segment.entityA === pairEntityA ||
      segment.entityB === pairEntityA ||
      segment.entityA === pairEntityB ||
      segment.entityB === pairEntityB
    ) {
      continue;
    }
    if (_segmentsProperlyIntersect(centerA, centerB, segment.a, segment.b, planeNormal ?? segment.planeNormal ?? DEFAULT_PLANE_NORMAL)) {
      maxHalfWidth = Math.max(maxHalfWidth, segment.halfWidth);
    }
  }
  return maxHalfWidth;
}

export class PBDUnifiedContactManifoldSystem {
  runInPause = false;

  update(world, _dt_unused) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    const obstacleEntities = world.query([ObstacleTagComponent, PositionComponent, RadiusComponent, ObstaclePushComponent]);
    const flipperEntities = world.query([FlipperTagComponent, PositionComponent, RadiusComponent, FlipperStateComponent]);
    const borderEntities = world.query([BorderComponent]);

    let borderContacts = world.getResource('ball_border_contacts');
    if (!Array.isArray(borderContacts)) {
      borderContacts = [];
      world.setResource('ball_border_contacts', borderContacts);
    }
    borderContacts.length = 0;

    let obstacleContacts = world.getResource('ball_obstacle_contacts');
    if (!Array.isArray(obstacleContacts)) {
      obstacleContacts = [];
      world.setResource('ball_obstacle_contacts', obstacleContacts);
    }
    obstacleContacts.length = 0;

    let flipperContacts = world.getResource('ball_flipper_contacts');
    if (!Array.isArray(flipperContacts)) {
      flipperContacts = [];
      world.setResource('ball_flipper_contacts', flipperContacts);
    }
    flipperContacts.length = 0;

    let ballBallContacts = world.getResource('ball_ball_contacts');
    if (!Array.isArray(ballBallContacts)) {
      ballBallContacts = [];
      world.setResource('ball_ball_contacts', ballBallContacts);
    }
    ballBallContacts.length = 0;

    let obstacleActivePairs = world.getResource('ball_obstacle_active_pairs');
    if (!(obstacleActivePairs instanceof Set)) {
      obstacleActivePairs = new Set();
      world.setResource('ball_obstacle_active_pairs', obstacleActivePairs);
    }
    const nextObstacleActivePairs = new Set();
    const grabbed = world.getResource('grabbedBall');

    const useSectorSupports = layeringEnabled(world) && world.getResource('layeringCollisionSectorSolvers') !== false;
    const pinchShareEnabled = layeringEnabled(world) && world.getResource('layeringCollisionPinchShare') !== false;
    const cableJointSegments = pinchShareEnabled ? _collectCableJointSegments(world) : null;

    const borderId = borderEntities.length > 0 ? borderEntities[0] : null;
    const borderPoints = borderId !== null ? (world.getComponent(borderId, BorderComponent)?.points ?? []) : [];
    const borderPlaneNormal = borderId !== null
      ? normalizePlaneNormal(world.getComponent(borderId, BorderComponent)?.planeNormal ?? DEFAULT_PLANE_NORMAL)
      : DEFAULT_PLANE_NORMAL.clone();
    const borderRestitution = borderId !== null ? world.getComponent(borderId, RestitutionComponent)?.restitution : null;
    const borderFriction = borderId !== null ? world.getComponent(borderId, CoefficientOfFrictionComponent)?.mu : null;

    for (let i = 0; i < ballEntities.length; i++) {
      const ballId = ballEntities[i];
      const pBall = world.getComponent(ballId, PositionComponent)?.pos;
      if (!pBall) {
        continue;
      }

      if (borderPoints.length >= 2) {
        const broadRadius = useSectorSupports ? getMaxCollisionRadius(world, ballId) : _getBaseCollisionRadius(world, ballId);
        for (let segmentIndex = 0; segmentIndex < borderPoints.length; segmentIndex++) {
          const a = borderPoints[segmentIndex].clone();
          const b = borderPoints[(segmentIndex + 1) % borderPoints.length].clone();
          const closest = closestPointOnSegment(pBall, a, b);
          const distSq = pBall.distanceToSq(closest);
          if (distSq > (broadRadius + 1e-9) * (broadRadius + 1e-9)) {
            continue;
          }
          const normal = _borderCollisionNormal(pBall, closest, a, b, borderPlaneNormal);
          const support = _collisionSupportToward(world, ballId, normal.clone().scale(-1.0), useSectorSupports);
          if (!support) {
            continue;
          }
          const dist = Math.sqrt(distSq);
          const penetration = support.projection - dist;
          if (penetration <= 0.0) {
            continue;
          }
          const deltaLambda = _resolveRigidContactSingle(world, ballId, support.offset, normal, penetration);
          borderContacts.push({
            ball_id: ballId,
            normal: normal.clone(),
            delta_lambda: deltaLambda,
            ball_contact_radius: support.projection,
            ball_contact_offset: support.offset.clone(),
            ball_friction: support.friction,
            restitution: borderRestitution,
            friction: borderFriction,
            border_segment_index: segmentIndex
          });
        }
      }

      for (const obsId of obstacleEntities) {
        const pObs = world.getComponent(obsId, PositionComponent)?.pos;
        if (!pObs) {
          continue;
        }
        const delta = pBall.clone().subtract(pObs);
        const dSq = delta.lengthSq();
        if (dSq <= 1e-12) {
          continue;
        }
        const planeNormal = entityPlaneNormal(world, ballId);
        const d = Math.sqrt(dSq);
        const normal = delta.scale(1.0 / d);
        const betweenHalfWidth = (
          pinchShareEnabled &&
          cableJointSegments &&
          world.hasComponent(ballId, CableLinkComponent) &&
          world.hasComponent(obsId, CableLinkComponent)
        )
          ? _crossingCableHalfWidth(cableJointSegments, pBall, pObs, ballId, obsId, planeNormal)
          : 0.0;
        const betweenGapAllowance = betweenHalfWidth > 1e-9 ? (2.0 * betweenHalfWidth) : 0.0;
        const broadBall = useSectorSupports ? getMaxCollisionRadius(world, ballId) : _getBaseCollisionRadius(world, ballId);
        const broadObs = useSectorSupports ? getMaxCollisionRadius(world, obsId) : _getBaseCollisionRadius(world, obsId);
        if (d > broadBall + broadObs + betweenGapAllowance + 1e-9) {
          continue;
        }
        const supportBall = _collisionSupportToward(world, ballId, normal.clone().scale(-1.0), useSectorSupports);
        const supportObs = _collisionSupportToward(world, obsId, normal, useSectorSupports);
        if (!supportBall || !supportObs) {
          continue;
        }

        let resolvedSupportBall = supportBall;
        let resolvedSupportObs = supportObs;
        let pinchShared = false;
        if (betweenHalfWidth > 1e-9) {
          pinchShared = true;
          const baseBall = _getBaseCollisionRadius(world, ballId);
          const baseObs = _getBaseCollisionRadius(world, obsId);
          const sharedProjectionBall = baseBall + betweenHalfWidth;
          const sharedProjectionObs = baseObs + betweenHalfWidth;
          resolvedSupportBall = {
            ...supportBall,
            projection: sharedProjectionBall,
            offset: normal.clone().scale(-sharedProjectionBall)
          };
          resolvedSupportObs = {
            ...supportObs,
            projection: sharedProjectionObs,
            offset: normal.clone().scale(sharedProjectionObs)
          };
        }

        const rSum = resolvedSupportBall.projection + resolvedSupportObs.projection;
        if (d > rSum + 1e-9) {
          continue;
        }
        const penetration = rSum - d;
        if (penetration <= 0.0) {
          continue;
        }
        const deltaLambda = _resolveRigidContactSingle(world, ballId, resolvedSupportBall.offset, normal, penetration);
        const rawBallRadius = getRawCollisionRadius(world, ballId);
        const rawObsRadius = getRawCollisionRadius(world, obsId);
        const rawHit = d <= (rawBallRadius + rawObsRadius + 1e-9);
        obstacleContacts.push({
          ball_id: ballId,
          obs_id: obsId,
          direction: normal.clone(),
          raw_hit: rawHit,
          delta_lambda: deltaLambda,
          ball_contact_radius: resolvedSupportBall.projection,
          ball_contact_offset: resolvedSupportBall.offset.clone(),
          ball_friction: resolvedSupportBall.friction,
          obstacle_contact_radius: resolvedSupportObs.projection,
          obstacle_friction: resolvedSupportObs.friction,
          pinch_shared: pinchShared
        });
        const pairKey = `${ballId}:${obsId}`;
        if (rawHit) {
          nextObstacleActivePairs.add(pairKey);
        }
        if (rawHit && !obstacleActivePairs.has(pairKey) && ballId !== grabbed) {
          world.addComponent(ballId, new ScoredTagComponent());
        }
      }

      for (const flipId of flipperEntities) {
        const pFlip = world.getComponent(flipId, PositionComponent)?.pos;
        const flipperState = world.getComponent(flipId, FlipperStateComponent);
        if (!pFlip || !flipperState) {
          continue;
        }
        const tip = _flipperTipFromState(pFlip, flipperState);
        const closest = closestPointOnSegment(pBall, pFlip, tip);
        const delta = pBall.clone().subtract(closest);
        const dSq = delta.lengthSq();
        if (dSq <= 1e-12) {
          continue;
        }
        const planeNormal = flipperState.planeNormal ?? DEFAULT_PLANE_NORMAL;
        const d = Math.sqrt(dSq);
        const normal = delta.scale(1.0 / d);
        const broadBall = useSectorSupports ? getMaxCollisionRadius(world, ballId) : _getBaseCollisionRadius(world, ballId);
        const broadFlipper = useSectorSupports ? getMaxCollisionRadius(world, flipId) : _getBaseCollisionRadius(world, flipId);
        if (d > broadBall + broadFlipper + 1e-9) {
          continue;
        }
        const supportBall = _collisionSupportToward(world, ballId, normal.clone().scale(-1.0), useSectorSupports);
        if (!supportBall) {
          continue;
        }
        const flipperRadius = useSectorSupports ? getMaxCollisionRadius(world, flipId) : _getBaseCollisionRadius(world, flipId);
        const rSum = supportBall.projection + flipperRadius;
        if (d > rSum + 1e-9) {
          continue;
        }
        const penetration = rSum - d;
        if (penetration <= 0.0) {
          continue;
        }
        const deltaLambda = _resolveRigidContactSingle(world, ballId, supportBall.offset, normal, penetration);
        flipperContacts.push({
          ball_id: ballId,
          flip_id: flipId,
          normal: normal.clone(),
          contact_point_on_flipper: closest.clone(),
          ball_contact_radius: supportBall.projection,
          ball_contact_offset: supportBall.offset.clone(),
          ball_friction: supportBall.friction,
          delta_lambda: deltaLambda
        });
      }

      for (let j = i + 1; j < ballEntities.length; j++) {
        const otherId = ballEntities[j];
        const pOther = world.getComponent(otherId, PositionComponent)?.pos;
        if (!pOther) {
          continue;
        }
        const normalAB = pOther.clone().subtract(pBall);
        const dSq = normalAB.lengthSq();
        if (dSq <= 1e-12) {
          continue;
        }
        const planeNormal = entityPlaneNormal(world, ballId);
        const d = Math.sqrt(dSq);
        normalAB.scale(1.0 / d);
        const betweenHalfWidth = (
          pinchShareEnabled &&
          cableJointSegments &&
          world.hasComponent(ballId, CableLinkComponent) &&
          world.hasComponent(otherId, CableLinkComponent)
        )
          ? _crossingCableHalfWidth(cableJointSegments, pBall, pOther, ballId, otherId, planeNormal)
          : 0.0;
        const betweenGapAllowance = betweenHalfWidth > 1e-9 ? (2.0 * betweenHalfWidth) : 0.0;
        const broadA = useSectorSupports ? getMaxCollisionRadius(world, ballId) : _getBaseCollisionRadius(world, ballId);
        const broadB = useSectorSupports ? getMaxCollisionRadius(world, otherId) : _getBaseCollisionRadius(world, otherId);
        if (d > broadA + broadB + betweenGapAllowance + 1e-9) {
          continue;
        }
        const supportA = _collisionSupportToward(world, ballId, normalAB, useSectorSupports);
        const supportB = _collisionSupportToward(world, otherId, normalAB.clone().scale(-1.0), useSectorSupports);
        if (!supportA || !supportB) {
          continue;
        }

        let resolvedSupportA = supportA;
        let resolvedSupportB = supportB;
        let pinchShared = false;
        if (betweenHalfWidth > 1e-9) {
          pinchShared = true;
          const baseA = _getBaseCollisionRadius(world, ballId);
          const baseB = _getBaseCollisionRadius(world, otherId);
          const sharedProjectionA = baseA + betweenHalfWidth;
          const sharedProjectionB = baseB + betweenHalfWidth;
          resolvedSupportA = {
            ...supportA,
            projection: sharedProjectionA,
            offset: normalAB.clone().scale(sharedProjectionA)
          };
          resolvedSupportB = {
            ...supportB,
            projection: sharedProjectionB,
            offset: normalAB.clone().scale(-sharedProjectionB)
          };
        }

        const rSum = resolvedSupportA.projection + resolvedSupportB.projection;
        if (d > rSum + 1e-9) {
          continue;
        }
        const penetration = rSum - d;
        if (penetration <= 0.0) {
          continue;
        }
        const deltaLambda = _resolveRigidContactPair(world, ballId, resolvedSupportA.offset, otherId, resolvedSupportB.offset, normalAB, penetration);
        ballBallContacts.push({
          ball_a: ballId,
          ball_b: otherId,
          normal: normalAB.clone(),
          delta_lambda: deltaLambda,
          penetration,
          radius_a: resolvedSupportA.projection,
          radius_b: resolvedSupportB.projection,
          contact_offset_a: resolvedSupportA.offset.clone(),
          contact_offset_b: resolvedSupportB.offset.clone(),
          friction_a: resolvedSupportA.friction,
          friction_b: resolvedSupportB.friction,
          pinch_shared: pinchShared
        });
      }
    }

    obstacleActivePairs.clear();
    for (const pairKey of nextObstacleActivePairs) {
      obstacleActivePairs.add(pairKey);
    }
  }
}

export class PBDBallBorderCollisions {
  runInPause = false;

  update(world, _dt_unused) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    const borderEntities = world.query([BorderComponent]);
    if (borderEntities.length === 0) return;

    const borderId = borderEntities[0];
    const borderComp = world.getComponent(borderId, BorderComponent);
    const borderPoints = borderComp.points;

    const borderRestitutionComp = world.getComponent(borderId, RestitutionComponent);
    const borderFrictionComp = world.getComponent(borderId, CoefficientOfFrictionComponent);

    const restitution = borderRestitutionComp ? borderRestitutionComp.restitution : null;
    const friction = borderFrictionComp ? borderFrictionComp.mu : null;

    let contacts = world.getResource('ball_border_contacts');
    if (!contacts) {
      contacts = [];
      world.setResource('ball_border_contacts', contacts);
    }
    contacts.length = 0;

    if (borderPoints.length < 2) {
      return;
    }

    const planeNormal = normalizePlaneNormal(borderComp.planeNormal || DEFAULT_PLANE_NORMAL);
    const planeOffset = borderPoints.length > 0 ? planeNormal.dot(borderPoints[0]) : 0.0;

    for (const ballId of ballEntities) {
      const ballPos = world.getComponent(ballId, PositionComponent).pos;
      const radius = world.getComponent(ballId, RadiusComponent).radius;
      const massComp = world.getComponent(ballId, MassComponent);
      const invMass = massComp && massComp.mass > 0 ? 1.0 / massComp.mass : 0.0;
      projectPointToPlane(ballPos, planeNormal, planeOffset);

      let minDistSq = Infinity;
      let closestSegPoint = null;
      let edgeStart = null;
      let edgeEnd = null;

      for (let i = 0; i < borderPoints.length; i++) {
        const a = borderPoints[i].clone();
        const b = borderPoints[(i + 1) % borderPoints.length].clone();
        projectPointToPlane(a, planeNormal, planeOffset);
        projectPointToPlane(b, planeNormal, planeOffset);
        const closest = closestPointOnSegment(ballPos, a, b);
        const distSq = ballPos.distanceToSq(closest);

        if (distSq < minDistSq) {
          minDistSq = distSq;
          closestSegPoint = closest;
          edgeStart = a;
          edgeEnd = b;
        }
      }

      if (!closestSegPoint || minDistSq > radius * radius) {
        continue;
      }

      const ballToClosest = ballPos.clone().subtract(closestSegPoint);
      const edgeVec = edgeEnd.clone().subtract(edgeStart);
      const edgeNormal = planeNormal.clone().cross(edgeVec).normalize();

      let collisionNormal;
      if (ballToClosest.lengthSq() < 1e-9) {
        collisionNormal = edgeNormal.clone();
      } else {
        collisionNormal = ballToClosest.clone().normalize();
      }

      if (ballToClosest.dot(edgeNormal) < 0) {
        collisionNormal = edgeNormal.clone();
      }

      const dist = Math.sqrt(minDistSq);
      const penetration = radius - dist;

      let deltaLambda = 0;
      if (penetration > 0 && invMass > 0) {
        ballPos.add(collisionNormal, penetration);
        projectPointToPlane(ballPos, planeNormal, planeOffset);
        deltaLambda = penetration / invMass;
      }

      contacts.push({
        ball_id: ballId,
        normal: collisionNormal.clone(),
        delta_lambda: deltaLambda,
        restitution,
        friction
      });
    }
  }
}

export class PBDBallBallCollisions {
  runInPause = false;

  update(world, _dt_unused) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    for (let i = 0; i < ballEntities.length; i++) {
      for (let j = i + 1; j < ballEntities.length; j++) {
        const e1 = ballEntities[i];
        const e2 = ballEntities[j];

        const p1Comp = world.getComponent(e1, PositionComponent);
        const r1 = world.getComponent(e1, RadiusComponent).radius;
        const m1Comp = world.getComponent(e1, MassComponent);

        const p2Comp = world.getComponent(e2, PositionComponent);
        const r2 = world.getComponent(e2, RadiusComponent).radius;
        const m2Comp = world.getComponent(e2, MassComponent);

        const p1 = p1Comp.pos;
        const p2 = p2Comp.pos;
        const m1 = m1Comp ? m1Comp.mass : 0.0;
        const m2 = m2Comp ? m2Comp.mass : 0.0;

        const dir = new Vector3(p2.x - p1.x, p2.y - p1.y, 0.0);
        const dSq = dir.lengthSq();
        const rSum = r1 + r2;

        if (dSq === 0.0 || dSq > rSum * rSum) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d);

        const penetration = rSum - d;
        const invMass1 = m1 > 0 ? 1.0 / m1 : 0.0;
        const invMass2 = m2 > 0 ? 1.0 / m2 : 0.0;
        const totalInvMass = invMass1 + invMass2;

        if (totalInvMass <= 1e-9) continue;

        const corr = dir.clone().scale(penetration / totalInvMass);
        p1.add(corr, -invMass1);
        p2.add(corr, invMass2);
      }
    }
  }
}

export class PBDBallObstacleCollisions {
  runInPause = false;

  update(world, _dt_unused) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent]);
    const obstacleEntities = world.query([ObstacleTagComponent, PositionComponent, RadiusComponent, ObstaclePushComponent]);

    let contacts = world.getResource('ball_obstacle_contacts');
    if (!contacts) {
      contacts = [];
      world.setResource('ball_obstacle_contacts', contacts);
    }
    contacts.length = 0;

    for (const ballId of ballEntities) {
      const p1 = world.getComponent(ballId, PositionComponent).pos;
      const r1 = world.getComponent(ballId, RadiusComponent).radius;

      for (const obsId of obstacleEntities) {
        const p2 = world.getComponent(obsId, PositionComponent).pos;
        const r2 = world.getComponent(obsId, RadiusComponent).radius;

        const dir = new Vector3(p1.x - p2.x, p1.y - p2.y, 0.0);
        const dSq = dir.lengthSq();
        const rSum = r1 + r2;

        if (dSq === 0.0 || dSq > rSum * rSum) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d);

        contacts.push({ ball_id: ballId, obs_id: obsId, direction: dir.clone() });

        const corr = rSum - d;
        p1.add(dir, corr);

        const grabbed = world.getResource('grabbedBall');
        if (ballId !== grabbed) {
          world.addComponent(ballId, new ScoredTagComponent());
        }
      }
    }
  }
}

export class PBDBallFlipperCollisions {
  runInPause = false;

  _getFlipperTip(flipperPos, flipperState) {
    const angle = flipperState.restAngle + flipperState.sign * flipperState.rotation;
    const dir = new Vector3(Math.cos(angle), Math.sin(angle), 0);
    return flipperPos.clone().add(dir, flipperState.length);
  }

  update(world, _dt_unused) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    const flipperEntities = world.query([FlipperTagComponent, PositionComponent, RadiusComponent, FlipperStateComponent]);

    let contacts = world.getResource('ball_flipper_contacts');
    if (!contacts) {
      contacts = [];
      world.setResource('ball_flipper_contacts', contacts);
    }
    contacts.length = 0;

    for (const ballId of ballEntities) {
      const ballPos = world.getComponent(ballId, PositionComponent).pos;
      const ballRadius = world.getComponent(ballId, RadiusComponent).radius;
      const massComp = world.getComponent(ballId, MassComponent);
      const invMass = massComp && massComp.mass > 0 ? 1.0 / massComp.mass : 0.0;

      for (const flipperId of flipperEntities) {
        const pivot = world.getComponent(flipperId, PositionComponent).pos;
        const flipperRadius = world.getComponent(flipperId, RadiusComponent).radius;
        const flipperState = world.getComponent(flipperId, FlipperStateComponent);

        const tip = this._getFlipperTip(pivot, flipperState);
        const closest = closestPointOnSegment(ballPos, pivot, tip);

        const dir = ballPos.clone().subtract(closest);
        const dSq = dir.lengthSq();
        const rSum = ballRadius + flipperRadius;

        if (dSq === 0.0 || dSq > rSum * rSum) {
          continue;
        }

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d);

        const correction = rSum - d;
        if (invMass > 0) {
          ballPos.add(dir, correction);
        }

        let deltaLambda = 0;
        if (invMass > 0) {
          deltaLambda = correction / invMass;
        }

        contacts.push({
          ball_id: ballId,
          flip_id: flipperId,
          normal: dir.clone(),
          contact_point_on_flipper: closest.clone(),
          delta_lambda: deltaLambda
        });
      }
    }
  }
}
