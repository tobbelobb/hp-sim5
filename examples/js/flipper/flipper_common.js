import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  PositionComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  OrientationComponent,
  MomentOfInertiaComponent,
  RestitutionComponent,
  PrevFinalPosComponent,
  RenderableComponent,
  CoefficientOfFrictionComponent
} from '../../../src/js/cable_joints/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
} from '../../../src/js/cable_joints/cable_joints_core.js';
import { closestPointOnSegment, rightOfLine } from '../../../src/js/cable_joints/geometry.js';

export class BallTagComponent { }

export class ObstacleTagComponent { }

export class ObstaclePushComponent { constructor(pushVel = 2.0) { this.pushVel = pushVel; } }

export class FlipperTagComponent { }

export class FlipperStateComponent {
  constructor(length, restAngle, maxRotation, angularVelocity) {
    this.length = length;
    this.restAngle = restAngle;
    this.maxRotation = Math.abs(maxRotation);
    this.sign = Math.sign(maxRotation); // Direction it rotates
    this.angularVelocity = angularVelocity;
    // Dynamic state
    this.rotation = 0.0; // Current rotation from restAngle
    this.currentAngularVelocity = 0.0; // Velocity in the last frame
    this.pressed = false; // Was it activated?
  }
}

export class FlipperTipComponent {
  constructor(flipperEntityId) {
    this.flipperEntityId = flipperEntityId;
  }
}

export class BorderComponent { constructor(points = []) { this.points = points.map(p => p.clone()); } }

export class ScoredTagComponent { }

export class ScoreComponent { constructor(score = 0) { this.value = score; } }

export class PauseStateComponent { constructor(paused = true) { this.paused = paused; } }

export class CircleCamTag { }

export class OverlayRadiusComponent {
  constructor(radius = 0.0) {
    this.radius = Number.isFinite(radius) ? Math.max(0.0, radius) : 0.0;
  }
}

export class CircleSectorComponent {
  constructor(radius = 0.0, startAngle = 0.0, endAngle = 0.0, cw = false) {
    this.radius = Number.isFinite(radius) ? Math.max(0.0, radius) : 0.0;
    this.startAngle = Number.isFinite(startAngle) ? startAngle : 0.0;
    this.endAngle = Number.isFinite(endAngle) ? endAngle : 0.0;
    this.cw = cw === true;
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
          cw: sector.cw === true
        }))
      : [];
  }
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
    // Clockwise sector can be treated as CCW in mirrored angle space.
    return _isAngleInSector(-a, -s, -e, false);
  }
  const span = _ccwDiff(s, e);
  const rel = _ccwDiff(s, a);
  return rel <= span + 1e-9;
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

export function getRawCollisionRadius(world, entityId) {
  const radius = world.getComponent(entityId, RadiusComponent)?.radius;
  return Number.isFinite(radius) ? Math.max(0.0, radius) : 0.0;
}

function _getBaseCollisionRadius(world, entityId) {
  let radius = getRawCollisionRadius(world, entityId);
  const overlayComp = world.getComponent(entityId, OverlayRadiusComponent);
  if (overlayComp && Number.isFinite(overlayComp.radius)) {
    radius = Math.max(radius, overlayComp.radius);
  }
  return radius;
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

function _normalizedDirection(vec, fallback = new Vector2(1.0, 0.0)) {
  if (!vec || vec.lengthSq() <= 1e-12) {
    return fallback.clone();
  }
  return vec.clone().normalize();
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
  const dir = _normalizedDirection(directionTowardOther);
  const baseRadius = _getBaseCollisionRadius(world, entityId);
  let projection = baseRadius;
  let offset = dir.clone().scale(baseRadius);
  const sectorList = _entitySectorList(world, entityId);
  const angle = Math.atan2(dir.y, dir.x);

  for (const sectorComp of sectorList) {
    if (
      !sectorComp ||
      !Number.isFinite(sectorComp.radius) ||
      !(sectorComp.radius > baseRadius + 1e-9)
    ) {
      continue;
    }

    const weight = _sectorWeight(angle, sectorComp.startAngle, sectorComp.endAngle, sectorComp.cw === true);
    if (weight > 1e-9) {
      const taperedRadius = baseRadius + (sectorComp.radius - baseRadius) * weight;
      if (taperedRadius > projection + 1e-9) {
        projection = taperedRadius;
        offset = dir.clone().scale(projection);
      }
    }

    // Corner supports are essential for stability at cable ends, but we taper their
    // projection to match the smoothed sector profile.
    const cornerAngles = [sectorComp.startAngle, sectorComp.endAngle];
    for (const cornerAngle of cornerAngles) {
      const cornerWeight = _sectorWeight(cornerAngle, sectorComp.startAngle, sectorComp.endAngle, sectorComp.cw === true);
      const effectiveCornerRadius = baseRadius + (sectorComp.radius - baseRadius) * cornerWeight;

      const cornerOffset = new Vector2(
        Math.cos(cornerAngle) * effectiveCornerRadius,
        Math.sin(cornerAngle) * effectiveCornerRadius
      );
      const cornerProjection = cornerOffset.dot(dir);
      if (cornerProjection > projection + 1e-9) {
        projection = cornerProjection;
        offset = cornerOffset;
      }
    }
  }

  return {
    center,
    baseRadius,
    projection,
    offset
  };
}

export function getCompositeSupportToward(world, entityId, directionTowardOther) {
  const support = _compositeSupportToward(world, entityId, directionTowardOther);
  if (!support) {
    return null;
  }
  return {
    baseRadius: support.baseRadius,
    projection: support.projection,
    offset: support.offset.clone()
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
  const dir = _normalizedDirection(directionTowardOther);
  const baseRadius = _getBaseCollisionRadius(world, entityId);
  return {
    center,
    baseRadius,
    projection: baseRadius,
    offset: dir.clone().scale(baseRadius)
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
    const joint = world.getComponent(path.jointEntities[0], CableJointComponent);
    return joint?.attachmentPointA_world?.clone() ?? null;
  }
  if (linkIndex === path.linkTypes.length - 1) {
    const joint = world.getComponent(path.jointEntities[path.jointEntities.length - 1], CableJointComponent);
    return joint?.attachmentPointB_world?.clone() ?? null;
  }
  const leftJoint = world.getComponent(path.jointEntities[linkIndex - 1], CableJointComponent);
  return leftJoint?.attachmentPointB_world?.clone() ?? null;
}

function _decomposeStoredWrap(storedLength, firstLayerRadius, layerStep) {
  const stored = Math.max(0.0, storedLength ?? 0.0);
  if (!(stored > 1e-9) || !(firstLayerRadius > 1e-9) || !(layerStep > 1e-9)) {
    return null;
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
    return {
      fullLayers,
      partialLength: remaining,
      partialRadius: layerRadius
    };
  }
  return {
    fullLayers,
    partialLength: 0.0,
    partialRadius: firstLayerRadius + fullLayers * layerStep
  };
}

const LAYER_RADIUS_RAMP_ANGLE = Math.PI / 2.0; // 90 degree ramp for smoother growth
const SECTOR_TAPER_ANGLE = Math.PI / 36.0; // Sharper 5 degree taper to prevent sinking

function _closingOverlayBlend(span) {
  if (!(LAYER_RADIUS_RAMP_ANGLE > 1e-9)) {
    return 0.0;
  }
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
  if (!(LAYER_RADIUS_RAMP_ANGLE > 1e-9)) {
    return rampTargetRadius;
  }
  const spanValue = Number.isFinite(span) ? Math.max(0.0, span) : 0.0;
  const rampAlpha = Math.max(0.0, Math.min(1.0, spanValue / LAYER_RADIUS_RAMP_ANGLE));
  return rampStartRadius + (rampTargetRadius - rampStartRadius) * rampAlpha;
}

function _sectorWeight(angle, startAngle, endAngle, cw) {
  const a = _normalizeAngle(angle);
  const s = _normalizeAngle(startAngle);
  const e = _normalizeAngle(endAngle);
  if (cw) {
    return _sectorWeight(-a, -s, -e, false);
  }
  const span = _ccwDiff(s, e);
  const rel = _ccwDiff(s, a);

  if (rel > span + 1e-9) return 0.0;

  const taper = Math.min(span * 0.4, SECTOR_TAPER_ANGLE);
  if (taper < 1e-9) return 1.0;

  if (rel < taper) return rel / taper;
  if (rel > span - taper) return (span - rel) / taper;
  return 1.0;
}

export class OverlayRadiusAndCircleSectorSystem {
  runInPause = false;

  update(world, _dt_unused) {
    if (!_layeringEnabled(world)) {
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

    const overlayEnabled = _layeringFlag(world, 'layeringCollisionOverlayRadius', true);
    const overlayRampEnabled = _layeringFlag(world, 'layeringCollisionOverlayRamp', true);
    const sectorEnabled = _layeringFlag(world, 'layeringCollisionCircleSectors', true);

    const overlayByEntity = new Map();
    const sectorListByEntity = new Map();
    const pathEntities = world.query([CablePathComponent]);

    for (const pathId of pathEntities) {
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
        if (!(stored > 1e-9)) {
          continue;
        }

        const entityId = _pathLinkEntity(world, path, linkIndex);
        if (entityId === null || entityId === undefined) {
          continue;
        }
        const center = world.getComponent(entityId, PositionComponent)?.pos;
        const rawRadius = getRawCollisionRadius(world, entityId);
        if (!center || !(rawRadius > 1e-9)) {
          continue;
        }

        const firstLayerRadius = rawRadius + halfWidth;
        const decomposition = _decomposeStoredWrap(stored, firstLayerRadius, layerStep);
        if (!decomposition) {
          continue;
        }

        const baseOverlayRadius = rawRadius + layerStep * decomposition.fullLayers;
        let overlayEnvelopeRadius = baseOverlayRadius;
        if (
          overlayRampEnabled &&
          decomposition.partialLength > 1e-9 &&
          decomposition.partialRadius > 1e-9
        ) {
          const span = decomposition.partialLength / decomposition.partialRadius;
          const closingBlend = _closingOverlayBlend(span);
          if (closingBlend > 1e-9) {
            overlayEnvelopeRadius = baseOverlayRadius + layerStep * closingBlend;
          }
        }

        if (overlayEnabled) {
          if (overlayEnvelopeRadius > rawRadius + 1e-9) {
            const prev = overlayByEntity.get(entityId) ?? 0.0;
            overlayByEntity.set(entityId, Math.max(prev, overlayEnvelopeRadius));
          }
        }

        if (sectorEnabled && decomposition.partialLength > 1e-9) {
          const startPoint = _pathLinkStartAttachment(world, path, linkIndex);
          if (!startPoint) {
            continue;
          }
          const rel = startPoint.clone().subtract(center);
          if (rel.lengthSq() <= 1e-12) {
            continue;
          }
          const startAngle = Math.atan2(rel.y, rel.x);
          const span = decomposition.partialLength / decomposition.partialRadius;
          const sectorRadius = _smoothedSectorRadius(
            rawRadius,
            decomposition,
            halfWidth,
            span
          );
          if (!(sectorRadius > rawRadius + 1e-9)) {
            continue;
          }
          const cw = Boolean(path.cw[linkIndex]);
          const endAngle = cw ? (startAngle - span) : (startAngle + span);
          const sector = {
            radius: sectorRadius,
            startAngle,
            endAngle,
            cw
          };
          const list = sectorListByEntity.get(entityId) ?? [];
          list.push(sector);
          sectorListByEntity.set(entityId, list);
        }
      }
    }

    for (const [entityId, radius] of overlayByEntity.entries()) {
      if (world.hasComponent(entityId, OverlayRadiusComponent)) {
        world.getComponent(entityId, OverlayRadiusComponent).radius = radius;
      } else {
        world.addComponent(entityId, new OverlayRadiusComponent(radius));
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
      const sector = validSectors.reduce((best, candidate) => (
        !best || candidate.radius > best.radius ? candidate : best
      ), null);
      if (!sector) {
        continue;
      }
      if (world.hasComponent(entityId, CircleSectorComponent)) {
        const comp = world.getComponent(entityId, CircleSectorComponent);
        comp.radius = sector.radius;
        comp.startAngle = sector.startAngle;
        comp.endAngle = sector.endAngle;
        comp.cw = sector.cw;
      } else {
        world.addComponent(
          entityId,
          new CircleSectorComponent(sector.radius, sector.startAngle, sector.endAngle, sector.cw)
        );
      }
      if (world.hasComponent(entityId, CircleSectorsComponent)) {
        const comp = world.getComponent(entityId, CircleSectorsComponent);
        comp.sectors = validSectors.map((entry) => ({
          radius: entry.radius,
          startAngle: entry.startAngle,
          endAngle: entry.endAngle,
          cw: entry.cw === true
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

// --- System: Input --- (Simplified Click Handling)
export class InputSystem {
     runInPause = true; // Input should work even when paused to unpause/interact

     constructor(canvas, world, grabSpringParams = { restLength: 0.1, springConstant: () => 10.0 }) {
         this.canvas = canvas;
         this.world = world;
         this.grabSpringParams = grabSpringParams;
         this.clicks = [];
         this.releases = [];
         this.eventLog = [];     // record inputs per frame
         this.frame = 0;         // frame counter
         this.grabSpring = null; // { ptrE, jointE, pathE, ballE }
         this.canvas.setAttribute('tabindex', '0');
         this.canvas.style.outline = 'none';
         this.canvas.focus();
         // listen globally so ups/downs outside the canvas still fire
         document.addEventListener('pointerdown', this.handlePointerDown.bind(this));
         document.addEventListener('pointerup', this.handlePointerUp.bind(this));
         this.canvas.addEventListener('pointermove', this.handlePointerMove.bind(this));
         this.canvas.addEventListener('keydown', this.handleKeydown.bind(this));
         this.canvas.addEventListener('keyup', this.handleKeyup.bind(this));
     }

    // on Space emit minimal debug dump
    dumpDebugScenario() {
        // Custom stringification to serialize Vector2 instances
        const resStr = JSON.stringify(this.world.resources);
        const logEntries = this.eventLog.map(frame => {
            const clicksStr = frame.clicks.map(p => `new Vector2(${p.x}, ${p.y})`).join(', ');
            const releasesStr = frame.releases.map(p => `new Vector2(${p.x}, ${p.y})`).join(', ');
            return `{ "frame": ${frame.frame}, "clicks": [${clicksStr}], "releases": [${releasesStr}] }`;
        }).join(', ');
        console.log('DEBUG_SCENARIO_DUMP', `{ "resources": ${resStr}, "inputLog": [${logEntries}] }`);
    }

     handleKeyup(event) {
         if (event.key == 'ArrowLeft' || event.key == 'ArrowRight') {
             const flipperEntities = this.world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent]);
             if (flipperEntities.length < 2) {
               return;
             }
             var firstPos = this.world.getComponent(flipperEntities[0], PositionComponent).pos;
             var secondPos = this.world.getComponent(flipperEntities[1], PositionComponent).pos;
             if (firstPos.x < secondPos.x) {
                 if (event.key == 'ArrowLeft') {
                   this.releases.push(firstPos);
                 }
                 if  (event.key == 'ArrowRight') {
                   this.releases.push(secondPos);
                 }
             } else {
                 if (event.key == 'ArrowLeft') {
                   this.releases.push(secondPos);
                 }
                 if  (event.key == 'ArrowRight') {
                   this.releases.push(firstPos);
                 }
             }
         }
     }

     handleKeydown(event) {
         if (event.code === 'Space') {
             this.dumpDebugScenario();
         }
         if (event.key == 'ArrowLeft' || event.key == 'ArrowRight') {
             const flipperEntities = this.world.query([FlipperTagComponent, PositionComponent, FlipperStateComponent]);
             if (flipperEntities.length < 2) {
               return;
             }
             var firstPos = this.world.getComponent(flipperEntities[0], PositionComponent).pos;
             var secondPos = this.world.getComponent(flipperEntities[1], PositionComponent).pos;
             if (firstPos.x < secondPos.x) {
                 if (event.key == 'ArrowLeft') {
                   this.clicks.push(firstPos);
                 }
                 if  (event.key == 'ArrowRight') {
                   this.clicks.push(secondPos);
                 }
             } else {
                 if (event.key == 'ArrowLeft') {
                   this.clicks.push(secondPos);
                 }
                 if  (event.key == 'ArrowRight') {
                   this.clicks.push(firstPos);
                 }
             }
         }
     }

     handlePointerDown(event) {
         event.preventDefault();
         const rect = this.canvas.getBoundingClientRect();
         const scale = this.canvas.height / this.world.getResource('simHeight');
         const simX = (event.clientX - rect.left) / scale;
         const simY = (this.canvas.height - (event.clientY - rect.top)) / scale;

         const cmOnScreen = 0.5; // Desired clickable radius increase in physical cm
         const dpi = 96; // Approximate screen DPI; adjust or measure if needed
         const pixelsPerCm = dpi / 2.54;
         const extraPixels = cmOnScreen * pixelsPerCm;
         const extraClickableRadius = extraPixels / scale;

         const clickVec = new Vector2(simX, simY);

         let closestBall = null;
         let closestDistSq = Infinity;
         for (const b of this.world.query([BallTagComponent, PositionComponent, RadiusComponent])) {
           const pos = this.world.getComponent(b, PositionComponent).pos;
           const r1 = getMaxCollisionRadius(this.world, b) + extraClickableRadius;
           const distSq = clickVec.clone().subtract(pos).lengthSq();
           if (distSq > r1 * r1) continue;
           if (distSq < closestDistSq) {
             closestBall = b;
             closestDistSq = distSq;
           }
         }

         if (closestBall !== null) {
           const ptrE = this.world.createEntity();
           this.world.addComponent(ptrE, new PositionComponent(simX, simY));
           this.world.addComponent(ptrE, new CableLinkComponent(simX, simY));

           const ballPos = this.world.getComponent(closestBall, PositionComponent).pos.clone();
           const ballMass = this.world.getComponent(closestBall, MassComponent).mass;
           const ptrPos  = new Vector2(simX, simY);
           const jointE = this.world.createEntity();
           this.world.addComponent(jointE,
             new CableJointComponent(
               closestBall,
               ptrE,
               this.grabSpringParams.restLength,
               ballPos,
               ptrPos
             )
           );
           this.world.addComponent(jointE, new RenderableComponent('line', '#888888'));

           const pathE = this.world.createEntity();
           const pathComp = new CablePathComponent(
             this.world,
             [ jointE ],
             ['attachment', 'attachment'],
             [ true ],
             this.grabSpringParams.springConstant(ballMass)
           );
           this.world.addComponent(pathE, pathComp);

           this.grabSpring = { ptrE, jointE, pathE, ballE: closestBall };
           this.world.setResource('grabbedBall', closestBall);

           const pauseState = this.world.getResource('pauseState');
           pauseState.paused = false;
           document.getElementById("pauseBtn").textContent = "Pause";

           return;
         }

         this.clicks.push(new Vector2(simX, simY));
     }

     handlePointerUp(event) {
         event.preventDefault();
         this.canvas.releasePointerCapture(event.pointerId);
         const rect = this.canvas.getBoundingClientRect();
         const scale = this.canvas.height / this.world.getResource('simHeight');
         const simX = (event.clientX - rect.left) / scale;
         const simY = (this.canvas.height - (event.clientY - rect.top)) / scale;

         if (this.grabSpring) {
           const { ptrE, jointE, pathE, ballE } = this.grabSpring;
           const velComp = this.world.getComponent(ballE, VelocityComponent);
           const posComp = this.world.getComponent(ballE, PositionComponent);
           const prevFinalPosComp = this.world.getComponent(ballE, PrevFinalPosComponent);

           const dt = this.world.getResource('dt');
           if (velComp && prevFinalPosComp) {
             velComp.vel.set(posComp.pos.clone().subtract(prevFinalPosComp.pos).scale(1.0/dt));
           }
           this.world.destroyEntity(pathE);
           this.world.destroyEntity(jointE);
           this.world.destroyEntity(ptrE);
           this.grabSpring = null;
           this.world.setResource('grabbedBall', null);
           return;
         }
         this.releases.push(new Vector2(simX, simY));
     }

     update(world, dt) {
         const clicksFrame = this.clicks.slice();
         const releasesFrame = this.releases.slice();
         if (clicksFrame.length > 0 || releasesFrame.length > 0) {
             this.eventLog.push({ frame: this.frame, clicks: clicksFrame, releases: releasesFrame });
         }
         this.frame++;

         if (clicksFrame.length > 0) {
             const clickPos = this.clicks.shift();
           const flipperEntities = world.query([
             FlipperTagComponent,
             PositionComponent,
             FlipperStateComponent
           ]);

           const pressExtremeFlipper = (preferRight) => {
             if (flipperEntities.length < 1) {
               return false;
             }
             let chosenId = null;
             let chosenX = preferRight ? -Infinity : Infinity;
             for (const id of flipperEntities) {
               const pos = world.getComponent(id, PositionComponent)?.pos;
               if (!pos || !Number.isFinite(pos.x)) {
                 continue;
               }
               if ((preferRight && pos.x > chosenX) || (!preferRight && pos.x < chosenX)) {
                 chosenX = pos.x;
                 chosenId = id;
               }
             }
             if (chosenId === null) {
               return false;
             }
             const state = world.getComponent(chosenId, FlipperStateComponent);
             if (!state) {
               return false;
             }
             state.pressed = true;
             return true;
           };

           const borderEnts = world.query([BorderComponent]);
           let handled = false;
           if (borderEnts.length > 0 && flipperEntities.length > 0) {
             const borderPoints = world.getComponent(borderEnts[0], BorderComponent)?.points ?? [];
             const isValidPoint = (p) => p && Number.isFinite(p.x) && Number.isFinite(p.y);
             const hasClassicSideZones =
               borderPoints.length >= 8 &&
               isValidPoint(borderPoints[0]) &&
               isValidPoint(borderPoints[1]) &&
               isValidPoint(borderPoints[2]) &&
               isValidPoint(borderPoints[5]) &&
               isValidPoint(borderPoints[6]) &&
               isValidPoint(borderPoints[7]);

             if (hasClassicSideZones) {
               const rightClick =
                 rightOfLine(clickPos, borderPoints[0], borderPoints[1]) &&
                 rightOfLine(clickPos, borderPoints[1], borderPoints[2]);
               const leftClick =
                 rightOfLine(clickPos, borderPoints[5], borderPoints[6]) &&
                 rightOfLine(clickPos, borderPoints[6], borderPoints[7]);
               if (rightClick) {
                 handled = pressExtremeFlipper(true);
               } else if (leftClick) {
                 handled = pressExtremeFlipper(false);
               }
             }
           }

           if (!handled) {
             for (const id of flipperEntities) {
               const pos   = world.getComponent(id, PositionComponent)?.pos;
               const state = world.getComponent(id, FlipperStateComponent);
               if (!pos || !state) {
                 continue;
               }
               if (clickPos.clone().subtract(pos).lengthSq() < state.length ** 2) {
                 state.pressed = true;
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

     handlePointerMove(event) {
       event.preventDefault();
       const rect = this.canvas.getBoundingClientRect();
       const scale = this.canvas.height / this.world.getResource('simHeight');
       const simX = (event.clientX - rect.left) / scale;
       const simY = (this.canvas.height - (event.clientY - rect.top)) / scale;
       if (this.grabSpring) {
         const { ptrE } = this.grabSpring;
         const pos = this.world.getComponent(ptrE, PositionComponent).pos;
         pos.set(new Vector2(simX, simY));
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
  update(world, dt) {
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
    update(world, dt) {
        const scoreEntity = world.query([ScoreComponent])[0];
        const scoreComp = scoreEntity !== undefined ? world.getComponent(scoreEntity, ScoreComponent) : null;
        if (!scoreComp) return;

        const scoredEntities = world.query([ScoredTagComponent]);
        for (const scoredId of scoredEntities) {
           world.removeComponent(scoredId, ScoredTagComponent);
           scoreComp.value++;
        }
    }
}

export class ScoreDisplaySystem {
    runInPause = true;
    constructor(elementId) {
        this.scoreElement = document.getElementById(elementId);
    }
    update(world, dt) {
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
            state.currentAngularVelocity = (dt > 1e-6) ? state.sign * (state.rotation - prevRotation) / dt : 0.0;
        }
    }
}

export class FlipperTipLinkSystem {
  runInPause = false;
  update(world, dt) {
    // for each tip‐entity, compute its current position
    for (const tipId of world.query([FlipperTipComponent, PositionComponent])) {
      const tipComp   = world.getComponent(tipId, FlipperTipComponent);
      const flipId    = tipComp.flipperEntityId;
      const pivotPos  = world.getComponent(flipId, PositionComponent).pos;
      const state     = world.getComponent(flipId, FlipperStateComponent);
      // same math you use elsewhere to find the tip
      const angle = state.restAngle + state.sign * state.rotation;
      const dir   = new Vector2(Math.cos(angle), Math.sin(angle));
      const tipPos = pivotPos.clone().add(dir, state.length);
      // write it into the tip entity’s PositionComponent
      world.getComponent(tipId, PositionComponent).pos.set(tipPos);
    }
  }
}

function _borderCollisionNormal(point, closestPoint, edgeStart, edgeEnd) {
  const ballToClosest = new Vector2().subtractVectors(point, closestPoint);
  const edgeVec = new Vector2().subtractVectors(edgeEnd, edgeStart);
  const normal = new Vector2(-edgeVec.y, edgeVec.x).normalize();

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

function _resolveRigidContactSingle(world, entityId, contactOffset, normal, penetration) {
  if (!(penetration > 0.0)) {
    return 0.0;
  }
  const posComp = world.getComponent(entityId, PositionComponent);
  if (!posComp?.pos) {
    return 0.0;
  }

  const massComp = world.getComponent(entityId, MassComponent);
  const invMass = (massComp && massComp.mass > 0.0) ? 1.0 / massComp.mass : 0.0;
  const moiComp = world.getComponent(entityId, MomentOfInertiaComponent);
  const invInertia = moiComp ? moiComp.invInertia : 0.0;
  const orientComp = world.getComponent(entityId, OrientationComponent);

  const r = contactOffset?.clone?.() ?? new Vector2(0.0, 0.0);
  const rn = r.x * normal.y - r.y * normal.x;
  const denom = invMass + invInertia * rn * rn;
  if (denom <= 1e-12) {
    return 0.0;
  }

  const lambda = penetration / denom;
  if (invMass > 0.0) {
    posComp.pos.add(normal, invMass * lambda);
  }
  if (invInertia > 0.0 && orientComp) {
    orientComp.angle += invInertia * rn * lambda;
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

  const massAComp = world.getComponent(entityA, MassComponent);
  const massBComp = world.getComponent(entityB, MassComponent);
  const invMassA = (massAComp && massAComp.mass > 0.0) ? 1.0 / massAComp.mass : 0.0;
  const invMassB = (massBComp && massBComp.mass > 0.0) ? 1.0 / massBComp.mass : 0.0;

  const moiAComp = world.getComponent(entityA, MomentOfInertiaComponent);
  const moiBComp = world.getComponent(entityB, MomentOfInertiaComponent);
  const invInertiaA = moiAComp ? moiAComp.invInertia : 0.0;
  const invInertiaB = moiBComp ? moiBComp.invInertia : 0.0;

  const orientAComp = world.getComponent(entityA, OrientationComponent);
  const orientBComp = world.getComponent(entityB, OrientationComponent);

  const rA = offsetA?.clone?.() ?? new Vector2(0.0, 0.0);
  const rB = offsetB?.clone?.() ?? new Vector2(0.0, 0.0);
  const rnA = rA.x * normalAB.y - rA.y * normalAB.x;
  const rnB = rB.x * normalAB.y - rB.y * normalAB.x;
  const denom = (
    invMassA +
    invMassB +
    invInertiaA * rnA * rnA +
    invInertiaB * rnB * rnB
  );
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
  if (invInertiaA > 0.0 && orientAComp) {
    orientAComp.angle += -invInertiaA * rnA * lambda;
  }
  if (invInertiaB > 0.0 && orientBComp) {
    orientBComp.angle += invInertiaB * rnB * lambda;
  }
  return lambda;
}

function _flipperTipFromState(flipperPos, flipperState) {
  const angle = flipperState.restAngle + flipperState.sign * flipperState.rotation;
  const dir = new Vector2(Math.cos(angle), Math.sin(angle));
  return flipperPos.clone().add(dir, flipperState.length);
}

function _isFiniteVec2(point) {
  return (
    point &&
    Number.isFinite(point.x) &&
    Number.isFinite(point.y)
  );
}

function _orientation2D(a, b, c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

function _segmentsProperlyIntersect(a0, a1, b0, b1, eps = 1e-9) {
  if (
    !_isFiniteVec2(a0) ||
    !_isFiniteVec2(a1) ||
    !_isFiniteVec2(b0) ||
    !_isFiniteVec2(b1)
  ) {
    return false;
  }
  const rX = a1.x - a0.x;
  const rY = a1.y - a0.y;
  const sX = b1.x - b0.x;
  const sY = b1.y - b0.y;
  const rCrossS = (rX * sY) - (rY * sX);
  if (Math.abs(rCrossS) <= eps) {
    return false;
  }

  const qpX = b0.x - a0.x;
  const qpY = b0.y - a0.y;
  const t = ((qpX * sY) - (qpY * sX)) / rCrossS;
  const u = ((qpX * rY) - (qpY * rX)) / rCrossS;
  if (!Number.isFinite(t) || !Number.isFinite(u)) {
    return false;
  }

  // Treat near-endpoint hits as non-crossing to avoid flicker from tiny
  // numerical drift when a cable segment merely touches one endpoint.
  const rLen = Math.hypot(rX, rY);
  const sLen = Math.hypot(sX, sY);
  const endpointLinearMargin = 1e-4;
  const tMargin = Math.min(0.49, endpointLinearMargin / Math.max(rLen, 1e-9));
  const uMargin = Math.min(0.49, endpointLinearMargin / Math.max(sLen, 1e-9));

  return (
    t > tMargin &&
    t < (1.0 - tMargin) &&
    u > uMargin &&
    u < (1.0 - uMargin)
  );
}

function _segmentEndpointAtCurrentPose(world, entityId, attachmentPointWorld) {
  if (!_isFiniteVec2(attachmentPointWorld)) {
    return null;
  }

  const currentPos = world.getComponent(entityId, PositionComponent)?.pos;
  if (!_isFiniteVec2(currentPos)) {
    return attachmentPointWorld.clone();
  }

  const currentAngleComp = world.getComponent(entityId, OrientationComponent);
  const currentAngle = Number.isFinite(currentAngleComp?.angle) ? currentAngleComp.angle : 0.0;

  const linkComp = world.getComponent(entityId, CableLinkComponent);
  const cachedPos = _isFiniteVec2(linkComp?.prevCableAttachmentTimePos)
    ? linkComp.prevCableAttachmentTimePos
    : currentPos;
  const cachedAngle = Number.isFinite(linkComp?.prevCableAttachmentTimeAngle)
    ? linkComp.prevCableAttachmentTimeAngle
    : currentAngle;

  const dx = attachmentPointWorld.x - cachedPos.x;
  const dy = attachmentPointWorld.y - cachedPos.y;
  const cosCached = Math.cos(-cachedAngle);
  const sinCached = Math.sin(-cachedAngle);
  const localX = dx * cosCached - dy * sinCached;
  const localY = dx * sinCached + dy * cosCached;

  const cosCurrent = Math.cos(currentAngle);
  const sinCurrent = Math.sin(currentAngle);
  const worldX = currentPos.x + localX * cosCurrent - localY * sinCurrent;
  const worldY = currentPos.y + localX * sinCurrent + localY * cosCurrent;
  return new Vector2(worldX, worldY);
}

function _collectCableJointSegments(world) {
  const segmentByJointId = new Map();
  const pathEntities = world.query([CablePathComponent]);
  for (const pathId of pathEntities) {
    const path = world.getComponent(pathId, CablePathComponent);
    const halfWidth = Number.isFinite(path?.cableHalfWidth)
      ? Math.max(0.0, path.cableHalfWidth)
      : 0.0;
    if (!(halfWidth > 1e-9)) {
      continue;
    }
    for (const jointId of path?.jointEntities ?? []) {
      const joint = world.getComponent(jointId, CableJointComponent);
      if (!joint) {
        continue;
      }
      const segmentA = _segmentEndpointAtCurrentPose(
        world,
        joint.entityA,
        joint.attachmentPointA_world
      );
      const segmentB = _segmentEndpointAtCurrentPose(
        world,
        joint.entityB,
        joint.attachmentPointB_world
      );
      if (!_isFiniteVec2(segmentA) || !_isFiniteVec2(segmentB)) {
        continue;
      }
      const prev = segmentByJointId.get(jointId);
      if (!prev || halfWidth > prev.halfWidth) {
        segmentByJointId.set(jointId, {
          halfWidth,
          a: segmentA.clone(),
          b: segmentB.clone(),
          entityA: joint.entityA,
          entityB: joint.entityB
        });
      }
    }
  }
  return Array.from(segmentByJointId.values());
}

function _crossingCableHalfWidth(cableSegments, centerA, centerB, pairEntityA, pairEntityB) {
  if (!Array.isArray(cableSegments) || cableSegments.length === 0) {
    return 0.0;
  }
  if (pairEntityA === pairEntityB) {
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
    if (_segmentsProperlyIntersect(centerA, centerB, segment.a, segment.b, 1e-9)) {
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

    const useSectorSupports = _layeringFlag(world, 'layeringCollisionSectorSolvers', true);
    const pinchShareEnabled = _layeringFlag(world, 'layeringCollisionPinchShare', true);
    const cableJointSegments = pinchShareEnabled
      ? _collectCableJointSegments(world)
      : null;

    const borderId = borderEntities.length > 0 ? borderEntities[0] : null;
    const borderPoints = borderId !== null
      ? (world.getComponent(borderId, BorderComponent)?.points ?? [])
      : [];
    const borderRestitution = borderId !== null
      ? world.getComponent(borderId, RestitutionComponent)?.restitution
      : null;
    const borderFriction = borderId !== null
      ? world.getComponent(borderId, CoefficientOfFrictionComponent)?.mu
      : null;

    const candidates = [];
    for (let i = 0; i < ballEntities.length; i++) {
      const ballId = ballEntities[i];
      if (borderPoints.length >= 2) {
        for (let segmentIndex = 0; segmentIndex < borderPoints.length; segmentIndex++) {
          candidates.push({ type: 'border', ballId, segmentIndex });
        }
      }
      for (const obsId of obstacleEntities) {
        candidates.push({ type: 'obstacle', ballId, obsId });
      }
      for (const flipId of flipperEntities) {
        candidates.push({ type: 'flipper', ballId, flipId });
      }
      for (let j = i + 1; j < ballEntities.length; j++) {
        candidates.push({ type: 'ball-ball', ballA: ballId, ballB: ballEntities[j] });
      }
    }

    for (const candidate of candidates) {
      if (candidate.type === 'border') {
        const pBall = world.getComponent(candidate.ballId, PositionComponent)?.pos;
        if (!pBall || borderPoints.length < 2) {
          continue;
        }
        const a = borderPoints[candidate.segmentIndex];
        const b = borderPoints[(candidate.segmentIndex + 1) % borderPoints.length];
        const closest = closestPointOnSegment(pBall, a, b);
        const distSq = pBall.distanceToSq(closest);
        const broadRadius = useSectorSupports
          ? getMaxCollisionRadius(world, candidate.ballId)
          : _getBaseCollisionRadius(world, candidate.ballId);
        if (distSq > (broadRadius + 1e-9) * (broadRadius + 1e-9)) {
          continue;
        }

        const normal = _borderCollisionNormal(pBall, closest, a, b);
        const support = _collisionSupportToward(
          world,
          candidate.ballId,
          normal.clone().scale(-1.0),
          useSectorSupports
        );
        if (!support) {
          continue;
        }
        const dist = Math.sqrt(distSq);
        const penetration = support.projection - dist;
        if (penetration <= 0.0) {
          continue;
        }

        const deltaLambda = _resolveRigidContactSingle(
          world,
          candidate.ballId,
          support.offset,
          normal,
          penetration
        );
        borderContacts.push({
          ball_id: candidate.ballId,
          normal: normal.clone(),
          delta_lambda: deltaLambda,
          ball_contact_radius: support.projection,
          ball_contact_offset: support.offset.clone(),
          restitution: borderRestitution,
          friction: borderFriction,
          border_segment_index: candidate.segmentIndex
        });
        continue;
      }

      if (candidate.type === 'ball-ball') {
        const pA = world.getComponent(candidate.ballA, PositionComponent)?.pos;
        const pB = world.getComponent(candidate.ballB, PositionComponent)?.pos;
        if (!pA || !pB) {
          continue;
        }
        const normalAB = new Vector2().subtractVectors(pB, pA);
        const dSq = normalAB.lengthSq();
        if (dSq <= 1e-12) {
          continue;
        }
        const d = Math.sqrt(dSq);
        normalAB.scale(1.0 / d);

        const betweenHalfWidth = (
          pinchShareEnabled &&
          cableJointSegments &&
          world.hasComponent(candidate.ballA, CableLinkComponent) &&
          world.hasComponent(candidate.ballB, CableLinkComponent)
        )
          ? _crossingCableHalfWidth(
            cableJointSegments,
            pA,
            pB,
            candidate.ballA,
            candidate.ballB
          )
          : 0.0;
        const betweenGapAllowance = betweenHalfWidth > 1e-9 ? (2.0 * betweenHalfWidth) : 0.0;

        const broadA = useSectorSupports
          ? getMaxCollisionRadius(world, candidate.ballA)
          : _getBaseCollisionRadius(world, candidate.ballA);
        const broadB = useSectorSupports
          ? getMaxCollisionRadius(world, candidate.ballB)
          : _getBaseCollisionRadius(world, candidate.ballB);
        if (d > broadA + broadB + betweenGapAllowance + 1e-9) {
          continue;
        }

        const supportA = _collisionSupportToward(world, candidate.ballA, normalAB, useSectorSupports);
        const supportB = _collisionSupportToward(world, candidate.ballB, normalAB.clone().scale(-1.0), useSectorSupports);
        if (!supportA || !supportB) {
          continue;
        }

        let resolvedSupportA = supportA;
        let resolvedSupportB = supportB;
        let pinchShared = false;

        if (betweenHalfWidth > 1e-9) {
          pinchShared = true;
          const baseA = _getBaseCollisionRadius(world, candidate.ballA);
          const baseB = _getBaseCollisionRadius(world, candidate.ballB);
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
        const deltaLambda = _resolveRigidContactPair(
          world,
          candidate.ballA,
          resolvedSupportA.offset,
          candidate.ballB,
          resolvedSupportB.offset,
          normalAB,
          penetration
        );
        ballBallContacts.push({
          ball_a: candidate.ballA,
          ball_b: candidate.ballB,
          normal: normalAB.clone(),
          delta_lambda: deltaLambda,
          penetration,
          radius_a: resolvedSupportA.projection,
          radius_b: resolvedSupportB.projection,
          pinch_shared: pinchShared
        });
        continue;
      }

      if (candidate.type === 'obstacle') {
        const pBall = world.getComponent(candidate.ballId, PositionComponent)?.pos;
        const pObs = world.getComponent(candidate.obsId, PositionComponent)?.pos;
        if (!pBall || !pObs) {
          continue;
        }
        const normal = new Vector2().subtractVectors(pBall, pObs);
        const dSq = normal.lengthSq();
        if (dSq <= 1e-12) {
          continue;
        }
        const d = Math.sqrt(dSq);
        normal.scale(1.0 / d); // obstacle -> ball

        const betweenHalfWidth = (
          pinchShareEnabled &&
          cableJointSegments &&
          world.hasComponent(candidate.ballId, CableLinkComponent) &&
          world.hasComponent(candidate.obsId, CableLinkComponent)
        )
          ? _crossingCableHalfWidth(
            cableJointSegments,
            pBall,
            pObs,
            candidate.ballId,
            candidate.obsId
          )
          : 0.0;
        const betweenGapAllowance = betweenHalfWidth > 1e-9 ? (2.0 * betweenHalfWidth) : 0.0;

        const broadBall = useSectorSupports
          ? getMaxCollisionRadius(world, candidate.ballId)
          : _getBaseCollisionRadius(world, candidate.ballId);
        const broadObs = useSectorSupports
          ? getMaxCollisionRadius(world, candidate.obsId)
          : _getBaseCollisionRadius(world, candidate.obsId);
        if (d > broadBall + broadObs + betweenGapAllowance + 1e-9) {
          continue;
        }

        const supportBall = _collisionSupportToward(
          world,
          candidate.ballId,
          normal.clone().scale(-1.0),
          useSectorSupports
        );
        const supportObs = _collisionSupportToward(world, candidate.obsId, normal, useSectorSupports);
        if (!supportBall || !supportObs) {
          continue;
        }

        let resolvedSupportBall = supportBall;
        let resolvedSupportObs = supportObs;
        let pinchShared = false;
        if (betweenHalfWidth > 1e-9) {
          pinchShared = true;
          const baseBall = _getBaseCollisionRadius(world, candidate.ballId);
          const baseObs = _getBaseCollisionRadius(world, candidate.obsId);
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
        const deltaLambda = _resolveRigidContactSingle(
          world,
          candidate.ballId,
          resolvedSupportBall.offset,
          normal,
          penetration
        );

        const rawBallRadius = getRawCollisionRadius(world, candidate.ballId);
        const rawObsRadius = getRawCollisionRadius(world, candidate.obsId);
        const rawHit = d <= (rawBallRadius + rawObsRadius + 1e-9);

        obstacleContacts.push({
          ball_id: candidate.ballId,
          obs_id: candidate.obsId,
          direction: normal.clone(),
          raw_hit: rawHit,
          delta_lambda: deltaLambda,
          ball_contact_radius: resolvedSupportBall.projection,
          ball_contact_offset: resolvedSupportBall.offset.clone(),
          obstacle_contact_radius: resolvedSupportObs.projection,
          pinch_shared: pinchShared
        });

        const pairKey = `${candidate.ballId}:${candidate.obsId}`;
        if (rawHit) {
          nextObstacleActivePairs.add(pairKey);
        }
        if (rawHit && !obstacleActivePairs.has(pairKey) && candidate.ballId !== grabbed) {
          world.addComponent(candidate.ballId, new ScoredTagComponent());
        }
        continue;
      }

      if (candidate.type === 'flipper') {
        const pBall = world.getComponent(candidate.ballId, PositionComponent)?.pos;
        const pFlip = world.getComponent(candidate.flipId, PositionComponent)?.pos;
        const flipperState = world.getComponent(candidate.flipId, FlipperStateComponent);
        if (!pBall || !pFlip || !flipperState) {
          continue;
        }
        const tip = _flipperTipFromState(pFlip, flipperState);
        const closest = closestPointOnSegment(pBall, pFlip, tip);
        const normal = new Vector2().subtractVectors(pBall, closest); // flipper -> ball
        const dSq = normal.lengthSq();
        if (dSq <= 1e-12) {
          continue;
        }
        const d = Math.sqrt(dSq);
        normal.scale(1.0 / d);

        const broadBall = useSectorSupports
          ? getMaxCollisionRadius(world, candidate.ballId)
          : _getBaseCollisionRadius(world, candidate.ballId);
        const broadFlipper = useSectorSupports
          ? getMaxCollisionRadius(world, candidate.flipId)
          : _getBaseCollisionRadius(world, candidate.flipId);
        if (d > broadBall + broadFlipper + 1e-9) {
          continue;
        }

        const supportBall = _collisionSupportToward(
          world,
          candidate.ballId,
          normal.clone().scale(-1.0),
          useSectorSupports
        );
        if (!supportBall) {
          continue;
        }
        const flipperRadius = useSectorSupports
          ? getMaxCollisionRadius(world, candidate.flipId)
          : _getBaseCollisionRadius(world, candidate.flipId);
        const rSum = supportBall.projection + flipperRadius;
        if (d > rSum + 1e-9) {
          continue;
        }
        const penetration = rSum - d;
        if (penetration <= 0.0) {
          continue;
        }

        const deltaLambda = _resolveRigidContactSingle(
          world,
          candidate.ballId,
          supportBall.offset,
          normal,
          penetration
        );

        flipperContacts.push({
          ball_id: candidate.ballId,
          flip_id: candidate.flipId,
          normal: normal.clone(),
          contact_point_on_flipper: closest.clone(),
          ball_contact_radius: supportBall.projection,
          ball_contact_offset: supportBall.offset.clone(),
          delta_lambda: deltaLambda
        });
      }
    }

    obstacleActivePairs.clear();
    for (const pairKey of nextObstacleActivePairs) {
      obstacleActivePairs.add(pairKey);
    }
  }
}
