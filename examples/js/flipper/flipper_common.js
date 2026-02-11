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
  const sectorComp = world.getComponent(entityId, CircleSectorComponent);
  if (sectorComp && Number.isFinite(sectorComp.radius)) {
    radius = Math.max(radius, sectorComp.radius);
  }
  return radius;
}

function _normalizedDirection(vec, fallback = new Vector2(1.0, 0.0)) {
  if (!vec || vec.lengthSq() <= 1e-12) {
    return fallback.clone();
  }
  return vec.clone().normalize();
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
  const sectorComp = world.getComponent(entityId, CircleSectorComponent);
  if (
    sectorComp &&
    Number.isFinite(sectorComp.radius) &&
    sectorComp.radius > baseRadius + 1e-9
  ) {
    const angle = Math.atan2(dir.y, dir.x);
    if (_isAngleInSector(angle, sectorComp.startAngle, sectorComp.endAngle, sectorComp.cw === true)) {
      if (sectorComp.radius > projection + 1e-9) {
        projection = sectorComp.radius;
        offset = dir.clone().scale(projection);
      }
    }

    const cornerAngles = [sectorComp.startAngle, sectorComp.endAngle];
    for (const cornerAngle of cornerAngles) {
      const cornerOffset = new Vector2(
        Math.cos(cornerAngle) * sectorComp.radius,
        Math.sin(cornerAngle) * sectorComp.radius
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

const LAYER_RADIUS_RAMP_ANGLE = (2.0 * Math.PI) / 100.0;

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

function _smoothedSectorRadius(rawRadius, decomposition, halfWidth, layerStep, span) {
  if (!(halfWidth > 1e-9)) {
    return 0.0;
  }
  // The sector should represent the same outer support envelope as overlay-radius
  // once the arc closes a full layer.
  const targetRadius = (decomposition?.partialRadius ?? 0.0) + halfWidth;
  if (!Number.isFinite(targetRadius) || !(targetRadius > 1e-9)) {
    return 0.0;
  }
  if (!(layerStep > 1e-9)) {
    return targetRadius;
  }
  if (!(LAYER_RADIUS_RAMP_ANGLE > 1e-9)) {
    return targetRadius;
  }
  const spanValue = Number.isFinite(span) ? Math.max(0.0, span) : 0.0;
  const rampAlpha = Math.min(1.0, spanValue / LAYER_RADIUS_RAMP_ANGLE);

  // Ramp each new layer from the support radius of completed layers to the support
  // radius of the current partial layer. This keeps sector->overlay transitions
  // continuous when a partial layer closes into a full one.
  const fullLayers = Math.max(0, decomposition?.fullLayers ?? 0);
  const rampStartRadius = rawRadius + layerStep * fullLayers;

  return rampStartRadius + (targetRadius - rampStartRadius) * rampAlpha;
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
      for (const entityId of world.query([CircleCamTag])) {
        world.removeComponent(entityId, CircleCamTag);
      }
      return;
    }

    const overlayEnabled = _layeringFlag(world, 'layeringCollisionOverlayRadius', true);
    const sectorEnabled = _layeringFlag(world, 'layeringCollisionCircleSectors', true);

    const overlayByEntity = new Map();
    const sectorByEntity = new Map();
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

        if (overlayEnabled) {
          const baseOverlayRadius = rawRadius + layerStep * decomposition.fullLayers;
          let overlayRadius = baseOverlayRadius;
          if (decomposition.partialLength > 1e-9 && decomposition.partialRadius > 1e-9) {
            const span = decomposition.partialLength / decomposition.partialRadius;
            const closingBlend = _closingOverlayBlend(span);
            if (closingBlend > 1e-9) {
              overlayRadius = baseOverlayRadius + layerStep * closingBlend;
            }
          }
          if (overlayRadius > rawRadius + 1e-9) {
            const prev = overlayByEntity.get(entityId) ?? 0.0;
            overlayByEntity.set(entityId, Math.max(prev, overlayRadius));
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
          const smoothedRadius = _smoothedSectorRadius(
            rawRadius,
            decomposition,
            halfWidth,
            layerStep,
            span
          );
          if (!(smoothedRadius > 1e-9)) {
            continue;
          }
          const cw = path.cw[linkIndex] === true;
          const endAngle = cw ? (startAngle - span) : (startAngle + span);
          const sector = {
            radius: smoothedRadius,
            startAngle,
            endAngle,
            cw
          };
          const prev = sectorByEntity.get(entityId);
          if (!prev || sector.radius > prev.radius) {
            sectorByEntity.set(entityId, sector);
          }
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

    for (const [entityId, sector] of sectorByEntity.entries()) {
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
    }
    for (const entityId of world.query([CircleSectorComponent])) {
      if (!sectorByEntity.has(entityId)) {
        world.removeComponent(entityId, CircleSectorComponent);
      }
    }

    const camEntities = new Set([...overlayByEntity.keys(), ...sectorByEntity.keys()]);
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

export class PBDBallBorderCollisions {
  update(world, dt) {
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

    if (borderPoints.length >= 2) {
        for (const ballId of ballEntities) {
            for (let i = 0; i < borderPoints.length; i++) {
                const posComp = world.getComponent(ballId, PositionComponent);
                const p1 = posComp?.pos;
                if (!p1) {
                  break;
                }
                const a = borderPoints[i];
                const b = borderPoints[(i + 1) % borderPoints.length];
                const closestPtOnSeg = closestPointOnSegment(p1, a, b);
                const distSq = p1.distanceToSq(closestPtOnSeg);
                const r1 = _getBaseCollisionRadius(world, ballId);
                if (distSq > (r1 + 1e-9) * (r1 + 1e-9)) {
                  continue;
                }

                const collisionNormal = _borderCollisionNormal(p1, closestPtOnSeg, a, b);
                const dist = Math.sqrt(distSq);
                const penetration = r1 - dist;
                if (penetration <= 0.0) {
                  continue;
                }
                const contactOffset = collisionNormal.clone().scale(-r1);
                const delta_lambda = _resolveRigidContactSingle(
                  world,
                  ballId,
                  contactOffset,
                  collisionNormal,
                  penetration
                );

                contacts.push({
                    ball_id: ballId,
                    normal: collisionNormal.clone(),
                    delta_lambda,
                    ball_contact_radius: r1,
                    ball_contact_offset: contactOffset,
                    restitution,
                    friction,
                    border_segment_index: i
                });
            }
        }
    }
  }
}

export class PBDBallBallCollisions {
  runInPause = false;
  update(world, dt) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    for (let i = 0; i < ballEntities.length; i++) {
      for (let j = i + 1; j < ballEntities.length; j++) {
        const e1 = ballEntities[i];
        const e2 = ballEntities[j];

        const p1Comp = world.getComponent(e1, PositionComponent);
        const m1Comp = world.getComponent(e1, MassComponent);

        const p2Comp = world.getComponent(e2, PositionComponent);
        const m2Comp = world.getComponent(e2, MassComponent);

        const p1 = p1Comp.pos;
        const m1 = m1Comp ? m1Comp.mass : 0.0;
        const p2 = p2Comp.pos;
        const m2 = m2Comp ? m2Comp.mass : 0.0;

        const dir = new Vector2().subtractVectors(p2, p1);
        const dSq = dir.lengthSq();
        if (dSq == 0.0) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d); // Normalize
        const broadR1 = getMaxCollisionRadius(world, e1);
        const broadR2 = getMaxCollisionRadius(world, e2);
        if (d > broadR1 + broadR2) continue;

        const r1 = _getBaseCollisionRadius(world, e1);
        const r2 = _getBaseCollisionRadius(world, e2);
        const rSum = r1 + r2;
        if (d > rSum) continue;

        // Resolve penetration
        const penetration = rSum - d;
        const invMass1 = (m1 > 0) ? 1.0 / m1 : 0.0;
        const invMass2 = (m2 > 0) ? 1.0 / m2 : 0.0;
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
  update(world, dt) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent]);
    const obstacleEntities = world.query([ObstacleTagComponent, PositionComponent, RadiusComponent, ObstaclePushComponent]);

    let contacts = world.getResource('ball_obstacle_contacts');
    if (!contacts) {
        contacts = [];
        world.setResource('ball_obstacle_contacts', contacts);
    }
    contacts.length = 0; // Clear existing contacts
    let activePairs = world.getResource('ball_obstacle_active_pairs');
    if (!(activePairs instanceof Set)) {
      activePairs = new Set();
      world.setResource('ball_obstacle_active_pairs', activePairs);
    }
    const nextActivePairs = new Set();

    for (const ballId of ballEntities) {
      const p1 = world.getComponent(ballId, PositionComponent).pos;
      const rawBallRadius = getRawCollisionRadius(world, ballId);

      for (const obsId of obstacleEntities) {
        const p2 = world.getComponent(obsId, PositionComponent).pos;
        const rawObsRadius = getRawCollisionRadius(world, obsId);

        const dir = new Vector2().subtractVectors(p1, p2);
        const dSq = dir.lengthSq();
        if (dSq == 0.0) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d); // Normalize
        const r1 = _getBaseCollisionRadius(world, ballId);
        const r2 = _getBaseCollisionRadius(world, obsId);
        const rSum = r1 + r2;
        if (d > rSum) continue;
        const rawHit = d <= (rawBallRadius + rawObsRadius + 1e-9);

        // Store contact info for the velocity-based bump system
        contacts.push({
          ball_id: ballId,
          obs_id: obsId,
          direction: dir.clone(),
          raw_hit: rawHit
        });

        // Resolve penetration
        const corr = rSum - d;
        p1.add(dir, corr);
        const pairKey = `${ballId}:${obsId}`;
        nextActivePairs.add(pairKey);

        // Velocity resolution is now handled by BallObstacleBumpSystem
        const grabbed = world.getResource('grabbedBall');
        if (rawHit && !activePairs.has(pairKey) && ballId !== grabbed) {
          world.addComponent(ballId, new ScoredTagComponent());
        }
      }
    }
    activePairs.clear();
    for (const pairKey of nextActivePairs) {
      activePairs.add(pairKey);
    }
  }
}

export class PBDBallFlipperCollisions {
  _getFlipperTip(flipperPos, flipperState) {
    const angle = flipperState.restAngle + flipperState.sign * flipperState.rotation;
    const dir = new Vector2(Math.cos(angle), Math.sin(angle));
    return flipperPos.clone().add(dir, flipperState.length);
  }

  update(world, dt) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    const flipperEntities = world.query([FlipperTagComponent, PositionComponent, RadiusComponent, FlipperStateComponent]);

    let contacts = world.getResource('ball_flipper_contacts');
    if (!contacts) {
        contacts = [];
        world.setResource('ball_flipper_contacts', contacts);
    }
    contacts.length = 0;

    for (const ballId of ballEntities) {
      const p1Comp = world.getComponent(ballId, PositionComponent);
      const p1 = p1Comp.pos;
      const massComp = world.getComponent(ballId, MassComponent);
      const invMass = (massComp && massComp.mass > 0) ? 1.0 / massComp.mass : 0.0;

      for (const flipId of flipperEntities) {
        const fp = world.getComponent(flipId, PositionComponent).pos;
        const fs = world.getComponent(flipId, FlipperStateComponent);

        const tip = this._getFlipperTip(fp, fs);
        const closest = closestPointOnSegment(p1, fp, tip);

        const dir = new Vector2().subtractVectors(p1, closest);
        const dSq = dir.lengthSq();
        if (dSq == 0.0) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d);
        const broadR1 = getMaxCollisionRadius(world, ballId);
        const broadFr = getMaxCollisionRadius(world, flipId);
        if (d > broadR1 + broadFr) continue;

        const r1 = _getBaseCollisionRadius(world, ballId);
        const fr = _getBaseCollisionRadius(world, flipId);
        const rSum = r1 + fr;
        if (d > rSum) continue;

        const corr = rSum - d;
        if (invMass > 0) {
            p1.add(dir, corr);
        }

        let delta_lambda = 0;
        if (invMass > 0) {
            const w_inv = invMass;
            delta_lambda = corr / w_inv;
        }

        contacts.push({
            'ball_id': ballId,
            'flip_id': flipId,
            'normal': dir.clone(),
            'contact_point_on_flipper': closest.clone(),
            'ball_contact_radius': r1,
            'delta_lambda': delta_lambda
        });
      }
    }
  }
}

function _findClosestBorderSegment(borderPoints, point) {
  if (!Array.isArray(borderPoints) || borderPoints.length < 2) {
    return null;
  }
  let minDistSq = Infinity;
  let closestPoint = null;
  let edgeStart = null;
  let edgeEnd = null;
  for (let i = 0; i < borderPoints.length; i++) {
    const a = borderPoints[i];
    const b = borderPoints[(i + 1) % borderPoints.length];
    const closest = closestPointOnSegment(point, a, b);
    const distSq = point.distanceToSq(closest);
    if (distSq < minDistSq) {
      minDistSq = distSq;
      closestPoint = closest;
      edgeStart = a;
      edgeEnd = b;
    }
  }
  if (!closestPoint || !edgeStart || !edgeEnd) {
    return null;
  }
  return {
    minDistSq,
    closestPoint,
    edgeStart,
    edgeEnd
  };
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

export class PBDBorderCircleSectorCollisions {
  runInPause = false;

  update(world, _dt_unused) {
    if (!_layeringFlag(world, 'layeringCollisionSectorSolvers', true)) {
      return;
    }
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent, CircleSectorComponent]);
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
    if (!Array.isArray(contacts)) {
      contacts = [];
      world.setResource('ball_border_contacts', contacts);
    }

    for (const ballId of ballEntities) {
      for (let i = 0; i < borderPoints.length; i++) {
        const posComp = world.getComponent(ballId, PositionComponent);
        const p1 = posComp?.pos;
        if (!p1) break;

        const a = borderPoints[i];
        const b = borderPoints[(i + 1) % borderPoints.length];
        const closestPoint = closestPointOnSegment(p1, a, b);
        const minDistSq = p1.distanceToSq(closestPoint);
        const dist = Math.sqrt(minDistSq);
        const normal = _borderCollisionNormal(p1, closestPoint, a, b);
        const towardBorder = normal.clone().scale(-1.0);
        const support = _compositeSupportToward(world, ballId, towardBorder);
        if (!support) continue;
        if (support.projection <= support.baseRadius + 1e-9) continue;
        const penetration = support.projection - dist;
        if (penetration <= 0.0) continue;

        const deltaLambda = _resolveRigidContactSingle(
          world,
          ballId,
          support.offset,
          normal,
          penetration
        );

        contacts.push({
          ball_id: ballId,
          normal: normal.clone(),
          delta_lambda: deltaLambda,
          ball_contact_radius: support.projection,
          ball_contact_offset: support.offset.clone(),
          restitution,
          friction,
          border_segment_index: i
        });
      }
    }
  }
}

export class PBDBallCircleSectorCollisions {
  runInPause = false;

  update(world, _dt_unused) {
    if (!_layeringFlag(world, 'layeringCollisionSectorSolvers', true)) {
      return;
    }
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    for (let i = 0; i < ballEntities.length; i++) {
      for (let j = i + 1; j < ballEntities.length; j++) {
        const e1 = ballEntities[i];
        const e2 = ballEntities[j];

        const p1 = world.getComponent(e1, PositionComponent)?.pos;
        const p2 = world.getComponent(e2, PositionComponent)?.pos;
        if (!p1 || !p2) continue;

        const dir = new Vector2().subtractVectors(p2, p1);
        const dSq = dir.lengthSq();
        if (dSq <= 1e-12) continue;
        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d);

        const support1 = _compositeSupportToward(world, e1, dir);
        const support2 = _compositeSupportToward(world, e2, dir.clone().scale(-1.0));
        if (!support1 || !support2) continue;
        if (
          support1.projection <= support1.baseRadius + 1e-9 &&
          support2.projection <= support2.baseRadius + 1e-9
        ) {
          continue;
        }

        const rSum = support1.projection + support2.projection;
        if (d > rSum) continue;
        const penetration = rSum - d;
        _resolveRigidContactPair(
          world,
          e1,
          support1.offset,
          e2,
          support2.offset,
          dir,
          penetration
        );
      }
    }
  }
}

export class PBDObstacleCircleSectorCollisions {
  runInPause = false;

  update(world, _dt_unused) {
    if (!_layeringFlag(world, 'layeringCollisionSectorSolvers', true)) {
      return;
    }
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    const obstacleEntities = world.query([ObstacleTagComponent, PositionComponent, RadiusComponent, ObstaclePushComponent]);

    let contacts = world.getResource('ball_obstacle_contacts');
    if (!Array.isArray(contacts)) {
      contacts = [];
      world.setResource('ball_obstacle_contacts', contacts);
    }

    for (const ballId of ballEntities) {
      const pBall = world.getComponent(ballId, PositionComponent)?.pos;
      if (!pBall) continue;

      for (const obsId of obstacleEntities) {
        const pObs = world.getComponent(obsId, PositionComponent)?.pos;
        if (!pObs) continue;

        const normal = new Vector2().subtractVectors(pBall, pObs);
        const dSq = normal.lengthSq();
        if (dSq <= 1e-12) continue;
        const d = Math.sqrt(dSq);
        normal.scale(1.0 / d); // obstacle -> ball

        const supportBall = _compositeSupportToward(world, ballId, normal.clone().scale(-1.0));
        const supportObs = _compositeSupportToward(world, obsId, normal);
        if (!supportBall || !supportObs) continue;
        if (
          supportBall.projection <= supportBall.baseRadius + 1e-9 &&
          supportObs.projection <= supportObs.baseRadius + 1e-9
        ) {
          continue;
        }

        const rSum = supportBall.projection + supportObs.projection;
        if (d > rSum) continue;
        const penetration = rSum - d;
        if (penetration <= 0.0) continue;
        _resolveRigidContactSingle(world, ballId, supportBall.offset, normal, penetration);

        const existing = contacts.find((c) => c.ball_id === ballId && c.obs_id === obsId);
        if (!existing) {
          contacts.push({
            ball_id: ballId,
            obs_id: obsId,
            direction: normal.clone(),
            raw_hit: false
          });
        }
      }
    }
  }
}

export class FlipperCircleSectorCollisions {
  runInPause = false;

  _getFlipperTip(flipperPos, flipperState) {
    const angle = flipperState.restAngle + flipperState.sign * flipperState.rotation;
    const dir = new Vector2(Math.cos(angle), Math.sin(angle));
    return flipperPos.clone().add(dir, flipperState.length);
  }

  update(world, _dt_unused) {
    if (!_layeringFlag(world, 'layeringCollisionSectorSolvers', true)) {
      return;
    }
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    const flipperEntities = world.query([FlipperTagComponent, PositionComponent, RadiusComponent, FlipperStateComponent]);

    let contacts = world.getResource('ball_flipper_contacts');
    if (!Array.isArray(contacts)) {
      contacts = [];
      world.setResource('ball_flipper_contacts', contacts);
    }

    for (const ballId of ballEntities) {
      const pBall = world.getComponent(ballId, PositionComponent)?.pos;
      if (!pBall) continue;

      for (const flipId of flipperEntities) {
        const fp = world.getComponent(flipId, PositionComponent)?.pos;
        const fs = world.getComponent(flipId, FlipperStateComponent);
        if (!fp || !fs) continue;

        const tip = this._getFlipperTip(fp, fs);
        const closest = closestPointOnSegment(pBall, fp, tip);
        const normal = new Vector2().subtractVectors(pBall, closest); // flipper segment -> ball
        const dSq = normal.lengthSq();
        if (dSq <= 1e-12) continue;
        const d = Math.sqrt(dSq);
        normal.scale(1.0 / d);

        const supportBall = _compositeSupportToward(world, ballId, normal.clone().scale(-1.0));
        if (!supportBall) continue;
        if (supportBall.projection <= supportBall.baseRadius + 1e-9) continue;

        const flipperRadius = getMaxCollisionRadius(world, flipId);
        const rSum = supportBall.projection + flipperRadius;
        if (d > rSum) continue;
        const penetration = rSum - d;
        if (penetration <= 0.0) continue;

        const deltaLambda = _resolveRigidContactSingle(
          world,
          ballId,
          supportBall.offset,
          normal,
          penetration
        );

        const existing = contacts.find((c) => c.ball_id === ballId && c.flip_id === flipId);
        if (existing) {
          existing.normal = normal.clone();
          existing.contact_point_on_flipper = closest.clone();
          existing.ball_contact_radius = supportBall.projection;
          existing.ball_contact_offset = supportBall.offset.clone();
          existing.delta_lambda = (Number.isFinite(existing.delta_lambda) ? existing.delta_lambda : 0.0) + deltaLambda;
        } else {
          contacts.push({
            ball_id: ballId,
            flip_id: flipId,
            normal: normal.clone(),
            contact_point_on_flipper: closest.clone(),
            ball_contact_radius: supportBall.projection,
            ball_contact_offset: supportBall.offset.clone(),
            delta_lambda: deltaLambda
          });
        }
      }
    }
  }
}
