import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  PositionComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
  PrevFinalPosComponent,
  RenderableComponent,
  CoefficientOfFrictionComponent
} from '../../../src/js/cable_joints/ecs.js';
import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent,
  getHybridEndpointWrapExpansion,
  getHybridEndpointRollingRadius,
  getHybridEndpointCamCorners,
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

const FLIPPER_CAM_TRACE_CONFIG_RESOURCE = 'flipperCamTraceConfig';
const FLIPPER_CAM_TRACE_SAMPLES_RESOURCE = 'flipperCamTraceSamples';

function _toNumberOrNull(value) {
  return Number.isFinite(value) ? value : null;
}

export function appendFlipperCamTrace(world, sample) {
  if (!world || !sample) {
    return;
  }
  const cfg = world.getResource(FLIPPER_CAM_TRACE_CONFIG_RESOURCE);
  if (!cfg || cfg.enabled !== true) {
    return;
  }
  if (Number.isInteger(cfg.ballId) && sample.ball_id !== cfg.ballId) {
    return;
  }
  if (Number.isInteger(cfg.flipId) && sample.flip_id !== cfg.flipId) {
    return;
  }
  if (cfg.onlyWrap === true && sample.raw_contact !== false) {
    return;
  }

  let samples = world.getResource(FLIPPER_CAM_TRACE_SAMPLES_RESOURCE);
  if (!Array.isArray(samples)) {
    samples = [];
    world.setResource(FLIPPER_CAM_TRACE_SAMPLES_RESOURCE, samples);
  }

  samples.push(sample);
  const maxSamples = Number.isFinite(cfg.maxSamples) ? Math.max(100, Math.floor(cfg.maxSamples)) : 3000;
  if (samples.length > maxSamples) {
    samples.splice(0, samples.length - maxSamples);
  }

  if (cfg.logToConsole === true) {
    console.debug('[FlipperCamTrace]', sample);
  }
}

export function getEffectiveCollisionRadius(world, entityId, baseRadius, normalTowardContact) {
  if (!(baseRadius > 0.0)) {
    return baseRadius;
  }

  let effectiveRadius = baseRadius;
  if (normalTowardContact && normalTowardContact.lengthSq() > 1e-12) {
    const pos = world.getComponent(entityId, PositionComponent)?.pos;
    if (pos) {
      const pointOnBody = pos.clone().add(normalTowardContact, baseRadius);
      const wrapExpansion = getHybridEndpointWrapExpansion(world, entityId, pointOnBody);
      effectiveRadius = Math.max(effectiveRadius, baseRadius + wrapExpansion);
    }
  }

  // Guard against tangent degeneracy in cable updates: if an endpoint is in
  // rolling mode, the opposite attachment must stay outside that rolling radius.
  const rollingRadius = getHybridEndpointRollingRadius(world, entityId);
  if (Number.isFinite(rollingRadius)) {
    effectiveRadius = Math.max(effectiveRadius, rollingRadius);
  }
  return effectiveRadius;
}

export function getEffectiveCollisionSupportPoint(world, entityId, baseRadius, directionTowardContact) {
  const center = world.getComponent(entityId, PositionComponent)?.pos;
  if (!center) {
    return null;
  }

  let dir = directionTowardContact?.clone() ?? new Vector2(1.0, 0.0);
  if (dir.lengthSq() <= 1e-12) {
    dir = new Vector2(1.0, 0.0);
  } else {
    dir.normalize();
  }

  const radialRadius = getEffectiveCollisionRadius(world, entityId, baseRadius, dir.clone());
  let bestPoint = center.clone().add(dir, radialRadius);
  let bestProjection = radialRadius;
  let source = 'radial';
  let cornerMeta = null;

  const camCorners = getHybridEndpointCamCorners(world, entityId);
  for (const corner of camCorners) {
    for (const endpoint of ['start', 'end']) {
      const point = endpoint === 'start' ? corner.startPoint : corner.endPoint;
      const rel = point.clone().subtract(center);
      const projection = rel.dot(dir);
      if (projection > bestProjection + 1e-9) {
        bestProjection = projection;
        bestPoint = point.clone();
        source = 'corner';
        cornerMeta = {
          pathId: corner.pathId,
          linkIndex: corner.linkIndex,
          endpoint
        };
      }
    }
  }

  return {
    point: bestPoint,
    projection: bestProjection,
    source,
    cornerMeta
  };
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
           const r = this.world.getComponent(b, RadiusComponent).radius + extraClickableRadius;
           const distSq = clickVec.clone().subtract(pos).lengthSq();
           if (distSq <= r * r && distSq < closestDistSq) {
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

           const borderEnts = world.query([BorderComponent]);
           if (borderEnts.length > 0) {
               const borderPoints = world.getComponent(borderEnts[0], BorderComponent).points;
               const rightClick =
                 rightOfLine(clickPos, borderPoints[0], borderPoints[1]) &&
                 rightOfLine(clickPos, borderPoints[1], borderPoints[2]);
               const leftClick =
                 rightOfLine(clickPos, borderPoints[5], borderPoints[6]) &&
                 rightOfLine(clickPos, borderPoints[6], borderPoints[7]);
               if (rightClick) {
                 const flippers = world.query([FlipperStateComponent, PositionComponent]);
                 const flipperPos0 = world.getComponent(flippers[0], PositionComponent).pos;
                 const flipperPos1 = world.getComponent(flippers[1], PositionComponent).pos;
                 if (flipperPos0.x > flipperPos1.x) {
                   world.getComponent(flippers[0], FlipperStateComponent).pressed = true;
                 } else {
                   world.getComponent(flippers[1], FlipperStateComponent).pressed = true;
                 }
               } else if (leftClick) {
                 const flippers = world.query([FlipperStateComponent, PositionComponent]);
                 const flipperPos0 = world.getComponent(flippers[0], PositionComponent).pos;
                 const flipperPos1 = world.getComponent(flippers[1], PositionComponent).pos;
                 if (flipperPos0.x < flipperPos1.x) {
                   world.getComponent(flippers[0], FlipperStateComponent).pressed = true;
                 } else {
                   world.getComponent(flippers[1], FlipperStateComponent).pressed = true;
                 }
               } else {
                 for (const id of flipperEntities) {
                   const pos   = world.getComponent(id, PositionComponent).pos;
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
            const p1Comp = world.getComponent(ballId, PositionComponent);
            const p1 = p1Comp.pos;
            const r1 = world.getComponent(ballId, RadiusComponent).radius;
            const massComp = world.getComponent(ballId, MassComponent);
            const invMass = (massComp && massComp.mass > 0) ? 1.0 / massComp.mass : 0.0;

            let minDistSq = Infinity;
            let closestSegPoint = new Vector2();
            let edgeStart = null;
            let edgeEnd = null;

            for (let i = 0; i < borderPoints.length; i++) {
                const a = borderPoints[i];
                const b = borderPoints[(i + 1) % borderPoints.length];
                const closestPtOnSeg = closestPointOnSegment(p1, a, b);
                const distSq = p1.distanceToSq(closestPtOnSeg);

                if (distSq < minDistSq) {
                    minDistSq = distSq;
                    closestSegPoint.set(closestPtOnSeg);
                    edgeStart = a;
                    edgeEnd = b;
                }
            }

            const ballToClosest = new Vector2().subtractVectors(p1, closestSegPoint);
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

            const dist = Math.sqrt(minDistSq);
            const effectiveRadius = getEffectiveCollisionRadius(world, ballId, r1, collisionNormal.clone().scale(-1.0));
            const penetration = effectiveRadius - dist;
            if (penetration <= 0) {
                continue;
            }
            let delta_lambda = 0;
            if (invMass > 0) {
                p1.add(collisionNormal, penetration);
                const w_inv = invMass;
                delta_lambda = penetration / w_inv;
            }

            contacts.push({
                'ball_id': ballId,
                'normal': collisionNormal.clone(),
                'delta_lambda': delta_lambda,
                'ball_contact_radius': effectiveRadius,
                'restitution': restitution,
                'friction': friction
            });
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
        const r1 = world.getComponent(e1, RadiusComponent).radius;
        const m1Comp = world.getComponent(e1, MassComponent);

        const p2Comp = world.getComponent(e2, PositionComponent);
        const r2 = world.getComponent(e2, RadiusComponent).radius;
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

        const effectiveR1 = getEffectiveCollisionRadius(world, e1, r1, dir.clone());
        const effectiveR2 = getEffectiveCollisionRadius(world, e2, r2, dir.clone().scale(-1.0));
        const rSum = effectiveR1 + effectiveR2;
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
    const collisionDebug = world.getResource('flipperCollisionDebug') === true;

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
      const r1 = world.getComponent(ballId, RadiusComponent).radius;

      for (const obsId of obstacleEntities) {
        if (world.hasComponent(obsId, FlipperTagComponent)) {
          continue;
        }
        const p2 = world.getComponent(obsId, PositionComponent).pos;
        const r2 = world.getComponent(obsId, RadiusComponent).radius;

        const dir = new Vector2().subtractVectors(p1, p2);
        const dSq = dir.lengthSq();
        if (dSq == 0.0) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d); // Normalize

        const effectiveBallRadius = getEffectiveCollisionRadius(world, ballId, r1, dir.clone().scale(-1.0));
        const effectiveObsRadius = getEffectiveCollisionRadius(world, obsId, r2, dir.clone());
        const rSum = effectiveBallRadius + effectiveObsRadius;
        if (d > rSum) continue;
        const rawRSum = r1 + r2;
        const rawContact = d <= rawRSum + 1e-9;

        // Store contact info for the velocity-based bump system
        contacts.push({
          ball_id: ballId,
          obs_id: obsId,
          direction: dir.clone(),
          ball_contact_radius: effectiveBallRadius,
          obs_contact_radius: effectiveObsRadius,
          raw_contact: rawContact
        });

        // Resolve penetration
        const corr = rSum - d;
        p1.add(dir, corr);

        const pairKey = `${ballId}:${obsId}`;
        nextActivePairs.add(pairKey);
        const enteredThisFrame = !activePairs.has(pairKey);

        // Velocity resolution is handled by BallObstacleBumpSystem, but only raw
        // circle-circle hits should trigger bumper push/scoring.
        const grabbed = world.getResource('grabbedBall');
        if (rawContact && enteredThisFrame && ballId !== grabbed) {
          world.addComponent(ballId, new ScoredTagComponent());
        }
        if (collisionDebug) {
          console.debug(
            `[FlipperCollisionDebug] obstacle-contact ball=${ballId} obs=${obsId} ` +
            `entered=${enteredThisFrame} d=${d.toFixed(6)} rSum=${rSum.toFixed(6)} ` +
            `rBall=${effectiveBallRadius.toFixed(6)} rObs=${effectiveObsRadius.toFixed(6)} ` +
            `raw=${rawContact}`
          );
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

  _segmentNormalToward(segStart, segEnd, towardPoint) {
    const tangent = new Vector2().subtractVectors(segEnd, segStart);
    if (tangent.lengthSq() <= 1e-12) {
      return null;
    }
    const normal = new Vector2(-tangent.y, tangent.x).normalize();
    if (towardPoint) {
      const toPoint = towardPoint.clone().subtract(segStart);
      if (toPoint.dot(normal) < 0.0) {
        normal.scale(-1.0);
      }
    }
    return normal;
  }

  _computeBallSupportAgainstSegment(world, ballId, baseRadius, segStart, segEnd, initialNormal) {
    let normal = initialNormal?.clone() ?? this._segmentNormalToward(segStart, segEnd, null);
    if (!normal || normal.lengthSq() <= 1e-12) {
      return null;
    }
    normal.normalize();

    let support = null;
    let closest = null;
    for (let iter = 0; iter < 2; iter++) {
      support = getEffectiveCollisionSupportPoint(
        world,
        ballId,
        baseRadius,
        normal.clone().scale(-1.0)
      );
      if (!support) {
        return null;
      }
      closest = closestPointOnSegment(support.point, segStart, segEnd);
      const separation = new Vector2().subtractVectors(support.point, closest);
      if (separation.lengthSq() > 1e-12) {
        normal = separation.clone().normalize();
      } else {
        const fallback = this._segmentNormalToward(segStart, segEnd, support.point);
        if (!fallback) {
          return null;
        }
        normal = fallback;
      }
    }

    if (!support || !closest) {
      return null;
    }

    const finalSeparation = new Vector2().subtractVectors(support.point, closest);
    const separationDistance = finalSeparation.length();
    if (separationDistance > 1e-12) {
      normal = finalSeparation.clone().scale(1.0 / separationDistance);
    } else {
      const fallback = this._segmentNormalToward(segStart, segEnd, support.point);
      if (!fallback) {
        return null;
      }
      normal = fallback;
    }

    return {
      support,
      closest,
      normal,
      separationDistance
    };
  }

  update(world, dt) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    const flipperEntities = world.query([FlipperTagComponent, PositionComponent, RadiusComponent, FlipperStateComponent]);
    const collisionDebug = world.getResource('flipperCollisionDebug') === true;
    const collisionWarnings = world.getResource('flipperCollisionWarnings') === true;
    const contactTuning = world.getResource('flipperContactTuning') || {};
    const smoothWrapRadiusOnset = contactTuning.smoothWrapRadiusOnset === true;
    const wrapRadiusRiseRate = Number.isFinite(contactTuning.wrapRadiusRiseRate) ? Math.max(0.0, contactTuning.wrapRadiusRiseRate) : 0.01;
    const wrapRadiusFallRate = Number.isFinite(contactTuning.wrapRadiusFallRate) ? Math.max(0.0, contactTuning.wrapRadiusFallRate) : 0.05;
    const softWrapEnhancedContacts = contactTuning.softWrapEnhancedContacts === true;
    const wrapEnhancedCorrectionFraction = Number.isFinite(contactTuning.wrapEnhancedCorrectionFraction)
      ? Math.max(0.0, Math.min(1.0, contactTuning.wrapEnhancedCorrectionFraction))
      : 0.2;
    const maxWrapEnhancedCorrection = Number.isFinite(contactTuning.maxWrapEnhancedCorrection)
      ? Math.max(0.0, contactTuning.maxWrapEnhancedCorrection)
      : 0.0015;
    const softRawEntryContacts = contactTuning.softRawEntryContacts === true;
    const rawEntryCorrectionFraction = Number.isFinite(contactTuning.rawEntryCorrectionFraction)
      ? Math.max(0.0, Math.min(1.0, contactTuning.rawEntryCorrectionFraction))
      : 0.12;
    const maxRawEntryCorrection = Number.isFinite(contactTuning.maxRawEntryCorrection)
      ? Math.max(0.0, contactTuning.maxRawEntryCorrection)
      : 0.0008;
    const previousWrapRampResource = world.getResource('flipperWrapRadiusRamp');
    const previousWrapRamp = (previousWrapRampResource instanceof Map) ? previousWrapRampResource : new Map();
    const nextWrapRamp = new Map();
    const traceStep = (world.getResource('flipperCamTraceStep') ?? 0) + 1;
    world.setResource('flipperCamTraceStep', traceStep);
    const pinchContacts = world.getResource('cablePinchContacts');
    const pinchedPairs = new Set();
    if (Array.isArray(pinchContacts)) {
      for (const pinch of pinchContacts) {
        const a = pinch.entityA;
        const b = pinch.entityB;
        if (!Number.isInteger(a) || !Number.isInteger(b)) {
          continue;
        }
        const low = Math.min(a, b);
        const high = Math.max(a, b);
        pinchedPairs.add(`${low}:${high}`);
      }
    }
    const previousContacts = world.getResource('flipper_prev_contact_state');
    const previousByPair = (previousContacts instanceof Map) ? previousContacts : new Map();
    const nextByPair = new Map();

    let contacts = world.getResource('ball_flipper_contacts');
    if (!contacts) {
        contacts = [];
        world.setResource('ball_flipper_contacts', contacts);
    }
    contacts.length = 0;

    for (const ballId of ballEntities) {
      const p1Comp = world.getComponent(ballId, PositionComponent);
      const p1 = p1Comp.pos;
      const r1 = world.getComponent(ballId, RadiusComponent).radius;
      const massComp = world.getComponent(ballId, MassComponent);
      const invMass = (massComp && massComp.mass > 0) ? 1.0 / massComp.mass : 0.0;

      for (const flipId of flipperEntities) {
        const fp = world.getComponent(flipId, PositionComponent).pos;
        const fr = world.getComponent(flipId, RadiusComponent).radius;
        const fs = world.getComponent(flipId, FlipperStateComponent);

        const tip = this._getFlipperTip(fp, fs);
        const closestToCenter = closestPointOnSegment(p1, fp, tip);
        const centerToSegment = new Vector2().subtractVectors(p1, closestToCenter);
        const centerDistance = centerToSegment.length();
        let initialNormal = null;
        if (centerDistance > 1e-12) {
          initialNormal = centerToSegment.clone().scale(1.0 / centerDistance);
        } else {
          initialNormal = this._segmentNormalToward(fp, tip, p1);
        }
        if (!initialNormal) {
          continue;
        }

        const supportData = this._computeBallSupportAgainstSegment(
          world,
          ballId,
          r1,
          fp,
          tip,
          initialNormal
        );
        if (!supportData) {
          continue;
        }
        const { support, closest, normal, separationDistance } = supportData;
        const effectiveFlipperRadius = getEffectiveCollisionRadius(world, flipId, fr, normal.clone());
        const corrTarget = effectiveFlipperRadius - separationDistance;
        if (corrTarget <= 0.0) {
          continue;
        }
        const pairKey = `${ballId}:${flipId}`;
        const prevPair = previousByPair.get(pairKey);
        const rawContact = centerDistance <= (r1 + fr + 1e-9);
        const rawEntered = rawContact && !(prevPair?.raw === true);
        const ballContactOffsetTarget = support.point.clone().subtract(p1);
        const ballContactRadiusTarget = Math.max(r1, Math.max(0.0, -ballContactOffsetTarget.dot(normal)));
        const targetWrapExtra = Math.max(0.0, ballContactRadiusTarget - r1);
        const previousWrapExtra = Math.max(0.0, previousWrapRamp.get(pairKey) ?? 0.0);
        let wrapExtra = targetWrapExtra;
        if (smoothWrapRadiusOnset) {
          const stepDt = Number.isFinite(dt) ? Math.max(0.0, dt) : 0.0;
          const rate = targetWrapExtra >= previousWrapExtra ? wrapRadiusRiseRate : wrapRadiusFallRate;
          const maxDelta = rate * stepDt;
          if (maxDelta > 0.0) {
            const delta = targetWrapExtra - previousWrapExtra;
            const clampedDelta = Math.max(-maxDelta, Math.min(maxDelta, delta));
            wrapExtra = previousWrapExtra + clampedDelta;
          }
        }
        wrapExtra = Math.max(0.0, Math.min(targetWrapExtra, wrapExtra));
        nextWrapRamp.set(pairKey, wrapExtra);

        const ballContactRadius = r1 + wrapExtra;
        let ballContactOffset = ballContactOffsetTarget.clone();
        if (ballContactRadiusTarget > 1e-9) {
          const scale = Math.max(0.0, Math.min(1.0, ballContactRadius / ballContactRadiusTarget));
          ballContactOffset.scale(scale);
        }
        let corr = Math.max(0.0, corrTarget - (ballContactRadiusTarget - ballContactRadius));
        if (corr <= 0.0) {
          continue;
        }
        const wrapEnhanced = ballContactRadius > (r1 + 1e-9);
        if (wrapEnhanced && softWrapEnhancedContacts) {
          corr = Math.min(corr * wrapEnhancedCorrectionFraction, maxWrapEnhancedCorrection);
          if (corr <= 0.0) {
            continue;
          }
        }
        if (rawEntered && softRawEntryContacts) {
          corr = Math.min(corr * rawEntryCorrectionFraction, maxRawEntryCorrection);
          if (corr <= 0.0) {
            continue;
          }
        }

        if (invMass > 0) {
            p1.add(normal, corr);
        }

        let delta_lambda = 0;
        if (invMass > 0) {
            const w_inv = invMass;
            delta_lambda = corr / w_inv;
        }
        const flipperSurfacePoint = closest.clone().add(normal, effectiveFlipperRadius);
        const lowPair = Math.min(ballId, flipId);
        const highPair = Math.max(ballId, flipId);
        const pinchPairActive = pinchedPairs.has(`${lowPair}:${highPair}`);

        contacts.push({
            'ball_id': ballId,
            'flip_id': flipId,
            'normal': normal.clone(),
            'contact_point_on_flipper': flipperSurfacePoint,
            'delta_lambda': delta_lambda,
            'ball_contact_radius': ballContactRadius,
            'ball_raw_radius': r1,
            'wrap_enhanced': wrapEnhanced,
            'ball_contact_offset': ballContactOffset,
            'raw_contact': rawContact,
            'raw_entered': rawEntered,
            'trace_step': traceStep,
            'support_source': support.source,
            'pinch_pair_active': pinchPairActive
        });

        appendFlipperCamTrace(world, {
          type: 'position_contact',
          step: traceStep,
          ball_id: ballId,
          flip_id: flipId,
          raw_contact: rawContact,
          raw_entered: rawEntered,
          wrap_enhanced: wrapEnhanced,
          support_source: support.source,
          support_corner: support.cornerMeta ?? null,
          pinch_pair_active: pinchPairActive,
          wrap_extra_target: _toNumberOrNull(targetWrapExtra),
          wrap_extra_applied: _toNumberOrNull(wrapExtra),
          center_distance: _toNumberOrNull(centerDistance),
          support_separation: _toNumberOrNull(separationDistance),
          correction: _toNumberOrNull(corr),
          delta_lambda: _toNumberOrNull(delta_lambda),
          normal_x: _toNumberOrNull(normal.x),
          normal_y: _toNumberOrNull(normal.y),
          ball_center_x: _toNumberOrNull(p1.x),
          ball_center_y: _toNumberOrNull(p1.y),
          support_x: _toNumberOrNull(support.point.x),
          support_y: _toNumberOrNull(support.point.y),
          contact_on_flipper_x: _toNumberOrNull(flipperSurfacePoint.x),
          contact_on_flipper_y: _toNumberOrNull(flipperSurfacePoint.y),
          ball_contact_offset_x: _toNumberOrNull(ballContactOffset.x),
          ball_contact_offset_y: _toNumberOrNull(ballContactOffset.y),
          ball_contact_radius: _toNumberOrNull(ballContactRadius),
          ball_contact_radius_target: _toNumberOrNull(ballContactRadiusTarget),
          ball_radius_raw: _toNumberOrNull(r1),
          flipper_radius_effective: _toNumberOrNull(effectiveFlipperRadius),
          flipper_radius_raw: _toNumberOrNull(fr),
        });

        if (collisionWarnings && prevPair && !rawContact) {
          const radiusJump = Math.abs(ballContactRadius - prevPair.ballRadius);
          const corrJump = Math.abs(corr - prevPair.corr);
          const normalDot = prevPair.normal.dot(normal);
          const sourceChanged = prevPair.source !== support.source;
          if (radiusJump > 0.01 || corrJump > 0.01 || normalDot < 0.95 || sourceChanged) {
            console.warn(
              `[FlipperCollisionWarn] wrap-only contact jump pair=${pairKey} ` +
              `source=${prevPair.source}->${support.source} ` +
              `radiusJump=${radiusJump.toFixed(6)} corrJump=${corrJump.toFixed(6)} normalDot=${normalDot.toFixed(6)} ` +
              `rBall=${ballContactRadius.toFixed(6)} corr=${corr.toFixed(6)}`
            );
          }
        }
        nextByPair.set(pairKey, {
          ballRadius: ballContactRadius,
          corr,
          normal: normal.clone(),
          raw: rawContact,
          source: support.source
        });
        if (collisionDebug) {
          console.debug(
            `[FlipperCollisionDebug] flipper-contact ball=${ballId} flip=${flipId} ` +
            `source=${support.source} dCenter=${centerDistance.toFixed(6)} sep=${separationDistance.toFixed(6)} ` +
            `rBall=${ballContactRadius.toFixed(6)} rFlip=${effectiveFlipperRadius.toFixed(6)} ` +
            `corr=${corr.toFixed(6)} raw=${rawContact}`
          );
        }
      }
    }
    world.setResource('flipper_prev_contact_state', nextByPair);
    world.setResource('flipperWrapRadiusRamp', nextWrapRamp);
  }
}
