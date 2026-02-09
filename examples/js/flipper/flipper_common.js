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

function _effectiveWrappedRadius(world, entityId, baseRadius, normalTowardContact) {
  if (!(baseRadius > 0.0) || !normalTowardContact || normalTowardContact.lengthSq() <= 1e-12) {
    return baseRadius;
  }
  const pos = world.getComponent(entityId, PositionComponent)?.pos;
  if (!pos) {
    return baseRadius;
  }
  const pointOnBody = pos.clone().add(normalTowardContact, baseRadius);
  const wrapExpansion = getHybridEndpointWrapExpansion(world, entityId, pointOnBody);
  return baseRadius + wrapExpansion;
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

            if (minDistSq > r1 * r1) continue;

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
            const penetration = r1 - dist;
            let delta_lambda = 0;
            if (penetration > 0) {
                if (invMass > 0) {
                    p1.add(collisionNormal, penetration);
                    const w_inv = invMass;
                    delta_lambda = penetration / w_inv;
                }
            }

            contacts.push({
                'ball_id': ballId,
                'normal': collisionNormal.clone(),
                'delta_lambda': delta_lambda,
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
        const rSum = r1 + r2;

        if (dSq == 0.0 || dSq > rSum * rSum) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d); // Normalize

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
        const rSum = r1 + r2;

        if (dSq == 0.0 || dSq > rSum * rSum) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d); // Normalize

        // Store contact info for the velocity-based bump system
        contacts.push({ ball_id: ballId, obs_id: obsId, direction: dir.clone() });

        // Resolve penetration
        const corr = rSum - d;
        p1.add(dir, corr);

        const pairKey = `${ballId}:${obsId}`;
        nextActivePairs.add(pairKey);
        const enteredThisFrame = !activePairs.has(pairKey);

        // Velocity resolution is now handled by BallObstacleBumpSystem
        const grabbed = world.getResource('grabbedBall');
        if (enteredThisFrame && ballId !== grabbed) {
          world.addComponent(ballId, new ScoredTagComponent());
        }
        if (collisionDebug) {
          console.debug(
            `[FlipperCollisionDebug] obstacle-contact ball=${ballId} obs=${obsId} ` +
            `entered=${enteredThisFrame} d=${d.toFixed(6)} rSum=${rSum.toFixed(6)}`
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

  update(world, dt) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, RadiusComponent, MassComponent]);
    const flipperEntities = world.query([FlipperTagComponent, PositionComponent, RadiusComponent, FlipperStateComponent]);
    const collisionDebug = world.getResource('flipperCollisionDebug') === true;

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
        const closest = closestPointOnSegment(p1, fp, tip);

        const dir = new Vector2().subtractVectors(p1, closest);
        const dSq = dir.lengthSq();
        if (dSq == 0.0) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d);

        const effectiveBallRadius = _effectiveWrappedRadius(world, ballId, r1, dir.clone().scale(-1.0));
        const rSum = effectiveBallRadius + fr;
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
            'delta_lambda': delta_lambda
        });
        if (collisionDebug) {
          console.debug(
            `[FlipperCollisionDebug] flipper-contact ball=${ballId} flip=${flipId} ` +
            `d=${d.toFixed(6)} rSum=${rSum.toFixed(6)} wrapExtra=${(effectiveBallRadius - r1).toFixed(6)}`
          );
        }
      }
    }
  }
}
