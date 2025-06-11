import Vector2 from './vector2.js';

import {
  PositionComponent,
  VelocityComponent,
  GravityAffectedComponent,
  BallTagComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
  PrevFinalPosComponent,
  OrientationComponent,
  MomentOfInertiaComponent,
  AngularVelocityComponent,
  CoefficientOfFrictionComponent,
  ObstacleTagComponent,
  ObstaclePushComponent,
  ScoredTagComponent,
  DistanceConstraintComponent,
  PrevFinalOrientationComponent,
  FlipperTagComponent,
  FlipperTipComponent,
  FlipperStateComponent,
  BorderComponent
} from './ecs.js';

import {
  closestPointOnSegment
} from './geometry.js';


export class PrevFinalOrientationSystem {
    runInPause = false;
    update(world, dt) {
        const entities = world.query([OrientationComponent, PrevFinalOrientationComponent]);
        for (const entityId of entities) {
            const orientationComp = world.getComponent(entityId, OrientationComponent);
            const prevFinalOrientationComp = world.getComponent(entityId, PrevFinalOrientationComponent);
            prevFinalOrientationComp.angle = orientationComp.angle;
        }
    }
}

export class PBDAngularVelocityUpdateSystem {
  runInPause = false;

  _normalizeAngle(angle) { // Helper to find the shortest angle difference
    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return angle;
  }

  update(world, dt) {
    const grabbed = world.getResource('grabbedBall');
    const entities = world.query([
      OrientationComponent,
      AngularVelocityComponent,
      PrevFinalOrientationComponent,
      MomentOfInertiaComponent
    ]);
    const epsilon = 1e-9;

    if (dt <= epsilon) return;

    for (const entityId of entities) {
      if (entityId === grabbed) continue;

      const moiComp = world.getComponent(entityId, MomentOfInertiaComponent);
      if (moiComp && moiComp.invInertia <= 0) continue; // Skip static or non-rotational objects

      const orientationComp = world.getComponent(entityId, OrientationComponent);
      const angularVelComp = world.getComponent(entityId, AngularVelocityComponent);
      const prevFinalOrientationComp = world.getComponent(entityId, PrevFinalOrientationComponent);

      const currentAngle = orientationComp.angle;
      const prevAngle = prevFinalOrientationComp.angle;

      let deltaAngle = this._normalizeAngle(currentAngle - prevAngle);

      angularVelComp.angularVelocity = deltaAngle / dt;
    }
  }
}

export class PBDVelocityUpdateSystem {
  runInPause = false;
  update(world, dt) {
    const grabbed = world.getResource('grabbedBall');
    const entities = world.query([PositionComponent, VelocityComponent, PrevFinalPosComponent, MassComponent]);
    const epsilon = 1e-9;

    if (dt <= epsilon) return; // Avoid division by zero or very small dt

    for (const entityId of entities) {
      if (entityId === grabbed) continue; // Don't update velocity of grabbed ball this way

      const massComp = world.getComponent(entityId, MassComponent);
      // Only update velocities for dynamic objects (mass > 0)
      if (massComp && massComp.mass <= 0) continue;


      const posComp = world.getComponent(entityId, PositionComponent);
      const velComp = world.getComponent(entityId, VelocityComponent);
      const prevFinalPosComp = world.getComponent(entityId, PrevFinalPosComponent);

      // v = (x_new - x_old) / dt
      velComp.vel.subtractVectors(posComp.pos, prevFinalPosComp.pos).scale(1.0 / dt);
    }
  }
}


export class FlipperTipLinkSystem {
  runInPause = false;
  update(world, dt) {
    // for each tip‐entity, compute its current position
    for (const tipId of world.query([FlipperTipComponent, PositionComponent, FlipperTipComponent])) {
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

export class GravitySystem {
  runInPause = false;
  update(world, dt) {
    const grabbed = world.getResource('grabbedBall');
    const gravity = world.getResource('gravity');
    if (!gravity) return;

    const entities = world.query([VelocityComponent, GravityAffectedComponent]);
    for (const entityId of entities) {
      if (entityId === grabbed) continue;
      const velComp = world.getComponent(entityId, VelocityComponent);
      velComp.vel.add(gravity, dt);
    }
  }
}

export class XPBDDistanceConstraintSystem {
  runInPause = false;
  update(world, dt) {
    const constraintEntities = world.query([DistanceConstraintComponent]);
    const epsilon = 1e-9; // Small value to avoid division by zero

    for (const entityId of constraintEntities) {
      const constraint = world.getComponent(entityId, DistanceConstraintComponent);

      const entityA = constraint.entityA;
      const entityB = constraint.entityB;

      const posAComp = world.getComponent(entityA, PositionComponent);
      const posBComp = world.getComponent(entityB, PositionComponent);

      if (!posAComp || !posBComp) continue;

      const pA = posAComp.pos;
      const pB = posBComp.pos;

      const massAComp = world.getComponent(entityA, MassComponent);
      const invMassA = (massAComp && massAComp.mass > 0) ? 1.0 / massAComp.mass : 0.0;
      const moiAComp = world.getComponent(entityA, MomentOfInertiaComponent);
      const invInertiaA = moiAComp ? moiAComp.invInertia : 0.0;

      const massBComp = world.getComponent(entityB, MassComponent);
      const invMassB = (massBComp && massBComp.mass > 0) ? 1.0 / massBComp.mass : 0.0;
      const moiBComp = world.getComponent(entityB, MomentOfInertiaComponent);
      const invInertiaB = moiBComp ? moiBComp.invInertia : 0.0;

      // If both entities are effectively immovable, skip
      if (invMassA + invMassB + invInertiaA + invInertiaB <= epsilon) {
        continue;
      }

      const diff = new Vector2().subtractVectors(pB, pA);
      const currentLength = diff.length();
      if (currentLength <= epsilon) continue;

      const dir = diff.clone().scale(1.0 / currentLength); // Normalized direction from A to B

      const C = currentLength - constraint.restLength; // Constraint violation

      // XPBD compliance term (alpha_tilde in the paper)
      const alpha_tilde = constraint.compliance / (dt * dt);

      // Generalized inverse mass for the constraint
      // For distance constraints, the gradient w.r.t position is 'dir' or '-dir'
      // For rotation, it's more complex if attachment points are offset, but here we assume CoM constraint
      // For simplicity, assuming constraint acts on CoM directly (no rotational coupling here)
      // More general XPBD would include r x n terms for rotational parts.
      // This simplified version matches standard PBD distance constraint generalized mass.
      let w_sum = invMassA + invMassB; // Simplified: only translational

      // Denominator for delta_lambda
      const denominator = w_sum + alpha_tilde;
      if (denominator <= epsilon) continue;

      const delta_lambda = (-C - alpha_tilde * constraint.lambda) / denominator;
      constraint.lambda += delta_lambda;

      const correction = dir.clone().scale(delta_lambda);

      // Apply positional corrections
      if (invMassA > 0) {
        const dpA = correction.clone().scale(-invMassA);
        pA.add(dpA);
        const velAComp = world.getComponent(entityA, VelocityComponent);
        if (velAComp && dt > epsilon) {
            // velAComp.vel.add(dpA, 1.0 / dt); // This would be for PBD velocity update
        }
      }
      if (invMassB > 0) {
        const dpB = correction.clone().scale(invMassB);
        pB.add(dpB);
        const velBComp = world.getComponent(entityB, VelocityComponent);
        if (velBComp && dt > epsilon) {
            // velBComp.vel.add(dpB, 1.0 / dt); // This would be for PBD velocity update
        }
      }
    }
  }
}

export class MovementSystem {
  runInPause = false;
  update(world, dt) {
    const grabbed = world.getResource('grabbedBall');
    // Update linear position
    const linearEntities = world.query([PositionComponent, VelocityComponent]);
    for (const entityId of linearEntities) {
      if (entityId === grabbed) continue;
      const posComp = world.getComponent(entityId, PositionComponent);
      const velComp = world.getComponent(entityId, VelocityComponent);
      posComp.pos.add(velComp.vel, dt);
    }
  }
}

export class PrevFinalPosSystem {
    runInPause = false;
    update(world, dt) {
        const entities = world.query([PositionComponent, PrevFinalPosComponent]);
        for (const entityId of entities) {
            const posComponent = world.getComponent(entityId, PositionComponent);
            const prevFinalPosComponent = world.getComponent(entityId,PrevFinalPosComponent);
            prevFinalPosComponent.pos.set(posComponent.pos);
        }
    }
}


export class AngularMovementSystem {
    runInPause = false;
    update(world, dt) {
        const entities = world.query([OrientationComponent, AngularVelocityComponent]);
        for (const entityId of entities) {
            const orientation = world.getComponent(entityId, OrientationComponent);
            const angularVel = world.getComponent(entityId, AngularVelocityComponent);

            orientation.angle += angularVel.angularVelocity * dt;
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

    let contacts = world.getResource('ball_obstacle_contacts');
    if (!contacts) {
        contacts = [];
        world.setResource('ball_obstacle_contacts', contacts);
    }
    contacts.length = 0; // Clear existing contacts

    for (const ballId of ballEntities) {
      const p1 = world.getComponent(ballId, PositionComponent).pos;
      const r1 = world.getComponent(ballId, RadiusComponent).radius;

      for (const obsId of obstacleEntities) {
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

        // Velocity resolution is now handled by BallObstacleBumpSystem
        const grabbed = world.getResource('grabbedBall');
        if (ballId !== grabbed) {
          world.addComponent(ballId, new ScoredTagComponent());
        }
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
  update(world, dt) {
    if (this.inputLog.length > 0) {
      if (this.inputSystem.frame === this.inputLog[0].frame) {
        const frame = this.inputLog.shift();
        this.inputSystem.clicks = frame.clicks.slice();
        this.inputSystem.releases = frame.releases.slice();
      }
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
        const rSum = r1 + fr;

        if (dSq == 0.0 || dSq > rSum * rSum) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d);

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
      }
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
                'delta_lambda': delta_lambda
            });
        }
    }
  }
}
