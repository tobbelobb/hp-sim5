import Vector2 from './vector2.js';

import {
  PositionComponent,
  VelocityComponent,
  GravityAffectedComponent,
  MassComponent,
  PrevFinalPosComponent,
  OrientationComponent,
  MomentOfInertiaComponent,
  AngularVelocityComponent,
  DistanceConstraintComponent,
  PrevFinalOrientationComponent
} from './ecs.js';

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
