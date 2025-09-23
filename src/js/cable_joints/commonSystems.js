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
  PrevFinalOrientationComponent,
  RigidGroupComponent
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

// Enforce rigid motion of grouped bodies via 2D shape matching (best-fit rigid transform).
// References: Müller et al. 2005 (shape matching), PBDBodies (rigid body constraints),
// but implemented as a single-shot Jacobi-style correction per group to avoid order bias.
export class RigidGroupSystem {
  runInPause = false;

  _computeCOM(world, members) {
    let sumMass = 0.0;
    const com = new Vector2(0, 0);
    for (const id of members) {
      const pos = world.getComponent(id, PositionComponent)?.pos;
      const m = world.getComponent(id, MassComponent)?.mass ?? 0.0;
      if (!pos || !(m > 0)) continue;
      com.add(pos.clone().scale(m));
      sumMass += m;
    }
    if (sumMass > 0) com.scale(1.0 / sumMass);
    return { com, sumMass };
  }

  update(world, dt) {
    const groupEntities = world.query([RigidGroupComponent]);
    if (!groupEntities || groupEntities.length === 0) return;

    for (const gid of groupEntities) {
      const group = world.getComponent(gid, RigidGroupComponent);
      const members = group.members || [];
      if (members.length < 2) continue;

      // Initialize rest offsets once from current configuration
      if (!group.restLocal) {
        const { com } = this._computeCOM(world, members);
        group.restLocal = members.map((id) => {
          const pos = world.getComponent(id, PositionComponent)?.pos;
          return pos ? pos.clone().subtract(com) : new Vector2(0, 0);
        });
      }

      // Current COM and relative positions
      const { com, sumMass } = this._computeCOM(world, members);
      if (!(sumMass > 0)) continue;

      // Compute best-fit rotation (2D) mapping restLocal -> currentRel using weighted Procrustes
      let Sx = 0.0; // sum m (r.x*p.x + r.y*p.y)
      let Sy = 0.0; // sum m (r.x*p.y - r.y*p.x)
      for (let i = 0; i < members.length; i++) {
        const id = members[i];
        const pos = world.getComponent(id, PositionComponent)?.pos;
        const m = world.getComponent(id, MassComponent)?.mass ?? 0.0;
        if (!pos || !(m > 0)) continue;
        const p = pos.clone().subtract(com);
        const r = group.restLocal[i] || new Vector2(0, 0);
        Sx += m * (r.x * p.x + r.y * p.y);
        Sy += m * (r.x * p.y - r.y * p.x);
      }

      const angle = Math.atan2(Sy, Sx);
      const c = Math.cos(angle);
      const s = Math.sin(angle);

      // Target positions under best-fit rigid transform
      const stiffness = Math.max(0, Math.min(1, group.stiffness ?? 1.0));
      for (let i = 0; i < members.length; i++) {
        const id = members[i];
        const posComp = world.getComponent(id, PositionComponent);
        if (!posComp) continue;
        const r = group.restLocal[i] || new Vector2(0, 0);
        const qx = c * r.x - s * r.y + com.x;
        const qy = s * r.x + c * r.y + com.y;
        const dp = new Vector2(qx - posComp.pos.x, qy - posComp.pos.y).scale(stiffness);
        posComp.pos.add(dp);
      }

      // Apply group rotation to each member's local orientation
      let deltaAngle = angle - (group.prevAngle || 0.0);
      // normalize to [-pi, pi] for shortest rotation
      while (deltaAngle > Math.PI) deltaAngle -= 2 * Math.PI;
      while (deltaAngle < -Math.PI) deltaAngle += 2 * Math.PI;
      if (Math.abs(deltaAngle) > 0) {
        for (let i = 0; i < members.length; i++) {
          const id = members[i];
          const o = world.getComponent(id, OrientationComponent);
          if (o) {
            o.angle += deltaAngle;
          }
        }
      }

      // Optional debug output if enabled
      try {
        if (world.getResource && world.getResource('debugAngles')) {
          const first = members[0];
          const o0 = world.getComponent(first, OrientationComponent);
          console.log('[RigidGroupSystem]', { angle, deltaAngle, o0: o0 ? o0.angle : null });
        }
      } catch(_) {}

      group.prevAngle = angle;
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
