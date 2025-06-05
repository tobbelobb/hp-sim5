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
  DistanceConstraintComponent
} from './ecs.js';


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
    const ballEntities = world.query([BallTagComponent, PositionComponent, VelocityComponent, RadiusComponent, MassComponent, RestitutionComponent]);
    for (let i = 0; i < ballEntities.length; i++) {
      for (let j = i + 1; j < ballEntities.length; j++) {
        const e1 = ballEntities[i];
        const e2 = ballEntities[j];

        const p1 = world.getComponent(e1, PositionComponent).pos;
        const v1 = world.getComponent(e1, VelocityComponent).vel;
        const r1 = world.getComponent(e1, RadiusComponent).radius;
        const m1 = world.getComponent(e1, MassComponent).mass;
        const res1 = world.getComponent(e1, RestitutionComponent).restitution;

        const p2 = world.getComponent(e2, PositionComponent).pos;
        const v2 = world.getComponent(e2, VelocityComponent).vel;
        const r2 = world.getComponent(e2, RadiusComponent).radius;
        const m2 = world.getComponent(e2, MassComponent).mass;
        const res2 = world.getComponent(e2, RestitutionComponent).restitution;

        const restitution = Math.min(res1, res2);
        const dir = new Vector2().subtractVectors(p2, p1);
        const dSq = dir.lengthSq();
        const rSum = r1 + r2;

        if (dSq == 0.0 || dSq > rSum * rSum) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d); // Normalize

        // Resolve penetration
        const corr = (rSum - d) / 2.0;
        p1.add(dir, -corr);
        p2.add(dir, corr);

        // Resolve velocity
        const vel1_dot = v1.dot(dir);
        const vel2_dot = v2.dot(dir);

        const newV1_dot = (m1 * vel1_dot + m2 * vel2_dot - m2 * (vel1_dot - vel2_dot) * restitution) / (m1 + m2);
        const newV2_dot = (m1 * vel1_dot + m2 * vel2_dot - m1 * (vel2_dot - vel1_dot) * restitution) / (m1 + m2);

        v1.add(dir, newV1_dot - vel1_dot);
        v2.add(dir, newV2_dot - vel2_dot);
      }
    }
  }
}

export class PBDBallObstacleCollisions {
  update(world, dt) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, VelocityComponent, RadiusComponent, MassComponent]); // RestitutionComponent not used here
    const obstacleEntities = world.query([ObstacleTagComponent, PositionComponent, RadiusComponent, ObstaclePushComponent]);

    for (const ballId of ballEntities) {
      const p1 = world.getComponent(ballId, PositionComponent).pos;
      const v1 = world.getComponent(ballId, VelocityComponent).vel;
      const r1 = world.getComponent(ballId, RadiusComponent).radius;
      const ballMassComp = world.getComponent(ballId, MassComponent);
      const ballAngVelComp = world.getComponent(ballId, AngularVelocityComponent);
      const ballMoIComp = world.getComponent(ballId, MomentOfInertiaComponent);

      for (const obsId of obstacleEntities) {
        const p2 = world.getComponent(obsId, PositionComponent).pos;
        const r2 = world.getComponent(obsId, RadiusComponent).radius;
        const pushVel = world.getComponent(obsId, ObstaclePushComponent).pushVel;

        const dir = new Vector2().subtractVectors(p1, p2);
        const dSq = dir.lengthSq();
        const rSum = r1 + r2;

        if (dSq == 0.0 || dSq > rSum * rSum) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d); // Normalize

        // Resolve penetration
        const corr = rSum - d;
        p1.add(dir, corr);

        // Resolve velocity & rotation with friction and obstacle rotation
        const obsAngVelComp = world.getComponent(obsId, AngularVelocityComponent);
        const obsMoIComp = world.getComponent(obsId, MomentOfInertiaComponent);
        const obsFrictionComp = world.getComponent(obsId, CoefficientOfFrictionComponent);

        const omega_obs = obsAngVelComp ? obsAngVelComp.angularVelocity : 0.0;
        const mu = obsFrictionComp ? obsFrictionComp.mu : 0.0;

        const tangent = new Vector2(-dir.y, dir.x); // Tangential direction

        // Calculate obstacle's surface tangential velocity component at contact point
        let v_surf_obs_tangential_comp = 0;
        if (omega_obs !== 0) {
            // r_vec_obs is vector from obstacle center to contact point on its surface
            // Contact point on obs surface is p2 + dir * r2. So r_vec_obs = dir * r2.
            // Surface velocity v_surf = omega_obs x r_vec_obs.
            // v_surf.x = -omega_obs * r_vec_obs.y = -omega_obs * (dir.y * r2)
            // v_surf.y =  omega_obs * r_vec_obs.x =  omega_obs * (dir.x * r2)
            // v_surf_obs_tangential_comp = v_surf.dot(tangent)
            v_surf_obs_tangential_comp = (-omega_obs * dir.y * r2) * tangent.x + (omega_obs * dir.x * r2) * tangent.y;
        }

        // Determine effective push direction
        const s_sign = (v_surf_obs_tangential_comp === 0 || mu === 0) ? 0 : Math.sign(v_surf_obs_tangential_comp);
        let effectivePushDir = dir.clone();
        if (s_sign !== 0) {
            effectivePushDir.add(tangent.clone().scale(mu * s_sign));
        }
        effectivePushDir.normalize(); // Normalize to ensure pushVel is the magnitude along this dir

        // Apply translational push
        v1.add(effectivePushDir, pushVel);

        // Rotational transfer if friction is present
        if (mu > 0 && ballMassComp) {
            // The tangential component of the velocity change *actually applied* to the ball
            const delta_v_ball_tangential_actual_scalar_comp = effectivePushDir.dot(tangent) * pushVel;

            if (Math.abs(delta_v_ball_tangential_actual_scalar_comp) > 1e-9) {
                const J_t_on_ball_vec = tangent.clone().scale(delta_v_ball_tangential_actual_scalar_comp * ballMassComp.mass);

                // Apply torque to ball
                if (ballAngVelComp && ballMoIComp && ballMoIComp.invInertia > 0) {
                    const r_contact_ball = dir.clone().scale(-r1); // Vector from ball center to contact point
                    const delta_L_ball = r_contact_ball.x * J_t_on_ball_vec.y - r_contact_ball.y * J_t_on_ball_vec.x;
                    ballAngVelComp.angularVelocity += delta_L_ball * ballMoIComp.invInertia;
                }

                // Apply torque to obstacle
                if (obsAngVelComp && obsMoIComp && obsMoIComp.invInertia > 0) {
                    const J_t_on_obs_vec = J_t_on_ball_vec.clone().scale(-1);
                    const r_contact_obs = dir.clone().scale(r2); // Vector from obstacle center to contact point
                    const delta_L_obs = r_contact_obs.x * J_t_on_obs_vec.y - r_contact_obs.y * J_t_on_obs_vec.x;
                    obsAngVelComp.angularVelocity += delta_L_obs * obsMoIComp.invInertia;
                }
            }
        }

        // Add ScoredTagComponent to the ball that scored
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
