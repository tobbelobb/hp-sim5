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
  AngularVelocityComponent
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
  runInPause = false;

  update(world, dt) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, VelocityComponent, RadiusComponent, MassComponent, RestitutionComponent]);
    const obstacleEntities = world.query([ObstacleTagComponent, PositionComponent, RadiusComponent]);
    for (const ballId of ballEntities) {
      const p1 = world.getComponent(ballId, PositionComponent).pos;
      const v1 = world.getComponent(ballId, VelocityComponent).vel;
      const r1 = world.getComponent(ballId, RadiusComponent).radius;

      for (const obsId of obstacleEntities) {
        const p2 = world.getComponent(obsId, PositionComponent).pos;
        const r2 = world.getComponent(obsId, RadiusComponent).radius;
        const pushVel = 2.0;

        const dir = new Vector2().subtractVectors(p1, p2);
        const dSq = dir.lengthSq();
        const rSum = r1 + r2;

        if (dSq == 0.0 || dSq > rSum * rSum) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d); // Normalize

        // Resolve penetration
        const corr = rSum - d;
        p1.add(dir, corr);

        // Resolve velocity (simple push)
        const v_dot = v1.dot(dir);
        v1.add(dir, pushVel - v_dot); // Impart obstacle's push velocity along normal
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
