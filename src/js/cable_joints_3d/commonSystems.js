import {
  OrientationComponent,
  PrevFinalOrientationComponent,
  AngularVelocityComponent,
  VelocityComponent,
  GravityAffectedComponent,
  PositionComponent,
  PrevFinalPosComponent,
  MassComponent,
  MomentOfInertiaComponent
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
    const linearEntities = world.query([PositionComponent, VelocityComponent]);
    for (const entityId of linearEntities) {
      if (entityId === grabbed) continue;
      const posComp = world.getComponent(entityId, PositionComponent);
      const velComp = world.getComponent(entityId, VelocityComponent);
      posComp.pos.add(velComp.vel, dt);
    }
  }
}

export class PrevFinalOrientationSystem {
  runInPause = false;
  update(world, dt) {
    const entities = world.query([OrientationComponent, PrevFinalOrientationComponent]);
    for (const entityId of entities) {
      const orientationComp = world.getComponent(entityId, OrientationComponent);
      const prevFinalOrientationComp = world.getComponent(entityId, PrevFinalOrientationComponent);
      prevFinalOrientationComp.quaternion.set(orientationComp.quaternion);
    }
  }
}

export class AngularMovementSystem {
  runInPause = false;
  update(world, dt) {
    const entities = world.query([OrientationComponent, AngularVelocityComponent]);
    const epsilon = 1e-12;

    for (const entityId of entities) {
      const orientationComp = world.getComponent(entityId, OrientationComponent);
      const angularVelComp = world.getComponent(entityId, AngularVelocityComponent);

      const omega = angularVelComp.omega;
      const speed = omega.length();
      if (speed <= epsilon) continue;

      const axis = omega.clone().scale(1.0 / speed);
      const angle = speed * dt;

      const dq = orientationComp.quaternion.clone().setFromAxisAngle(axis, angle);
      orientationComp.quaternion.multiplyQuaternions(dq, orientationComp.quaternion).normalize();
    }
  }
}

export class PrevFinalPosSystem {
  runInPause = false;
  update(world, dt) {
    const entities = world.query([PositionComponent, PrevFinalPosComponent]);
    for (const entityId of entities) {
      const posComp = world.getComponent(entityId, PositionComponent);
      const prevFinalPosComp = world.getComponent(entityId, PrevFinalPosComponent);
      prevFinalPosComp.pos.set(posComp.pos);
    }
  }
}

export class PBDVelocityUpdateSystem {
  runInPause = false;
  update(world, dt) {
    const grabbed = world.getResource('grabbedBall');
    const entities = world.query([PositionComponent, VelocityComponent, PrevFinalPosComponent, MassComponent]);
    const epsilon = 1e-9;

    if (dt <= epsilon) return;

    for (const entityId of entities) {
      if (entityId === grabbed) continue;

      const massComp = world.getComponent(entityId, MassComponent);
      if (massComp && massComp.mass <= 0) continue;

      const posComp = world.getComponent(entityId, PositionComponent);
      const velComp = world.getComponent(entityId, VelocityComponent);
      const prevFinalPosComp = world.getComponent(entityId, PrevFinalPosComponent);

      velComp.vel.subtractVectors(posComp.pos, prevFinalPosComp.pos).scale(1.0 / dt);
    }
  }
}

export class PBDAngularVelocityUpdateSystem {
  runInPause = false;
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
      if (moiComp && moiComp.invInertia <= 0) continue;

      const orientationComp = world.getComponent(entityId, OrientationComponent);
      const angularVelComp = world.getComponent(entityId, AngularVelocityComponent);
      const prevFinalOrientationComp = world.getComponent(entityId, PrevFinalOrientationComponent);

      const qCurr = orientationComp.quaternion;
      const qPrevInv = prevFinalOrientationComp.quaternion.clone().conjugate().normalize();
      const qDelta = qCurr.clone().multiplyQuaternions(qCurr, qPrevInv).normalize();

      if (qDelta.w < 0) {
        qDelta.x *= -1;
        qDelta.y *= -1;
        qDelta.z *= -1;
        qDelta.w *= -1;
      }

      const w = Math.max(-1.0, Math.min(1.0, qDelta.w));
      const angle = 2.0 * Math.acos(w);
      const sinHalf = Math.sqrt(Math.max(0.0, 1.0 - w * w));

      if (sinHalf <= 1e-12 || angle <= epsilon) {
        angularVelComp.omega.x = 0.0;
        angularVelComp.omega.y = 0.0;
        angularVelComp.omega.z = 0.0;
        continue;
      }

      const scale = angle / (dt * sinHalf);
      angularVelComp.omega.x = qDelta.x * scale;
      angularVelComp.omega.y = qDelta.y * scale;
      angularVelComp.omega.z = qDelta.z * scale;
    }
  }
}
