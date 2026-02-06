import {
  VelocityComponent,
  GravityAffectedComponent,
  PositionComponent,
  PrevFinalPosComponent,
  MassComponent
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
