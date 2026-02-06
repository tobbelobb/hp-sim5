import Vector3 from './vector3.js';
import {
  BallTagComponent,
  ObstacleTagComponent,
  ObstaclePushComponent,
  PositionComponent,
  RadiusComponent
} from './ecs.js';

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
    contacts.length = 0;

    for (const ballId of ballEntities) {
      const p1 = world.getComponent(ballId, PositionComponent).pos;
      const r1 = world.getComponent(ballId, RadiusComponent).radius;

      for (const obsId of obstacleEntities) {
        const p2 = world.getComponent(obsId, PositionComponent).pos;
        const r2 = world.getComponent(obsId, RadiusComponent).radius;

        const dir = new Vector3().subtractVectors(p1, p2);
        const dSq = dir.lengthSq();
        const rSum = r1 + r2;

        if (dSq === 0.0 || dSq > rSum * rSum) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d); // Normalize

        contacts.push({ ball_id: ballId, obs_id: obsId, direction: dir.clone() });

        const corr = rSum - d;
        p1.add(dir, corr);
      }
    }
  }
}
