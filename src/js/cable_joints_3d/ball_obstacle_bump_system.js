import Vector3 from './vector3.js';
import {
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
  ObstaclePushComponent
} from './ecs.js';

export class BallObstacleBumpSystem {
  runInPause = false;

  update(world, dt) {
    const contacts = world.getResource('ball_obstacle_contacts');
    if (!contacts || contacts.length === 0) {
      return;
    }

    const epsilon = 1e-9;

    for (const contact of contacts) {
      const { ball_id, obs_id, direction } = contact;

      const v1Comp = world.getComponent(ball_id, VelocityComponent);
      const r1Comp = world.getComponent(ball_id, RadiusComponent);
      const ballMassComp = world.getComponent(ball_id, MassComponent);
      const ballAngVelComp = world.getComponent(ball_id, AngularVelocityComponent);
      const ballMoiComp = world.getComponent(ball_id, MomentOfInertiaComponent);

      const r2Comp = world.getComponent(obs_id, RadiusComponent);
      const pushComp = world.getComponent(obs_id, ObstaclePushComponent);
      const obsAngVelComp = world.getComponent(obs_id, AngularVelocityComponent);
      const obsMoiComp = world.getComponent(obs_id, MomentOfInertiaComponent);
      const obsFrictionComp = world.getComponent(obs_id, CoefficientOfFrictionComponent);

      if (!v1Comp || !r1Comp || !ballMassComp || !r2Comp || !pushComp) {
        continue;
      }

      const v1 = v1Comp.vel;
      const r1 = r1Comp.radius;
      const r2 = r2Comp.radius;
      const pushVel = pushComp.pushVel;

      const omegaBall = ballAngVelComp ? ballAngVelComp.omega : null;
      const omegaObs = obsAngVelComp ? obsAngVelComp.omega : null;
      const mu = obsFrictionComp ? obsFrictionComp.mu : 0.0;

      const normal = direction.clone().normalize();

      let vSurfObs = new Vector3();
      if (omegaObs && omegaObs.lengthSq() > epsilon) {
        const rContactObs = normal.clone().scale(r2);
        vSurfObs = new Vector3().crossVectors(omegaObs, rContactObs);
      }

      let vBallAtContact = v1.clone();
      if (omegaBall && omegaBall.lengthSq() > epsilon) {
        const rContactBall = normal.clone().scale(-r1);
        const vAngularBall = new Vector3().crossVectors(omegaBall, rContactBall);
        vBallAtContact.add(vAngularBall);
      }

      const vRel = vBallAtContact.clone().subtract(vSurfObs);
      const vRelNormal = normal.clone().scale(vRel.dot(normal));
      const vRelTangential = vRel.clone().subtract(vRelNormal);

      let effectivePushDir = normal.clone();
      let tangentDir = null;

      if (mu !== 0 && vRelTangential.lengthSq() > epsilon) {
        tangentDir = vRelTangential.normalize();
        const frictionDir = tangentDir.clone().scale(-1);
        effectivePushDir.add(frictionDir, mu).normalize();
      }

      v1.add(effectivePushDir, pushVel);

      if (mu > 0 && tangentDir && ballMassComp.mass > 0) {
        const deltaVTangential = effectivePushDir.dot(tangentDir) * pushVel;
        if (Math.abs(deltaVTangential) > epsilon) {
          const jTOnBall = tangentDir.clone().scale(deltaVTangential * ballMassComp.mass);

          if (ballAngVelComp && ballMoiComp && ballMoiComp.invInertia > 0) {
            const rContactBall = normal.clone().scale(-r1);
            const deltaLBall = new Vector3().crossVectors(rContactBall, jTOnBall);
            ballAngVelComp.omega.add(deltaLBall, ballMoiComp.invInertia);
          }

          if (obsAngVelComp && obsMoiComp && obsMoiComp.invInertia > 0) {
            const jTOnObs = jTOnBall.clone().scale(-1);
            const rContactObs = normal.clone().scale(r2);
            const deltaLObs = new Vector3().crossVectors(rContactObs, jTOnObs);
            obsAngVelComp.omega.add(deltaLObs, obsMoiComp.invInertia);
          }
        }
      }
    }
  }
}
