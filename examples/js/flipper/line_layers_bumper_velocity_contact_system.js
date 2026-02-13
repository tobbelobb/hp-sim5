import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
} from '../../../src/js/cable_joints/ecs.js';

export class LineLayersBumperVelocityContactSystem {
  runInPause = false;

  _finiteNumber(value, fallback = 0.0) {
    return Number.isFinite(value) ? value : fallback;
  }

  _nonNegativeNumber(value, fallback = 0.0) {
    const finite = this._finiteNumber(value, fallback);
    return finite > 0.0 ? finite : 0.0;
  }

  _clampedRestitution(value, fallback = 0.0) {
    const finite = this._finiteNumber(value, fallback);
    if (finite <= 0.0) return 0.0;
    if (finite >= 1.0) return 1.0;
    return finite;
  }

  _invMass(world, entityId) {
    const mass = world.getComponent(entityId, MassComponent)?.mass;
    return Number.isFinite(mass) && mass > 0.0 ? 1.0 / mass : 0.0;
  }

  _contactSurfaceVelocity(linearVel, angularVel, contactOffset) {
    const vAngular = new Vector2(
      -angularVel * contactOffset.y,
      angularVel * contactOffset.x
    );
    return linearVel.clone().add(vAngular);
  }

  update(world, dt) {
    const contacts = world.getResource('ball_obstacle_contacts');
    if (Array.isArray(contacts) === false || contacts.length < 1) {
      return;
    }

    for (const contact of contacts) {
      if (contact?.raw_hit === true) {
        continue;
      }

      const ballId = contact.ball_id;
      const obsId = contact.obs_id;
      const normal = contact.direction;
      if (
        ballId === undefined ||
        obsId === undefined ||
        normal === undefined ||
        normal === null ||
        Number.isFinite(normal.x) === false ||
        Number.isFinite(normal.y) === false
      ) {
        continue;
      }

      const ballVelComp = world.getComponent(ballId, VelocityComponent);
      const ballRadiusComp = world.getComponent(ballId, RadiusComponent);
      const ballAngVelComp = world.getComponent(ballId, AngularVelocityComponent);
      const ballMoiComp = world.getComponent(ballId, MomentOfInertiaComponent);
      const ballRestComp = world.getComponent(ballId, RestitutionComponent);
      if (
        ballVelComp === undefined ||
        ballRadiusComp === undefined ||
        ballAngVelComp === undefined ||
        ballMoiComp === undefined ||
        ballRestComp === undefined
      ) {
        continue;
      }

      const obsVelComp = world.getComponent(obsId, VelocityComponent);
      const obsRadiusComp = world.getComponent(obsId, RadiusComponent);
      const obsAngVelComp = world.getComponent(obsId, AngularVelocityComponent);
      const obsMoiComp = world.getComponent(obsId, MomentOfInertiaComponent);
      const obsRestComp = world.getComponent(obsId, RestitutionComponent);
      const obsFrictionComp = world.getComponent(obsId, CoefficientOfFrictionComponent);
      const ballFrictionComp = world.getComponent(ballId, CoefficientOfFrictionComponent);

      const ballLinearVel = ballVelComp.vel;
      const obsLinearVel = obsVelComp ? obsVelComp.vel : new Vector2(0.0, 0.0);
      const ballOmega = this._finiteNumber(ballAngVelComp.angularVelocity, 0.0);
      const obsOmega = this._finiteNumber(obsAngVelComp?.angularVelocity, 0.0);
      const ballInvInertia = this._finiteNumber(ballMoiComp.invInertia, 0.0);
      const obsInvInertia = this._finiteNumber(obsMoiComp?.invInertia, 0.0);
      const ballInvMass = this._invMass(world, ballId);
      const obsInvMass = this._invMass(world, obsId);

      if (
        ballInvMass === 0.0 &&
        obsInvMass === 0.0 &&
        ballInvInertia === 0.0 &&
        obsInvInertia === 0.0
      ) {
        continue;
      }

      const fallbackBallOffset = normal.clone().scale(-ballRadiusComp.radius);
      const ballContactOffset = (
        contact.ball_contact_offset &&
        Number.isFinite(contact.ball_contact_offset.x) &&
        Number.isFinite(contact.ball_contact_offset.y)
      )
        ? new Vector2(contact.ball_contact_offset.x, contact.ball_contact_offset.y)
        : fallbackBallOffset;

      const obstacleContactRadius = Number.isFinite(contact.obstacle_contact_radius)
        ? Math.max(0.0, contact.obstacle_contact_radius)
        : (obsRadiusComp ? Math.max(0.0, obsRadiusComp.radius) : 0.0);
      const obsContactOffset = normal.clone().scale(obstacleContactRadius);

      const restitutionBall = this._clampedRestitution(ballRestComp.restitution, 0.0);
      const restitutionObs = this._clampedRestitution(obsRestComp?.restitution, 0.0);
      const restitution = 0.5 * (restitutionBall + restitutionObs);

      const muBallBase = this._nonNegativeNumber(ballFrictionComp?.mu, 0.0);
      const muObsBase = this._nonNegativeNumber(obsFrictionComp?.mu, 0.0);
      const muBall = this._nonNegativeNumber(contact.ball_friction, muBallBase);
      const muObs = this._nonNegativeNumber(contact.obstacle_friction, muObsBase);
      const mu = 0.5 * (muBall + muObs);

      const ballSurfaceVel = this._contactSurfaceVelocity(ballLinearVel, ballOmega, ballContactOffset);
      const obsSurfaceVel = this._contactSurfaceVelocity(obsLinearVel, obsOmega, obsContactOffset);
      const relativeVel = ballSurfaceVel.clone().subtract(obsSurfaceVel);
      const relativeNormalVel = relativeVel.dot(normal);

      const rBallCrossN = ballContactOffset.x * normal.y - ballContactOffset.y * normal.x;
      const rObsCrossN = obsContactOffset.x * normal.y - obsContactOffset.y * normal.x;
      const normalDenominator = (
        ballInvMass +
        obsInvMass +
        ballInvInertia * rBallCrossN * rBallCrossN +
        obsInvInertia * rObsCrossN * rObsCrossN
      );

      let normalImpulseMagnitude = 0.0;
      if (relativeNormalVel < 0.0 && normalDenominator > 1e-9) {
        normalImpulseMagnitude = -(1.0 + restitution) * relativeNormalVel / normalDenominator;
        const normalImpulse = normal.clone().scale(normalImpulseMagnitude);
        if (ballInvMass > 0.0) {
          ballLinearVel.add(normalImpulse, ballInvMass);
        }
        if (obsInvMass > 0.0 && obsVelComp) {
          obsLinearVel.add(normalImpulse, -obsInvMass);
        }
        if (ballInvInertia > 0.0) {
          const torqueBall = rBallCrossN * normalImpulseMagnitude;
          ballAngVelComp.angularVelocity += torqueBall * ballInvInertia;
        }
        if (obsInvInertia > 0.0 && obsAngVelComp) {
          const torqueObs = rObsCrossN * (-normalImpulseMagnitude);
          obsAngVelComp.angularVelocity += torqueObs * obsInvInertia;
        }
      }

      const normalImpulseFromConstraint = dt > 1e-9
        ? this._finiteNumber(contact.delta_lambda, 0.0) / dt
        : 0.0;
      const frictionNormalImpulse = Math.max(0.0, normalImpulseMagnitude + normalImpulseFromConstraint);

      const ballSurfaceVelAfterNormal = this._contactSurfaceVelocity(
        ballLinearVel,
        this._finiteNumber(ballAngVelComp.angularVelocity, 0.0),
        ballContactOffset
      );
      const obsSurfaceVelAfterNormal = this._contactSurfaceVelocity(
        obsLinearVel,
        this._finiteNumber(obsAngVelComp?.angularVelocity, 0.0),
        obsContactOffset
      );
      const relativeVelAfterNormal = ballSurfaceVelAfterNormal.clone().subtract(obsSurfaceVelAfterNormal);
      const relativeTangentialVel = relativeVelAfterNormal
        .clone()
        .subtract(normal.clone().scale(relativeVelAfterNormal.dot(normal)));
      const tangentialSpeed = relativeTangentialVel.length();

      if (mu <= 0.0 || tangentialSpeed <= 1e-9) {
        continue;
      }

      const tangent = relativeTangentialVel.clone().scale(1.0 / tangentialSpeed);
      const rBallCrossT = ballContactOffset.x * tangent.y - ballContactOffset.y * tangent.x;
      const rObsCrossT = obsContactOffset.x * tangent.y - obsContactOffset.y * tangent.x;
      const tangentDenominator = (
        ballInvMass +
        obsInvMass +
        ballInvInertia * rBallCrossT * rBallCrossT +
        obsInvInertia * rObsCrossT * rObsCrossT
      );
      if (tangentDenominator <= 1e-9) {
        continue;
      }

      const tangentialImpulseNoSlip = -tangentialSpeed / tangentDenominator;
      const tangentialImpulseLimit = mu * frictionNormalImpulse;
      const tangentialImpulseMagnitude = Math.max(
        -tangentialImpulseLimit,
        Math.min(tangentialImpulseNoSlip, tangentialImpulseLimit)
      );
      const tangentialImpulse = tangent.clone().scale(tangentialImpulseMagnitude);

      if (ballInvMass > 0.0) {
        ballLinearVel.add(tangentialImpulse, ballInvMass);
      }
      if (obsInvMass > 0.0 && obsVelComp) {
        obsLinearVel.add(tangentialImpulse, -obsInvMass);
      }
      if (ballInvInertia > 0.0) {
        const torqueBall = rBallCrossT * tangentialImpulseMagnitude;
        ballAngVelComp.angularVelocity += torqueBall * ballInvInertia;
      }
      if (obsInvInertia > 0.0 && obsAngVelComp) {
        const torqueObs = rObsCrossT * (-tangentialImpulseMagnitude);
        obsAngVelComp.angularVelocity += torqueObs * obsInvInertia;
      }
    }
  }
}
