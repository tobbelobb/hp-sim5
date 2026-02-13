import Vector2 from '../../../src/js/cable_joints/vector2.js';
import {
  PositionComponent,
  PrevFinalPosComponent,
  OrientationComponent,
  PrevFinalOrientationComponent,
  RadiusComponent,
  MassComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
} from '../../../src/js/cable_joints/ecs.js';

function _invMass(world, entityId) {
  const mass = world.getComponent(entityId, MassComponent)?.mass;
  return Number.isFinite(mass) && mass > 0.0 ? 1.0 / mass : 0.0;
}

function _invInertia(world, entityId) {
  const invInertia = world.getComponent(entityId, MomentOfInertiaComponent)?.invInertia;
  return Number.isFinite(invInertia) ? Math.max(0.0, invInertia) : 0.0;
}

function _mu(world, entityId, layeredMu) {
  if (Number.isFinite(layeredMu)) {
    return Math.max(0.0, layeredMu);
  }
  const baseMu = world.getComponent(entityId, CoefficientOfFrictionComponent)?.mu;
  return Number.isFinite(baseMu) ? Math.max(0.0, baseMu) : 0.0;
}

function _rotate(vec, angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return new Vector2(
    vec.x * c - vec.y * s,
    vec.x * s + vec.y * c
  );
}

function _offsetAtPrevPose(world, entityId, offsetCurrent) {
  const orientationComp = world.getComponent(entityId, OrientationComponent);
  const prevOrientationComp = world.getComponent(entityId, PrevFinalOrientationComponent);
  if (orientationComp === undefined || prevOrientationComp === undefined) {
    return offsetCurrent.clone();
  }
  const currentAngle = Number.isFinite(orientationComp.angle) ? orientationComp.angle : 0.0;
  const prevAngle = Number.isFinite(prevOrientationComp.angle) ? prevOrientationComp.angle : currentAngle;
  const local = _rotate(offsetCurrent, -currentAngle);
  return _rotate(local, prevAngle);
}

function _prevPosition(world, entityId) {
  const prevComp = world.getComponent(entityId, PrevFinalPosComponent);
  if (prevComp && prevComp.pos) {
    return prevComp.pos;
  }
  return world.getComponent(entityId, PositionComponent)?.pos;
}

export class LayerContactStaticFrictionSystem {
  runInPause = false;

  update(world, _dt_unused) {
    const contacts = world.getResource('ball_obstacle_contacts');
    if (Array.isArray(contacts) === false || contacts.length < 1) {
      return;
    }

    for (const contact of contacts) {
      if (contact?.raw_hit === true) {
        continue;
      }

      const ballId = contact?.ball_id;
      const obsId = contact?.obs_id;
      const normal = contact?.direction;
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

      const ballPos = world.getComponent(ballId, PositionComponent)?.pos;
      const obsPos = world.getComponent(obsId, PositionComponent)?.pos;
      const ballPrevPos = _prevPosition(world, ballId);
      const obsPrevPos = _prevPosition(world, obsId);
      if (
        ballPos === undefined ||
        obsPos === undefined ||
        ballPrevPos === undefined ||
        obsPrevPos === undefined
      ) {
        continue;
      }

      const normalLengthSq = (normal.x * normal.x) + (normal.y * normal.y);
      if (normalLengthSq <= 1e-12) {
        continue;
      }
      const unitNormal = normal.clone().scale(1.0 / Math.sqrt(normalLengthSq));

      const ballRadius = world.getComponent(ballId, RadiusComponent)?.radius;
      const ballOffsetCurrent = (
        contact.ball_contact_offset &&
        Number.isFinite(contact.ball_contact_offset.x) &&
        Number.isFinite(contact.ball_contact_offset.y)
      )
        ? new Vector2(contact.ball_contact_offset.x, contact.ball_contact_offset.y)
        : unitNormal.clone().scale(-(Number.isFinite(ballRadius) ? ballRadius : 0.0));

      const obsRadius = world.getComponent(obsId, RadiusComponent)?.radius;
      const obstacleContactRadius = Number.isFinite(contact.obstacle_contact_radius)
        ? Math.max(0.0, contact.obstacle_contact_radius)
        : (Number.isFinite(obsRadius) ? Math.max(0.0, obsRadius) : 0.0);
      const obsOffsetCurrent = unitNormal.clone().scale(obstacleContactRadius);

      const ballOffsetPrev = _offsetAtPrevPose(world, ballId, ballOffsetCurrent);
      const obsOffsetPrev = _offsetAtPrevPose(world, obsId, obsOffsetCurrent);

      const pBallCurrent = ballPos.clone().add(ballOffsetCurrent);
      const pObsCurrent = obsPos.clone().add(obsOffsetCurrent);
      const pBallPrev = ballPrevPos.clone().add(ballOffsetPrev);
      const pObsPrev = obsPrevPos.clone().add(obsOffsetPrev);

      const deltaP = pBallCurrent.clone().subtract(pBallPrev).subtract(pObsCurrent.clone().subtract(pObsPrev));
      const deltaNormal = unitNormal.clone().scale(deltaP.dot(unitNormal));
      const deltaTangential = deltaP.clone().subtract(deltaNormal);
      const tangentialMagnitude = deltaTangential.length();
      if (tangentialMagnitude <= 1e-9) {
        continue;
      }

      const tangentCorrectionDir = deltaTangential.clone().scale(-1.0 / tangentialMagnitude);

      const invMassBall = _invMass(world, ballId);
      const invMassObs = _invMass(world, obsId);
      const invInertiaBall = _invInertia(world, ballId);
      const invInertiaObs = _invInertia(world, obsId);

      const rBallCrossT = ballOffsetCurrent.x * tangentCorrectionDir.y - ballOffsetCurrent.y * tangentCorrectionDir.x;
      const rObsCrossT = obsOffsetCurrent.x * tangentCorrectionDir.y - obsOffsetCurrent.y * tangentCorrectionDir.x;
      const denominator = (
        invMassBall +
        invMassObs +
        invInertiaBall * rBallCrossT * rBallCrossT +
        invInertiaObs * rObsCrossT * rObsCrossT
      );
      if (denominator <= 1e-12) {
        continue;
      }

      const muBall = _mu(world, ballId, contact.ball_friction);
      const muObs = _mu(world, obsId, contact.obstacle_friction);
      const muStatic = 0.5 * (muBall + muObs);
      if (muStatic <= 0.0) {
        continue;
      }

      const lambdaNormal = Number.isFinite(contact.delta_lambda) ? Math.max(0.0, contact.delta_lambda) : 0.0;
      const lambdaStaticLimit = muStatic * lambdaNormal;
      if (lambdaStaticLimit <= 1e-12) {
        continue;
      }

      const lambdaNoSlip = tangentialMagnitude / denominator;
      const lambdaTangential = Math.min(lambdaNoSlip, lambdaStaticLimit);
      if (lambdaTangential <= 1e-12) {
        continue;
      }

      const impulse = tangentCorrectionDir.clone().scale(lambdaTangential);

      if (invMassBall > 0.0) {
        ballPos.add(impulse, invMassBall);
      }
      if (invMassObs > 0.0) {
        obsPos.add(impulse, -invMassObs);
      }

      const ballOrientation = world.getComponent(ballId, OrientationComponent);
      const obsOrientation = world.getComponent(obsId, OrientationComponent);
      if (invInertiaBall > 0.0 && ballOrientation) {
        const torqueBall = ballOffsetCurrent.x * impulse.y - ballOffsetCurrent.y * impulse.x;
        ballOrientation.angle += torqueBall * invInertiaBall;
      }
      if (invInertiaObs > 0.0 && obsOrientation) {
        const torqueObs = obsOffsetCurrent.x * (-impulse.y) - obsOffsetCurrent.y * (-impulse.x);
        obsOrientation.angle += torqueObs * invInertiaObs;
      }
    }
  }
}
