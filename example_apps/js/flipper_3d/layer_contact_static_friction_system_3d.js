import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import {
  PositionComponent,
  PrevFinalPosComponent,
  OrientationComponent,
  PrevFinalOrientationComponent,
  RadiusComponent,
  MassComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';

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

function _prevPosition(world, entityId) {
  return world.getComponent(entityId, PrevFinalPosComponent)?.pos ?? world.getComponent(entityId, PositionComponent)?.pos;
}

function _offsetAtPrevPose(world, entityId, offsetCurrent) {
  const orientationComp = world.getComponent(entityId, OrientationComponent)?.quaternion;
  const prevOrientationComp = world.getComponent(entityId, PrevFinalOrientationComponent)?.quaternion;
  if (!orientationComp || !prevOrientationComp) {
    return offsetCurrent.clone();
  }
  const local = orientationComp.clone().conjugate().normalize().transformVector(offsetCurrent.clone());
  return prevOrientationComp.transformVector(local);
}

function _planeNormal(world, entityId) {
  return world.getComponent(entityId, OrientationComponent)?.quaternion
    ? new Vector3(0.0, 0.0, 1.0)
    : new Vector3(0.0, 0.0, 1.0);
}

function _applyAxisAngle(quaternion, deltaAngle, axis) {
  if (!quaternion || !Number.isFinite(deltaAngle) || Math.abs(deltaAngle) <= 1e-12) {
    return;
  }
  const normalizedAxis = axis.clone().normalize();
  const dq = new Quaternion().setFromAxisAngle(normalizedAxis, deltaAngle);
  quaternion.multiplyQuaternions(dq, quaternion).normalize();
}

export class LayerContactStaticFrictionSystem3D {
  runInPause = false;

  update(world, _dt_unused) {
    const contacts = world.getResource('ball_obstacle_contacts');
    if (!Array.isArray(contacts) || contacts.length < 1) {
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
        !normal ||
        !Number.isFinite(normal.x) ||
        !Number.isFinite(normal.y) ||
        !Number.isFinite(normal.z)
      ) {
        continue;
      }

      const ballPos = world.getComponent(ballId, PositionComponent)?.pos;
      const obsPos = world.getComponent(obsId, PositionComponent)?.pos;
      const ballPrevPos = _prevPosition(world, ballId);
      const obsPrevPos = _prevPosition(world, obsId);
      if (!ballPos || !obsPos || !ballPrevPos || !obsPrevPos) {
        continue;
      }

      const unitNormal = normal.clone();
      if (unitNormal.lengthSq() <= 1e-12) {
        continue;
      }
      unitNormal.normalize();

      const ballRadius = world.getComponent(ballId, RadiusComponent)?.radius;
      const ballOffsetCurrent = (
        contact.ball_contact_offset &&
        Number.isFinite(contact.ball_contact_offset.x) &&
        Number.isFinite(contact.ball_contact_offset.y) &&
        Number.isFinite(contact.ball_contact_offset.z)
      )
        ? new Vector3(contact.ball_contact_offset.x, contact.ball_contact_offset.y, contact.ball_contact_offset.z)
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
      const planeNormalBall = _planeNormal(world, ballId);
      const planeNormalObs = _planeNormal(world, obsId);
      const rBallCrossT = ballOffsetCurrent.cross(tangentCorrectionDir).dot(planeNormalBall);
      const rObsCrossT = obsOffsetCurrent.cross(tangentCorrectionDir).dot(planeNormalObs);
      const denominator = invMassBall + invMassObs + invInertiaBall * rBallCrossT * rBallCrossT + invInertiaObs * rObsCrossT * rObsCrossT;
      if (denominator <= 1e-12) {
        continue;
      }

      const muStatic = 0.5 * (_mu(world, ballId, contact.ball_friction) + _mu(world, obsId, contact.obstacle_friction));
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

      if (invInertiaBall > 0.0) {
        _applyAxisAngle(
          world.getComponent(ballId, OrientationComponent)?.quaternion,
          ballOffsetCurrent.cross(impulse).dot(planeNormalBall) * invInertiaBall,
          planeNormalBall
        );
      }
      if (invInertiaObs > 0.0) {
        _applyAxisAngle(
          world.getComponent(obsId, OrientationComponent)?.quaternion,
          obsOffsetCurrent.cross(impulse.clone().scale(-1.0)).dot(planeNormalObs) * invInertiaObs,
          planeNormalObs
        );
      }
    }
  }
}
