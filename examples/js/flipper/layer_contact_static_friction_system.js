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
import {
  FlipperStateComponent,
  FlipperTipComponent,
} from './flipper_common.js';

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

function _flipperTipPointAtPrevPose(world, flipperId, pointCurrent, dt) {
  const pivotCurrent = world.getComponent(flipperId, PositionComponent)?.pos;
  const pivotPrev = _prevPosition(world, flipperId);
  const flipperState = world.getComponent(flipperId, FlipperStateComponent);
  if (
    pivotCurrent === undefined ||
    pivotPrev === undefined ||
    flipperState === undefined ||
    pointCurrent === undefined
  ) {
    return pointCurrent?.clone?.() ?? null;
  }

  const sign = Number.isFinite(flipperState.sign) ? flipperState.sign : 1.0;
  const currentRotation = Number.isFinite(flipperState.rotation) ? flipperState.rotation : 0.0;
  const currentAngularVelocity = Number.isFinite(flipperState.currentAngularVelocity)
    ? flipperState.currentAngularVelocity
    : 0.0;
  const currentAngle = (Number.isFinite(flipperState.restAngle) ? flipperState.restAngle : 0.0) + sign * currentRotation;
  const prevAngle = currentAngle - (currentAngularVelocity * dt);

  const relCurrent = pointCurrent.clone().subtract(pivotCurrent);
  const relLocal = _rotate(relCurrent, -currentAngle);
  const relPrev = _rotate(relLocal, prevAngle);
  return pivotPrev.clone().add(relPrev);
}

function _applyStaticFrictionConstraint({
  ballPos,
  ballPrevPos,
  ballOffsetCurrent,
  ballOffsetPrev,
  otherPos,
  otherPrevPos,
  otherOffsetCurrent,
  otherOffsetPrev,
  invMassBall,
  invMassOther,
  invInertiaBall,
  invInertiaOther,
  ballOrientation,
  otherOrientation,
  muStatic,
  lambdaNormal,
  normal,
}) {
  const pBallCurrent = ballPos.clone().add(ballOffsetCurrent);
  const pOtherCurrent = otherPos.clone().add(otherOffsetCurrent);
  const pBallPrev = ballPrevPos.clone().add(ballOffsetPrev);
  const pOtherPrev = otherPrevPos.clone().add(otherOffsetPrev);

  const deltaP = pBallCurrent.clone().subtract(pBallPrev).subtract(pOtherCurrent.clone().subtract(pOtherPrev));
  const deltaNormal = normal.clone().scale(deltaP.dot(normal));
  const deltaTangential = deltaP.clone().subtract(deltaNormal);
  const tangentialMagnitude = deltaTangential.length();
  if (tangentialMagnitude <= 1e-9) {
    return;
  }

  const tangentCorrectionDir = deltaTangential.clone().scale(-1.0 / tangentialMagnitude);
  const rBallCrossT = ballOffsetCurrent.x * tangentCorrectionDir.y - ballOffsetCurrent.y * tangentCorrectionDir.x;
  const rOtherCrossT = otherOffsetCurrent.x * tangentCorrectionDir.y - otherOffsetCurrent.y * tangentCorrectionDir.x;

  const denominator = (
    invMassBall +
    invMassOther +
    invInertiaBall * rBallCrossT * rBallCrossT +
    invInertiaOther * rOtherCrossT * rOtherCrossT
  );
  if (denominator <= 1e-12) {
    return;
  }

  const lambdaStaticLimit = muStatic * lambdaNormal;
  if (lambdaStaticLimit <= 1e-12) {
    return;
  }
  const lambdaNoSlip = tangentialMagnitude / denominator;
  const lambdaTangential = Math.min(lambdaNoSlip, lambdaStaticLimit);
  if (lambdaTangential <= 1e-12) {
    return;
  }

  const impulse = tangentCorrectionDir.clone().scale(lambdaTangential);
  if (invMassBall > 0.0) {
    ballPos.add(impulse, invMassBall);
  }
  if (invMassOther > 0.0) {
    otherPos.add(impulse, -invMassOther);
  }
  if (invInertiaBall > 0.0 && ballOrientation) {
    const torqueBall = ballOffsetCurrent.x * impulse.y - ballOffsetCurrent.y * impulse.x;
    ballOrientation.angle += torqueBall * invInertiaBall;
  }
  if (invInertiaOther > 0.0 && otherOrientation) {
    const torqueOther = otherOffsetCurrent.x * (-impulse.y) - otherOffsetCurrent.y * (-impulse.x);
    otherOrientation.angle += torqueOther * invInertiaOther;
  }
}

export class LayerContactStaticFrictionSystem {
  runInPause = false;

  update(world, _dt_unused) {
    const obstacleContacts = world.getResource('ball_obstacle_contacts');
    const borderContacts = world.getResource('ball_border_contacts');
    const flipperContacts = world.getResource('ball_flipper_contacts');
    const dtResource = world.getResource('dt');
    const dt = Number.isFinite(dtResource) ? dtResource : (1.0 / 500.0);

    const tipToFlipper = new Map();
    for (const tipId of world.query([FlipperTipComponent])) {
      const tip = world.getComponent(tipId, FlipperTipComponent);
      if (tip && tip.flipperEntityId !== undefined) {
        tipToFlipper.set(tip.flipperEntityId, tipId);
      }
    }

    if (Array.isArray(obstacleContacts)) {
      for (const contact of obstacleContacts) {
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

        const muBall = _mu(world, ballId, contact.ball_friction);
        const muObs = _mu(world, obsId, contact.obstacle_friction);
        const muStatic = 0.5 * (muBall + muObs);
        if (muStatic <= 0.0) {
          continue;
        }

        const lambdaNormal = Number.isFinite(contact.delta_lambda) ? Math.max(0.0, contact.delta_lambda) : 0.0;
        if (lambdaNormal <= 1e-12) {
          continue;
        }

        const ballOffsetPrev = _offsetAtPrevPose(world, ballId, ballOffsetCurrent);
        const obsOffsetPrev = _offsetAtPrevPose(world, obsId, obsOffsetCurrent);
        _applyStaticFrictionConstraint({
          ballPos,
          ballPrevPos,
          ballOffsetCurrent,
          ballOffsetPrev,
          otherPos: obsPos,
          otherPrevPos: obsPrevPos,
          otherOffsetCurrent: obsOffsetCurrent,
          otherOffsetPrev: obsOffsetPrev,
          invMassBall: _invMass(world, ballId),
          invMassOther: _invMass(world, obsId),
          invInertiaBall: _invInertia(world, ballId),
          invInertiaOther: _invInertia(world, obsId),
          ballOrientation: world.getComponent(ballId, OrientationComponent),
          otherOrientation: world.getComponent(obsId, OrientationComponent),
          muStatic,
          lambdaNormal,
          normal: unitNormal,
        });
      }
    }

    if (Array.isArray(borderContacts)) {
      for (const contact of borderContacts) {
        const ballId = contact?.ball_id;
        const normal = contact?.normal;
        if (
          ballId === undefined ||
          normal === undefined ||
          normal === null ||
          Number.isFinite(normal.x) === false ||
          Number.isFinite(normal.y) === false
        ) {
          continue;
        }

        const ballPos = world.getComponent(ballId, PositionComponent)?.pos;
        const ballPrevPos = _prevPosition(world, ballId);
        if (ballPos === undefined || ballPrevPos === undefined) {
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
        const ballOffsetPrev = _offsetAtPrevPose(world, ballId, ballOffsetCurrent);

        const muBall = _mu(world, ballId, contact.ball_friction);
        const muBorder = Number.isFinite(contact.friction) ? Math.max(0.0, contact.friction) : 0.0;
        const muStatic = 0.5 * (muBall + muBorder);
        if (muStatic <= 0.0) {
          continue;
        }

        const lambdaNormal = Number.isFinite(contact.delta_lambda) ? Math.max(0.0, contact.delta_lambda) : 0.0;
        if (lambdaNormal <= 1e-12) {
          continue;
        }

        _applyStaticFrictionConstraint({
          ballPos,
          ballPrevPos,
          ballOffsetCurrent,
          ballOffsetPrev,
          otherPos: new Vector2(0.0, 0.0),
          otherPrevPos: new Vector2(0.0, 0.0),
          otherOffsetCurrent: new Vector2(0.0, 0.0),
          otherOffsetPrev: new Vector2(0.0, 0.0),
          invMassBall: _invMass(world, ballId),
          invMassOther: 0.0,
          invInertiaBall: _invInertia(world, ballId),
          invInertiaOther: 0.0,
          ballOrientation: world.getComponent(ballId, OrientationComponent),
          otherOrientation: null,
          muStatic,
          lambdaNormal,
          normal: unitNormal,
        });
      }
    }

    if (Array.isArray(flipperContacts)) {
      for (const contact of flipperContacts) {
        const ballId = contact?.ball_id;
        const flipperId = contact?.flip_id;
        const normal = contact?.normal;
        if (
          ballId === undefined ||
          flipperId === undefined ||
          normal === undefined ||
          normal === null ||
          Number.isFinite(normal.x) === false ||
          Number.isFinite(normal.y) === false
        ) {
          continue;
        }

        const ballPos = world.getComponent(ballId, PositionComponent)?.pos;
        const ballPrevPos = _prevPosition(world, ballId);
        const pointOnFlipperCurrent = contact?.contact_point_on_flipper;
        const pointOnFlipperPrev = _flipperTipPointAtPrevPose(world, flipperId, pointOnFlipperCurrent, dt);
        if (
          ballPos === undefined ||
          ballPrevPos === undefined ||
          pointOnFlipperCurrent === undefined ||
          pointOnFlipperPrev === undefined
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
        const ballOffsetPrev = _offsetAtPrevPose(world, ballId, ballOffsetCurrent);

        const flipperTipId = tipToFlipper.get(flipperId);
        const flipperMu = Number.isFinite(contact.friction)
          ? Math.max(0.0, contact.friction)
          : (Number.isFinite(world.getComponent(flipperTipId, CoefficientOfFrictionComponent)?.mu)
            ? Math.max(0.0, world.getComponent(flipperTipId, CoefficientOfFrictionComponent).mu)
            : 0.0);
        const muBall = _mu(world, ballId, contact.ball_friction);
        const muStatic = 0.5 * (muBall + flipperMu);
        if (muStatic <= 0.0) {
          continue;
        }

        const lambdaNormal = Number.isFinite(contact.delta_lambda) ? Math.max(0.0, contact.delta_lambda) : 0.0;
        if (lambdaNormal <= 1e-12) {
          continue;
        }

        _applyStaticFrictionConstraint({
          ballPos,
          ballPrevPos,
          ballOffsetCurrent,
          ballOffsetPrev,
          otherPos: pointOnFlipperCurrent.clone(),
          otherPrevPos: pointOnFlipperPrev.clone(),
          otherOffsetCurrent: new Vector2(0.0, 0.0),
          otherOffsetPrev: new Vector2(0.0, 0.0),
          invMassBall: _invMass(world, ballId),
          invMassOther: 0.0,
          invInertiaBall: _invInertia(world, ballId),
          invInertiaOther: 0.0,
          ballOrientation: world.getComponent(ballId, OrientationComponent),
          otherOrientation: null,
          muStatic,
          lambdaNormal,
          normal: unitNormal,
        });
      }
    }
  }
}
