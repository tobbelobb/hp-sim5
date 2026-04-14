import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  PositionComponent,
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
  layeringEnabled
} from '../../../src/js/cable_joints_3d/ecs.js';
import {
  FlipperStateComponent,
  FlipperTipComponent
} from './flipper_common_3d.js';

const EPSILON = 1e-9;

export class BallBorderOrFlipperVelocityContactSystem3D {
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

  _handleBallContact(
    world,
    ballId,
    normal,
    surfaceVelocity,
    restitutionOther,
    frictionOther,
    frictionSelf,
    deltaLambda,
    dt,
    contactOffset = null
  ) {
    const posComp = world.getComponent(ballId, PositionComponent);
    const velComp = world.getComponent(ballId, VelocityComponent);
    const radiusComp = world.getComponent(ballId, RadiusComponent);
    const massComp = world.getComponent(ballId, MassComponent);
    const moiComp = world.getComponent(ballId, MomentOfInertiaComponent);
    const angVelComp = world.getComponent(ballId, AngularVelocityComponent);
    const restitutionComp = world.getComponent(ballId, RestitutionComponent);
    const frictionComp = world.getComponent(ballId, CoefficientOfFrictionComponent);

    if (!posComp || !velComp || !radiusComp || !massComp || !moiComp || !angVelComp || !restitutionComp) {
      return;
    }

    const mass = massComp.mass;
    const invMass = mass > 0 ? 1.0 / mass : 0.0;
    const invInertia = this._finiteNumber(moiComp.invInertia, 0.0);

    if (invMass === 0.0 && invInertia === 0.0) {
      return;
    }

    const radius = this._finiteNumber(radiusComp.radius, 0.0);
    const restitutionBall = this._clampedRestitution(restitutionComp.restitution, 0.0);
    const muBallBase = this._nonNegativeNumber(frictionComp ? frictionComp.mu : 0.0, 0.0);
    const muBall = this._nonNegativeNumber(frictionSelf, muBallBase);

    const restitutionOtherResolved = this._clampedRestitution(restitutionOther, 0.0);
    const frictionOtherResolved = this._nonNegativeNumber(frictionOther, 0.0);

    const restitution = 0.5 * (restitutionBall + restitutionOtherResolved);
    const mu = 0.5 * (muBall + frictionOtherResolved);

    const useContactOffset =
      layeringEnabled(world) &&
      world.getResource('layeringVelocityContactOffset') !== false;
    const useOffset =
      useContactOffset &&
      contactOffset &&
      Number.isFinite(contactOffset.x) &&
      Number.isFinite(contactOffset.y) &&
      Number.isFinite(contactOffset.z);
    const rBall = useOffset
      ? new Vector3(contactOffset.x, contactOffset.y, contactOffset.z)
      : normal.clone().scale(-radius);

    const vAngularAtContact = new Vector3().crossVectors(angVelComp.omega, rBall);
    const vBallAtContact = velComp.vel.clone().add(vAngularAtContact);

    const vRel = vBallAtContact.clone().subtract(surfaceVelocity);
    const vRelNormalScalar = vRel.dot(normal);

    let jNormalRestitution = 0.0;
    if (vRelNormalScalar < 0.0) {
      const rCrossN = new Vector3().crossVectors(rBall, normal);
      const wInvN = invMass + invInertia * rCrossN.lengthSq();
      if (wInvN > EPSILON) {
        jNormalRestitution = -(1.0 + restitution) * vRelNormalScalar / wInvN;
        const impulseNormal = normal.clone().scale(jNormalRestitution);

        velComp.vel.add(impulseNormal, invMass);

        const deltaL = new Vector3().crossVectors(rBall, impulseNormal);
        angVelComp.omega.add(deltaL, invInertia);
      }
    }

    let jNormalForce = 0.0;
    if (dt > EPSILON) {
      jNormalForce = this._finiteNumber(deltaLambda, 0.0) / dt;
    }
    const jNormalForFriction = Math.max(0.0, jNormalRestitution + jNormalForce);

    const vAngularAfterNormal = new Vector3().crossVectors(angVelComp.omega, rBall);
    const vBallAfterNormal = velComp.vel.clone().add(vAngularAfterNormal);
    const vRelAfterNormal = vBallAfterNormal.clone().subtract(surfaceVelocity);

    const vRelTangent = vRelAfterNormal.clone().subtract(normal.clone().scale(vRelAfterNormal.dot(normal)));
    const vRelTangentMag = vRelTangent.length();

    if (mu > 0 && vRelTangentMag > EPSILON) {
      const tangent = vRelTangent.clone().scale(1.0 / vRelTangentMag);
      const rCrossT = new Vector3().crossVectors(rBall, tangent);
      const wInvT = invMass + invInertia * rCrossT.lengthSq();

      if (wInvT > EPSILON) {
        const jTangentNoSlip = -vRelTangentMag / wInvT;
        const maxFriction = mu * jNormalForFriction;
        const jTangent = Math.max(-maxFriction, Math.min(jTangentNoSlip, maxFriction));
        const impulseTangent = tangent.clone().scale(jTangent);

        velComp.vel.add(impulseTangent, invMass);

        const deltaL = new Vector3().crossVectors(rBall, impulseTangent);
        angVelComp.omega.add(deltaL, invInertia);
      }
    }
  }

  update(world, dt) {
    const borderContacts = world.getResource('ball_border_contacts');
    if (borderContacts) {
      for (const contact of borderContacts) {
        this._handleBallContact(
          world,
          contact.ball_id,
          contact.normal,
          new Vector3(0, 0, 0),
          contact.restitution,
          contact.friction,
          contact.ball_friction,
          contact.delta_lambda,
          dt,
          contact.ball_contact_offset
        );
      }
    }

    const flipperContacts = world.getResource('ball_flipper_contacts');
    if (!flipperContacts) {
      return;
    }

    for (const contact of flipperContacts) {
      const flipperPosComp = world.getComponent(contact.flip_id, PositionComponent);
      const flipperStateComp = world.getComponent(contact.flip_id, FlipperStateComponent);
      const flipperRestitutionComp = world.getComponent(contact.flip_id, RestitutionComponent);

      let flipperTipId = null;
      const tipEntities = world.query([FlipperTipComponent]);
      for (const tipId of tipEntities) {
        const tipComp = world.getComponent(tipId, FlipperTipComponent);
        if (tipComp.flipperEntityId === contact.flip_id) {
          flipperTipId = tipId;
          break;
        }
      }

      const flipperFrictionComp = flipperTipId !== null
        ? world.getComponent(flipperTipId, CoefficientOfFrictionComponent)
        : null;

      if (!flipperPosComp || !flipperStateComp || !flipperRestitutionComp) {
        continue;
      }

      const flipperPivotPos = flipperPosComp.pos;
      const axis = flipperStateComp.planeNormal
        ? flipperStateComp.planeNormal.clone().normalize()
        : new Vector3(0, 0, 1);
      const flipperOmega = axis.scale(flipperStateComp.currentAngularVelocity);

      const rFlipper = contact.contact_point_on_flipper.clone().subtract(flipperPivotPos);
      const vFlipper = new Vector3().crossVectors(flipperOmega, rFlipper);

      const restitutionFlipper = flipperRestitutionComp.restitution;
      const frictionFlipper = flipperFrictionComp ? flipperFrictionComp.mu : 0.0;

      this._handleBallContact(
        world,
        contact.ball_id,
        contact.normal,
        vFlipper,
        restitutionFlipper,
        frictionFlipper,
        contact.ball_friction,
        contact.delta_lambda,
        dt,
        contact.ball_contact_offset
      );
    }
  }
}
