import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  VelocityComponent,
  RadiusComponent,
  MassComponent,
  RestitutionComponent,
  AngularVelocityComponent,
  MomentOfInertiaComponent,
  CoefficientOfFrictionComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';

export class BallBallVelocityContactSystem3D {
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
    return linearVel.clone().add(new Vector3().crossVectors(angularVel, contactOffset));
  }

  _contactOffsetA(world, contact, normal) {
    const offset = contact.contact_offset_a;
    if (offset && Number.isFinite(offset.x) && Number.isFinite(offset.y) && Number.isFinite(offset.z)) {
      return new Vector3(offset.x, offset.y, offset.z);
    }
    const radiusFromContact = Number.isFinite(contact.radius_a) ? Math.max(0.0, contact.radius_a) : null;
    if (radiusFromContact !== null) {
      return normal.clone().scale(radiusFromContact);
    }
    const radiusComp = world.getComponent(contact.ball_a, RadiusComponent);
    const radius = Number.isFinite(radiusComp?.radius) ? Math.max(0.0, radiusComp.radius) : 0.0;
    return normal.clone().scale(radius);
  }

  _contactOffsetB(world, contact, normal) {
    const offset = contact.contact_offset_b;
    if (offset && Number.isFinite(offset.x) && Number.isFinite(offset.y) && Number.isFinite(offset.z)) {
      return new Vector3(offset.x, offset.y, offset.z);
    }
    const radiusFromContact = Number.isFinite(contact.radius_b) ? Math.max(0.0, contact.radius_b) : null;
    if (radiusFromContact !== null) {
      return normal.clone().scale(-radiusFromContact);
    }
    const radiusComp = world.getComponent(contact.ball_b, RadiusComponent);
    const radius = Number.isFinite(radiusComp?.radius) ? Math.max(0.0, radiusComp.radius) : 0.0;
    return normal.clone().scale(-radius);
  }

  update(world, dt) {
    const contacts = world.getResource('ball_ball_contacts');
    if (!Array.isArray(contacts) || contacts.length < 1) {
      return;
    }

    for (const contact of contacts) {
      const idA = contact?.ball_a;
      const idB = contact?.ball_b;
      const normal = contact?.normal;
      if (
        idA === undefined ||
        idB === undefined ||
        !normal ||
        !Number.isFinite(normal.x) ||
        !Number.isFinite(normal.y) ||
        !Number.isFinite(normal.z)
      ) {
        continue;
      }

      const velAComp = world.getComponent(idA, VelocityComponent);
      const velBComp = world.getComponent(idB, VelocityComponent);
      const angAComp = world.getComponent(idA, AngularVelocityComponent);
      const angBComp = world.getComponent(idB, AngularVelocityComponent);
      const moiAComp = world.getComponent(idA, MomentOfInertiaComponent);
      const moiBComp = world.getComponent(idB, MomentOfInertiaComponent);
      if (!velAComp || !velBComp || !angAComp || !angBComp || !moiAComp || !moiBComp) {
        continue;
      }

      const invMassA = this._invMass(world, idA);
      const invMassB = this._invMass(world, idB);
      const invInertiaA = this._finiteNumber(moiAComp.invInertia, 0.0);
      const invInertiaB = this._finiteNumber(moiBComp.invInertia, 0.0);
      if (invMassA === 0.0 && invMassB === 0.0 && invInertiaA === 0.0 && invInertiaB === 0.0) {
        continue;
      }

      const restitution = 0.5 * (
        this._clampedRestitution(world.getComponent(idA, RestitutionComponent)?.restitution, 0.0) +
        this._clampedRestitution(world.getComponent(idB, RestitutionComponent)?.restitution, 0.0)
      );

      const muA = this._nonNegativeNumber(
        contact.friction_a,
        this._nonNegativeNumber(world.getComponent(idA, CoefficientOfFrictionComponent)?.mu, 0.0)
      );
      const muB = this._nonNegativeNumber(
        contact.friction_b,
        this._nonNegativeNumber(world.getComponent(idB, CoefficientOfFrictionComponent)?.mu, 0.0)
      );
      const mu = 0.5 * (muA + muB);

      const offsetA = this._contactOffsetA(world, contact, normal);
      const offsetB = this._contactOffsetB(world, contact, normal);
      const velA = velAComp.vel;
      const velB = velBComp.vel;
      const omegaA = angAComp.omega;
      const omegaB = angBComp.omega;

      const surfaceVelA = this._contactSurfaceVelocity(velA, omegaA, offsetA);
      const surfaceVelB = this._contactSurfaceVelocity(velB, omegaB, offsetB);
      const relativeVel = surfaceVelA.clone().subtract(surfaceVelB);
      const relativeNormalVel = relativeVel.dot(normal);

      const rAxN = new Vector3().crossVectors(offsetA, normal).lengthSq();
      const rBxN = new Vector3().crossVectors(offsetB, normal).lengthSq();
      const normalDenominator = invMassA + invMassB + invInertiaA * rAxN + invInertiaB * rBxN;

      let normalImpulseMagnitude = 0.0;
      if (relativeNormalVel < 0.0 && normalDenominator > 1e-9) {
        normalImpulseMagnitude = -(1.0 + restitution) * relativeNormalVel / normalDenominator;
        const impulseN = normal.clone().scale(normalImpulseMagnitude);
        if (invMassA > 0.0) {
          velA.add(impulseN, invMassA);
        }
        if (invMassB > 0.0) {
          velB.add(impulseN, -invMassB);
        }
        if (invInertiaA > 0.0) {
          angAComp.omega.add(new Vector3().crossVectors(offsetA, impulseN), invInertiaA);
        }
        if (invInertiaB > 0.0) {
          angBComp.omega.add(new Vector3().crossVectors(offsetB, impulseN.clone().scale(-1.0)), invInertiaB);
        }
      }

      const normalImpulseFromConstraint = dt > 1e-9 ? this._finiteNumber(contact.delta_lambda, 0.0) / dt : 0.0;
      const frictionNormalImpulse = Math.max(0.0, normalImpulseMagnitude + normalImpulseFromConstraint);

      const surfaceVelAAfterNormal = this._contactSurfaceVelocity(velA, angAComp.omega, offsetA);
      const surfaceVelBAfterNormal = this._contactSurfaceVelocity(velB, angBComp.omega, offsetB);
      const relativeVelAfterNormal = surfaceVelAAfterNormal.clone().subtract(surfaceVelBAfterNormal);
      const relativeTangentialVel = relativeVelAfterNormal.clone().subtract(normal.clone().scale(relativeVelAfterNormal.dot(normal)));
      const tangentialSpeed = relativeTangentialVel.length();
      if (mu <= 0.0 || tangentialSpeed <= 1e-9) {
        continue;
      }

      const tangent = relativeTangentialVel.clone().scale(1.0 / tangentialSpeed);
      const rAxT = new Vector3().crossVectors(offsetA, tangent).lengthSq();
      const rBxT = new Vector3().crossVectors(offsetB, tangent).lengthSq();
      const tangentDenominator = invMassA + invMassB + invInertiaA * rAxT + invInertiaB * rBxT;
      if (tangentDenominator <= 1e-9) {
        continue;
      }

      const tangentialImpulseNoSlip = -tangentialSpeed / tangentDenominator;
      const tangentialImpulseLimit = mu * frictionNormalImpulse;
      const tangentialImpulseMagnitude = Math.max(
        -tangentialImpulseLimit,
        Math.min(tangentialImpulseNoSlip, tangentialImpulseLimit)
      );
      const impulseT = tangent.clone().scale(tangentialImpulseMagnitude);
      if (invMassA > 0.0) {
        velA.add(impulseT, invMassA);
      }
      if (invMassB > 0.0) {
        velB.add(impulseT, -invMassB);
      }
      if (invInertiaA > 0.0) {
        angAComp.omega.add(new Vector3().crossVectors(offsetA, impulseT), invInertiaA);
      }
      if (invInertiaB > 0.0) {
        angBComp.omega.add(new Vector3().crossVectors(offsetB, impulseT.clone().scale(-1.0)), invInertiaB);
      }
    }
  }
}
