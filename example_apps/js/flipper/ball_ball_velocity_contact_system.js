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

export class BallBallVelocityContactSystem {
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

  _contactOffsetA(world, contact, normal) {
    const offset = contact.contact_offset_a;
    if (
      offset &&
      Number.isFinite(offset.x) &&
      Number.isFinite(offset.y)
    ) {
      return new Vector2(offset.x, offset.y);
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
    if (
      offset &&
      Number.isFinite(offset.x) &&
      Number.isFinite(offset.y)
    ) {
      return new Vector2(offset.x, offset.y);
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
    if (Array.isArray(contacts) === false || contacts.length < 1) {
      return;
    }

    for (const contact of contacts) {
      const idA = contact?.ball_a;
      const idB = contact?.ball_b;
      const normal = contact?.normal;
      if (
        idA === undefined ||
        idB === undefined ||
        normal === undefined ||
        normal === null ||
        Number.isFinite(normal.x) === false ||
        Number.isFinite(normal.y) === false
      ) {
        continue;
      }

      const velAComp = world.getComponent(idA, VelocityComponent);
      const velBComp = world.getComponent(idB, VelocityComponent);
      const angAComp = world.getComponent(idA, AngularVelocityComponent);
      const angBComp = world.getComponent(idB, AngularVelocityComponent);
      const moiAComp = world.getComponent(idA, MomentOfInertiaComponent);
      const moiBComp = world.getComponent(idB, MomentOfInertiaComponent);
      if (
        velAComp === undefined ||
        velBComp === undefined ||
        angAComp === undefined ||
        angBComp === undefined ||
        moiAComp === undefined ||
        moiBComp === undefined
      ) {
        continue;
      }

      const invMassA = this._invMass(world, idA);
      const invMassB = this._invMass(world, idB);
      const invInertiaA = this._finiteNumber(moiAComp.invInertia, 0.0);
      const invInertiaB = this._finiteNumber(moiBComp.invInertia, 0.0);
      if (
        invMassA === 0.0 &&
        invMassB === 0.0 &&
        invInertiaA === 0.0 &&
        invInertiaB === 0.0
      ) {
        continue;
      }

      const restAComp = world.getComponent(idA, RestitutionComponent);
      const restBComp = world.getComponent(idB, RestitutionComponent);
      const restitutionA = this._clampedRestitution(restAComp?.restitution, 0.0);
      const restitutionB = this._clampedRestitution(restBComp?.restitution, 0.0);
      const restitution = 0.5 * (restitutionA + restitutionB);

      const frictionAComp = world.getComponent(idA, CoefficientOfFrictionComponent);
      const frictionBComp = world.getComponent(idB, CoefficientOfFrictionComponent);
      const muABase = this._nonNegativeNumber(frictionAComp?.mu, 0.0);
      const muBBase = this._nonNegativeNumber(frictionBComp?.mu, 0.0);
      const muA = this._nonNegativeNumber(contact.friction_a, muABase);
      const muB = this._nonNegativeNumber(contact.friction_b, muBBase);
      const mu = 0.5 * (muA + muB);

      const offsetA = this._contactOffsetA(world, contact, normal);
      const offsetB = this._contactOffsetB(world, contact, normal);

      const velA = velAComp.vel;
      const velB = velBComp.vel;
      const omegaA = this._finiteNumber(angAComp.angularVelocity, 0.0);
      const omegaB = this._finiteNumber(angBComp.angularVelocity, 0.0);
      const surfaceVelA = this._contactSurfaceVelocity(velA, omegaA, offsetA);
      const surfaceVelB = this._contactSurfaceVelocity(velB, omegaB, offsetB);
      const relativeVel = surfaceVelA.clone().subtract(surfaceVelB);
      const relativeNormalVel = relativeVel.dot(normal);

      const rAxN = offsetA.x * normal.y - offsetA.y * normal.x;
      const rBxN = offsetB.x * normal.y - offsetB.y * normal.x;
      const normalDenominator = (
        invMassA +
        invMassB +
        invInertiaA * rAxN * rAxN +
        invInertiaB * rBxN * rBxN
      );

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
          angAComp.angularVelocity += rAxN * normalImpulseMagnitude * invInertiaA;
        }
        if (invInertiaB > 0.0) {
          angBComp.angularVelocity += rBxN * (-normalImpulseMagnitude) * invInertiaB;
        }
      }

      const normalImpulseFromConstraint = dt > 1e-9
        ? this._finiteNumber(contact.delta_lambda, 0.0) / dt
        : 0.0;
      const frictionNormalImpulse = Math.max(0.0, normalImpulseMagnitude + normalImpulseFromConstraint);

      const surfaceVelAAfterNormal = this._contactSurfaceVelocity(
        velA,
        this._finiteNumber(angAComp.angularVelocity, 0.0),
        offsetA
      );
      const surfaceVelBAfterNormal = this._contactSurfaceVelocity(
        velB,
        this._finiteNumber(angBComp.angularVelocity, 0.0),
        offsetB
      );
      const relativeVelAfterNormal = surfaceVelAAfterNormal.clone().subtract(surfaceVelBAfterNormal);
      const relativeTangentialVel = relativeVelAfterNormal
        .clone()
        .subtract(normal.clone().scale(relativeVelAfterNormal.dot(normal)));
      const tangentialSpeed = relativeTangentialVel.length();
      if (mu <= 0.0 || tangentialSpeed <= 1e-9) {
        continue;
      }

      const tangent = relativeTangentialVel.clone().scale(1.0 / tangentialSpeed);
      const rAxT = offsetA.x * tangent.y - offsetA.y * tangent.x;
      const rBxT = offsetB.x * tangent.y - offsetB.y * tangent.x;
      const tangentDenominator = (
        invMassA +
        invMassB +
        invInertiaA * rAxT * rAxT +
        invInertiaB * rBxT * rBxT
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
      const impulseT = tangent.clone().scale(tangentialImpulseMagnitude);
      if (invMassA > 0.0) {
        velA.add(impulseT, invMassA);
      }
      if (invMassB > 0.0) {
        velB.add(impulseT, -invMassB);
      }
      if (invInertiaA > 0.0) {
        angAComp.angularVelocity += rAxT * tangentialImpulseMagnitude * invInertiaA;
      }
      if (invInertiaB > 0.0) {
        angBComp.angularVelocity += rBxT * (-tangentialImpulseMagnitude) * invInertiaB;
      }
    }
  }
}
