import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';

import {
  EncoderComponent,
  OrientationComponent,
  PrevFinalOrientationComponent,
  AngularVelocityComponent,
  VelocityComponent,
  GravityAffectedComponent,
  PositionComponent,
  PrevFinalPosComponent,
  MassComponent,
  MomentOfInertiaComponent,
  DistanceConstraintComponent,
  RigidBodyComponent,
  RigidBodyMemberComponent,
} from './ecs.js';
import {
  SpoolStateComponent,
  rotateSpoolReferenceOrientation,
  constrainSpoolOrientation,
  constrainSpoolAngularVelocity,
  getSpoolRotationAngle,
  getSpoolWorldAxis,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';
import {
  initializeRigidBodySyncState,
  computeWorldAttachment,
  getEntityWorldOrientation,
  getEntityWorldPosition,
  resolveRigidBodySolverEndpoint,
  updateRigidBodyMemberLocalOrientation,
} from './rigid_bodies.js';

const DEFAULT_PLANE_NORMAL = new Vector3(0, 0, 1);

function getDefaultPlaneNormal(world) {
  const planeNormal = world?.getResource?.('defaultPlaneNormal');
  if (planeNormal && typeof planeNormal.clone === 'function' && planeNormal.lengthSq() > 1e-12) {
    return planeNormal.clone().normalize();
  }
  return DEFAULT_PLANE_NORMAL.clone();
}

function unwrapAngleNear(reference, wrappedValue) {
  let value = wrappedValue;
  while (value - reference > Math.PI) value -= Math.PI * 2.0;
  while (value - reference < -Math.PI) value += Math.PI * 2.0;
  return value;
}

function getEncoderAxis(encoderComp, world) {
  const axis = encoderComp?.axis;
  if (axis && typeof axis.lengthSq === 'function' && axis.lengthSq() > 1e-12) {
    return axis.clone().normalize();
  }
  return getDefaultPlaneNormal(world);
}

function planeBasisForAxis(axis) {
  const n = axis.clone().normalize();
  let reference = Math.abs(n.x) < 0.9 ? new Vector3(1, 0, 0) : new Vector3(0, 1, 0);
  const nDotRef = n.dot(reference);
  let u = reference.clone().subtract(n, nDotRef);
  if (u.lengthSq() <= 1e-12) {
    reference = new Vector3(0, 0, 1);
    u = reference.clone().subtract(n, n.dot(reference));
  }
  u.normalize();
  const v = n.cross(u);
  return { n, u, v };
}

function orientationAngleAroundAxis(quaternion, axis) {
  if (!quaternion) {
    return 0.0;
  }
  const basis = planeBasisForAxis(axis);
  const rotatedU = quaternion.transformVector(basis.u);
  const projected = rotatedU.clone().subtract(basis.n, rotatedU.dot(basis.n));
  if (projected.lengthSq() <= 1e-12) {
    return 0.0;
  }
  projected.normalize();
  return Math.atan2(projected.dot(basis.v), projected.dot(basis.u));
}

function applyOrientationDelta(world, entityId, deltaRotation) {
  const orientation = world.getComponent(entityId, OrientationComponent)?.quaternion;
  if (!orientation || !deltaRotation) {
    return;
  }
  orientation.multiplyQuaternions(deltaRotation, orientation).normalize();
  const spoolState = world.getComponent(entityId, SpoolStateComponent);
  if (spoolState) {
    rotateSpoolReferenceOrientation(spoolState, deltaRotation);
  }
}

function getRigidBodyMemberSpoolFrame(world, entityId, spoolState) {
  const member = world.getComponent(entityId, RigidBodyMemberComponent);
  if (!member?.localOrientation) {
    return null;
  }
  const bodyOrientation = world.getComponent(member.bodyEntity, OrientationComponent)?.quaternion;
  if (!bodyOrientation) {
    return null;
  }
  const bodyInverse = bodyOrientation.clone().conjugate().normalize();
  const localReferenceOrientation = new Quaternion()
    .multiplyQuaternions(bodyInverse, spoolState.referenceOrientation)
    .normalize();
  const worldOrientation = new Quaternion()
    .multiplyQuaternions(bodyOrientation, member.localOrientation)
    .normalize();
  return {
    member,
    worldOrientation,
    localSpoolState: {
      axisLocal: spoolState.axisLocal,
      referenceOrientation: localReferenceOrientation,
    },
  };
}

export class GravitySystem {
  runInPause = false;

  update(world, dt) {
    const grabbed = world.getResource('grabbedBall');
    const gravity = world.getResource('gravity');
    if (!gravity) return;

    const entities = world.query([VelocityComponent, GravityAffectedComponent]);
    for (const entityId of entities) {
      if (entityId === grabbed) continue;
      if (world.getComponent(entityId, RigidBodyMemberComponent)) continue;
      const velComp = world.getComponent(entityId, VelocityComponent);
      velComp.vel.add(gravity, dt);
    }
  }
}

export class XPBDDistanceConstraintSystem {
  runInPause = false;

  update(world, dt) {
    const constraintEntities = world.query([DistanceConstraintComponent]);
    const epsilon = 1e-9;

    for (const entityId of constraintEntities) {
      const constraint = world.getComponent(entityId, DistanceConstraintComponent);
      if (!constraint) {
        continue;
      }

      const entityA = constraint.entityA;
      const entityB = constraint.entityB;
      const pointA = getEntityWorldPosition(world, entityA);
      const pointB = getEntityWorldPosition(world, entityB);
      if (!pointA || !pointB) {
        continue;
      }

      const mappedA = resolveRigidBodySolverEndpoint(world, entityA, entityB, pointA);
      const mappedB = resolveRigidBodySolverEndpoint(world, entityB, entityA, pointB);
      const solverEntityA = mappedA.entityId;
      const solverEntityB = mappedB.entityId;
      if (solverEntityA === solverEntityB) {
        continue;
      }

      const solverPointA = computeWorldAttachment(world, solverEntityA, mappedA.localPoint);
      const solverPointB = computeWorldAttachment(world, solverEntityB, mappedB.localPoint);
      if (!solverPointA || !solverPointB) {
        continue;
      }

      const posAComp = world.getComponent(solverEntityA, PositionComponent);
      const posBComp = world.getComponent(solverEntityB, PositionComponent);
      if (!posAComp?.pos || !posBComp?.pos) {
        continue;
      }

      const massAComp = world.getComponent(solverEntityA, MassComponent);
      const massBComp = world.getComponent(solverEntityB, MassComponent);
      const invMassA = massAComp && massAComp.mass > 0 ? 1.0 / massAComp.mass : 0.0;
      const invMassB = massBComp && massBComp.mass > 0 ? 1.0 / massBComp.mass : 0.0;
      const moiAComp = world.getComponent(solverEntityA, MomentOfInertiaComponent);
      const moiBComp = world.getComponent(solverEntityB, MomentOfInertiaComponent);
      const invInertiaA = moiAComp ? moiAComp.invInertia : 0.0;
      const invInertiaB = moiBComp ? moiBComp.invInertia : 0.0;

      const diff = new Vector3().subtractVectors(solverPointB, solverPointA);
      const currentLength = diff.length();
      if (currentLength <= epsilon) {
        continue;
      }

      const dir = diff.clone().scale(1.0 / currentLength);
      const violation = currentLength - constraint.restLength;
      const gradPosA = dir.clone();
      const gradPosB = dir.clone().scale(-1.0);
      const rA = new Vector3().subtractVectors(solverPointA, posAComp.pos);
      const rB = new Vector3().subtractVectors(solverPointB, posBComp.pos);
      const gradAngA = rA.cross(gradPosA);
      const gradAngB = rB.cross(gradPosB);
      const alphaTilde = constraint.compliance / (dt * dt);
      const denominator = (
        invMassA * gradPosA.lengthSq()
        + invInertiaA * gradAngA.lengthSq()
        + invMassB * gradPosB.lengthSq()
        + invInertiaB * gradAngB.lengthSq()
        + alphaTilde
      );
      if (denominator <= epsilon) {
        continue;
      }

      const deltaLambda = (-violation - alphaTilde * constraint.lambda) / denominator;
      constraint.lambda += deltaLambda;

      if (invMassA > 0.0) {
        posAComp.pos.add(gradPosA.clone().scale(-invMassA * deltaLambda));
      }
      if (invInertiaA > 0.0) {
        const deltaAngA = gradAngA.clone().scale(-invInertiaA * deltaLambda);
        const orientationCompA = world.getComponent(solverEntityA, OrientationComponent);
        if (orientationCompA?.quaternion) {
          const angle = deltaAngA.length();
          if (angle > epsilon) {
            const axis = deltaAngA.clone().scale(1.0 / angle);
            const dq = new Quaternion().setFromAxisAngle(axis, angle);
            orientationCompA.quaternion.multiplyQuaternions(dq, orientationCompA.quaternion).normalize();
            updateRigidBodyMemberLocalOrientation(world, solverEntityA);
          }
        }
      }
      if (invMassB > 0.0) {
        posBComp.pos.add(gradPosB.clone().scale(-invMassB * deltaLambda));
      }
      if (invInertiaB > 0.0) {
        const deltaAngB = gradAngB.clone().scale(-invInertiaB * deltaLambda);
        const orientationCompB = world.getComponent(solverEntityB, OrientationComponent);
        if (orientationCompB?.quaternion) {
          const angle = deltaAngB.length();
          if (angle > epsilon) {
            const axis = deltaAngB.clone().scale(1.0 / angle);
            const dq = new Quaternion().setFromAxisAngle(axis, angle);
            orientationCompB.quaternion.multiplyQuaternions(dq, orientationCompB.quaternion).normalize();
            updateRigidBodyMemberLocalOrientation(world, solverEntityB);
          }
        }
      }
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
      if (world.getComponent(entityId, RigidBodyMemberComponent)) continue;
      const posComp = world.getComponent(entityId, PositionComponent);
      const velComp = world.getComponent(entityId, VelocityComponent);
      posComp.pos.add(velComp.vel, dt);
    }
  }
}

export class PrevFinalOrientationSystem {
  runInPause = false;
  update(world, dt) {
    const entities = world.query([OrientationComponent, PrevFinalOrientationComponent]);
    for (const entityId of entities) {
      const orientationComp = world.getComponent(entityId, OrientationComponent);
      const prevFinalOrientationComp = world.getComponent(entityId, PrevFinalOrientationComponent);
      prevFinalOrientationComp.quaternion.set(orientationComp.quaternion);
    }
  }
}

export class EncoderUpdateSystem {
  runInPause = false;

  update(world, dt) {
    const entities = world.query([
      OrientationComponent,
      EncoderComponent,
    ]);

    for (const entityId of entities) {
      const orientationComp = world.getComponent(entityId, OrientationComponent);
      const encoderComp = world.getComponent(entityId, EncoderComponent);
      if (!orientationComp?.quaternion || !encoderComp) {
        continue;
      }

      const spoolState = world.getComponent(entityId, SpoolStateComponent);
      const worldOrientation = getEntityWorldOrientation(world, entityId) || orientationComp.quaternion;
      const rigidBodyMemberSpoolFrame = spoolState
        ? getRigidBodyMemberSpoolFrame(world, entityId, spoolState)
        : null;
      const spoolWorldOrientation = rigidBodyMemberSpoolFrame?.worldOrientation || worldOrientation;
      const axis = spoolState
        ? getSpoolWorldAxis(spoolState, spoolWorldOrientation)
        : getEncoderAxis(encoderComp, world);
      const wrappedAngle = spoolState
        ? (
          rigidBodyMemberSpoolFrame
            ? getSpoolRotationAngle(
              rigidBodyMemberSpoolFrame.localSpoolState,
              rigidBodyMemberSpoolFrame.member.localOrientation,
            )
            : getSpoolRotationAngle(spoolState, spoolWorldOrientation)
        )
        : orientationAngleAroundAxis(worldOrientation, axis);
      if (!Number.isFinite(wrappedAngle)) {
        continue;
      }
      encoderComp.angle = Number.isFinite(encoderComp.angle)
        ? unwrapAngleNear(encoderComp.angle, wrappedAngle)
        : wrappedAngle;
      encoderComp.axis = axis;
    }
  }
}

export class RigidBodySyncSystem {
  runInPause = false;

  update(world, _dt) {
    const standaloneSpools = world.query([SpoolStateComponent, OrientationComponent]);
    for (const entityId of standaloneSpools) {
      if (world.getComponent(entityId, RigidBodyMemberComponent)) {
        continue;
      }
      const spoolState = world.getComponent(entityId, SpoolStateComponent);
      const orientationComp = world.getComponent(entityId, OrientationComponent);
      if (spoolState && orientationComp?.quaternion) {
        orientationComp.quaternion.set(
          constrainSpoolOrientation(spoolState, orientationComp.quaternion),
        );
      }

      const angularVelComp = world.getComponent(entityId, AngularVelocityComponent);
      if (spoolState && angularVelComp?.omega && orientationComp?.quaternion) {
        angularVelComp.omega.set(
          constrainSpoolAngularVelocity(spoolState, orientationComp.quaternion, angularVelComp.omega),
        );
      }
    }

    const bodyEntities = world.query([RigidBodyComponent, PositionComponent, OrientationComponent]);
    if (!bodyEntities || bodyEntities.length === 0) {
      return;
    }

    for (const bodyEntityId of bodyEntities) {
      const body = world.getComponent(bodyEntityId, RigidBodyComponent);
      const bodyPosComp = world.getComponent(bodyEntityId, PositionComponent);
      const bodyOrientationComp = world.getComponent(bodyEntityId, OrientationComponent);
      const bodyVelComp = world.getComponent(bodyEntityId, VelocityComponent);
      const bodyAngularVelComp = world.getComponent(bodyEntityId, AngularVelocityComponent);
      const members = Array.isArray(body?.members) ? body.members : [];
      if (!body || !bodyPosComp?.pos || !bodyOrientationComp?.quaternion || members.length < 1) {
        continue;
      }

      if (!body.syncedPosition || !body.syncedOrientation) {
        initializeRigidBodySyncState(world, bodyEntityId);
      }

      const previousBodyOrientation = body.syncedOrientation.clone().normalize();
      const currentBodyOrientation = bodyOrientationComp.quaternion.clone().normalize();
      const bodyDelta = currentBodyOrientation.clone()
        .multiplyQuaternions(
          currentBodyOrientation,
          previousBodyOrientation.clone().conjugate().normalize(),
        )
        .normalize();

      for (const entityId of members) {
        const member = world.getComponent(entityId, RigidBodyMemberComponent);
        if (!member) {
          continue;
        }

        const orientationComp = world.getComponent(entityId, OrientationComponent);
        if (orientationComp?.quaternion) {
          applyOrientationDelta(world, entityId, bodyDelta);
          updateRigidBodyMemberLocalOrientation(world, entityId);
          orientationComp.quaternion.set(
            new Quaternion()
              .multiplyQuaternions(currentBodyOrientation, member.localOrientation)
              .normalize(),
          );
          const spoolState = world.getComponent(entityId, SpoolStateComponent);
          if (spoolState) {
            // Spools are modeled as rigid members with one preserved local
            // twist DOF. Sync follows the parent body, then projects away
            // swing instead of solving a compliant body-rotor hinge.
            orientationComp.quaternion.set(
              constrainSpoolOrientation(spoolState, orientationComp.quaternion),
            );
            updateRigidBodyMemberLocalOrientation(world, entityId);
          }
        }

        const posComp = world.getComponent(entityId, PositionComponent);
        if (posComp?.pos) {
          const rotatedLocal = currentBodyOrientation.transformVector(member.localPosition);
          posComp.pos.set(bodyPosComp.pos.clone().add(rotatedLocal));
        }

        const velComp = world.getComponent(entityId, VelocityComponent);
        if (velComp?.vel) {
          const linearVelocity = bodyVelComp?.vel ? bodyVelComp.vel.clone() : new Vector3(0.0, 0.0, 0.0);
          if (bodyAngularVelComp?.omega && member.localPosition) {
            const worldOffset = currentBodyOrientation.transformVector(member.localPosition);
            linearVelocity.add(bodyAngularVelComp.omega.cross(worldOffset));
          }
          velComp.vel.set(linearVelocity);
        }

        const spoolState = world.getComponent(entityId, SpoolStateComponent);
        const angularVelComp = world.getComponent(entityId, AngularVelocityComponent);
        if (spoolState && angularVelComp?.omega && orientationComp?.quaternion) {
          angularVelComp.omega.set(
            constrainSpoolAngularVelocity(spoolState, orientationComp.quaternion, angularVelComp.omega),
          );
        }
      }

      body.syncedPosition.set(bodyPosComp.pos);
      body.syncedOrientation.set(currentBodyOrientation);
    }
  }
}

export class AngularMovementSystem {
  runInPause = false;
  update(world, dt) {
    const entities = world.query([OrientationComponent, AngularVelocityComponent]);
    const epsilon = 1e-12;

    for (const entityId of entities) {
      if (
        world.getComponent(entityId, RigidBodyMemberComponent)
        && world.getComponent(entityId, SpoolStateComponent)
      ) {
        continue;
      }
      const orientationComp = world.getComponent(entityId, OrientationComponent);
      const angularVelComp = world.getComponent(entityId, AngularVelocityComponent);

      const omega = angularVelComp.omega;
      const speed = omega.length();
      if (speed <= epsilon) continue;

      const axis = omega.clone().scale(1.0 / speed);
      const angle = speed * dt;

      const dq = orientationComp.quaternion.clone().setFromAxisAngle(axis, angle);
      orientationComp.quaternion.multiplyQuaternions(dq, orientationComp.quaternion).normalize();
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
      if (world.getComponent(entityId, RigidBodyMemberComponent)) continue;

      const massComp = world.getComponent(entityId, MassComponent);
      if (massComp && massComp.mass <= 0) continue;

      const posComp = world.getComponent(entityId, PositionComponent);
      const velComp = world.getComponent(entityId, VelocityComponent);
      const prevFinalPosComp = world.getComponent(entityId, PrevFinalPosComponent);

      velComp.vel.subtractVectors(posComp.pos, prevFinalPosComp.pos).scale(1.0 / dt);
    }
  }
}

export class PBDAngularVelocityUpdateSystem {
  runInPause = false;
  update(world, dt) {
    const grabbed = world.getResource('grabbedBall');
    const entities = world.query([
      OrientationComponent,
      AngularVelocityComponent,
      PrevFinalOrientationComponent,
      MomentOfInertiaComponent
    ]);
    const epsilon = 1e-9;

    if (dt <= epsilon) return;

    for (const entityId of entities) {
      if (entityId === grabbed) continue;
      if (world.getComponent(entityId, RigidBodyMemberComponent)) continue;

      const moiComp = world.getComponent(entityId, MomentOfInertiaComponent);
      if (moiComp && moiComp.invInertia <= 0) continue;

      const orientationComp = world.getComponent(entityId, OrientationComponent);
      const angularVelComp = world.getComponent(entityId, AngularVelocityComponent);
      const prevFinalOrientationComp = world.getComponent(entityId, PrevFinalOrientationComponent);

      const qCurr = orientationComp.quaternion;
      const qPrevInv = prevFinalOrientationComp.quaternion.clone().conjugate().normalize();
      const qDelta = qCurr.clone().multiplyQuaternions(qCurr, qPrevInv).normalize();

      if (qDelta.w < 0) {
        qDelta.x *= -1;
        qDelta.y *= -1;
        qDelta.z *= -1;
        qDelta.w *= -1;
      }

      const w = Math.max(-1.0, Math.min(1.0, qDelta.w));
      const angle = 2.0 * Math.acos(w);
      const sinHalf = Math.sqrt(Math.max(0.0, 1.0 - w * w));

      if (sinHalf <= 1e-12 || angle <= epsilon) {
        angularVelComp.omega.x = 0.0;
        angularVelComp.omega.y = 0.0;
        angularVelComp.omega.z = 0.0;
        continue;
      }

      const scale = angle / (dt * sinHalf);
      angularVelComp.omega.x = qDelta.x * scale;
      angularVelComp.omega.y = qDelta.y * scale;
      angularVelComp.omega.z = qDelta.z * scale;
    }
  }
}
