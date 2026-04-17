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
  PrevRigidBodyLocalOrientationComponent,
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

function quaternionDot(a, b) {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
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

function negateQuaternion(quaternion) {
  quaternion.x *= -1.0;
  quaternion.y *= -1.0;
  quaternion.z *= -1.0;
  quaternion.w *= -1.0;
  return quaternion;
}

function chooseReferenceTriangle(restLocal) {
  if (!Array.isArray(restLocal) || restLocal.length < 3) {
    return null;
  }
  const epsilon = 1e-12;
  for (let i = 0; i < restLocal.length - 2; i += 1) {
    const a = restLocal[i];
    if (!a) continue;
    for (let j = i + 1; j < restLocal.length - 1; j += 1) {
      const b = restLocal[j];
      if (!b) continue;
      const edge = b.clone().subtract(a);
      if (edge.lengthSq() <= epsilon) continue;
      for (let k = j + 1; k < restLocal.length; k += 1) {
        const c = restLocal[k];
        if (!c) continue;
        const span = c.clone().subtract(a);
        if (edge.clone().cross(span).lengthSq() > epsilon) {
          return [i, j, k];
        }
      }
    }
  }
  return null;
}

function buildOrthonormalFrame(a, b, c) {
  const x = b.clone().subtract(a);
  if (x.lengthSq() <= 1e-12) {
    return null;
  }
  x.normalize();
  const span = c.clone().subtract(a);
  const z = x.clone().cross(span);
  if (z.lengthSq() <= 1e-12) {
    return null;
  }
  z.normalize();
  const y = z.clone().cross(x).normalize();
  return { x, y, z };
}

function quaternionFromRotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22) {
  const trace = m00 + m11 + m22;
  if (trace > 0.0) {
    const s = Math.sqrt(trace + 1.0) * 2.0;
    return new Quaternion(
      (m21 - m12) / s,
      (m02 - m20) / s,
      (m10 - m01) / s,
      0.25 * s
    ).normalize();
  }
  if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1.0 + m00 - m11 - m22) * 2.0;
    return new Quaternion(
      0.25 * s,
      (m01 + m10) / s,
      (m02 + m20) / s,
      (m21 - m12) / s
    ).normalize();
  }
  if (m11 > m22) {
    const s = Math.sqrt(1.0 + m11 - m00 - m22) * 2.0;
    return new Quaternion(
      (m01 + m10) / s,
      0.25 * s,
      (m12 + m21) / s,
      (m02 - m20) / s
    ).normalize();
  }
  const s = Math.sqrt(1.0 + m22 - m00 - m11) * 2.0;
  return new Quaternion(
    (m02 + m20) / s,
    (m12 + m21) / s,
    0.25 * s,
    (m10 - m01) / s
  ).normalize();
}

function estimateGroupRotation(world, members, group) {
  const referenceTriangle = Array.isArray(group.referenceTriangle) ? group.referenceTriangle : null;
  if (!referenceTriangle) {
    return group.prevRotation?.clone?.().normalize?.() || new Quaternion();
  }

  const [i, j, k] = referenceTriangle;
  const restA = group.restLocal?.[i];
  const restB = group.restLocal?.[j];
  const restC = group.restLocal?.[k];
  const currentA = world.getComponent(members[i], PositionComponent)?.pos;
  const currentB = world.getComponent(members[j], PositionComponent)?.pos;
  const currentC = world.getComponent(members[k], PositionComponent)?.pos;
  if (!restA || !restB || !restC || !currentA || !currentB || !currentC) {
    return group.prevRotation?.clone?.().normalize?.() || new Quaternion();
  }

  const restFrame = buildOrthonormalFrame(restA, restB, restC);
  const currentFrame = buildOrthonormalFrame(currentA, currentB, currentC);
  if (!restFrame || !currentFrame) {
    return group.prevRotation?.clone?.().normalize?.() || new Quaternion();
  }

  const m00 = currentFrame.x.x * restFrame.x.x + currentFrame.y.x * restFrame.y.x + currentFrame.z.x * restFrame.z.x;
  const m01 = currentFrame.x.x * restFrame.x.y + currentFrame.y.x * restFrame.y.y + currentFrame.z.x * restFrame.z.y;
  const m02 = currentFrame.x.x * restFrame.x.z + currentFrame.y.x * restFrame.y.z + currentFrame.z.x * restFrame.z.z;
  const m10 = currentFrame.x.y * restFrame.x.x + currentFrame.y.y * restFrame.y.x + currentFrame.z.y * restFrame.z.x;
  const m11 = currentFrame.x.y * restFrame.x.y + currentFrame.y.y * restFrame.y.y + currentFrame.z.y * restFrame.z.y;
  const m12 = currentFrame.x.y * restFrame.x.z + currentFrame.y.y * restFrame.y.z + currentFrame.z.y * restFrame.z.z;
  const m20 = currentFrame.x.z * restFrame.x.x + currentFrame.y.z * restFrame.y.x + currentFrame.z.z * restFrame.z.x;
  const m21 = currentFrame.x.z * restFrame.x.y + currentFrame.y.z * restFrame.y.y + currentFrame.z.z * restFrame.z.y;
  const m22 = currentFrame.x.z * restFrame.x.z + currentFrame.y.z * restFrame.y.z + currentFrame.z.z * restFrame.z.z;

  const rotation = quaternionFromRotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  if (group.prevRotation instanceof Quaternion && quaternionDot(rotation, group.prevRotation) < 0.0) {
    negateQuaternion(rotation);
  }
  return rotation;
}

function estimatePlanarGroupAngle(world, members, group, com, planeNormal) {
  const basis = planeBasisForAxis(planeNormal);
  let sumX = 0.0;
  let sumY = 0.0;
  for (let index = 0; index < members.length; index += 1) {
    const entityId = members[index];
    const pos = world.getComponent(entityId, PositionComponent)?.pos;
    const mass = world.getComponent(entityId, MassComponent)?.mass ?? 0.0;
    const restRel = group.restLocal?.[index];
    if (!pos || !restRel || !(mass > 0.0)) {
      continue;
    }
    const currentRel = pos.clone().subtract(com);
    const restU = restRel.dot(basis.u);
    const restV = restRel.dot(basis.v);
    const currentU = currentRel.dot(basis.u);
    const currentV = currentRel.dot(basis.v);
    sumX += mass * (restU * currentU + restV * currentV);
    sumY += mass * (restU * currentV - restV * currentU);
  }
  return Math.atan2(sumY, sumX);
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

      const posAComp = world.getComponent(entityA, PositionComponent);
      const posBComp = world.getComponent(entityB, PositionComponent);
      if (!posAComp?.pos || !posBComp?.pos) {
        continue;
      }

      const massAComp = world.getComponent(entityA, MassComponent);
      const massBComp = world.getComponent(entityB, MassComponent);
      const invMassA = massAComp && massAComp.mass > 0 ? 1.0 / massAComp.mass : 0.0;
      const invMassB = massBComp && massBComp.mass > 0 ? 1.0 / massBComp.mass : 0.0;
      if (invMassA + invMassB <= epsilon) {
        continue;
      }

      const diff = new Vector3().subtractVectors(posBComp.pos, posAComp.pos);
      const currentLength = diff.length();
      if (currentLength <= epsilon) {
        continue;
      }

      const dir = diff.clone().scale(1.0 / currentLength);
      const violation = currentLength - constraint.restLength;
      const alphaTilde = constraint.compliance / (dt * dt);
      const denominator = invMassA + invMassB + alphaTilde;
      if (denominator <= epsilon) {
        continue;
      }

      const deltaLambda = (-violation - alphaTilde * constraint.lambda) / denominator;
      constraint.lambda += deltaLambda;

      const correction = dir.clone().scale(deltaLambda);
      if (invMassA > 0.0) {
        posAComp.pos.add(correction, -invMassA);
      }
      if (invMassB > 0.0) {
        posBComp.pos.add(correction, invMassB);
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

export class PrevRigidBodyLocalOrientationSystem {
  runInPause = false;

  update(world, _dt) {
    const entities = world.query([RigidBodyMemberComponent, PrevRigidBodyLocalOrientationComponent]);
    for (const entityId of entities) {
      const member = world.getComponent(entityId, RigidBodyMemberComponent);
      const prevLocal = world.getComponent(entityId, PrevRigidBodyLocalOrientationComponent);
      if (!member?.localOrientation || !prevLocal?.quaternion) {
        continue;
      }
      prevLocal.quaternion.set(member.localOrientation);
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
      const axis = spoolState
        ? getSpoolWorldAxis(spoolState, orientationComp.quaternion)
        : getEncoderAxis(encoderComp, world);
      const wrappedAngle = spoolState
        ? getSpoolRotationAngle(spoolState, orientationComp.quaternion)
        : orientationAngleAroundAxis(orientationComp.quaternion, axis);
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

export class RigidBodyMemberAngularVelocityUpdateSystem {
  runInPause = false;

  update(world, dt) {
    const entities = world.query([
      RigidBodyMemberComponent,
      AngularVelocityComponent,
      PrevRigidBodyLocalOrientationComponent,
    ]);
    const epsilon = 1e-9;

    if (dt <= epsilon) return;

    for (const entityId of entities) {
      const member = world.getComponent(entityId, RigidBodyMemberComponent);
      const prevLocal = world.getComponent(entityId, PrevRigidBodyLocalOrientationComponent);
      const angularVel = world.getComponent(entityId, AngularVelocityComponent);
      const bodyOrientation = world.getComponent(member?.bodyEntity, OrientationComponent)?.quaternion;
      if (!member?.localOrientation || !prevLocal?.quaternion || !angularVel?.omega || !bodyOrientation) {
        continue;
      }

      const qCurr = member.localOrientation.clone().normalize();
      const qPrevInv = prevLocal.quaternion.clone().conjugate().normalize();
      const qDelta = qCurr.clone().multiplyQuaternions(qCurr, qPrevInv).normalize();

      if (qDelta.w < 0.0) {
        qDelta.x *= -1.0;
        qDelta.y *= -1.0;
        qDelta.z *= -1.0;
        qDelta.w *= -1.0;
      }

      const w = Math.max(-1.0, Math.min(1.0, qDelta.w));
      const angle = 2.0 * Math.acos(w);
      const sinHalf = Math.sqrt(Math.max(0.0, 1.0 - (w * w)));

      if (sinHalf <= 1e-12 || angle <= epsilon) {
        angularVel.omega.x = 0.0;
        angularVel.omega.y = 0.0;
        angularVel.omega.z = 0.0;
        continue;
      }

      const axisLocal = new Vector3(
        qDelta.x / sinHalf,
        qDelta.y / sinHalf,
        qDelta.z / sinHalf,
      );
      const axisWorld = bodyOrientation.transformVector(axisLocal).normalize();
      const scale = angle / dt;
      angularVel.omega.x = axisWorld.x * scale;
      angularVel.omega.y = axisWorld.y * scale;
      angularVel.omega.z = axisWorld.z * scale;
    }
  }
}
