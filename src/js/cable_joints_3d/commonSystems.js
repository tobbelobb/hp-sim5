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
  RigidGroupComponent
} from './ecs.js';

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
}

function negateQuaternion(quaternion) {
  quaternion.x *= -1.0;
  quaternion.y *= -1.0;
  quaternion.z *= -1.0;
  quaternion.w *= -1.0;
  return quaternion;
}

function normalizeVector4(vector) {
  const length = Math.hypot(vector[0], vector[1], vector[2], vector[3]);
  if (length <= 1e-12) {
    vector[0] = 1.0;
    vector[1] = 0.0;
    vector[2] = 0.0;
    vector[3] = 0.0;
    return vector;
  }
  vector[0] /= length;
  vector[1] /= length;
  vector[2] /= length;
  vector[3] /= length;
  return vector;
}

function dominantEigenQuaternion(matrix, initialQuaternion = null) {
  const vector = initialQuaternion
    ? [initialQuaternion.w, initialQuaternion.x, initialQuaternion.y, initialQuaternion.z]
    : [1.0, 0.0, 0.0, 0.0];
  normalizeVector4(vector);

  for (let iteration = 0; iteration < 32; iteration += 1) {
    const next = [
      matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2] + matrix[0][3] * vector[3],
      matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2] + matrix[1][3] * vector[3],
      matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2] + matrix[2][3] * vector[3],
      matrix[3][0] * vector[0] + matrix[3][1] * vector[1] + matrix[3][2] * vector[2] + matrix[3][3] * vector[3],
    ];
    normalizeVector4(next);
    vector[0] = next[0];
    vector[1] = next[1];
    vector[2] = next[2];
    vector[3] = next[3];
  }

  return new Quaternion(vector[1], vector[2], vector[3], vector[0]).normalize();
}

function bestFitRigidRotation(members, group, world, com) {
  let sxx = 0.0;
  let sxy = 0.0;
  let sxz = 0.0;
  let syx = 0.0;
  let syy = 0.0;
  let syz = 0.0;
  let szx = 0.0;
  let szy = 0.0;
  let szz = 0.0;

  for (let index = 0; index < members.length; index += 1) {
    const entityId = members[index];
    const pos = world.getComponent(entityId, PositionComponent)?.pos;
    const mass = world.getComponent(entityId, MassComponent)?.mass ?? 0.0;
    if (!pos || !(mass > 0.0)) {
      continue;
    }

    const currentRel = pos.clone().subtract(com);
    const restRel = group.restLocal[index] || new Vector3(0, 0, 0);

    sxx += mass * currentRel.x * restRel.x;
    sxy += mass * currentRel.x * restRel.y;
    sxz += mass * currentRel.x * restRel.z;
    syx += mass * currentRel.y * restRel.x;
    syy += mass * currentRel.y * restRel.y;
    syz += mass * currentRel.y * restRel.z;
    szx += mass * currentRel.z * restRel.x;
    szy += mass * currentRel.z * restRel.y;
    szz += mass * currentRel.z * restRel.z;
  }

  const covarianceMagnitude = Math.abs(sxx) + Math.abs(sxy) + Math.abs(sxz)
    + Math.abs(syx) + Math.abs(syy) + Math.abs(syz)
    + Math.abs(szx) + Math.abs(szy) + Math.abs(szz);
  if (covarianceMagnitude <= 1e-12) {
    return group.prevRotation?.clone?.().normalize?.() || new Quaternion();
  }

  const trace = sxx + syy + szz;
  const d0 = syz - szy;
  const d1 = szx - sxz;
  const d2 = sxy - syx;
  const matrix = [
    [trace, d0, d1, d2],
    [d0, sxx - syy - szz, sxy + syx, szx + sxz],
    [d1, sxy + syx, -sxx + syy - szz, syz + szy],
    [d2, szx + sxz, syz + szy, -sxx - syy + szz],
  ];

  const initialRotation = group.prevRotation instanceof Quaternion
    ? group.prevRotation.clone().conjugate().normalize()
    : null;
  const rotation = dominantEigenQuaternion(matrix, initialRotation).conjugate().normalize();
  if (group.prevRotation instanceof Quaternion && quaternionDot(rotation, group.prevRotation) < 0.0) {
    negateQuaternion(rotation);
  }
  return rotation;
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

      const axis = getEncoderAxis(encoderComp, world);
      const wrappedAngle = orientationAngleAroundAxis(orientationComp.quaternion, axis);
      if (!Number.isFinite(wrappedAngle)) {
        continue;
      }
      encoderComp.angle = Number.isFinite(encoderComp.angle)
        ? unwrapAngleNear(encoderComp.angle, wrappedAngle)
        : wrappedAngle;
    }
  }
}

export class RigidGroupSystem {
  runInPause = false;

  _computeCOM(world, members) {
    let sumMass = 0.0;
    const com = new Vector3(0, 0, 0);
    for (const id of members) {
      const pos = world.getComponent(id, PositionComponent)?.pos;
      const mass = world.getComponent(id, MassComponent)?.mass ?? 0.0;
      if (!pos || !(mass > 0.0)) {
        continue;
      }
      com.add(pos, mass);
      sumMass += mass;
    }
    if (sumMass > 0.0) {
      com.scale(1.0 / sumMass);
    }
    return { com, sumMass };
  }

  update(world, dt) {
    const groupEntities = world.query([RigidGroupComponent]);
    if (!groupEntities || groupEntities.length === 0) {
      return;
    }

    for (const groupId of groupEntities) {
      const group = world.getComponent(groupId, RigidGroupComponent);
      const members = Array.isArray(group?.members) ? group.members : [];
      if (members.length < 2) {
        continue;
      }

      if (!group.restLocal) {
        const { com } = this._computeCOM(world, members);
        group.restLocal = members.map((entityId) => {
          const pos = world.getComponent(entityId, PositionComponent)?.pos;
          return pos ? pos.clone().subtract(com) : new Vector3(0, 0, 0);
        });
      }

      const { com, sumMass } = this._computeCOM(world, members);
      if (!(sumMass > 0.0)) {
        continue;
      }
      const rotation = bestFitRigidRotation(members, group, world, com);
      const stiffness = Math.max(0.0, Math.min(1.0, group.stiffness ?? 1.0));

      for (let index = 0; index < members.length; index += 1) {
        const entityId = members[index];
        const posComp = world.getComponent(entityId, PositionComponent);
        if (!posComp?.pos) {
          continue;
        }
        const restRel = group.restLocal[index] || new Vector3(0, 0, 0);
        const target = rotation.transformVector(restRel).add(com);
        const delta = new Vector3().subtractVectors(target, posComp.pos);
        posComp.pos.add(delta, stiffness);
      }

      const previousRotation = group.prevRotation instanceof Quaternion
        ? group.prevRotation.clone().normalize()
        : new Quaternion();
      const deltaRotation = rotation.clone()
        .multiplyQuaternions(rotation, previousRotation.clone().conjugate().normalize())
        .normalize();
      const rotationDeltaMagnitude = Math.abs(deltaRotation.x) + Math.abs(deltaRotation.y)
        + Math.abs(deltaRotation.z) + Math.abs(deltaRotation.w - 1.0);
      if (rotationDeltaMagnitude > 1e-12) {
        for (const entityId of members) {
          applyOrientationDelta(world, entityId, deltaRotation);
        }
      }
      group.prevRotation = rotation.clone();
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
