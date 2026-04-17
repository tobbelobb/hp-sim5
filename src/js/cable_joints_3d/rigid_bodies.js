import Vector3 from './vector3.js';
import Quaternion from './quaternion.js';
import {
  PositionComponent,
  OrientationComponent,
  RigidBodyComponent,
  RigidBodyMemberComponent,
} from './ecs.js';

function isFiniteQuaternion(quaternion) {
  return Boolean(
    quaternion
    && Number.isFinite(quaternion.x)
    && Number.isFinite(quaternion.y)
    && Number.isFinite(quaternion.z)
    && Number.isFinite(quaternion.w)
  );
}

function identityQuaternion() {
  return new Quaternion(0.0, 0.0, 0.0, 1.0);
}

export function getRigidBodyMember(world, entityId) {
  return world?.getComponent?.(entityId, RigidBodyMemberComponent) || null;
}

export function getRigidBodyEntityForMember(world, entityId) {
  const member = getRigidBodyMember(world, entityId);
  return member?.bodyEntity ?? null;
}

export function getRigidBodyForEntity(world, entityId) {
  if (entityId === null || entityId === undefined) {
    return null;
  }
  if (world?.getComponent?.(entityId, RigidBodyComponent)) {
    return entityId;
  }
  return getRigidBodyEntityForMember(world, entityId);
}

export function isRigidBodyMember(world, entityId) {
  return getRigidBodyMember(world, entityId) !== null;
}

export function getEntityWorldPosition(world, entityId) {
  const member = getRigidBodyMember(world, entityId);
  if (member) {
    const bodyPos = world.getComponent(member.bodyEntity, PositionComponent)?.pos;
    const bodyOrientation = world.getComponent(member.bodyEntity, OrientationComponent)?.quaternion;
    if (bodyPos && bodyOrientation && member.localPosition) {
      return bodyPos.clone().add(bodyOrientation.transformVector(member.localPosition));
    }
  }
  const pos = world.getComponent(entityId, PositionComponent)?.pos;
  return pos ? pos.clone() : null;
}

export function getEntityWorldOrientation(world, entityId) {
  const member = getRigidBodyMember(world, entityId);
  if (member) {
    const bodyOrientation = world.getComponent(member.bodyEntity, OrientationComponent)?.quaternion;
    if (bodyOrientation && isFiniteQuaternion(member.localOrientation)) {
      return new Quaternion()
        .multiplyQuaternions(bodyOrientation, member.localOrientation)
        .normalize();
    }
  }
  const orientation = world.getComponent(entityId, OrientationComponent)?.quaternion;
  return isFiniteQuaternion(orientation) ? orientation.clone().normalize() : null;
}

export function computeWorldAttachment(world, entityId, localPoint) {
  if (!localPoint) {
    return null;
  }
  const position = getEntityWorldPosition(world, entityId);
  const orientation = getEntityWorldOrientation(world, entityId);
  if (!position) {
    return localPoint.clone();
  }
  if (!orientation) {
    return position.clone().add(localPoint);
  }
  return position.clone().add(orientation.transformVector(localPoint));
}

export function computeLocalAttachment(world, entityId, worldPoint) {
  if (!worldPoint) {
    return null;
  }
  const position = getEntityWorldPosition(world, entityId);
  if (!position) {
    return worldPoint.clone();
  }
  const rel = worldPoint.clone().subtract(position);
  const orientation = getEntityWorldOrientation(world, entityId);
  if (!orientation) {
    return rel;
  }
  const inverse = orientation.clone().conjugate().normalize();
  return inverse.transformVector(rel);
}

export function resolveRigidBodySolverEndpoint(world, entityId, counterpartEntityId, worldPoint) {
  const member = getRigidBodyMember(world, entityId);
  if (!member) {
    return {
      entityId,
      localPoint: computeLocalAttachment(world, entityId, worldPoint),
      worldPoint: worldPoint?.clone?.() ?? null,
      internalToBody: false,
    };
  }

  const counterpartMember = getRigidBodyMember(world, counterpartEntityId);
  const internalToBody = counterpartMember && counterpartMember.bodyEntity === member.bodyEntity;
  const solverEntityId = internalToBody ? entityId : member.bodyEntity;

  return {
    entityId: solverEntityId,
    localPoint: computeLocalAttachment(world, solverEntityId, worldPoint),
    worldPoint: worldPoint?.clone?.() ?? null,
    internalToBody,
  };
}

export function updateRigidBodyMemberLocalOrientation(world, entityId) {
  const member = getRigidBodyMember(world, entityId);
  if (!member) {
    return;
  }
  const bodyOrientation = world.getComponent(member.bodyEntity, OrientationComponent)?.quaternion;
  const memberOrientation = world.getComponent(entityId, OrientationComponent)?.quaternion;
  if (!isFiniteQuaternion(bodyOrientation) || !isFiniteQuaternion(memberOrientation)) {
    return;
  }
  const bodyInverse = bodyOrientation.clone().conjugate().normalize();
  member.localOrientation.set(
    new Quaternion().multiplyQuaternions(bodyInverse, memberOrientation).normalize(),
  );
}

export function initializeRigidBodySyncState(world, bodyEntityId) {
  const body = world.getComponent(bodyEntityId, RigidBodyComponent);
  const bodyPos = world.getComponent(bodyEntityId, PositionComponent)?.pos;
  const bodyOrientation = world.getComponent(bodyEntityId, OrientationComponent)?.quaternion;
  if (!body) {
    return;
  }
  body.syncedPosition = bodyPos ? bodyPos.clone() : new Vector3(0.0, 0.0, 0.0);
  body.syncedOrientation = isFiniteQuaternion(bodyOrientation)
    ? bodyOrientation.clone().normalize()
    : identityQuaternion();
}
