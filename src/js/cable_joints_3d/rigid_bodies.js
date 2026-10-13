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

export function getRigidBodyMemberComponent(world, entityId) {
  return world?.getComponent?.(entityId, RigidBodyMemberComponent) || null;
}

export function getRigidBodyEntityForMember(world, entityId) {
  const member = getRigidBodyMemberComponent(world, entityId);
  return member?.bodyEntity ?? null;
}

// Destinations must be caller-owned, not live ECS component values.
// Omitting the destination continues to return a detached value.
export function getEntityWorldPosition(world, entityId, output = new Vector3()) {
  const member = getRigidBodyMemberComponent(world, entityId);
  if (member) {
    const bodyPos = world.getComponent(member.bodyEntity, PositionComponent)?.pos;
    const bodyOrientation = world.getComponent(member.bodyEntity, OrientationComponent)?.quaternion;
    if (bodyPos && bodyOrientation && member.localPosition) {
      bodyOrientation.transformVectorInto(member.localPosition, output);
      return output.addVectors(bodyPos, output);
    }
  }
  const pos = world.getComponent(entityId, PositionComponent)?.pos;
  if (!pos) {
    return null;
  }
  output.set(pos);
  return output;
}

export function getEntityWorldOrientation(world, entityId, output = new Quaternion()) {
  const member = getRigidBodyMemberComponent(world, entityId);
  if (member) {
    const bodyOrientation = world.getComponent(member.bodyEntity, OrientationComponent)?.quaternion;
    if (bodyOrientation && isFiniteQuaternion(member.localOrientation)) {
      return output
        .multiplyQuaternions(bodyOrientation, member.localOrientation)
        .normalize();
    }
  }
  const orientation = world.getComponent(entityId, OrientationComponent)?.quaternion;
  return isFiniteQuaternion(orientation) ? output.set(orientation).normalize() : null;
}

export function createAttachmentTransformScratch() {
  return { position: new Vector3(), orientation: new Quaternion() };
}

// Scratch is overwritten by each call and must be distinct from inputs, output,
// and ECS components. The output may alias the input point. Retained results
// need their own output vectors, even when sharing this scratch sequentially.
export function computeWorldAttachment(
  world, entityId, localPoint, output = new Vector3(), scratch = createAttachmentTransformScratch(),
) {
  if (!localPoint) {
    return null;
  }
  const position = getEntityWorldPosition(world, entityId, scratch.position);
  const orientation = getEntityWorldOrientation(world, entityId, scratch.orientation);
  if (!position) {
    output.set(localPoint);
    return output;
  }
  if (!orientation) {
    output.set(localPoint);
  } else {
    orientation.transformVectorInto(localPoint, output);
  }
  return output.addVectors(position, output);
}

export function computeLocalAttachment(
  world, entityId, worldPoint, output = new Vector3(), scratch = createAttachmentTransformScratch(),
) {
  if (!worldPoint) {
    return null;
  }
  const position = getEntityWorldPosition(world, entityId, scratch.position);
  if (!position) {
    output.set(worldPoint);
    return output;
  }
  output.subtractVectors(worldPoint, position);
  const orientation = getEntityWorldOrientation(world, entityId, scratch.orientation);
  if (!orientation) {
    return output;
  }
  orientation.conjugate().normalize();
  return orientation.transformVectorInto(output, output);
}

export function resolveRigidBodySolverEndpoint(world, entityId, counterpartEntityId, worldPoint, scratch) {
  const member = getRigidBodyMemberComponent(world, entityId);
  if (!member) {
    return {
      entityId,
      localPoint: computeLocalAttachment(world, entityId, worldPoint, new Vector3(), scratch),
      worldPoint: worldPoint?.clone?.() ?? null,
      internalToBody: false,
    };
  }

  const counterpartMember = getRigidBodyMemberComponent(world, counterpartEntityId);
  const internalToBody = counterpartMember && counterpartMember.bodyEntity === member.bodyEntity;
  const solverEntityId = internalToBody ? entityId : member.bodyEntity;

  return {
    entityId: solverEntityId,
    localPoint: computeLocalAttachment(world, solverEntityId, worldPoint, new Vector3(), scratch),
    worldPoint: worldPoint?.clone?.() ?? null,
    internalToBody,
  };
}

export function updateRigidBodyMemberLocalOrientation(world, entityId) {
  const member = getRigidBodyMemberComponent(world, entityId);
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
