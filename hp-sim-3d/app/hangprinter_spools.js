import Vector3 from '../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../src/js/cable_joints_3d/quaternion.js';
import {
  OrientationComponent,
  AngularVelocityComponent,
} from '../../src/js/cable_joints_3d/ecs.js';
import { updateRigidBodyMemberLocalOrientation } from '../../src/js/cable_joints_3d/rigid_bodies.js';

const DEFAULT_SPOOL_AXIS_LOCAL = new Vector3(0.0, 0.0, 1.0);
const EPSILON = 1e-12;

function normalizeAngle(angle) {
  let wrapped = angle;
  while (wrapped > Math.PI) wrapped -= Math.PI * 2.0;
  while (wrapped < -Math.PI) wrapped += Math.PI * 2.0;
  return wrapped;
}

function cloneQuaternionOrIdentity(quaternion) {
  if (
    quaternion &&
    Number.isFinite(quaternion.x) &&
    Number.isFinite(quaternion.y) &&
    Number.isFinite(quaternion.z) &&
    Number.isFinite(quaternion.w)
  ) {
    return quaternion.clone().normalize();
  }
  return new Quaternion();
}

export function normalizeSpoolAxisLocal(axisLike) {
  const axis = axisLike instanceof Vector3
    ? axisLike.clone()
    : new Vector3(
      Number(axisLike?.x ?? axisLike?.[0] ?? 0.0),
      Number(axisLike?.y ?? axisLike?.[1] ?? 0.0),
      Number(axisLike?.z ?? axisLike?.[2] ?? 1.0),
    );
  if (axis.lengthSq() <= EPSILON) {
    return DEFAULT_SPOOL_AXIS_LOCAL.clone();
  }
  return axis.normalize();
}

function twistQuaternionAroundLocalAxis(quaternion, axisLocal) {
  const axis = normalizeSpoolAxisLocal(axisLocal);
  const projection = axis.clone().scale(
    (quaternion.x * axis.x) + (quaternion.y * axis.y) + (quaternion.z * axis.z),
  );
  const twist = new Quaternion(projection.x, projection.y, projection.z, quaternion.w);
  if (twist.lengthSq() <= EPSILON) {
    return new Quaternion();
  }
  return twist.normalize();
}

export function getSpoolLocalAxis(spoolState) {
  return normalizeSpoolAxisLocal(spoolState?.axisLocal);
}

export function decomposeSpoolOrientation(spoolState, orientationQuaternion) {
  const axisLocal = getSpoolLocalAxis(spoolState);
  const referenceOrientation = cloneQuaternionOrIdentity(spoolState?.referenceOrientation);
  const orientation = cloneQuaternionOrIdentity(orientationQuaternion);
  const referenceInverse = referenceOrientation.clone().conjugate().normalize();
  const relativeOrientation = new Quaternion()
    .multiplyQuaternions(referenceInverse, orientation)
    .normalize();
  const twist = twistQuaternionAroundLocalAxis(relativeOrientation, axisLocal);
  const twistInverse = twist.clone().conjugate().normalize();
  const swing = new Quaternion()
    .multiplyQuaternions(relativeOrientation, twistInverse)
    .normalize();
  const signedSinHalf = (twist.x * axisLocal.x) + (twist.y * axisLocal.y) + (twist.z * axisLocal.z);
  const angle = normalizeAngle(2.0 * Math.atan2(signedSinHalf, twist.w));
  return {
    axisLocal,
    swing,
    twist,
    angle,
    referenceOrientation,
  };
}

export function getSpoolRotationAngle(spoolState, orientationQuaternion) {
  return decomposeSpoolOrientation(spoolState, orientationQuaternion).angle;
}

export function getSpoolWorldAxis(spoolState, orientationQuaternion = null) {
  const axisLocal = getSpoolLocalAxis(spoolState);
  const orientation = orientationQuaternion || spoolState?.referenceOrientation;
  if (orientation && typeof orientation.transformVector === 'function') {
    const worldAxis = orientation.transformVector(axisLocal);
    if (worldAxis.lengthSq() > EPSILON) {
      return worldAxis.normalize();
    }
  }
  return axisLocal;
}

export function composeSpoolOrientation(spoolState, swingQuaternion, angle) {
  const axisLocal = getSpoolLocalAxis(spoolState);
  const relativeOrientation = new Quaternion().multiplyQuaternions(
    swingQuaternion ? swingQuaternion.clone().normalize() : new Quaternion(),
    new Quaternion().setFromAxisAngle(axisLocal, angle),
  ).normalize();
  const referenceOrientation = cloneQuaternionOrIdentity(spoolState?.referenceOrientation);
  return new Quaternion().multiplyQuaternions(referenceOrientation, relativeOrientation).normalize();
}

export function constrainSpoolOrientation(spoolState, orientationQuaternion) {
  const { angle } = decomposeSpoolOrientation(spoolState, orientationQuaternion);
  return composeSpoolOrientation(spoolState, null, angle);
}

export function rotateSpoolReferenceOrientation(spoolState, deltaRotation) {
  if (!spoolState?.referenceOrientation || !deltaRotation) {
    return;
  }
  spoolState.referenceOrientation.multiplyQuaternions(
    deltaRotation,
    spoolState.referenceOrientation,
  ).normalize();
}

export function constrainSpoolAngularVelocity(spoolState, orientationQuaternion, angularVelocityLike) {
  const worldAxis = getSpoolWorldAxis(spoolState, orientationQuaternion);
  const projectedSpeed = angularVelocityLike?.dot?.(worldAxis) ?? 0.0;
  return worldAxis.scale(projectedSpeed);
}

export class SpoolTagComponent {}

export class SpoolStateComponent {
  constructor(axis = null, axisLocal = null, referenceOrientation = null) {
    this.axis = axis;
    this.axisLocal = normalizeSpoolAxisLocal(axisLocal);
    this.referenceOrientation = cloneQuaternionOrIdentity(referenceOrientation);
  }
}

export class SpoolAxisConstraintSystem {
  runInPause = false;

  update(world, _dt) {
    const entities = world.query([SpoolStateComponent, OrientationComponent]);
    for (const entityId of entities) {
      const spoolState = world.getComponent(entityId, SpoolStateComponent);
      const orientation = world.getComponent(entityId, OrientationComponent);
      if (!spoolState || !orientation?.quaternion) {
        continue;
      }

      orientation.quaternion.set(constrainSpoolOrientation(spoolState, orientation.quaternion));
      updateRigidBodyMemberLocalOrientation(world, entityId);

      const angularVelocity = world.getComponent(entityId, AngularVelocityComponent);
      if (angularVelocity?.omega) {
        angularVelocity.omega.set(
          constrainSpoolAngularVelocity(spoolState, orientation.quaternion, angularVelocity.omega),
        );
      }
    }
  }
}
