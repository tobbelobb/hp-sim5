import Vector3 from '../../src/js/cable_joints_3d/vector3.js';

export function orientationToDegrees(orientation) {
  if (!orientation || typeof orientation !== 'object') {
    return null;
  }

  if (Number.isFinite(orientation.angle)) {
    return orientation.angle * (180 / Math.PI);
  }

  const quaternion = orientation.quaternion;
  if (!quaternion || typeof quaternion.transformVector !== 'function') {
    return null;
  }

  const axis = quaternion.transformVector(new Vector3(1.0, 0.0, 0.0));
  const x = Number(axis?.x);
  const y = Number(axis?.y);
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    return null;
  }
  return Math.atan2(y, x) * (180 / Math.PI);
}
