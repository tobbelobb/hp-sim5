import Quaternion from '../cable_joints_3d/quaternion.js';
import Vector3 from '../cable_joints_3d/vector3.js';
import { getAttribute } from './stage.js';

const RESET = '!resetXformStack!';
const INVERT = '!invert!';
const AXES = {
  X: new Vector3(1, 0, 0),
  Y: new Vector3(0, 1, 0),
  Z: new Vector3(0, 0, 1),
};
const DEGREES_TO_RADIANS = Math.PI / 180;

function readLocalTransform(prim, path) {
  const order = getAttribute(prim, 'xformOpOrder') ?? [];
  if (!Array.isArray(order) || order.some(op => typeof op !== 'string')) {
    throw new Error(`${path}: invalid USD xformOpOrder.`);
  }

  const position = new Vector3();
  const orientation = new Quaternion();
  const resetIndex = order.lastIndexOf(RESET);
  // USD lists the least local op first. Append each op to the current frame.
  for (const token of order.slice(resetIndex + 1)) {
    const inverse = token.startsWith(INVERT);
    const name = inverse ? token.slice(INVERT.length) : token;
    const value = getAttribute(prim, name, { resolvePrecision: true });
    // USD skips order entries whose referenced attribute has no value.
    if (value == null) continue;

    const op = name.match(/^xformOp:(translate|orient|rotate(XYZ|XZY|YXZ|YZX|ZXY|ZYX|X|Y|Z))(?::.+)?$/);
    if (!op) throw new Error(`${path}: unsupported USD transform op ${token}.`);
    if (op[1] === 'translate' && isTuple(value, 3)) {
      const offset = new Vector3(...value).scale(inverse ? -1 : 1);
      position.add(orientation.transformVector(offset));
      continue;
    }

    const rotation = readRotation(op, value);
    if (!rotation) throw new Error(`${path}: invalid USD transform value for ${token}.`);
    if (inverse) rotation.conjugate();
    orientation.multiply(rotation).normalize();
  }
  return { position, orientation, resetsParent: resetIndex >= 0 };
}

function isTuple(value, size) {
  return Array.isArray(value) && value.length === size && value.every(Number.isFinite);
}

function readRotation(op, value) {
  if (op[1] === 'orient' && isTuple(value, 4) && value.some(component => component !== 0)) {
    const [w, x, y, z] = value;
    return new Quaternion(x, y, z, w).normalize();
  }

  const axes = op[2];
  if (!axes) return null;
  if (axes.length === 1) {
    return Number.isFinite(value)
      ? new Quaternion().setFromAxisAngle(AXES[axes], value * DEGREES_TO_RADIANS)
      : null;
  }
  if (!isTuple(value, 3)) return null;

  const rotation = new Quaternion();
  for (const axis of axes) {
    const degrees = value['XYZ'.indexOf(axis)];
    const axisRotation = new Quaternion().setFromAxisAngle(AXES[axis], degrees * DEGREES_TO_RADIANS);
    rotation.multiplyQuaternions(axisRotation, rotation);
  }
  return rotation.normalize();
}

export function createRigidTransformReader(getPrimAtPath) {
  const cache = new Map();
  function readWorldTransform(path) {
    if (cache.has(path)) return cache.get(path);

    const prim = getPrimAtPath(path);
    const { position, orientation, resetsParent } = readLocalTransform(prim, path);
    const parentPath = path.slice(0, path.lastIndexOf('/'));
    if (parentPath && !resetsParent) {
      const parent = readWorldTransform(parentPath);
      parent.orientation.transformVectorInto(position, position).add(parent.position);
      orientation.multiplyQuaternions(parent.orientation, orientation).normalize();
    }

    const transform = { position, orientation };
    cache.set(path, transform);
    return transform;
  }
  return readWorldTransform;
}
