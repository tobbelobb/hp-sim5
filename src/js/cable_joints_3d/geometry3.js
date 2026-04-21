import Vector3 from './vector3.js';
import Vector2 from '../cable_joints/vector2.js';
import {
  _tangentPointCircle,
  tangentFromCircleToCircle,
  signedArcLengthOnWheel as signedArcLengthOnWheel2D,
  rightOfLine
} from '../cable_joints/geometry.js';

const EPSILON = 1e-9;

function buildPlaneBasis(planeNormal) {
  const n = planeNormal.clone();
  if (n.lengthSq() <= EPSILON) {
    console.warn('Cable tangent calculation: planeNormal is near zero. Using default basis.');
    return {
      n: new Vector3(0, 0, 1),
      u: new Vector3(1, 0, 0),
      v: new Vector3(0, 1, 0)
    };
  }

  n.normalize();
  let reference = Math.abs(n.x) < 0.9 ? new Vector3(1, 0, 0) : new Vector3(0, 1, 0);
  // Remove the normal component to keep reference in the plane.
  const nDotRef = n.dot(reference);
  let u = reference.clone().subtract(n, nDotRef);
  if (u.lengthSq() <= EPSILON) {
    reference = new Vector3(0, 0, 1);
    const nDotRef2 = n.dot(reference);
    u = reference.clone().subtract(n, nDotRef2);
  }
  u.normalize();
  const v = n.cross(u);
  return { n, u, v };
}

function projectToPlane2D(point, origin, basis) {
  const rel = new Vector3().subtractVectors(point, origin);
  return new Vector2(rel.dot(basis.u), rel.dot(basis.v));
}

function liftFromPlane2D(point2, origin, basis) {
  return origin.clone().add(basis.u, point2.x).add(basis.v, point2.y);
}

export function closestPointOnSegment(p, a, b) {
  const ab = new Vector3().subtractVectors(b, a);
  const ap = new Vector3().subtractVectors(p, a);
  let t = ap.dot(ab);
  if (t <= 0.0) return a.clone();
  const denom = ab.dot(ab);
  if (t >= denom) return b.clone();
  t = t / denom;
  return a.clone().add(ab, t);
}

export function lineSegmentSphereIntersection(p1, p2, center, radius, isAPierceAnIntersection = false) {
  if (p1.distanceTo(center) <= radius || p2.distanceTo(center) <= radius) {
    return isAPierceAnIntersection;
  }
  const d = new Vector3().subtractVectors(p2, p1);
  const lc = new Vector3().subtractVectors(center, p1);
  const dLenSq = d.lengthSq();
  let t = lc.dot(d);
  if (dLenSq > 1e-9) t /= dLenSq;
  if (t < 0.0 || t > 1.0) return false;
  const closest = p1.clone().add(d, t);
  return closest.distanceTo(center) <= radius;
}

/**
 * Calculates the tangent point on a sphere for a cable coming from an attachment point.
 * This is a simplified 3D version. It assumes the cable wraps in a plane.
 * @param {Vector3} p_attach - The attachment point in world space.
 * @param {Vector3} p_sphere - The center of the sphere in world space.
 * @param {number} r_sphere - The radius of the sphere.
 * @param {Vector3} wrap_axis - A vector defining the plane of wrapping (normal to the plane).
 * @returns {{a_attach: Vector3, a_sphere: Vector3}} The attachment point and the tangent point on the sphere.
 */
export function _tangentPointSphere(p_attach, p_sphere, r_sphere, planeNormal, cw, pointIsFirst) {
  const basis = buildPlaneBasis(planeNormal);
  const origin = p_sphere;
  const pAttach2 = projectToPlane2D(p_attach, origin, basis);
  const pSphere2 = projectToPlane2D(p_sphere, origin, basis);

  const result2 = _tangentPointCircle(pAttach2, pSphere2, r_sphere, cw, pointIsFirst);
  return {
    a_attach: p_attach.clone(),
    a_sphere: liftFromPlane2D(result2.a_circle, origin, basis)
  };
}

export function tangentFromPointToSphere(p_attach, p_sphere, r_sphere, planeNormal, cw) {
  return _tangentPointSphere(p_attach, p_sphere, r_sphere, planeNormal, cw, true);
}

export function tangentFromSphereToPoint(p_attach, p_sphere, r_sphere, planeNormal, cw) {
  return _tangentPointSphere(p_attach, p_sphere, r_sphere, planeNormal, cw, false);
}

/**
 * Calculates the common tangent points between two spheres.
 * This is highly complex. This is a placeholder implementation.
 * @param {Vector3} posA - Center of sphere A.
 * @param {number} radiusA - Radius of sphere A.
 * @param {Vector3} wrap_axisA - Wrap axis for sphere A.
 * @param {Vector3} posB - Center of sphere B.
 * @param {number} radiusB - Radius of sphere B.
 * @param {Vector3} wrap_axisB - Wrap axis for sphere B.
 * @returns {{a_sphere: Vector3, b_sphere: Vector3}} The tangent points on each sphere.
 */
export function tangentFromSphereToSphere(posA, radiusA, cwA, posB, radiusB, cwB, planeNormal) {
  const basis = buildPlaneBasis(planeNormal);
  const origin = posA;
  const posA2 = projectToPlane2D(posA, origin, basis);
  const posB2 = projectToPlane2D(posB, origin, basis);

  const result2 = tangentFromCircleToCircle(posA2, radiusA, cwA, posB2, radiusB, cwB);
  return {
    a_sphere: liftFromPlane2D(result2.a_circle, origin, basis),
    b_sphere: liftFromPlane2D(result2.b_circle, origin, basis)
  };
}

export function signedArcLengthOnWheel(prevPoint, currPoint, center, radius, clockwisePreference, planeNormal, force_positive = false) {
  const basis = buildPlaneBasis(planeNormal ?? new Vector3(0, 0, 1));
  const prev2 = projectToPlane2D(prevPoint, center, basis);
  const curr2 = projectToPlane2D(currPoint, center, basis);
  const center2 = new Vector2(0, 0);
  return signedArcLengthOnWheel2D(prev2, curr2, center2, radius, clockwisePreference, force_positive);
}

export function rightOfPlane(x, p0, p1, planeNormal) {
  const basis = buildPlaneBasis(planeNormal ?? new Vector3(0, 0, 1));
  const x2 = projectToPlane2D(x, p0, basis);
  const p12 = projectToPlane2D(p1, p0, basis);
  const p02 = new Vector2(0, 0);
  return rightOfLine(x2, p02, p12);
}
