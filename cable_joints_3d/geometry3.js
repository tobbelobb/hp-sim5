import Vector3 from './vector3.js';

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

export function lineSegmentSphereIntersection(p1, p2, center, radius) {
  if (p1.distanceTo(center) <= radius || p2.distanceTo(center) <= radius) {
    return true;
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
export function tangentFromPointToSphere(p_attach, p_sphere, r_sphere, wrap_axis) {
    // This is a complex geometric problem that depends on the wrapping strategy.
    // The 2D version had a 'cw' flag to resolve ambiguity between two tangents.
    // In 3D, there's a circle of possible tangent points.
    // This simplified version returns a point on the direct line of sight.
    const dir = new Vector3().subtractVectors(p_attach, p_sphere).normalize();
    return {
        a_attach: p_attach.clone(),
        a_sphere: p_sphere.clone().add(dir, r_sphere)
    };
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
export function tangentFromSphereToSphere(posA, radiusA, wrap_axisA, posB, radiusB, wrap_axisB) {
    // This is a very complex geometric problem.
    // A simplified approach is to find the tangent on a 2D plane and project back to 3D.
    // For now, returning a simple straight line connection on the direct line of sight.
    const dir = new Vector3().subtractVectors(posB, posA).normalize();
    return {
        a_sphere: posA.clone().add(dir, radiusA),
        b_sphere: posB.clone().subtract(dir, radiusB)
    };
}

/**
 * Calculates the arc length between two points on a sphere's surface.
 * @param {Vector3} p1 - Start point on sphere surface.
 * @param {Vector3} p2 - End point on sphere surface.
 * @param {Vector3} center - Center of the sphere.
 * @param {number} radius - Radius of the sphere.
 * @returns {number} The arc length.
 */
export function arcLengthOnSphere(p1, p2, center, radius) {
    const v1 = new Vector3().subtractVectors(p1, center).normalize();
    const v2 = new Vector3().subtractVectors(p2, center).normalize();
    const angle = Math.acos(Math.max(-1, Math.min(1, v1.dot(v2)))); // Clamp for precision
    return radius * angle;
}
