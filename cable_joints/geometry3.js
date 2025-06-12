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
