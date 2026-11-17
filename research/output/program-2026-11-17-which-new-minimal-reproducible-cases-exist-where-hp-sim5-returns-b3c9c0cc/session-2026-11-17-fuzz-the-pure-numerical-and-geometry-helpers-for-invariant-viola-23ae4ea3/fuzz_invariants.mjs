// Deterministic invariant fuzz for hp-sim5's pure numerical / geometry helpers.
//
// Usage:  node fuzz_invariants.mjs
// Exits 0 when every checked invariant holds within its stated tolerance,
// exits 1 and prints the failing checks otherwise.
//
// Everything is seeded (mulberry32), so every run is byte-identical.
import Quaternion from '../../../../src/js/cable_joints_3d/quaternion.js';
import Vector3 from '../../../../src/js/cable_joints_3d/vector3.js';
import Vector2, {
  _tangentPointCircle,
  tangentFromCircleToCircle,
  signedArcLengthOnWheel as arc2D,
  rightOfLine,
} from '../../../../src/js/cable_joints_3d/planar_math.js';
import * as geo from '../../../../src/js/cable_joints_3d/geometry3.js';
import * as it from '../../../../src/js/cable_joints_3d/inertia_tensor.js';
import { writeSlackCablePositions } from '../../../../src/js/cable_joints_3d/cable_shape.js';
import {
  decomposeSpoolOrientation,
  composeSpoolOrientation,
  constrainSpoolOrientation,
  getSpoolWorldAxis,
  constrainSpoolAngularVelocity,
  normalizeSpoolAxisLocal,
} from '../../../../hp-sim-3d/app/hangprinter_spools.js';

// ---------------------------------------------------------------- utilities
function mulberry32(seed) {
  return function next() {
    seed |= 0;
    seed = (seed + 0x6D2B79F5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const rng = mulberry32(0x5eed1234);
const U = (lo, hi) => lo + (hi - lo) * rng();
const qrand = () => new Quaternion(U(-1, 1), U(-1, 1), U(-1, 1), U(-1, 1)).normalize();
const vrand = (s = 1) => new Vector3(U(-s, s), U(-s, s), U(-s, s));

const checks = [];
function check(name, worst, tol, note = '') {
  checks.push({ name, worst, tol, ok: worst <= tol, note });
  const flag = worst <= tol ? 'pass' : 'FAIL';
  console.log(`${flag}  ${name.padEnd(52)} worst=${worst.toExponential(3)} tol=${tol.toExponential(1)}${note ? `  (${note})` : ''}`);
}
function quatDistance(a, b) {
  return Math.min(
    Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w),
    Math.hypot(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w),
  );
}
// symmetric 3x3 eigenvalues (independent Jacobi, used only for sign tests)
function eigenSym3(m) {
  const a = m.map(row => row.slice());
  for (let sweep = 0; sweep < 100; sweep += 1) {
    let off = 0;
    for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) off += a[p][q] * a[p][q];
    if (off < 1e-30) break;
    for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
      if (Math.abs(a[p][q]) < 1e-300) continue;
      const theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
      const t = (theta >= 0 ? 1 : -1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
      const c = 1 / Math.sqrt(t * t + 1);
      const s = t * c;
      for (let k = 0; k < 3; k += 1) {
        const akp = a[k][p];
        const akq = a[k][q];
        a[k][p] = c * akp - s * akq;
        a[k][q] = s * akp + c * akq;
      }
      for (let k = 0; k < 3; k += 1) {
        const apk = a[p][k];
        const aqk = a[q][k];
        a[p][k] = c * apk - s * aqk;
        a[q][k] = s * apk + c * aqk;
      }
    }
  }
  return [a[0][0], a[1][1], a[2][2]].sort((x, y) => x - y);
}

// ------------------------------------------------------------- quaternions
{
  let worst = 0;
  for (let i = 0; i < 4000; i += 1) {
    const p = qrand().multiplyQuaternions(qrand(), qrand());
    worst = Math.max(worst, Math.abs(p.lengthSq() - 1));
  }
  check('quaternion: |q1*q2|^2 = 1 for unit inputs', worst, 1e-14);
}
{
  let worst = 0;
  for (let i = 0; i < 4000; i += 1) {
    const q = qrand();
    const id = q.clone().multiply(q.clone().conjugate());
    worst = Math.max(worst, Math.abs(Math.abs(id.w) - 1), Math.hypot(id.x, id.y, id.z));
  }
  check('quaternion: q * q^-1 = identity', worst, 1e-15);
}
{
  let worstBack = 0;
  let worstNorm = 0;
  for (let i = 0; i < 6000; i += 1) {
    const q = qrand();
    const v = vrand(10);
    const r = q.transformVector(v);
    worstBack = Math.max(worstBack, q.clone().conjugate().transformVector(r).distanceTo(v));
    worstNorm = Math.max(worstNorm, Math.abs(r.length() - v.length()));
  }
  check('quaternion: rotate then unrotate recovers v', worstBack, 1e-13);
  check('quaternion: rotation preserves |v|', worstNorm, 1e-13);
}
{
  // (a*b) rotating v must equal a rotating (b rotating v) for column vectors
  let worst = 0;
  for (let i = 0; i < 4000; i += 1) {
    const a = qrand();
    const b = qrand();
    const v = vrand(3);
    const composed = a.clone().multiply(b).transformVector(v);
    const sequential = a.transformVector(b.transformVector(v));
    worst = Math.max(worst, composed.distanceTo(sequential));
  }
  check('quaternion: composition order matches sequential rotate', worst, 1e-14);
}
{
  let worst = 0;
  for (let i = 0; i < 2000; i += 1) {
    const q = qrand();
    const v = vrand(4);
    const p = new Quaternion(v.x, v.y, v.z, 0);
    const ref = q.clone().multiply(p).multiply(q.clone().conjugate());
    worst = Math.max(worst, new Vector3(ref.x, ref.y, ref.z).distanceTo(q.transformVector(v)));
  }
  check('quaternion: transformVector = q (0,v) q^-1', worst, 1e-14);
}
{
  let worst = 0;
  for (let i = 0; i < 2000; i += 1) {
    const q = qrand();
    const v = vrand();
    const expected = q.transformVector(v.clone());
    const alias = v.clone();
    q.transformVectorInto(alias, alias);
    worst = Math.max(worst, alias.distanceTo(expected));
  }
  check('quaternion: transformVectorInto aliasing (v == result)', worst, 1e-15);
}
{
  let worst = 0;
  for (let i = 0; i < 2000; i += 1) {
    const q = new Quaternion(U(-1e-6, 1e-6), U(-1e-6, 1e-6), U(-1e-6, 1e-6), U(-1e-6, 1e-6)).normalize();
    worst = Math.max(worst, Math.abs(q.lengthSq() - 1));
  }
  check('quaternion: normalize then |q|^2 = 1', worst, 1e-14);
}

// ---------------------------------------------------------------- vector3
{
  let worst = 0;
  for (let i = 0; i < 4000; i += 1) {
    const a = vrand();
    const b = vrand();
    const c = a.clone().cross(b);
    worst = Math.max(worst, Math.abs(c.dot(a)), Math.abs(c.dot(b)));
  }
  check('vector3: cross is perpendicular to both inputs', worst, 1e-14);
}
{
  let worst = 0;
  for (let i = 0; i < 3000; i += 1) {
    const axis = vrand();
    const angle = U(-Math.PI, Math.PI);
    const v = vrand();
    const viaAxis = v.clone().rotateAroundAxis(axis, angle);
    const q = new Quaternion().setFromAxisAngle(axis.clone().normalize(), angle);
    worst = Math.max(worst, viaAxis.distanceTo(q.transformVector(v)));
  }
  check('vector3: rotateAroundAxis = quaternion rotation', worst, 1e-13);
}
{
  let worst = 0;
  for (let i = 0; i < 3000; i += 1) {
    const a = vrand();
    const b = vrand();
    const ref = Math.atan2(a.clone().cross(b).length(), a.dot(b));
    worst = Math.max(worst, Math.abs(a.angleTo(b) - ref));
  }
  check('vector3: angleTo = atan2(|axb|, a.b)', worst, 1e-8);
}

// ------------------------------------------------------------- planar math
{
  let worstTangency = 0;
  let worstRadius = 0;
  let worstDirection = 0;
  for (let i = 0; i < 4000; i += 1) {
    const c = new Vector2(U(-5, 5), U(-5, 5));
    const r = U(0.01, 4);
    const theta = U(-Math.PI, Math.PI);
    const d = r * U(1.001, 40);
    const p = new Vector2(c.x + d * Math.cos(theta), c.y + d * Math.sin(theta));
    for (const cw of [false, true]) {
      for (const first of [false, true]) {
        const { a_circle: t } = _tangentPointCircle(p, c, r, cw, first);
        const radial = new Vector2(t.x - c.x, t.y - c.y);
        worstRadius = Math.max(worstRadius, Math.abs(radial.length() - r) / r);
        worstTangency = Math.max(worstTangency, Math.abs(radial.dot(new Vector2(t.x - p.x, t.y - p.y))) / (r * r));
        const wrap = cw ? new Vector2(radial.y, -radial.x) : new Vector2(-radial.y, radial.x);
        wrap.scale(1 / r);
        const travel = first ? new Vector2(t.x - p.x, t.y - p.y) : new Vector2(p.x - t.x, p.y - t.y);
        travel.scale(1 / travel.length());
        worstDirection = Math.max(worstDirection, travel.clone().subtract(wrap).length());
      }
    }
  }
  check('planar: point-circle tangent point on circle', worstRadius, 1e-12);
  check('planar: point-circle tangent is perpendicular to radius', worstTangency, 1e-12);
  check('planar: point-circle travel = wrap direction', worstDirection, 1e-12);
}
{
  let worstPerp = 0;
  let worstRadius = 0;
  let count = 0;
  for (let i = 0; i < 3000; i += 1) {
    const a = new Vector2(U(-5, 5), U(-5, 5));
    const b = new Vector2(U(-5, 5), U(-5, 5));
    const ra = U(0.05, 2);
    const rb = U(0.05, 2);
    if (a.distanceTo(b) < ra + rb + 0.01) continue;
    for (const cwA of [false, true]) {
      for (const cwB of [false, true]) {
        const { a_circle: A, b_circle: B } = tangentFromCircleToCircle(a, ra, cwA, b, rb, cwB);
        const u = new Vector2(B.x - A.x, B.y - A.y);
        u.scale(1 / u.length());
        const vA = cwA ? new Vector2(A.y - a.y, -(A.x - a.x)) : new Vector2(-(A.y - a.y), A.x - a.x);
        const vB = cwB ? new Vector2(B.y - b.y, -(B.x - b.x)) : new Vector2(-(B.y - b.y), B.x - b.x);
        vA.scale(1 / ra);
        vB.scale(1 / rb);
        worstPerp = Math.max(worstPerp, u.clone().subtract(vA).length(), u.clone().subtract(vB).length());
        worstRadius = Math.max(worstRadius, Math.abs(A.distanceTo(a) - ra) / ra, Math.abs(B.distanceTo(b) - rb) / rb);
        count += 1;
      }
    }
  }
  check(`planar: circle-circle common tangent (${count} cases)`, worstPerp, 1e-12);
  check('planar: circle-circle tangent points on circles', worstRadius, 1e-14);
}
{
  let worst = 0;
  let worstSign = 0;
  let worstComplement = 0;
  for (let i = 0; i < 4000; i += 1) {
    const center = new Vector2(U(-2, 2), U(-2, 2));
    const r = U(0.1, 3);
    const a1 = U(-Math.PI, Math.PI);
    const a2 = U(-Math.PI, Math.PI);
    const p = new Vector2(center.x + r * Math.cos(a1), center.y + r * Math.sin(a1));
    const q = new Vector2(center.x + r * Math.cos(a2), center.y + r * Math.sin(a2));
    const forward = arc2D(p, q, center, r, false, false);
    const backward = arc2D(q, p, center, r, false, false);
    worst = Math.max(worst, Math.abs(forward + backward), Math.abs(forward) - Math.PI * r);
    // reference: principal-value wrapped angle, then arc length
    let delta = a2 - a1;
    while (delta > Math.PI) delta -= 2 * Math.PI;
    while (delta < -Math.PI) delta += 2 * Math.PI;
    worst = Math.max(worst, Math.abs(forward - r * delta));
    const cwPositive = arc2D(p, q, center, 1, true, true);
    const ccwPositive = arc2D(p, q, center, 1, false, true);
    worstSign = Math.max(worstSign, cwPositive < 0 ? -cwPositive : 0, ccwPositive < 0 ? -ccwPositive : 0);
    worstComplement = Math.max(worstComplement, Math.abs(cwPositive + ccwPositive - 2 * Math.PI));
  }
  check('planar: signed arc matches wrapped atan2 difference', worst, 1e-13);
  check('planar: force_positive arc >= 0', worstSign, 1e-15);
  check('planar: cw + ccw arcs sum to 2*pi*r', worstComplement, 1e-13);
}
{
  // rightOfLine / rightOfPlane sign conventions
  let worstMismatch = 0;
  for (let i = 0; i < 4000; i += 1) {
    const p0 = new Vector2(U(-3, 3), U(-3, 3));
    const p1 = new Vector2(U(-3, 3), U(-3, 3));
    const x = new Vector2(U(-3, 3), U(-3, 3));
    const cross = (p1.x - p0.x) * (x.y - p0.y) - (p1.y - p0.y) * (x.x - p0.x);
    const got = rightOfLine(x, p0, p1);
    if (got !== (cross < 0) && Math.abs(cross) > 1e-9) worstMismatch += 1;
  }
  check('planar: rightOfLine = (2D cross < 0)', worstMismatch, 0);
  let mismatch3 = 0;
  for (let i = 0; i < 4000; i += 1) {
    const n = vrand();
    if (n.length() < 0.3) continue;
    const p0 = vrand(3);
    const p1 = vrand(3);
    const x = vrand(3);
    const triple = n.clone().cross(p1.clone().subtract(p0)).dot(x.clone().subtract(p0));
    if (geo.rightOfPlane(x, p0, p1, n) !== (triple < 0) && Math.abs(triple) > 1e-9) mismatch3 += 1;
  }
  check('planar: rightOfPlane = (n.(p1-p0)x(x-p0) < 0)', mismatch3, 0);
}

// --------------------------------------------------------------- geometry3
{
  let worst = 0;
  for (let i = 0; i < 3000; i += 1) {
    const a = vrand(5);
    const b = vrand(5);
    const p = vrand(5);
    const got = geo.closestPointOnSegment(p, a, b).distanceTo(p);
    // closed form
    const ab = b.clone().subtract(a);
    const abLenSq = ab.lengthSq();
    let t = abLenSq > 0 ? p.clone().subtract(a).dot(ab) / abLenSq : 0;
    t = Math.max(0, Math.min(1, t));
    const reference = a.clone().add(ab, t).distanceTo(p);
    worst = Math.max(worst, Math.abs(got - reference));
  }
  check('geometry3: closestPointOnSegment = closed form', worst, 1e-13);
}
{
  let gross = 0;
  for (let i = 0; i < 20000; i += 1) {
    const p1 = vrand(2);
    const p2 = vrand(2);
    const c = vrand(2);
    const r = U(0.01, 1);
    const got = geo.lineSegmentSphereIntersection(p1, p2, c, r);
    const ab = p2.clone().subtract(p1);
    const denom = ab.lengthSq();
    const t = Math.max(0, Math.min(1, denom > 0 ? c.clone().subtract(p1).dot(ab) / denom : 0));
    const closest = p1.clone().add(ab, t).distanceTo(c);
    const outside = p1.distanceTo(c) > r && p2.distanceTo(c) > r;
    const reference = outside ? closest <= r : false;
    if (got !== reference && Math.abs(closest - r) / r > 1e-9) gross += 1;
  }
  check('geometry3: lineSegmentSphereIntersection = closed form', gross, 0);
}
{
  let worstRadius = 0;
  let worstPlane = 0;
  let worstDirection = 0;
  let degenerateSkipped = 0;
  for (let i = 0; i < 4000; i += 1) {
    const normal = vrand();
    if (normal.length() < 0.3) continue;
    const n = normal.clone().normalize();
    const c = vrand(3);
    const p = vrand(3);
    const r = U(0.05, 2);
    // skip inputs where the in-plane projection of p is inside the guide:
    // no tangent exists there and _tangentPointCircle takes its fallback branch
    const pIn = p.clone().subtract(c).subtract(n, p.clone().subtract(c).dot(n));
    if (pIn.length() < r * 1.001) { degenerateSkipped += 1; continue; }
    for (const cw of [false, true]) {
      for (const first of [true, false]) {
        const fn = first ? geo.tangentFromPointToSphere : geo.tangentFromSphereToPoint;
        const t = fn(p, c, r, n, cw).a_sphere;
        const radial = t.clone().subtract(c);
        worstRadius = Math.max(worstRadius, Math.abs(radial.length() - r) / r);
        worstPlane = Math.max(worstPlane, Math.abs(radial.dot(n)) / r);
        const wrap = cw
          ? n.clone().cross(radial).scale(-1 / r)
          : n.clone().cross(radial).scale(1 / r);
        const travel = first ? t.clone().subtract(p) : p.clone().subtract(t);
        travel.subtract(n, travel.dot(n)).normalize();
        worstDirection = Math.max(worstDirection, travel.clone().subtract(wrap).length());
      }
    }
  }
  check('geometry3: point-sphere tangent point radius', worstRadius, 1e-13);
  check('geometry3: point-sphere tangent point in wrap plane', worstPlane, 1e-13);
  check(`geometry3: point-sphere tangent direction (skipped ${degenerateSkipped})`, worstDirection, 1e-12);
}
{
  let worstA = 0;
  let worstB = 0;
  let worstDirection = 0;
  for (let i = 0; i < 3000; i += 1) {
    const normal = vrand();
    if (normal.length() < 0.3) continue;
    const n = normal.clone().normalize();
    const posA = vrand(3);
    const posB = vrand(3);
    const ra = U(0.05, 1.5);
    const rb = U(0.05, 1.5);
    const delta = posB.clone().subtract(posA);
    const inPlane = delta.clone().subtract(n, delta.dot(n));
    if (inPlane.length() < ra + rb + 0.01) continue;
    for (const cwA of [false, true]) {
      for (const cwB of [false, true]) {
        const { a_sphere: A, b_sphere: B } = geo.tangentFromSphereToSphere(posA, ra, cwA, posB, rb, cwB, n);
        worstA = Math.max(worstA, Math.abs(A.distanceTo(posA) - ra) / ra);
        worstB = Math.max(worstB, Math.abs(B.distanceTo(posB) - rb) / rb);
        const u = B.clone().subtract(A);
        u.subtract(n, u.dot(n)).normalize();
        const vA = cwA
          ? n.clone().cross(A.clone().subtract(posA)).scale(-1 / ra)
          : n.clone().cross(A.clone().subtract(posA)).scale(1 / ra);
        const vB = cwB
          ? n.clone().cross(B.clone().subtract(posB)).scale(-1 / rb)
          : n.clone().cross(B.clone().subtract(posB)).scale(1 / rb);
        worstDirection = Math.max(worstDirection, u.clone().subtract(vA).length(), u.clone().subtract(vB).length());
      }
    }
  }
  check('geometry3: sphere-sphere tangent point on sphere A', worstA, 1e-13);
  check('geometry3: sphere-sphere tangent point on sphere B', worstB, 1e-13);
  check('geometry3: sphere-sphere span is tangent at both ends', worstDirection, 1e-12);
}
{
  let worst = 0;
  for (let i = 0; i < 3000; i += 1) {
    const normal = vrand();
    if (normal.length() < 0.2) continue;
    const n = normal.clone().normalize();
    const center = vrand(2);
    const radius = U(0.1, 2);
    const a1 = U(-Math.PI, Math.PI);
    const a2 = U(-Math.PI, Math.PI);
    const u = new Vector3(1, 0, 0).subtract(n, n.x).normalize();
    const v = n.clone().cross(u);
    const P = center.clone().add(u, radius * Math.cos(a1)).add(v, radius * Math.sin(a1));
    const Q = center.clone().add(u, radius * Math.cos(a2)).add(v, radius * Math.sin(a2));
    const got = geo.signedArcLengthOnWheel(P, Q, center, radius, false, n, false);
    let delta = a2 - a1;
    while (delta > Math.PI) delta -= 2 * Math.PI;
    while (delta < -Math.PI) delta += 2 * Math.PI;
    worst = Math.max(worst, Math.abs(got - radius * delta));
  }
  check('geometry3: 3D arc length = projected 2D arc', worst, 1e-10);
}

// ---------------------------------------------------------------- inertia
{
  let worstOrtho = 0;
  let worstDet = 0;
  for (let i = 0; i < 4000; i += 1) {
    const R = it.rotationMatrixFromQuaternion(qrand());
    for (let r = 0; r < 3; r += 1) {
      for (let c = 0; c < 3; c += 1) {
        const dot = R[0][r] * R[0][c] + R[1][r] * R[1][c] + R[2][r] * R[2][c];
        worstOrtho = Math.max(worstOrtho, Math.abs(dot - (r === c ? 1 : 0)));
      }
    }
    const det = R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
      - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
      + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);
    worstDet = Math.max(worstDet, Math.abs(det - 1));
  }
  check('inertia: R^T R = I for quaternion rotation', worstOrtho, 1e-14);
  check('inertia: det(R) = +1', worstDet, 1e-14);
}
{
  let worstSym = 0;
  let worstPd = 0;
  let worstInv = 0;
  for (let i = 0; i < 3000; i += 1) {
    const diag = [[U(0.1, 5), 0, 0], [0, U(0.1, 5), 0], [0, 0, U(0.1, 5)]];
    const W = it.transformInertiaTensorToWorld(diag, qrand());
    for (let r = 0; r < 3; r += 1) {
      for (let c = 0; c < 3; c += 1) worstSym = Math.max(worstSym, Math.abs(W[r][c] - W[c][r]));
    }
    const eig = eigenSym3(W);
    worstPd = Math.max(worstPd, -eig[0] / eig[2]);
    const prod = it.multiplyMatrix3(it.invertMatrix3(W), W);
    for (let r = 0; r < 3; r += 1) {
      for (let c = 0; c < 3; c += 1) worstInv = Math.max(worstInv, Math.abs(prod[r][c] - (r === c ? 1 : 0)));
    }
  }
  check('inertia: world tensor symmetric', worstSym, 1e-13);
  check('inertia: world tensor positive definite', worstPd, 1e-14);
  check('inertia: invertMatrix3(W) * W = I', worstInv, 1e-11);
}
{
  // physical consistency for translated bodies: composite of point masses
  let worst = 0;
  let worstTriangle = 0;
  for (let i = 0; i < 3000; i += 1) {
    const nPoints = 2 + Math.floor(rng() * 7);
    const points = [];
    for (let k = 0; k < nPoints; k += 1) points.push({ m: U(0.05, 3), p: vrand(1.5) });
    let mass = 0;
    const com = new Vector3();
    for (const { m, p } of points) { mass += m; com.add(p, m); }
    com.scale(1 / mass);
    const composite = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    const direct = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (const { m, p } of points) {
      const o = p.clone().subtract(com);
      // parallelAxisTensor + (zero member tensor) must equal the direct sum
      const pat = it.parallelAxisTensor(m, o);
      for (let r = 0; r < 3; r += 1) for (let c = 0; c < 3; c += 1) composite[r][c] += pat[r][c];
      direct[0][0] += m * (o.y ** 2 + o.z ** 2);
      direct[1][1] += m * (o.x ** 2 + o.z ** 2);
      direct[2][2] += m * (o.x ** 2 + o.y ** 2);
      direct[0][1] -= m * o.x * o.y;
      direct[1][0] -= m * o.x * o.y;
      direct[0][2] -= m * o.x * o.z;
      direct[2][0] -= m * o.x * o.z;
      direct[1][2] -= m * o.y * o.z;
      direct[2][1] -= m * o.y * o.z;
    }
    for (let r = 0; r < 3; r += 1) {
      for (let c = 0; c < 3; c += 1) worst = Math.max(worst, Math.abs(composite[r][c] - direct[r][c]));
    }
    // triangle inequality on principal moments, and symmetry of the aggregate
    const eig = eigenSym3(composite);
    worstTriangle = Math.max(worstTriangle, Math.max(0, eig[2] - eig[0] - eig[1]) / Math.max(eig[2], 1e-30));
    for (let r = 0; r < 3; r += 1) {
      for (let c = 0; c < 3; c += 1) worstTriangle = Math.max(worstTriangle, Math.abs(composite[r][c] - composite[c][r]));
    }
  }
  check('inertia: parallel axis = direct point-mass sum', worst, 1e-14);
  check('inertia: triangle inequality on principal moments', worstTriangle, 1e-14);
}
{
  let worstAxis = 0;
  let worstApply = 0;
  let worstQuad = 0;
  for (let i = 0; i < 3000; i += 1) {
    const d = [U(0.1, 5), U(0.1, 5), U(0.1, 5)];
    const tensor = [[d[0], 0, 0], [0, d[1], 0], [0, 0, d[2]]];
    const moi = new it.MomentOfInertiaComponent(tensor);
    const q = qrand();
    const axis = vrand().normalize();
    const world = it.transformInertiaTensorToWorld(tensor, q);
    const expectedAxis = axis.dot(it.multiplyMatrix3Vector(world, axis));
    worstAxis = Math.max(worstAxis, Math.abs(it.effectiveInertiaAboutWorldAxis(moi, q, axis) - expectedAxis) / expectedAxis);
    const v = vrand(2);
    const R = it.rotationMatrixFromQuaternion(q);
    const expectedApply = it.multiplyMatrix3Vector(
      it.multiplyMatrix3(it.multiplyMatrix3(R, it.invertMatrix3(tensor)), it.transposeMatrix3(R)),
      v,
    );
    worstApply = Math.max(worstApply, it.applyWorldInverseInertia(moi, q, v).distanceTo(expectedApply) / expectedApply.length());
    const local = it.multiplyMatrix3Vector(it.transposeMatrix3(R), v);
    const expectedQuad = local.x ** 2 / d[0] + local.y ** 2 / d[1] + local.z ** 2 / d[2];
    worstQuad = Math.max(worstQuad, Math.abs(it.inverseInertiaQuadraticForm(moi, q, v) - expectedQuad) / expectedQuad);
  }
  check('inertia: effectiveInertiaAboutWorldAxis = u^T R I R^T u', worstAxis, 1e-12);
  check('inertia: applyWorldInverseInertia = R I^-1 R^T v', worstApply, 1e-11);
  check('inertia: inverseInertiaQuadraticForm = v^T I^-1 v', worstQuad, 1e-12);
}
{
  // exactly singular (rod) tensors: pseudo-inverse identities must still hold
  let worst = 0;
  let worstMaxInv = 0;
  for (let i = 0; i < 3000; i += 1) {
    const W = it.transformInertiaTensorToWorld([[1, 0, 0], [0, 1, 0], [0, 0, 0]], qrand());
    const P = it.invertMatrix3(W);
    const MPM = it.multiplyMatrix3(it.multiplyMatrix3(W, P), W);
    for (let r = 0; r < 3; r += 1) {
      for (let c = 0; c < 3; c += 1) worst = Math.max(worst, Math.abs(MPM[r][c] - W[r][c]));
    }
    worstMaxInv = Math.max(worstMaxInv, Math.max(...P.flat().map(Math.abs)));
  }
  check('inertia: singular pseudo-inverse W P W = W', worst, 1e-13, `max|P|=${worstMaxInv.toFixed(2)}`);
}
{
  let worst = 0;
  for (const [stored, expected] of [
    [[1, 2, 3, 4, 5, 6, 7, 8, 9], [[1, 2, 3], [4, 5, 6], [7, 8, 9]]],
    [[[1, 2, 3], [4, 5, 6], [7, 8, 9]], [[1, 2, 3], [4, 5, 6], [7, 8, 9]]],
  ]) {
    const got = it.normalizeInertiaTensor(stored);
    for (let r = 0; r < 3; r += 1) for (let c = 0; c < 3; c += 1) worst = Math.max(worst, Math.abs(got[r][c] - expected[r][c]));
  }
  check('inertia: normalizeInertiaTensor row-major layouts', worst, 0);
}
{
  // zero/negative mass, zero offset, NaN inputs must not produce NaN
  let bad = 0;
  const z = it.parallelAxisTensor(0, new Vector3(1, 2, 3));
  const neg = it.parallelAxisTensor(-1, new Vector3(1, 2, 3));
  for (const M of [z, neg, it.normalizeInertiaTensor(NaN), it.invertMatrix3([[0, 0, 0], [0, 0, 0], [0, 0, 0]])]) {
    if (M.flat().some(v => !Number.isFinite(v))) bad += 1;
    if (M.flat().some(v => v !== 0)) bad += 1;
  }
  check('inertia: zero mass / zero tensor stay zero and finite', bad, 0);
}

// ------------------------------------------------------------- cable shape
function polylineLength(pos, segments) {
  let L = 0;
  for (let i = 0; i < segments; i += 1) {
    L += Math.hypot(
      pos[3 * i + 3] - pos[3 * i],
      pos[3 * i + 4] - pos[3 * i + 1],
      pos[3 * i + 5] - pos[3 * i + 2],
    );
  }
  return L;
}
{
  let worstEnd = 0;
  let worstLen = 0;
  let nan = 0;
  let worstStraight = 0;
  for (let i = 0; i < 600; i += 1) {
    const start = vrand(3);
    const end = vrand(3);
    const straight = start.distanceTo(end);
    if (straight < 1e-3) continue; // avoid the documented near-vertical fallback
    const L = straight * U(1.001, 2.5);
    const up = vrand().normalize();
    const segments = 256;
    const pos = new Float64Array((segments + 1) * 3);
    writeSlackCablePositions(pos, start, end, L, up, segments);
    if (pos.some(v => !Number.isFinite(v))) nan += 1;
    worstEnd = Math.max(
      worstEnd,
      Math.hypot(pos[0] - start.x, pos[1] - start.y, pos[2] - start.z),
      Math.hypot(pos[3 * segments] - end.x, pos[3 * segments + 1] - end.y, pos[3 * segments + 2] - end.z),
    );
    // chord approximation error is O((L/h)^2); subdivide finely
    worstLen = Math.max(worstLen, Math.abs(polylineLength(pos, segments) - L) / L);
    // taut case stays straight
    const taut = new Float64Array((segments + 1) * 3);
    writeSlackCablePositions(taut, start, end, straight * 0.999, up, segments);
    let maxOff = 0;
    for (let k = 0; k <= segments; k += 1) {
      const t = k / segments;
      maxOff = Math.max(maxOff, Math.hypot(
        taut[3 * k] - (start.x + (end.x - start.x) * t),
        taut[3 * k + 1] - (start.y + (end.y - start.y) * t),
        taut[3 * k + 2] - (start.z + (end.z - start.z) * t),
      ));
    }
    worstStraight = Math.max(worstStraight, maxOff / straight);
  }
  check('cable_shape: endpoints are exact', worstEnd, 1e-15);
  check('cable_shape: polyline length = rest length', worstLen, 2e-3, '256 segments, chord error');
  check('cable_shape: taut cable is the straight chord', worstStraight, 1e-15);
  check('cable_shape: no NaN/Inf', nan, 0);
}

// ------------------------------------------------------ spool frame algebra
{
  let worst = 0;
  let worstAngle = 0;
  let worstTwist = 0;
  for (let i = 0; i < 3000; i += 1) {
    const spool = { axisLocal: vrand().normalize(), referenceOrientation: qrand() };
    const q = qrand();
    const { swing, angle } = decomposeSpoolOrientation(spool, q);
    worst = Math.max(worst, quatDistance(composeSpoolOrientation(spool, swing, angle), q));
    worstAngle = Math.max(worstAngle, Math.abs(decomposeSpoolOrientation(spool, q).angle - angle));
    const constrained = constrainSpoolOrientation(spool, q);
    const relative = spool.referenceOrientation.clone().conjugate().normalize().multiply(constrained);
    const axis = normalizeSpoolAxisLocal(spool.axisLocal);
    const vectorPart = new Vector3(relative.x, relative.y, relative.z);
    worstTwist = Math.max(worstTwist, vectorPart.clone().subtract(axis.clone().scale(vectorPart.dot(axis))).length());
  }
  check('spool: swing/twist recompose recovers the orientation', worst, 1e-13);
  check('spool: decomposed angle is stable', worstAngle, 1e-12);
  check('spool: constrained orientation is a pure twist', worstTwist, 1e-13);
}
{
  let worst = 0;
  for (let i = 0; i < 2000; i += 1) {
    const spool = { axisLocal: vrand().normalize(), referenceOrientation: qrand() };
    const q = qrand();
    const omega = vrand(5);
    const axis = getSpoolWorldAxis(spool, q);
    const projected = constrainSpoolAngularVelocity(spool, q, omega);
    worst = Math.max(worst, Math.abs(projected.dot(axis) - omega.dot(axis)), projected.clone().cross(axis).length());
  }
  check('spool: angular velocity projected onto the world axis', worst, 1e-13);
}

// ------------------------------------------------------------ final report
const failed = checks.filter(c => !c.ok);
console.log(`\n${checks.length - failed.length}/${checks.length} invariant checks passed.`);
if (failed.length > 0) {
  console.error('FAILED:');
  for (const f of failed) console.error(`  - ${f.name}: worst ${f.worst.toExponential(3)} > tol ${f.tol.toExponential(1)}`);
  process.exit(1);
}
