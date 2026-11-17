// Probe of the known *documented / graceful-degradation* boundaries of the
// helpers. These are not presented as defects; this script measures how far
// each one can be pushed, so the report can state the limits precisely.
//
// Usage: node probe_limitations.mjs   (always exits 0)
import Vector3 from '../../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../../src/js/cable_joints_3d/quaternion.js';
import Vector2, { _tangentPointCircle, signedArcLengthOnWheel } from '../../../../src/js/cable_joints_3d/planar_math.js';
import * as geo from '../../../../src/js/cable_joints_3d/geometry3.js';
import * as it from '../../../../src/js/cable_joints_3d/inertia_tensor.js';
import { writeSlackCablePositions } from '../../../../src/js/cable_joints_3d/cable_shape.js';

function mulberry32(seed) {
  return function next() {
    seed |= 0;
    seed = (seed + 0x6D2B79F5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const rng = mulberry32(20261117);
const U = (lo, hi) => lo + (hi - lo) * rng();

console.log('--- 1. _tangentPointCircle fallback when the attach point is inside the circle ---');
console.log('    (no tangent exists; the code returns the radial nearest point on the circle)');
{
  const c = new Vector2(0, 0);
  const r = 1.0;
  for (const d of [0.999, 0.9, 0.5, 0.0]) {
    const p = new Vector2(d, 0);
    const { a_circle: t } = _tangentPointCircle(p, c, r, true, true);
    const radial = new Vector2(t.x - c.x, t.y - c.y);
    const seg = new Vector2(t.x - p.x, t.y - p.y);
    const cosWithRadius = Math.abs(radial.clone().scale(1 / radial.length()).dot(seg.clone().scale(1 / seg.length())));
    console.log(`    |p|=${d}: tangent point=(${t.x.toFixed(6)}, ${t.y.toFixed(6)}), |T-C|=${radial.length().toFixed(6)}, ` +
      `|cos(radius, span)|=${cosWithRadius.toFixed(6)} (0 = tangent, 1 = radial)`);
  }
}
console.log('    3D variant: a point outside the sphere whose in-plane projection is inside the circle');
{
  const n = new Vector3(0, 0, 1);
  const c = new Vector3(0, 0, 0);
  const p = new Vector3(0.5, 0, 3.0); // 3.04 m from the centre, but projects to 0.5 m
  const out = geo.tangentFromPointToSphere(p, c, 1.0, n, true);
  const radial = out.a_sphere.clone().subtract(c);
  const seg = out.a_sphere.clone().subtract(p);
  seg.subtract(n, seg.dot(n));
  console.log(`    |p-C|=${p.distanceTo(c).toFixed(3)} (> r) but |proj(p-C)|=${Math.hypot(p.x, p.y).toFixed(3)} (< r); ` +
    `tangent point=(${out.a_sphere.x.toFixed(6)}, ${out.a_sphere.y.toFixed(6)}), |cos(radius, span)|=${Math.abs(radial.clone().normalize().dot(seg.clone().normalize())).toFixed(6)}`);
}

console.log('\n--- 2. symmetric pseudo-inverse near the singularity threshold ---');
console.log('    (M P M = M degrades in proportion to the smallest/largest eigenvalue ratio)');
{
  for (const exp of [-18, -16, -15, -14, -13, -12, -10, -8, -6]) {
    const eps = 10 ** exp;
    let worst = 0;
    let worstMaxInv = 0;
    for (let i = 0; i < 400; i += 1) {
      const q = new Quaternion(U(-1, 1), U(-1, 1), U(-1, 1), U(-1, 1)).normalize();
      const M = it.transformInertiaTensorToWorld([[1, 0, 0], [0, 1, 0], [0, 0, eps]], q);
      const P = it.invertMatrix3(M);
      const MPM = it.multiplyMatrix3(it.multiplyMatrix3(M, P), M);
      const scale = Math.max(...M.flat().map(Math.abs));
      for (let r = 0; r < 3; r += 1) {
        for (let cc = 0; cc < 3; cc += 1) worst = Math.max(worst, Math.abs(MPM[r][cc] - M[r][cc]) / scale);
      }
      worstMaxInv = Math.max(worstMaxInv, Math.max(...P.flat().map(Math.abs)));
    }
    console.log(`    I_min/I_max=1e${String(exp).padStart(3)}: worst rel |MPM-M|=${worst.toExponential(2)}  max|P|=${worstMaxInv.toExponential(2)}`);
  }
}

console.log('\n--- 3. Vector3.angleTo resolution (acos conditioning) ---');
for (const eps of [1e-4, 1e-8, 1e-10, 1e-14]) {
  const got = new Vector3(1, 0, 0).angleTo(new Vector3(1, eps, 0));
  console.log(`    angle between (1,0,0) and (1,${eps},0): angleTo=${got.toExponential(6)} atan2 ref=${Math.atan2(eps, 1).toExponential(6)}`);
}

console.log('\n--- 4. writeSlackCablePositions: parabolic fallback for near-vertical spans ---');
console.log('    (fallback is used when horizontal span <= 1e-5 m; it sags along "up", so a');
console.log('     vertical slack cable renders as a straight line and the slack is lost)');
{
  const segments = 64;
  const up = new Vector3(0, 0, 1);
  const rows = [
    { h: 0.0, v: 1.0, L: 1.5 },
    { h: 1e-7, v: 1.0, L: 1.5 },
    { h: 1e-6, v: 1.0, L: 1.5 },
    { h: 1e-5, v: 1.0, L: 1.5 },
    { h: 1e-4, v: 1.0, L: 1.5 },
    { h: 1e-3, v: 1.0, L: 1.5 },
    { h: 0.02, v: 0.0, L: 0.021 },
  ];
  for (const { h, v, L } of rows) {
    const start = new Vector3(0, 0, 0);
    const end = new Vector3(h, 0, v);
    const straight = start.distanceTo(end);
    const pos = new Float64Array((segments + 1) * 3);
    writeSlackCablePositions(pos, start, end, L, up, segments);
    let pl = 0;
    for (let i = 0; i < segments; i += 1) {
      pl += Math.hypot(pos[3 * i + 3] - pos[3 * i], pos[3 * i + 4] - pos[3 * i + 1], pos[3 * i + 5] - pos[3 * i + 2]);
    }
    console.log(`    h=${h.toExponential(0)} v=${v} straight=${straight.toFixed(4)} L=${L.toFixed(4)} -> polyline=${pl.toFixed(6)} (${((pl / L - 1) * 100).toFixed(1)}% vs L)`);
  }
  // realistic near-taut span: catenary branch, arc length error is chord-only
  const start = new Vector3(0, 0, 0);
  const end = new Vector3(0.05, 0, 0.2);
  const straight = start.distanceTo(end);
  for (const slack of [1e-6, 1e-4, 1e-2]) {
    const L = straight * (1 + slack);
    const pos = new Float64Array((segments + 1) * 3);
    writeSlackCablePositions(pos, start, end, L, up, segments);
    let pl = 0;
    for (let i = 0; i < segments; i += 1) {
      pl += Math.hypot(pos[3 * i + 3] - pos[3 * i], pos[3 * i + 4] - pos[3 * i + 1], pos[3 * i + 5] - pos[3 * i + 2]);
    }
    console.log(`    realistic span 0.206 m, slack ${slack}: polyline/L-1 = ${((pl / L - 1) * 100).toFixed(4)}%`);
  }
}

console.log('\n--- 5. signedArcLengthOnWheel cannot represent a wrap of 2*pi ---');
console.log('    (it returns the preferred-direction arc modulo 2*pi, always in [0, 2*pi))');
{
  const center = new Vector2(0, 0);
  const r = 1;
  const angles = [0, Math.PI / 2, Math.PI, 3 * Math.PI / 2, 2 * Math.PI + 0.3, 4 * Math.PI + 0.3];
  for (const a of angles) {
    const p = new Vector2(r, 0);
    const q = new Vector2(r * Math.cos(a), r * Math.sin(a));
    const forward = signedArcLengthOnWheel(p, q, center, r, false, false);
    const positive = signedArcLengthOnWheel(p, q, center, r, false, true);
    console.log(`    sweep ${a.toFixed(4)} rad: signed=${forward.toFixed(6)} force_positive=${positive.toFixed(6)}`);
  }
}

console.log('\n--- 6. normalizeInertiaTensor does not validate physical admissibility ---');
{
  const impossible = [[1, 0, 0], [0, 1, 0], [0, 0, 10]]; // I3 > I1 + I2
  const passed = it.normalizeInertiaTensor(impossible);
  const moi = new it.MomentOfInertiaComponent(impossible);
  console.log(`    authored diag(1,1,10) -> used as ${JSON.stringify(passed)}`);
  console.log(`    component inertia about z = ${moi.inertia}, inverse = ${moi.invInertia}`);
  console.log('    (all shipped scenes author isotropic diagonal tensors, so this is unexercised)');
}

console.log('\n--- 7. degenerate zero-length / zero-mass / duplicate-point inputs ---');
{
  const a = new Vector3(1, 2, 3);
  const dup = geo.closestPointOnSegment(new Vector3(0, 0, 0), a, a.clone());
  console.log(`    closestPointOnSegment(p, a, a) = (${dup.x}, ${dup.y}, ${dup.z}) (expected a)`);
  console.log(`    parallelAxisTensor(0, offset) = ${JSON.stringify(it.parallelAxisTensor(0, new Vector3(1, 2, 3)))}`);
  console.log(`    MomentOfInertiaComponent(0).inertia = ${new it.MomentOfInertiaComponent(0).inertia}`);
  const pos = new Float64Array(9);
  writeSlackCablePositions(pos, a, a.clone(), 0.1, new Vector3(0, 0, 1), 2);
  console.log(`    slack cable with start == end, L=0.1: ${Array.from(pos).map(v => v.toFixed(6)).join(', ')}`);
  const zeroSeg = new Float64Array(3);
  writeSlackCablePositions(zeroSeg, new Vector3(0, 0, 0), new Vector3(1, 0, 0), 2, new Vector3(0, 0, 1), 0);
  console.log(`    segments=0: ${Array.from(zeroSeg)} (only the end point survives, no NaN)`);
}
console.log('');
