// Emit JS-side inputs/outputs for the independent numpy / pxr reference check.
// Reads: nothing. Writes: js_reference.json next to this script.
import { writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import Quaternion from '../../../../src/js/cable_joints_3d/quaternion.js';
import Vector3 from '../../../../src/js/cable_joints_3d/vector3.js';
import * as it from '../../../../src/js/cable_joints_3d/inertia_tensor.js';
import { writeSlackCablePositions } from '../../../../src/js/cable_joints_3d/cable_shape.js';
import { OpenText } from '../../../../src/js/usd/stage.js';
import { readOrientationAttribute } from '../../../../hp-sim-3d/app/scene/usdValueReaders.js';

const here = dirname(fileURLToPath(import.meta.url));

function mulberry32(seed) {
  return function next() {
    seed |= 0;
    seed = (seed + 0x6D2B79F5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const rng = mulberry32(0xC0FFEE);
const U = (lo, hi) => lo + (hi - lo) * rng();
const qrand = () => new Quaternion(U(-1, 1), U(-1, 1), U(-1, 1), U(-1, 1)).normalize();
const vrand = (s = 1) => new Vector3(U(-s, s), U(-s, s), U(-s, s));

const out = { rotateABC: [], quaternions: [], inertia: [], inverse: [], catenary: [] };

// 1. USD rotateABC -> quaternion, for every order.
for (const order of ['XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX']) {
  for (let i = 0; i < 8; i += 1) {
    const angles = [U(-180, 180), U(-180, 180), U(-180, 180)];
    const source = `#usda 1.0
def Xform "A"
{
    float3 xformOp:rotate${order} = (${angles[0]}, ${angles[1]}, ${angles[2]})
    uniform token[] xformOpOrder = ["xformOp:rotate${order}"]
}
`;
    const stage = OpenText(source);
    const q = readOrientationAttribute(stage.GetPrimAtPath('/A'));
    out.rotateABC.push({ order, angles, quaternion: [q.x, q.y, q.z, q.w] });
  }
}

// 2. quaternion -> rotation matrix and transformVector.
for (let i = 0; i < 25; i += 1) {
  const q = qrand();
  const v = vrand(2);
  const t = q.transformVector(v);
  out.quaternions.push({
    q: [q.x, q.y, q.z, q.w],
    R: it.rotationMatrixFromQuaternion(q),
    v: [v.x, v.y, v.z],
    tv: [t.x, t.y, t.z],
  });
}

// 3. transformInertiaTensorToWorld vs numpy R I R^T; also effective inertia.
for (let i = 0; i < 15; i += 1) {
  const q = qrand();
  const tensor = [[U(0.1, 5), 0, 0], [0, U(0.1, 5), 0], [0, 0, U(0.1, 5)]];
  const axis = vrand().normalize();
  out.inertia.push({
    q: [q.x, q.y, q.z, q.w],
    tensor,
    world: it.transformInertiaTensorToWorld(tensor, q),
    axis: [axis.x, axis.y, axis.z],
    effective: it.effectiveInertiaAboutWorldAxis(new it.MomentOfInertiaComponent(tensor), q, axis),
  });
}

// 4. invertMatrix3 vs numpy inv / pinv.
for (let i = 0; i < 15; i += 1) {
  const M = it.transformInertiaTensorToWorld(
    [[U(0.1, 5), 0, 0], [0, U(0.1, 5), 0], [0, 0, U(0.1, 5)]],
    qrand(),
  );
  out.inverse.push({ M, P: it.invertMatrix3(M), singular: false });
}
for (let i = 0; i < 5; i += 1) {
  const M = it.transformInertiaTensorToWorld([[1, 0, 0], [0, 1, 0], [0, 0, 0]], qrand());
  out.inverse.push({ M, P: it.invertMatrix3(M), singular: true });
}

// 5. slack cable polylines for an independent catenary fit.
for (const [h, v, L] of [[0.4, 0.15, 0.52], [0.02, 0.3, 0.36], [0.5, -0.2, 0.62], [0.05, 0.0, 0.13]]) {
  const start = new Vector3(0, 0, 0);
  const end = new Vector3(h, 0, v);
  const up = new Vector3(0, 0, 1);
  const segments = 64;
  const pos = new Float64Array((segments + 1) * 3);
  writeSlackCablePositions(pos, start, end, L, up, segments);
  out.catenary.push({
    start: [start.x, start.y, start.z],
    end: [end.x, end.y, end.z],
    up: [up.x, up.y, up.z],
    L,
    segments,
    points: Array.from(pos),
  });
}

writeFileSync(join(here, 'js_reference.json'), JSON.stringify(out));
console.log(`wrote js_reference.json: ${out.rotateABC.length} rotateABC, ${out.quaternions.length} quaternions, `
  + `${out.inertia.length} inertia, ${out.inverse.length} inverse, ${out.catenary.length} catenary cases`);
