// Repro 2: Quaternion.normalize() and Vector3.normalize() do not produce unit
// vectors at the extremes of the double range.
//
// Expected: normalize() returns a unit vector for every finite nonzero input
// (the docstring/contract implies it; the only special case handled is
// length === 0, which returns the identity quaternion).
// Actual:
//   * quaternion with any component >= ~1e154 -> lengthSq overflows to
//     Infinity -> scale 1/Infinity = 0 -> the ZERO quaternion is returned.
//     Quaternion(0,0,0,0).transformVector(v) is (0,0,0), so every later
//     rotation silently collapses to the origin.
//   * vector with a subnormal length (<= ~2.2e-308) -> 1/|v| overflows to
//     Infinity -> the vector becomes (Infinity, NaN, NaN).
//
// Usage: node repro_normalize_range.mjs    (exit 1 while the bug is present)
import Quaternion from '../../../../src/js/cable_joints_3d/quaternion.js';
import Vector3 from '../../../../src/js/cable_joints_3d/vector3.js';

const failures = [];

const big = new Quaternion(1e200, 0.0, 0.0, 1.0);
const bigNorm = big.clone().normalize();
const bigLenSq = bigNorm.lengthSq();
console.log(`quaternion normalize((1e200,0,0,1)): (${bigNorm.x}, ${bigNorm.y}, ${bigNorm.z}, ${bigNorm.w}) |q|^2 = ${bigLenSq}`);
if (Math.abs(bigLenSq - 1) > 1e-12) {
  failures.push(`quaternion overflow: |q|^2 = ${bigLenSq}, expected 1`);
}
const collapsed = bigNorm.transformVector(new Vector3(1, 2, 3));
console.log(`  rotating (1,2,3) with it gives (${collapsed.x}, ${collapsed.y}, ${collapsed.z}) length ${collapsed.length()}`);
if (collapsed.length() > 1e-12) failures.push('expected the zero quaternion to collapse vectors to the origin');

const hugeAxis = new Vector3(1e200, 0.0, 0.0);
const qFromHugeAxis = new Quaternion().setFromAxisAngle(hugeAxis, 0.5).normalize();
console.log(`setFromAxisAngle((1e200,0,0), 0.5).normalize(): (${qFromHugeAxis.x}, ${qFromHugeAxis.y}, ${qFromHugeAxis.z}, ${qFromHugeAxis.w}) |q|^2 = ${qFromHugeAxis.lengthSq()}`);
if (Math.abs(qFromHugeAxis.lengthSq() - 1) > 1e-12) {
  failures.push(`setFromAxisAngle with a large unnormalized axis normalizes to |q|^2 = ${qFromHugeAxis.lengthSq()}`);
}

const subnormal = new Vector3(1e-310, 0.0, 0.0);
const subnormalNorm = subnormal.clone().normalize();
const finite = [subnormalNorm.x, subnormalNorm.y, subnormalNorm.z].every(Number.isFinite);
console.log(`vector normalize((1e-310,0,0)): (${subnormalNorm.x}, ${subnormalNorm.y}, ${subnormalNorm.z}) finite=${finite} length=${subnormalNorm.length()}`);
if (!finite || Math.abs(subnormalNorm.length() - 1) > 1e-12) {
  failures.push(`vector underflow: normalize() returned (${subnormalNorm.x}, ${subnormalNorm.y}, ${subnormalNorm.z})`);
}

console.log('');
if (failures.length > 0) {
  for (const f of failures) console.error(`  - ${f}`);
  console.error(`FAILED: ${failures.length} normalize() assertion(s). normalize() must return a unit vector for finite nonzero input; overflow/underflow of the reciprocal silently escapes the l === 0 guard.`);
  process.exit(1);
}
console.log('PASSED: normalize() is range-safe.');
