// Repro 1: Vector3.rotateAroundAxis() is not a rotation for a degenerate axis.
//
// Expected: because the method itself normalizes the axis, any axis it can
// normalize is accepted, and a zero axis should behave like the identity
// (this is what every other zero-normal guard in the codebase does:
// geometry3.buildPlaneBasis, inertia_tensor.normalizedAxis,
// hangprinter_spools.normalizeSpoolAxisLocal, cable_joints_core._projectPointToPlane).
// Actual: a zero axis is "normalized" to the zero vector and the Rodrigues
// terms collapse, so the result is v*cos(angle): the length of the vector is
// destroyed (angle = pi/2) or its direction flipped (angle = pi). A subnormal
// axis produces NaN because 1/|axis| overflows.
//
// Usage: node repro_rotate_zero_axis.mjs     (exit 1 while the bug is present)
import Vector3 from '../../../../src/js/cable_joints_3d/vector3.js';

const failures = [];
function expect(label, expected, actual, tol) {
  const err = Math.abs(expected - actual);
  const ok = err <= tol;
  console.log(`${ok ? 'ok  ' : 'BUG '} ${label}: expected ${expected.toExponential(6)} actual ${actual.toExponential(6)} |err| ${err.toExponential(3)} tol ${tol.toExponential(1)}`);
  if (!ok) failures.push({ label, expected, actual, err, tol });
}

const v = new Vector3(1.0, 2.0, 3.0);
const zeroAxis = new Vector3(0.0, 0.0, 0.0);

// Rotation must be an isometry; with a zero axis it must also be the identity.
const rotatedQuarter = v.clone().rotateAroundAxis(zeroAxis, Math.PI / 2);
expect('|rotateAroundAxis(0, pi/2) v|', v.length(), rotatedQuarter.length(), 1e-12 * v.length());
expect('rotateAroundAxis(0, pi/2) v .x', v.x, rotatedQuarter.x, 1e-12);
expect('rotateAroundAxis(0, pi/2) v .y', v.y, rotatedQuarter.y, 1e-12);
expect('rotateAroundAxis(0, pi/2) v .z', v.z, rotatedQuarter.z, 1e-12);

const rotatedPi = v.clone().rotateAroundAxis(zeroAxis, Math.PI);
expect('rotateAroundAxis(0, pi) v .x', v.x, rotatedPi.x, 1e-12);

// A subnormal-length axis is nonzero, but 1/|axis| overflows to Infinity.
const subnormalAxis = new Vector3(1e-310, 0.0, 0.0);
const rotatedSubnormal = v.clone().rotateAroundAxis(subnormalAxis, 0.5);
console.log(`${Number.isFinite(rotatedSubnormal.x) && Number.isFinite(rotatedSubnormal.y) && Number.isFinite(rotatedSubnormal.z) ? 'ok  ' : 'BUG '} rotateAroundAxis(subnormal axis) is finite: ${rotatedSubnormal.x}, ${rotatedSubnormal.y}, ${rotatedSubnormal.z}`);
if (![rotatedSubnormal.x, rotatedSubnormal.y, rotatedSubnormal.z].every(Number.isFinite)) {
  failures.push({ label: 'subnormal axis produces NaN/Inf', actual: [rotatedSubnormal.x, rotatedSubnormal.y, rotatedSubnormal.z] });
}

console.log('');
if (failures.length > 0) {
  console.error(`FAILED: ${failures.length} assertion(s). rotateAroundAxis collapses a vector when the axis normalizes to zero, so it is not a rotation (not length preserving).`);
  process.exit(1);
}
console.log('PASSED: rotateAroundAxis behaves as a rotation for degenerate axes.');
