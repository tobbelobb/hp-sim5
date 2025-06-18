import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';

describe('Quaternion', () => {
  test('constructor should create a valid quaternion', () => {
    const q1 = new Quaternion(0.1, 0.2, 0.3, 0.4);
    expect(q1.x).toBe(0.1);
    expect(q1.y).toBe(0.2);
    expect(q1.z).toBe(0.3);
    expect(q1.w).toBe(0.4);
  });

  test('default constructor should create an identity quaternion', () => {
    const q3 = new Quaternion();
    expect(q3.x).toBe(0);
    expect(q3.y).toBe(0);
    expect(q3.z).toBe(0);
    expect(q3.w).toBe(1);
  });

  test('clone should create a new instance with the same values', () => {
    const q1 = new Quaternion(0.1, 0.2, 0.3, 0.4);
    const q2 = q1.clone();
    expect(q2).toEqual(q1);
    expect(q2).not.toBe(q1);
  });

  test('setFromAxisAngle should correctly set quaternion values', () => {
    const axis = new Vector3(0, 1, 0); // Y-axis
    const angle = Math.PI / 2; // 90 degrees
    const q = new Quaternion().setFromAxisAngle(axis, angle);

    const halfAngle = angle / 2;
    const s = Math.sin(halfAngle);
    const c = Math.cos(halfAngle);

    expect(q.x).toBeCloseTo(0);
    expect(q.y).toBeCloseTo(s);
    expect(q.z).toBeCloseTo(0);
    expect(q.w).toBeCloseTo(c);
  });

  test('transformVector should rotate a vector correctly', () => {
    // Rotate (1, 0, 0) by 90 degrees around Y-axis. Expect (0, 0, -1).
    const v = new Vector3(1, 0, 0);
    const axisY = new Vector3(0, 1, 0);
    const q = new Quaternion().setFromAxisAngle(axisY, Math.PI / 2);
    const rotatedV = q.transformVector(v);

    expect(rotatedV.x).toBeCloseTo(0);
    expect(rotatedV.y).toBeCloseTo(0);
    expect(rotatedV.z).toBeCloseTo(-1);
  });

  test('multiplication should compose rotations correctly', () => {
    // 90 deg around Y then 90 deg around X
    const qY = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2);
    const qX = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), Math.PI / 2);

    // Combined rotation: Y first, then X. This corresponds to qX * qY.
    const qCombined = new Quaternion().multiplyQuaternions(qX, qY);

    const v = new Vector3(0, 0, 1); // Point on Z axis

    // Apply combined rotation
    const rotatedV = qCombined.transformVector(v);

    // Apply rotations sequentially for verification
    const v_after_Y = qY.transformVector(v);
    const v_after_Y_then_X = qX.transformVector(v_after_Y);

    expect(rotatedV.x).toBeCloseTo(v_after_Y_then_X.x);
    expect(rotatedV.y).toBeCloseTo(v_after_Y_then_X.y);
    expect(rotatedV.z).toBeCloseTo(v_after_Y_then_X.z);
  });

  test('normalize should produce a unit quaternion', () => {
    const q = new Quaternion(1, 2, 3, 4);
    q.normalize();
    expect(q.lengthSq()).toBeCloseTo(1);
  });

  test('conjugate should invert the vector part', () => {
    const q = new Quaternion(0.1, 0.2, 0.3, 0.4);
    q.conjugate();
    expect(q.x).toBe(-0.1);
    expect(q.y).toBe(-0.2);
    expect(q.z).toBe(-0.3);
    expect(q.w).toBe(0.4);
  });
});
