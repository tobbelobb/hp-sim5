import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { rightOfPlane } from '../../../src/js/cable_joints_3d/geometry3.js';

describe('rightOfPlane', () => {
  const planeNormal = new Vector3(0, 0, 1);

  test('point to the right of horizontal line', () => {
    const p0 = new Vector3(0, 0, 0);
    const p1 = new Vector3(1, 0, 0);
    const x = new Vector3(0, -1, 0);
    expect(rightOfPlane(x, p0, p1, planeNormal)).toBe(true);
  });

  test('point to the left of horizontal line', () => {
    const p0 = new Vector3(0, 0, 0);
    const p1 = new Vector3(1, 0, 0);
    const x = new Vector3(0, 1, 0);
    expect(rightOfPlane(x, p0, p1, planeNormal)).toBe(false);
  });

  test('point collinear on line returns false', () => {
    const p0 = new Vector3(0, 0, 0);
    const p1 = new Vector3(1, 0, 0);
    const x = new Vector3(0.5, 0, 0);
    expect(rightOfPlane(x, p0, p1, planeNormal)).toBe(false);
  });

  test('point to the right of slanted line', () => {
    const p0 = new Vector3(0, 0, 0);
    const p1 = new Vector3(1, 1, 0);
    const x = new Vector3(1, 0, 0);
    expect(rightOfPlane(x, p0, p1, planeNormal)).toBe(true);
  });
});
