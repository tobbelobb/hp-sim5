import Vector3 from '../cable_joints_3d/vector3.js';

test('length and distance work in 3D', () => {
  const a = new Vector3(1,2,3);
  const b = new Vector3(4,6,3);
  expect(a.length()).toBeCloseTo(Math.sqrt(14));
  expect(a.distanceTo(b)).toBeCloseTo(5);
});
