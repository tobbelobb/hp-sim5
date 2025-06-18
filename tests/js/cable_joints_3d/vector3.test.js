import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';

test('length and distance work in 3D', () => {
  const a = new Vector3(1,2,3);
  const b = new Vector3(4,6,3);
  expect(a.length()).toBeCloseTo(Math.sqrt(14));
  expect(a.distanceTo(b)).toBeCloseTo(5);
});

test('angleTo returns expected values', () => {
  const v1 = new Vector3(1,0,0);
  const v2 = new Vector3(0,1,0);
  expect(v1.angleTo(v2)).toBeCloseTo(Math.PI/2);
  expect(v1.angleTo(v1.clone())).toBeCloseTo(0);
});

test('rotateAroundAxis rotates vector correctly', () => {
  const v = new Vector3(1,0,0);
  const axis = new Vector3(0,0,1);
  v.rotateAroundAxis(axis, Math.PI/2);
  expect(v.x).toBeCloseTo(0);
  expect(v.y).toBeCloseTo(1);
  expect(v.z).toBeCloseTo(0);
});
