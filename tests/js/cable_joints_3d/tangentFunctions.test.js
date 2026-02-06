import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  _tangentPointSphere,
  tangentFromPointToSphere,
  tangentFromSphereToPoint,
  tangentFromSphereToSphere
} from '../../../src/js/cable_joints_3d/geometry3.js';

describe('_tangentPointSphere', () => {
  test('point outside sphere returns correct tangent points', () => {
    const p_attach = new Vector3(0, 0, 0);
    const p_sphere = new Vector3(2, 0, 0);
    const r_sphere = 1;
    const planeNormal = new Vector3(0, 0, 1);
    const result = _tangentPointSphere(p_attach, p_sphere, r_sphere, planeNormal, true, true);
    expect(result.a_attach.x).toBeCloseTo(0);
    expect(result.a_attach.y).toBeCloseTo(0);
    expect(result.a_attach.z).toBeCloseTo(0);
    expect(result.a_sphere.x).toBeCloseTo(1.5);
    expect(result.a_sphere.y).toBeCloseTo(Math.sqrt(3) / 2);
    expect(result.a_sphere.z).toBeCloseTo(0);
  });
});

describe('tangentFromPointToSphere', () => {
  test('wrapper function matches _tangentPointSphere with pointIsFirst=true', () => {
    const p_attach = new Vector3(1, 1, 0);
    const p_sphere = new Vector3(4, 1, 0);
    const r_sphere = 2;
    const cw = false;
    const planeNormal = new Vector3(0, 0, 1);
    const result = tangentFromPointToSphere(p_attach, p_sphere, r_sphere, planeNormal, cw);
    const expected = _tangentPointSphere(p_attach, p_sphere, r_sphere, planeNormal, cw, true);
    expect(result).toEqual(expected);
  });
});

describe('tangentFromSphereToPoint', () => {
  test('wrapper function matches _tangentPointSphere with pointIsFirst=false', () => {
    const p_attach = new Vector3(1, 1, 0);
    const p_sphere = new Vector3(4, 1, 0);
    const r_sphere = 2;
    const cw = true;
    const planeNormal = new Vector3(0, 0, 1);
    const result = tangentFromSphereToPoint(p_attach, p_sphere, r_sphere, planeNormal, cw);
    const expected = _tangentPointSphere(p_attach, p_sphere, r_sphere, planeNormal, cw, false);
    expect(result).toEqual(expected);
  });
});

describe('tangentFromSphereToSphere', () => {
  test('two identical spheres of radius 1 centered at (0,0,0) and (4,0,0)', () => {
    const posA = new Vector3(0, 0, 0);
    const radiusA = 1;
    const cwA = true;
    const posB = new Vector3(4, 0, 0);
    const radiusB = 1;
    const cwB = true;
    const planeNormal = new Vector3(0, 0, 1);
    const result = tangentFromSphereToSphere(posA, radiusA, cwA, posB, radiusB, cwB, planeNormal);
    expect(result.a_sphere.x).toBeCloseTo(0);
    expect(result.a_sphere.y).toBeCloseTo(1);
    expect(result.a_sphere.z).toBeCloseTo(0);
    expect(result.b_sphere.x).toBeCloseTo(4);
    expect(result.b_sphere.y).toBeCloseTo(1);
    expect(result.b_sphere.z).toBeCloseTo(0);
  });
});
