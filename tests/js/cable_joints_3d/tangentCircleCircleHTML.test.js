import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { tangentFromSphereToSphere } from '../../../src/js/cable_joints_3d/geometry3.js';

describe('tangentFromSphereToSphere HTML test cases (3D planar)', () => {
  const planeNormal = new Vector3(0, 0, 1);

  test('TT: cwA=true, cwB=true (outer tangent)', () => {
    const posA = new Vector3(0, 0, 0);
    const radiusA = 1.0;
    const cwA = true;
    const posB = new Vector3(3, 0, 0);
    const radiusB = 0.5;
    const cwB = true;
    const result = tangentFromSphereToSphere(posA, radiusA, cwA, posB, radiusB, cwB, planeNormal);
    const expectedA = new Vector3(1 / 6, Math.sqrt(35) / 6, 0);
    const expectedB = new Vector3(37 / 12, Math.sqrt(35) / 12, 0);
    expect(result.a_sphere.x).toBeCloseTo(expectedA.x);
    expect(result.a_sphere.y).toBeCloseTo(expectedA.y);
    expect(result.a_sphere.z).toBeCloseTo(0);
    expect(result.b_sphere.x).toBeCloseTo(expectedB.x);
    expect(result.b_sphere.y).toBeCloseTo(expectedB.y);
    expect(result.b_sphere.z).toBeCloseTo(0);
  });

  test('TF: cwA=true, cwB=false (internal tangent)', () => {
    const posA = new Vector3(0, 0, 0);
    const radiusA = 1.0;
    const cwA = true;
    const posB = new Vector3(3, 0, 0);
    const radiusB = 1.0;
    const cwB = false;
    const result = tangentFromSphereToSphere(posA, radiusA, cwA, posB, radiusB, cwB, planeNormal);
    const phi = Math.asin(2 / 3);
    const expectedA = new Vector3(Math.sin(phi), Math.cos(phi), 0);
    const expectedB = new Vector3(3 - Math.sin(phi), -Math.cos(phi), 0);
    expect(result.a_sphere.x).toBeCloseTo(expectedA.x);
    expect(result.a_sphere.y).toBeCloseTo(expectedA.y);
    expect(result.a_sphere.z).toBeCloseTo(0);
    expect(result.b_sphere.x).toBeCloseTo(expectedB.x);
    expect(result.b_sphere.y).toBeCloseTo(expectedB.y);
    expect(result.b_sphere.z).toBeCloseTo(0);
  });

  test('FT: cwA=false, cwB=true (internal tangent)', () => {
    const posA = new Vector3(0, 0, 0);
    const radiusA = 1.0;
    const cwA = false;
    const posB = new Vector3(3, 0, 0);
    const radiusB = 1.0;
    const cwB = true;
    const result = tangentFromSphereToSphere(posA, radiusA, cwA, posB, radiusB, cwB, planeNormal);
    const phi = Math.asin(2 / 3);
    const expectedA = new Vector3(Math.sin(phi), -Math.cos(phi), 0);
    const expectedB = new Vector3(3 - Math.sin(phi), Math.cos(phi), 0);
    expect(result.a_sphere.x).toBeCloseTo(expectedA.x);
    expect(result.a_sphere.y).toBeCloseTo(expectedA.y);
    expect(result.a_sphere.z).toBeCloseTo(0);
    expect(result.b_sphere.x).toBeCloseTo(expectedB.x);
    expect(result.b_sphere.y).toBeCloseTo(expectedB.y);
    expect(result.b_sphere.z).toBeCloseTo(0);
  });

  test('FF: cwA=false, cwB=false (outer tangent)', () => {
    const posA = new Vector3(0, 0, 0);
    const radiusA = 1.0;
    const cwA = false;
    const posB = new Vector3(3, 0, 0);
    const radiusB = 0.5;
    const cwB = false;
    const result = tangentFromSphereToSphere(posA, radiusA, cwA, posB, radiusB, cwB, planeNormal);
    const expectedA = new Vector3(1 / 6, -Math.sqrt(35) / 6, 0);
    const expectedB = new Vector3(37 / 12, -Math.sqrt(35) / 12, 0);
    expect(result.a_sphere.x).toBeCloseTo(expectedA.x);
    expect(result.a_sphere.y).toBeCloseTo(expectedA.y);
    expect(result.a_sphere.z).toBeCloseTo(0);
    expect(result.b_sphere.x).toBeCloseTo(expectedB.x);
    expect(result.b_sphere.y).toBeCloseTo(expectedB.y);
    expect(result.b_sphere.z).toBeCloseTo(0);
  });
});
