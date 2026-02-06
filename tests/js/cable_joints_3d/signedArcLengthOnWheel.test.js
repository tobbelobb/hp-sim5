import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  signedArcLengthOnWheel,
  tangentFromPointToSphere,
  tangentFromSphereToPoint
} from '../../../src/js/cable_joints_3d/geometry3.js';

describe('signedArcLengthOnWheel (3D planar)', () => {
  const center = new Vector3(0, 0, 0);
  const planeNormal = new Vector3(0, 0, 1);

  test('happy path: quarter circle without force_positive (default)', () => {
    const prevPoint = new Vector3(1, 0, 0);
    const currPoint = new Vector3(0, 1, 0);
    const radius = 1;
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, false, planeNormal);
    expect(length).toBeCloseTo(Math.PI / 2);
  });

  test('force_positive true with negative angle yields full positive arc', () => {
    const prevPoint = new Vector3(1, 0, 0);
    const currPoint = new Vector3(0, -1, 0);
    const radius = 1;
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, false, planeNormal, true);
    expect(length).toBeCloseTo((3 * Math.PI) / 2);
  });

  test('clockwise preference flips sign of arc length', () => {
    const prevPoint = new Vector3(1, 0, 0);
    const currPoint = new Vector3(0, 1, 0);
    const radius = 2;
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, true, planeNormal);
    expect(length).toBeCloseTo(-Math.PI);
  });

  test('radius zero always returns zero', () => {
    const prevPoint = new Vector3(1, 0, 0);
    const currPoint = new Vector3(0, 1, 0);
    const radius = 0;
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, false, planeNormal);
    expect(length).toBeCloseTo(0);
  });

  test('negative radius inverts the sign of arc length', () => {
    const prevPoint = new Vector3(1, 0, 0);
    const currPoint = new Vector3(0, 1, 0);
    const radius = -2;
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, false, planeNormal);
    expect(length).toBeCloseTo(-Math.PI);
  });

  test('prevPoint not on circle still computes based on angles', () => {
    const prevPoint = new Vector3(2, 0, 0);
    const currPoint = new Vector3(0, 1, 0);
    const radius = 1;
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, false, planeNormal);
    expect(length).toBeCloseTo(Math.PI / 2);
  });

  test('currPoint not on circle still computes based on angles', () => {
    const prevPoint = new Vector3(1, 0, 0);
    const currPoint = new Vector3(0, 2, 0);
    const radius = 1;
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, false, planeNormal);
    expect(length).toBeCloseTo(Math.PI / 2);
  });

  test('large initial wrap around obstacle', () => {
    const cw = true;
    const obsRadius = 0.1;
    const posObs = new Vector3(1.0, 0.3, 0);
    const pBall1 = new Vector3(1.025, -0.5, 0);
    const pBall2 = new Vector3(1.2, -0.7, 0);
    const ip1 = tangentFromPointToSphere(pBall1, posObs, obsRadius, planeNormal, cw);
    const ip2 = tangentFromSphereToPoint(pBall2, posObs, obsRadius, planeNormal, cw);
    const wrapLength = signedArcLengthOnWheel(ip1.a_sphere, ip2.a_sphere, posObs, obsRadius, cw, planeNormal, true);
    expect(wrapLength).toBeGreaterThan(0);
    expect(wrapLength).toBeGreaterThan(Math.PI * obsRadius);
    expect(wrapLength).toBeLessThan(Math.PI * 1.1 * obsRadius);
  });
});
