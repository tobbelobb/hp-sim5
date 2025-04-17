const { Vector2, signedArcLengthOnWheel } = require('../cable_joints/cable_joints_core');

describe('signedArcLengthOnWheel', () => {
  const center = new Vector2(0, 0);

  test('happy path: quarter circle without force_positive (default)', () => {
    const prevPoint = new Vector2(1, 0);
    const currPoint = new Vector2(0, 1);
    const radius = 1;
    // angle from 0 to π/2 = π/2
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, false);
    expect(length).toBeCloseTo(Math.PI / 2);
  });

  test('force_positive true with negative angle yields full positive arc', () => {
    const prevPoint = new Vector2(1, 0);
    const currPoint = new Vector2(0, -1);
    const radius = 1;
    // angle from 0 to -π/2 = -π/2, force_positive adds 2π -> 3π/2
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, false, true);
    expect(length).toBeCloseTo((3 * Math.PI) / 2);
  });

  test('clockwise preference flips sign of arc length', () => {
    const prevPoint = new Vector2(1, 0);
    const currPoint = new Vector2(0, 1);
    const radius = 2;
    // angle π/2, with clockwisePreference true => -π/2 * 2 = -π
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, true);
    expect(length).toBeCloseTo(-Math.PI);
  });

  test('radius zero always returns zero', () => {
    const prevPoint = new Vector2(1, 0);
    const currPoint = new Vector2(0, 1);
    const radius = 0;
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, false);
    expect(length).toBeCloseTo(0);
  });

  test('negative radius inverts the sign of arc length', () => {
    const prevPoint = new Vector2(1, 0);
    const currPoint = new Vector2(0, 1);
    const radius = -2;
    // angle π/2 * -2 = -π
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, false);
    expect(length).toBeCloseTo(-Math.PI);
  });

  test('prevPoint not on circle still computes based on angles', () => {
    const prevPoint = new Vector2(2, 0); // distance 2 instead of radius
    const currPoint = new Vector2(0, 1);
    const radius = 1;
    // still angle from 0 to π/2 = π/2
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, false);
    expect(length).toBeCloseTo(Math.PI / 2);
  });

  test('currPoint not on circle still computes based on angles', () => {
    const prevPoint = new Vector2(1, 0);
    const currPoint = new Vector2(0, 2); // distance 2 instead of radius
    const radius = 1;
    // angle π/2 * 1 = π/2
    const length = signedArcLengthOnWheel(prevPoint, currPoint, center, radius, false);
    expect(length).toBeCloseTo(Math.PI / 2);
  });
});
