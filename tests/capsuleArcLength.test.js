const { Vector2,
        capsuleArcLength,
        signedArcLengthOnWheel } = require('../cable_joints/cable_joints_core');

describe('capsuleArcLength', () => {
  // Common capsule : pivot at (0,0), tip at (0,2), radius = 1
  const pivot = new Vector2(0, 0);
  const tip   = new Vector2(0, 2);
  const radius = 1;

  test('quarter arc on pivot end-cap (CW default)', () => {
    const prev = new Vector2(1,  0);   // angle 0
    const curr = new Vector2(0, -1);   // angle –π/2  (external bottom)
    const len  = capsuleArcLength(prev, curr, pivot, tip, radius, true);
    expect(len).toBeCloseTo(Math.PI / 2);
  });

  test('Counter-clockwise preference flips sign', () => {
    const prev = new Vector2(1,  0);
    const curr = new Vector2(0, -1);
    const len  = capsuleArcLength(prev, curr, pivot, tip, radius, false);
    expect(len).toBeCloseTo(-Math.PI / 2);
  });

  test('force_positive turns negative arc into full positive perimeter', () => {
    const prev = new Vector2(1,  0);
    const curr = new Vector2(0, -1);
    // perimeter = 2*(π*r + L) = 2*(π + 2)  (here r=1, L=2)
    const expected = (3 * Math.PI) / 2 + 4; // = perimeter – π/2
    const len  = capsuleArcLength(prev, curr, pivot, tip, radius, false, true);
    expect(len).toBeCloseTo(expected);
  });

  test('side-segment length equals axial distance', () => {
    // along left side (side1) from tip to pivot (cw direction)
    const prev = new Vector2(-1, 2);   // t = L
    const curr = new Vector2(-1, 0);   // t = 0
    const len  = capsuleArcLength(prev, curr, pivot, tip, radius, false);
    expect(len).toBeCloseTo(2);        // L = 2
  });

  test('degenerate capsule (pivot == tip) falls back to circle logic', () => {
    const center = new Vector2(0, 0);
    const prev   = new Vector2(1, 0);
    const curr   = new Vector2(0, 1);
    const circleLen  = signedArcLengthOnWheel(prev, curr, center, radius, false);
    const capsuleLen = capsuleArcLength      (prev, curr, center, center, radius, false);
    expect(capsuleLen).toBeCloseTo(circleLen); // should be π/2
  });

  test('arc length along straight segment', () => {
    const prev   = new Vector2(-1, 2);
    const curr   = new Vector2(-1, 0);
    const capsuleLen = capsuleArcLength(prev, curr, pivot, tip, radius, false);
    expect(capsuleLen).toBeCloseTo(2.0);
  });
  test('arc length along straight segment 2', () => {
    const prev   = new Vector2(-1, 0);
    const curr   = new Vector2(-1, 2);
    const capsuleLen = capsuleArcLength(prev, curr, pivot, tip, radius, false);
    expect(capsuleLen).toBeCloseTo(-2.0);
  });
  test('arc length along straight segment 3', () => {
    const prev   = new Vector2(1, 0);
    const curr   = new Vector2(1, 2);
    const capsuleLen = capsuleArcLength(prev, curr, pivot, tip, radius, false);
    expect(capsuleLen).toBeCloseTo(2.0);
  });
  test('arc length along straight segment 4', () => {
    const prev   = new Vector2(1, 0);
    const curr   = new Vector2(1, 2);
    const capsuleLen = capsuleArcLength(prev, curr, pivot, tip, radius, true);
    expect(capsuleLen).toBeCloseTo(-2.0);
  });
  test('arc length along straight segment then quarter circle', () => {
    const prev   = new Vector2(1, 0);
    const curr   = new Vector2(0, 3);
    const capsuleLen = capsuleArcLength(prev, curr, pivot, tip, radius, true);
    expect(capsuleLen).toBeCloseTo(-2.0 - Math.PI/2);
  });
  test('arc length along quarter circle then straight segment then quarter circle', () => {
    const prev   = new Vector2(0, -1);
    const curr   = new Vector2(0, 3);
    const capsuleLen = capsuleArcLength(prev, curr, pivot, tip, radius, true);
    expect(capsuleLen).toBeCloseTo(-2.0 - Math.PI);
  });

  test('arc length tricky', () => {
    const prev   = new Vector2(0.8025903432334877, 1.5999664449799669);
    const curr   = new Vector2(0.3994033218808879, 1.5999982198602665);
    const capsuleLen = capsuleArcLength(
      prev,
      curr,
      new Vector2(0.4, 1.5),
      new Vector2(0.8, 1.5),
      0.1, true, false
    );
    expect(capsuleLen).toBeCloseTo(0.4031, 3);
  });
});
