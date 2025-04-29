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
});
