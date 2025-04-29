const {
  Vector2,
  tangentFromPointToCircle,
  tangentFromCircleToPoint,
  tangentFromPointToCapsule,
  tangentFromCapsuleToPoint
} = require('../cable_joints/cable_joints_core');

const EPS = 1e-6;

describe('tangentFromPointToCapsule', () => {
  it('degenerate segment (pA === pB) falls back to point→circle', () => {
    const p_attach = new Vector2(2,3);
    const center   = new Vector2(1,1);
    const r = 0.5;
    const cw = true;
    const circ = tangentFromPointToCircle(p_attach, center, r, cw);
    const cap  = tangentFromPointToCapsule(p_attach, center, center, r, cw);
    expect(cap.a_attach.x).toBeCloseTo(circ.a_attach.x, 6);
    expect(cap.a_attach.y).toBeCloseTo(circ.a_attach.y, 6);
    expect(cap.a_capsule.x).toBeCloseTo(circ.a_circle.x, 6);
    expect(cap.a_capsule.y).toBeCloseTo(circ.a_circle.y, 6);
  });

  it('projects attach onto segment and clamps to [pA,pB]', () => {
    const pA = new Vector2(0,0);
    const pB = new Vector2(1,0);
    const r  = 1.0;
    const cw = false;
    // attach beyond pB on +x
    const p_attach = new Vector2(3, 0.2);
    const out = tangentFromPointToCapsule(p_attach, pA, pB, r, cw);
    // a_attach should equal original
    expect(out.a_attach.x).toBeCloseTo(3, 6);
    expect(out.a_attach.y).toBeCloseTo(0.2, 6);
    // the capsule‐tangent point must lie on the circle around proj=(1,0)
    const proj = new Vector2(1,0);
    const d = out.a_capsule.clone().subtract(proj).length();
    expect(d).toBeCloseTo(r, 6);
  });

  // --- Tests ported from cable_joints_test.html ---
  // Common setup for ported tests
  const pA_common = new Vector2(0.0, 0.0);
  const pB_common = new Vector2(2.0, 0.0); // length=2, angle=0
  const r_common = 1.0;

  it('testTangentPointCapsule_leftCw', () => {
    const cw = true;
    const p_attach = new Vector2(1.0, 3.0);
    const out = tangentFromPointToCapsule(p_attach, pA_common, pB_common, r_common, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    const ang = 2.0 * Math.atan(1 / 3);
    const expect_x = 2.0 + Math.cos(ang); // Tangent on pB end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  it('testTangentPointCapsule_leftCCw', () => {
    const cw = false;
    const p_attach = new Vector2(1.0, 3.0);
    const out = tangentFromPointToCapsule(p_attach, pA_common, pB_common, r_common, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    const ang = Math.PI - 2.0 * Math.atan(1 / 3);
    const expect_x = Math.cos(ang); // Tangent on pA end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  it('testTangentPointCapsule_rightCw', () => {
    const cw = true;
    const p_attach = new Vector2(1.0, -3.0);
    const out = tangentFromPointToCapsule(p_attach, pA_common, pB_common, r_common, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    const ang = Math.PI + 2.0 * Math.atan(1 / 3);
    const expect_x = Math.cos(ang); // Tangent on pA end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  it('testTangentPointCapsule_rightCCw', () => {
    const cw = false;
    const p_attach = new Vector2(1.0, -3.0);
    const out = tangentFromPointToCapsule(p_attach, pA_common, pB_common, r_common, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    const ang = -2.0 * Math.atan(1 / 3);
    const expect_x = 2.0 + Math.cos(ang); // Tangent on pB end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  it('testTangentPointCapsule_inlineRightCw', () => {
    const cw = true;
    const p_attach = new Vector2(4.0, -0.1);
    const out = tangentFromPointToCapsule(p_attach, pA_common, pB_common, r_common, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    const ang = -Math.atan(0.1 / 2.0) - Math.acos(1.0 / Math.sqrt(4.01));
    const expect_x = 2.0 + Math.cos(ang); // Tangent on pB end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  it('testTangentPointCapsule_inlineRightCCw', () => {
    const cw = false;
    const p_attach = new Vector2(4.0, -0.1);
    const out = tangentFromPointToCapsule(p_attach, pA_common, pB_common, r_common, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    const ang = -Math.atan(0.1 / 2.0) + Math.acos(1.0 / Math.sqrt(4.01));
    const expect_x = 2.0 + Math.cos(ang); // Tangent on pB end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  it('testTangentPointCapsule_inlineRightCw2', () => {
    const cw = true;
    const p_attach = new Vector2(4.0, 0.1);
    const out = tangentFromPointToCapsule(p_attach, pA_common, pB_common, r_common, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    const ang = Math.atan(0.1 / 2.0) - Math.acos(1.0 / Math.sqrt(4.01));
    const expect_x = 2.0 + Math.cos(ang); // Tangent on pB end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  it('testTangentPointCapsule_inlineRightCCw2', () => {
    const cw = false;
    const p_attach = new Vector2(4.0, 0.1);
    const out = tangentFromPointToCapsule(p_attach, pA_common, pB_common, r_common, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    const ang = Math.atan(0.1 / 2.0) + Math.acos(1.0 / Math.sqrt(4.01));
    const expect_x = 2.0 + Math.cos(ang); // Tangent on pB end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  it('testTangentPointCapsule_inlineLeftCw', () => {
    const cw = true;
    const p_attach = new Vector2(-2.0, -0.1);
    const out = tangentFromPointToCapsule(p_attach, pA_common, pB_common, r_common, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    const ang = Math.PI + Math.atan(0.1 / 2.0) - Math.acos(1.0 / Math.sqrt(4.01));
    const expect_x = Math.cos(ang); // Tangent on pA end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  it('testTangentPointCapsule_inlineLeftCCw', () => {
    const cw = false;
    const p_attach = new Vector2(-2.0, -0.1);
    const out = tangentFromPointToCapsule(p_attach, pA_common, pB_common, r_common, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    const ang = Math.PI + Math.atan(0.1 / 2.0) + Math.acos(1.0 / Math.sqrt(4.01));
    const expect_x = Math.cos(ang); // Tangent on pA end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  it('testTangentPointCapsule_inlineLeftCw2', () => {
    const cw = true;
    const p_attach = new Vector2(-2.0, 0.1);
    const out = tangentFromPointToCapsule(p_attach, pA_common, pB_common, r_common, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    const ang = Math.PI - Math.atan(0.1 / 2.0) - Math.acos(1.0 / Math.sqrt(4.01));
    const expect_x = Math.cos(ang); // Tangent on pA end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  it('testTangentPointCapsule_inlineLeftCCw2', () => {
    const cw = false;
    const p_attach = new Vector2(-2.0, 0.1);
    const out = tangentFromPointToCapsule(p_attach, pA_common, pB_common, r_common, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    const ang = Math.PI - Math.atan(0.1 / 2.0) + Math.acos(1.0 / Math.sqrt(4.01));
    const expect_x = Math.cos(ang); // Tangent on pA end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });
});

describe('tangentFromCapsuleToPoint', () => {
  // Common setup for ported tests
  const pA_common = new Vector2(0.0, 0.0);
  const pB_common = new Vector2(2.0, 0.0); // length=2, angle=0
  const r_common = 1.0;

  // Test cases derived from tangentFromPointToCapsule using the identity:
  // tangentFromCapsuleToPoint(pA, pB, r, p_attach, cw) === tangentFromPointToCapsule(p_attach, pA, pB, r, !cw)

  it('degenerate segment (pA === pB) falls back to circle→point', () => {
    const p_attach = new Vector2(2,3);
    const center   = new Vector2(1,1);
    const r = 0.5;
    const cw = true; // For capsule->point
    const circ = tangentFromCircleToPoint(p_attach, center, r, cw);
    const cap  = tangentFromCapsuleToPoint(center, center, r, p_attach, cw);
    expect(cap.a_attach.x).toBeCloseTo(circ.a_attach.x, 6);
    expect(cap.a_attach.y).toBeCloseTo(circ.a_attach.y, 6);
    expect(cap.a_capsule.x).toBeCloseTo(circ.a_circle.x, 6);
    expect(cap.a_capsule.y).toBeCloseTo(circ.a_circle.y, 6);
  });

  it('testTangentCapsuleToPoint_leftCw (derived from PointToCapsule_leftCCw)', () => {
    const cw = true; // For capsule->point
    const p_attach = new Vector2(1.0, 3.0);
    const out = tangentFromCapsuleToPoint(pA_common, pB_common, r_common, p_attach, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    // Expected result from tangentFromPointToCapsule with cw=false
    const ang = Math.PI - 2.0 * Math.atan(1 / 3);
    const expect_x = Math.cos(ang); // Tangent on pA end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  it('testTangentCapsuleToPoint_leftCCw (derived from PointToCapsule_leftCw)', () => {
    const cw = false; // For capsule->point
    const p_attach = new Vector2(1.0, 3.0);
    const out = tangentFromCapsuleToPoint(pA_common, pB_common, r_common, p_attach, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    // Expected result from tangentFromPointToCapsule with cw=true
    const ang = 2.0 * Math.atan(1 / 3);
    const expect_x = 2.0 + Math.cos(ang); // Tangent on pB end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

   it('testTangentCapsuleToPoint_rightCw (derived from PointToCapsule_rightCCw)', () => {
    const cw = true; // For capsule->point
    const p_attach = new Vector2(1.0, -3.0);
    const out = tangentFromCapsuleToPoint(pA_common, pB_common, r_common, p_attach, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    // Expected result from tangentFromPointToCapsule with cw=false
    const ang = -2.0 * Math.atan(1 / 3);
    const expect_x = 2.0 + Math.cos(ang); // Tangent on pB end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  it('testTangentCapsuleToPoint_rightCCw (derived from PointToCapsule_rightCw)', () => {
    const cw = false; // For capsule->point
    const p_attach = new Vector2(1.0, -3.0);
    const out = tangentFromCapsuleToPoint(pA_common, pB_common, r_common, p_attach, cw);
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
    // Expected result from tangentFromPointToCapsule with cw=true
    const ang = Math.PI + 2.0 * Math.atan(1 / 3);
    const expect_x = Math.cos(ang); // Tangent on pA end cap
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });

  // --- Specific tests for tangentFromCapsuleToPoint ---

  it('tangent point on top straight segment (cw=true)', () => {
    const pA = new Vector2(0, 0);
    const pB = new Vector2(2, 0);
    const r = 0.5;
    const p_attach = new Vector2(1, 2); // Point above the middle
    const cw = true; // Capsule -> Point CW

    const out = tangentFromCapsuleToPoint(pA, pB, r, p_attach, cw);

    // Expected tangent point on capsule should be (p_attach.x, r)
    expect(out.a_capsule.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_capsule.y).toBeCloseTo(r, 6);
    // Attachment point should be the original point
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
  });

  it('tangent point on top straight segment (cw=false)', () => {
    const pA = new Vector2(0, 0);
    const pB = new Vector2(2, 0);
    const r = 0.5;
    const p_attach = new Vector2(1, 2); // Point above the middle
    const cw = false; // Capsule -> Point CCW

    const out = tangentFromCapsuleToPoint(pA, pB, r, p_attach, cw);

    // Expected tangent point on capsule should be (p_attach.x, r)
    // Note: For a point directly above the straight segment, CW and CCW tangents from the capsule *to the point* should land on the same capsule point (the closest point on the boundary).
    // The difference lies in which *direction* the tangent line goes.
    expect(out.a_capsule.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_capsule.y).toBeCloseTo(r, 6);
    // Attachment point should be the original point
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
  });

  it('tangent point on bottom straight segment (cw=true)', () => {
    const pA = new Vector2(0, 0);
    const pB = new Vector2(2, 0);
    const r = 0.5;
    const p_attach = new Vector2(1, -2); // Point below the middle
    const cw = true; // Capsule -> Point CW

    const out = tangentFromCapsuleToPoint(pA, pB, r, p_attach, cw);

    // Expected tangent point on capsule should be (p_attach.x, -r)
    expect(out.a_capsule.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_capsule.y).toBeCloseTo(-r, 6);
    // Attachment point should be the original point
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
  });

    it('tangent point on bottom straight segment (cw=false)', () => {
    const pA = new Vector2(0, 0);
    const pB = new Vector2(2, 0);
    const r = 0.5;
    const p_attach = new Vector2(1, -2); // Point below the middle
    const cw = false; // Capsule -> Point CCW

    const out = tangentFromCapsuleToPoint(pA, pB, r, p_attach, cw);

    // Expected tangent point on capsule should be (p_attach.x, -r)
    expect(out.a_capsule.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_capsule.y).toBeCloseTo(-r, 6);
    // Attachment point should be the original point
    expect(out.a_attach.x).toBeCloseTo(p_attach.x, 6);
    expect(out.a_attach.y).toBeCloseTo(p_attach.y, 6);
  });

});
