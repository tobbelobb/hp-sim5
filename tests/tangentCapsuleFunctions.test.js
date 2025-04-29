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

  it('for a point directly above the middle, tangent‐point is correct', () => {
    const pA = new Vector2(0,0);
    const pB = new Vector2(2,0);
    const r  = 1.0;
    const cw = true;
    // attach directly above x=1 by 3 units
    const p_attach = new Vector2(1, 3);
    const out = tangentFromPointToCapsule(p_attach, pA, pB, r, cw);
    expect(out.a_attach.x).toBeCloseTo(1, 6);
    expect(out.a_attach.y).toBeCloseTo(3, 6);
    const ang = 2.0*Math.atan(1/3);
    const expect_x = 2.0 + Math.cos(ang);
    const expect_y = Math.sin(ang);
    expect(out.a_capsule.x).toBeCloseTo(expect_x, 6);
    expect(out.a_capsule.y).toBeCloseTo(expect_y, 6);
  });
});

describe('tangentFromCapsuleToPoint', () => {
});
