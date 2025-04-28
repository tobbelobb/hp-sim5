/*
// tests/tangentCapsuleFunctions.test.js
*/
const {
  Vector2,
  tangentFromPointToCircle,
  tangentFromCircleToPoint,
  tangentFromPointToCapsule,
  tangentFromCapsuleToPoint
} = require('../cable_joints_core');

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
    // projection is (1,0), so tangent circle center=(1,0)
    expect(out.a_attach.x).toBeCloseTo(1, 6);
    expect(out.a_attach.y).toBeCloseTo(3, 6);
    // the a_capsule should lie on circle (1,0) radius=1
    const d = out.a_capsule.clone().subtract(new Vector2(1,0)).length();
    expect(d).toBeCloseTo(r, 6);
    // And because cw=true, the tangent should land on the left hemisphere (x<1)
    expect(out.a_capsule.x).toBeLessThan(1 + EPS);
  });
});

describe('tangentFromCapsuleToPoint', () => {
  it('is inverse of tangentFromPointToCapsule (swap roles)', () => {
    const pA = new Vector2(0,0);
    const pB = new Vector2(1,0);
    const r  = 0.8;
    const cw = false;
    const p_attach = new Vector2(2,1);
    const out1 = tangentFromPointToCapsule(p_attach, pA, pB, r, cw);
    const out2 = tangentFromCapsuleToPoint(pA, pB, r, p_attach, cw);
    // swap names: out1.a_capsule ≈ out2.a_attach, out1.a_attach ≈ out2.a_capsule
    expect(out1.a_capsule.x).toBeCloseTo(out2.a_attach.x, 6);
    expect(out1.a_capsule.y).toBeCloseTo(out2.a_attach.y, 6);
    expect(out1.a_attach.x).toBeCloseTo(out2.a_capsule.x, 6);
    expect(out1.a_attach.y).toBeCloseTo(out2.a_capsule.y, 6);
  });
});
