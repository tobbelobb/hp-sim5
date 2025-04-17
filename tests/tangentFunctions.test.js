const { Vector2, _tangentPointCircle, tangentFromPointToCircle, tangentFromCircleToPoint, tangentFromCircleToCircle } = require('../cable_joints/cable_joints_core');

describe('_tangentPointCircle', () => {
  test('point outside circle returns correct tangent points', () => {
    const p_attach = new Vector2(0, 0);
    const p_circle = new Vector2(2, 0);
    const r_circle = 1;
    const result = _tangentPointCircle(p_attach, p_circle, r_circle, true, true);
    expect(result.a_attach.x).toBeCloseTo(0);
    expect(result.a_attach.y).toBeCloseTo(0);
    expect(result.a_circle.x).toBeCloseTo(1.5);
    expect(result.a_circle.y).toBeCloseTo(Math.sqrt(3)/2);
  });
});

describe('tangentFromPointToCircle', () => {
  test('wrapper function matches _tangentPointCircle with pointIsFirst=true', () => {
    const p_attach = new Vector2(1, 1);
    const p_circle = new Vector2(4, 1);
    const r_circle = 2;
    const cw = false;
    const result = tangentFromPointToCircle(p_attach, p_circle, r_circle, cw);
    const expected = _tangentPointCircle(p_attach, p_circle, r_circle, cw, true);
    expect(result).toEqual(expected);
  });
});

describe('tangentFromCircleToPoint', () => {
  test('wrapper function matches _tangentPointCircle with pointIsFirst=false', () => {
    const p_attach = new Vector2(1, 1);
    const p_circle = new Vector2(4, 1);
    const r_circle = 2;
    const cw = true;
    const result = tangentFromCircleToPoint(p_attach, p_circle, r_circle, cw);
    const expected = _tangentPointCircle(p_attach, p_circle, r_circle, cw, false);
    expect(result).toEqual(expected);
  });
});

describe('tangentFromCircleToCircle', () => {
  test('two identical circles of radius 1 centered at (0,0) and (4,0)', () => {
    const posA = new Vector2(0, 0);
    const radiusA = 1;
    const cwA = true;
    const posB = new Vector2(4, 0);
    const radiusB = 1;
    const cwB = true;
    const result = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
    expect(result.a_circle.x).toBeCloseTo(0);
    expect(result.a_circle.y).toBeCloseTo(1);
    expect(result.b_circle.x).toBeCloseTo(4);
    expect(result.b_circle.y).toBeCloseTo(1);
  });
});
