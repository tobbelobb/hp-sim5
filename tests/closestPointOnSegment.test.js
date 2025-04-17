const { Vector2, closestPointOnSegment } = require('../cable_joints/cable_joints_core');

describe('closestPointOnSegment', () => {
  test('point projects inside segment', () => {
    const a = new Vector2(0, 0);
    const b = new Vector2(10, 0);
    const p = new Vector2(5, 5);
    const cp = closestPointOnSegment(p, a, b);
    expect(cp.x).toBeCloseTo(5);
    expect(cp.y).toBeCloseTo(0);
  });

  test('point projects before segment start', () => {
    const a = new Vector2(0, 0);
    const b = new Vector2(10, 0);
    const p = new Vector2(-5, 5);
    const cp = closestPointOnSegment(p, a, b);
    expect(cp.x).toBeCloseTo(0);
    expect(cp.y).toBeCloseTo(0);
  });

  test('point projects after segment end', () => {
    const a = new Vector2(0, 0);
    const b = new Vector2(10, 0);
    const p = new Vector2(15, -5);
    const cp = closestPointOnSegment(p, a, b);
    expect(cp.x).toBeCloseTo(10);
    expect(cp.y).toBeCloseTo(0);
  });

  test('zero segment', () => {
    const a = new Vector2(0, 0);
    const b = new Vector2(0, 0);
    const p = new Vector2(15, -5);
    const cp = closestPointOnSegment(p, a, b);
    expect(cp.x).toBeCloseTo(0);
    expect(cp.y).toBeCloseTo(0);
  });

  test('point projects on segment', () => {
    const a = new Vector2(0, 10);
    const b = new Vector2(0, 0);
    const p = new Vector2(0, 5);
    const cp = closestPointOnSegment(p, a, b);
    expect(cp.x).toBeCloseTo(0);
    expect(cp.y).toBeCloseTo(5);
  });
});
