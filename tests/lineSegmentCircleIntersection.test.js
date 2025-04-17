const { Vector2, lineSegmentCircleIntersection } = require('../cable_joints/cable_joints_core');

describe('lineSegmentCircleIntersection', () => {
  const center = new Vector2(0, 0);

  test('segment completely outside circle', () => {
    const p1 = new Vector2(0, -1.0000001);
    const p2 = new Vector2(2, -1);
    expect(lineSegmentCircleIntersection(p1, p2, center, 1)).toBe(false);
  });

  test('segment endpoint just inside circle', () => {
    const p1 = new Vector2(0.99999999, 0);
    const p2 = new Vector2(5, 0);
    expect(lineSegmentCircleIntersection(p1, p2, center, 1)).toBe(true);
  });

  test('segment passes through circle', () => {
    const p1 = new Vector2(-2, 0);
    const p2 = new Vector2(2, 0);
    expect(lineSegmentCircleIntersection(p1, p2, center, 1)).toBe(true);
  });

  test('segment tangent to circle', () => {
    const p1 = new Vector2(1, -1);
    const p2 = new Vector2(1, 1);
    expect(lineSegmentCircleIntersection(p1, p2, center, 1)).toBe(true);
  });

  test('zero-length segment inside circle', () => {
    const p1 = new Vector2(0, 0);
    const p2 = new Vector2(0, 0);
    expect(lineSegmentCircleIntersection(p1, p2, center, 1)).toBe(true);
  });

  test('zero-length segment outside circle', () => {
    const p1 = new Vector2(2, 0);
    const p2 = new Vector2(2, 0);
    expect(lineSegmentCircleIntersection(p1, p2, center, 1)).toBe(false);
  });

  test('going through zero-radius circle', () => {
    const p1 = new Vector2(-2, -2);
    const p2 = new Vector2(2, 2);
    expect(lineSegmentCircleIntersection(p1, p2, center, 0.0)).toBe(true);
  });

  test('segment through circle, negative radius', () => {
    const p1 = new Vector2(-2, 0);
    const p2 = new Vector2(2, 0);
    expect(lineSegmentCircleIntersection(p1, p2, center, -1)).toBe(false);
  });

  test('segment tangent to circle, negative radius', () => {
    const p1 = new Vector2(1, -1);
    const p2 = new Vector2(1, 1);
    expect(lineSegmentCircleIntersection(p1, p2, center, -1)).toBe(false);
  });

});
