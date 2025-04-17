const { Vector2, rightOfLine } = require('../cable_joints/cable_joints_core');

describe('rightOfLine', () => {
  test('point to the right of horizontal line', () => {
    const p0 = new Vector2(0, 0);
    const p1 = new Vector2(1, 0);
    const x = new Vector2(0, -1);
    expect(rightOfLine(x, p0, p1)).toBe(true);
  });

  test('point to the left of horizontal line', () => {
    const p0 = new Vector2(0, 0);
    const p1 = new Vector2(1, 0);
    const x = new Vector2(0, 1);
    expect(rightOfLine(x, p0, p1)).toBe(false);
  });

  test('point collinear on line returns false', () => {
    const p0 = new Vector2(0, 0);
    const p1 = new Vector2(1, 0);
    const x = new Vector2(0.5, 0);
    expect(rightOfLine(x, p0, p1)).toBe(false);
  });

  test('point to the right of slanted line', () => {
    const p0 = new Vector2(0, 0);
    const p1 = new Vector2(1, 1);
    const x = new Vector2(1, 0);
    expect(rightOfLine(x, p0, p1)).toBe(true);
  });
});
