import Vector2 from '../../../src/js/cable_joints/vector2.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  lineSegmentCircleIntersection
} from '../../../src/js/cable_joints/geometry.js';
import {
  lineSegmentSphereIntersection
} from '../../../src/js/cable_joints_3d/geometry3.js';

describe('geometry3 lineSegmentSphereIntersection', () => {
  test('matches 2d default semantics when a segment endpoint starts inside the radius', () => {
    const p12 = new Vector2(0.0, 0.0);
    const p22 = new Vector2(2.0, 0.0);
    const center2 = new Vector2(0.5, 0.0);
    const radius = 0.75;

    const p13 = new Vector3(0.0, 0.0, 0.0);
    const p23 = new Vector3(2.0, 0.0, 0.0);
    const center3 = new Vector3(0.5, 0.0, 0.0);

    expect(lineSegmentCircleIntersection(p12, p22, center2, radius)).toBe(false);
    expect(lineSegmentSphereIntersection(p13, p23, center3, radius)).toBe(false);
  });

  test('supports explicitly treating endpoint pierce as an intersection', () => {
    const p1 = new Vector3(0.0, 0.0, 0.0);
    const p2 = new Vector3(2.0, 0.0, 0.0);
    const center = new Vector3(0.5, 0.0, 0.0);
    const radius = 0.75;

    expect(lineSegmentSphereIntersection(p1, p2, center, radius, true)).toBe(true);
  });

  test('agrees with the 2d helper for a planar piercing segment', () => {
    const p12 = new Vector2(-2.0, 0.0);
    const p22 = new Vector2(2.0, 0.0);
    const center2 = new Vector2(0.0, 0.0);
    const radius = 0.5;

    const p13 = new Vector3(-2.0, 0.0, 0.0);
    const p23 = new Vector3(2.0, 0.0, 0.0);
    const center3 = new Vector3(0.0, 0.0, 0.0);

    expect(lineSegmentCircleIntersection(p12, p22, center2, radius)).toBe(true);
    expect(lineSegmentSphereIntersection(p13, p23, center3, radius)).toBe(true);
  });
});
