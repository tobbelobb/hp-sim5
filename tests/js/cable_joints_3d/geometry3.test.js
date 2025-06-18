import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { closestPointOnSegment, lineSegmentSphereIntersection } from '../../../src/js/cable_joints_3d/geometry3.js';

describe('Geometry3', () => {
  describe('closestPointOnSegment', () => {
    const a = new Vector3(0, 0, 0);
    const b = new Vector3(10, 0, 0);

    test('should return point on segment when projection is inside', () => {
      const p1 = new Vector3(5, 5, 0);
      const c1 = closestPointOnSegment(p1, a, b);
      expect(c1.x).toBeCloseTo(5);
      expect(c1.y).toBeCloseTo(0);
      expect(c1.z).toBeCloseTo(0);
    });

    test('should return start point when projection is before start', () => {
      const p2 = new Vector3(-5, 5, 0);
      const c2 = closestPointOnSegment(p2, a, b);
      expect(c2.x).toBeCloseTo(0);
      expect(c2.y).toBeCloseTo(0);
      expect(c2.z).toBeCloseTo(0);
    });

    test('should return end point when projection is after end', () => {
      const p3 = new Vector3(15, 5, 0);
      const c3 = closestPointOnSegment(p3, a, b);
      expect(c3.x).toBeCloseTo(10);
      expect(c3.y).toBeCloseTo(0);
      expect(c3.z).toBeCloseTo(0);
    });
  });

  describe('lineSegmentSphereIntersection', () => {
    const center = new Vector3(0, 0, 0);
    const radius = 1;

    test('should return false for no intersection', () => {
      const p1a = new Vector3(2, 2, 0);
      const p1b = new Vector3(3, 2, 0);
      expect(lineSegmentSphereIntersection(p1a, p1b, center, radius)).toBe(false);
    });

    test('should return true when segment passes through sphere', () => {
      const p2a = new Vector3(-2, 0, 0);
      const p2b = new Vector3(2, 0, 0);
      expect(lineSegmentSphereIntersection(p2a, p2b, center, radius)).toBe(true);
    });

    test('should return true when one endpoint is inside sphere', () => {
      const p3a = new Vector3(0.5, 0, 0);
      const p3b = new Vector3(2, 0, 0);
      expect(lineSegmentSphereIntersection(p3a, p3b, center, radius)).toBe(true);
    });

    test('should return true for a tangent segment', () => {
      const p4a = new Vector3(-2, 1, 0);
      const p4b = new Vector3(2, 1, 0);
      expect(lineSegmentSphereIntersection(p4a, p4b, center, radius)).toBe(true);
    });

    test('should return true when segment is completely inside sphere', () => {
      const p5a = new Vector3(-0.5, 0, 0);
      const p5b = new Vector3(0.5, 0, 0);
      expect(lineSegmentSphereIntersection(p5a, p5b, center, radius)).toBe(true);
    });
  });
});
