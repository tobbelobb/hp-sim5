import Vector2 from '../cable_joints/vector2.js';
import { tangentFromCircleToCircle } from '../cable_joints/geometry.js';

describe('tangentFromCircleToCircle HTML test cases', () => {
  test('TT: cwA=true, cwB=true (outer tangent)', () => {
    const posA = new Vector2(0, 0);
    const radiusA = 1.0;
    const cwA = true;
    const posB = new Vector2(3, 0);
    const radiusB = 0.5;
    const cwB = true;
    const result = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
    const expectedA = new Vector2(1/6, Math.sqrt(35)/6);
    const expectedB = new Vector2(37/12, Math.sqrt(35)/12);
    expect(result.a_circle.x).toBeCloseTo(expectedA.x);
    expect(result.a_circle.y).toBeCloseTo(expectedA.y);
    expect(result.b_circle.x).toBeCloseTo(expectedB.x);
    expect(result.b_circle.y).toBeCloseTo(expectedB.y);
  });

  test('TF: cwA=true, cwB=false (internal tangent)', () => {
    const posA = new Vector2(0, 0);
    const radiusA = 1.0;
    const cwA = true;
    const posB = new Vector2(3, 0);
    const radiusB = 1.0;
    const cwB = false;
    const result = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
    const phi = Math.asin(2/3);
    const expectedA = new Vector2(Math.sin(phi), Math.cos(phi));
    const expectedB = new Vector2(3 - Math.sin(phi), -Math.cos(phi));
    expect(result.a_circle.x).toBeCloseTo(expectedA.x);
    expect(result.a_circle.y).toBeCloseTo(expectedA.y);
    expect(result.b_circle.x).toBeCloseTo(expectedB.x);
    expect(result.b_circle.y).toBeCloseTo(expectedB.y);
  });

  test('FT: cwA=false, cwB=true (internal tangent)', () => {
    const posA = new Vector2(0, 0);
    const radiusA = 1.0;
    const cwA = false;
    const posB = new Vector2(3, 0);
    const radiusB = 1.0;
    const cwB = true;
    const result = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
    const phi = Math.asin(2/3);
    const expectedA = new Vector2(Math.sin(phi), -Math.cos(phi));
    const expectedB = new Vector2(3 - Math.sin(phi), Math.cos(phi));
    expect(result.a_circle.x).toBeCloseTo(expectedA.x);
    expect(result.a_circle.y).toBeCloseTo(expectedA.y);
    expect(result.b_circle.x).toBeCloseTo(expectedB.x);
    expect(result.b_circle.y).toBeCloseTo(expectedB.y);
  });

  test('FF: cwA=false, cwB=false (outer tangent)', () => {
    const posA = new Vector2(0, 0);
    const radiusA = 1.0;
    const cwA = false;
    const posB = new Vector2(3, 0);
    const radiusB = 0.5;
    const cwB = false;
    const result = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
    const expectedA = new Vector2(1/6, -Math.sqrt(35)/6);
    const expectedB = new Vector2(37/12, -Math.sqrt(35)/12);
    expect(result.a_circle.x).toBeCloseTo(expectedA.x);
    expect(result.a_circle.y).toBeCloseTo(expectedA.y);
    expect(result.b_circle.x).toBeCloseTo(expectedB.x);
    expect(result.b_circle.y).toBeCloseTo(expectedB.y);
  });
});
