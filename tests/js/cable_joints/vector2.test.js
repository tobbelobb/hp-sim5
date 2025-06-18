import Vector2 from '../../../src/js/cable_joints/vector2.js';

describe('Vector2.angleTo', () => {
    const v1 = new Vector2(1, 0);
    const v2 = new Vector2(0, 1);
    const v3 = new Vector2(-1, 0);
    const v4 = new Vector2(1, 1);
    const v5 = new Vector2(0, 0);
    const v6 = new Vector2(2, 0); // Same direction as v1
    const v7 = new Vector2(-1, 1);

    test('should return 0 for angle between (1,0) and (0,0)', () => {
        expect(v1.angleTo(v5)).toBeCloseTo(0.0);
    });

    test('should return 0 for angle between (0,0) and (1,0)', () => {
        expect(v5.angleTo(v1)).toBeCloseTo(0.0);
    });

    test('should return 0 for angle between identical non-zero vectors (1,0) and (1,0)', () => {
        expect(v1.angleTo(v1.clone())).toBeCloseTo(0.0);
    });

    test('should return 0 for angle between vectors in the same direction (1,0) and (2,0)', () => {
        expect(v1.angleTo(v6)).toBeCloseTo(0.0);
    });

    test('should return PI for angle between opposite vectors (1,0) and (-1,0)', () => {
        expect(v1.angleTo(v3)).toBeCloseTo(Math.PI);
    });

    test('should return PI/2 for angle between orthogonal vectors (1,0) and (0,1)', () => {
        expect(v1.angleTo(v2)).toBeCloseTo(Math.PI / 2);
    });

    test('should return PI/2 for angle between (0,1) and (1,0) - symmetry', () => {
        expect(v2.angleTo(v1)).toBeCloseTo(Math.PI / 2);
    });

    test('should return PI/4 for angle at 45 degrees (1,0) and (1,1)', () => {
        expect(v1.angleTo(v4)).toBeCloseTo(Math.PI / 4);
    });

    test('should return 3*PI/4 for angle at 135 degrees (1,0) and (-1,1)', () => {
        expect(v1.angleTo(v7)).toBeCloseTo(3 * Math.PI / 4);
    });

    test('should return 3*PI/4 for angle at 135 degrees (1,1) and (-1,0)', () => {
        expect(v4.angleTo(v3)).toBeCloseTo(3 * Math.PI / 4);
    });

    test('should return PI/2 for angle between (-1,-1) and (1,-1)', () => {
        const v8 = new Vector2(-1, -1);
        const v9 = new Vector2(1, -1);
        expect(v8.angleTo(v9)).toBeCloseTo(Math.PI / 2);
    });
});
