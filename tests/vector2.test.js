import Vector2 from '../cable_joints/vector2.js';

function runAngleToTests() {
    console.log("Running Vector2.angleTo() tests...");
    const epsilon = 1e-9; // A small epsilon for float comparisons

    function Equals(a, b, message) {
        if (Math.abs(a - b) < epsilon) {
            console.log(`PASS: ${message} (Expected: ${b}, Got: ${a})`);
        } else {
            console.error(`FAIL: ${message} (Expected: ${b}, Got: ${a})`);
        }
    }

    // Test cases
    const v1 = new Vector2(1, 0);
    const v2 = new Vector2(0, 1);
    const v3 = new Vector2(-1, 0);
    const v4 = new Vector2(1, 1);
    const v5 = new Vector2(0, 0);
    const v6 = new Vector2(2, 0); // Same direction as v1
    const v7 = new Vector2(-1, 1);

    // 1. Angle with a zero vector (v1 to v5)
    let angle = v1.angleTo(v5);
    Equals(angle, 0.0, "Angle between (1,0) and (0,0) should be 0");

    // 2. Angle with a zero vector (v5 to v1)
    angle = v5.angleTo(v1);
    Equals(angle, 0.0, "Angle between (0,0) and (1,0) should be 0");

    // 3. Angle between identical non-zero vectors (v1 to v1)
    angle = v1.angleTo(v1.clone());
    Equals(angle, 0.0, "Angle between (1,0) and (1,0) should be 0");

    // 4. Angle between vectors in the same direction (v1 to v6)
    angle = v1.angleTo(v6);
    Equals(angle, 0.0, "Angle between (1,0) and (2,0) should be 0");

    // 5. Angle between opposite vectors (v1 to v3)
    angle = v1.angleTo(v3);
    Equals(angle, Math.PI, `Angle between (1,0) and (-1,0) should be ${Math.PI}`);

    // 6. Angle between orthogonal vectors (v1 to v2)
    angle = v1.angleTo(v2);
    Equals(angle, Math.PI / 2, `Angle between (1,0) and (0,1) should be ${Math.PI / 2}`);

    // 7. Angle between (0,1) and (1,0) - symmetry
    angle = v2.angleTo(v1);
    Equals(angle, Math.PI / 2, `Angle between (0,1) and (1,0) should be ${Math.PI / 2}`);

    // 8. Angle at 45 degrees (v1 to v4)
    angle = v1.angleTo(v4);
    Equals(angle, Math.PI / 4, `Angle between (1,0) and (1,1) should be ${Math.PI / 4}`);

    // 9. Angle at 135 degrees (v1 to v7)
    angle = v1.angleTo(v7);
    Equals(angle, 3 * Math.PI / 4, `Angle between (1,0) and (-1,1) should be ${3 * Math.PI / 4}`);

    // 10. Angle at 135 degrees (v4 to v3)
    angle = v4.angleTo(v3); // (1,1) to (-1,0)
    Equals(angle, 3 * Math.PI / 4, `Angle between (1,1) and (-1,0) should be ${3 * Math.PI / 4}`);

    // 11. Test with negative components
    const v8 = new Vector2(-1, -1);
    const v9 = new Vector2(1, -1);
    angle = v8.angleTo(v9); // (-1,-1) to (1,-1) -> angle should be PI/2
    Equals(angle, Math.PI / 2, `Angle between (-1,-1) and (1,-1) should be ${Math.PI / 2}`);

    console.log("Vector2.angleTo() tests finished.");
}

runAngleToTests();
