import Vector3 from '../vector3.js';

export const vector3Tests = {
  'Vector3: Constructor & Clone': (assert, scene) => {
    let success = true;
    const v1 = new Vector3(1, 2, 3);
    success &&= assert.equal(v1.x, 1, 'v1.x');
    success &&= assert.equal(v1.y, 2, 'v1.y');
    success &&= assert.equal(v1.z, 3, 'v1.z');

    const v2 = v1.clone();
    success &&= assert.equal(v2.x, 1, 'v2.x (from clone)');
    success &&= assert.equal(v2.y, 2, 'v2.y (from clone)');
    success &&= assert.equal(v2.z, 3, 'v2.z (from clone)');
    success &&= assert.true(v1 !== v2, 'cloned object is a new instance');

    const v3 = new Vector3();
    success &&= assert.equal(v3.x, 0, 'default constructor x');
    success &&= assert.equal(v3.y, 0, 'default constructor y');
    success &&= assert.equal(v3.z, 0, 'default constructor z');
    return success;
  },

  'Vector3: Set': (assert, scene) => {
    const v1 = new Vector3();
    const v2 = new Vector3(5, -6, 7);
    v1.set(v2);
    let success = true;
    success &&= assert.equal(v1.x, 5, 'v1.x after set');
    success &&= assert.equal(v1.y, -6, 'v1.y after set');
    success &&= assert.equal(v1.z, 7, 'v1.z after set');
    return success;
  },

  'Vector3: Add & Subtract': (assert, scene) => {
    const vA = new Vector3(1, 2, 3);
    const vB = new Vector3(4, 5, 6);
    let success = true;

    const res1 = vA.clone().add(vB);
    success &&= assert.near(res1.x, 5, 1e-9, 'add x');
    success &&= assert.near(res1.y, 7, 1e-9, 'add y');
    success &&= assert.near(res1.z, 9, 1e-9, 'add z');

    const res2 = vA.clone().subtract(vB);
    success &&= assert.near(res2.x, -3, 1e-9, 'subtract x');
    success &&= assert.near(res2.y, -3, 1e-9, 'subtract y');
    success &&= assert.near(res2.z, -3, 1e-9, 'subtract z');

    const res3 = new Vector3().addVectors(vA, vB);
    success &&= assert.near(res3.x, 5, 1e-9, 'addVectors x');
    success &&= assert.near(res3.y, 7, 1e-9, 'addVectors y');
    success &&= assert.near(res3.z, 9, 1e-9, 'addVectors z');

    const res4 = new Vector3().subtractVectors(vA, vB);
    success &&= assert.near(res4.x, -3, 1e-9, 'subtractVectors x');
    success &&= assert.near(res4.y, -3, 1e-9, 'subtractVectors y');
    success &&= assert.near(res4.z, -3, 1e-9, 'subtractVectors z');

    return success;
  },

  'Vector3: Scale': (assert, scene) => {
    const v = new Vector3(1, -2, 3);
    v.scale(3);
    let success = true;
    success &&= assert.equal(v.x, 3, 'scale x');
    success &&= assert.equal(v.y, -6, 'scale y');
    success &&= assert.equal(v.z, 9, 'scale z');
    return success;
  },

  'Vector3: Length & Distance': (assert, scene) => {
    const v1 = new Vector3(3, 4, 0);
    const v2 = new Vector3(3, 4, 12);
    const v3 = new Vector3(6, 8, 0);
    let success = true;
    success &&= assert.near(v1.length(), 5, 1e-9, 'length() on 3,4,0');
    success &&= assert.near(v1.lengthSq(), 25, 1e-9, 'lengthSq() on 3,4,0');
    success &&= assert.near(v2.length(), 13, 1e-9, 'length() on 3,4,12');
    success &&= assert.near(v2.lengthSq(), 169, 1e-9, 'lengthSq() on 3,4,12');
    success &&= assert.near(v1.distanceTo(v3), 5, 1e-9, 'distanceTo()');
    success &&= assert.near(v1.distanceToSq(v3), 25, 1e-9, 'distanceToSq()');
    return success;
  },

  'Vector3: Dot Product': (assert, scene) => {
    const v1 = new Vector3(1, 2, 3);
    const v2 = new Vector3(4, -5, 6);
    const dot = v1.dot(v2); // 1*4 + 2*(-5) + 3*6 = 4 - 10 + 18 = 12
    return assert.near(dot, 12, 1e-9, 'dot product');
  },

  'Vector3: Cross Product': (assert, scene) => {
    const vX = new Vector3(1, 0, 0);
    const vY = new Vector3(0, 1, 0);
    const vZ = new Vector3(0, 0, 1);
    let success = true;

    const res1 = vX.clone().cross(vY); // Should be Z
    success &&= assert.near(res1.x, 0, 1e-9, 'X cross Y -> x');
    success &&= assert.near(res1.y, 0, 1e-9, 'X cross Y -> y');
    success &&= assert.near(res1.z, 1, 1e-9, 'X cross Y -> z');

    const res2 = vY.clone().cross(vX); // Should be -Z
    success &&= assert.near(res2.x, 0, 1e-9, 'Y cross X -> x');
    success &&= assert.near(res2.y, 0, 1e-9, 'Y cross X -> y');
    success &&= assert.near(res2.z, -1, 1e-9, 'Y cross X -> z');

    const vA = new Vector3(2, 3, 4);
    const vB = new Vector3(5, 6, 7);
    const res3 = vA.clone().cross(vB);
    // x: 3*7 - 4*6 = 21 - 24 = -3
    // y: 4*5 - 2*7 = 20 - 14 = 6
    // z: 2*6 - 3*5 = 12 - 15 = -3
    success &&= assert.near(res3.x, -3, 1e-9, 'general cross x');
    success &&= assert.near(res3.y, 6, 1e-9, 'general cross y');
    success &&= assert.near(res3.z, -3, 1e-9, 'general cross z');

    return success;
  },

  'Vector3: Normalize': (assert, scene) => {
    const v = new Vector3(0, 5, 0);
    v.normalize();
    let success = true;
    success &&= assert.near(v.x, 0, 1e-9, 'normalize x');
    success &&= assert.near(v.y, 1, 1e-9, 'normalize y');
    success &&= assert.near(v.z, 0, 1e-9, 'normalize z');
    success &&= assert.near(v.length(), 1, 1e-9, 'length after normalize');

    const vZero = new Vector3(0, 0, 0);
    vZero.normalize();
    success &&= assert.near(vZero.length(), 0, 1e-9, 'normalize zero vector has length 0');

    return success;
  },
};
