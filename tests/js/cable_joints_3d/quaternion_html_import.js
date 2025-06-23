import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';

export const quaternionTests = {
  'Quaternion: Constructor & Clone': (assert, scene) => {
    let success = true;
    const q1 = new Quaternion(0.1, 0.2, 0.3, 0.4);
    success &&= assert.equal(q1.x, 0.1, 'q1.x');
    success &&= assert.equal(q1.y, 0.2, 'q1.y');
    success &&= assert.equal(q1.z, 0.3, 'q1.z');
    success &&= assert.equal(q1.w, 0.4, 'q1.w');

    const q2 = q1.clone();
    success &&= assert.equal(q2.x, 0.1, 'q2.x (from clone)');
    success &&= assert.true(q1 !== q2, 'cloned object is a new instance');

    const q3 = new Quaternion();
    success &&= assert.equal(q3.x, 0, 'default constructor x');
    success &&= assert.equal(q3.y, 0, 'default constructor y');
    success &&= assert.equal(q3.z, 0, 'default constructor z');
    success &&= assert.equal(q3.w, 1, 'default constructor w (identity)');
    return success;
  },

  'Quaternion: Set from Axis-Angle': (assert, scene) => {
    const axis = new Vector3(0, 1, 0); // Y-axis
    const angle = Math.PI / 2; // 90 degrees
    const q = new Quaternion().setFromAxisAngle(axis, angle);
    let success = true;

    const halfAngle = angle / 2; // PI / 4
    const s = Math.sin(halfAngle); // sqrt(2)/2
    const c = Math.cos(halfAngle); // sqrt(2)/2

    success &&= assert.near(q.x, 0, 1e-9, 'x');
    success &&= assert.near(q.y, s, 1e-9, 'y');
    success &&= assert.near(q.z, 0, 1e-9, 'z');
    success &&= assert.near(q.w, c, 1e-9, 'w');
    return success;
  },

  'Quaternion: Vector Transformation': (assert, scene) => {
    let success = true;
    // Rotate (1, 0, 0) by 90 degrees around Y-axis. Expect (0, 0, -1).
    const v = new Vector3(1, 0, 0);
    const axisY = new Vector3(0, 1, 0);
    const q = new Quaternion().setFromAxisAngle(axisY, Math.PI / 2);
    const rotatedV = q.transformVector(v);

    success &&= assert.near(rotatedV.x, 0, 1e-9, 'rotated x');
    success &&= assert.near(rotatedV.y, 0, 1e-9, 'rotated y');
    success &&= assert.near(rotatedV.z, -1, 1e-9, 'rotated z');

    // Visualize
    if (scene) {
        const { addVector } = scene.userData.helpers;
        addVector(new Vector3(0,0,0), v, 0xff0000); // Original in red
        addVector(new Vector3(0,0,0), axisY, 0x00ff00); // Axis in green
        addVector(new Vector3(0,0,0), rotatedV, 0x0000ff); // Rotated in blue
    }

    return success;
  },

  'Quaternion: Multiplication (Composition)': (assert, scene) => {
    let success = true;
    // 90 deg around Y then 90 deg around X
    const qY = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 2);
    const qX = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), Math.PI / 2);

    const qCombined = qX.clone().multiply(qY); // Y rotation first, then X

    const v = new Vector3(0, 0, 1); // Point on Z axis
    const rotatedV = qCombined.transformVector(v);

    // Applying Y rotation to (0,0,1) -> (1,0,0)
    // Applying X rotation to (1,0,0) -> (1,0,0)
    // Let's recheck the math. v' = q*v*q^-1
    // Y rotation on (0,0,1) gives (1,0,0).
    // X rotation on (1,0,0) gives (1,0,0).
    // So rotatedV should be (1,0,0).
    const v_after_Y = qY.transformVector(v);
    const v_after_Y_then_X = qX.transformVector(v_after_Y);

    success &&= assert.near(v_after_Y_then_X.x, rotatedV.x, 1e-9, 'composed rotation x');
    success &&= assert.near(v_after_Y_then_X.y, rotatedV.y, 1e-9, 'composed rotation y');
    success &&= assert.near(v_after_Y_then_X.z, rotatedV.z, 1e-9, 'composed rotation z');

    // Visualize
    if (scene) {
        const { addVector } = scene.userData.helpers;
        addVector(new Vector3(0,0,0), v, 0xff0000); // Original
        addVector(new Vector3(0,0,0), rotatedV, 0x0000ff); // Rotated
    }

    return success;
  },

  'Quaternion: Normalize & Conjugate': (assert, scene) => {
    let success = true;
    const q = new Quaternion(1, 2, 3, 4);
    q.normalize();
    success &&= assert.near(q.lengthSq(), 1, 1e-9, 'lengthSq after normalize is 1');

    const q2 = new Quaternion(0.1, 0.2, 0.3, 0.4);
    q2.conjugate();
    success &&= assert.equal(q2.x, -0.1, 'conjugate x');
    success &&= assert.equal(q2.y, -0.2, 'conjugate y');
    success &&= assert.equal(q2.z, -0.3, 'conjugate z');
    success &&= assert.equal(q2.w, 0.4, 'conjugate w is unchanged');

    return success;
  },
};
