import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { closestPointOnSegment, lineSegmentSphereIntersection } from '../../../src/js/cable_joints_3d/geometry3.js';

export const geometry3Tests = {
  'Geometry3: Closest Point on Segment': (assert, scene) => {
    let success = true;
    const a = new Vector3(0, 0, 0);
    const b = new Vector3(10, 0, 0);

    // Case 1: Point projects onto segment
    const p1 = new Vector3(5, 5, 0);
    const c1 = closestPointOnSegment(p1, a, b);
    success &&= assert.near(c1.x, 5, 1e-9, 'projects inside x');
    success &&= assert.near(c1.y, 0, 1e-9, 'projects inside y');
    success &&= assert.near(c1.z, 0, 1e-9, 'projects inside z');

    // Case 2: Point projects before start
    const p2 = new Vector3(-5, 5, 0);
    const c2 = closestPointOnSegment(p2, a, b);
    success &&= assert.near(c2.x, 0, 1e-9, 'projects before x');
    success &&= assert.near(c2.y, 0, 1e-9, 'projects before y');
    success &&= assert.near(c2.z, 0, 1e-9, 'projects before z');

    // Case 3: Point projects after end
    const p3 = new Vector3(15, 5, 0);
    const c3 = closestPointOnSegment(p3, a, b);
    success &&= assert.near(c3.x, 10, 1e-9, 'projects after x');
    success &&= assert.near(c3.y, 0, 1e-9, 'projects after y');
    success &&= assert.near(c3.z, 0, 1e-9, 'projects after z');

    if (scene) {
        const { addLine, addSphere } = scene.userData.helpers;
        addLine(a, b, 0xffffff);
        addSphere(p1, 0.1, 0xff0000); addSphere(c1, 0.1, 0x00ff00);
        addSphere(p2, 0.1, 0xff0000); addSphere(c2, 0.1, 0x00ff00);
        addSphere(p3, 0.1, 0xff0000); addSphere(c3, 0.1, 0x00ff00);
    }
    return success;
  },

  'Geometry3: Line-Sphere Intersection': (assert, scene) => {
    let success = true;
    const center = new Vector3(0, 0, 0);
    const radius = 1;

    // Case 1: No intersection
    const p1a = new Vector3(2, 2, 0);
    const p1b = new Vector3(3, 2, 0);
    success &&= assert.false(lineSegmentSphereIntersection(p1a, p1b, center, radius), 'no intersection');

    // Case 2: Segment passes through
    const p2a = new Vector3(-2, 0, 0);
    const p2b = new Vector3(2, 0, 0);
    success &&= assert.true(lineSegmentSphereIntersection(p2a, p2b, center, radius), 'passes through');

    // Case 3: One endpoint inside
    const p3a = new Vector3(0.5, 0, 0);
    const p3b = new Vector3(2, 0, 0);
    success &&= assert.false(lineSegmentSphereIntersection(p3a, p3b, center, radius), 'one endpoint inside by default');

    // Case 3b: One endpoint inside with explicit pierce semantics
    success &&= assert.true(lineSegmentSphereIntersection(p3a, p3b, center, radius, true), 'one endpoint inside with pierce enabled');

    // Case 4: Tangent
    const p4a = new Vector3(-2, 1, 0);
    const p4b = new Vector3(2, 1, 0);
    success &&= assert.true(lineSegmentSphereIntersection(p4a, p4b, center, radius), 'tangent');

    // Case 5: Segment completely inside
    const p5a = new Vector3(-0.5, 0, 0);
    const p5b = new Vector3(0.5, 0, 0);
    success &&= assert.false(lineSegmentSphereIntersection(p5a, p5b, center, radius), 'completely inside by default');

    if (scene) {
        const { addLine, addSphere } = scene.userData.helpers;
        addSphere(center, radius, 0x888888, true); // wireframe sphere
        addLine(p1a, p1b, 0xff0000); // No intersect - red
        addLine(p2a, p2b, 0x00ff00); // Intersect - green
        addLine(p3a, p3b, 0xff0000); // No intersect by default - red
        addLine(p4a, p4b, 0x00ff00); // Intersect - green
        addLine(p5a, p5b, 0xff0000); // No intersect by default - red
    }
    return success;
  },
};
