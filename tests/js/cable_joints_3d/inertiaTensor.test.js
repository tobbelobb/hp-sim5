import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import { MomentOfInertiaComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import {
  applyWorldInverseInertia,
  effectiveInertiaAboutWorldAxis,
  inverseInertiaQuadraticForm,
} from '../../../src/js/cable_joints_3d/inertia_tensor.js';

describe('3D moment of inertia tensor', () => {
  test('stores full tensor and inverse tensor while preserving axis scalar compatibility', () => {
    const inertia = new MomentOfInertiaComponent(
      [
        [2, 0, 0],
        [0, 4, 0],
        [0, 0, 8],
      ],
      { axisLocal: new Vector3(0, 1, 0) },
    );

    expect(inertia.inertiaTensor[0][0]).toBeCloseTo(2, 12);
    expect(inertia.inertiaTensor[1][1]).toBeCloseTo(4, 12);
    expect(inertia.inertiaTensor[2][2]).toBeCloseTo(8, 12);
    expect(inertia.invInertiaTensor[0][0]).toBeCloseTo(0.5, 12);
    expect(inertia.invInertiaTensor[1][1]).toBeCloseTo(0.25, 12);
    expect(inertia.invInertiaTensor[2][2]).toBeCloseTo(0.125, 12);
    expect(inertia.inertia).toBeCloseTo(4, 12);
    expect(inertia.invInertia).toBeCloseTo(0.25, 12);
  });

  test('applies world inverse inertia through orientation', () => {
    const inertia = new MomentOfInertiaComponent([
      [2, 0, 0],
      [0, 4, 0],
      [0, 0, 8],
    ]);
    const orientation = new Quaternion()
      .setFromAxisAngle(new Vector3(0, 0, 1), Math.PI / 2);

    const deltaX = applyWorldInverseInertia(inertia, orientation, new Vector3(1, 0, 0));
    const deltaY = applyWorldInverseInertia(inertia, orientation, new Vector3(0, 1, 0));

    expect(deltaX.x).toBeCloseTo(0.25, 12);
    expect(deltaX.y).toBeCloseTo(0.0, 12);
    expect(deltaY.x).toBeCloseTo(0.0, 12);
    expect(deltaY.y).toBeCloseTo(0.5, 12);
    expect(effectiveInertiaAboutWorldAxis(inertia, orientation, new Vector3(1, 0, 0))).toBeCloseTo(4, 12);
    expect(inverseInertiaQuadraticForm(inertia, orientation, new Vector3(1, 0, 0))).toBeCloseTo(0.25, 12);
  });
});
