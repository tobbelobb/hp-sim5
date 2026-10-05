import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import { MomentOfInertiaComponent } from '../../../src/js/cable_joints_3d/ecs.js';
import {
  applyWorldInverseInertia,
  effectiveInertiaAboutWorldAxis,
  inverseInertiaQuadraticForm,
  invertMatrix3,
  multiplyMatrix3,
  transformInertiaTensorToWorld,
} from '../../../src/js/cable_joints_3d/inertia_tensor.js';

describe('3D moment of inertia tensor', () => {
  test.each([1, 1e-6, 1e-12])('inverts rotated SPD inertia independently of scale %s', (scale) => {
    const rotation = new Quaternion().setFromAxisAngle(new Vector3(1, 2, 3).normalize(), .7);
    const tensor = transformInertiaTensorToWorld([
      [scale * .5, 0, 0], [0, scale, 0], [0, 0, scale * 1.5],
    ], rotation);
    const identity = multiplyMatrix3(invertMatrix3(tensor), tensor);
    identity.forEach((row, i) => row.forEach((value, j) => expect(value).toBeCloseTo(i === j ? 1 : 0, 12)));
  });

  test.each([[[0, 1, 2]], [[1, 0, 0]], [[0, 0, 0]]])('preserves supported directions of rotated PSD inertia %s', (moments) => {
    const rotation = new Quaternion().setFromAxisAngle(new Vector3(1, 2, 3).normalize(), .7);
    const tensor = transformInertiaTensorToWorld(moments.map((value, i) => moments.map((_, j) => i === j ? value * 1e-6 : 0)), rotation);
    const expected = transformInertiaTensorToWorld(moments.map((value, i) => moments.map((_, j) => i === j && value > 0 ? 1e6 / value : 0)), rotation);
    invertMatrix3(tensor).forEach((row, i) => row.forEach((value, j) => expect(value).toBeCloseTo(expected[i][j], 7)));
  });

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
