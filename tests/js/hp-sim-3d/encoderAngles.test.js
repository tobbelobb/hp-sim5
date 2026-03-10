import Quaternion from '../../../src/js/cable_joints_3d/quaternion.js';
import { orientationToDegrees } from '../../../hp-sim-3d/assets/encoder_angles.js';

describe('hp-sim-3d encoder angle helper', () => {
  test('returns degrees for 2D angle-based orientations', () => {
    expect(orientationToDegrees({ angle: Math.PI / 2 })).toBeCloseTo(90.0);
  });

  test('returns degrees for 3D quaternion orientations', () => {
    const quaternion = new Quaternion().setFromAxisAngle({ x: 0.0, y: 0.0, z: 1.0 }, Math.PI / 3);
    expect(orientationToDegrees({ quaternion })).toBeCloseTo(60.0);
  });

  test('returns null when no finite orientation can be resolved', () => {
    expect(orientationToDegrees({})).toBeNull();
  });
});
