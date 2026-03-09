import * as THREE from 'three';

jest.mock('three/addons/controls/OrbitControls.js', () => ({
  OrbitControls: class OrbitControls {}
}));

import { RenderSystem3D } from '../../../src/js/cable_joints_3d/render_system_3d.js';

describe('RenderSystem3D lighting defaults', () => {

  test('keeps the scene brighter with a lighter background and fill light', () => {
    expect(RenderSystem3D.DEFAULT_BACKGROUND_COLOR).toBe(0x1b2b3c);

    const lightsGroup = RenderSystem3D.createDefaultLightingGroup();
    expect(lightsGroup).toBeInstanceOf(THREE.Group);
    const hemisphere = lightsGroup.children.find((light) => light.type === 'HemisphereLight');
    expect(hemisphere).toBeDefined();
    expect(hemisphere.intensity).toBeCloseTo(0.35, 5);
  });
});
