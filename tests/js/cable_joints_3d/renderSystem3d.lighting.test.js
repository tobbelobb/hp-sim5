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
    const ambient = lightsGroup.children.find((light) => light.type === 'AmbientLight');
    expect(ambient).toBeDefined();
    expect(ambient.intensity).toBeCloseTo(1.35, 2);

    const directional = lightsGroup.children.find((light) => light.type === 'DirectionalLight');
    expect(directional).toBeDefined();
    expect(directional.intensity).toBeCloseTo(1.35, 2);

    const hemisphere = lightsGroup.children.find((light) => light.type === 'HemisphereLight');
    expect(hemisphere).toBeDefined();
    expect(hemisphere.intensity).toBeCloseTo(0.45, 2);
  });

  test('exposes border defaults for floor and walls', () => {
    expect(RenderSystem3D.DEFAULT_BORDER_FLOOR_COLOR).toBe(0x1d2434);
    expect(RenderSystem3D.DEFAULT_BORDER_WALL_COLOR).toBe(0x0c111f);
    expect(RenderSystem3D.DEFAULT_BORDER_FLOOR_Z).toBeCloseTo(-0.035, 5);
  });

  test('buildBorderShape returns null for insufficient points', () => {
    expect(RenderSystem3D.buildBorderShape([{ x: 0, y: 0 }, { x: 1, y: 0 }])).toBeNull();
  });

  test('buildBorderShape creates a shape that follows the points', () => {
    const points = [
      { x: 0, y: 0 },
      { x: 2, y: 0 },
      { x: 2, y: 1 },
      { x: 0, y: 1 }
    ];
    const shape = RenderSystem3D.buildBorderShape(points);

    expect(shape).toBeInstanceOf(THREE.Shape);
    const extracted = shape.extractPoints().shape;
    expect(extracted).toHaveLength(5);
    expect(extracted[4].x).toBeCloseTo(extracted[0].x);
    expect(extracted[4].y).toBeCloseTo(extracted[0].y);
    expect(extracted[0].x).toBeCloseTo(0);
    expect(extracted[0].y).toBeCloseTo(0);
    expect(extracted[2].x).toBeCloseTo(2);
    expect(extracted[2].y).toBeCloseTo(1);
  });
});
