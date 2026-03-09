import * as THREE from 'three';

jest.mock('three/addons/controls/OrbitControls.js', () => ({
  OrbitControls: class OrbitControls {}
}));

import { RenderSystem3D } from '../../../src/js/cable_joints_3d/render_system_3d.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  OrientationComponent
} from '../../../src/js/cable_joints_3d/ecs.js';
import { RenderableComponent } from '../../../src/js/cable_joints/ecs.js';

function createRenderSystemStub() {
  const system = Object.create(RenderSystem3D.prototype);
  system.root = new THREE.Group();
  system.sharedSphereGeometry = new THREE.SphereGeometry(1, 12, 8);
  system.sharedOrientationGeometry = RenderSystem3D.prototype._createOrientationHelperGeometry.call(system);
  system.sphereMaterialCache = new Map();
  system.orientationHelperMaterialCache = new Map();
  system.circleMeshes = new Map();
  system._activeCircleIds = new Set();

  system._getSphereMaterial = RenderSystem3D.prototype._getSphereMaterial;
  system._getOrientationHelperMaterial = RenderSystem3D.prototype._getOrientationHelperMaterial;
  system._ensureOrientationHelper = RenderSystem3D.prototype._ensureOrientationHelper;

  return system;
}

function disposeRenderSystemStub(system) {
  if (!system) return;

  for (const material of system.sphereMaterialCache.values()) {
    material.dispose();
  }
  system.sphereMaterialCache.clear();

  for (const material of system.orientationHelperMaterialCache.values()) {
    material.dispose();
  }
  system.orientationHelperMaterialCache.clear();

  system.sharedSphereGeometry.dispose();
  system.sharedOrientationGeometry.dispose();
}

describe('RenderSystem3D orientation helpers', () => {
  test('adds a shared visible line helper only for circles with OrientationComponent', () => {
    const system = createRenderSystemStub();

    try {
      const world = new World();

      const orientedA = world.createEntity();
      world.addComponent(orientedA, new PositionComponent(0.1, 0.2, 0.0));
      world.addComponent(orientedA, new RadiusComponent(0.04));
      world.addComponent(orientedA, new RenderableComponent('circle', '#dddddd'));
      world.addComponent(orientedA, new OrientationComponent());

      const orientedB = world.createEntity();
      world.addComponent(orientedB, new PositionComponent(0.3, 0.4, 0.0));
      world.addComponent(orientedB, new RadiusComponent(0.05));
      world.addComponent(orientedB, new RenderableComponent('circle', '#dddddd'));
      world.addComponent(orientedB, new OrientationComponent());

      const plainCircle = world.createEntity();
      world.addComponent(plainCircle, new PositionComponent(0.5, 0.6, 0.0));
      world.addComponent(plainCircle, new RadiusComponent(0.06));
      world.addComponent(plainCircle, new RenderableComponent('circle', '#dddddd'));

      RenderSystem3D.prototype._syncCircles.call(system, world);

      const meshA = system.circleMeshes.get(orientedA);
      const meshB = system.circleMeshes.get(orientedB);
      const plainMesh = system.circleMeshes.get(plainCircle);
      const helperA = meshA.userData.orientationHelper;
      const helperB = meshB.userData.orientationHelper;

      expect(helperA).toBeInstanceOf(THREE.Line);
      expect(helperB).toBeInstanceOf(THREE.Line);
      expect(helperA.visible).toBe(true);
      expect(helperB.visible).toBe(true);
      expect(helperA.geometry).toBe(system.sharedOrientationGeometry);
      expect(helperB.geometry).toBe(system.sharedOrientationGeometry);
      expect(helperA.material).toBe(helperB.material);
      expect(meshA.children).toContain(helperA);
      expect(meshB.children).toContain(helperB);
      expect(plainMesh.userData.orientationHelper).toBeUndefined();

      world.removeComponent(orientedA, OrientationComponent);
      RenderSystem3D.prototype._syncCircles.call(system, world);

      expect(meshA.userData.orientationHelper.visible).toBe(false);
      expect(meshB.userData.orientationHelper.visible).toBe(true);
    } finally {
      disposeRenderSystemStub(system);
    }
  });
});
