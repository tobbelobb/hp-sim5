import * as THREE from 'three';

jest.mock('three/addons/controls/OrbitControls.js', () => ({
  OrbitControls: class OrbitControls {}
}));

import { RenderSystem3D } from '../../../src/js/cable_joints_3d/render_system_3d.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  OrientationComponent,
  HybridKnotAngleComponent
} from '../../../src/js/cable_joints_3d/ecs.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { RenderableComponent } from '../../../src/js/cable_joints/ecs.js';
import {
  CableJointComponent,
  CableLinkComponent,
  CablePathComponent
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';

const ORIENTATION_BACK_COLOR = new THREE.Color('#2a3542');
const ORIENTATION_FRONT_COLOR = new THREE.Color('#dddddd');

function createRenderSystemStub() {
  const system = Object.create(RenderSystem3D.prototype);
  system.root = new THREE.Group();
  system.sharedSphereGeometry = new THREE.SphereGeometry(1, 12, 8);
  system.sphereMaterialCache = new Map();
  system.flipperMaterialCache = new Map();
  system.orientedSphereGeometryCache = new Map();
  system.orientedSphereMaterial = new THREE.MeshStandardMaterial({
    vertexColors: true,
    roughness: 0.42,
    metalness: 0.1
  });
  system.knotMarkerMaterial = new THREE.MeshBasicMaterial({ color: '#ff3b30' });
  system.circleMeshes = new Map();
  system.jointLines = [];
  system.wrapArcs = [];
  system.knotMarkers = [];
  system._activeCircleIds = new Set();
  system.defaultPlaneNormal = new Vector3(0, 0, 1);

  system._getSphereMaterial = RenderSystem3D.prototype._getSphereMaterial;
  system._getOrientedSphereGeometry = RenderSystem3D.prototype._getOrientedSphereGeometry;
  system._createLine = RenderSystem3D.prototype._createLine;
  system._createArcLine = RenderSystem3D.prototype._createArcLine;
  system._ensureLineCapacity = RenderSystem3D.prototype._ensureLineCapacity;
  system._ensureKnotMarkerCapacity = RenderSystem3D.prototype._ensureKnotMarkerCapacity;
  system._hideLines = RenderSystem3D.prototype._hideLines;
  system._hideMeshes = RenderSystem3D.prototype._hideMeshes;
  system._buildKnotMarkerSpec = RenderSystem3D.prototype._buildKnotMarkerSpec;

  return system;
}

function disposeRenderSystemStub(system) {
  if (!system) return;

  for (const material of system.sphereMaterialCache.values()) {
    material.dispose();
  }
  system.sphereMaterialCache.clear();

  for (const geometry of system.orientedSphereGeometryCache.values()) {
    geometry.dispose();
  }
  system.orientedSphereGeometryCache.clear();

  system.orientedSphereMaterial.dispose();
  system.knotMarkerMaterial.dispose();
  system.sharedSphereGeometry.dispose();
}

function findVertexColor(geometry, predicate) {
  const positionAttr = geometry.attributes.position;
  const colorAttr = geometry.attributes.color;
  for (let i = 0; i < positionAttr.count; i += 1) {
    if (predicate(positionAttr.getX(i))) {
      return new THREE.Color(
        colorAttr.getX(i),
        colorAttr.getY(i),
        colorAttr.getZ(i)
      );
    }
  }
  return null;
}

describe('RenderSystem3D oriented circles', () => {
  test('renders OrientationComponent circles with cached half-and-half surface colors', () => {
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
      const geometryA = meshA.geometry;
      const geometryB = meshB.geometry;
      const positiveXColor = findVertexColor(geometryA, (x) => x > 0.01);
      const negativeXColor = findVertexColor(geometryA, (x) => x < -0.01);

      expect(meshA.material).toBe(system.orientedSphereMaterial);
      expect(meshB.material).toBe(system.orientedSphereMaterial);
      expect(geometryA).toBe(geometryB);
      expect(geometryA).not.toBe(system.sharedSphereGeometry);
      expect(geometryA.attributes.color).toBeDefined();
      expect(positiveXColor.r).toBeCloseTo(ORIENTATION_FRONT_COLOR.r, 6);
      expect(positiveXColor.g).toBeCloseTo(ORIENTATION_FRONT_COLOR.g, 6);
      expect(positiveXColor.b).toBeCloseTo(ORIENTATION_FRONT_COLOR.b, 6);
      expect(negativeXColor.r).toBeCloseTo(ORIENTATION_BACK_COLOR.r, 6);
      expect(negativeXColor.g).toBeCloseTo(ORIENTATION_BACK_COLOR.g, 6);
      expect(negativeXColor.b).toBeCloseTo(ORIENTATION_BACK_COLOR.b, 6);

      expect(plainMesh.geometry).toBe(system.sharedSphereGeometry);
      expect(plainMesh.material).not.toBe(system.orientedSphereMaterial);

      world.removeComponent(orientedA, OrientationComponent);
      RenderSystem3D.prototype._syncCircles.call(system, world);

      expect(meshA.geometry).toBe(system.sharedSphereGeometry);
      expect(meshA.material).not.toBe(system.orientedSphereMaterial);
      expect(meshB.geometry).toBe(geometryB);
    } finally {
      disposeRenderSystemStub(system);
    }
  });
});

describe('RenderSystem3D cable knot markers', () => {
  test('renders a small red knot marker at the hybrid endpoint knot position', () => {
    const system = createRenderSystemStub();

    try {
      const world = new World();
      world.setResource('enableLayering', true);

      const spool = world.createEntity();
      world.addComponent(spool, new PositionComponent(1.0, 2.0, 0.0));
      world.addComponent(spool, new RadiusComponent(0.5));
      world.addComponent(spool, new CableLinkComponent(1.0, 2.0, 0.0, null, new Vector3(0, 0, 1)));
      world.addComponent(spool, new OrientationComponent(0, 0, 0, 1));
      world.addComponent(spool, new HybridKnotAngleComponent(Math.PI / 4));

      const anchor = world.createEntity();
      const jointId = world.createEntity();
      world.addComponent(
        jointId,
        new CableJointComponent(
          spool,
          anchor,
          0.2,
          new Vector3(1.6, 2.0, 0.0),
          new Vector3(2.0, 2.0, 0.0)
        )
      );
      world.addComponent(jointId, new RenderableComponent('line', '#ffd34d'));

      const pathId = world.createEntity();
      const path = new CablePathComponent(
        world,
        [jointId],
        ['hybrid', 'attachment'],
        [false, false],
        1e6,
        [0.25, 0.0],
        0.1
      );
      world.addComponent(pathId, path);

      RenderSystem3D.prototype._syncCable.call(system, world);

      expect(system.knotMarkers).toHaveLength(1);

      const marker = system.knotMarkers[0];
      const expectedRadius = 0.6;
      const expectedCoord = expectedRadius / Math.sqrt(2.0);

      expect(marker.visible).toBe(true);
      expect(marker.material).toBe(system.knotMarkerMaterial);
      expect(marker.position.x).toBeCloseTo(1.0 + expectedCoord, 6);
      expect(marker.position.y).toBeCloseTo(2.0 + expectedCoord, 6);
      expect(marker.position.z).toBeCloseTo(0.0, 6);
      expect(marker.scale.x).toBeCloseTo(0.01, 8);
      expect(marker.scale.y).toBeCloseTo(0.01, 8);
      expect(marker.scale.z).toBeCloseTo(0.01, 8);
    } finally {
      disposeRenderSystemStub(system);
    }
  });
});
