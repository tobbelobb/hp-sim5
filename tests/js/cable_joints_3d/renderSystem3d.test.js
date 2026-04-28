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
  RigidGroupComponent
} from '../../../src/js/cable_joints_3d/ecs.js';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import { RenderableComponent } from '../../../src/js/cable_joints/ecs.js';
import {
  CableJointComponent,
  CableLinkComponent,
  CablePathComponent
} from '../../../src/js/cable_joints_3d/cable_joints_core.js';
import { ObstaclePushComponent } from '../../../example_apps/js/flipper/flipper_common.js';
import { SpoolStateComponent } from '../../../hp-sim-3d/app/hangprinter_spools.js';

const ORIENTATION_BACK_COLOR = new THREE.Color('#2a3542');
const ORIENTATION_FRONT_COLOR = new THREE.Color('#dddddd');

function createRenderSystemStub() {
  const system = Object.create(RenderSystem3D.prototype);
  system.root = new THREE.Group();
  system.sharedSphereGeometry = new THREE.SphereGeometry(1, 12, 8);
  system.sharedCylinderGeometry = new THREE.CylinderGeometry(1, 1, 1, 12, 1, false);
  system.sphereMaterialCache = new Map();
  system.flipperMaterialCache = new Map();
  system.orientedSphereGeometryCache = new Map();
  system.orientedCylinderGeometryCache = new Map();
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
  system.forceSigns = [];
  system.rigidGroupLines = [];
  system._activeCircleIds = new Set();
  system.defaultPlaneNormal = new Vector3(0, 0, 1);
  system._bumperFxGroup = new THREE.Group();
  system._bumperFxBursts = [];
  system._bumperFxActivePairs = new Set();
  system._bumperFxLastTimeSec = Number.NaN;
  system._bumperFxGeometry = new THREE.SphereGeometry(1, 12, 10);
  system._lastObstacleActivePairs = new Set();

  system._getSphereMaterial = RenderSystem3D.prototype._getSphereMaterial;
  system._getOrientedSphereGeometry = RenderSystem3D.prototype._getOrientedSphereGeometry;
  system._getOrientedCylinderGeometry = RenderSystem3D.prototype._getOrientedCylinderGeometry;
  system._createLine = RenderSystem3D.prototype._createLine;
  system._createArcLine = RenderSystem3D.prototype._createArcLine;
  system._ensureLineCapacity = RenderSystem3D.prototype._ensureLineCapacity;
  system._ensureKnotMarkerCapacity = RenderSystem3D.prototype._ensureKnotMarkerCapacity;
  system._ensureForceSignCapacity = RenderSystem3D.prototype._ensureForceSignCapacity;
  system._createForceSign = RenderSystem3D.prototype._createForceSign;
  system._updateForceSignText = RenderSystem3D.prototype._updateForceSignText;
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

  for (const geometry of system.orientedCylinderGeometryCache.values()) {
    geometry.dispose();
  }
  system.orientedCylinderGeometryCache.clear();

  system.orientedSphereMaterial.dispose();
  system.knotMarkerMaterial.dispose();
  system.sharedSphereGeometry.dispose();
  system.sharedCylinderGeometry.dispose();
  for (const sign of system.forceSigns ?? []) {
    if (sign.material?.map) {
      sign.material.map.dispose();
    }
    sign.material?.dispose?.();
  }
  if (typeof RenderSystem3D.prototype._clearBumperFx === 'function') {
    RenderSystem3D.prototype._clearBumperFx.call(system);
  }
  if (system._bumperFxGeometry) {
    system._bumperFxGeometry.dispose();
  }
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

function geometryHasVertexColor(geometry, targetColor, tolerance = 1e-6) {
  const colorAttr = geometry.attributes.color;
  if (!colorAttr) {
    return false;
  }
  for (let i = 0; i < colorAttr.count; i += 1) {
    const dr = Math.abs(colorAttr.getX(i) - targetColor.r);
    const dg = Math.abs(colorAttr.getY(i) - targetColor.g);
    const db = Math.abs(colorAttr.getZ(i) - targetColor.b);
    if (dr <= tolerance && dg <= tolerance && db <= tolerance) {
      return true;
    }
  }
  return false;
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

  test('renders spools as cylinders aligned to their local rotation axis', () => {
    const system = createRenderSystemStub();

    try {
      const world = new World();
      const spoolColor = new THREE.Color('#999999');
      const spool = world.createEntity();
      const orientation = new OrientationComponent();
      orientation.quaternion.setFromAxisAngle(new Vector3(1.0, 0.0, 0.0), Math.PI / 4);

      world.addComponent(spool, new PositionComponent(0.1, 0.2, 0.3));
      world.addComponent(spool, new RadiusComponent(0.04));
      world.addComponent(spool, new RenderableComponent('cylinder', '#999999'));
      world.addComponent(spool, orientation);
      world.addComponent(spool, new SpoolStateComponent('A'));

      RenderSystem3D.prototype._syncCircles.call(system, world);

      const mesh = system.circleMeshes.get(spool);
      const cylinderAxis = new THREE.Vector3(0, 1, 0).applyQuaternion(mesh.quaternion).normalize();
      const expectedAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(
        new THREE.Quaternion(
          orientation.quaternion.x,
          orientation.quaternion.y,
          orientation.quaternion.z,
          orientation.quaternion.w,
        ),
      ).normalize();

      expect(mesh.material).toBe(system.orientedSphereMaterial);
      expect(mesh.geometry).not.toBe(system.sharedCylinderGeometry);
      expect(mesh.geometry.attributes.color).toBeDefined();
      expect(geometryHasVertexColor(mesh.geometry, spoolColor)).toBe(true);
      expect(geometryHasVertexColor(mesh.geometry, ORIENTATION_BACK_COLOR)).toBe(true);
      expect(mesh.scale.x).toBeCloseTo(0.04, 8);
      expect(mesh.scale.y).toBeCloseTo(0.04, 8);
      expect(mesh.scale.z).toBeCloseTo(0.04, 8);
      expect(cylinderAxis.x).toBeCloseTo(expectedAxis.x, 6);
      expect(cylinderAxis.y).toBeCloseTo(expectedAxis.y, 6);
      expect(cylinderAxis.z).toBeCloseTo(expectedAxis.z, 6);
    } finally {
      disposeRenderSystemStub(system);
    }
  });

  test('uses authored cylinder height without changing default cylinder scale', () => {
    const system = createRenderSystemStub();

    try {
      const world = new World();

      const defaultCylinder = world.createEntity();
      world.addComponent(defaultCylinder, new PositionComponent(0.0, 0.0, 0.0));
      world.addComponent(defaultCylinder, new RadiusComponent(0.04));
      world.addComponent(defaultCylinder, new RenderableComponent('cylinder', '#999999'));

      const authoredCylinder = world.createEntity();
      world.addComponent(authoredCylinder, new PositionComponent(0.1, 0.0, 0.0));
      world.addComponent(authoredCylinder, new RadiusComponent(0.04));
      world.addComponent(authoredCylinder, new RenderableComponent('cylinder', '#999999', { height: 0.004 }));

      RenderSystem3D.prototype._syncCircles.call(system, world);

      const defaultMesh = system.circleMeshes.get(defaultCylinder);
      const authoredMesh = system.circleMeshes.get(authoredCylinder);

      expect(defaultMesh.scale.x).toBeCloseTo(0.04, 8);
      expect(defaultMesh.scale.y).toBeCloseTo(0.04, 8);
      expect(defaultMesh.scale.z).toBeCloseTo(0.04, 8);
      expect(authoredMesh.scale.x).toBeCloseTo(0.04, 8);
      expect(authoredMesh.scale.y).toBeCloseTo(0.004, 8);
      expect(authoredMesh.scale.z).toBeCloseTo(0.04, 8);
    } finally {
      disposeRenderSystemStub(system);
    }
  });
});

describe('RenderSystem3D bumper hit fx', () => {
  test('spawns a burst when render bumper fx is enabled', () => {
    const system = createRenderSystemStub();

    try {
      const world = new World();
      const ball = world.createEntity();
      world.addComponent(ball, new PositionComponent(0.1, 0.1, 0.0));
      world.addComponent(ball, new RadiusComponent(0.04));

      const obstacle = world.createEntity();
      world.addComponent(obstacle, new PositionComponent(0.3, 0.1, 0.0));
      world.addComponent(obstacle, new RadiusComponent(0.05));
      world.addComponent(obstacle, new RenderableComponent('circle', '#ff9000'));
      world.addComponent(obstacle, new ObstaclePushComponent(1.8));

      world.setResource('renderBumperHitFx', true);
      world.setResource('ball_obstacle_contacts', [{
        ball_id: ball,
        obs_id: obstacle,
        raw_hit: true,
        delta_lambda: 0.42,
        direction: new Vector3(1, 0, 0)
      }]);

      let currentTime = 0;
      system._nowSeconds = () => currentTime;
      RenderSystem3D.prototype._updateBumperHitFx.call(system, world);

      expect(system._bumperFxBursts).toHaveLength(1);
      expect(system._bumperFxGroup.children).toHaveLength(1);
      const burst = system._bumperFxBursts[0];
      expect(burst.targetScale).toBeGreaterThanOrEqual(0.03);
      expect(burst.mesh?.material?.opacity).toBeGreaterThanOrEqual(0);
    } finally {
      disposeRenderSystemStub(system);
    }
  });

  test('clears bursts when the render flag is turned off', () => {
    const system = createRenderSystemStub();

    try {
      const world = new World();
      const ball = world.createEntity();
      world.addComponent(ball, new PositionComponent(0.1, 0.1, 0.0));
      world.addComponent(ball, new RadiusComponent(0.04));

      const obstacle = world.createEntity();
      world.addComponent(obstacle, new PositionComponent(0.3, 0.1, 0.0));
      world.addComponent(obstacle, new RadiusComponent(0.05));
      world.addComponent(obstacle, new RenderableComponent('circle', '#ff9000'));
      world.addComponent(obstacle, new ObstaclePushComponent(1.8));

      world.setResource('renderBumperHitFx', true);
      world.setResource('ball_obstacle_contacts', [{
        ball_id: ball,
        obs_id: obstacle,
        raw_hit: true,
        delta_lambda: 0.2,
        direction: new Vector3(1, 0, 0)
      }]);

      let currentTime = 0;
      system._nowSeconds = () => currentTime;
      RenderSystem3D.prototype._updateBumperHitFx.call(system, world);
      expect(system._bumperFxBursts).toHaveLength(1);

      world.setResource('renderBumperHitFx', false);
      currentTime += 0.01;
      RenderSystem3D.prototype._updateBumperHitFx.call(system, world);

      expect(system._bumperFxBursts).toHaveLength(0);
      expect(system._bumperFxGroup.children).toHaveLength(0);
    } finally {
      disposeRenderSystemStub(system);
    }
  });

  test('does not spawn bursts for overlay-only contacts (raw_hit false)', () => {
    const system = createRenderSystemStub();

    try {
      const world = new World();
      const ball = world.createEntity();
      world.addComponent(ball, new PositionComponent(0.1, 0.1, 0.0));
      world.addComponent(ball, new RadiusComponent(0.04));

      const obstacle = world.createEntity();
      world.addComponent(obstacle, new PositionComponent(0.3, 0.1, 0.0));
      world.addComponent(obstacle, new RadiusComponent(0.05));
      world.addComponent(obstacle, new RenderableComponent('circle', '#ff9000'));
      world.addComponent(obstacle, new ObstaclePushComponent(1.8));

      world.setResource('renderBumperHitFx', true);
      world.setResource('ball_obstacle_contacts', [{
        ball_id: ball,
        obs_id: obstacle,
        raw_hit: false,
        delta_lambda: 0.48,
        direction: new Vector3(1, 0, 0)
      }]);

      let currentTime = 0;
      system._nowSeconds = () => currentTime;
      RenderSystem3D.prototype._updateBumperHitFx.call(system, world);

      expect(system._bumperFxBursts).toHaveLength(0);
      expect(system._bumperFxGroup.children).toHaveLength(0);
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
      const expectedAngle = 0.25 / 0.6;
      const expectedX = 1.0 + (0.6 * Math.cos(expectedAngle));
      const expectedY = 2.0 + (0.6 * Math.sin(expectedAngle));

      expect(marker.visible).toBe(true);
      expect(marker.material).toBe(system.knotMarkerMaterial);
      expect(marker.position.x).toBeCloseTo(expectedX, 6);
      expect(marker.position.y).toBeCloseTo(expectedY, 6);
      expect(marker.position.z).toBeCloseTo(0.0, 6);
      expect(marker.scale.x).toBeCloseTo(0.002, 8);
      expect(marker.scale.y).toBeCloseTo(0.002, 8);
      expect(marker.scale.z).toBeCloseTo(0.002, 8);
    } finally {
      disposeRenderSystemStub(system);
    }
  });

  test('renders hybrid wrap visuals in the spool local plane after spool tilt', () => {
    const system = createRenderSystemStub();

    try {
      const world = new World();
      world.setResource('enableLayering', true);

      const spool = world.createEntity();
      const center = new Vector3(1.0, 2.0, 0.0);
      const tilt = new Vector3(1.0, 0.0, 0.0);
      const orientation = new OrientationComponent();
      orientation.quaternion.setFromAxisAngle(tilt, Math.PI / 4);

      world.addComponent(spool, new PositionComponent(center.x, center.y, center.z));
      world.addComponent(spool, new RadiusComponent(0.5));
      world.addComponent(spool, new CableLinkComponent(
        center.x,
        center.y,
        center.z,
        orientation.quaternion.clone(),
        null,
        new Vector3(0, 0, 1),
      ));
      world.addComponent(spool, orientation);

      const anchor = world.createEntity();
      const jointId = world.createEntity();
      const attachmentLocal = new Vector3(0.6, 0.0, 0.0);
      const attachmentWorld = center.clone().add(orientation.quaternion.transformVector(attachmentLocal));
      world.addComponent(
        jointId,
        new CableJointComponent(
          spool,
          anchor,
          0.2,
          attachmentWorld,
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
      expect(system.wrapArcs).toHaveLength(1);

      const expectedAngle = 0.25 / 0.6;
      const expectedLocal = new Vector3(
        0.6 * Math.cos(expectedAngle),
        0.6 * Math.sin(expectedAngle),
        0.0,
      );
      const expectedWorld = center.clone().add(orientation.quaternion.transformVector(expectedLocal));
      const marker = system.knotMarkers[0];

      expect(marker.position.x).toBeCloseTo(expectedWorld.x, 6);
      expect(marker.position.y).toBeCloseTo(expectedWorld.y, 6);
      expect(marker.position.z).toBeCloseTo(expectedWorld.z, 6);
      expect(Math.abs(marker.position.z)).toBeGreaterThan(0.05);

      const arc = system.wrapArcs[0];
      const positions = arc.geometry.attributes.position;
      let sawOutOfPlanePoint = false;
      for (let i = 0; i < positions.count; i += 1) {
        if (Math.abs(positions.getZ(i)) > 0.05) {
          sawOutOfPlanePoint = true;
          break;
        }
      }
      expect(sawOutOfPlanePoint).toBe(true);
    } finally {
      disposeRenderSystemStub(system);
    }
  });
});

describe('RenderSystem3D cable sag', () => {
  test('renders a floating force sign for each cable joint', () => {
    const system = createRenderSystemStub();

    try {
      const world = new World();
      world.setResource('gravity', new Vector3(0.0, 0.0, -9.81));

      const jointId = world.createEntity();
      const joint = new CableJointComponent(
        world.createEntity(),
        world.createEntity(),
        1.0,
        new Vector3(0.0, 0.0, 0.2),
        new Vector3(1.0, 0.0, 0.2)
      );
      joint.constraintForceMagnitude = 12.345;
      world.addComponent(jointId, joint);
      world.addComponent(jointId, new RenderableComponent('line', '#ffd34d'));

      const pathId = world.createEntity();
      world.addComponent(
        pathId,
        new CablePathComponent(
          world,
          [jointId],
          ['attachment', 'attachment'],
          [false, false],
          1e6,
          [0.0, 0.0],
          0.0
        )
      );

      RenderSystem3D.prototype._syncCable.call(system, world);

      expect(system.forceSigns).toHaveLength(1);
      const sign = system.forceSigns[0];
      expect(sign.visible).toBe(true);
      expect(sign.userData.text).toBe('12.3 N');
      expect(sign.position.x).toBeCloseTo(0.5, 6);
      expect(sign.position.y).toBeCloseTo(0.0, 6);
      expect(sign.position.z).toBeCloseTo(0.224, 6);
    } finally {
      disposeRenderSystemStub(system);
    }
  });

  test('hides force signs when the force overlay resource is disabled', () => {
    const system = createRenderSystemStub();

    try {
      const world = new World();
      world.setResource('showConstraintForces', false);

      const jointId = world.createEntity();
      const joint = new CableJointComponent(
        world.createEntity(),
        world.createEntity(),
        1.0,
        new Vector3(0.0, 0.0, 0.2),
        new Vector3(1.0, 0.0, 0.2)
      );
      joint.constraintForceMagnitude = 12.345;
      world.addComponent(jointId, joint);
      world.addComponent(jointId, new RenderableComponent('line', '#ffd34d'));

      const pathId = world.createEntity();
      world.addComponent(
        pathId,
        new CablePathComponent(
          world,
          [jointId],
          ['attachment', 'attachment'],
          [false, false],
          1e6,
          [0.0, 0.0],
          0.0
        )
      );

      RenderSystem3D.prototype._syncCable.call(system, world);
      expect(system.forceSigns).toHaveLength(0);

      world.setResource('showConstraintForces', true);
      RenderSystem3D.prototype._syncCable.call(system, world);
      expect(system.forceSigns).toHaveLength(1);
      expect(system.forceSigns[0].visible).toBe(true);

      world.setResource('showConstraintForces', false);
      RenderSystem3D.prototype._syncCable.call(system, world);
      expect(system.forceSigns[0].visible).toBe(false);
    } finally {
      disposeRenderSystemStub(system);
    }
  });

  test('renders slack cable joints as a sagging catenary polyline', () => {
    const system = createRenderSystemStub();

    try {
      const world = new World();
      world.setResource('gravity', new Vector3(0.0, 0.0, -9.81));

      const jointId = world.createEntity();
      world.addComponent(
        jointId,
        new CableJointComponent(
          world.createEntity(),
          world.createEntity(),
          1.2,
          new Vector3(0.0, 0.0, 0.0),
          new Vector3(1.0, 0.0, 0.0)
        )
      );
      world.addComponent(jointId, new RenderableComponent('line', '#ffd34d'));

      const pathId = world.createEntity();
      world.addComponent(
        pathId,
        new CablePathComponent(
          world,
          [jointId],
          ['attachment', 'attachment'],
          [false, false],
          1e6,
          [0.0, 0.0],
          0.0
        )
      );

      RenderSystem3D.prototype._syncCable.call(system, world);

      expect(system.jointLines).toHaveLength(1);
      const line = system.jointLines[0];
      const positions = line.geometry.attributes.position;
      const midpointIndex = Math.floor(positions.count / 2);

      expect(line.visible).toBe(true);
      expect(positions.count).toBeGreaterThan(2);
      expect(positions.getX(0)).toBeCloseTo(0.0, 6);
      expect(positions.getY(0)).toBeCloseTo(0.0, 6);
      expect(positions.getZ(0)).toBeCloseTo(0.0, 6);
      expect(positions.getX(positions.count - 1)).toBeCloseTo(1.0, 6);
      expect(positions.getY(positions.count - 1)).toBeCloseTo(0.0, 6);
      expect(positions.getZ(positions.count - 1)).toBeCloseTo(0.0, 6);
      expect(positions.getX(midpointIndex)).toBeCloseTo(0.5, 2);
      expect(positions.getY(midpointIndex)).toBeCloseTo(0.0, 6);
      expect(positions.getZ(midpointIndex)).toBeLessThan(-0.05);
    } finally {
      disposeRenderSystemStub(system);
    }
  });

  test('keeps taut cable joints straight', () => {
    const system = createRenderSystemStub();

    try {
      const world = new World();
      world.setResource('gravity', new Vector3(0.0, 0.0, -9.81));

      const jointId = world.createEntity();
      world.addComponent(
        jointId,
        new CableJointComponent(
          world.createEntity(),
          world.createEntity(),
          1.0,
          new Vector3(0.0, 0.0, 0.2),
          new Vector3(1.0, 0.0, 0.2)
        )
      );
      world.addComponent(jointId, new RenderableComponent('line', '#ffd34d'));

      const pathId = world.createEntity();
      world.addComponent(
        pathId,
        new CablePathComponent(
          world,
          [jointId],
          ['attachment', 'attachment'],
          [false, false],
          1e6,
          [0.0, 0.0],
          0.0
        )
      );

      RenderSystem3D.prototype._syncCable.call(system, world);

      const line = system.jointLines[0];
      const positions = line.geometry.attributes.position;
      const midpointIndex = Math.floor(positions.count / 2);

      expect(positions.getX(midpointIndex)).toBeCloseTo(0.5, 6);
      expect(positions.getY(midpointIndex)).toBeCloseTo(0.0, 6);
      expect(positions.getZ(midpointIndex)).toBeCloseTo(0.2, 6);
    } finally {
      disposeRenderSystemStub(system);
    }
  });
});

describe('RenderSystem3D rigid group overlays', () => {
  test('does not leave trailing vertices at the origin when drawing rigid group lines', () => {
    const system = createRenderSystemStub();

    try {
      const world = new World();

      const a = world.createEntity();
      world.addComponent(a, new PositionComponent(-0.25, 0.1, 0.1));

      const b = world.createEntity();
      world.addComponent(b, new PositionComponent(0.25, 0.1, 0.1));

      const rigidGroup = world.createEntity();
      world.addComponent(rigidGroup, new RigidGroupComponent([a, b], 1.0));
      world.addComponent(rigidGroup, new RenderableComponent('line', '#55ff88'));

      RenderSystem3D.prototype._syncRigidGroups.call(system, world);

      expect(system.rigidGroupLines).toHaveLength(1);
      const line = system.rigidGroupLines[0];
      const positions = line.geometry.attributes.position;
      const midpointIndex = Math.floor(positions.count / 2);

      expect(line.visible).toBe(true);
      expect(positions.getX(0)).toBeCloseTo(-0.25, 6);
      expect(positions.getY(0)).toBeCloseTo(0.1, 6);
      expect(positions.getZ(0)).toBeCloseTo(0.1, 6);
      expect(positions.getX(midpointIndex)).toBeCloseTo(0.0, 6);
      expect(positions.getY(midpointIndex)).toBeCloseTo(0.1, 6);
      expect(positions.getZ(midpointIndex)).toBeCloseTo(0.1, 6);
      expect(positions.getX(positions.count - 1)).toBeCloseTo(0.25, 6);
      expect(positions.getY(positions.count - 1)).toBeCloseTo(0.1, 6);
      expect(positions.getZ(positions.count - 1)).toBeCloseTo(0.1, 6);
    } finally {
      disposeRenderSystemStub(system);
    }
  });
});
