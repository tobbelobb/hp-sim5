import * as THREE from 'three';

jest.mock('three/addons/controls/OrbitControls.js', () => ({
  OrbitControls: class OrbitControls {}
}));

import { RenderSystem3D } from '../../../src/js/cable_joints_3d/render_system_3d.js';
import {
  World,
  PositionComponent,
  RadiusComponent,
  RenderableComponent,
  RigidGroupComponent,
} from '../../../src/js/cable_joints_3d/ecs.js';
import { ExtruderComponent } from '../../../hp-sim-3d/app/hangprinter_extruder.js';

function createCompatStub(canvasOverrides = {}) {
  const system = Object.create(RenderSystem3D.prototype);
  system.scene = new THREE.Scene();
  system.canvas = {
    clientWidth: 640,
    clientHeight: 480,
    width: 640,
    height: 480,
    getBoundingClientRect() {
      return {
        left: 0,
        top: 0,
        width: this.clientWidth,
        height: this.clientHeight,
      };
    },
    ...canvasOverrides,
  };
  system.referenceColor = '#1e90ff';
  system.referencePaths = [];
  system.referenceRequestedVisible = false;
  system.referenceVisible = false;
  system.referenceDirty = false;
  system.positionTraceEnabled = false;
  system.positionTraceColor = '#ffffff';
  system.positionTraceRadiusPx = 1.25;
  system.positionTracePoints = [];
  system.positionTraceMarkers = [];
  system.drawnPositionTraceCount = 0;
  system.drawnPositionTraceMarkerCount = 0;
  system.drawnExtrusionCount = 0;
  system.requestRender = jest.fn();
  system.renderer = { render: jest.fn() };
  system._animationLoopActive = false;
  system._requestRenderHandle = null;
  system._lastWorld = null;
  system.referenceMaterial = new THREE.LineBasicMaterial({ color: '#1e90ff' });
  system.referenceLines = new THREE.LineSegments(new THREE.BufferGeometry(), system.referenceMaterial);
  system.extrusionPointsMaterial = new THREE.PointsMaterial({ size: 7, sizeAttenuation: false, vertexColors: true });
  system.extrusionPoints = new THREE.Points(new THREE.BufferGeometry(), system.extrusionPointsMaterial);
  system.positionTraceMaterial = new THREE.PointsMaterial({ color: '#ffffff', size: 6, sizeAttenuation: false });
  system.positionTracePointsObject = new THREE.Points(new THREE.BufferGeometry(), system.positionTraceMaterial);
  system.positionTraceMarkerMaterial = new THREE.PointsMaterial({ color: '#2dd4bf', size: 8, sizeAttenuation: false });
  system.positionTraceMarkersObject = new THREE.Points(new THREE.BufferGeometry(), system.positionTraceMarkerMaterial);
  system.navigationCursorVisible = false;
  system.navigationCursorMaterial = new THREE.LineBasicMaterial({ color: '#ffd34d' });
  system.navigationCursorGeometry = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(-1, 0, 0),
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, -1, 0),
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(0, 0, -1),
    new THREE.Vector3(0, 0, 1),
  ]);
  system.navigationCursorObject = new THREE.LineSegments(system.navigationCursorGeometry, system.navigationCursorMaterial);
  system.root = new THREE.Group();
  system.board = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), new THREE.MeshBasicMaterial());
  system.boardOutline = new THREE.LineLoop(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(-0.5, -0.5, 0),
      new THREE.Vector3(0.5, -0.5, 0),
      new THREE.Vector3(0.5, 0.5, 0),
      new THREE.Vector3(-0.5, 0.5, 0),
    ]),
    new THREE.LineBasicMaterial({ color: '#5f7492' })
  );
  system.boardOutline.frustumCulled = false;
  system.borderComponentClass = null;
  system.rigidGroupLines = [];
  system.camera = new THREE.PerspectiveCamera(48, 640 / 480, 0.01, 30.0);
  system.controls = null;
  system.viewScaleMultiplier = 1.0;
  system.viewOffsetX = 0.0;
  system.viewOffsetY = 0.0;
  system.viewOffsetZ = 0.0;
  system.orbitMinPolarAngle = 1e-6;
  system.orbitMaxPolarAngle = Math.PI - 1e-6;
  system.orbitRotateSpeed = 1.0;
  system._baseViewTarget = new THREE.Vector3(0.0, 0.0, 0.0);
  system._fixedCameraPosition = new THREE.Vector3();
  system._fixedCameraQuaternion = new THREE.Quaternion();
  system._cameraDistanceForScale = RenderSystem3D.prototype._cameraDistanceForScale;
  system._getViewTarget = RenderSystem3D.prototype._getViewTarget;
  system._clientToCanvasPixels = RenderSystem3D.prototype._clientToCanvasPixels;
  system.getViewPlaneMetrics = RenderSystem3D.prototype.getViewPlaneMetrics;
  system._buildOrbitOffset = RenderSystem3D.prototype._buildOrbitOffset;
  system._applyCameraFromViewTransform = RenderSystem3D.prototype._applyCameraFromViewTransform;
  system.rotateOrbitByPixels = RenderSystem3D.prototype.rotateOrbitByPixels;
  system._updatePointObject = RenderSystem3D.prototype._updatePointObject;
  system._syncReferencePaths = RenderSystem3D.prototype._syncReferencePaths;
  system._syncExtrusions = RenderSystem3D.prototype._syncExtrusions;
  system._syncExtruder = RenderSystem3D.prototype._syncExtruder;
  system._syncPositionTrace = RenderSystem3D.prototype._syncPositionTrace;
  system._syncPositionTraceMarkers = RenderSystem3D.prototype._syncPositionTraceMarkers;
  system._syncBoard = RenderSystem3D.prototype._syncBoard;
  system._computePrintSurfaceHalfExtent = RenderSystem3D.prototype._computePrintSurfaceHalfExtent;
  system._syncRigidGroups = RenderSystem3D.prototype._syncRigidGroups;
  system._ensureLineCapacity = RenderSystem3D.prototype._ensureLineCapacity;
  system._createLine = RenderSystem3D.prototype._createLine;
  system._hideLines = RenderSystem3D.prototype._hideLines;
  system.setReferencePaths = RenderSystem3D.prototype.setReferencePaths;
  system.setPositionTraceEnabled = RenderSystem3D.prototype.setPositionTraceEnabled;
  system.setNavigationCursorVisible = RenderSystem3D.prototype.setNavigationCursorVisible;
  system.addPositionTraceMarker = RenderSystem3D.prototype.addPositionTraceMarker;
  system.clearPositionTrace = RenderSystem3D.prototype.clearPositionTrace;
  system.clearPositionTraceMarkers = RenderSystem3D.prototype.clearPositionTraceMarkers;
  system.clearExtrusions = RenderSystem3D.prototype.clearExtrusions;
  system.getCameraPlaneNormal = RenderSystem3D.prototype.getCameraPlaneNormal;
  system.projectClientToPlane = RenderSystem3D.prototype.projectClientToPlane;
  system.projectCanvasToPlane = RenderSystem3D.prototype.projectCanvasToPlane;
  system.projectClientToSim = RenderSystem3D.prototype.projectClientToSim;
  system.projectCanvasToSim = RenderSystem3D.prototype.projectCanvasToSim;
  system._projectCanvasToSim = RenderSystem3D.prototype._projectCanvasToSim;
  system._projectCanvasToPlane = RenderSystem3D.prototype._projectCanvasToPlane;
  system._baseCameraDistance = 2.2;
  system.orbitAzimuth = -Math.PI * 0.25;
  system.orbitPolar = 1.05;
  system._raycaster = new THREE.Raycaster();
  system._rayNdc = new THREE.Vector2();
  system._rayPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  system._rayPlaneNormal = new THREE.Vector3(0, 0, 1);
  system._rayPlanePoint = new THREE.Vector3();
  system._rayHit = new THREE.Vector3();
  system.scene.add(system.referenceLines);
  system.scene.add(system.extrusionPoints);
  system.scene.add(system.positionTracePointsObject);
  system.scene.add(system.positionTraceMarkersObject);
  system.scene.add(system.root);
  system.scene.add(system.board);
  system.scene.add(system.boardOutline);
  system.scene.add(system.navigationCursorObject);
  system._applyCameraFromViewTransform();
  return system;
}

function disposeCompatStub(system) {
  system.referenceLines.geometry.dispose();
  system.referenceMaterial.dispose();
  system.extrusionPoints.geometry.dispose();
  system.extrusionPointsMaterial.dispose();
  system.positionTracePointsObject.geometry.dispose();
  system.positionTraceMaterial.dispose();
  system.positionTraceMarkersObject.geometry.dispose();
  system.positionTraceMarkerMaterial.dispose();
  system.navigationCursorObject.geometry.dispose();
  system.navigationCursorMaterial.dispose();
  system.board.geometry.dispose();
  system.board.material.dispose();
  system.boardOutline.geometry.dispose();
  system.boardOutline.material.dispose();
  for (const line of system.rigidGroupLines) {
    line.geometry.dispose();
    line.material.dispose();
  }
}

describe('RenderSystem3D hp-sim compatibility helpers', () => {
  test('syncs reference segments into a Three.js line object', () => {
    const system = createCompatStub();

    try {
      system.setReferencePaths([
        {
          start: [0, 0, -0.001],
          end: [1, 1, 0.002],
        },
      ], {
        visible: true,
        color: '#ff7a18',
      });
      system._syncReferencePaths();

      expect(system.referenceLines.visible).toBe(true);
      expect(system.referenceMaterial.color.getHexString()).toBe('ff7a18');
      expect(system.referenceLines.geometry.getAttribute('position').count).toBe(2);
      expect(system.referenceLines.geometry.getAttribute('position').getZ(0)).toBeCloseTo(-0.001, 6);
      expect(system.referenceLines.geometry.getAttribute('position').getZ(1)).toBeCloseTo(0.002, 6);
    } finally {
      disposeCompatStub(system);
    }
  });

  test('projects client coordinates using the canvas backing-store scale', () => {
    const system = createCompatStub({
      clientWidth: 640,
      clientHeight: 480,
      width: 960,
      height: 720,
    });

    try {
      const center = system.projectClientToSim(320, 240);
      const clientProjected = system.projectClientToSim(160, 120);
      const canvasProjected = system.projectCanvasToSim(240, 180);

      expect(center?.x).toBeCloseTo(0.0, 6);
      expect(center?.y).toBeCloseTo(0.0, 6);
      expect(clientProjected?.x).toBeCloseTo(canvasProjected?.x ?? Number.NaN, 6);
      expect(clientProjected?.y).toBeCloseTo(canvasProjected?.y ?? Number.NaN, 6);
    } finally {
      disposeCompatStub(system);
    }
  });

  test('builds extrusion and position-trace point clouds from the extruder component', () => {
    const system = createCompatStub();

    try {
      const world = new World();
      const extruderEntity = world.createEntity();
      const extruder = new ExtruderComponent();
      extruder.extrusions.push({
        pos: [0.25, 0.75, -0.001],
        color: '#ffaa00',
        length: 0.02,
      });
      extruder.centerPos = { x: 0.5, y: 0.6, z: -0.001 };
      extruder.tipPos = { x: 0.45, y: 0.55, z: -0.002 };
      world.addComponent(extruderEntity, extruder);

      system._syncExtrusions(world);
      expect(system.drawnExtrusionCount).toBe(1);
      expect(system.extrusionPoints.geometry.getAttribute('position').count).toBe(1);
      expect(system.extrusionPoints.geometry.getAttribute('position').getZ(0)).toBeCloseTo(-0.001, 6);

      system.setPositionTraceEnabled(true);
      system._syncPositionTrace(world);
      expect(system.positionTracePointsObject.geometry.getAttribute('position').getZ(0)).toBeCloseTo(-0.002, 6);

      extruder.centerPos = { x: 0.5, y: 0.6, z: -0.003 };
      extruder.tipPos = { x: 0.45, y: 0.55, z: -0.004 };
      system._syncPositionTrace(world);
      system.addPositionTraceMarker(0.1, 0.2, 'A');

      expect(system.positionTracePoints).toHaveLength(2);
      expect(system.positionTracePointsObject.geometry.getAttribute('position').getZ(1)).toBeCloseTo(-0.004, 6);
      expect(system.positionTracePointsObject.visible).toBe(true);
      expect(system.positionTraceMarkersObject.visible).toBe(true);
    } finally {
      disposeCompatStub(system);
    }
  });

  test('renders the extruder as a line from cold end to tip', () => {
    const system = createCompatStub();

    try {
      const world = new World();
      const extruderEntity = world.createEntity();
      const extruder = new ExtruderComponent();
      extruder.coldEndPos = { x: 0.1, y: 0.2, z: 0.3 };
      extruder.tipPos = { x: 0.4, y: 0.5, z: 0.6 };
      world.addComponent(extruderEntity, extruder);

      system._syncExtruder(world);

      expect(system.extruderLine).toBeDefined();
      expect(system.extruderLine.visible).toBe(true);
      const positions = system.extruderLine.geometry.getAttribute('position');
      expect(positions.getX(0)).toBeCloseTo(0.1, 6);
      expect(positions.getY(0)).toBeCloseTo(0.2, 6);
      expect(positions.getZ(0)).toBeCloseTo(0.3, 6);
      expect(positions.getX(positions.count - 1)).toBeCloseTo(0.4, 6);
      expect(positions.getY(positions.count - 1)).toBeCloseTo(0.5, 6);
      expect(positions.getZ(positions.count - 1)).toBeCloseTo(0.6, 6);
    } finally {
      disposeCompatStub(system);
    }
  });

  test('prefers live quality colors for extrusion points when available', () => {
    const system = createCompatStub();

    try {
      const world = new World();
      const extruderEntity = world.createEntity();
      const extruder = new ExtruderComponent();
      extruder.extrusions.push({
        pos: [0.25, 0.75, -0.001],
        color: '#ffaa00',
        qualityColor: '#ffffff',
        length: 0.02,
      });
      world.addComponent(extruderEntity, extruder);

      system._syncExtrusions(world);

      const colorAttr = system.extrusionPoints.geometry.getAttribute('color');
      expect(colorAttr.getX(0)).toBeCloseTo(1.0, 6);
      expect(colorAttr.getY(0)).toBeCloseTo(1.0, 6);
      expect(colorAttr.getZ(0)).toBeCloseTo(1.0, 6);
    } finally {
      disposeCompatStub(system);
    }
  });

  test('clearPositionTrace preserves stored trace history for redraw after view changes', () => {
    const system = createCompatStub();

    try {
      const world = new World();
      const extruderEntity = world.createEntity();
      const extruder = new ExtruderComponent();
      extruder.centerPos = { x: 0.5, y: 0.6, z: -0.001 };
      world.addComponent(extruderEntity, extruder);

      system.setPositionTraceEnabled(true);
      system._syncPositionTrace(world);
      extruder.centerPos = { x: 0.55, y: 0.65, z: -0.001 };
      system._syncPositionTrace(world);

      expect(system.positionTracePoints).toHaveLength(2);
      expect(system.positionTracePointsObject.geometry.getAttribute('position').count).toBe(2);

      system.clearPositionTrace({ keepMarkers: true });

      expect(system.positionTracePoints).toHaveLength(2);
      expect(system.drawnPositionTraceCount).toBe(0);
      expect(system.positionTracePointsObject.visible).toBe(false);

      system._syncPositionTrace(world);

      expect(system.positionTracePointsObject.visible).toBe(true);
      expect(system.positionTracePointsObject.geometry.getAttribute('position').count).toBe(2);
      expect(system.drawnPositionTraceCount).toBe(2);
    } finally {
      disposeCompatStub(system);
    }
  });

  test('skips position-trace marker uploads when marker data is unchanged', () => {
    const system = createCompatStub();

    try {
      system.addPositionTraceMarker(0.1, 0.2, 'A');
      const updateSpy = jest.spyOn(system, '_updatePointObject');

      system._syncPositionTraceMarkers();

      expect(updateSpy).not.toHaveBeenCalled();
      expect(system.drawnPositionTraceMarkerCount).toBe(1);
      updateSpy.mockRestore();
    } finally {
      disposeCompatStub(system);
    }
  });

  test('keeps the navigation cursor pinned to the current view target', () => {
    const system = createCompatStub();

    try {
      system.viewOffsetX = 0.22;
      system.viewOffsetY = -0.14;
      system.viewOffsetZ = 0.08;
      system.setNavigationCursorVisible(true);
      system._applyCameraFromViewTransform();

      expect(system.navigationCursorObject.visible).toBe(true);
      expect(system.navigationCursorObject.position.x).toBeCloseTo(0.22, 6);
      expect(system.navigationCursorObject.position.y).toBeCloseTo(-0.14, 6);
      expect(system.navigationCursorObject.position.z).toBeCloseTo(0.08, 6);
      expect(system.navigationCursorObject.scale.x).toBeGreaterThan(0.0);
      expect(system.navigationCursorObject.scale.x).toBeCloseTo(system.navigationCursorObject.scale.y, 6);
      expect(system.navigationCursorObject.scale.x).toBeCloseTo(system.navigationCursorObject.scale.z, 6);
    } finally {
      disposeCompatStub(system);
    }
  });

  test('renders rigid-group beam edges from renderSegments', () => {
    const system = createCompatStub();

    try {
      const world = new World();
      const points = [
        [0.0, 0.0],
        [0.2, 0.0],
        [0.1, 0.2],
        [-0.3, 0.0],
        [-0.1, 0.3],
        [0.1, 0.3],
        [0.3, 0.0],
      ];
      const members = points.map(([x, y]) => {
        const entityId = world.createEntity();
        world.addComponent(entityId, new PositionComponent(x, y, 0.0));
        world.addComponent(entityId, new RadiusComponent(0.01));
        return entityId;
      });
      const groupId = world.createEntity();
      world.addComponent(groupId, new RigidGroupComponent(members, 1.0, [[3, 4, 5, 6, 3]]));
      world.addComponent(groupId, new RenderableComponent('line', '#33ff66'));

      system._syncRigidGroups(world);

      expect(system.rigidGroupLines).toHaveLength(4);
      expect(system.rigidGroupLines.every((line) => line.visible)).toBe(true);
      expect(system.rigidGroupLines[0].material.color.getHexString()).toBe('33ff66');
      const positions = system.rigidGroupLines[0].geometry.getAttribute('position');
      const midpointIndex = Math.floor(positions.count / 2);
      expect(positions.getX(0)).toBeCloseTo(-0.3, 6);
      expect(positions.getY(0)).toBeCloseTo(0.0, 6);
      expect(positions.getZ(0)).toBeCloseTo(0.0, 6);
      expect(positions.getX(midpointIndex)).toBeCloseTo(-0.2, 6);
      expect(positions.getY(midpointIndex)).toBeCloseTo(0.15, 6);
      expect(positions.getZ(midpointIndex)).toBeCloseTo(0.0, 6);
      expect(positions.getX(positions.count - 1)).toBeCloseTo(-0.1, 6);
      expect(positions.getY(positions.count - 1)).toBeCloseTo(0.3, 6);
      expect(positions.getZ(positions.count - 1)).toBeCloseTo(0.0, 6);
    } finally {
      disposeCompatStub(system);
    }
  });

  test('keeps the print surface centered on the origin and square', () => {
    const system = createCompatStub();

    try {
      const world = new World();
      const left = world.createEntity();
      world.addComponent(left, new PositionComponent(-0.4, 0.1, 0.0));
      world.addComponent(left, new RadiusComponent(0.03));
      const right = world.createEntity();
      world.addComponent(right, new PositionComponent(0.2, 0.6, 0.0));
      world.addComponent(right, new RadiusComponent(0.02));

      system._syncBoard(world);

      expect(system.board.position.x).toBeCloseTo(0.0, 6);
      expect(system.board.position.y).toBeCloseTo(0.0, 6);
      expect(system.board.scale.x).toBeCloseTo(system.board.scale.y, 6);
      expect(system.boardOutline.position.x).toBeCloseTo(0.0, 6);
      expect(system.boardOutline.position.y).toBeCloseTo(0.0, 6);
      expect(system.board.scale.x).toBeGreaterThan(1.2);
    } finally {
      disposeCompatStub(system);
    }
  });

  test('updates the camera position when orbiting the 3D view', () => {
    const system = createCompatStub();

    try {
      const before = system.camera.position.clone();
      system.rotateOrbitByPixels(120, -40);
      const after = system.camera.position.clone();

      expect(after.distanceTo(before)).toBeGreaterThan(0.02);
      expect(system.camera.up.z).toBeCloseTo(1.0, 6);
      expect(system.camera.position.z).toBeGreaterThan(0.0);
    } finally {
      disposeCompatStub(system);
    }
  });

  test('dragging up tilts the camera downward', () => {
    const system = createCompatStub();

    try {
      const before = system.camera.position.z;
      system.rotateOrbitByPixels(0, -80);

      expect(system.camera.position.z).toBeLessThan(before);
    } finally {
      disposeCompatStub(system);
    }
  });

  test('allows orbiting below the z=0 print plane', () => {
    const system = createCompatStub();

    try {
      system.rotateOrbitByPixels(0, -420);

      expect(system.camera.position.z).toBeLessThan(0.0);
    } finally {
      disposeCompatStub(system);
    }
  });

  test('projects client rays onto a camera-parallel drag plane', () => {
    const system = createCompatStub();

    try {
      const planeNormal = system.getCameraPlaneNormal();
      const centerRayDirection = new THREE.Vector3(planeNormal.x, planeNormal.y, planeNormal.z);
      const planePoint = system.camera.position.clone().add(centerRayDirection.clone().multiplyScalar(1.25));
      const projected = system.projectClientToPlane(320, 240, planePoint, planeNormal);

      expect(projected).not.toBeNull();
      expect(projected.x).toBeCloseTo(planePoint.x, 6);
      expect(projected.y).toBeCloseTo(planePoint.y, 6);
      expect(projected.z).toBeCloseTo(planePoint.z, 6);
    } finally {
      disposeCompatStub(system);
    }
  });
});
