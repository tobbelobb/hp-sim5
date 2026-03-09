import * as THREE from 'three';

jest.mock('three/addons/controls/OrbitControls.js', () => ({
  OrbitControls: class OrbitControls {}
}));

import { RenderSystem3D } from '../../../src/js/cable_joints_3d/render_system_3d.js';
import { World } from '../../../src/js/cable_joints_3d/ecs.js';
import { ExtruderComponent } from '../../../examples/js/slideprinter/slideprinter_common.js';

function createCompatStub() {
  const system = Object.create(RenderSystem3D.prototype);
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
  system.drawnExtrusionCount = 0;
  system.referenceMaterial = new THREE.LineBasicMaterial({ color: '#1e90ff' });
  system.referenceLines = new THREE.LineSegments(new THREE.BufferGeometry(), system.referenceMaterial);
  system.extrusionPointsMaterial = new THREE.PointsMaterial({ size: 7, sizeAttenuation: false, vertexColors: true });
  system.extrusionPoints = new THREE.Points(new THREE.BufferGeometry(), system.extrusionPointsMaterial);
  system.positionTraceMaterial = new THREE.PointsMaterial({ color: '#ffffff', size: 6, sizeAttenuation: false });
  system.positionTracePointsObject = new THREE.Points(new THREE.BufferGeometry(), system.positionTraceMaterial);
  system.positionTraceMarkerMaterial = new THREE.PointsMaterial({ color: '#2dd4bf', size: 8, sizeAttenuation: false });
  system.positionTraceMarkersObject = new THREE.Points(new THREE.BufferGeometry(), system.positionTraceMarkerMaterial);
  system._updatePointObject = RenderSystem3D.prototype._updatePointObject;
  system._syncReferencePaths = RenderSystem3D.prototype._syncReferencePaths;
  system._syncExtrusions = RenderSystem3D.prototype._syncExtrusions;
  system._syncPositionTrace = RenderSystem3D.prototype._syncPositionTrace;
  system._syncPositionTraceMarkers = RenderSystem3D.prototype._syncPositionTraceMarkers;
  system.setReferencePaths = RenderSystem3D.prototype.setReferencePaths;
  system.setPositionTraceEnabled = RenderSystem3D.prototype.setPositionTraceEnabled;
  system.addPositionTraceMarker = RenderSystem3D.prototype.addPositionTraceMarker;
  system.clearPositionTrace = RenderSystem3D.prototype.clearPositionTrace;
  system.clearPositionTraceMarkers = RenderSystem3D.prototype.clearPositionTraceMarkers;
  system.clearExtrusions = RenderSystem3D.prototype.clearExtrusions;
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
}

describe('RenderSystem3D hp-sim compatibility helpers', () => {
  test('syncs reference segments into a Three.js line object', () => {
    const system = createCompatStub();

    try {
      system.setReferencePaths([
        {
          start: [0, 0, 0],
          end: [1, 1, 0],
        },
      ], {
        visible: true,
        color: '#ff7a18',
      });
      system._syncReferencePaths();

      expect(system.referenceLines.visible).toBe(true);
      expect(system.referenceMaterial.color.getHexString()).toBe('ff7a18');
      expect(system.referenceLines.geometry.getAttribute('position').count).toBe(2);
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
        pos: [0.25, 0.75, 0.0],
        color: '#ffaa00',
        length: 0.02,
      });
      extruder.centerPos = { x: 0.5, y: 0.6, z: 0.0 };
      world.addComponent(extruderEntity, extruder);

      system._syncExtrusions(world);
      expect(system.drawnExtrusionCount).toBe(1);
      expect(system.extrusionPoints.geometry.getAttribute('position').count).toBe(1);

      system.setPositionTraceEnabled(true);
      system._syncPositionTrace(world);
      system.addPositionTraceMarker(0.1, 0.2, 'A');

      expect(system.positionTracePoints).toHaveLength(1);
      expect(system.positionTracePointsObject.visible).toBe(true);
      expect(system.positionTraceMarkersObject.visible).toBe(true);
    } finally {
      disposeCompatStub(system);
    }
  });
});
