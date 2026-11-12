import * as THREE from 'three';
import { LineSegments2 } from 'three/addons/lines/LineSegments2.js';
import { LineSegmentsGeometry } from 'three/addons/lines/LineSegmentsGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';

// All cable spans and wraps of a frame in one antialiased screen-space-width
// line draw. Buffers grow on demand and are rewritten in place each frame.

export class CableLineBatch {
  constructor(parent, { linewidth }) {
    this.parent = parent;
    this.material = new LineMaterial({
      linewidth,
      vertexColors: true,
      worldUnits: false,
    });
    this.object = new LineSegments2(undefined, this.material);
    this.object.frustumCulled = false;
    this.object.renderOrder = 870;
    parent.add(this.object);
    this.count = 0;
    this.capacity = 0;
    this._color = new THREE.Color();
    this._allocate(4096);
  }

  _allocate(capacity) {
    const positions = new Float32Array(capacity * 6);
    const colors = new Float32Array(capacity * 6);
    if (this.positions) {
      positions.set(this.positions);
      colors.set(this.colors);
    }
    this.positions = positions;
    this.colors = colors;
    this.capacity = capacity;
    this._replaceGeometry();

    const positionBuffer = new THREE.InstancedInterleavedBuffer(positions, 6, 1);
    positionBuffer.setUsage(THREE.DynamicDrawUsage);
    this.geometry.setAttribute('instanceStart', new THREE.InterleavedBufferAttribute(positionBuffer, 3, 0));
    this.geometry.setAttribute('instanceEnd', new THREE.InterleavedBufferAttribute(positionBuffer, 3, 3));
    const colorBuffer = new THREE.InstancedInterleavedBuffer(colors, 6, 1);
    colorBuffer.setUsage(THREE.DynamicDrawUsage);
    this.geometry.setAttribute('instanceColorStart', new THREE.InterleavedBufferAttribute(colorBuffer, 3, 0));
    this.geometry.setAttribute('instanceColorEnd', new THREE.InterleavedBufferAttribute(colorBuffer, 3, 3));
    this.positionBuffer = positionBuffer;
    this.colorBuffer = colorBuffer;
  }

  // A fresh geometry per allocation: the renderer caches an instanced
  // geometry's maximum instance count, so growing buffers in place would
  // keep drawing only the old capacity.
  _replaceGeometry() {
    const previous = this.geometry;
    this.geometry = new LineSegmentsGeometry();
    this.object.geometry = this.geometry;
    previous?.dispose();
  }

  setResolution(width, height) {
    this.material.resolution.set(width, height);
  }

  begin() {
    this.count = 0;
  }

  // Appends the polyline stored as xyz triples in points[0 .. pointCount*3).
  addPolyline(points, pointCount, color) {
    const segments = pointCount - 1;
    if (segments < 1) {
      return;
    }
    if (this.count + segments > this.capacity) {
      let capacity = this.capacity;
      while (this.count + segments > capacity) {
        capacity *= 2;
      }
      this._allocate(capacity);
    }
    this._color.set(color);
    const { r, g, b } = this._color;
    const positions = this.positions;
    const colors = this.colors;
    for (let i = 0; i < segments; i += 1) {
      const out = (this.count + i) * 6;
      const p = i * 3;
      positions[out] = points[p];
      positions[out + 1] = points[p + 1];
      positions[out + 2] = points[p + 2];
      positions[out + 3] = points[p + 3];
      positions[out + 4] = points[p + 4];
      positions[out + 5] = points[p + 5];
      colors[out] = r;
      colors[out + 1] = g;
      colors[out + 2] = b;
      colors[out + 3] = r;
      colors[out + 4] = g;
      colors[out + 5] = b;
    }
    this.count += segments;
  }

  end() {
    this.geometry.instanceCount = this.count;
    this.positionBuffer.addUpdateRange(0, this.count * 6);
    this.positionBuffer.needsUpdate = true;
    this.colorBuffer.addUpdateRange(0, this.count * 6);
    this.colorBuffer.needsUpdate = true;
  }

  dispose() {
    this.parent.remove(this.object);
    this.geometry.dispose();
    this.material.dispose();
  }
}
