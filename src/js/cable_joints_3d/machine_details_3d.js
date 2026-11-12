import * as THREE from 'three';

import {
  PositionComponent,
  RenderableComponent,
  RigidBodyComponent,
  RigidGroupComponent,
  SceneEntityInfoComponent,
} from './ecs.js';
import { CableJointComponent } from './cable_joints_core.js';

// Slim structural details at the simulated points: the rigid frame as tubes
// and eyelet rings where lines pass through pinholes and anchors. Everything
// is thin so bearings, wraps and the print stay in view.

const FRAME_TUBE_RADIUS = 0.004;
const EYELET_RING_RADIUS = 0.0028;
const EYELET_TUBE_RADIUS = 0.0008;
const ANCHOR_RING_RADIUS = 0.0045;
const ANCHOR_TUBE_RADIUS = 0.0012;

const Y_AXIS = new THREE.Vector3(0, 1, 0);
const Z_AXIS = new THREE.Vector3(0, 0, 1);

class PartPool {
  constructor(parent, geometry, material, { colored = false } = {}) {
    this.parent = parent;
    this.geometry = geometry;
    this.material = material;
    this.colored = colored;
    this.count = 0;
    this.capacity = 0;
    this.mesh = null;
    this._allocate(32);
  }

  _allocate(capacity) {
    const previous = this.mesh;
    const mesh = new THREE.InstancedMesh(this.geometry, this.material, capacity);
    mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    if (this.colored) {
      mesh.instanceColor = new THREE.InstancedBufferAttribute(new Float32Array(capacity * 3), 3);
      mesh.instanceColor.setUsage(THREE.DynamicDrawUsage);
    }
    mesh.frustumCulled = false;
    mesh.count = 0;
    if (previous) {
      mesh.instanceMatrix.array.set(previous.instanceMatrix.array);
      if (this.colored) {
        mesh.instanceColor.array.set(previous.instanceColor.array);
      }
      this.parent.remove(previous);
      previous.dispose();
    }
    this.parent.add(mesh);
    this.mesh = mesh;
    this.capacity = capacity;
  }

  begin() {
    this.count = 0;
  }

  add(matrix, color = null) {
    if (this.count >= this.capacity) {
      this._allocate(this.capacity * 2);
    }
    this.mesh.setMatrixAt(this.count, matrix);
    if (color) {
      this.mesh.setColorAt(this.count, color);
    }
    this.count += 1;
  }

  end() {
    this.mesh.count = this.count;
    this.mesh.instanceMatrix.needsUpdate = true;
    if (this.colored) {
      this.mesh.instanceColor.needsUpdate = true;
    }
  }

  dispose() {
    this.parent.remove(this.mesh);
    this.mesh.dispose();
  }
}

export class MachineDetails3D {
  constructor(parent, { collectEdges }) {
    this.collectEdges = collectEdges;
    this.group = new THREE.Group();
    this.group.name = 'machine-details';
    parent.add(this.group);

    this.materials = {
      frame: new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.35, metalness: 0.9 }),
      eyelet: new THREE.MeshStandardMaterial({ color: 0xdfe2e6, roughness: 0.4, metalness: 1.0 }),
    };
    this.geometries = {
      cylinder: new THREE.CylinderGeometry(1, 1, 1, 12, 1, false),
      eyelet: new THREE.TorusGeometry(EYELET_RING_RADIUS, EYELET_TUBE_RADIUS, 8, 24),
      anchorEyelet: new THREE.TorusGeometry(ANCHOR_RING_RADIUS, ANCHOR_TUBE_RADIUS, 8, 24),
    };
    this.pools = {
      frame: new PartPool(this.group, this.geometries.cylinder, this.materials.frame, { colored: true }),
      eyelet: new PartPool(this.group, this.geometries.eyelet, this.materials.eyelet),
      anchorEyelet: new PartPool(this.group, this.geometries.anchorEyelet, this.materials.eyelet),
    };

    this._matrix = new THREE.Matrix4();
    this._quaternion = new THREE.Quaternion();
    this._position = new THREE.Vector3();
    this._scale = new THREE.Vector3();
    this._a = new THREE.Vector3();
    this._b = new THREE.Vector3();
    this._d = new THREE.Vector3();
    this._color = new THREE.Color();
    this._tint = new THREE.Color();
    this.theme = null;
    this._lineDirections = new Map();
    this._eyeletOrientations = new Map();
    this._handled = new Set();
  }

  // Pinholes and anchors drawn here as eyelets; the generic renderer skips them.
  handles(entityId) {
    return this._handled.has(entityId);
  }

  update(world) {
    for (const pool of Object.values(this.pools)) {
      pool.begin();
    }
    this._handled.clear();

    this._addFrames(world);
    this._addEyelets(world);

    for (const pool of Object.values(this.pools)) {
      pool.end();
    }
  }

  // Sum of unit directions of the line spans meeting each entity, sign-aligned
  // so a line passing straight through adds up instead of cancelling.
  _collectLineDirections(world) {
    const directions = this._lineDirections;
    for (const direction of directions.values()) {
      direction.set(0, 0, 0);
    }
    for (const jointId of world.query([CableJointComponent])) {
      const joint = world.getComponent(jointId, CableJointComponent);
      const a = joint.attachmentPointA_world;
      const b = joint.attachmentPointB_world;
      this._d.set(b.x - a.x, b.y - a.y, b.z - a.z);
      if (this._d.lengthSq() < 1e-12) {
        continue;
      }
      this._d.normalize();
      for (const entityId of [joint.entityA, joint.entityB]) {
        let direction = directions.get(entityId);
        if (!direction) {
          direction = new THREE.Vector3();
          directions.set(entityId, direction);
        }
        direction.add(direction.dot(this._d) < 0 ? this._d.clone().negate() : this._d);
      }
    }
  }

  _span(pool, a, b, radius, color = null) {
    this._d.subVectors(b, a);
    const length = this._d.length();
    if (length < 1e-6) {
      return;
    }
    this._quaternion.setFromUnitVectors(Y_AXIS, this._d.divideScalar(length));
    this._position.addVectors(a, b).multiplyScalar(0.5);
    this._scale.set(radius, length, radius);
    this._matrix.compose(this._position, this._quaternion, this._scale);
    pool.add(this._matrix, color);
  }

  setTheme(theme) {
    this.theme = theme;
    this.materials.eyelet.color.set(theme.eyelet);
  }

  // The theme's frame colour with the machine tint mixed in; a null theme
  // colour uses the machine tint itself.
  _frameColor(world, groupId) {
    const tint = world.getComponent(groupId, RenderableComponent)?.color;
    const hasTint = typeof tint === 'string' && tint.length > 0;
    if (this.theme.frame === null) {
      return this._color.set(hasTint ? tint : '#c4c8ce');
    }
    this._color.set(this.theme.frame);
    if (hasTint) {
      this._color.lerp(this._tint.set(tint), this.theme.frameTint);
    }
    return this._color;
  }

  // Tubes along the authored render edges, stopping short of eyelet rings.
  _addFrames(world) {
    for (const ComponentClass of [RigidBodyComponent, RigidGroupComponent]) {
      for (const groupId of world.query([ComponentClass])) {
        const group = world.getComponent(groupId, ComponentClass);
        const members = Array.isArray(group?.members) ? group.members : [];
        const positions = members.map((id) => world.getComponent(id, PositionComponent)?.pos ?? null);
        const color = this._frameColor(world, groupId);
        for (const [i, j] of this.collectEdges(members.length, group?.renderSegments)) {
          if (!positions[i] || !positions[j]) {
            continue;
          }
          const start = this._a.copy(positions[i]);
          const end = this._b.copy(positions[j]);
          this._trimAtEyelet(world, members[i], start, end);
          this._trimAtEyelet(world, members[j], end, start);
          this._span(this.pools.frame, start, end, FRAME_TUBE_RADIUS, color);
        }
      }
    }
  }

  // Moves `point` towards `other` by the outer radius of the eyelet at point.
  _trimAtEyelet(world, entityId, point, other) {
    if (!this._isEyelet(world, entityId)) {
      return;
    }
    const anchor = world.getComponent(entityId, SceneEntityInfoComponent).tags.includes('Anchor');
    const outer = anchor ? ANCHOR_RING_RADIUS + ANCHOR_TUBE_RADIUS : EYELET_RING_RADIUS + EYELET_TUBE_RADIUS;
    this._d.subVectors(other, point);
    const length = this._d.length();
    if (length > outer * 2) {
      point.addScaledVector(this._d, outer / length);
    }
  }

  reset() {
    this._eyeletOrientations.clear();
  }

  _isEyelet(world, entityId) {
    const tags = world.getComponent(entityId, SceneEntityInfoComponent)?.tags ?? [];
    return tags.includes('Anchor') || tags.includes('Pinhole') || tags.includes('Eyelet');
  }

  _addEyelets(world) {
    let directionsCollected = false;
    for (const entityId of world.query([SceneEntityInfoComponent, PositionComponent])) {
      if (!this._isEyelet(world, entityId)) {
        continue;
      }
      const anchor = world.getComponent(entityId, SceneEntityInfoComponent).tags.includes('Anchor');
      this._handled.add(entityId);
      const pos = world.getComponent(entityId, PositionComponent).pos;
      // The torus lies in its XY plane: its hole faces +Z, along the line.
      // The orientation is taken once, so rings never snap as lines move.
      let orientation = this._eyeletOrientations.get(entityId);
      if (!orientation) {
        if (!directionsCollected) {
          this._collectLineDirections(world);
          directionsCollected = true;
        }
        orientation = new THREE.Quaternion();
        const direction = this._lineDirections.get(entityId);
        if (direction && direction.lengthSq() > 1e-12) {
          orientation.setFromUnitVectors(Z_AXIS, this._d.copy(direction).normalize());
          this._eyeletOrientations.set(entityId, orientation);
        }
      }
      this._quaternion.copy(orientation);
      this._position.set(pos.x, pos.y, pos.z);
      this._matrix.compose(this._position, this._quaternion, this._scale.setScalar(1));
      (anchor ? this.pools.anchorEyelet : this.pools.eyelet).add(this._matrix);
    }
  }

  dispose() {
    for (const pool of Object.values(this.pools)) {
      pool.dispose();
    }
    for (const geometry of Object.values(this.geometries)) {
      geometry.dispose();
    }
    for (const material of Object.values(this.materials)) {
      material.dispose();
    }
    this.group.parent?.remove(this.group);
  }
}
