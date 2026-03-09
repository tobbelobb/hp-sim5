import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

import Vector3 from './vector3.js';
import {
  PositionComponent,
  RadiusComponent,
  OrientationComponent,
  layeringEnabled
} from './ecs.js';
import { RenderableComponent } from '../cable_joints/ecs.js';
import {
  CablePathComponent,
  CableJointComponent,
  CableLinkComponent
} from './cable_joints_core.js';

const EPSILON = 1e-9;
const ARC_SEGMENTS = 48;
const SLACK_COLOR = '#ff9f43';
const DEFAULT_CABLE_COLOR = '#ffd34d';
const DEFAULT_PLANE_NORMAL = new Vector3(0, 0, 1);
const ORIENTATION_HELPER_LENGTH = 1.5;
const ORIENTATION_HELPER_DARK_COLOR = '#111111';
const ORIENTATION_HELPER_LIGHT_COLOR = '#f5f8ff';
const ORIENTATION_HELPER_COLOR_SAMPLE = new THREE.Color();

function buildPlaneBasis(planeNormal) {
  const n = planeNormal.clone();
  if (n.lengthSq() <= EPSILON) {
    return {
      n: new Vector3(0, 0, 1),
      u: new Vector3(1, 0, 0),
      v: new Vector3(0, 1, 0)
    };
  }

  n.normalize();
  let reference = Math.abs(n.x) < 0.9 ? new Vector3(1, 0, 0) : new Vector3(0, 1, 0);
  const nDotRef = n.dot(reference);
  let u = reference.clone().subtract(n, nDotRef);

  if (u.lengthSq() <= EPSILON) {
    reference = new Vector3(0, 0, 1);
    const nDotRef2 = n.dot(reference);
    u = reference.clone().subtract(n, nDotRef2);
  }

  u.normalize();
  const v = n.cross(u);
  return { n, u, v };
}

function angleOnPlane(point, center, basis) {
  const rel = point.clone().subtract(center);
  const x = rel.dot(basis.u);
  const y = rel.dot(basis.v);
  return Math.atan2(y, x);
}

function appendStoredWrapArcSpecs(arcSpecs, path, linkIndex, center, bodyRadius, attachmentPoint, basis, color, layering) {
  if (!center || !Number.isFinite(bodyRadius) || bodyRadius <= EPSILON || !attachmentPoint) {
    return;
  }

  const stored = Math.max(0.0, path?.stored?.[linkIndex] ?? 0.0);
  if (!(stored > EPSILON)) {
    return;
  }

  const startAngle = angleOnPlane(attachmentPoint, center, basis);
  const cw = Boolean(path?.cw?.[linkIndex]);
  const maxRenderableAngle = (2.0 * Math.PI) - 0.0001;

  if (!layering) {
    let deltaTheta = stored / bodyRadius;
    if (deltaTheta >= maxRenderableAngle) {
      deltaTheta = maxRenderableAngle;
    }
    arcSpecs.push({
      center,
      radius: bodyRadius,
      basis,
      startAngle,
      endAngle: cw ? (startAngle - deltaTheta) : (startAngle + deltaTheta),
      cw,
      color
    });
    return;
  }

  const halfWidth = Number.isFinite(path?.cableHalfWidth) ? Math.max(0.0, path.cableHalfWidth) : 0.0;
  const baseRadius = bodyRadius + halfWidth;
  if (!(baseRadius > EPSILON)) {
    return;
  }

  const fullWidth = 2.0 * halfWidth;
  const MAX_LAYERS = 128;
  let remainingLength = stored;
  let layerIndex = 0;

  while (remainingLength > EPSILON && layerIndex < MAX_LAYERS) {
    const layerRadius = baseRadius + fullWidth * layerIndex;
    if (!(layerRadius > EPSILON)) {
      break;
    }
    const layerCircumference = 2.0 * Math.PI * layerRadius;
    if (!(layerCircumference > EPSILON)) {
      break;
    }
    const layerArcLength = Math.min(remainingLength, layerCircumference);
    if (!(layerArcLength > EPSILON)) {
      break;
    }

    const fullCircle = layerArcLength >= layerCircumference - 1e-6;
    const deltaTheta = fullCircle
      ? (2.0 * Math.PI)
      : Math.min(layerArcLength / layerRadius, maxRenderableAngle);
    arcSpecs.push({
      center,
      radius: layerRadius,
      basis,
      startAngle,
      endAngle: cw ? (startAngle - deltaTheta) : (startAngle + deltaTheta),
      cw,
      color
    });

    remainingLength -= layerArcLength;
    if (!(fullWidth > EPSILON)) {
      break;
    }
    layerIndex += 1;
  }
}

function writeArcPositions(line, center, radius, basis, startAngle, endAngle, cw) {
  const positions = line.geometry.attributes.position.array;
  const segments = line.userData.arcSegments ?? ARC_SEGMENTS;
  const twoPi = Math.PI * 2;

  let end = endAngle;
  if (cw) {
    while (end > startAngle) end -= twoPi;
  } else {
    while (end < startAngle) end += twoPi;
  }

  const sweep = end - startAngle;
  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const ang = startAngle + sweep * t;
    const cos = Math.cos(ang);
    const sin = Math.sin(ang);
    const idx = i * 3;

    positions[idx] = center.x + basis.u.x * radius * cos + basis.v.x * radius * sin;
    positions[idx + 1] = center.y + basis.u.y * radius * cos + basis.v.y * radius * sin;
    positions[idx + 2] = center.z + basis.u.z * radius * cos + basis.v.z * radius * sin;
  }

  line.geometry.attributes.position.needsUpdate = true;
}

function setMaterialColor(material, color) {
  if (!material || typeof color !== 'string') return;
  if (material.userData.__color === color) return;
  material.color.set(color);
  material.userData.__color = color;
}

function disposeObject(obj) {
  if (!obj) return;
  obj.traverse((child) => {
    if (child.geometry && child.userData.ownsGeometry) {
      child.geometry.dispose();
    }
    if (child.material && child.userData.ownsMaterial) {
      if (Array.isArray(child.material)) {
        child.material.forEach((mat) => mat.dispose());
      } else {
        child.material.dispose();
      }
    }
  });
}

export class RenderSystem3D {
  runInPause = true;

  constructor(canvas, options = {}) {
    this.canvas = canvas;
    this.options = options;

    this.borderComponentClass = options.BorderComponent ?? null;
    this.flipperTagComponentClass = options.FlipperTagComponent ?? null;
    this.flipperStateComponentClass = options.FlipperStateComponent ?? null;
    this.defaultPlaneNormal = options.planeNormal ? options.planeNormal.clone().normalize() : DEFAULT_PLANE_NORMAL.clone();

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(options.backgroundColor ?? 0x0f141c);

    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: options.antialias ?? true,
      powerPreference: 'high-performance'
    });

    const isMobile = /Android|iPhone|iPad|iPod/i.test(navigator.userAgent);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, isMobile ? 1.5 : 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;

    const width = Math.max(1, canvas.clientWidth || canvas.width || 1);
    const height = Math.max(1, canvas.clientHeight || canvas.height || 1);
    this.renderer.setSize(width, height, false);

    this.camera = new THREE.PerspectiveCamera(48, width / height, 0.01, 30.0);
    const targetX = Number.isFinite(options.targetX) ? options.targetX : 0.5;
    const targetY = Number.isFinite(options.targetY) ? options.targetY : 0.85;
    const camZ = Number.isFinite(options.cameraZ) ? options.cameraZ : 2.2;
    this.camera.position.set(targetX, targetY + 0.04, camZ);
    this.camera.lookAt(targetX, targetY, 0);

    this.debugEnabled = options.debugEnabled ?? Boolean(window._flipper3dDebug);
    this._debugFrame = 0;
    this.renderOnSimulationStep = options.renderOnSimulationStep ?? false;

    this.controls = null;
    this.controlsEnabled = options.controlsEnabled ?? true;
    this.rotateWithShiftRightMouse = Boolean(options.rotateWithShiftRightMouse);
    this._onShiftKeyDown = null;
    this._onShiftKeyUp = null;
    if (this.controlsEnabled) {
      this.controls = new OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enabled = true;
      this.controls.enableDamping = options.enableDamping ?? false;
      this.controls.enablePan = options.enablePan ?? false;
      this.controls.enableRotate = options.enableRotate ?? false;
      this.controls.enableZoom = options.enableZoom ?? false;
      this.controls.minPolarAngle = Number.isFinite(options.minPolarAngle) ? options.minPolarAngle : 0.0;
      this.controls.maxPolarAngle = Number.isFinite(options.maxPolarAngle) ? options.maxPolarAngle : Math.PI;
      this.controls.minDistance = 0.45;
      this.controls.maxDistance = 8.0;
      this.controls.target.set(targetX, targetY, 0);
      if (options.rotateWithRightMouse || this.rotateWithShiftRightMouse) {
        this.controls.mouseButtons.LEFT = THREE.MOUSE.PAN;
        this.controls.mouseButtons.MIDDLE = THREE.MOUSE.DOLLY;
        this.controls.mouseButtons.RIGHT = options.rotateWithRightMouse ? THREE.MOUSE.ROTATE : THREE.MOUSE.PAN;
        this.controls.touches.ONE = THREE.TOUCH.ROTATE;
        this.controls.touches.TWO = THREE.TOUCH.DOLLY_PAN;
      }
      this.controls.update();
    }

    if (this.controls && this.rotateWithShiftRightMouse && typeof window !== 'undefined') {
      this._onShiftKeyDown = (event) => {
        if (event.key === 'Shift') {
          this.controls.mouseButtons.RIGHT = THREE.MOUSE.ROTATE;
        }
      };
      this._onShiftKeyUp = (event) => {
        if (event.key === 'Shift') {
          this.controls.mouseButtons.RIGHT = THREE.MOUSE.PAN;
        }
      };
      window.addEventListener('keydown', this._onShiftKeyDown);
      window.addEventListener('keyup', this._onShiftKeyUp);
    }

    this._fixedCameraPosition = this.camera.position.clone();
    this._fixedCameraQuaternion = this.camera.quaternion.clone();

    const lights = new THREE.Group();
    lights.add(new THREE.AmbientLight(0x5f7389, 0.8));
    const key = new THREE.DirectionalLight(0xe8f5ff, 1.0);
    key.position.set(1.8, 2.6, 2.8);
    lights.add(key);
    this.scene.add(lights);

    this.boardMaterial = new THREE.MeshStandardMaterial({
      color: 0x1c2430,
      roughness: 0.92,
      metalness: 0.04
    });
    this.boardGeometry = new THREE.PlaneGeometry(1, 1, 1, 1);
    this.board = new THREE.Mesh(this.boardGeometry, this.boardMaterial);
    this.board.position.z = -0.03;
    this.board.userData.ownsGeometry = true;
    this.board.userData.ownsMaterial = true;
    this.scene.add(this.board);

    this.root = new THREE.Group();
    this.scene.add(this.root);

    this.sharedSphereGeometry = new THREE.SphereGeometry(1, 24, 16);
    this.sharedFlipperBarGeometry = new THREE.BoxGeometry(1, 1, 1);
    this.sharedOrientationGeometry = this._createOrientationHelperGeometry();

    this.sphereMaterialCache = new Map();
    this.flipperMaterialCache = new Map();
    this.orientationHelperMaterialCache = new Map();

    this.circleMeshes = new Map();
    this.flipperMeshes = new Map();

    this.jointLines = [];
    this.wrapArcs = [];

    this.borderLine = null;
    this.borderVertexCount = 0;

    this._activeCircleIds = new Set();
    this._activeFlipperIds = new Set();

    this._raycaster = new THREE.Raycaster();
    this._rayNdc = new THREE.Vector2();
    this._rayPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
    this._rayHit = new THREE.Vector3();

    this._lastWidth = width;
    this._lastHeight = height;

    this._onWindowResize = null;
    if (options.autoResize !== false && typeof window !== 'undefined') {
      this._onWindowResize = () => this._resizeToDisplaySize();
      window.addEventListener('resize', this._onWindowResize);
    }
  }

  _collectPoseDebug(world) {
    const pose = {
      camera: {
        position: {
          x: this.camera.position.x,
          y: this.camera.position.y,
          z: this.camera.position.z
        },
        quaternion: {
          x: this.camera.quaternion.x,
          y: this.camera.quaternion.y,
          z: this.camera.quaternion.z,
          w: this.camera.quaternion.w
        }
      },
      controlsTarget: this.controls
        ? {
            x: this.controls.target.x,
            y: this.controls.target.y,
            z: this.controls.target.z
          }
        : null,
      paused: world.getResource('pauseState')?.paused ?? null,
      flippers: [],
      borderCenter: null
    };

    if (this.flipperTagComponentClass && this.flipperStateComponentClass) {
      const flippers = world.query([
        this.flipperTagComponentClass,
        PositionComponent,
        this.flipperStateComponentClass
      ]);

      for (const entityId of flippers) {
        const pos = world.getComponent(entityId, PositionComponent)?.pos;
        const state = world.getComponent(entityId, this.flipperStateComponentClass);
        if (!pos || !state) continue;

        const angle = state.restAngle + state.sign * state.rotation;
        const tip = {
          x: pos.x + Math.cos(angle) * state.length,
          y: pos.y + Math.sin(angle) * state.length,
          z: pos.z
        };

        pose.flippers.push({
          id: entityId,
          pivot: { x: pos.x, y: pos.y, z: pos.z },
          tip,
          angle
        });
      }
    }

    if (this.borderComponentClass) {
      const borderEntities = world.query([this.borderComponentClass]);
      if (borderEntities.length > 0) {
        const border = world.getComponent(borderEntities[0], this.borderComponentClass);
        const points = border?.points ?? [];
        if (points.length > 0) {
          const center = { x: 0, y: 0, z: 0 };
          for (const point of points) {
            center.x += point.x;
            center.y += point.y;
            center.z += point.z;
          }
          const inv = 1.0 / points.length;
          center.x *= inv;
          center.y *= inv;
          center.z *= inv;
          pose.borderCenter = center;
        }
      }
    }

    return pose;
  }

  dumpPose(world) {
    if (!world) return null;
    return this._collectPoseDebug(world);
  }

  setAnimationLoop(callback) {
    this.renderer.setAnimationLoop(callback);
  }

  clearAnimationLoop() {
    this.renderer.setAnimationLoop(null);
  }

  setCanvasSize(width, height) {
    if (!Number.isFinite(width) || !Number.isFinite(height)) return;
    if (width <= 0 || height <= 0) return;
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this._lastWidth = width;
    this._lastHeight = height;
  }

  setComponentClasses(classes = {}) {
    if (classes.BorderComponent !== undefined) this.borderComponentClass = classes.BorderComponent;
    if (classes.FlipperTagComponent !== undefined) this.flipperTagComponentClass = classes.FlipperTagComponent;
    if (classes.FlipperStateComponent !== undefined) this.flipperStateComponentClass = classes.FlipperStateComponent;
  }

  projectClientToSim(clientX, clientY) {
    const rect = this.canvas.getBoundingClientRect();
    if (!rect || rect.width <= 0 || rect.height <= 0) {
      return null;
    }

    this._rayNdc.x = ((clientX - rect.left) / rect.width) * 2 - 1;
    this._rayNdc.y = -((clientY - rect.top) / rect.height) * 2 + 1;
    this._raycaster.setFromCamera(this._rayNdc, this.camera);

    const hit = this._raycaster.ray.intersectPlane(this._rayPlane, this._rayHit);
    if (!hit) return null;

    return new Vector3(hit.x, hit.y, hit.z);
  }

  update(world, dt = 0) {
    if (!this.renderOnSimulationStep && Number.isFinite(dt) && dt > 0) {
      return;
    }

    this._syncBoard(world);
    this._syncBorder(world);
    this._syncCircles(world);
    this._syncFlippers(world);
    this._syncCable(world);

    if (this.controlsEnabled && this.controls) {
      this.controls.update();
    } else {
      // Hard lock to eliminate any camera drift from external handlers/layout churn.
      this.camera.position.copy(this._fixedCameraPosition);
      this.camera.quaternion.copy(this._fixedCameraQuaternion);
      this.camera.updateMatrixWorld();
    }

    if (this.debugEnabled) {
      this._debugFrame += 1;
      if (this._debugFrame % 30 === 0) {
        console.debug('[flipper3d-pose]', this._collectPoseDebug(world));
      }
    }
    this.renderer.render(this.scene, this.camera);
  }

  resetVisuals() {
    for (const mesh of this.circleMeshes.values()) {
      this.root.remove(mesh);
    }
    this.circleMeshes.clear();

    for (const group of this.flipperMeshes.values()) {
      this.root.remove(group);
    }
    this.flipperMeshes.clear();

    for (const line of this.jointLines) {
      this.root.remove(line);
      disposeObject(line);
    }
    this.jointLines.length = 0;

    for (const arc of this.wrapArcs) {
      this.root.remove(arc);
      disposeObject(arc);
    }
    this.wrapArcs.length = 0;

    if (this.borderLine) {
      this.root.remove(this.borderLine);
      disposeObject(this.borderLine);
      this.borderLine = null;
      this.borderVertexCount = 0;
    }
  }

  dispose() {
    this.clearAnimationLoop();
    this.resetVisuals();

    for (const material of this.sphereMaterialCache.values()) {
      material.dispose();
    }
    this.sphereMaterialCache.clear();

    for (const material of this.flipperMaterialCache.values()) {
      material.dispose();
    }
    this.flipperMaterialCache.clear();

    for (const material of this.orientationHelperMaterialCache.values()) {
      material.dispose();
    }
    this.orientationHelperMaterialCache.clear();

    this.sharedSphereGeometry.dispose();
    this.sharedFlipperBarGeometry.dispose();
    this.sharedOrientationGeometry.dispose();

    disposeObject(this.board);

    if (this.controls) {
      this.controls.dispose();
    }
    if (this._onWindowResize && typeof window !== 'undefined') {
      window.removeEventListener('resize', this._onWindowResize);
    }
    if (this._onShiftKeyDown && typeof window !== 'undefined') {
      window.removeEventListener('keydown', this._onShiftKeyDown);
    }
    if (this._onShiftKeyUp && typeof window !== 'undefined') {
      window.removeEventListener('keyup', this._onShiftKeyUp);
    }
    this.renderer.dispose();
  }

  _resizeToDisplaySize() {
    const rect = this.canvas.getBoundingClientRect();
    const width = Math.max(1, Math.round(rect.width || this.canvas.clientWidth || this.canvas.width || 1));
    const height = Math.max(1, Math.round(rect.height || this.canvas.clientHeight || this.canvas.height || 1));

    if (width === this._lastWidth && height === this._lastHeight) {
      return;
    }

    this.setCanvasSize(width, height);
  }

  _syncBoard(world) {
    const simWidth = Number.isFinite(world.getResource('simWidth')) ? world.getResource('simWidth') : 1.0;
    const simHeight = Number.isFinite(world.getResource('simHeight')) ? world.getResource('simHeight') : 1.7;

    this.board.scale.set(simWidth, simHeight, 1);
    this.board.position.set(simWidth * 0.5, simHeight * 0.5, -0.03);
  }

  _syncBorder(world) {
    if (!this.borderComponentClass) {
      if (this.borderLine) {
        this.root.remove(this.borderLine);
        disposeObject(this.borderLine);
        this.borderLine = null;
        this.borderVertexCount = 0;
      }
      return;
    }

    const borderEntities = world.query([this.borderComponentClass, RenderableComponent]);
    if (borderEntities.length === 0) {
      if (this.borderLine) {
        this.root.remove(this.borderLine);
        disposeObject(this.borderLine);
        this.borderLine = null;
        this.borderVertexCount = 0;
      }
      return;
    }

    const borderId = borderEntities[0];
    const borderComp = world.getComponent(borderId, this.borderComponentClass);
    const renderComp = world.getComponent(borderId, RenderableComponent);
    const points = borderComp?.points ?? [];

    if (points.length < 2) {
      if (this.borderLine) {
        this.borderLine.visible = false;
      }
      return;
    }

    if (!this.borderLine || this.borderVertexCount !== points.length) {
      if (this.borderLine) {
        this.root.remove(this.borderLine);
        disposeObject(this.borderLine);
      }

      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(points.length * 3), 3));

      const material = new THREE.LineBasicMaterial({ color: renderComp?.color || '#000000' });
      material.userData.__color = renderComp?.color || '#000000';

      this.borderLine = new THREE.LineLoop(geometry, material);
      this.borderLine.frustumCulled = false;
      this.borderLine.position.z = 0.001;
      this.borderLine.userData.ownsGeometry = true;
      this.borderLine.userData.ownsMaterial = true;
      this.root.add(this.borderLine);
      this.borderVertexCount = points.length;
    }

    this.borderLine.visible = true;
    setMaterialColor(this.borderLine.material, renderComp?.color || '#000000');

    const attr = this.borderLine.geometry.attributes.position;
    const arr = attr.array;
    for (let i = 0; i < points.length; i++) {
      const idx = i * 3;
      const p = points[i];
      arr[idx] = p.x;
      arr[idx + 1] = p.y;
      arr[idx + 2] = p.z;
    }
    attr.needsUpdate = true;
  }

  _syncCircles(world) {
    this._activeCircleIds.clear();

    const entities = world.query([PositionComponent, RadiusComponent, RenderableComponent]);
    for (const entityId of entities) {
      const renderComp = world.getComponent(entityId, RenderableComponent);
      if (!renderComp || renderComp.shape !== 'circle') {
        continue;
      }

      this._activeCircleIds.add(entityId);

      const pos = world.getComponent(entityId, PositionComponent)?.pos;
      const radius = world.getComponent(entityId, RadiusComponent)?.radius;
      if (!pos || !Number.isFinite(radius) || radius <= 0) {
        continue;
      }

      let mesh = this.circleMeshes.get(entityId);
      if (!mesh) {
        mesh = new THREE.Mesh(this.sharedSphereGeometry, this._getSphereMaterial(renderComp.color));
        this.circleMeshes.set(entityId, mesh);
        this.root.add(mesh);
      }

      const material = this._getSphereMaterial(renderComp.color);
      if (mesh.material !== material) {
        mesh.material = material;
      }

      mesh.position.set(pos.x, pos.y, pos.z);
      mesh.scale.set(radius, radius, radius);

      const orientationComp = world.getComponent(entityId, OrientationComponent);
      if (orientationComp?.quaternion) {
        const q = orientationComp.quaternion;
        mesh.quaternion.set(q.x, q.y, q.z, q.w);
        this._ensureOrientationHelper(mesh, renderComp?.color);
      } else {
        mesh.quaternion.identity();
        const helper = mesh.userData.orientationHelper;
        if (helper) {
          helper.visible = false;
        }
      }
    }

    for (const [entityId, mesh] of this.circleMeshes.entries()) {
      if (this._activeCircleIds.has(entityId)) continue;
      this.root.remove(mesh);
      this.circleMeshes.delete(entityId);
    }
  }

  _syncFlippers(world) {
    if (!this.flipperTagComponentClass || !this.flipperStateComponentClass) {
      for (const [entityId, group] of this.flipperMeshes.entries()) {
        this.root.remove(group);
        this.flipperMeshes.delete(entityId);
      }
      return;
    }

    this._activeFlipperIds.clear();

    const flippers = world.query([
      this.flipperTagComponentClass,
      PositionComponent,
      RadiusComponent,
      this.flipperStateComponentClass,
      RenderableComponent
    ]);

    for (const entityId of flippers) {
      this._activeFlipperIds.add(entityId);

      const pos = world.getComponent(entityId, PositionComponent)?.pos;
      const radius = world.getComponent(entityId, RadiusComponent)?.radius;
      const state = world.getComponent(entityId, this.flipperStateComponentClass);
      const renderComp = world.getComponent(entityId, RenderableComponent);

      if (!pos || !state || !Number.isFinite(radius) || radius <= 0) {
        continue;
      }

      let group = this.flipperMeshes.get(entityId);
      if (!group) {
        group = this._createFlipperVisual(renderComp?.color);
        this.flipperMeshes.set(entityId, group);
        this.root.add(group);
      }

      const color = renderComp?.color || '#ff0000';
      if (group.userData.color !== color) {
        const material = this._getFlipperMaterial(color);
        group.userData.bar.material = material;
        group.userData.pivot.material = material;
        group.userData.tip.material = material;
        group.userData.color = color;
      }

      const length = state.length;

      group.userData.bar.scale.set(length, radius * 2, radius * 2);
      group.userData.bar.position.set(length * 0.5, 0, 0);

      group.userData.pivot.scale.set(radius, radius, radius);
      group.userData.tip.scale.set(radius, radius, radius);
      group.userData.tip.position.set(length, 0, 0);

      const angle = state.restAngle + state.sign * state.rotation;
      group.position.set(pos.x, pos.y, pos.z);
      group.rotation.set(0, 0, angle);
    }

    for (const [entityId, group] of this.flipperMeshes.entries()) {
      if (this._activeFlipperIds.has(entityId)) continue;
      this.root.remove(group);
      this.flipperMeshes.delete(entityId);
    }
  }

  _syncCable(world) {
    const pathEntities = world.query([CablePathComponent]);
    const layering =
      layeringEnabled(world) &&
      world.getResource('layeringRenderWraps') !== false;

    if (pathEntities.length === 0) {
      this._hideLines(this.jointLines);
      this._hideLines(this.wrapArcs);
      return;
    }

    const jointSpecs = [];
    const arcSpecs = [];

    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (!path || !Array.isArray(path.jointEntities) || path.jointEntities.length === 0) {
        continue;
      }

      for (const jointId of path.jointEntities) {
        const joint = world.getComponent(jointId, CableJointComponent);
        if (!joint) continue;

        const renderComp = world.getComponent(jointId, RenderableComponent);
        const baseColor = renderComp?.color || DEFAULT_CABLE_COLOR;
        const dist = joint.attachmentPointA_world.distanceTo(joint.attachmentPointB_world);
        const taut = dist > joint.restLength + 1e-6;

        jointSpecs.push({
          joint,
          color: taut ? baseColor : SLACK_COLOR
        });
      }

      for (let i = 1; i < path.linkTypes.length - 1; i++) {
        const linkType = path.linkTypes[i];
        if (linkType !== 'rolling' && linkType !== 'hybrid') continue;
        if (i - 1 < 0 || i >= path.jointEntities.length) continue;

        const prevJoint = world.getComponent(path.jointEntities[i - 1], CableJointComponent);
        const nextJoint = world.getComponent(path.jointEntities[i], CableJointComponent);
        if (!prevJoint || !nextJoint) continue;

        const rollerId = prevJoint.entityB;
        const center = world.getComponent(rollerId, PositionComponent)?.pos;
        const bodyRadius = world.getComponent(rollerId, RadiusComponent)?.radius;
        const radius = Number.isFinite(bodyRadius)
          ? bodyRadius + (layering ? Math.max(0.0, path.cableHalfWidth ?? 0.0) : 0.0)
          : bodyRadius;
        if (!center || !Number.isFinite(radius) || radius <= EPSILON) continue;

        const planeNormal = world.getComponent(rollerId, CableLinkComponent)?.cablePlaneNormal || this.defaultPlaneNormal;
        const basis = buildPlaneBasis(planeNormal);

        const p1 = prevJoint.attachmentPointB_world;
        const p2 = nextJoint.attachmentPointA_world;

        const a1 = angleOnPlane(p1, center, basis);
        const a2 = angleOnPlane(p2, center, basis);

        const distPrev = prevJoint.attachmentPointA_world.distanceTo(prevJoint.attachmentPointB_world);
        const tensionPrev = distPrev > prevJoint.restLength + 1e-6;
        const distNext = nextJoint.attachmentPointA_world.distanceTo(nextJoint.attachmentPointB_world);
        const tensionNext = distNext > nextJoint.restLength + 1e-6;

        const renderComp = world.getComponent(path.jointEntities[i], RenderableComponent);
        const baseColor = renderComp?.color || DEFAULT_CABLE_COLOR;

        arcSpecs.push({
          center,
          radius,
          basis,
          startAngle: a1,
          endAngle: a2,
          cw: Boolean(path.cw[i]),
          color: tensionPrev && tensionNext ? baseColor : SLACK_COLOR
        });
      }

      const nLinks = path.linkTypes.length;

      if (path.linkTypes[0] === 'hybrid' && path.jointEntities.length > 0) {
        const joint0 = world.getComponent(path.jointEntities[0], CableJointComponent);
        if (joint0) {
          const renderComp = world.getComponent(path.jointEntities[0], RenderableComponent);
          const color = renderComp?.color || DEFAULT_CABLE_COLOR;
          const rollerA = joint0.entityA;
          const center = world.getComponent(rollerA, PositionComponent)?.pos;
          const bodyRadius = world.getComponent(rollerA, RadiusComponent)?.radius;

          if (center && Number.isFinite(bodyRadius) && bodyRadius > EPSILON) {
            const planeNormal = world.getComponent(rollerA, CableLinkComponent)?.cablePlaneNormal || this.defaultPlaneNormal;
            const basis = buildPlaneBasis(planeNormal);
            appendStoredWrapArcSpecs(
              arcSpecs,
              path,
              0,
              center,
              bodyRadius,
              joint0.attachmentPointA_world,
              basis,
              color,
              layering
            );
          }
        }
      }

      if (path.linkTypes[nLinks - 1] === 'hybrid' && path.jointEntities.length > 0) {
        const jointN = world.getComponent(path.jointEntities[nLinks - 2], CableJointComponent);
        if (jointN) {
          const renderComp = world.getComponent(path.jointEntities[nLinks - 2], RenderableComponent);
          const color = renderComp?.color || DEFAULT_CABLE_COLOR;
          const rollerB = jointN.entityB;
          const center = world.getComponent(rollerB, PositionComponent)?.pos;
          const bodyRadius = world.getComponent(rollerB, RadiusComponent)?.radius;

          if (center && Number.isFinite(bodyRadius) && bodyRadius > EPSILON) {
            const planeNormal = world.getComponent(rollerB, CableLinkComponent)?.cablePlaneNormal || this.defaultPlaneNormal;
            const basis = buildPlaneBasis(planeNormal);
            appendStoredWrapArcSpecs(
              arcSpecs,
              path,
              nLinks - 1,
              center,
              bodyRadius,
              jointN.attachmentPointB_world,
              basis,
              color,
              layering
            );
          }
        }
      }
    }

    this._ensureLineCapacity(this.jointLines, jointSpecs.length, false);
    this._ensureLineCapacity(this.wrapArcs, arcSpecs.length, true);

    for (let i = 0; i < this.jointLines.length; i++) {
      const line = this.jointLines[i];
      const spec = jointSpecs[i];
      if (!spec) {
        line.visible = false;
        continue;
      }

      line.visible = true;
      setMaterialColor(line.material, spec.color);

      const p = line.geometry.attributes.position.array;
      p[0] = spec.joint.attachmentPointA_world.x;
      p[1] = spec.joint.attachmentPointA_world.y;
      p[2] = spec.joint.attachmentPointA_world.z;
      p[3] = spec.joint.attachmentPointB_world.x;
      p[4] = spec.joint.attachmentPointB_world.y;
      p[5] = spec.joint.attachmentPointB_world.z;
      line.geometry.attributes.position.needsUpdate = true;
    }

    for (let i = 0; i < this.wrapArcs.length; i++) {
      const arc = this.wrapArcs[i];
      const spec = arcSpecs[i];
      if (!spec) {
        arc.visible = false;
        continue;
      }

      arc.visible = true;
      setMaterialColor(arc.material, spec.color);
      writeArcPositions(
        arc,
        spec.center,
        spec.radius,
        spec.basis,
        spec.startAngle,
        spec.endAngle,
        spec.cw
      );
    }
  }

  _ensureLineCapacity(target, count, isArc) {
    while (target.length < count) {
      const line = isArc ? this._createArcLine() : this._createLine();
      target.push(line);
      this.root.add(line);
    }
  }

  _createLine() {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));

    const material = new THREE.LineBasicMaterial({ color: DEFAULT_CABLE_COLOR });
    material.userData.__color = DEFAULT_CABLE_COLOR;

    const line = new THREE.Line(geometry, material);
    line.frustumCulled = false;
    line.userData.ownsGeometry = true;
    line.userData.ownsMaterial = true;
    return line;
  }

  _createArcLine() {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array((ARC_SEGMENTS + 1) * 3), 3));

    const material = new THREE.LineBasicMaterial({ color: DEFAULT_CABLE_COLOR });
    material.userData.__color = DEFAULT_CABLE_COLOR;

    const line = new THREE.Line(geometry, material);
    line.frustumCulled = false;
    line.userData.ownsGeometry = true;
    line.userData.ownsMaterial = true;
    line.userData.arcSegments = ARC_SEGMENTS;
    return line;
  }

  _hideLines(lines) {
    for (const line of lines) {
      line.visible = false;
    }
  }

  _createOrientationHelperGeometry() {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array([0, 0, 0, ORIENTATION_HELPER_LENGTH, 0, 0]), 3)
    );
    return geometry;
  }

  _getOrientationHelperMaterial(color) {
    ORIENTATION_HELPER_COLOR_SAMPLE.set(typeof color === 'string' && color.length > 0 ? color : '#a0a0a0');
    const luminance = (
      (0.2126 * ORIENTATION_HELPER_COLOR_SAMPLE.r) +
      (0.7152 * ORIENTATION_HELPER_COLOR_SAMPLE.g) +
      (0.0722 * ORIENTATION_HELPER_COLOR_SAMPLE.b)
    );
    const key = luminance > 0.58 ? 'dark' : 'light';

    let material = this.orientationHelperMaterialCache.get(key);
    if (!material) {
      material = new THREE.LineBasicMaterial({
        color: key === 'dark' ? ORIENTATION_HELPER_DARK_COLOR : ORIENTATION_HELPER_LIGHT_COLOR
      });
      this.orientationHelperMaterialCache.set(key, material);
    }
    return material;
  }

  _ensureOrientationHelper(mesh, color) {
    let helper = mesh.userData.orientationHelper;
    const material = this._getOrientationHelperMaterial(color);

    if (!helper) {
      helper = new THREE.Line(this.sharedOrientationGeometry, material);
      helper.frustumCulled = false;
      helper.userData.isOrientationHelper = true;
      mesh.add(helper);
      mesh.userData.orientationHelper = helper;
    } else if (helper.material !== material) {
      helper.material = material;
    }

    helper.visible = true;
    return helper;
  }

  _getSphereMaterial(color) {
    const key = typeof color === 'string' && color.length > 0 ? color : '#a0a0a0';
    let material = this.sphereMaterialCache.get(key);
    if (!material) {
      material = new THREE.MeshStandardMaterial({
        color: key,
        roughness: 0.42,
        metalness: 0.1
      });
      this.sphereMaterialCache.set(key, material);
    }
    return material;
  }

  _getFlipperMaterial(color) {
    const key = typeof color === 'string' && color.length > 0 ? color : '#ff0000';
    let material = this.flipperMaterialCache.get(key);
    if (!material) {
      material = new THREE.MeshStandardMaterial({
        color: key,
        roughness: 0.34,
        metalness: 0.16
      });
      this.flipperMaterialCache.set(key, material);
    }
    return material;
  }

  _createFlipperVisual(color) {
    const material = this._getFlipperMaterial(color);

    const group = new THREE.Group();

    const bar = new THREE.Mesh(this.sharedFlipperBarGeometry, material);
    const pivot = new THREE.Mesh(this.sharedSphereGeometry, material);
    const tip = new THREE.Mesh(this.sharedSphereGeometry, material);

    group.add(bar);
    group.add(pivot);
    group.add(tip);

    group.userData.bar = bar;
    group.userData.pivot = pivot;
    group.userData.tip = tip;
    group.userData.color = typeof color === 'string' && color.length > 0 ? color : '#ff0000';

    return group;
  }
}
