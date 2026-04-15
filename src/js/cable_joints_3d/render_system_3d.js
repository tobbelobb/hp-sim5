import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

import Vector3 from './vector3.js';
import {
  PositionComponent,
  RadiusComponent,
  OrientationComponent,
  ObstaclePushComponent,
  RenderableComponent,
  RigidGroupComponent,
  layeringEnabled
} from './ecs.js';
import {
  CablePathComponent,
  CableJointComponent,
  CableLinkComponent
} from './cable_joints_core.js';
import { ExtruderComponent } from '../../../hp-sim-3d/app/hangprinter_extruder.js';

const EPSILON = 1e-9;
const ARC_SEGMENTS = 48;
const JOINT_LINE_SEGMENTS = 24;
const SLACK_COLOR = '#ff9f43';
const DEFAULT_CABLE_COLOR = '#ffd34d';
const DEFAULT_PLANE_NORMAL = new Vector3(0, 0, 1);
const ORIENTATION_BACK_COLOR = '#2a3542';
const KNOT_MARKER_COLOR = '#ff3b30';
const KNOT_MARKER_RADIUS = 0.002;
const DEFAULT_BACKGROUND_COLOR = 0x1b2b3c;
const PRINT_SURFACE_Z = -0.0005;
const PRINT_SURFACE_OUTLINE_Z = 0.0008;
const PRINT_SURFACE_MIN_HALF_EXTENT = 0.18;
const PRINT_SURFACE_MARGIN = 0.08;
const PRINT_SURFACE_COLOR = 0x243248;
const PRINT_SURFACE_OUTLINE_COLOR = 0x5f7492;
const BORDER_FLOOR_OFFSET_Z = -0.035;
const BORDER_FLOOR_COLOR = 0x1d2434;
const BORDER_FLOOR_COLOR_HEX = '#1d2434';
const BORDER_WALL_HEIGHT = 0.08;
const BORDER_WALL_THICKNESS = 0.012;
const BORDER_WALL_COLOR = 0x0c111f;
const BORDER_WALL_COLOR_HEX = '#0c111f';
const DEFAULT_RIGID_GROUP_COLOR = '#55ff88';
const DEFAULT_EXTRUDER_COLOR = '#ff8a3d';
const BUMPER_FX_MAX_BURSTS = 96;
const BUMPER_FX_MIN_RADIUS = 0.03;
const DEFAULT_REFERENCE_COLOR = '#1e90ff';
const DEFAULT_TRACE_COLOR = '#ffffff';
const DEFAULT_TRACE_MARKER_COLOR = '#2dd4bf';
const DEFAULT_NAV_CURSOR_COLOR = '#ffd34d';
const DEFAULT_TRACE_POINT_SIZE = 3;
const DEFAULT_EXTRUSION_POINT_SIZE = 2;
const DEFAULT_TRACE_Z = 0.0025;
const DEFAULT_MARKER_Z = 0.005;
const DEFAULT_NAV_CURSOR_MIN_SIZE = 0.012;
const DEFAULT_NAV_CURSOR_PIXEL_SIZE = 18;
const DEFAULT_ORBIT_AZIMUTH = -Math.PI * 0.25;
const DEFAULT_ORBIT_POLAR = 1.05;
const CATENARY_HORIZONTAL_EPSILON = 1e-5;
const CATENARY_SOLVER_ITERATIONS = 48;
const CATENARY_MAX_SINH_ARGUMENT = 20;
const PARABOLIC_SAG_SCALE = 0.5;

function finiteOr(value, fallback) {
  return Number.isFinite(value) ? value : fallback;
}

function getUpDirection(world) {
  const gravity = world?.getResource?.('gravity');
  if (
    gravity &&
    Number.isFinite(gravity.x) &&
    Number.isFinite(gravity.y) &&
    Number.isFinite(gravity.z) &&
    gravity.lengthSq() > EPSILON
  ) {
    return gravity.clone().normalize().scale(-1);
  }
  return new Vector3(0, 0, 1);
}

function safeSinh(value) {
  if (value >= CATENARY_MAX_SINH_ARGUMENT) {
    return Number.POSITIVE_INFINITY;
  }
  return Math.sinh(value);
}

function solveCatenaryScale(horizontalSpan, verticalOffset, cableLength) {
  if (
    !Number.isFinite(horizontalSpan) ||
    !Number.isFinite(verticalOffset) ||
    !Number.isFinite(cableLength) ||
    !(horizontalSpan > CATENARY_HORIZONTAL_EPSILON) ||
    !(cableLength > Math.abs(verticalOffset) + EPSILON)
  ) {
    return null;
  }

  const targetSquared = (cableLength * cableLength) - (verticalOffset * verticalOffset);
  if (!(targetSquared > (horizontalSpan * horizontalSpan) + EPSILON)) {
    return null;
  }

  const target = Math.sqrt(targetSquared);
  const evaluate = (a) => {
    if (!Number.isFinite(a) || !(a > EPSILON)) {
      return Number.POSITIVE_INFINITY;
    }
    const arg = horizontalSpan / (2.0 * a);
    return (2.0 * a * safeSinh(arg)) - target;
  };

  let low = Math.max(horizontalSpan * 1e-6, EPSILON);
  let high = Math.max(horizontalSpan, target, 1e-3);
  while (!(evaluate(high) < 0.0) && high < 1e9) {
    high *= 2.0;
  }
  if (!(evaluate(high) < 0.0)) {
    return null;
  }

  for (let i = 0; i < CATENARY_SOLVER_ITERATIONS; i += 1) {
    const mid = 0.5 * (low + high);
    if (evaluate(mid) > 0.0) {
      low = mid;
    } else {
      high = mid;
    }
  }

  return high;
}

function writeStraightCablePositions(positions, start, end, segments) {
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const dz = end.z - start.z;
  for (let i = 0; i <= segments; i += 1) {
    const t = i / segments;
    const idx = i * 3;
    positions[idx] = start.x + (dx * t);
    positions[idx + 1] = start.y + (dy * t);
    positions[idx + 2] = start.z + (dz * t);
  }
}

function writeParabolicSagCablePositions(positions, start, end, cableLength, upDirection, segments) {
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const dz = end.z - start.z;
  const straightLength = Math.hypot(dx, dy, dz);
  const sagDepth = Math.max(0.0, cableLength - straightLength) * PARABOLIC_SAG_SCALE;

  for (let i = 0; i <= segments; i += 1) {
    const t = i / segments;
    const sagOffset = sagDepth * 4.0 * t * (1.0 - t);
    const idx = i * 3;
    positions[idx] = start.x + (dx * t) - (upDirection.x * sagOffset);
    positions[idx + 1] = start.y + (dy * t) - (upDirection.y * sagOffset);
    positions[idx + 2] = start.z + (dz * t) - (upDirection.z * sagOffset);
  }
}

function writeSlackCablePositions(positions, start, end, cableLength, upDirection, segments) {
  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const dz = end.z - start.z;
  const straightLength = Math.hypot(dx, dy, dz);
  if (!(cableLength > straightLength + EPSILON)) {
    writeStraightCablePositions(positions, start, end, segments);
    return;
  }

  const verticalOffset = (dx * upDirection.x) + (dy * upDirection.y) + (dz * upDirection.z);
  const horizontalDx = dx - (upDirection.x * verticalOffset);
  const horizontalDy = dy - (upDirection.y * verticalOffset);
  const horizontalDz = dz - (upDirection.z * verticalOffset);
  const horizontalSpan = Math.hypot(horizontalDx, horizontalDy, horizontalDz);
  if (!(horizontalSpan > CATENARY_HORIZONTAL_EPSILON)) {
    writeParabolicSagCablePositions(positions, start, end, cableLength, upDirection, segments);
    return;
  }

  const invHorizontalSpan = 1.0 / horizontalSpan;
  const horizontalUnitX = horizontalDx * invHorizontalSpan;
  const horizontalUnitY = horizontalDy * invHorizontalSpan;
  const horizontalUnitZ = horizontalDz * invHorizontalSpan;

  const catenaryScale = solveCatenaryScale(horizontalSpan, verticalOffset, cableLength);
  if (!Number.isFinite(catenaryScale) || !(catenaryScale > EPSILON)) {
    writeParabolicSagCablePositions(positions, start, end, cableLength, upDirection, segments);
    return;
  }

  const target = Math.sqrt(Math.max(0.0, (cableLength * cableLength) - (verticalOffset * verticalOffset)));
  if (!(target > horizontalSpan + EPSILON)) {
    writeParabolicSagCablePositions(positions, start, end, cableLength, upDirection, segments);
    return;
  }

  const halfSpan = horizontalSpan * 0.5;
  const p = Math.asinh(verticalOffset / target);
  const q = halfSpan / catenaryScale;
  const endpointOffset = catenaryScale * Math.cosh(q - p);

  for (let i = 0; i <= segments; i += 1) {
    const t = i / segments;
    const x = horizontalSpan * t;
    const y = (catenaryScale * Math.cosh(((x - halfSpan) / catenaryScale) + p)) - endpointOffset;
    const idx = i * 3;
    positions[idx] = start.x + (horizontalUnitX * x) + (upDirection.x * y);
    positions[idx + 1] = start.y + (horizontalUnitY * x) + (upDirection.y * y);
    positions[idx + 2] = start.z + (horizontalUnitZ * x) + (upDirection.z * y);
  }

  positions[0] = start.x;
  positions[1] = start.y;
  positions[2] = start.z;
  const lastIndex = segments * 3;
  positions[lastIndex] = end.x;
  positions[lastIndex + 1] = end.y;
  positions[lastIndex + 2] = end.z;
}

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

function knotMarkerDeltaAngle(storedLength, baseRadius, halfWidth, layering) {
  const stored = Math.max(0.0, storedLength ?? 0.0);
  if (!(stored > EPSILON) || !(baseRadius > EPSILON)) {
    return 0.0;
  }

  if (!layering || !(halfWidth > EPSILON)) {
    return stored / baseRadius;
  }

  const fullWidth = 2.0 * halfWidth;
  let layerRadius = baseRadius;
  let remainingLength = stored;
  let layerCount = 0;
  const MAX_LAYERS = 128;

  while (fullWidth > EPSILON && layerCount < MAX_LAYERS) {
    const layerCircumference = 2.0 * Math.PI * layerRadius;
    if (!(remainingLength > layerCircumference + EPSILON)) {
      break;
    }
    remainingLength -= layerCircumference;
    layerCount += 1;
    layerRadius = baseRadius + fullWidth * layerCount;
  }

  return layerRadius > EPSILON ? (remainingLength / layerRadius) : 0.0;
}

function createDualColorSphereGeometry(baseGeometry, primaryColor, secondaryColor = ORIENTATION_BACK_COLOR) {
  const geometry = baseGeometry.clone();
  const positions = geometry.attributes.position;
  const colors = new Float32Array(positions.count * 3);
  const primary = new THREE.Color(primaryColor);
  const secondary = new THREE.Color(secondaryColor);

  for (let i = 0; i < positions.count; i += 1) {
    const source = positions.getX(i) >= 0 ? primary : secondary;
    const idx = i * 3;
    colors[idx] = source.r;
    colors[idx + 1] = source.g;
    colors[idx + 2] = source.b;
  }

  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  return geometry;
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

export function collectRigidGroupEdges(memberCount, renderSegments) {
  if (!Number.isInteger(memberCount) || memberCount < 2) {
    return [];
  }

  const edges = [];
  const seen = new Set();
  const addEdge = (aIdx, bIdx) => {
    if (!Number.isInteger(aIdx) || !Number.isInteger(bIdx)) {
      return;
    }
    if (aIdx < 0 || bIdx < 0 || aIdx >= memberCount || bIdx >= memberCount || aIdx === bIdx) {
      return;
    }
    const key = aIdx < bIdx ? `${aIdx}:${bIdx}` : `${bIdx}:${aIdx}`;
    if (seen.has(key)) {
      return;
    }
    seen.add(key);
    edges.push([aIdx, bIdx]);
  };

  if (Array.isArray(renderSegments) && renderSegments.length > 0) {
    for (const segment of renderSegments) {
      if (!Array.isArray(segment) || segment.length < 2) {
        continue;
      }
      for (let i = 0; i < segment.length - 1; i += 1) {
        addEdge(segment[i], segment[i + 1]);
      }
      const first = segment[0];
      const last = segment[segment.length - 1];
      if (segment.length > 2 && first !== last) {
        addEdge(last, first);
      }
    }
    return edges;
  }

  for (let aIdx = 0; aIdx < memberCount; aIdx += 1) {
    for (let bIdx = aIdx + 1; bIdx < memberCount; bIdx += 1) {
      addEdge(aIdx, bIdx);
    }
  }
  return edges;
}

export class RenderSystem3D {
  static createDefaultLightingGroup() {
    const lights = new THREE.Group();
    lights.add(new THREE.AmbientLight(0x7a92b0, 1.35));
    const key = new THREE.DirectionalLight(0xf6fbff, 1.35);
    key.position.set(1.8, 2.6, 2.8);
    lights.add(key);
    const skyLight = new THREE.HemisphereLight(0xa6caff, 0x27303c, 0.45);
    lights.add(skyLight);
    return lights;
  }

  static buildBorderShape(points) {
    if (!points || points.length < 3) {
      return null;
    }

    const shape = new THREE.Shape();
    shape.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i += 1) {
      shape.lineTo(points[i].x, points[i].y);
    }
    shape.closePath();
    return shape;
  }

  static get DEFAULT_BACKGROUND_COLOR() {
    return DEFAULT_BACKGROUND_COLOR;
  }
  static get DEFAULT_BORDER_FLOOR_COLOR() {
    return BORDER_FLOOR_COLOR;
  }
  static get DEFAULT_BORDER_WALL_COLOR() {
    return BORDER_WALL_COLOR;
  }
  static get DEFAULT_BORDER_FLOOR_Z() {
    return BORDER_FLOOR_OFFSET_Z;
  }
  runInPause = true;

  constructor(canvas, options = {}) {
    this.canvas = canvas;
    this.options = options;

    this.borderComponentClass = options.BorderComponent ?? null;
    this.flipperTagComponentClass = options.FlipperTagComponent ?? null;
    this.flipperStateComponentClass = options.FlipperStateComponent ?? null;
    this.defaultPlaneNormal = options.planeNormal ? options.planeNormal.clone().normalize() : DEFAULT_PLANE_NORMAL.clone();

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(options.backgroundColor ?? RenderSystem3D.DEFAULT_BACKGROUND_COLOR);

    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: options.antialias ?? true,
      powerPreference: 'high-performance'
    });

    const userAgent = typeof navigator !== 'undefined' ? (navigator.userAgent || '') : '';
    const devicePixelRatio = typeof window !== 'undefined' ? (window.devicePixelRatio || 1) : 1;
    const isMobile = /Android|iPhone|iPad|iPod/i.test(userAgent);
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, isMobile ? 1.5 : 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;

    const width = Math.max(1, canvas.clientWidth || canvas.width || 1);
    const height = Math.max(1, canvas.clientHeight || canvas.height || 1);
    this.renderer.setSize(width, height, false);

    this.camera = new THREE.PerspectiveCamera(48, width / height, 0.01, 30.0);
    const targetX = Number.isFinite(options.targetX) ? options.targetX : 0.0;
    const targetY = Number.isFinite(options.targetY) ? options.targetY : 0.0;
    const camZ = Number.isFinite(options.cameraZ) ? options.cameraZ : 2.2;
    this.camera.position.set(targetX, targetY + 0.04, camZ);
    this.camera.up.set(0, 0, 1);
    this.camera.lookAt(targetX, targetY, 0);
    this._baseViewTarget = new THREE.Vector3(targetX, targetY, 0);
    this._baseCameraOffset = this.camera.position.clone().sub(this._baseViewTarget);
    this._baseCameraDistance = Math.max(
      0.1,
      Number.isFinite(options.cameraDistance) ? options.cameraDistance : this._baseCameraOffset.length()
    );
    this.viewScaleMultiplier = 1.0;
    this.viewOffsetX = 0.0;
    this.viewOffsetY = 0.0;
    this.viewOffsetZ = 0.0;
    this.orbitMinPolarAngle = Number.isFinite(options.minOrbitPolarAngle) ? options.minOrbitPolarAngle : 1e-6;
    this.orbitMaxPolarAngle = Number.isFinite(options.maxOrbitPolarAngle) ? options.maxOrbitPolarAngle : (Math.PI - 1e-6);
    this.orbitRotateSpeed = Number.isFinite(options.orbitRotateSpeed) ? options.orbitRotateSpeed : 1.0;
    this.orbitAzimuth = Number.isFinite(options.initialOrbitAzimuth) ? options.initialOrbitAzimuth : DEFAULT_ORBIT_AZIMUTH;
    this.orbitPolar = THREE.MathUtils.clamp(
      Number.isFinite(options.initialOrbitPolar) ? options.initialOrbitPolar : DEFAULT_ORBIT_POLAR,
      this.orbitMinPolarAngle,
      this.orbitMaxPolarAngle
    );
    this._applyCameraFromViewTransform();

    this.debugEnabled = options.debugEnabled ?? (typeof window !== 'undefined' && Boolean(window._flipper3dDebug));
    this._debugFrame = 0;
    this.renderOnSimulationStep = options.renderOnSimulationStep ?? false;
    this.drawingSuspended = false;
    this.referencePaths = [];
    this.referenceMetadata = null;
    this.referenceColor = DEFAULT_REFERENCE_COLOR;
    this.referenceRequestedVisible = false;
    this.referenceVisible = false;
    this.referenceDirty = false;
    this.positionTraceEnabled = false;
    this.positionTraceColor = DEFAULT_TRACE_COLOR;
    this.positionTraceRadiusPx = 1.25;
    this.positionTracePoints = [];
    this.positionTraceMarkers = [];
    this.drawnPositionTraceCount = 0;
    this.drawnPositionTraceMarkerCount = 0;
    this.drawnExtrusionCount = 0;
    this._animationLoopActive = false;
    this._requestRenderHandle = null;
    this._lastWorld = null;
    this.extruderLine = null;

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

    const lights = RenderSystem3D.createDefaultLightingGroup();
    this.scene.add(lights);

    this.boardMaterial = new THREE.MeshStandardMaterial({
      color: PRINT_SURFACE_COLOR,
      roughness: 0.94,
      metalness: 0.02,
      transparent: true,
      opacity: 0.32,
      depthWrite: false,
      side: THREE.DoubleSide,
      toneMapped: false
    });
    this.boardGeometry = new THREE.PlaneGeometry(1, 1, 1, 1);
    this.board = new THREE.Mesh(this.boardGeometry, this.boardMaterial);
    this.board.position.z = PRINT_SURFACE_Z;
    this.board.renderOrder = 40;
    this.board.userData.ownsGeometry = true;
    this.board.userData.ownsMaterial = true;
    this.scene.add(this.board);

    this.boardOutlineMaterial = new THREE.LineBasicMaterial({
      color: PRINT_SURFACE_OUTLINE_COLOR,
      transparent: true,
      opacity: 0.95,
      depthWrite: false,
      toneMapped: false
    });
    this.boardOutlineGeometry = new THREE.BufferGeometry();
    this.boardOutlineGeometry.setAttribute(
      'position',
      new THREE.BufferAttribute(
        new Float32Array([
          -0.5, -0.5, 0.0,
          0.5, -0.5, 0.0,
          0.5, 0.5, 0.0,
          -0.5, 0.5, 0.0
        ]),
        3
      )
    );
    this.boardOutline = new THREE.LineLoop(this.boardOutlineGeometry, this.boardOutlineMaterial);
    this.boardOutline.position.z = PRINT_SURFACE_OUTLINE_Z;
    this.boardOutline.renderOrder = 50;
    this.boardOutline.frustumCulled = false;
    this.boardOutline.userData.ownsGeometry = true;
    this.boardOutline.userData.ownsMaterial = true;
    this.scene.add(this.boardOutline);

    this.borderFloorMaterial = new THREE.MeshStandardMaterial({
      color: BORDER_FLOOR_COLOR,
      roughness: 0.92,
      metalness: 0.02
    });
    this.borderFloorMaterial.polygonOffset = true;
    this.borderFloorMaterial.polygonOffsetFactor = 1;
    this.borderFloorMaterial.polygonOffsetUnits = 2;
    this.borderFloor = new THREE.Mesh(new THREE.BufferGeometry(), this.borderFloorMaterial);
    this.borderFloor.position.z = BORDER_FLOOR_OFFSET_Z;
    this.borderFloor.visible = false;
    this.borderFloor.frustumCulled = false;
    this.borderFloor.userData.ownsGeometry = true;
    this.borderFloor.userData.ownsMaterial = true;
    this.scene.add(this.borderFloor);

    this.borderWallMaterial = new THREE.MeshStandardMaterial({
      color: BORDER_WALL_COLOR,
      roughness: 0.92,
      metalness: 0.08
    });
    this.borderWallMaterial.polygonOffset = true;
    this.borderWallMaterial.polygonOffsetFactor = 1;
    this.borderWallMaterial.polygonOffsetUnits = 2;
    this.borderWallGroup = new THREE.Group();
    this.borderWallGroup.frustumCulled = false;
    this.scene.add(this.borderWallGroup);

    this.root = new THREE.Group();
    this.scene.add(this.root);

    this.referenceMaterial = new THREE.LineBasicMaterial({
      color: DEFAULT_REFERENCE_COLOR,
      transparent: true,
      opacity: 0.95,
      depthWrite: false,
      toneMapped: false
    });
    this.referenceLines = new THREE.LineSegments(new THREE.BufferGeometry(), this.referenceMaterial);
    this.referenceLines.visible = false;
    this.referenceLines.renderOrder = 900;
    this.referenceLines.frustumCulled = false;
    this.referenceLines.userData.ownsGeometry = true;
    this.referenceLines.userData.ownsMaterial = true;
    this.scene.add(this.referenceLines);

    this.extrusionPointsMaterial = new THREE.PointsMaterial({
      size: DEFAULT_EXTRUSION_POINT_SIZE,
      sizeAttenuation: false,
      vertexColors: true,
      transparent: true,
      opacity: 0.95,
      depthWrite: false,
      toneMapped: false
    });
    this.extrusionPoints = new THREE.Points(new THREE.BufferGeometry(), this.extrusionPointsMaterial);
    this.extrusionPoints.frustumCulled = false;
    this.extrusionPoints.renderOrder = 850;
    this.extrusionPoints.userData.ownsGeometry = true;
    this.extrusionPoints.userData.ownsMaterial = true;
    this.scene.add(this.extrusionPoints);

    this.positionTraceMaterial = new THREE.PointsMaterial({
      color: DEFAULT_TRACE_COLOR,
      size: DEFAULT_TRACE_POINT_SIZE,
      sizeAttenuation: false,
      transparent: true,
      opacity: 0.92,
      depthWrite: false,
      toneMapped: false
    });
    this.positionTracePointsObject = new THREE.Points(new THREE.BufferGeometry(), this.positionTraceMaterial);
    this.positionTracePointsObject.frustumCulled = false;
    this.positionTracePointsObject.renderOrder = 920;
    this.positionTracePointsObject.userData.ownsGeometry = true;
    this.positionTracePointsObject.userData.ownsMaterial = true;
    this.scene.add(this.positionTracePointsObject);

    this.positionTraceMarkerMaterial = new THREE.PointsMaterial({
      color: DEFAULT_TRACE_MARKER_COLOR,
      size: DEFAULT_TRACE_POINT_SIZE + 2,
      sizeAttenuation: false,
      transparent: true,
      opacity: 0.98,
      depthWrite: false,
      toneMapped: false
    });
    this.positionTraceMarkersObject = new THREE.Points(new THREE.BufferGeometry(), this.positionTraceMarkerMaterial);
    this.positionTraceMarkersObject.frustumCulled = false;
    this.positionTraceMarkersObject.renderOrder = 930;
    this.positionTraceMarkersObject.userData.ownsGeometry = true;
    this.positionTraceMarkersObject.userData.ownsMaterial = true;
    this.scene.add(this.positionTraceMarkersObject);

    this.navigationCursorVisible = false;
    this.navigationCursorMaterial = new THREE.LineBasicMaterial({
      color: DEFAULT_NAV_CURSOR_COLOR,
      transparent: true,
      opacity: 0.95,
      depthWrite: false,
      depthTest: false,
      toneMapped: false
    });
    this.navigationCursorGeometry = new THREE.BufferGeometry();
    this.navigationCursorGeometry.setAttribute(
      'position',
      new THREE.BufferAttribute(
        new Float32Array([
          -1, 0, 0,
          1, 0, 0,
          0, -1, 0,
          0, 1, 0,
          0, 0, -1,
          0, 0, 1,
        ]),
        3
      )
    );
    this.navigationCursorObject = new THREE.LineSegments(this.navigationCursorGeometry, this.navigationCursorMaterial);
    this.navigationCursorObject.visible = false;
    this.navigationCursorObject.renderOrder = 940;
    this.navigationCursorObject.frustumCulled = false;
    this.navigationCursorObject.userData.ownsGeometry = true;
    this.navigationCursorObject.userData.ownsMaterial = true;
    this.scene.add(this.navigationCursorObject);

    this._bumperFxGroup = new THREE.Group();
    this._bumperFxGroup.frustumCulled = false;
    this._bumperFxGroup.renderOrder = 1000;
    this.scene.add(this._bumperFxGroup);

    this._bumperFxGeometry = new THREE.SphereGeometry(1, 12, 10);
    this._bumperFxBursts = [];
    this._bumperFxLastTimeSec = Number.NaN;

    this.sharedSphereGeometry = new THREE.SphereGeometry(1, 24, 16);
    this.sharedFlipperBarGeometry = new THREE.BoxGeometry(1, 1, 1);
    this.orientedSphereGeometryCache = new Map();

    this.sphereMaterialCache = new Map();
    this.flipperMaterialCache = new Map();
    this.orientedSphereMaterial = new THREE.MeshStandardMaterial({
      vertexColors: true,
      roughness: 0.42,
      metalness: 0.1
    });
    this.knotMarkerMaterial = new THREE.MeshBasicMaterial({ color: KNOT_MARKER_COLOR });

    this.circleMeshes = new Map();
    this.flipperMeshes = new Map();

    this.jointLines = [];
    this.wrapArcs = [];
    this.knotMarkers = [];
    this.rigidGroupLines = [];

    this.borderLine = null;
    this.borderVertexCount = 0;

    this._activeCircleIds = new Set();
    this._activeFlipperIds = new Set();

    this._raycaster = new THREE.Raycaster();
    this._rayNdc = new THREE.Vector2();
    this._rayPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
    this._rayPlaneNormal = new THREE.Vector3(0, 0, 1);
    this._rayPlanePoint = new THREE.Vector3();
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
    if (this._requestRenderHandle != null && typeof cancelAnimationFrame === 'function') {
      cancelAnimationFrame(this._requestRenderHandle);
      this._requestRenderHandle = null;
    }
    this._animationLoopActive = typeof callback === 'function';
    this.renderer.setAnimationLoop(callback);
  }

  clearAnimationLoop() {
    this._animationLoopActive = false;
    if (this._requestRenderHandle != null && typeof cancelAnimationFrame === 'function') {
      cancelAnimationFrame(this._requestRenderHandle);
      this._requestRenderHandle = null;
    }
    this.renderer.setAnimationLoop(null);
  }

  requestRender(world = null) {
    if (world) {
      this._lastWorld = world;
    }
    if (this.drawingSuspended || this._animationLoopActive) {
      return;
    }
    if (this._requestRenderHandle != null) {
      return;
    }

    const draw = () => {
      this._requestRenderHandle = null;
      if (this.drawingSuspended || this._animationLoopActive) {
        return;
      }
      if (this._lastWorld) {
        this.update(this._lastWorld, 0);
        return;
      }
      this.renderer.render(this.scene, this.camera);
    };

    if (typeof requestAnimationFrame === 'function') {
      this._requestRenderHandle = requestAnimationFrame(draw);
      return;
    }
    draw();
  }

  setCanvasSize(width, height) {
    if (!Number.isFinite(width) || !Number.isFinite(height)) return;
    if (width <= 0 || height <= 0) return;
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this._lastWidth = width;
    this._lastHeight = height;
    this.requestRender();
  }

  setDrawingSuspended(suspended) {
    this.drawingSuspended = Boolean(suspended);
    if (!this.drawingSuspended) {
      this.requestRender();
    }
  }

  setViewTransform({ scaleMultiplier, offsetX, offsetY, offsetZ } = {}) {
    if (typeof scaleMultiplier === 'number' && Number.isFinite(scaleMultiplier) && scaleMultiplier > 0) {
      this.viewScaleMultiplier = scaleMultiplier;
    }
    if (typeof offsetX === 'number' && Number.isFinite(offsetX)) {
      this.viewOffsetX = offsetX;
    }
    if (typeof offsetY === 'number' && Number.isFinite(offsetY)) {
      this.viewOffsetY = offsetY;
    }
    if (typeof offsetZ === 'number' && Number.isFinite(offsetZ)) {
      this.viewOffsetZ = offsetZ;
    }

    this._applyCameraFromViewTransform();
    this.requestRender();
  }

  setReferencePaths(segments, options = {}) {
    this.referencePaths = Array.isArray(segments) ? segments : [];
    if (options && typeof options === 'object') {
      if (options.metadata !== undefined) {
        this.referenceMetadata = options.metadata || null;
      }
      if (typeof options.visible === 'boolean') {
        this.referenceRequestedVisible = options.visible;
      }
      if (typeof options.color === 'string' && options.color.length > 0) {
        this.referenceColor = options.color;
      }
    }
    this.referenceVisible = Boolean(this.referenceRequestedVisible) && this.referencePaths.length > 0;
    this.referenceDirty = true;
    this.requestRender();
  }

  setPositionTraceEnabled(enabled, options = {}) {
    this.positionTraceEnabled = Boolean(enabled);
    if (typeof options.color === 'string' && options.color.length > 0) {
      this.positionTraceColor = options.color;
    }
    if (Number.isFinite(options.radiusPx) && options.radiusPx > 0) {
      this.positionTraceRadiusPx = options.radiusPx;
    }
    this.positionTraceMaterial.color.set(this.positionTraceColor);
    if (!this.positionTraceEnabled) {
      this.clearPositionTrace();
      return;
    }
    this.requestRender();
  }

  clearPositionTrace({ keepMarkers = false } = {}) {
    this.drawnPositionTraceCount = 0;
    this._updatePointObject(this.positionTracePointsObject, [], null, DEFAULT_TRACE_Z);
    if (!keepMarkers) {
      this.clearPositionTraceMarkers();
    }
    this.requestRender();
  }

  clearPositionTracePoints() {
    this.positionTracePoints = [];
    this.clearPositionTrace({ keepMarkers: true });
  }

  setNavigationCursorVisible(visible) {
    this.navigationCursorVisible = Boolean(visible);
    if (this.navigationCursorObject) {
      this.navigationCursorObject.visible = this.navigationCursorVisible;
    }
    this.requestRender();
  }

  clearPositionTraceMarkers() {
    this.positionTraceMarkers = [];
    this.drawnPositionTraceMarkerCount = 0;
    this._updatePointObject(this.positionTraceMarkersObject, [], null, DEFAULT_MARKER_Z);
    this.requestRender();
  }

  addPositionTraceMarker(simX, simY, label = '') {
    if (!Number.isFinite(simX) || !Number.isFinite(simY)) {
      return;
    }
    this.positionTraceMarkers.push({
      x: simX,
      y: simY,
      z: DEFAULT_MARKER_Z,
      label
    });
    this._syncPositionTraceMarkers();
    this.requestRender();
  }

  clearExtrusions() {
    this.drawnExtrusionCount = 0;
    this._updatePointObject(this.extrusionPoints, [], [], DEFAULT_TRACE_Z);
    this.requestRender();
  }

  simXFromCanvas(pixelX, pixelY = this.canvas.height * 0.5) {
    return this._projectCanvasToSim(pixelX, pixelY)?.x ?? Number.NaN;
  }

  simYFromCanvas(pixelY, pixelX = this.canvas.width * 0.5) {
    return this._projectCanvasToSim(pixelX, pixelY)?.y ?? Number.NaN;
  }

  setComponentClasses(classes = {}) {
    if (classes.BorderComponent !== undefined) this.borderComponentClass = classes.BorderComponent;
    if (classes.FlipperTagComponent !== undefined) this.flipperTagComponentClass = classes.FlipperTagComponent;
    if (classes.FlipperStateComponent !== undefined) this.flipperStateComponentClass = classes.FlipperStateComponent;
  }

  _clientToCanvasPixels(clientX, clientY) {
    const rect = this.canvas.getBoundingClientRect();
    if (!rect || rect.width <= 0 || rect.height <= 0) {
      return null;
    }
    const width = Math.max(1, this.canvas.width || this.canvas.clientWidth || 1);
    const height = Math.max(1, this.canvas.height || this.canvas.clientHeight || 1);
    return {
      x: (clientX - rect.left) * (width / rect.width),
      y: (clientY - rect.top) * (height / rect.height),
    };
  }

  projectClientToSim(clientX, clientY) {
    const pixel = this._clientToCanvasPixels(clientX, clientY);
    if (!pixel) {
      return null;
    }
    return this._projectCanvasToSim(pixel.x, pixel.y);
  }

  getCameraPlaneNormal() {
    this.camera.getWorldDirection(this._rayPlaneNormal);
    return new Vector3(
      this._rayPlaneNormal.x,
      this._rayPlaneNormal.y,
      this._rayPlaneNormal.z
    );
  }

  projectClientToPlane(clientX, clientY, planePoint, planeNormal = null) {
    const pixel = this._clientToCanvasPixels(clientX, clientY);
    if (!pixel) {
      return null;
    }
    return this._projectCanvasToPlane(pixel.x, pixel.y, planePoint, planeNormal);
  }

  projectCanvasToPlane(pixelX, pixelY, planePoint, planeNormal = null) {
    return this._projectCanvasToPlane(pixelX, pixelY, planePoint, planeNormal);
  }

  projectCanvasToSim(pixelX, pixelY) {
    return this._projectCanvasToSim(pixelX, pixelY);
  }

  _projectCanvasToSim(pixelX, pixelY) {
    return this._projectCanvasToPlane(
      pixelX,
      pixelY,
      { x: 0.0, y: 0.0, z: 0.0 },
      { x: 0.0, y: 0.0, z: 1.0 }
    );
  }

  _projectCanvasToPlane(pixelX, pixelY, planePoint, planeNormal = null) {
    if (!planePoint) {
      return null;
    }
    const width = Math.max(1, this.canvas.width || this.canvas.clientWidth || 1);
    const height = Math.max(1, this.canvas.height || this.canvas.clientHeight || 1);
    this._rayNdc.x = (pixelX / width) * 2 - 1;
    this._rayNdc.y = -(pixelY / height) * 2 + 1;
    this._raycaster.setFromCamera(this._rayNdc, this.camera);

    const normalX = planeNormal?.x ?? 0.0;
    const normalY = planeNormal?.y ?? 0.0;
    const normalZ = planeNormal?.z ?? 1.0;
    this._rayPlaneNormal.set(normalX, normalY, normalZ);
    if (this._rayPlaneNormal.lengthSq() <= EPSILON) {
      return null;
    }
    this._rayPlaneNormal.normalize();
    this._rayPlanePoint.set(
      planePoint.x ?? 0.0,
      planePoint.y ?? 0.0,
      planePoint.z ?? 0.0
    );
    this._rayPlane.setFromNormalAndCoplanarPoint(this._rayPlaneNormal, this._rayPlanePoint);

    const hit = this._raycaster.ray.intersectPlane(this._rayPlane, this._rayHit);
    if (!hit) {
      return null;
    }
    return new Vector3(hit.x, hit.y, hit.z);
  }

  _updatePointObject(object, points, colors = null, defaultZ = 0.0) {
    if (!object) {
      return;
    }

    const safePoints = Array.isArray(points) ? points : [];
    const positionArray = new Float32Array(safePoints.length * 3);
    for (let index = 0; index < safePoints.length; index += 1) {
      const point = safePoints[index];
      const base = index * 3;
      positionArray[base] = point?.x ?? 0.0;
      positionArray[base + 1] = point?.y ?? 0.0;
      positionArray[base + 2] = point?.z ?? defaultZ;
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positionArray, 3));

    if (Array.isArray(colors) && colors.length === safePoints.length) {
      const colorArray = new Float32Array(colors.length * 3);
      for (let index = 0; index < colors.length; index += 1) {
        const color = new THREE.Color(colors[index] || '#ffffff');
        const base = index * 3;
        colorArray[base] = color.r;
        colorArray[base + 1] = color.g;
        colorArray[base + 2] = color.b;
      }
      geometry.setAttribute('color', new THREE.BufferAttribute(colorArray, 3));
      if (object.material) {
        object.material.vertexColors = true;
      }
    } else if (object.material) {
      object.material.vertexColors = false;
    }

    const previous = object.geometry;
    object.geometry = geometry;
    object.visible = safePoints.length > 0;
    if (previous) {
      previous.dispose();
    }
  }

  _syncReferencePaths() {
    if (!this.referenceDirty) {
      return;
    }
    this.referenceVisible = Boolean(this.referenceRequestedVisible) && this.referencePaths.length > 0;
    this.referenceMaterial.color.set(this.referenceColor);
    if (!this.referenceVisible) {
      this.referenceLines.visible = false;
      this.referenceDirty = false;
      return;
    }

    const pointPairs = [];
    for (const segment of this.referencePaths) {
      if (!Array.isArray(segment?.start) || !Array.isArray(segment?.end)) {
        continue;
      }
      pointPairs.push(
        {
          x: finiteOr(segment.start[0], 0.0),
          y: finiteOr(segment.start[1], 0.0),
          z: finiteOr(segment.start[2], 0.0)
        },
        {
          x: finiteOr(segment.end[0], 0.0),
          y: finiteOr(segment.end[1], 0.0),
          z: finiteOr(segment.end[2], 0.0)
        }
      );
    }
    const geometry = new THREE.BufferGeometry();
    const positions = new Float32Array(pointPairs.length * 3);
    for (let index = 0; index < pointPairs.length; index += 1) {
      const point = pointPairs[index];
      const base = index * 3;
      positions[base] = point.x;
      positions[base + 1] = point.y;
      positions[base + 2] = point.z;
    }
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const previous = this.referenceLines.geometry;
    this.referenceLines.geometry = geometry;
    this.referenceLines.visible = pointPairs.length > 0;
    if (previous) {
      previous.dispose();
    }
    this.referenceDirty = false;
  }

  _syncExtrusions(world) {
    const extruderEntities = world.query([ExtruderComponent]);
    if (extruderEntities.length === 0) {
      if (this.drawnExtrusionCount !== 0 || this.extrusionPoints.visible) {
        this.clearExtrusions();
      }
      return;
    }

    const extruderComp = world.getComponent(extruderEntities[0], ExtruderComponent);
    const extrusions = Array.isArray(extruderComp?.extrusions) ? extruderComp.extrusions : [];
    if (extrusions.length === this.drawnExtrusionCount && this.extrusionPoints.visible === (extrusions.length > 0)) {
      return;
    }

    const points = [];
    const colors = [];
    for (const extrusion of extrusions) {
      if (!Array.isArray(extrusion?.pos) || extrusion.pos.length < 2) {
        continue;
      }
      const machineTip = extrusion?.machineId
        ? extruderComp?.machineTips?.[extrusion.machineId] || extruderComp?.machineCenters?.[extrusion.machineId]
        : extruderComp?.tipPos || extruderComp?.centerPos;
      points.push({
        x: finiteOr(extrusion.pos[0], finiteOr(machineTip?.x, 0.0)),
        y: finiteOr(extrusion.pos[1], finiteOr(machineTip?.y, 0.0)),
        z: finiteOr(extrusion.pos[2], finiteOr(machineTip?.z, DEFAULT_TRACE_Z))
      });
      colors.push(extrusion.qualityColor || extrusion.color || DEFAULT_CABLE_COLOR);
    }
    this.drawnExtrusionCount = extrusions.length;
    this._updatePointObject(this.extrusionPoints, points, colors, DEFAULT_TRACE_Z);
  }

  _syncPositionTrace(world) {
    if (!this.positionTraceEnabled) {
      if (this.positionTracePointsObject.visible) {
        this._updatePointObject(this.positionTracePointsObject, [], null, DEFAULT_TRACE_Z);
      }
      return;
    }

    const extruderEntities = world.query([ExtruderComponent]);
    if (extruderEntities.length > 0) {
      const extruderComp = world.getComponent(extruderEntities[0], ExtruderComponent);
      const tip = extruderComp?.tipPos || extruderComp?.centerPos;
      const tipZ = finiteOr(tip?.z, DEFAULT_TRACE_Z);
      if (tip && Number.isFinite(tip.x) && Number.isFinite(tip.y)) {
        const last = this.positionTracePoints[this.positionTracePoints.length - 1];
        if (
          !last
          || Math.abs(last.x - tip.x) > 1e-9
          || Math.abs(last.y - tip.y) > 1e-9
          || Math.abs((last.z ?? DEFAULT_TRACE_Z) - tipZ) > 1e-9
        ) {
          this.positionTracePoints.push({ x: tip.x, y: tip.y, z: tipZ });
        }
      }
    }

    if (this.positionTracePoints.length !== this.drawnPositionTraceCount) {
      this.drawnPositionTraceCount = this.positionTracePoints.length;
      this.positionTraceMaterial.color.set(this.positionTraceColor);
      this._updatePointObject(this.positionTracePointsObject, this.positionTracePoints, null, DEFAULT_TRACE_Z);
    }
  }

  _syncPositionTraceMarkers() {
    if (
      this.positionTraceMarkers.length === this.drawnPositionTraceMarkerCount
      && this.positionTraceMarkersObject.visible === (this.positionTraceMarkers.length > 0)
    ) {
      return;
    }
    this.drawnPositionTraceMarkerCount = this.positionTraceMarkers.length;
    this._updatePointObject(this.positionTraceMarkersObject, this.positionTraceMarkers, null, DEFAULT_MARKER_Z);
  }

  _syncExtruder(world) {
    const extruderEntities = world.query([ExtruderComponent]);
    if (extruderEntities.length === 0) {
      if (this.extruderLine) {
        this.extruderLine.visible = false;
      }
      return;
    }

    const extruderComp = world.getComponent(extruderEntities[0], ExtruderComponent);
    const lineStart = extruderComp?.coldEndPos || extruderComp?.centerPos || null;
    const lineEnd = extruderComp?.tipPos || extruderComp?.centerPos || null;
    if (!lineStart || !lineEnd) {
      if (this.extruderLine) {
        this.extruderLine.visible = false;
      }
      return;
    }

    if (!this.extruderLine) {
      this.extruderLine = this._createLine();
      this.extruderLine.renderOrder = 875;
      this.root.add(this.extruderLine);
    }

    this.extruderLine.visible = true;
    setMaterialColor(this.extruderLine.material, DEFAULT_EXTRUDER_COLOR);

    const positions = this.extruderLine.geometry.attributes.position.array;
    const segments = this.extruderLine.userData.lineSegments ?? JOINT_LINE_SEGMENTS;
    writeStraightCablePositions(
      positions,
      new Vector3(lineStart.x, lineStart.y, lineStart.z || 0.0),
      new Vector3(lineEnd.x, lineEnd.y, lineEnd.z || 0.0),
      segments
    );
    this.extruderLine.geometry.attributes.position.needsUpdate = true;
  }

  update(world, dt = 0) {
    if (world) {
      this._lastWorld = world;
    }
    if (this.drawingSuspended) {
      return;
    }
    if (!this.renderOnSimulationStep && Number.isFinite(dt) && dt > 0) {
      return;
    }

    this._syncBoard(world);
    this._syncBorder(world);
    this._syncCircles(world);
    this._syncFlippers(world);
    this._syncRigidGroups(world);
    this._syncExtruder(world);
    this._syncCable(world);
    this._syncReferencePaths();
    this._syncExtrusions(world);
    this._syncPositionTrace(world);
    this._syncPositionTraceMarkers();

    this._updateBumperHitFx(world);

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

    for (const marker of this.knotMarkers) {
      this.root.remove(marker);
    }
    this.knotMarkers.length = 0;

    for (const line of this.rigidGroupLines) {
      this.root.remove(line);
      disposeObject(line);
    }
    this.rigidGroupLines.length = 0;

    if (this.extruderLine) {
      this.root.remove(this.extruderLine);
      disposeObject(this.extruderLine);
      this.extruderLine = null;
    }

    this._clearBumperFx();

    if (this.borderLine) {
      this.root.remove(this.borderLine);
      disposeObject(this.borderLine);
      this.borderLine = null;
      this.borderVertexCount = 0;
    }
    this._hideBorderFloor();
    this._clearBorderWalls();
    this.clearExtrusions();
    this.clearPositionTrace();
    this.clearPositionTraceMarkers();
    this.referencePaths = [];
    this.referenceVisible = false;
    this.referenceRequestedVisible = false;
    this.referenceDirty = true;
    this._syncReferencePaths();
  }

  dispose() {
    this.clearAnimationLoop();
    if (this._requestRenderHandle != null && typeof cancelAnimationFrame === 'function') {
      cancelAnimationFrame(this._requestRenderHandle);
      this._requestRenderHandle = null;
    }
    this.resetVisuals();

    for (const material of this.sphereMaterialCache.values()) {
      material.dispose();
    }
    this.sphereMaterialCache.clear();

    for (const material of this.flipperMaterialCache.values()) {
      material.dispose();
    }
    this.flipperMaterialCache.clear();

    for (const geometry of this.orientedSphereGeometryCache.values()) {
      geometry.dispose();
    }
    this.orientedSphereGeometryCache.clear();

    this.orientedSphereMaterial.dispose();
    this.knotMarkerMaterial.dispose();
    this.sharedSphereGeometry.dispose();
    this.sharedFlipperBarGeometry.dispose();

    if (this.board && this.scene) {
      this.scene.remove(this.board);
      disposeObject(this.board);
    }
    if (this.boardOutline && this.scene) {
      this.scene.remove(this.boardOutline);
      disposeObject(this.boardOutline);
    }

    if (this.borderFloor) {
      this.scene.remove(this.borderFloor);
      disposeObject(this.borderFloor);
    }
    if (this.borderWallGroup) {
      this._clearBorderWalls();
      this.scene.remove(this.borderWallGroup);
      disposeObject(this.borderWallGroup);
    }
    this.borderFloorMaterial.dispose();
    this.borderWallMaterial.dispose();

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

    this._clearBumperFx();
    if (this._bumperFxGeometry) {
      this._bumperFxGeometry.dispose();
    }
    if (this._bumperFxGroup && this.scene) {
      this.scene.remove(this._bumperFxGroup);
    }
    if (this.referenceLines && this.scene) {
      this.scene.remove(this.referenceLines);
      disposeObject(this.referenceLines);
    }
    if (this.extrusionPoints && this.scene) {
      this.scene.remove(this.extrusionPoints);
      disposeObject(this.extrusionPoints);
    }
    if (this.positionTracePointsObject && this.scene) {
      this.scene.remove(this.positionTracePointsObject);
      disposeObject(this.positionTracePointsObject);
    }
    if (this.positionTraceMarkersObject && this.scene) {
      this.scene.remove(this.positionTraceMarkersObject);
      disposeObject(this.positionTraceMarkersObject);
    }
    if (this.navigationCursorObject && this.scene) {
      this.scene.remove(this.navigationCursorObject);
      disposeObject(this.navigationCursorObject);
    }
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

  _nowSeconds() {
    if (typeof performance !== 'undefined' && typeof performance.now === 'function') {
      return performance.now() / 1000;
    }
    return Date.now() / 1000;
  }

  _spawnBumperHitFx(world, contact, pushVel) {
    const obsPosComp = world.getComponent(contact.obs_id, PositionComponent);
    if (!obsPosComp) {
      return;
    }
    const obsRadiusComp = world.getComponent(contact.obs_id, RadiusComponent);
    const obsRadius = obsRadiusComp && Number.isFinite(obsRadiusComp.radius)
      ? Math.max(0.0, obsRadiusComp.radius)
      : 0.04;
    const direction = new Vector3(1.0, 0.0, 0.0);
    if (contact?.direction && contact.direction.lengthSq() > EPSILON) {
      direction.set(contact.direction);
    }

    const deltaLambda = Number.isFinite(contact?.delta_lambda) ? Math.max(0.0, contact.delta_lambda) : 0.0;
    const intensity = Math.max(
      0.7,
      Math.min(2.8, 0.85 + (0.2 * pushVel) + (0.32 * Math.sqrt(deltaLambda + 1e-9)))
    );
    const effectRadius = Math.max(BUMPER_FX_MIN_RADIUS, obsRadius * (0.7 + 0.12 * intensity));
    const renderComp = world.getComponent(contact.obs_id, RenderableComponent);
    const color = renderComp?.color ?? '#ffd34d';
    const material = new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: 0.0,
      depthWrite: false,
      blending: THREE.AdditiveBlending,
      toneMapped: false
    });
    const mesh = new THREE.Mesh(this._bumperFxGeometry, material);
    mesh.frustumCulled = false;
    mesh.renderOrder = 1000;
    mesh.position.copy(obsPosComp.pos);
    mesh.scale.setScalar(0.001);
    mesh.userData.ownsMaterial = true;
    mesh.userData.ownsGeometry = false;
    this._bumperFxGroup.add(mesh);

    const burst = {
      mesh,
      origin: obsPosComp.pos.clone(),
      direction,
      age: 0.0,
      life: 0.32 + (0.05 * intensity),
      targetScale: effectRadius,
      intensity
    };
    this._bumperFxBursts.push(burst);

    if (this._bumperFxBursts.length > BUMPER_FX_MAX_BURSTS) {
      const overflow = this._bumperFxBursts.length - BUMPER_FX_MAX_BURSTS;
      for (let i = 0; i < overflow; i += 1) {
        const oldest = this._bumperFxBursts.shift();
        if (oldest) {
          this._removeBurst(oldest);
        }
      }
    }
  }

  _removeBurst(burst) {
    if (!burst || !burst.mesh) {
      return;
    }
    this._bumperFxGroup.remove(burst.mesh);
    disposeObject(burst.mesh);
    burst.mesh = null;
  }

  _clearBumperFx() {
    for (const burst of this._bumperFxBursts) {
      this._removeBurst(burst);
    }
    this._bumperFxBursts.length = 0;
    this._bumperFxLastTimeSec = Number.NaN;
  }

  _updateBumperHitFx(world) {
    const enabled = world?.getResource?.('renderBumperHitFx') === true;
    if (!enabled) {
      this._clearBumperFx();
      return;
    }

    const nowSec = this._nowSeconds();
    if (!Number.isFinite(this._bumperFxLastTimeSec)) {
      this._bumperFxLastTimeSec = nowSec;
    }
    let dtSec = nowSec - this._bumperFxLastTimeSec;
    this._bumperFxLastTimeSec = nowSec;
    if (!Number.isFinite(dtSec) || dtSec < 0.0) {
      dtSec = 0.0;
    }
    dtSec = Math.min(dtSec, 0.05);

    const contacts = Array.isArray(world.getResource('ball_obstacle_contacts'))
      ? world.getResource('ball_obstacle_contacts')
      : [];
    for (const contact of contacts) {
      if (!contact || contact.raw_hit !== true) {
        continue;
      }
      const pushComp = world.getComponent(contact.obs_id, ObstaclePushComponent);
      if (!pushComp) {
        continue;
      }
      this._spawnBumperHitFx(world, contact, pushComp.pushVel);
    }

    const survivingBursts = [];
    for (const burst of this._bumperFxBursts) {
      burst.age += dtSec;
      const progress = burst.life > 0 ? Math.min(1.0, burst.age / burst.life) : 1.0;
      const ease = Math.min(1.0, progress * 1.2);
      const scale = THREE.MathUtils.lerp(0.01 * burst.targetScale, burst.targetScale, ease);
      if (burst.mesh) {
        burst.mesh.scale.set(scale, scale, scale);
        if (burst.mesh.material) {
          burst.mesh.material.opacity = Math.max(0.0, Math.min(1.0, (1.0 - progress) * 0.85));
        }
        const offset = Math.min(0.15, scale * 0.35) * (1.0 - progress);
        if (!burst._offset) {
          burst._offset = new Vector3();
        }
        burst._offset.set(burst.direction);
        burst._offset.scale(offset);
        burst.mesh.position.copy(burst.origin).add(burst._offset);
      }
      if (burst.age < burst.life) {
        survivingBursts.push(burst);
      } else {
        this._removeBurst(burst);
      }
    }
    this._bumperFxBursts = survivingBursts;
  }

  _syncBoard(world) {
    const halfExtent = this._computePrintSurfaceHalfExtent(world);
    const surfaceSize = halfExtent * 2.0;
    this.board.scale.set(surfaceSize, surfaceSize, 1);
    this.board.position.set(0.0, 0.0, PRINT_SURFACE_Z);
    this.boardOutline.scale.set(surfaceSize, surfaceSize, 1);
    this.boardOutline.position.set(0.0, 0.0, PRINT_SURFACE_OUTLINE_Z);
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
      this._hideBorderFloor();
      this._clearBorderWalls();
      if (this.borderLine) {
        this.borderLine.visible = false;
      }
      return;
    }

    const shape = RenderSystem3D.buildBorderShape(points);
    if (shape) {
      this._updateBorderFloor(shape, renderComp);
      this._updateBorderWalls(points);
    } else {
      this._hideBorderFloor();
      this._clearBorderWalls();
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

  _updateBorderFloor(shape, renderComp) {
    if (!shape || !this.borderFloor) {
      this._hideBorderFloor();
      return;
    }

    const geometry = new THREE.ShapeGeometry(shape);
    if (this.borderFloor.geometry) {
      this.borderFloor.geometry.dispose();
    }
    this.borderFloor.geometry = geometry;
    this.borderFloor.visible = true;
    setMaterialColor(this.borderFloorMaterial, renderComp?.color ?? BORDER_FLOOR_COLOR_HEX);
  }

  _hideBorderFloor() {
    if (!this.borderFloor) return;
    this.borderFloor.visible = false;
    if (this.borderFloor.geometry) {
      this.borderFloor.geometry.dispose();
      this.borderFloor.geometry = new THREE.BufferGeometry();
    }
  }

  _updateBorderWalls(points) {
    if (!this.borderWallGroup) return;
    this._clearBorderWalls();
    if (!points || points.length < 2) {
      return;
    }

    for (let i = 0; i < points.length; i += 1) {
      const start = points[i];
      const end = points[(i + 1) % points.length];
      const dx = end.x - start.x;
      const dy = end.y - start.y;
      const length = Math.hypot(dx, dy);
      if (length <= EPSILON) {
        continue;
      }

      const geometry = new THREE.BoxGeometry(length, BORDER_WALL_THICKNESS, BORDER_WALL_HEIGHT);
      const wall = new THREE.Mesh(geometry, this.borderWallMaterial);
      wall.userData.ownsGeometry = true;
      wall.userData.ownsMaterial = false;
      wall.position.set(
        (start.x + end.x) * 0.5,
        (start.y + end.y) * 0.5,
        BORDER_FLOOR_OFFSET_Z + (BORDER_WALL_HEIGHT / 2)
      );
      wall.rotation.set(0, 0, Math.atan2(dy, dx));
      wall.frustumCulled = false;
      this.borderWallGroup.add(wall);
    }
  }

  _clearBorderWalls() {
    if (!this.borderWallGroup) return;
    while (this.borderWallGroup.children.length > 0) {
      const child = this.borderWallGroup.children.pop();
      disposeObject(child);
    }
  }

  _syncRigidGroups(world) {
    const rigidGroups = world.query([RigidGroupComponent]);
    if (!rigidGroups || rigidGroups.length === 0) {
      this._hideLines(this.rigidGroupLines);
      return;
    }

    const lineSpecs = [];
    for (const groupId of rigidGroups) {
      const group = world.getComponent(groupId, RigidGroupComponent);
      const members = Array.isArray(group?.members) ? group.members : [];
      if (members.length < 2) {
        continue;
      }

      const memberPositions = members.map((entityId) => world.getComponent(entityId, PositionComponent)?.pos || null);
      const edges = collectRigidGroupEdges(members.length, group?.renderSegments);
      const renderComp = world.getComponent(groupId, RenderableComponent);
      const color = renderComp?.color || DEFAULT_RIGID_GROUP_COLOR;

      for (const [aIdx, bIdx] of edges) {
        const pA = memberPositions[aIdx];
        const pB = memberPositions[bIdx];
        if (!pA || !pB) {
          continue;
        }
        lineSpecs.push({ pA, pB, color });
      }
    }

    this._ensureLineCapacity(this.rigidGroupLines, lineSpecs.length, false);
    for (let i = 0; i < this.rigidGroupLines.length; i += 1) {
      const line = this.rigidGroupLines[i];
      const spec = lineSpecs[i];
      if (!spec) {
        line.visible = false;
        continue;
      }

      line.visible = true;
      setMaterialColor(line.material, spec.color);

      const positions = line.geometry.attributes.position.array;
      const segments = line.userData.lineSegments ?? JOINT_LINE_SEGMENTS;
      const lineStart = new Vector3(spec.pA.x, spec.pA.y, spec.pA.z || 0.0);
      const lineEnd = new Vector3(spec.pB.x, spec.pB.y, spec.pB.z || 0.0);
      writeStraightCablePositions(positions, lineStart, lineEnd, segments);
      line.geometry.attributes.position.needsUpdate = true;
    }
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

      const orientationComp = world.getComponent(entityId, OrientationComponent);
      const geometry = orientationComp?.quaternion
        ? this._getOrientedSphereGeometry(renderComp?.color)
        : this.sharedSphereGeometry;
      if (mesh.geometry !== geometry) {
        mesh.geometry = geometry;
      }

      const material = orientationComp?.quaternion
        ? this.orientedSphereMaterial
        : this._getSphereMaterial(renderComp.color);
      if (mesh.material !== material) {
        mesh.material = material;
      }

      mesh.position.set(pos.x, pos.y, pos.z);
      mesh.scale.set(radius, radius, radius);

      if (orientationComp?.quaternion) {
        const q = orientationComp.quaternion;
        mesh.quaternion.set(q.x, q.y, q.z, q.w);
      } else {
        mesh.quaternion.identity();
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
    const upDirection = getUpDirection(world);

    if (pathEntities.length === 0) {
      this._hideLines(this.jointLines);
      this._hideLines(this.wrapArcs);
      this._hideMeshes(this.knotMarkers);
      return;
    }

    const jointSpecs = [];
    const arcSpecs = [];
    const knotSpecs = [];

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

      const startKnotSpec = this._buildKnotMarkerSpec(world, path, pathId, 0);
      if (startKnotSpec) {
        knotSpecs.push(startKnotSpec);
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

      if (nLinks > 1) {
        const endKnotSpec = this._buildKnotMarkerSpec(world, path, pathId, nLinks - 1);
        if (endKnotSpec) {
          knotSpecs.push(endKnotSpec);
        }
      }
    }

    this._ensureLineCapacity(this.jointLines, jointSpecs.length, false);
    this._ensureLineCapacity(this.wrapArcs, arcSpecs.length, true);
    this._ensureKnotMarkerCapacity(knotSpecs.length);

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
      const segments = line.userData.lineSegments ?? JOINT_LINE_SEGMENTS;
      writeSlackCablePositions(
        p,
        spec.joint.attachmentPointA_world,
        spec.joint.attachmentPointB_world,
        spec.joint.restLength,
        upDirection,
        segments
      );
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

    for (let i = 0; i < this.knotMarkers.length; i++) {
      const marker = this.knotMarkers[i];
      const spec = knotSpecs[i];
      if (!spec) {
        marker.visible = false;
        continue;
      }

      marker.visible = true;
      marker.position.set(spec.position.x, spec.position.y, spec.position.z);
      marker.scale.setScalar(spec.radius);
    }
  }

  _buildKnotMarkerSpec(world, path, pathId, endpointIndex) {
    if (!path || !Array.isArray(path.linkTypes) || !Array.isArray(path.jointEntities) || path.jointEntities.length < 1) {
      return null;
    }
    if (endpointIndex !== 0 && endpointIndex !== path.linkTypes.length - 1) {
      return null;
    }

    const linkType = path.linkTypes[endpointIndex];
    if (linkType !== 'hybrid' && linkType !== 'hybrid-attachment') {
      return null;
    }

    const isStart = endpointIndex === 0;
    const jointId = isStart ? path.jointEntities[0] : path.jointEntities[path.jointEntities.length - 1];
    const joint = world.getComponent(jointId, CableJointComponent);
    if (!joint) {
      return null;
    }

    const entityId = isStart ? joint.entityA : joint.entityB;
    const attachmentPoint = isStart ? joint.attachmentPointA_world : joint.attachmentPointB_world;
    if (!attachmentPoint) {
      return null;
    }
    if (linkType === 'hybrid-attachment') {
      return { position: attachmentPoint.clone(), radius: KNOT_MARKER_RADIUS };
    }

    const center = world.getComponent(entityId, PositionComponent)?.pos;
    const baseRadius = world.getComponent(entityId, RadiusComponent)?.radius;
    if (!center || !Number.isFinite(baseRadius) || !(baseRadius > EPSILON)) {
      return null;
    }

    const planeNormal = world.getComponent(entityId, CableLinkComponent)?.cablePlaneNormal || this.defaultPlaneNormal;
    const basis = buildPlaneBasis(planeNormal);
    const halfWidth = layeringEnabled(world) ? Math.max(0.0, path.cableHalfWidth ?? 0.0) : 0.0;
    const stored = Math.max(0.0, path.stored?.[endpointIndex] ?? 0.0);
    const baseRenderRadius = baseRadius + halfWidth;
    const tangentAngle = angleOnPlane(attachmentPoint, center, basis);
    const deltaAngle = knotMarkerDeltaAngle(stored, baseRenderRadius, halfWidth, layeringEnabled(world));
    const worldAngle = Boolean(path.cw?.[endpointIndex])
      ? (tangentAngle - deltaAngle)
      : (tangentAngle + deltaAngle);
    const cos = Math.cos(worldAngle);
    const sin = Math.sin(worldAngle);

    return {
      position: new Vector3(
        center.x + basis.u.x * baseRenderRadius * cos + basis.v.x * baseRenderRadius * sin,
        center.y + basis.u.y * baseRenderRadius * cos + basis.v.y * baseRenderRadius * sin,
        center.z + basis.u.z * baseRenderRadius * cos + basis.v.z * baseRenderRadius * sin
      ),
      radius: KNOT_MARKER_RADIUS
    };
  }

  _ensureLineCapacity(target, count, isArc) {
    while (target.length < count) {
      const line = isArc ? this._createArcLine() : this._createLine();
      target.push(line);
      this.root.add(line);
    }
  }

  _ensureKnotMarkerCapacity(count) {
    while (this.knotMarkers.length < count) {
      const marker = new THREE.Mesh(this.sharedSphereGeometry, this.knotMarkerMaterial);
      marker.frustumCulled = false;
      this.knotMarkers.push(marker);
      this.root.add(marker);
    }
  }

  _createLine() {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array((JOINT_LINE_SEGMENTS + 1) * 3), 3));

    const material = new THREE.LineBasicMaterial({ color: DEFAULT_CABLE_COLOR });
    material.userData.__color = DEFAULT_CABLE_COLOR;

    const line = new THREE.Line(geometry, material);
    line.frustumCulled = false;
    line.userData.ownsGeometry = true;
    line.userData.ownsMaterial = true;
    line.userData.lineSegments = JOINT_LINE_SEGMENTS;
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

  _hideMeshes(meshes) {
    for (const mesh of meshes) {
      mesh.visible = false;
    }
  }

  _getOrientedSphereGeometry(color) {
    const key = typeof color === 'string' && color.length > 0 ? color : '#a0a0a0';
    let geometry = this.orientedSphereGeometryCache.get(key);
    if (!geometry) {
      geometry = createDualColorSphereGeometry(this.sharedSphereGeometry, key);
      this.orientedSphereGeometryCache.set(key, geometry);
    }
    return geometry;
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

  _computePrintSurfaceHalfExtent(world) {
    let maxAbs = PRINT_SURFACE_MIN_HALF_EXTENT;

    const entities = world.query([PositionComponent, RadiusComponent]);
    for (const entityId of entities) {
      const pos = world.getComponent(entityId, PositionComponent)?.pos;
      if (!pos) {
        continue;
      }
      const radius = world.getComponent(entityId, RadiusComponent)?.radius;
      const pad = Number.isFinite(radius) && radius > 0 ? radius : 0.0;
      maxAbs = Math.max(
        maxAbs,
        Math.abs(pos.x) + pad,
        Math.abs(pos.y) + pad
      );
    }

    if (this.borderComponentClass) {
      const borderEntities = world.query([this.borderComponentClass]);
      for (const entityId of borderEntities) {
        const border = world.getComponent(entityId, this.borderComponentClass);
        const points = Array.isArray(border?.points) ? border.points : [];
        for (const point of points) {
          maxAbs = Math.max(maxAbs, Math.abs(point.x || 0.0), Math.abs(point.y || 0.0));
        }
      }
    }

    return maxAbs + PRINT_SURFACE_MARGIN;
  }

  _setOrbitFromOffset(offset) {
    const distance = Math.max(0.1, offset.length());
    const radialXY = Math.hypot(offset.x, offset.y);
    this.orbitAzimuth = Math.atan2(offset.y, offset.x);
    this.orbitPolar = THREE.MathUtils.clamp(
      Math.atan2(radialXY, offset.z),
      this.orbitMinPolarAngle,
      this.orbitMaxPolarAngle
    );
    this._baseCameraDistance = distance;
  }

  _cameraDistanceForScale(scaleMultiplier = this.viewScaleMultiplier) {
    const scale = Math.max(0.05, scaleMultiplier);
    return Math.max(0.08, this._baseCameraDistance / scale);
  }

  _getViewTarget() {
    const target = this._baseViewTarget.clone();
    target.x += finiteOr(this.viewOffsetX, 0.0);
    target.y += finiteOr(this.viewOffsetY, 0.0);
    target.z += finiteOr(this.viewOffsetZ, 0.0);
    return target;
  }

  getViewPlaneMetrics(scaleMultiplier = this.viewScaleMultiplier) {
    const height = Math.max(1, this.canvas.height || this.canvas.clientHeight || 1);
    const distance = this._cameraDistanceForScale(scaleMultiplier);
    const worldUnitsPerPixel = (2 * distance * Math.tan(THREE.MathUtils.degToRad(this.camera.fov) * 0.5)) / height;
    const right = new THREE.Vector3(1, 0, 0).applyQuaternion(this.camera.quaternion).normalize();
    const up = new THREE.Vector3(0, 1, 0).applyQuaternion(this.camera.quaternion).normalize();
    const target = this._getViewTarget();

    return {
      target: new Vector3(target.x, target.y, target.z),
      right: new Vector3(right.x, right.y, right.z),
      up: new Vector3(up.x, up.y, up.z),
      worldUnitsPerPixel,
    };
  }

  _buildOrbitOffset(distance) {
    const radialXY = Math.sin(this.orbitPolar) * distance;
    return new THREE.Vector3(
      Math.cos(this.orbitAzimuth) * radialXY,
      Math.sin(this.orbitAzimuth) * radialXY,
      Math.cos(this.orbitPolar) * distance
    );
  }

  _applyCameraFromViewTransform() {
    const target = this._getViewTarget();

    const cameraOffset = this._buildOrbitOffset(this._cameraDistanceForScale());
    this.camera.up.set(0, 0, 1);
    this.camera.position.copy(target).add(cameraOffset);
    this.camera.lookAt(target);
    this.camera.updateMatrixWorld();

    if (this.controls) {
      this.controls.target.copy(target);
      this.controls.update();
    }

    if (this.navigationCursorObject) {
      const size = Math.max(
        DEFAULT_NAV_CURSOR_MIN_SIZE,
        this.getViewPlaneMetrics().worldUnitsPerPixel * DEFAULT_NAV_CURSOR_PIXEL_SIZE
      );
      this.navigationCursorObject.position.copy(target);
      this.navigationCursorObject.scale.setScalar(size);
      this.navigationCursorObject.visible = this.navigationCursorVisible;
    }

    this._fixedCameraPosition = this.camera.position.clone();
    this._fixedCameraQuaternion = this.camera.quaternion.clone();
  }

  rotateOrbitByPixels(deltaX, deltaY) {
    if (!Number.isFinite(deltaX) || !Number.isFinite(deltaY)) {
      return;
    }

    const width = Math.max(1, this.canvas.clientWidth || this.canvas.width || 1);
    const height = Math.max(1, this.canvas.clientHeight || this.canvas.height || 1);
    const azimuthDelta = (deltaX / width) * Math.PI * this.orbitRotateSpeed;
    const polarDelta = (-deltaY / height) * Math.PI * this.orbitRotateSpeed;

    this.orbitAzimuth -= azimuthDelta;
    this.orbitPolar = THREE.MathUtils.clamp(
      this.orbitPolar + polarDelta,
      this.orbitMinPolarAngle,
      this.orbitMaxPolarAngle
    );
    this._applyCameraFromViewTransform();
    this.requestRender();
  }
}
