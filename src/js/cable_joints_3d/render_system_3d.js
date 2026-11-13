import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

import Vector3 from './vector3.js';
import { writeSlackCablePositions } from './cable_shape.js';
import { Stage3D } from './stage_3d.js';
import { CableLineBatch } from './cable_lines_3d.js';
import { DEFAULT_COLOR_THEME, resolveColorTheme } from './color_themes.js';
import { MachineDetails3D } from './machine_details_3d.js';
import {
  PositionComponent,
  RadiusComponent,
  OrientationComponent,
  ObstaclePushComponent,
  RenderableComponent,
  RigidBodyComponent,
  RigidGroupComponent,
  layeringEnabled
} from './ecs.js';
import {
  CablePathComponent,
  CableJointComponent,
  CableLinkComponent,
} from './cable_joints_core.js';
import { ExtruderComponent } from '../../../hp-sim-3d/app/hangprinter_extruder.js';
import {
  SpoolStateComponent,
  getSpoolLocalAxis,
} from '../../../hp-sim-3d/app/hangprinter_spools.js';

const EPSILON = 1e-9;
const ARC_SEGMENTS = 48;
const JOINT_LINE_SEGMENTS = 24;
const CABLE_LINE_WIDTH_PX = 1.6;
const AXIS_HALF_LENGTH = 2.5;
const DEFAULT_CABLE_COLOR = '#ffd34d';
const DEFAULT_PLANE_NORMAL = new Vector3(0, 0, 1);
const ORIENTATION_BACK_COLOR = '#2a3542';
const KNOT_MARKER_COLOR = '#ff3b30';
const KNOT_MARKER_RADIUS = 0.002;
const FORCE_SIGN_FALLBACK_FLOAT_OFFSET = 0.024;
const FORCE_SIGN_WIDTH = 0.11*0.5;
const FORCE_SIGN_HEIGHT = 0.034*0.6;
const FORCE_SIGN_WIDTH_PX = 96*0.5;
const FORCE_SIGN_HEIGHT_PX = 30*0.6;
const FORCE_SIGN_JOINT_CLEARANCE_PX = 10;
const FORCE_SIGN_CONNECTOR_COLOR = '#ffd34d';
const FORCE_SIGN_SPREAD_MARGIN = 0.004;
const FORCE_SIGN_SPREAD_MARGIN_PX = 8;
const FORCE_SIGN_SPREAD_ITERATIONS = 10;
const FORCE_SIGN_TEXTURE_WIDTH = 256*0.55;
const FORCE_SIGN_TEXTURE_HEIGHT = 80*0.65;
const PRINT_SURFACE_Z = 0.0;
const MACHINE_MIN_HALF_EXTENT = 0.18;
const MACHINE_EXTENT_MARGIN = 0.08;
const STAGE_FLOOR_CLEARANCE = 0.02;
const BORDER_FLOOR_OFFSET_Z = -0.035;
const BORDER_FLOOR_COLOR = 0x1d2434;
const BORDER_FLOOR_COLOR_HEX = '#1d2434';
const BORDER_WALL_HEIGHT = 0.08;
const BORDER_WALL_THICKNESS = 0.012;
const BORDER_WALL_COLOR = 0x0c111f;
const BORDER_WALL_COLOR_HEX = '#0c111f';
const DEFAULT_EXTRUDER_COLOR = '#dda46b';
const BUMPER_FX_MAX_BURSTS = 96;
const BUMPER_FX_MIN_RADIUS = 0.03;
const DEFAULT_REFERENCE_COLOR = '#79b1e0';
const DEFAULT_TRACE_COLOR = '#ffffff';
const DEFAULT_TRACE_MARKER_COLOR = '#76c5bb';
const DEFAULT_NAV_CURSOR_COLOR = '#ffd34d';
const DEFAULT_TRACE_POINT_SIZE = 3;
const EXTRUSION_BEAD_WIDTH = 0.0008;
const EXTRUSION_MIN_PIXEL_SIZE = 3;
const DEFAULT_TRACE_Z = 0.0;
const DEFAULT_MARKER_Z = 0.005;
const TRACE_LABEL_TEXTURE_WIDTH = 256*1.3;
const TRACE_LABEL_TEXTURE_HEIGHT = 64;
const TRACE_LABEL_WIDTH_PX = 126*1.3;
const TRACE_LABEL_HEIGHT_PX = 32;
const TRACE_LABEL_OFFSET_PX = 14;
const TRACE_PREVIEW_POINT_SIZE = DEFAULT_TRACE_POINT_SIZE;
const TRACE_ENTITY_SNAP_RADIUS_PX = 12;
const DEFAULT_NAV_CURSOR_MIN_SIZE = 0.0012;
const DEFAULT_NAV_CURSOR_PIXEL_SIZE = 18;
const DEFAULT_ORBIT_AZIMUTH = -Math.PI * 0.25;
const DEFAULT_ORBIT_POLAR = 1.05;
const CAMERA_MIN_NEAR = 0.0001;
const CAMERA_NEAR_PER_DISTANCE = 0.004;
const CYLINDER_LOCAL_UP = new THREE.Vector3(0, 1, 0);

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

function getRenderPlaneNormal(world, entityId, defaultPlaneNormal) {
  const linkComp = world.getComponent(entityId, CableLinkComponent);
  if (linkComp?.cablePlaneNormalLocal) {
    const localNormal = linkComp.cablePlaneNormalLocal.clone();
    const orientation = world.getComponent(entityId, OrientationComponent)?.quaternion;
    if (orientation && typeof orientation.transformVector === 'function') {
      const worldNormal = orientation.transformVector(localNormal);
      if (worldNormal.lengthSq() > EPSILON) {
        return worldNormal.normalize();
      }
    }
    if (localNormal.lengthSq() > EPSILON) {
      return localNormal.normalize();
    }
  }

  if (linkComp?.cablePlaneNormal && linkComp.cablePlaneNormal.lengthSq() > EPSILON) {
    return linkComp.cablePlaneNormal.clone().normalize();
  }

  return defaultPlaneNormal.clone().normalize();
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

// Writes the arc with segments proportional to its sweep (ARC_SEGMENTS per
// full turn, at most ARC_SEGMENTS) and returns the number of points written.
function writeArcPositions(positions, center, radius, basis, startAngle, endAngle, cw) {
  const twoPi = Math.PI * 2;

  let end = endAngle;
  if (cw) {
    while (end > startAngle) end -= twoPi;
  } else {
    while (end < startAngle) end += twoPi;
  }

  const sweep = end - startAngle;
  const segments = Math.min(ARC_SEGMENTS, Math.max(2, Math.ceil((Math.abs(sweep) / twoPi) * ARC_SEGMENTS)));
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
  return segments + 1;
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

function createDualColorCylinderGeometry(baseGeometry, primaryColor, secondaryColor = ORIENTATION_BACK_COLOR) {
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
        child.material.forEach((mat) => {
          if (mat.map && child.userData.ownsTexture) {
            mat.map.dispose();
          }
          mat.dispose();
        });
      } else {
        if (child.material.map && child.userData.ownsTexture) {
          child.material.map.dispose();
        }
        child.material.dispose();
      }
    }
  });
}

function formatConstraintForce(forceN) {
  const value = Number.isFinite(forceN) ? Math.max(0.0, forceN) : 0.0;
  if (value >= 1000.0) {
    return `${(value / 1000.0).toFixed(1)} kN`;
  }
  if (value >= 100.0) {
    return `${value.toFixed(0)} N`;
  }
  if (value >= 10.0) {
    return `${value.toFixed(1)} N`;
  }
  return `${value.toFixed(2)} N`;
}

function totalConstraintForceMagnitude(joint) {
  const direct = Number.isFinite(joint?.constraintForceMagnitude)
    ? Math.max(0.0, joint.constraintForceMagnitude)
    : 0.0;
  const transferred = Number.isFinite(joint?.transferredConstraintForceMagnitude)
    ? Math.max(0.0, joint.transferredConstraintForceMagnitude)
    : 0.0;
  return Math.max(direct, transferred);
}

function createForceSignTexture(text) {
  if (typeof document === 'undefined' || typeof document.createElement !== 'function') {
    return null;
  }

  const canvas = document.createElement('canvas');
  canvas.width = FORCE_SIGN_TEXTURE_WIDTH;
  canvas.height = FORCE_SIGN_TEXTURE_HEIGHT;
  const context = canvas.getContext('2d');
  if (!context) {
    return null;
  }

  paintForceSignCanvas(context, text);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.needsUpdate = true;
  return texture;
}

function paintForceSignCanvas(context, text) {
  const width = FORCE_SIGN_TEXTURE_WIDTH;
  const height = FORCE_SIGN_TEXTURE_HEIGHT;
  context.clearRect(0, 0, width, height);
  context.fillStyle = 'rgba(12, 17, 31, 1.0)';
  context.strokeStyle = 'rgba(255, 211, 77, 0.96)';
  context.lineWidth = 5;
  context.beginPath();
  if (typeof context.roundRect === 'function') {
    context.roundRect(4, 4, width - 8, height - 8, 12);
  } else {
    context.rect(4, 4, width - 8, height - 8);
  }
  context.fill();
  context.stroke();
  context.fillStyle = '#f7fbff';
  context.font = '600 36px system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.fillText(text, width * 0.5, height * 0.52);
}

function createTraceLabelTexture(text, { preview = false } = {}) {
  if (typeof document === 'undefined' || typeof document.createElement !== 'function') {
    return null;
  }

  const canvas = document.createElement('canvas');
  canvas.width = TRACE_LABEL_TEXTURE_WIDTH;
  canvas.height = TRACE_LABEL_TEXTURE_HEIGHT;
  const context = canvas.getContext('2d');
  if (!context) {
    return null;
  }

  context.clearRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = preview ? 'rgba(18, 38, 52, 0.86)' : 'rgba(12, 17, 31, 0.92)';
  context.strokeStyle = preview ? 'rgba(45, 212, 191, 0.82)' : 'rgba(56, 189, 248, 0.95)';
  context.lineWidth = 3;
  context.beginPath();
  if (typeof context.roundRect === 'function') {
    context.roundRect(3, 3, canvas.width - 6, canvas.height - 6, 10);
  } else {
    context.rect(3, 3, canvas.width - 6, canvas.height - 6);
  }
  context.fill();
  context.stroke();
  context.fillStyle = preview ? '#bff8f2' : '#62d5ff';
  context.font = '600 24px system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.fillText(text, canvas.width * 0.5, canvas.height * 0.53);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.needsUpdate = true;
  return texture;
}

function finiteVector3Like(value) {
  return Boolean(
    value &&
    Number.isFinite(value.x) &&
    Number.isFinite(value.y) &&
    Number.isFinite(value.z)
  );
}

function dotVector3Like(value, axis) {
  return value.x * axis.x + value.y * axis.y + value.z * axis.z;
}

function getForceSignViewportHeight(camera) {
  const domElement = camera?.userData?.rendererDomElement;
  return Math.max(
    1,
    domElement?.clientHeight ||
      domElement?.height ||
      camera?.userData?.viewportHeight ||
      1
  );
}

function getWorldUnitsPerScreenPixel(camera, worldPosition, viewportHeight = getForceSignViewportHeight(camera)) {
  if (!camera || !worldPosition || !Number.isFinite(viewportHeight) || viewportHeight <= 0) {
    return null;
  }

  if (camera.isPerspectiveCamera && Number.isFinite(camera.fov)) {
    const forward = new THREE.Vector3();
    camera.getWorldDirection(forward);
    const depth = worldPosition.clone().sub(camera.position).dot(forward);
    if (!(depth > EPSILON)) {
      return null;
    }
    return (2.0 * depth * Math.tan(THREE.MathUtils.degToRad(camera.fov) * 0.5)) / viewportHeight;
  }

  if (camera.isOrthographicCamera && Number.isFinite(camera.top) && Number.isFinite(camera.bottom)) {
    const zoom = Number.isFinite(camera.zoom) && camera.zoom > EPSILON ? camera.zoom : 1.0;
    return ((camera.top - camera.bottom) / zoom) / viewportHeight;
  }

  return null;
}

function getForceSignScreenScale(camera, viewportHeight = getForceSignViewportHeight(camera)) {
  if (!camera?.isPerspectiveCamera || !Number.isFinite(camera.fov) || !Number.isFinite(viewportHeight) || viewportHeight <= 0) {
    return { width: FORCE_SIGN_WIDTH, height: FORCE_SIGN_HEIGHT };
  }

  const fovScale = 2.0 / (viewportHeight / Math.tan(THREE.MathUtils.degToRad(camera.fov) * 0.5));
  return {
    width: FORCE_SIGN_WIDTH_PX * fovScale,
    height: FORCE_SIGN_HEIGHT_PX * fovScale
  };
}

function getForceSignJointPosition(spec) {
  const pA = spec?.joint?.attachmentPointA_world;
  const pB = spec?.joint?.attachmentPointB_world;
  if (!finiteVector3Like(pA) || !finiteVector3Like(pB)) {
    return null;
  }

  return new THREE.Vector3(
    (pA.x + pB.x) * 0.5,
    (pA.y + pB.y) * 0.5,
    (pA.z + pB.z) * 0.5
  );
}

function getForceSignAnchor(spec, camera, viewportHeight) {
  const jointPosition = getForceSignJointPosition(spec);
  const upDirection = spec?.upDirection;
  if (!jointPosition || !finiteVector3Like(upDirection)) {
    return null;
  }

  const worldUnitsPerPixel = getWorldUnitsPerScreenPixel(camera, jointPosition, viewportHeight);
  const offset = worldUnitsPerPixel
    ? (FORCE_SIGN_HEIGHT_PX * 0.5 + FORCE_SIGN_JOINT_CLEARANCE_PX) * worldUnitsPerPixel
    : FORCE_SIGN_FALLBACK_FLOAT_OFFSET;

  return jointPosition.add(new THREE.Vector3(
    upDirection.x * offset,
    upDirection.y * offset,
    upDirection.z * offset
  ));
}

function getForceSignSpreadBasis(camera, fallbackUpDirection) {
  const right = new THREE.Vector3(1, 0, 0);
  const up = new THREE.Vector3(0, 1, 0);
  if (camera?.quaternion) {
    right.applyQuaternion(camera.quaternion).normalize();
    up.applyQuaternion(camera.quaternion).normalize();
    return { right, up };
  }

  if (finiteVector3Like(fallbackUpDirection)) {
    up.set(fallbackUpDirection.x, fallbackUpDirection.y, fallbackUpDirection.z).normalize();
  }
  return { right, up };
}

function spreadForceSignSpecs(forceSignSpecs, camera, viewportHeight) {
  if (!Array.isArray(forceSignSpecs) || forceSignSpecs.length < 2) {
    for (const spec of forceSignSpecs ?? []) {
      spec.jointPosition = getForceSignJointPosition(spec);
      spec.position = getForceSignAnchor(spec, camera, viewportHeight);
    }
    return;
  }

  const { right, up } = getForceSignSpreadBasis(camera, forceSignSpecs[0]?.upDirection);
  const items = [];

  for (let i = 0; i < forceSignSpecs.length; i += 1) {
    const spec = forceSignSpecs[i];
    const jointPosition = getForceSignJointPosition(spec);
    const anchor = getForceSignAnchor(spec, camera, viewportHeight);
    spec.jointPosition = jointPosition;
    spec.position = anchor;
    if (!anchor) {
      continue;
    }

    const worldUnitsPerPixel = getWorldUnitsPerScreenPixel(camera, anchor, viewportHeight);
    const halfWidth = worldUnitsPerPixel ? FORCE_SIGN_WIDTH_PX * 0.5 * worldUnitsPerPixel : FORCE_SIGN_WIDTH * 0.5;
    const halfHeight = worldUnitsPerPixel ? FORCE_SIGN_HEIGHT_PX * 0.5 * worldUnitsPerPixel : FORCE_SIGN_HEIGHT * 0.5;
    const margin = worldUnitsPerPixel ? FORCE_SIGN_SPREAD_MARGIN_PX * worldUnitsPerPixel : FORCE_SIGN_SPREAD_MARGIN;
    const x = dotVector3Like(anchor, right);
    const y = dotVector3Like(anchor, up);
    items.push({
      spec,
      anchor,
      baseX: x,
      baseY: y,
      x,
      y,
      halfWidth,
      halfHeight,
      margin
    });
  }

  for (let iteration = 0; iteration < FORCE_SIGN_SPREAD_ITERATIONS; iteration += 1) {
    let moved = false;
    for (let i = 0; i < items.length; i += 1) {
      for (let j = i + 1; j < items.length; j += 1) {
        const a = items[i];
        const b = items[j];
        const dx = b.x - a.x;
        const dy = b.y - a.y;
        const minX = a.halfWidth + b.halfWidth + Math.max(a.margin, b.margin);
        const minY = a.halfHeight + b.halfHeight + Math.max(a.margin, b.margin);
        const overlapX = minX - Math.abs(dx);
        const overlapY = minY - Math.abs(dy);
        if (!(overlapX > 0.0 && overlapY > 0.0)) {
          continue;
        }

        moved = true;
        if (overlapX < overlapY) {
          const direction = dx === 0.0 ? (i % 2 === 0 ? 1.0 : -1.0) : Math.sign(dx);
          const shift = overlapX * 0.5 * direction;
          a.x -= shift;
          b.x += shift;
        } else {
          const direction = dy === 0.0 ? (i % 2 === 0 ? 1.0 : -1.0) : Math.sign(dy);
          const shift = overlapY * 0.5 * direction;
          a.y -= shift;
          b.y += shift;
        }
      }
    }
    if (!moved) {
      break;
    }
  }

  for (const item of items) {
    item.spec.position = item.anchor.clone()
      .addScaledVector(right, item.x - item.baseX)
      .addScaledVector(up, item.y - item.baseY);
  }
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

    this.camera = new THREE.PerspectiveCamera(48, width / height, 0.0001, 30.0);
    const targetX = Number.isFinite(options.targetX) ? options.targetX : 0.0;
    const targetY = Number.isFinite(options.targetY) ? options.targetY : 0.0;
    const camZ = Number.isFinite(options.cameraZ) ? options.cameraZ : 2.2;
    this.camera.position.set(targetX, targetY + 0.04, camZ);
    this.camera.up.set(0, 0, 1);
    this.camera.lookAt(targetX, targetY, 0);
    this._baseViewTarget = new THREE.Vector3(targetX, targetY, 0);
    this._baseCameraOffset = this.camera.position.clone().sub(this._baseViewTarget);
    this._baseCameraDistance = Math.max(
      0.001,
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
    this.measureEnabled = false;
    this.positionTraceColor = DEFAULT_TRACE_COLOR;
    this.positionTraceRadiusPx = 1.25;
    this.positionTracePoints = [];
    this.positionTraceMarkers = [];
    this.positionTraceMarkerLabels = [];
    this.positionTracePreview = null;
    this.drawnPositionTraceCount = 0;
    this.drawnPositionTraceMarkerCount = 0;
    this.drawnExtrusionCount = 0;
    this.extrusionPointCloud = new AppendOnlyPointCloud(this.scene, {
      chunkSize: 65536,
      color: DEFAULT_CABLE_COLOR,
      beadWidth: EXTRUSION_BEAD_WIDTH,
      minPixelSize: EXTRUSION_MIN_PIXEL_SIZE,
      zOffset: DEFAULT_TRACE_Z,
    });
    this.extrusionPointCloud.setViewport(height, this.renderer.getPixelRatio());
    this._animationLoopActive = false;
    this._requestRenderHandle = null;
    this._lastWorld = null;

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
      this.controls.minDistance = 0.0001;
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

    this.stage = new Stage3D(this.scene, this.renderer);
    this._stageMovingMembers = new Set();

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

    // Axes through the origin mark the z=0 print plane; colours come from
    // the colour theme.
    const axesGeometry = new THREE.BufferGeometry();
    axesGeometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array([
      -AXIS_HALF_LENGTH, 0, 0, AXIS_HALF_LENGTH, 0, 0,
      0, -AXIS_HALF_LENGTH, 0, 0, AXIS_HALF_LENGTH, 0,
      0, 0, -AXIS_HALF_LENGTH, 0, 0, AXIS_HALF_LENGTH,
    ]), 3));
    axesGeometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(18), 3));
    const axes = new THREE.LineSegments(axesGeometry, new THREE.LineBasicMaterial({
      vertexColors: true,
      transparent: true,
      depthWrite: false,
    }));
    this.axes = axes;
    axes.frustumCulled = false;
    axes.userData.ownsGeometry = true;
    axes.userData.ownsMaterial = true;
    this.scene.add(axes);

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

    this.positionTracePreviewMaterial = new THREE.PointsMaterial({
      color: DEFAULT_TRACE_MARKER_COLOR,
      size: TRACE_PREVIEW_POINT_SIZE,
      sizeAttenuation: false,
      transparent: true,
      opacity: 0.64,
      depthWrite: false,
      depthTest: false,
      toneMapped: false
    });
    this.positionTracePreviewObject = new THREE.Points(new THREE.BufferGeometry(), this.positionTracePreviewMaterial);
    this.positionTracePreviewObject.visible = false;
    this.positionTracePreviewObject.frustumCulled = false;
    this.positionTracePreviewObject.renderOrder = 935;
    this.positionTracePreviewObject.userData.ownsGeometry = true;
    this.positionTracePreviewObject.userData.ownsMaterial = true;
    this.scene.add(this.positionTracePreviewObject);

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
    this.sharedCylinderGeometry = new THREE.CylinderGeometry(1, 1, 1, 24, 1, false);
    this.sharedFlipperBarGeometry = new THREE.BoxGeometry(1, 1, 1);
    this.orientedSphereGeometryCache = new Map();
    this.orientedCylinderGeometryCache = new Map();

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

    this.cableLines = new CableLineBatch(this.root, { linewidth: CABLE_LINE_WIDTH_PX });
    this.cableLines.setResolution(width, height);
    this._lineScratch = new Float32Array((ARC_SEGMENTS + 1) * 3);
    this.machineDetails = new MachineDetails3D(this.root, { collectEdges: collectRigidGroupEdges });
    this.colorThemeId = null;
    this._applyColorTheme(DEFAULT_COLOR_THEME);
    this.knotMarkers = [];
    this.forceSigns = [];
    this.forceSignConnectors = [];

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
    this._traceLabelRight = new THREE.Vector3(1, 0, 0);
    this._traceLabelUp = new THREE.Vector3(0, 1, 0);

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
    this.cableLines.setResolution(width, height);
    this.extrusionPointCloud?.setViewport(height, this.renderer.getPixelRatio());
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
      if (this.positionTracePointsObject.visible) {
        this._updatePointObject(this.positionTracePointsObject, [], null, DEFAULT_TRACE_Z);
      }
      this.requestRender();
      return;
    }
    this.requestRender();
  }

  setMeasureEnabled(enabled) {
    this.measureEnabled = Boolean(enabled);
    if (!this.measureEnabled) {
      this.clearPositionTracePreview();
    }
    this._syncPositionTraceMarkers();
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
    this._clearPositionTraceMarkerLabels();
    this.drawnPositionTraceMarkerCount = 0;
    this._updatePointObject(this.positionTraceMarkersObject, [], null, DEFAULT_MARKER_Z);
    this.requestRender();
  }

  clearMeasureMarkers() {
    this.clearPositionTraceMarkers();
  }

  addPositionTraceMarker(simX, simY, label = '', simZ = DEFAULT_MARKER_Z) {
    if (!Number.isFinite(simX) || !Number.isFinite(simY)) {
      return;
    }
    this.positionTraceMarkers.push({
      x: simX,
      y: simY,
      z: Number.isFinite(simZ) ? simZ : DEFAULT_MARKER_Z,
      label
    });
    this._syncPositionTraceMarkers();
    this.requestRender();
  }

  addMeasureMarker(simX, simY, label = '', simZ = DEFAULT_MARKER_Z) {
    this.addPositionTraceMarker(simX, simY, label, simZ);
  }

  setPositionTracePreview(point = null, label = '') {
    if (
      !this.measureEnabled
      || !point
      || !Number.isFinite(point.x)
      || !Number.isFinite(point.y)
    ) {
      this.positionTracePreview = null;
      this._syncPositionTracePreview();
      this.requestRender();
      return;
    }

    const z = Number.isFinite(point.z) ? point.z : DEFAULT_MARKER_Z;
    this.positionTracePreview = {
      x: point.x,
      y: point.y,
      z,
      label
    };
    this._syncPositionTracePreview();
    this.requestRender();
  }

  clearPositionTracePreview() {
    this.setPositionTracePreview(null);
  }

  clearExtrusions() {
    this.extrusionPointCloud?.clear?.();
    this.drawnExtrusionCount = 0;
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

  resolvePositionTracePoint(world, clientX, clientY) {
    const pixel = this._clientToCanvasPixels(clientX, clientY);
    if (!pixel) {
      return null;
    }

    const ray = this._setRayFromCanvasPixel(pixel.x, pixel.y);
    const snapped = ray && world ? this._findPositionTraceSnap(world, ray) : null;
    if (snapped) {
      return snapped;
    }

    const point = this._projectCanvasToSim(pixel.x, pixel.y);
    if (!point || !Number.isFinite(point.x) || !Number.isFinite(point.y)) {
      return null;
    }
    return {
      point: {
        x: point.x,
        y: point.y,
        z: Number.isFinite(point.z) ? point.z : DEFAULT_MARKER_Z,
      },
      snapped: false,
      entityId: null,
    };
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
    this._setRayFromCanvasPixel(pixelX, pixelY);

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

  _setRayFromCanvasPixel(pixelX, pixelY) {
    const width = Math.max(1, this.canvas.width || this.canvas.clientWidth || 1);
    const height = Math.max(1, this.canvas.height || this.canvas.clientHeight || 1);
    if (!Number.isFinite(pixelX) || !Number.isFinite(pixelY)) {
      return null;
    }
    this._rayNdc.x = (pixelX / width) * 2 - 1;
    this._rayNdc.y = -(pixelY / height) * 2 + 1;
    this._raycaster.setFromCamera(this._rayNdc, this.camera);
    return this._raycaster.ray;
  }

  _findPositionTraceSnap(world, ray) {
    if (!world || !ray || typeof world.query !== 'function') {
      return null;
    }

    const viewportHeight = Math.max(1, this.canvas.clientHeight || this.canvas.height || 1);
    let best = null;
    for (const entityId of world.query([PositionComponent])) {
      const pos = world.getComponent(entityId, PositionComponent)?.pos;
      if (!pos || !Number.isFinite(pos.x) || !Number.isFinite(pos.y) || !Number.isFinite(pos.z)) {
        continue;
      }
      const radiusComp = world.getComponent(entityId, RadiusComponent);
      const renderComp = world.getComponent(entityId, RenderableComponent);
      if (!radiusComp && !renderComp) {
        continue;
      }

      const target = this._rayPlanePoint.set(pos.x, pos.y, pos.z);
      const depth = target.clone().sub(ray.origin).dot(ray.direction);
      if (!(depth > 0)) {
        continue;
      }

      const radius = Number.isFinite(radiusComp?.radius) ? Math.max(0.0, radiusComp.radius) : 0.0;
      const worldUnitsPerPixel = getWorldUnitsPerScreenPixel(this.camera, target, viewportHeight) ?? 0.0;
      const snapRadius = Math.max(radius, TRACE_ENTITY_SNAP_RADIUS_PX * worldUnitsPerPixel);
      if (!(snapRadius > 0.0)) {
        continue;
      }

      const distSq = ray.distanceSqToPoint(target);
      if (distSq > snapRadius * snapRadius) {
        continue;
      }
      if (!best || depth < best.depth || (Math.abs(depth - best.depth) <= EPSILON && distSq < best.distSq)) {
        best = { entityId, pos, depth, distSq };
      }
    }

    if (!best) {
      return null;
    }
    return {
      point: { x: best.pos.x, y: best.pos.y, z: best.pos.z },
      snapped: true,
      entityId: best.entityId,
    };
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

  _ensureExtrusionPointCloud() {
    if (!this.extrusionPointCloud) {
      this.extrusionPointCloud = new AppendOnlyPointCloud(this.scene, {
        chunkSize: 65536,
        color: DEFAULT_CABLE_COLOR,
        beadWidth: EXTRUSION_BEAD_WIDTH,
        minPixelSize: EXTRUSION_MIN_PIXEL_SIZE,
        zOffset: DEFAULT_TRACE_Z,
      });
      this.extrusionPointCloud.setViewport(this._lastHeight ?? this.canvas.clientHeight, this.renderer.getPixelRatio());
      this.extrusionPointCloud.setShading(this.colorTheme.printShading);
    }
    return this.extrusionPointCloud;
  }

  _syncExtrusions(world) {
    const extruderEntities = world.query([ExtruderComponent]);
    if (extruderEntities.length === 0) {
      if (this.drawnExtrusionCount !== 0 || (this.extrusionPointCloud?.totalPoints ?? 0) !== 0) {
        this.clearExtrusions();
      }
      return;
    }

    const extruderComp = world.getComponent(extruderEntities[0], ExtruderComponent);
    const extrusions = Array.isArray(extruderComp?.extrusions) ? extruderComp.extrusions : [];
    if (extrusions.length === this.drawnExtrusionCount) {
      return;
    }

    const pointCloud = this._ensureExtrusionPointCloud();
    if (this.drawnExtrusionCount > extrusions.length) {
      pointCloud.clear();
      this.drawnExtrusionCount = 0;
    }

    for (let i = this.drawnExtrusionCount; i < extrusions.length; i += 1) {
      const extrusion = extrusions[i];
      const point = getExtrusionPoint(extrusion, extruderComp);
      if (!point) {
        continue;
      }
      pointCloud.appendPoint(
        point.x,
        point.y,
        point.z,
        extrusion?.qualityColor || this.colorTheme.print || extrusion?.color || DEFAULT_CABLE_COLOR
      );
    }
    pointCloud.flushUpdates();
    this.drawnExtrusionCount = extrusions.length;
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

    if (
      this.positionTracePoints.length !== this.drawnPositionTraceCount
      || this.positionTracePointsObject.visible !== (this.positionTracePoints.length > 0)
    ) {
      this.drawnPositionTraceCount = this.positionTracePoints.length;
      this.positionTraceMaterial.color.set(this.positionTraceColor);
      this._updatePointObject(this.positionTracePointsObject, this.positionTracePoints, null, DEFAULT_TRACE_Z);
    }
  }

  _syncPositionTraceMarkers() {
    if (!this.measureEnabled) {
      if (this.positionTraceMarkersObject.visible) {
        this._updatePointObject(this.positionTraceMarkersObject, [], null, DEFAULT_MARKER_Z);
      }
      this._syncPositionTraceMarkerLabels();
      return;
    }
    if (
      this.positionTraceMarkers.length === this.drawnPositionTraceMarkerCount
      && this.positionTraceMarkersObject.visible === (this.positionTraceMarkers.length > 0)
    ) {
      this._syncPositionTraceMarkerLabels();
      return;
    }
    this.drawnPositionTraceMarkerCount = this.positionTraceMarkers.length;
    this._updatePointObject(this.positionTraceMarkersObject, this.positionTraceMarkers, null, DEFAULT_MARKER_Z);
    this._syncPositionTraceMarkerLabels();
  }

  _syncPositionTraceMarkerLabels() {
    if (!this.measureEnabled) {
      for (const sign of this.positionTraceMarkerLabels) {
        if (sign) {
          sign.visible = false;
        }
      }
      return;
    }
    const viewportHeight = Math.max(1, this.canvas?.clientHeight || this.canvas?.height || 1);
    for (let i = 0; i < this.positionTraceMarkers.length; i += 1) {
      const marker = this.positionTraceMarkers[i];
      let sign = this.positionTraceMarkerLabels[i];
      if (!sign) {
        sign = this._createPositionTraceLabel(marker?.label || '', false);
        this.positionTraceMarkerLabels[i] = sign;
        this.scene.add(sign);
      }
      this._updatePositionTraceLabel(sign, marker?.label || '', false);
      this._positionTraceLabelAt(sign, marker, viewportHeight);
      sign.visible = Boolean(marker?.label);
    }

    for (let i = this.positionTraceMarkers.length; i < this.positionTraceMarkerLabels.length; i += 1) {
      const sign = this.positionTraceMarkerLabels[i];
      if (!sign) {
        continue;
      }
      this.scene.remove(sign);
      disposeObject(sign);
    }
    this.positionTraceMarkerLabels.length = this.positionTraceMarkers.length;
  }

  _syncPositionTracePreview() {
    if (!this.positionTracePreview) {
      this._updatePointObject(this.positionTracePreviewObject, [], null, DEFAULT_MARKER_Z);
      if (this.positionTracePreviewLabel) {
        this.positionTracePreviewLabel.visible = false;
      }
      return;
    }

    this._updatePointObject(this.positionTracePreviewObject, [this.positionTracePreview], null, DEFAULT_MARKER_Z);
    if (!this.positionTracePreviewLabel) {
      this.positionTracePreviewLabel = this._createPositionTraceLabel(this.positionTracePreview.label || '', true);
      this.scene.add(this.positionTracePreviewLabel);
    }
    this._updatePositionTraceLabel(this.positionTracePreviewLabel, this.positionTracePreview.label || '', true);
    this._positionTraceLabelAt(
      this.positionTracePreviewLabel,
      this.positionTracePreview,
      Math.max(1, this.canvas?.clientHeight || this.canvas?.height || 1)
    );
    this.positionTracePreviewLabel.visible = Boolean(this.positionTracePreview.label);
  }

  _createPositionTraceLabel(text, preview) {
    const texture = createTraceLabelTexture(text, { preview });
    const material = new THREE.SpriteMaterial({
      map: texture,
      transparent: true,
      depthWrite: false,
      depthTest: false,
      toneMapped: false,
      opacity: preview ? 0.82 : 1.0,
    });
    const sign = new THREE.Sprite(material);
    sign.frustumCulled = false;
    sign.renderOrder = preview ? 936 : 936;
    sign.userData.ownsMaterial = true;
    sign.userData.ownsTexture = true;
    sign.userData.traceLabelText = text;
    sign.userData.traceLabelPreview = Boolean(preview);
    return sign;
  }

  _updatePositionTraceLabel(sign, text, preview) {
    if (!sign || sign.userData.traceLabelText === text && sign.userData.traceLabelPreview === Boolean(preview)) {
      return;
    }
    const previous = sign.material?.map || null;
    const texture = createTraceLabelTexture(text, { preview });
    if (sign.material) {
      sign.material.map = texture;
      sign.material.needsUpdate = true;
    }
    if (previous) {
      previous.dispose();
    }
    sign.userData.traceLabelText = text;
    sign.userData.traceLabelPreview = Boolean(preview);
  }

  _positionTraceLabelAt(sign, marker, viewportHeight) {
    if (!sign || !marker) {
      return;
    }
    const worldPosition = this._rayPlanePoint.set(
      marker.x,
      marker.y,
      Number.isFinite(marker.z) ? marker.z : DEFAULT_MARKER_Z
    );
    const worldUnitsPerPixel = getWorldUnitsPerScreenPixel(this.camera, worldPosition, viewportHeight)
      ?? (1.0 / Math.max(1, viewportHeight));
    const right = this._traceLabelRight.set(1, 0, 0).applyQuaternion(this.camera.quaternion).normalize();
    const up = this._traceLabelUp.set(0, 1, 0).applyQuaternion(this.camera.quaternion).normalize();
    sign.position
      .copy(worldPosition)
      .addScaledVector(right, TRACE_LABEL_WIDTH_PX * 0.5 * worldUnitsPerPixel + TRACE_LABEL_OFFSET_PX * worldUnitsPerPixel)
      .addScaledVector(up, TRACE_LABEL_HEIGHT_PX * 0.15 * worldUnitsPerPixel);
    sign.scale.set(
      TRACE_LABEL_WIDTH_PX * worldUnitsPerPixel,
      TRACE_LABEL_HEIGHT_PX * worldUnitsPerPixel,
      1.0
    );
  }

  _clearPositionTraceMarkerLabels() {
    for (const sign of this.positionTraceMarkerLabels) {
      if (!sign) {
        continue;
      }
      this.scene.remove(sign);
      disposeObject(sign);
    }
    this.positionTraceMarkerLabels = [];
  }

  _syncExtruder(world) {
    const extruderEntities = world.query([ExtruderComponent]);
    if (extruderEntities.length === 0) {
      return;
    }

    const extruderComp = world.getComponent(extruderEntities[0], ExtruderComponent);
    const lineStart = extruderComp?.coldEndPos || extruderComp?.centerPos || null;
    const lineEnd = extruderComp?.tipPos || extruderComp?.centerPos || null;
    if (!lineStart || !lineEnd) {
      return;
    }

    const positions = this._lineScratch;
    positions[0] = lineStart.x;
    positions[1] = lineStart.y;
    positions[2] = lineStart.z || 0.0;
    positions[3] = lineEnd.x;
    positions[4] = lineEnd.y;
    positions[5] = lineEnd.z || 0.0;
    this.cableLines.addPolyline(positions, 2, DEFAULT_EXTRUDER_COLOR);
  }

  _applyColorTheme(themeId) {
    this.colorThemeId = themeId;
    const theme = resolveColorTheme(themeId);
    this.colorTheme = theme;
    this.stage.setColors(theme.backdrop, theme.floor);
    const axisColors = this.axes.geometry.attributes.color;
    const color = new THREE.Color();
    theme.axes.forEach((hex, axis) => {
      color.set(hex);
      axisColors.setXYZ(axis * 2, color.r, color.g, color.b);
      axisColors.setXYZ(axis * 2 + 1, color.r, color.g, color.b);
    });
    axisColors.needsUpdate = true;
    this.axes.material.opacity = theme.axesOpacity;
    this.machineDetails.setTheme(theme);
    // Satin metal spools, distinct from the polished eyelets and painted bodies.
    const contrast = themeId === 'contrast';
    this.orientedSphereMaterial.roughness = contrast ? 0.85 : 0.3;
    this.orientedSphereMaterial.metalness = contrast ? 0 : 0.65;
    // Printed material is coloured as it is appended; redraw it in the new theme.
    this.extrusionPointCloud?.setShading(theme.printShading);
    this.clearExtrusions();
    this.requestRender();
  }

  measureRenderBlock(world, name, fn) {
    const perf = world?.getResource?.('performanceMonitor');
    if (perf?.enabled && typeof perf.measureBlock === 'function') {
      return perf.measureBlock(`RenderSystem3D.${name}`, fn);
    }
    return fn();
  }

  update(world, dt = 0) {
    const perf = world?.getResource?.('performanceMonitor');

    if (perf?.enabled) {
      const t0 = performance.now();
      try {
        return this._updateImpl(world, dt);
      } finally {
        perf.measureSample?.('RenderSystem3D.update.inner', performance.now() - t0);
      }
    }

    return this._updateImpl(world, dt);
  }

  _updateImpl(world, dt) {
    if (world) {
      this._lastWorld = world;
      const themeId = world.getResource?.('colorTheme') ?? DEFAULT_COLOR_THEME;
      if (themeId !== this.colorThemeId) {
        this._applyColorTheme(themeId);
      }
    }
    if (this.drawingSuspended) {
      return;
    }
    if (!this.renderOnSimulationStep && Number.isFinite(dt) && dt > 0) {
      return;
    }


    this.measureRenderBlock(world, '_syncStage', () => {
      this._syncStage(world);
    });
    this.measureRenderBlock(world, '_syncBorder', () => {
      this._syncBorder(world);
    });
    this.measureRenderBlock(world, '_syncMachineDetails', () => {
      this.machineDetails.update(world);
    });
    this.measureRenderBlock(world, '_syncCircles', () => {
      this._syncCircles(world);
    });
    this.measureRenderBlock(world, '_syncFlippers', () => {
      this._syncFlippers(world);
    });
    this.cableLines.begin();
    this.measureRenderBlock(world, '_syncExtruder', () => {
      this._syncExtruder(world);
    });
    this.measureRenderBlock(world, '_syncCable', () => {
      this._syncCable(world);
    });
    this.cableLines.end();
    this.measureRenderBlock(world, '_syncReferencePaths', () => {
      this._syncReferencePaths();
    });
    this.measureRenderBlock(world, '_syncExtrusions', () => {
      this._syncExtrusions(world);
    });
    this.measureRenderBlock(world, '_syncPositionTrace', () => {
      this._syncPositionTrace(world);
    });
    this.measureRenderBlock(world, '_syncPositionTraceMarkers', () => {
      this._syncPositionTraceMarkers();
    });
    this.measureRenderBlock(world, '_syncPositionTracePreview', () => {
      this._syncPositionTracePreview();
    });
    this.measureRenderBlock(world, '_updateBumperHitFx', () => {
      this._updateBumperHitFx(world);
    });

    this.measureRenderBlock(world, 'the_rest', () => {
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
    });
    this.measureRenderBlock(world, 'this.renderer.render', () => {
      this.renderer.render(this.scene, this.camera);
    });
  }

  resetVisuals() {
    this.machineDetails.reset();
    for (const mesh of this.circleMeshes.values()) {
      this.root.remove(mesh);
    }
    this.circleMeshes.clear();

    for (const group of this.flipperMeshes.values()) {
      this.root.remove(group);
    }
    this.flipperMeshes.clear();

    for (const marker of this.knotMarkers) {
      this.root.remove(marker);
    }
    this.knotMarkers.length = 0;

    for (const sign of this.forceSigns) {
      this.root.remove(sign);
      disposeObject(sign);
    }
    this.forceSigns.length = 0;

    for (const line of this.forceSignConnectors) {
      this.root.remove(line);
      disposeObject(line);
    }
    this.forceSignConnectors.length = 0;

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
    this.clearPositionTrace({ keepMarkers: true });
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

    for (const geometry of this.orientedCylinderGeometryCache.values()) {
      geometry.dispose();
    }
    this.orientedCylinderGeometryCache.clear();

    this.orientedSphereMaterial.dispose();
    this.knotMarkerMaterial.dispose();
    this.sharedSphereGeometry.dispose();
    this.sharedCylinderGeometry.dispose();
    this.sharedFlipperBarGeometry.dispose();

    this.stage.dispose();
    this.cableLines.dispose();
    this.machineDetails.dispose();
    this.extrusionPointCloud?.dispose();

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
    if (this.positionTracePointsObject && this.scene) {
      this.scene.remove(this.positionTracePointsObject);
      disposeObject(this.positionTracePointsObject);
    }
    if (this.positionTraceMarkersObject && this.scene) {
      this.scene.remove(this.positionTraceMarkersObject);
      disposeObject(this.positionTraceMarkersObject);
    }
    if (this.positionTracePreviewObject && this.scene) {
      this.scene.remove(this.positionTracePreviewObject);
      disposeObject(this.positionTracePreviewObject);
    }
    this._clearPositionTraceMarkerLabels();
    if (this.positionTracePreviewLabel && this.scene) {
      this.scene.remove(this.positionTracePreviewLabel);
      disposeObject(this.positionTracePreviewLabel);
      this.positionTracePreviewLabel = null;
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

  // The floor sits just under the lowest fixed part of the machine.
  _syncStage(world) {
    const movingMembers = this._stageMovingMembers;
    movingMembers.clear();
    for (const ComponentClass of [RigidBodyComponent, RigidGroupComponent]) {
      for (const groupId of world.query([ComponentClass])) {
        for (const memberId of world.getComponent(groupId, ComponentClass)?.members ?? []) {
          movingMembers.add(memberId);
        }
      }
    }

    let lowestZ = PRINT_SURFACE_Z;
    for (const entityId of world.query([PositionComponent, RadiusComponent])) {
      const pos = world.getComponent(entityId, PositionComponent)?.pos;
      if (pos && !movingMembers.has(entityId)) {
        const radius = world.getComponent(entityId, RadiusComponent)?.radius;
        lowestZ = Math.min(lowestZ, (pos.z || 0.0) - (Number.isFinite(radius) ? radius : 0.0));
      }
    }

    this.stage.setLayout({
      reach: this._computeMachineHalfExtent(world),
      floorZ: lowestZ - STAGE_FLOOR_CLEARANCE,
    });
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

  _syncCircles(world) {
    this._activeCircleIds.clear();

    const entities = world.query([PositionComponent, RadiusComponent, RenderableComponent]);
    for (const entityId of entities) {
      const renderComp = world.getComponent(entityId, RenderableComponent);
      if (!renderComp || (renderComp.shape !== 'circle' && renderComp.shape !== 'cylinder')) {
        continue;
      }
      if (this.machineDetails.handles(entityId)) {
        continue;
      }

      this._activeCircleIds.add(entityId);

      const pos = world.getComponent(entityId, PositionComponent)?.pos;
      const radius = world.getComponent(entityId, RadiusComponent)?.radius;
      if (!pos || !Number.isFinite(radius) || radius <= 0) {
        continue;
      }
      const spoolState = world.getComponent(entityId, SpoolStateComponent);
      const orientationComp = world.getComponent(entityId, OrientationComponent);
      const isCylinder = renderComp.shape === 'cylinder';

      let mesh = this.circleMeshes.get(entityId);
      if (!mesh) {
        mesh = new THREE.Mesh(
          isCylinder ? this._getOrientedCylinderGeometry(this.colorTheme.rotor ?? renderComp.color) : this.sharedSphereGeometry,
          isCylinder ? this.orientedSphereMaterial : this._getSphereMaterial(renderComp.color),
        );
        this.circleMeshes.set(entityId, mesh);
        this.root.add(mesh);
      }

      const geometry = isCylinder
        ? this._getOrientedCylinderGeometry(this.colorTheme.rotor ?? renderComp.color)
        : orientationComp?.quaternion
        ? this._getOrientedSphereGeometry(renderComp?.color)
        : this.sharedSphereGeometry;
      if (mesh.geometry !== geometry) {
        mesh.geometry = geometry;
      }

      const material = isCylinder
        ? this.orientedSphereMaterial
        : orientationComp?.quaternion
        ? this.orientedSphereMaterial
        : this._getSphereMaterial(renderComp.color);
      if (mesh.material !== material) {
        mesh.material = material;
      }

      mesh.position.set(pos.x, pos.y, pos.z);
      const height = Number.isFinite(renderComp.height) && renderComp.height > 0
        ? renderComp.height
        : radius;
      mesh.scale.set(radius, isCylinder ? height : radius, radius);

      if (orientationComp?.quaternion) {
        const q = orientationComp.quaternion;
        if (isCylinder) {
          const axisLocal = getSpoolLocalAxis(spoolState);
          const axisVector = new THREE.Vector3(axisLocal.x, axisLocal.y, axisLocal.z);
          const localAlignment = new THREE.Quaternion().setFromUnitVectors(CYLINDER_LOCAL_UP, axisVector.normalize());
          mesh.quaternion.set(q.x, q.y, q.z, q.w).multiply(localAlignment);
        } else {
          mesh.quaternion.set(q.x, q.y, q.z, q.w);
        }
      } else {
        if (isCylinder) {
          const axisLocal = getSpoolLocalAxis(spoolState);
          const axisVector = new THREE.Vector3(axisLocal.x, axisLocal.y, axisLocal.z);
          mesh.quaternion.setFromUnitVectors(CYLINDER_LOCAL_UP, axisVector.normalize());
        } else {
          mesh.quaternion.identity();
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
    const showForceSigns = world.getResource('showConstraintForces') !== false;
    const theme = this.colorTheme;
    const highlightSlack = world.getResource('highlightSlackLines') === true;
    const lineColor = (entityId) => theme.line
      ?? world.getComponent(entityId, RenderableComponent)?.color
      ?? DEFAULT_CABLE_COLOR;
    const upDirection = getUpDirection(world);

    if (pathEntities.length === 0) {
      this._hideMeshes(this.knotMarkers);
      this._hideMeshes(this.forceSigns);
      this._hideLines(this.forceSignConnectors);
      return;
    }

    const jointSpecs = [];
    const arcSpecs = [];
    const knotSpecs = [];
    const forceSignSpecs = [];

    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (!path || !Array.isArray(path.jointEntities) || path.jointEntities.length === 0) {
        continue;
      }

      for (const jointId of path.jointEntities) {
        const joint = world.getComponent(jointId, CableJointComponent);
        if (!joint) continue;

        const dist = joint.attachmentPointA_world.distanceTo(joint.attachmentPointB_world);
        const taut = dist > joint.restLength + 1e-6;

        jointSpecs.push({
          joint,
          color: taut || !highlightSlack ? lineColor(jointId) : theme.slack,
          taut,
        });

        if (showForceSigns) {
          forceSignSpecs.push({
            joint,
            text: formatConstraintForce(totalConstraintForceMagnitude(joint)),
            upDirection
          });
        }
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

        const planeNormal = getRenderPlaneNormal(world, rollerId, this.defaultPlaneNormal);
        const basis = buildPlaneBasis(planeNormal);

        const p1 = prevJoint.attachmentPointB_world;
        const p2 = nextJoint.attachmentPointA_world;

        const a1 = angleOnPlane(p1, center, basis);
        const a2 = angleOnPlane(p2, center, basis);

        const distPrev = prevJoint.attachmentPointA_world.distanceTo(prevJoint.attachmentPointB_world);
        const tensionPrev = distPrev > prevJoint.restLength + 1e-6;
        const distNext = nextJoint.attachmentPointA_world.distanceTo(nextJoint.attachmentPointB_world);
        const tensionNext = distNext > nextJoint.restLength + 1e-6;

        arcSpecs.push({
          center,
          radius,
          basis,
          startAngle: a1,
          endAngle: a2,
          cw: Boolean(path.cw[i]),
          color: (tensionPrev && tensionNext) || !highlightSlack ? lineColor(path.jointEntities[i]) : theme.slack
        });
      }

      const nLinks = path.linkTypes.length;

      if (path.linkTypes[0] === 'hybrid' && path.jointEntities.length > 0) {
        const joint0 = world.getComponent(path.jointEntities[0], CableJointComponent);
        if (joint0) {
          const color = lineColor(path.jointEntities[0]);
          const rollerA = joint0.entityA;
          const center = world.getComponent(rollerA, PositionComponent)?.pos;
          const bodyRadius = world.getComponent(rollerA, RadiusComponent)?.radius;

          if (center && Number.isFinite(bodyRadius) && bodyRadius > EPSILON) {
            const planeNormal = getRenderPlaneNormal(world, rollerA, this.defaultPlaneNormal);
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
          const color = lineColor(path.jointEntities[nLinks - 2]);
          const rollerB = jointN.entityB;
          const center = world.getComponent(rollerB, PositionComponent)?.pos;
          const bodyRadius = world.getComponent(rollerB, RadiusComponent)?.radius;

          if (center && Number.isFinite(bodyRadius) && bodyRadius > EPSILON) {
            const planeNormal = getRenderPlaneNormal(world, rollerB, this.defaultPlaneNormal);
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

    this._ensureKnotMarkerCapacity(knotSpecs.length);
    this._ensureForceSignCapacity(forceSignSpecs.length);
    this._ensureForceSignConnectorCapacity(forceSignSpecs.length);
    const forceSignViewportHeight = Math.max(
      1,
      this.renderer?.domElement?.clientHeight ||
        this.renderer?.domElement?.height ||
        this.canvas?.clientHeight ||
        this.canvas?.height ||
        1
    );
    spreadForceSignSpecs(forceSignSpecs, this.camera, forceSignViewportHeight);

    const positions = this._lineScratch;
    for (const spec of jointSpecs) {
      // A taut span is straight; only slack spans need segments for their sag.
      const segments = spec.taut ? 1 : JOINT_LINE_SEGMENTS;
      writeSlackCablePositions(
        positions,
        spec.joint.attachmentPointA_world,
        spec.joint.attachmentPointB_world,
        spec.joint.restLength,
        upDirection,
        segments
      );
      this.cableLines.addPolyline(positions, segments + 1, spec.color);
    }

    for (const spec of arcSpecs) {
      const pointCount = writeArcPositions(
        positions,
        spec.center,
        spec.radius,
        spec.basis,
        spec.startAngle,
        spec.endAngle,
        spec.cw
      );
      this.cableLines.addPolyline(positions, pointCount, spec.color);
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

    for (let i = 0; i < this.forceSigns.length; i++) {
      const sign = this.forceSigns[i];
      const connector = this.forceSignConnectors[i];
      const spec = forceSignSpecs[i];
      if (!spec) {
        sign.visible = false;
        if (connector) {
          connector.visible = false;
        }
        continue;
      }

      if (!spec.position || !spec.jointPosition) {
        sign.visible = false;
        if (connector) {
          connector.visible = false;
        }
        continue;
      }

      this._updateForceSignText(sign, spec.text);
      sign.visible = true;
      sign.position.copy(spec.position);
      const scale = getForceSignScreenScale(this.camera, forceSignViewportHeight);
      sign.scale.set(scale.width, scale.height, 1.0);

      if (connector) {
        connector.visible = true;
        const p = connector.geometry.attributes.position.array;
        p[0] = spec.jointPosition.x;
        p[1] = spec.jointPosition.y;
        p[2] = spec.jointPosition.z;
        p[3] = spec.position.x;
        p[4] = spec.position.y;
        p[5] = spec.position.z;
        connector.geometry.attributes.position.needsUpdate = true;
      }
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

    const planeNormal = getRenderPlaneNormal(world, entityId, this.defaultPlaneNormal);
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

  _ensureKnotMarkerCapacity(count) {
    while (this.knotMarkers.length < count) {
      const marker = new THREE.Mesh(this.sharedSphereGeometry, this.knotMarkerMaterial);
      marker.frustumCulled = false;
      this.knotMarkers.push(marker);
      this.root.add(marker);
    }
  }

  _ensureForceSignCapacity(count) {
    while (this.forceSigns.length < count) {
      const sign = this._createForceSign();
      this.forceSigns.push(sign);
      this.root.add(sign);
    }
  }

  _ensureForceSignConnectorCapacity(count) {
    while (this.forceSignConnectors.length < count) {
      const line = this._createForceSignConnector();
      this.forceSignConnectors.push(line);
      this.root.add(line);
    }
  }

  _createForceSign() {
    const texture = createForceSignTexture('0.00 N');
    const material = new THREE.SpriteMaterial({
      map: texture,
      color: texture ? 0xffffff : 0xffd34d,
      transparent: true,
      depthTest: false,
      depthWrite: false,
      sizeAttenuation: false,
      toneMapped: false
    });
    const sign = new THREE.Sprite(material);
    sign.scale.set(FORCE_SIGN_WIDTH, FORCE_SIGN_HEIGHT, 1.0);
    sign.renderOrder = 950;
    sign.frustumCulled = false;
    sign.userData.ownsMaterial = true;
    sign.userData.ownsTexture = Boolean(texture);
    sign.userData.text = '0.00 N';
    return sign;
  }

  _createForceSignConnector() {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(2 * 3), 3));
    const material = new THREE.LineBasicMaterial({
      color: FORCE_SIGN_CONNECTOR_COLOR,
      transparent: true,
      opacity: 0.72,
      depthTest: false,
      depthWrite: false,
      toneMapped: false
    });
    const line = new THREE.Line(geometry, material);
    line.renderOrder = 949;
    line.frustumCulled = false;
    line.userData.ownsGeometry = true;
    line.userData.ownsMaterial = true;
    return line;
  }

  _updateForceSignText(sign, text) {
    if (sign.userData.text === text) {
      return;
    }

    sign.userData.text = text;
    const map = sign.material?.map;
    const canvas = map?.image;
    const context = canvas?.getContext?.('2d');
    if (!context) {
      return;
    }

    paintForceSignCanvas(context, text);
    map.needsUpdate = true;
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

  _getOrientedCylinderGeometry(color) {
    const primary = typeof color === 'string' && color.length > 0 ? color : '#a0a0a0';
    const back = this.colorTheme.rotorBack;
    const key = `${primary}|${back}`;
    let geometry = this.orientedCylinderGeometryCache.get(key);
    if (!geometry) {
      geometry = createDualColorCylinderGeometry(this.sharedCylinderGeometry, primary, back);
      this.orientedCylinderGeometryCache.set(key, geometry);
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

  _computeMachineHalfExtent(world) {
    let maxAbs = MACHINE_MIN_HALF_EXTENT;

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

    return maxAbs + MACHINE_EXTENT_MARGIN;
  }

  _cameraDistanceForScale(scaleMultiplier = this.viewScaleMultiplier) {
    const scale = scaleMultiplier;
    return this._baseCameraDistance / scale;
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

    const distance = this._cameraDistanceForScale();
    const cameraOffset = this._buildOrbitOffset(distance);
    // Near plane follows zoom so the depth buffer resolves millimetre parts
    // at any distance.
    this.camera.near = Math.max(CAMERA_MIN_NEAR, distance * CAMERA_NEAR_PER_DISTANCE);
    this.camera.updateProjectionMatrix();
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

function getExtrusionPoint(extrusion, extruderComp = null) {
  if (!extrusion) return null;

  const machineTip = extrusion?.machineId
    ? extruderComp?.machineTips?.[extrusion.machineId] || extruderComp?.machineCenters?.[extrusion.machineId]
    : extruderComp?.tipPos || extruderComp?.centerPos;

  if (Array.isArray(extrusion.pos) && extrusion.pos.length >= 2) {
    return {
      x: finiteOr(extrusion.pos[0], finiteOr(machineTip?.x, 0.0)),
      y: finiteOr(extrusion.pos[1], finiteOr(machineTip?.y, 0.0)),
      z: finiteOr(extrusion.pos[2], finiteOr(machineTip?.z, DEFAULT_TRACE_Z)),
    };
  }

  // Direct Vector3 shape
  if (
    Number.isFinite(extrusion.x) &&
    Number.isFinite(extrusion.y) &&
    Number.isFinite(extrusion.z)
  ) {
    return extrusion;
  }

  const point =
    extrusion.position ||
    extrusion.point ||
    extrusion.tipPos ||
    null;

  if (
    point &&
    Number.isFinite(point.x) &&
    Number.isFinite(point.y) &&
    Number.isFinite(point.z)
  ) {
    return point;
  }

  return machineTip && Number.isFinite(machineTip.x) && Number.isFinite(machineTip.y)
    ? {
        x: finiteOr(machineTip.x, 0.0),
        y: finiteOr(machineTip.y, 0.0),
        z: finiteOr(machineTip.z, DEFAULT_TRACE_Z),
      }
    : null;
}

// Extruded material drawn as round, shaded beads: beadWidth wide in the
// world, never smaller than minPixelSize on screen.
const BEAD_VERTEX_SHADER = `
uniform float uBeadWidth;
uniform float uMinPixelSize;
uniform float uViewportHeight;
varying vec3 vColor;
void main() {
  vColor = color;
  vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
  gl_Position = projectionMatrix * mvPosition;
  float pixelsPerUnit = projectionMatrix[1][1] * 0.5 * uViewportHeight / max(-mvPosition.z, 1e-6);
  gl_PointSize = max(uMinPixelSize, uBeadWidth * pixelsPerUnit);
}`;
const BEAD_FRAGMENT_SHADER = `
uniform float uShading;
varying vec3 vColor;
void main() {
  vec2 p = gl_PointCoord * 2.0 - 1.0;
  float r = length(p);
  // Soft rim, resolved by alpha-to-coverage, so edges do not stipple in motion.
  // The rim is at most half the radius so minimum-size beads stay solid.
  float coverage = 1.0 - smoothstep(1.0 - min(fwidth(r), 0.5), 1.0, r);
  if (coverage <= 0.0) discard;
  vec3 normal = vec3(p.x, -p.y, sqrt(max(0.0, 1.0 - r * r)));
  float diffuse = max(dot(normal, normalize(vec3(-0.35, 0.55, 0.75))), 0.0);
  gl_FragColor = vec4(vColor * mix(1.0, 0.42 + 0.68 * diffuse, uShading), coverage);
  #include <colorspace_fragment>
}`;

class AppendOnlyPointCloud {
  constructor(scene, {
    chunkSize,
    color,
    beadWidth,
    minPixelSize,
    zOffset,
  }) {
    this.scene = scene;
    this.chunkSize = chunkSize;
    this.color = color;
    this.zOffset = zOffset;
    this.material = new THREE.ShaderMaterial({
      uniforms: {
        uBeadWidth: { value: beadWidth },
        uMinPixelSize: { value: minPixelSize },
        uViewportHeight: { value: 1 },
        uShading: { value: 1 },
      },
      vertexShader: BEAD_VERTEX_SHADER,
      fragmentShader: BEAD_FRAGMENT_SHADER,
      vertexColors: true,
      alphaToCoverage: true,
    });
    this._minPixelSize = minPixelSize;

    this.chunks = [];
    this.totalPoints = 0;
    this._color = new THREE.Color(color);
  }

  setShading(amount) {
    this.material.uniforms.uShading.value = amount;
  }

  // cssHeight in CSS pixels; gl_PointSize is in device pixels.
  setViewport(cssHeight, pixelRatio) {
    this.material.uniforms.uViewportHeight.value = cssHeight * pixelRatio;
    this.material.uniforms.uMinPixelSize.value = this._minPixelSize * pixelRatio;
  }

  dispose() {
    this.clear();
    this.material.dispose();
  }

  clear() {
    for (const chunk of this.chunks) {
      this.scene.remove(chunk.points);
      chunk.geometry.dispose();
    }
    this.chunks.length = 0;
    this.totalPoints = 0;
  }

  appendPoint(x, y, z, color = this.color) {
    let chunk = this.chunks[this.chunks.length - 1];

    if (!chunk || chunk.count >= this.chunkSize) {
      chunk = this.createChunk();
      this.chunks.push(chunk);
    }

    const i = chunk.count;
    const base = i * 3;

    chunk.positions[base] = x;
    chunk.positions[base + 1] = y;
    chunk.positions[base + 2] = z + this.zOffset;
    this._color.set(color || this.color);
    chunk.colors[base] = this._color.r;
    chunk.colors[base + 1] = this._color.g;
    chunk.colors[base + 2] = this._color.b;

    chunk.count += 1;
    this.totalPoints += 1;

    chunk.geometry.setDrawRange(0, chunk.count);
    chunk.dirtyStart = Math.min(chunk.dirtyStart, base);
    chunk.dirtyEnd = Math.max(chunk.dirtyEnd, base + 3);
  }

  flushUpdates() {
    for (const chunk of this.chunks) {
      if (!(chunk.dirtyEnd > chunk.dirtyStart)) {
        continue;
      }
      markAttributeRangeDirty(chunk.geometry.attributes.position, chunk.dirtyStart, chunk.dirtyEnd);
      markAttributeRangeDirty(chunk.geometry.attributes.color, chunk.dirtyStart, chunk.dirtyEnd);
      chunk.dirtyStart = Infinity;
      chunk.dirtyEnd = -Infinity;
    }
  }

  createChunk() {
    const positions = new Float32Array(this.chunkSize * 3);
    const colors = new Float32Array(this.chunkSize * 3);

    const geometry = new THREE.BufferGeometry();
    const positionAttr = new THREE.BufferAttribute(positions, 3);
    const colorAttr = new THREE.BufferAttribute(colors, 3);
    positionAttr.setUsage(THREE.DynamicDrawUsage);
    colorAttr.setUsage(THREE.DynamicDrawUsage);
    geometry.setAttribute('position', positionAttr);
    geometry.setAttribute('color', colorAttr);
    geometry.setDrawRange(0, 0);

    const points = new THREE.Points(geometry, this.material);
    points.frustumCulled = false;
    points.renderOrder = 850;
    points.userData.ownsGeometry = true;

    this.scene.add(points);

    return {
      geometry,
      points,
      positions,
      colors,
      count: 0,
      dirtyStart: Infinity,
      dirtyEnd: -Infinity,
    };
  }
}

function markAttributeRangeDirty(attribute, start, end) {
  if (!attribute) {
    return;
  }

  const count = Math.max(0, end - start);
  if (typeof attribute.addUpdateRange === 'function') {
    attribute.addUpdateRange(start, count);
  } else if (attribute.updateRange) {
    attribute.updateRange.offset = start;
    attribute.updateRange.count = count;
  }
  attribute.needsUpdate = true;
}
