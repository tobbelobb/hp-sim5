import * as THREE from 'three';
import Vector3 from '../../../src/js/cable_joints_3d/vector3.js';
import {
  PositionComponent,
  RadiusComponent,
  ObstaclePushComponent
} from '../../../src/js/cable_joints_3d/ecs.js';
import { RenderableComponent } from '../../../src/js/cable_joints/ecs.js';

const EPSILON = 1e-12;

function _nowSeconds() {
  if (typeof performance !== 'undefined' && typeof performance.now === 'function') {
    return performance.now() / 1000;
  }
  return Date.now() / 1000;
}

function _clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function _parseColorToRgb(color) {
  if (!color || typeof color !== 'string') {
    return [255, 160, 40];
  }
  const hexMatch = /^#([0-9a-f]{6})$/i.exec(color);
  if (hexMatch) {
    const hex = parseInt(hexMatch[1], 16);
    return [
      (hex >> 16) & 0xff,
      (hex >> 8) & 0xff,
      hex & 0xff
    ];
  }
  const shortHexMatch = /^#([0-9a-f]{3})$/i.exec(color);
  if (shortHexMatch) {
    const digits = shortHexMatch[1];
    return [
      parseInt(digits[0] + digits[0], 16),
      parseInt(digits[1] + digits[1], 16),
      parseInt(digits[2] + digits[2], 16)
    ];
  }
  const rgbMatch = /^rgba?\s*\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)/i.exec(color);
  if (rgbMatch) {
    return [
      Number(rgbMatch[1]),
      Number(rgbMatch[2]),
      Number(rgbMatch[3])
    ];
  }
  return [255, 160, 40];
}

function _fxPaletteForColor(color) {
  const rgb = _parseColorToRgb(color);
  const r = rgb[0];
  const g = rgb[1];
  const b = rgb[2];
  const hue = Math.atan2(Math.sqrt(3) * (g - b), 2 * r - g - b) * (180 / Math.PI);
  const normalizedHue = (hue + 360) % 360;
  const isBlue = normalizedHue >= 170 && normalizedHue <= 260;
  if (isBlue) {
    return {
      core: [80, 190, 255],
      hot: [195, 240, 255],
      deep: [60, 115, 245]
    };
  }
  return {
    core: [255, 152, 44],
    hot: [255, 222, 128],
    deep: [255, 92, 22]
  };
}

function _rgba(rgb, alpha) {
  return `rgba(${_clamp(Math.round(rgb[0]), 0, 255)},${_clamp(Math.round(rgb[1]), 0, 255)},${_clamp(Math.round(rgb[2]), 0, 255)},${_clamp(alpha, 0, 1)})`;
}

function _createGradient(ctx, x, y, radius, palette, alpha) {
  const gradient = ctx.createRadialGradient(x, y, 0, x, y, radius);
  gradient.addColorStop(0, _rgba(palette.core, alpha * 0.85));
  gradient.addColorStop(0.65, _rgba(palette.hot, alpha * 0.55));
  gradient.addColorStop(1, _rgba(palette.hot, 0));
  return gradient;
}

export class BumperHitFxOverlay {
  constructor(world, canvas) {
    this.world = world;
    this.canvas = canvas;
    this.ctx = canvas ? canvas.getContext('2d') : null;
    this.bursts = [];
    this.lastTimeSec = Number.NaN;
    this.pixelScale = 0;
    this.simWidth = 1.0;
    this.simHeight = 1.7;
  }

  _syncSimSize(world) {
    const width = world.getResource('simWidth');
    const height = world.getResource('simHeight');
    if (Number.isFinite(width) && width > EPSILON) {
      this.simWidth = width;
    }
    if (Number.isFinite(height) && height > EPSILON) {
      this.simHeight = height;
    }
    if (this.canvas && this.canvas.height > 0 && Number.isFinite(this.simHeight)) {
      this.pixelScale = this.canvas.height / this.simHeight;
    }
  }

  _nowSec() {
    return _nowSeconds();
  }

  _resizeCanvas() {
    if (!this.canvas) {
      return;
    }
    const rect = this.canvas.getBoundingClientRect();
    if (!rect || rect.width <= 0 || rect.height <= 0) {
      return;
    }
    const width = Math.max(1, Math.round(rect.width));
    const height = Math.max(1, Math.round(rect.height));
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
  }

  _simToPx(vec, renderSystem) {
    if (!this.canvas) {
      return { x: 0, y: 0 };
    }
    const width = this.canvas.width || 1;
    const height = this.canvas.height || 1;
    const camera = renderSystem?.camera;
    if (camera) {
      const projected = new THREE.Vector3(vec.x, vec.y, vec.z ?? 0).project(camera);
      return {
        x: ((projected.x + 1) * 0.5) * width,
        y: ((-projected.y + 1) * 0.5) * height
      };
    }
    const simX = _clamp(vec.x, 0, this.simWidth);
    const simY = _clamp(vec.y, 0, this.simHeight);
    const x = (simX / Math.max(this.simWidth, 1e-6)) * width;
    const y = height - (simY / Math.max(this.simHeight, 1e-6)) * height;
    return { x, y };
  }

  _spawnBurst(world, contact, pushVel) {
    const obsPos = world.getComponent(contact.obs_id, PositionComponent)?.pos;
    if (!obsPos) {
      return;
    }
    const obsRadius = world.getComponent(contact.obs_id, RadiusComponent)?.radius ?? 0.03;
    const deltaLambda = Number.isFinite(contact?.delta_lambda) ? Math.max(0.0, contact.delta_lambda) : 0.0;
    const intensity = Math.max(
      0.7,
      Math.min(2.8, 0.85 + (0.2 * pushVel) + (0.32 * Math.sqrt(deltaLambda + 1e-9)))
    );
    const effectRadius = Math.max(0.015, obsRadius * (0.6 + 0.2 * intensity));
    const direction = new Vector3(1.0, 0.0, 0.0);
    if (contact?.direction && contact.direction.lengthSq() > EPSILON) {
      direction.set(contact.direction);
    }
    direction.normalize();
    const renderComp = world.getComponent(contact.obs_id, RenderableComponent);
    const palette = _fxPaletteForColor(renderComp?.color);
    this.bursts.push({
      origin: obsPos.clone(),
      direction,
      intensity,
      palette,
      life: 0.35 + 0.05 * Math.random(),
      age: 0.0,
      effectRadius
    });
  }

  _updateBursts(dt) {
    const next = [];
    for (const burst of this.bursts) {
      burst.age += dt;
      if (burst.age < burst.life) {
        next.push(burst);
      }
    }
    this.bursts = next;
  }

  _renderBursts(renderSystem) {
    if (!this.ctx || !this.canvas) {
      return;
    }
    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    if (this.bursts.length === 0) {
      return;
    }
    for (const burst of this.bursts) {
      const progress = Math.min(1.0, burst.age / Math.max(burst.life, EPSILON));
      const alpha = Math.max(0.0, 1.0 - progress);
      const pxPos = this._simToPx(burst.origin, renderSystem);
      const radiusPx = Math.max(6, burst.effectRadius * this.pixelScale * (1.0 + 0.6 * progress));
      ctx.fillStyle = _createGradient(ctx, pxPos.x, pxPos.y, radiusPx, burst.palette, alpha);
      ctx.fillRect(pxPos.x - radiusPx, pxPos.y - radiusPx, radiusPx * 2, radiusPx * 2);
      ctx.lineWidth = Math.max(1, radiusPx * 0.1);
      ctx.strokeStyle = _rgba(burst.palette.deep, alpha * 0.7);
      const baseAngle = Math.atan2(burst.direction.y, burst.direction.x);
      const spikeCount = 4 + Math.floor(Math.random() * 3);
      for (let i = 0; i < spikeCount; i += 1) {
        const angle = baseAngle + ((i / spikeCount) * (Math.PI * 2)) + (Math.random() - 0.5) * 0.3;
        const length = radiusPx * (0.8 + Math.random() * 0.45);
        const targetX = pxPos.x + Math.cos(angle) * length;
        const targetY = pxPos.y + Math.sin(angle) * length;
        ctx.beginPath();
        ctx.moveTo(pxPos.x, pxPos.y);
        ctx.lineTo(targetX, targetY);
        ctx.stroke();
      }
    }
  }

  update(world, renderSystem) {
    if (!this.canvas || !this.ctx || !world) {
      return;
    }
    this.world = world;
    this._resizeCanvas();
    this._syncSimSize(world);
    const enabled = world.getResource('renderBumperHitFx') === true;
    if (!enabled) {
      this._clearBursts();
      this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
      return;
    }
    const now = this._nowSec();
    if (!Number.isFinite(this.lastTimeSec)) {
      this.lastTimeSec = now;
    }
    let dt = now - this.lastTimeSec;
    this.lastTimeSec = now;
    if (!Number.isFinite(dt) || dt < 0.0) {
      dt = 0.0;
    }
    dt = Math.min(dt, 0.05);

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
      this._spawnBurst(world, contact, pushComp.pushVel);
    }
    this._updateBursts(dt);
    this._renderBursts(renderSystem);
  }

  _clearBursts() {
    this.bursts.length = 0;
    this.lastTimeSec = Number.NaN;
  }
}
