import Vector2 from '../../../src/js/cable_joints/vector2.js';

import {
  PositionComponent,
  RadiusComponent,
  RenderableComponent,
  OrientationComponent,
  DistanceConstraintComponent,
  AngularVelocityComponent,
  MachineTagComponent
} from '../../../src/js/cable_joints/ecs.js';
import { RigidGroupComponent } from '../../../src/js/cable_joints/ecs.js';

import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent
} from '../../../src/js/cable_joints/cable_joints_core.js';

import {
  BorderComponent,
  FlipperTagComponent,
  FlipperStateComponent,
  ObstacleTagComponent,
  ObstaclePushComponent,
  OverlayRadiusComponent,
  CircleSectorComponent,
  CircleSectorsComponent,
} from './flipper_common.js';
import { ExtruderComponent } from '../slideprinter/slideprinter_common.js';

export class RenderSystem {
  runInPause = true; // Always render
  constructor(canvas, cScale, simHeight, viewScaleMultiplier = 1.0, viewOffsetX_sim = 0.0, viewOffsetY_sim = 0.0, sag_multiplier = 0.1) {
    this.canvas = canvas;
    this.c = canvas.getContext("2d");
    this.baseCScale = cScale; // Original scale (pixels per sim unit at 1x zoom)
    this.simHeight = simHeight; // Original sim height (usually 2.0)

    // --- Viewport Settings (Stored on instance) ---
    this.viewScaleMultiplier = viewScaleMultiplier;
    this.viewOffsetX_sim = viewOffsetX_sim;
    this.viewOffsetY_sim = viewOffsetY_sim;
    // Calculate the effective scale used for drawing
    this.effectiveCScale = this.baseCScale * this.viewScaleMultiplier;
    // --- End Viewport Settings ---

    // Store potential obstacles for catenary drawing
    this.cableLinkObstacles = [];
    this.sag_multiplier = sag_multiplier;

    this.extrusionCanvas = document.createElement('canvas');
    this.extrusionCanvas.width = this.canvas.width;
    this.extrusionCanvas.height = this.canvas.height;
    this.extrusionCtx = this.extrusionCanvas.getContext('2d');
    this.drawnExtrusionCount = 0;

    this.positionTraceCanvas = document.createElement('canvas');
    this.positionTraceCanvas.width = this.canvas.width;
    this.positionTraceCanvas.height = this.canvas.height;
    this.positionTraceCtx = this.positionTraceCanvas.getContext('2d');
    this.positionTraceEnabled = false;
    this.positionTraceRadiusPx = 1.25;
    this.positionTraceColor = 'rgba(255,255,255,0.8)';
    this.positionTracePoints = [];
    this.drawnPositionTraceCount = 0;
    this.positionTraceMarkers = [];

    this.referenceCanvas = document.createElement('canvas');
    this.referenceCanvas.width = this.canvas.width;
    this.referenceCanvas.height = this.canvas.height;
    this.referenceCtx = this.referenceCanvas.getContext('2d');
    this.referencePaths = [];
    this.referenceMetadata = null;
    this.referenceColor = '#1e90ff';
    this.referenceRequestedVisible = false;
    this.referenceVisible = false;
    this.referenceDirty = false;
    this.drawingSuspended = false;

    this.bumperHitFxBursts = [];
    this.bumperHitFxActivePairs = new Set();
    this.bumperHitFxLastTimeSec = null;
    this.bumperWobbleByEntity = new Map();
  }

  // Coordinate transformation helpers using instance properties
  cX(simX) {
    // 1. Shift simulation coordinate relative to the view offset
    // 2. Scale by the effective scale
    // 3. Add canvas center offset
    return (simX - this.viewOffsetX_sim) * this.effectiveCScale + this.canvas.width / 2.0;
  }
  cY(simY) {
    // 1. Shift simulation coordinate relative to the view offset
    // 2. Scale by the effective scale
    // 3. Flip Y and center vertically
    const scaledY = (simY - this.viewOffsetY_sim) * this.effectiveCScale;
    // Center the view vertically. We map simY=viewOffsetY_sim to canvas.height/2
    return this.canvas.height / 2.0 - scaledY;
  }

  simXFromCanvas(px) {
    return (px - this.canvas.width / 2.0) / this.effectiveCScale + this.viewOffsetX_sim;
  }

  simYFromCanvas(py) {
    return (this.canvas.height / 2.0 - py) / this.effectiveCScale + this.viewOffsetY_sim;
  }

  // Use effective scale for radius
  drawDisc(simX, simY, simRadius) {
    this.c.beginPath();
    this.c.arc(
      this.cX(simX), // Use instance method
      this.cY(simY), // Use instance method
      simRadius * this.effectiveCScale, // Scale radius by zoom
      0.0, 2.0 * Math.PI
    );
    this.c.closePath();
    this.c.fill();
  }

  // Draw catenary curve for slack cable segment between pA and pB with given length,
  // avoiding obstacles.  New: sagDir (unit Vector2) says which way to sag.
  _drawCatenary(jointId, pA, pB, length, obstacles, machineId = '', sagDir = new Vector2(0, -1), segments = 20) {
      const ctx = this.c;
      const dx = pB.x - pA.x, dy = pB.y - pA.y;
      const D  = Math.hypot(dx, dy);
      if (D <= 1e-6) return;

      // parabolic sag magnitude
      const sag = Math.max(length - D, 0) * this.sag_multiplier;
      sagDir = sagDir.clone().normalize();

      ctx.beginPath();
      let prev_cx = -1, prev_cy = -1;

      for (let i = 0; i <= segments; i++) {
        const t = i / segments;
        // 1) ideal point along straight+parabola
        const sagOff = sag * 4 * t * (1 - t);
        let pt = new Vector2(pA.x + dx*t, pA.y + dy*t)
                     .add(sagDir, sagOff);

        // 2) obstacle‐avoidance
        for (const obs of obstacles) {
          if (obs.machineId !== machineId) {
            continue;
          }
          const v = pt.clone().subtract(obs.pos);
          const d2 = v.lengthSq(), r2 = obs.radius * obs.radius;
          if (d2 < r2) {
            const d = Math.sqrt(d2), push = obs.radius - d;
            if (d > 1e-9) {
              const pd = v.clone().normalize();
              // require same sign as last frame
              pt.add(pd, push);
            } else {
              // exactly at center -> force straight‐down
              pt = obs.pos.clone().add(new Vector2(0, -obs.radius));
            }
          }
        }

        // 3) draw
        const cx = this.cX(pt.x), cy = this.cY(pt.y);
        if (i === 0) ctx.moveTo(cx, cy);
        else         ctx.lineTo(cx, cy);

        prev_cx = cx; prev_cy = cy;
      }

      ctx.stroke();
  }


  setViewTransform({ scaleMultiplier, offsetX, offsetY }) {
    const prevMultiplier = this.viewScaleMultiplier;
    const prevOffsetX = this.viewOffsetX_sim;
    const prevOffsetY = this.viewOffsetY_sim;
    const prevEffectiveScale = this.effectiveCScale;
    if (typeof scaleMultiplier === 'number' && isFinite(scaleMultiplier) && scaleMultiplier > 0) {
      this.viewScaleMultiplier = scaleMultiplier;
      this.effectiveCScale = this.baseCScale * this.viewScaleMultiplier;
    }
    if (typeof offsetX === 'number' && isFinite(offsetX)) {
      this.viewOffsetX_sim = offsetX;
    }
    if (typeof offsetY === 'number' && isFinite(offsetY)) {
      this.viewOffsetY_sim = offsetY;
    }
    const scaleChanged = Math.abs(prevMultiplier - this.viewScaleMultiplier) > 1e-9
      || Math.abs(prevEffectiveScale - this.effectiveCScale) > 1e-9;
    const offsetChanged = Math.abs(prevOffsetX - this.viewOffsetX_sim) > 1e-9
      || Math.abs(prevOffsetY - this.viewOffsetY_sim) > 1e-9;
    if (scaleChanged || offsetChanged) {
      this.referenceDirty = true;
    }
    this._updateReferenceVisibility();
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
    this.referenceDirty = true;
    this._updateReferenceVisibility();
  }

  setDrawingSuspended(suspended) {
    this.drawingSuspended = Boolean(suspended);
  }

  _updateReferenceVisibility() {
    const hasData = Array.isArray(this.referencePaths) && this.referencePaths.length > 0;
    this.referenceVisible = Boolean(this.referenceRequestedVisible) && hasData;
  }

  _redrawReferencePaths() {
    if (!this.referenceCanvas || !this.referenceCtx) {
      this.referenceDirty = false;
      return;
    }
    const width = this.referenceCanvas.width | 0;
    const height = this.referenceCanvas.height | 0;
    if (width <= 0 || height <= 0) {
      this.referenceDirty = false;
      return;
    }

    const ctx = this.referenceCtx;
    ctx.clearRect(0, 0, width, height);

    if (!this.referenceVisible) {
      this.referenceDirty = false;
      return;
    }

    const segments = Array.isArray(this.referencePaths) ? this.referencePaths : [];
    if (segments.length === 0) {
      this.referenceDirty = false;
      return;
    }

    ctx.save();
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.strokeStyle = this._colorWithAlpha(this.referenceColor, 1.0);
    const baseWidthPx = Math.max(0.1875, (0.25 * this.effectiveCScale) / 250);
    ctx.lineWidth = baseWidthPx;

    for (const segment of segments) {
      if (!segment || !segment.start || !segment.end) {
        continue;
      }
      const start = segment.start;
      const end = segment.end;
      const x1 = this.cX(start[0]);
      const y1 = this.cY(start[1]);
      const x2 = this.cX(end[0]);
      const y2 = this.cY(end[1]);
      if (!Number.isFinite(x1) || !Number.isFinite(y1) || !Number.isFinite(x2) || !Number.isFinite(y2)) {
        continue;
      }
      ctx.beginPath();
      ctx.moveTo(x1, y1);
      ctx.lineTo(x2, y2);
      ctx.stroke();
    }

    ctx.restore();
    this.referenceDirty = false;
  }

  clearExtrusions() {
    this.extrusionCtx.clearRect(0, 0, this.extrusionCanvas.width, this.extrusionCanvas.height);
    this.drawnExtrusionCount = 0;
  }

  clearPositionTrace({ keepMarkers = true } = {}) {
    if (this.positionTraceCtx) {
      this.positionTraceCtx.clearRect(0, 0, this.positionTraceCanvas.width, this.positionTraceCanvas.height);
    }
    this.drawnPositionTraceCount = 0;
    if (!keepMarkers) {
      this.positionTraceMarkers = [];
    }
  }

  clearPositionTracePoints() {
    this.positionTracePoints = [];
    this.drawnPositionTraceCount = 0;
    if (this.positionTraceCtx) {
      this.positionTraceCtx.clearRect(0, 0, this.positionTraceCanvas.width, this.positionTraceCanvas.height);
    }
  }

  clearPositionTraceMarkers() {
    this.positionTraceMarkers = [];
  }

  setPositionTraceEnabled(enabled, options = {}) {
    this.positionTraceEnabled = Boolean(enabled);
    if (options && typeof options === 'object') {
      if (typeof options.radiusPx === 'number' && isFinite(options.radiusPx) && options.radiusPx > 0) {
        this.positionTraceRadiusPx = options.radiusPx;
      }
      if (typeof options.color === 'string' && options.color.length > 0) {
        this.positionTraceColor = options.color;
      }
    }
  }

  addPositionTraceMarker(simX, simY, labelText) {
    if (!Number.isFinite(simX) || !Number.isFinite(simY)) {
      return;
    }
    this.positionTraceMarkers.push({
      x: simX,
      y: simY,
      text: typeof labelText === 'string' ? labelText : '',
    });
  }

  _drawPositionTraceAxes() {
    const ctx = this.c;
    const widthSim = this.canvas.width / this.effectiveCScale;
    const heightSim = this.canvas.height / this.effectiveCScale;
    const minX = this.viewOffsetX_sim - widthSim / 2;
    const maxX = this.viewOffsetX_sim + widthSim / 2;
    const minY = this.viewOffsetY_sim - heightSim / 2;
    const maxY = this.viewOffsetY_sim + heightSim / 2;

    ctx.save();
    ctx.lineWidth = 1;

    ctx.strokeStyle = 'rgba(255,255,255,0.8)';
    ctx.beginPath();
    ctx.moveTo(this.cX(minX), this.cY(0));
    ctx.lineTo(this.cX(maxX), this.cY(0));
    ctx.stroke();

    ctx.strokeStyle = 'rgba(255,255,255,0.8)';
    ctx.beginPath();
    ctx.moveTo(this.cX(0), this.cY(minY));
    ctx.lineTo(this.cX(0), this.cY(maxY));
    ctx.stroke();

    ctx.restore();
  }

  _drawPositionTraceMarkers() {
    if (!Array.isArray(this.positionTraceMarkers) || this.positionTraceMarkers.length === 0) {
      return;
    }
    const ctx = this.c;
    ctx.save();
    ctx.fillStyle = 'rgba(102, 204, 255, 0.9)';
    ctx.font = '12px sans-serif';
    ctx.textBaseline = 'top';
    for (const marker of this.positionTraceMarkers) {
      if (!marker) continue;
      const px = this.cX(marker.x);
      const py = this.cY(marker.y);
      if (!Number.isFinite(px) || !Number.isFinite(py)) continue;
      ctx.beginPath();
      ctx.arc(px, py, 3, 0, 2 * Math.PI);
      ctx.fill();
      if (marker.text) {
        ctx.fillText(marker.text, px + 6, py + 4);
      }
    }
    ctx.restore();
  }

  // Shift already drawn extrusions when panning without redrawing everything.
  // deltaOffsetX_sim and deltaOffsetY_sim are in simulation units (change in view offsets).
  // Only use when zoom (scale) is unchanged.
  shiftExtrusionsForPan(world, deltaOffsetX_sim, deltaOffsetY_sim) {
    if (!this.extrusionCanvas || !this.extrusionCtx) return;
    if (!Number.isFinite(deltaOffsetX_sim) && !Number.isFinite(deltaOffsetY_sim)) return;

    const scale = this.effectiveCScale; // pixels per sim unit at current zoom
    const dxPx = -deltaOffsetX_sim * scale;
    const dyPx =  deltaOffsetY_sim * scale;

    if (Math.abs(dxPx) < 0.5 && Math.abs(dyPx) < 0.5) {
      // Negligible shift; skip to avoid extra work.
      return;
    }

    const w = this.extrusionCanvas.width | 0;
    const h = this.extrusionCanvas.height | 0;
    if (w <= 0 || h <= 0) return;

    // Copy old buffer shifted into a temp canvas
    const tmp = document.createElement('canvas');
    tmp.width = w;
    tmp.height = h;
    const tctx = tmp.getContext('2d');
    tctx.clearRect(0, 0, w, h);
    tctx.drawImage(this.extrusionCanvas, dxPx, dyPx);

    // Swap back into the extrusion canvas
    this.extrusionCtx.clearRect(0, 0, w, h);
    this.extrusionCtx.drawImage(tmp, 0, 0);

    if (this.positionTraceCanvas && this.positionTraceCtx) {
      const tmpTrace = document.createElement('canvas');
      tmpTrace.width = w;
      tmpTrace.height = h;
      const tctx = tmpTrace.getContext('2d');
      tctx.clearRect(0, 0, w, h);
      tctx.drawImage(this.positionTraceCanvas, dxPx, dyPx);
      this.positionTraceCtx.clearRect(0, 0, w, h);
      this.positionTraceCtx.drawImage(tmpTrace, 0, 0);
    }

    // Determine newly exposed regions (up to four edge strips)
    const rects = [];
    if (dxPx > 0) {
      const rw = Math.min(dxPx, w);
      if (rw > 0) rects.push({ x: 0, y: 0, w: rw, h }); // left strip
    } else if (dxPx < 0) {
      const rw = Math.min(-dxPx, w);
      if (rw > 0) rects.push({ x: w - rw, y: 0, w: rw, h }); // right strip
    }
    if (dyPx > 0) {
      const rh = Math.min(dyPx, h);
      if (rh > 0) rects.push({ x: 0, y: 0, w: w, h: rh }); // top strip
    } else if (dyPx < 0) {
      const rh = Math.min(-dyPx, h);
      if (rh > 0) rects.push({ x: 0, y: h - rh, w: w, h: rh }); // bottom strip
    }

    if (rects.length === 0) return;

    // Backfill previously-drawn extrusions that were offscreen and now come into view.
    // We iterate already-rendered history only (up to drawnExtrusionCount).
    // This avoids re-rendering everything while making the newly exposed strips correct.
    // Note: Uses current view transform (this.viewOffset*, this.effectiveCScale),
    // so the caller should update setViewTransform before or after consistently.
    const extruderEntities = world.query([ExtruderComponent]);
    if (extruderEntities.length === 0) return;
    const extruderComp = world.getComponent(extruderEntities[0], ExtruderComponent);
    if (!extruderComp || !Array.isArray(extruderComp.extrusions)) return;

    const n = Math.min(this.drawnExtrusionCount, extruderComp.extrusions.length) | 0;
    if (n <= 0) return;

    this.extrusionCtx.save();
    this.extrusionCtx.beginPath();
    for (let r = 0; r < rects.length; r++) {
      const R = rects[r];
      this.extrusionCtx.rect(R.x, R.y, R.w, R.h);
    }
    this.extrusionCtx.clip();

    for (let i = 0; i < n; i++) {
      const extrusion = extruderComp.extrusions[i];
      if (!extrusion) continue;
      const pos = Array.isArray(extrusion) ? extrusion[0] : extrusion.pos;
      const length = Array.isArray(extrusion) ? extrusion[1] : extrusion.length;
      const color = this._getExtrusionColor(extrusion);

      const radiusSim = Math.sqrt(length / Math.PI) * 0.01 * 0.5;
      const px = this.cX(pos[0]);
      const py = this.cY(pos[1]);
      const pr = radiusSim * this.effectiveCScale;

      // Quick reject: only draw if inside any uncovered strip
      for (let r = 0; r < rects.length; r++) {
        const R = rects[r];
        if (px + pr < R.x || px - pr > R.x + R.w || py + pr < R.y || py - pr > R.y + R.h) {
          continue;
        }
        this.extrusionCtx.fillStyle = this._colorWithAlpha(color, 0.5);
        this.extrusionCtx.beginPath();
        this.extrusionCtx.arc(px, py, pr, 0, 2 * Math.PI);
        this.extrusionCtx.fill();
        break;
      }
    }
    this.extrusionCtx.restore();

    // Backfill position trace dots for newly exposed strips.
    if (this.positionTraceEnabled && this.positionTraceCtx && Array.isArray(this.positionTracePoints)) {
      const nTrace = Math.min(this.drawnPositionTraceCount, this.positionTracePoints.length) | 0;
      if (nTrace > 0) {
        this.positionTraceCtx.save();
        this.positionTraceCtx.beginPath();
        for (let r = 0; r < rects.length; r++) {
          const R = rects[r];
          this.positionTraceCtx.rect(R.x, R.y, R.w, R.h);
        }
        this.positionTraceCtx.clip();
        this.positionTraceCtx.fillStyle = this.positionTraceColor;
        for (let i = 0; i < nTrace; i++) {
          const pt = this.positionTracePoints[i];
          if (!pt) continue;
          const px = this.cX(pt[0]);
          const py = this.cY(pt[1]);
          if (!Number.isFinite(px) || !Number.isFinite(py)) continue;
          for (let r = 0; r < rects.length; r++) {
            const R = rects[r];
            if (px + this.positionTraceRadiusPx < R.x || px - this.positionTraceRadiusPx > R.x + R.w
              || py + this.positionTraceRadiusPx < R.y || py - this.positionTraceRadiusPx > R.y + R.h) {
              continue;
            }
            this.positionTraceCtx.beginPath();
            this.positionTraceCtx.arc(px, py, this.positionTraceRadiusPx, 0, 2 * Math.PI);
            this.positionTraceCtx.fill();
            break;
          }
        }
        this.positionTraceCtx.restore();
      }
    }
  }


  _nowSeconds() {
    if (typeof performance !== 'undefined' && typeof performance.now === 'function') {
      return performance.now() * 0.001;
    }
    return Date.now() * 0.001;
  }

  _clampByte(value) {
    if (!Number.isFinite(value)) {
      return 0;
    }
    return Math.max(0, Math.min(255, Math.round(value)));
  }

  _parseColorToRgb(color) {
    if (typeof color !== 'string') {
      return null;
    }
    const trimmed = color.trim();
    if (trimmed.length === 0) {
      return null;
    }
    if (trimmed.startsWith('#')) {
      let hex = trimmed.slice(1);
      if (hex.length === 3) {
        hex = hex.split('').map((ch) => ch + ch).join('');
      }
      if (hex.length === 6) {
        const r = Number.parseInt(hex.slice(0, 2), 16);
        const g = Number.parseInt(hex.slice(2, 4), 16);
        const b = Number.parseInt(hex.slice(4, 6), 16);
        if (Number.isFinite(r) && Number.isFinite(g) && Number.isFinite(b)) {
          return [r, g, b];
        }
      }
      return null;
    }
    if (trimmed.startsWith('rgb')) {
      const parts = trimmed.match(/\d+(?:\.\d+)?/g);
      if (parts && parts.length >= 3) {
        const r = Number(parts[0]);
        const g = Number(parts[1]);
        const b = Number(parts[2]);
        if (Number.isFinite(r) && Number.isFinite(g) && Number.isFinite(b)) {
          return [this._clampByte(r), this._clampByte(g), this._clampByte(b)];
        }
      }
    }
    return null;
  }

  _rgbHue(rgb) {
    if (!Array.isArray(rgb) || rgb.length < 3) {
      return null;
    }
    const r = rgb[0] / 255;
    const g = rgb[1] / 255;
    const b = rgb[2] / 255;
    const max = Math.max(r, g, b);
    const min = Math.min(r, g, b);
    const delta = max - min;
    if (delta <= 1e-9) {
      return null;
    }
    let hue = 0.0;
    if (max === r) {
      hue = ((g - b) / delta) % 6;
    } else if (max === g) {
      hue = ((b - r) / delta) + 2;
    } else {
      hue = ((r - g) / delta) + 4;
    }
    hue *= 60.0;
    if (hue < 0) {
      hue += 360.0;
    }
    return hue;
  }

  _fxPaletteForObstacleColor(color) {
    const rgb = this._parseColorToRgb(color);
    const hue = this._rgbHue(rgb);
    const isBlue = Number.isFinite(hue) && hue >= 170.0 && hue <= 260.0;
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

  _rgba(rgb, alpha) {
    if (!Array.isArray(rgb) || rgb.length < 3) {
      return `rgba(255, 160, 40, ${alpha})`;
    }
    const r = this._clampByte(rgb[0]);
    const g = this._clampByte(rgb[1]);
    const b = this._clampByte(rgb[2]);
    return `rgba(${r}, ${g}, ${b}, ${alpha})`;
  }

  _spawnBumperWobble(world, obstacleId, outwardDir, intensity = 1.0) {
    if (!Number.isInteger(obstacleId)) {
      return;
    }
    let dir = null;
    if (
      outwardDir &&
      Number.isFinite(outwardDir.x) &&
      Number.isFinite(outwardDir.y)
    ) {
      dir = outwardDir.clone();
    } else {
      dir = new Vector2(1.0, 0.0);
    }
    if (dir.lengthSq() <= 1e-12) {
      dir.x = 1.0;
      dir.y = 0.0;
    } else {
      dir.normalize();
    }

    const radiusValue = world.getComponent(obstacleId, RadiusComponent)?.radius;
    const radius = (Number.isFinite(radiusValue) && radiusValue > 1e-6) ? radiusValue : 0.02;
    const ampBase = Math.max(0.00045, Math.min(0.0022, radius * 0.08));
    const ampScale = Math.max(0.65, Math.min(1.4, 0.72 + 0.22 * intensity));

    this.bumperWobbleByEntity.set(obstacleId, {
      startSec: this._nowSeconds(),
      lifeSec: 0.1,
      amplitude: ampBase * ampScale,
      dirX: dir.x,
      dirY: dir.y,
      freqHz: 16.0 + Math.random() * 8.0,
      phase: Math.random() * Math.PI * 2.0
    });
  }

  _getBumperWobbleOffset(entityId, nowSec = null) {
    const wobble = this.bumperWobbleByEntity.get(entityId);
    if (!wobble) {
      return null;
    }
    const timeNow = Number.isFinite(nowSec) ? nowSec : this._nowSeconds();
    const elapsed = timeNow - wobble.startSec;
    if (!Number.isFinite(elapsed) || elapsed < 0.0) {
      return null;
    }
    if (elapsed >= wobble.lifeSec) {
      this.bumperWobbleByEntity.delete(entityId);
      return null;
    }
    const progress = wobble.lifeSec > 1e-9 ? Math.min(1.0, elapsed / wobble.lifeSec) : 1.0;
    const envelope = (1.0 - progress) * (1.0 - progress);
    const oscillation = Math.sin((2.0 * Math.PI * wobble.freqHz * elapsed) + wobble.phase);
    const kick = wobble.amplitude * 0.24 * Math.exp(-48.0 * elapsed);
    const offset = wobble.amplitude * envelope * oscillation + kick;
    return {
      x: wobble.dirX * offset,
      y: wobble.dirY * offset
    };
  }

  _spawnBumperHitBurst(world, contact, pushVel = 0.0) {
    const obsPos = world.getComponent(contact.obs_id, PositionComponent)?.pos;
    if (!obsPos) {
      return;
    }

    let outwardDir = null;
    if (
      contact.direction &&
      Number.isFinite(contact.direction.x) &&
      Number.isFinite(contact.direction.y)
    ) {
      outwardDir = contact.direction.clone();
    } else {
      const ballPos = world.getComponent(contact.ball_id, PositionComponent)?.pos;
      if (ballPos) {
        outwardDir = ballPos.clone().subtract(obsPos);
      }
    }
    if (!outwardDir || outwardDir.lengthSq() <= 1e-12) {
      outwardDir = new Vector2(1.0, 0.0);
    } else {
      outwardDir.normalize();
    }

    const obstacleRadiusValue = world.getComponent(contact.obs_id, RadiusComponent)?.radius;
    const obstacleRadius = (
      Number.isFinite(obstacleRadiusValue) && obstacleRadiusValue > 1e-6
        ? obstacleRadiusValue
        : 0.03
    );
    const effectRadius = Math.max(0.008, obstacleRadius * 0.5);

    const deltaLambda = Number.isFinite(contact.delta_lambda)
      ? Math.max(0.0, contact.delta_lambda)
      : 0.0;
    const intensity = Math.max(
      0.7,
      Math.min(2.8, 0.85 + (0.2 * pushVel) + (0.32 * Math.sqrt(deltaLambda + 1e-9)))
    );
    const spread = (Math.PI / 7.0) + (Math.random() * 0.16);
    const baseAngle = Math.atan2(outwardDir.y, outwardDir.x);
    const obstacleColor = world.getComponent(contact.obs_id, RenderableComponent)?.color;
    const palette = this._fxPaletteForObstacleColor(obstacleColor);
    this._spawnBumperWobble(world, contact.obs_id, outwardDir, intensity);

    const rayCount = Math.max(8, Math.min(16, Math.round(8 + intensity * 2 + (Math.random() * 2))));
    const rays = [];
    for (let i = 0; i < rayCount; i++) {
      const offset = (Math.random() * 2.0 - 1.0) * spread;
      const inward = Math.random() < 0.18;
      rays.push({
        offset,
        lengthScale: inward ? (0.24 + Math.random() * 0.28) : (0.78 + Math.random() * 0.62),
        widthScale: 0.7 + Math.random() * 0.75,
        alphaScale: 0.52 + Math.random() * 0.36,
        surfaceOffsetScale: -0.08 + Math.random() * 0.42,
        inward,
        inwardScale: 0.5 + Math.random() * 0.4,
        flicker: Math.random() * Math.PI * 2.0,
        hot: Math.random() < 0.52
      });
    }

    const sparkCount = Math.max(6, Math.min(14, Math.round(6 + intensity * 3 + (Math.random() * 2))));
    const sparks = [];
    const originX = obsPos.x + outwardDir.x * effectRadius;
    const originY = obsPos.y + outwardDir.y * effectRadius;
    for (let i = 0; i < sparkCount; i++) {
      const sparkAngle = baseAngle + ((Math.random() * 2.0 - 1.0) * spread * 1.2);
      const sparkSpeed = (0.2 + Math.random() * 0.5) * (0.45 + 0.5 * intensity);
      sparks.push({
        x: originX,
        y: originY,
        vx: Math.cos(sparkAngle) * sparkSpeed,
        vy: Math.sin(sparkAngle) * sparkSpeed,
        age: 0.0,
        life: 0.1 + Math.random() * 0.24,
        width: 0.7 + Math.random() * 1.3,
        hot: Math.random() < 0.5
      });
    }

    const ballSparks = [];
    const ballPos = world.getComponent(contact.ball_id, PositionComponent)?.pos;
    const ballRadiusValue = world.getComponent(contact.ball_id, RadiusComponent)?.radius;
    if (ballPos && Number.isFinite(ballRadiusValue) && ballRadiusValue > 1e-6) {
      const ballRadius = ballRadiusValue;
      const startX = ballPos.x - outwardDir.x * ballRadius;
      const startY = ballPos.y - outwardDir.y * ballRadius;
      const tangent = new Vector2(-outwardDir.y, outwardDir.x);
      const tangentSign = Math.random() < 0.5 ? -1.0 : 1.0;
      const sparkDir = outwardDir.clone().scale(-0.68).add(tangent, 0.35 * tangentSign).normalize();
      const speed = (0.25 + Math.random() * 0.33) * (0.8 + 0.22 * intensity);
      ballSparks.push({
        x: startX,
        y: startY,
        vx: sparkDir.x * speed,
        vy: sparkDir.y * speed,
        age: 0.0,
        life: 0.07 + Math.random() * 0.07,
        width: 1.1 + Math.random() * 0.9
      });
    }

    this.bumperHitFxBursts.push({
      x: obsPos.x,
      y: obsPos.y,
      dirX: outwardDir.x,
      dirY: outwardDir.y,
      bumperRadius: obstacleRadius,
      effectRadius,
      age: 0.0,
      life: 0.52 + Math.random() * 0.2,
      spread,
      maxRayLength: (0.035 + 0.05 * intensity),
      palette,
      rays,
      sparks,
      ballSparks
    });

    const MAX_BURSTS = 80;
    if (this.bumperHitFxBursts.length > MAX_BURSTS) {
      this.bumperHitFxBursts.splice(0, this.bumperHitFxBursts.length - MAX_BURSTS);
    }
  }

  _updateAndRenderBumperHitFx(world) {
    const nowSec = this._nowSeconds();
    if (!Number.isFinite(this.bumperHitFxLastTimeSec)) {
      this.bumperHitFxLastTimeSec = nowSec;
    }
    let dtSec = nowSec - this.bumperHitFxLastTimeSec;
    this.bumperHitFxLastTimeSec = nowSec;
    if (!Number.isFinite(dtSec) || dtSec < 0.0) {
      dtSec = 0.0;
    }
    dtSec = Math.min(dtSec, 0.05);
    const dtFx = dtSec * 2.0;

    const enabled = world.getResource('renderBumperHitFx') === true;
    if (!enabled) {
      this.bumperHitFxBursts.length = 0;
      this.bumperHitFxActivePairs.clear();
      this.bumperWobbleByEntity.clear();
      return;
    }

    const contacts = Array.isArray(world.getResource('ball_obstacle_contacts'))
      ? world.getResource('ball_obstacle_contacts')
      : [];
    const nextActivePairs = new Set();
    for (const contact of contacts) {
      if (!contact || contact.raw_hit === false) {
        continue;
      }
      const pushComp = world.getComponent(contact.obs_id, ObstaclePushComponent);
      if (!pushComp || !(pushComp.pushVel > 1e-9)) {
        continue;
      }
      const pairKey = `${contact.ball_id}:${contact.obs_id}`;
      nextActivePairs.add(pairKey);
      if (!this.bumperHitFxActivePairs.has(pairKey)) {
        this._spawnBumperHitBurst(world, contact, pushComp.pushVel);
      }
    }
    this.bumperHitFxActivePairs = nextActivePairs;

    if (this.bumperHitFxBursts.length === 0) {
      return;
    }

    const drag = Math.exp(-6.5 * dtFx);
    for (const burst of this.bumperHitFxBursts) {
      burst.age += dtFx;
      if (!Array.isArray(burst.sparks) || burst.sparks.length === 0) {
        continue;
      }
      const nextSparks = [];
      for (const spark of burst.sparks) {
        spark.age += dtFx;
        if (spark.age >= spark.life) {
          continue;
        }
        spark.x += spark.vx * dtFx;
        spark.y += spark.vy * dtFx;
        spark.vx *= drag;
        spark.vy *= drag;
        spark.vy -= 0.45 * dtFx;
        nextSparks.push(spark);
      }
      burst.sparks = nextSparks;

      if (Array.isArray(burst.ballSparks) && burst.ballSparks.length > 0) {
        const nextBallSparks = [];
        for (const spark of burst.ballSparks) {
          spark.age += dtFx;
          if (spark.age >= spark.life) {
            continue;
          }
          spark.x += spark.vx * dtFx;
          spark.y += spark.vy * dtFx;
          spark.vx *= drag;
          spark.vy *= drag;
          nextBallSparks.push(spark);
        }
        burst.ballSparks = nextBallSparks;
      }
    }

    this.bumperHitFxBursts = this.bumperHitFxBursts.filter((burst) => (
      (burst.age < burst.life) ||
      (Array.isArray(burst.sparks) && burst.sparks.length > 0) ||
      (Array.isArray(burst.ballSparks) && burst.ballSparks.length > 0)
    ));

    if (this.bumperHitFxBursts.length === 0) {
      return;
    }

    this.c.save();
    this.c.globalCompositeOperation = 'lighter';
    this.c.lineCap = 'round';
    this.c.lineJoin = 'round';
    const unitPx = Math.max(0.8, this.effectiveCScale / 250);

    for (const burst of this.bumperHitFxBursts) {
      const progress = burst.life > 1e-9 ? Math.min(1.0, burst.age / burst.life) : 1.0;
      const fade = Math.max(0.0, 1.0 - progress);
      const baseAngle = Math.atan2(burst.dirY, burst.dirX);
      const coneReach = burst.maxRayLength * (0.28 + 0.95 * progress);
      const originDistance = burst.bumperRadius + burst.effectRadius * (0.05 + 0.1 * progress);
      const originX = burst.x + burst.dirX * originDistance;
      const originY = burst.y + burst.dirY * originDistance;
      const palette = burst.palette || this._fxPaletteForObstacleColor(null);

      if (fade > 0.0) {
        const leftAngle = baseAngle - burst.spread;
        const rightAngle = baseAngle + burst.spread;
        const innerReach = coneReach * 0.58;

        this.c.fillStyle = this._rgba(palette.deep, 0.16 * fade);
        this.c.beginPath();
        this.c.moveTo(this.cX(originX), this.cY(originY));
        this.c.lineTo(
          this.cX(originX + Math.cos(leftAngle) * coneReach),
          this.cY(originY + Math.sin(leftAngle) * coneReach)
        );
        this.c.lineTo(
          this.cX(originX + Math.cos(rightAngle) * coneReach),
          this.cY(originY + Math.sin(rightAngle) * coneReach)
        );
        this.c.closePath();
        this.c.fill();

        this.c.fillStyle = this._rgba(palette.core, 0.24 * fade);
        this.c.beginPath();
        this.c.moveTo(this.cX(originX), this.cY(originY));
        this.c.lineTo(
          this.cX(originX + Math.cos(baseAngle - burst.spread * 0.52) * innerReach),
          this.cY(originY + Math.sin(baseAngle - burst.spread * 0.52) * innerReach)
        );
        this.c.lineTo(
          this.cX(originX + Math.cos(baseAngle + burst.spread * 0.52) * innerReach),
          this.cY(originY + Math.sin(baseAngle + burst.spread * 0.52) * innerReach)
        );
        this.c.closePath();
        this.c.fill();

        for (const ray of burst.rays) {
          const flutter = 1.0 + 0.24 * Math.sin(ray.flicker + progress * 13.0);
          const angle = baseAngle + ray.offset * flutter;
          const startDistance = burst.bumperRadius + (burst.effectRadius * ray.surfaceOffsetScale);
          const startX = burst.x + Math.cos(angle) * startDistance;
          const startY = burst.y + Math.sin(angle) * startDistance;
          const length = coneReach * ray.lengthScale;
          let endX = startX + Math.cos(angle) * length;
          let endY = startY + Math.sin(angle) * length;
          if (ray.inward === true) {
            const endDistance = Math.max(
              burst.bumperRadius * 0.15,
              startDistance - (length * ray.inwardScale)
            );
            endX = burst.x + Math.cos(angle) * endDistance;
            endY = burst.y + Math.sin(angle) * endDistance;
          }
          const rayColor = ray.hot ? palette.hot : palette.core;
          this.c.strokeStyle = this._rgba(rayColor, ray.alphaScale * fade);
          this.c.lineWidth = Math.max(0.65, (1.1 * ray.widthScale * unitPx * (0.6 + fade)));
          this.c.beginPath();
          this.c.moveTo(this.cX(startX), this.cY(startY));
          this.c.lineTo(this.cX(endX), this.cY(endY));
          this.c.stroke();
        }
      }

      if (Array.isArray(burst.sparks) && burst.sparks.length > 0) {
        for (const spark of burst.sparks) {
          const sparkFade = Math.max(0.0, 1.0 - (spark.age / spark.life));
          const tailX = spark.x - spark.vx * (0.03 + 0.02 * sparkFade);
          const tailY = spark.y - spark.vy * (0.03 + 0.02 * sparkFade);
          const sparkColor = spark.hot ? palette.hot : palette.core;
          this.c.strokeStyle = this._rgba(sparkColor, (spark.hot ? 0.65 : 0.5) * sparkFade);
          this.c.lineWidth = Math.max(0.55, spark.width * unitPx * (0.4 + sparkFade));
          this.c.beginPath();
          this.c.moveTo(this.cX(tailX), this.cY(tailY));
          this.c.lineTo(this.cX(spark.x), this.cY(spark.y));
          this.c.stroke();
        }
      }

      if (Array.isArray(burst.ballSparks) && burst.ballSparks.length > 0) {
        for (const spark of burst.ballSparks) {
          const sparkFade = Math.max(0.0, 1.0 - (spark.age / spark.life));
          const tailX = spark.x - spark.vx * (0.035 + 0.02 * sparkFade);
          const tailY = spark.y - spark.vy * (0.035 + 0.02 * sparkFade);

          this.c.strokeStyle = `rgba(150, 160, 172, ${0.52 * sparkFade})`;
          this.c.lineWidth = Math.max(0.75, spark.width * unitPx * (0.75 + sparkFade));
          this.c.beginPath();
          this.c.moveTo(this.cX(tailX), this.cY(tailY));
          this.c.lineTo(this.cX(spark.x), this.cY(spark.y));
          this.c.stroke();

          this.c.strokeStyle = `rgba(230, 236, 244, ${0.8 * sparkFade})`;
          this.c.lineWidth = Math.max(0.5, spark.width * unitPx * (0.38 + sparkFade * 0.52));
          this.c.beginPath();
          this.c.moveTo(this.cX(tailX), this.cY(tailY));
          this.c.lineTo(this.cX(spark.x), this.cY(spark.y));
          this.c.stroke();
        }
      }
    }

    this.c.restore();
  }


  update(world, dt) {
    if (this.drawingSuspended) {
      return;
    }
    // Viewport settings are now instance properties (this.viewScaleMultiplier, etc.)
    // effectiveCScale is also an instance property (this.effectiveCScale)

    this.c.clearRect(0, 0, this.canvas.width, this.canvas.height);

    if (this.positionTraceEnabled) {
      const extruderEntities = world.query([ExtruderComponent]);
      if (extruderEntities.length > 0) {
        const extruderComp = world.getComponent(extruderEntities[0], ExtruderComponent);
        const center = extruderComp?.centerPos;
        if (center && Number.isFinite(center.x) && Number.isFinite(center.y)) {
          this.positionTracePoints.push([center.x, center.y]);
        }
      }

      if (this.positionTraceCtx && Array.isArray(this.positionTracePoints)) {
        this.positionTraceCtx.fillStyle = this.positionTraceColor;
        for (let i = this.drawnPositionTraceCount; i < this.positionTracePoints.length; i++) {
          const pt = this.positionTracePoints[i];
          if (!pt) continue;
          const px = this.cX(pt[0]);
          const py = this.cY(pt[1]);
          if (!Number.isFinite(px) || !Number.isFinite(py)) continue;
          this.positionTraceCtx.beginPath();
          this.positionTraceCtx.arc(px, py, this.positionTraceRadiusPx, 0, 2 * Math.PI);
          this.positionTraceCtx.fill();
        }
        this.drawnPositionTraceCount = this.positionTracePoints.length;
      }

      this._drawPositionTraceAxes();
      if (this.positionTraceCanvas) {
        this.c.drawImage(this.positionTraceCanvas, 0, 0);
      }
    }

    // --- Query for Cable Link Obstacles ---
    this.cableLinkObstacles = [];
    const linkEntities = world.query([CableLinkComponent, PositionComponent, RadiusComponent]);
    for (const entityId of linkEntities) {
        const posComp = world.getComponent(entityId, PositionComponent);
        const radiusComp = world.getComponent(entityId, RadiusComponent);
        const machineTag = world.getComponent(entityId, MachineTagComponent);
        if (posComp && radiusComp) {
            this.cableLinkObstacles.push({
                pos: posComp.pos.clone(), // Use current position
                radius: radiusComp.radius,
                machineId: machineTag?.id || ''
            });
        }
    }
    // --- End Obstacle Query ---

    // Visualize orientation for entities with OrientationComponent
    {
      const oriEntities = world.query([OrientationComponent, PositionComponent, RadiusComponent]);
      if (oriEntities.length > 0) {
        this.c.save();
        this.c.strokeStyle = '#00AAFF';
        this.c.lineWidth = 2 * this.effectiveCScale/250;
        for (const e of oriEntities) {
          const pos = world.getComponent(e, PositionComponent)?.pos;
          const r   = world.getComponent(e, RadiusComponent)?.radius;
          const o   = world.getComponent(e, OrientationComponent)?.angle;
          if (!pos || !(r > 0) || typeof o !== 'number') continue;
          const tipX = pos.x + r * Math.cos(o);
          const tipY = pos.y + r * Math.sin(o);
          this.c.beginPath();
          this.c.moveTo(this.cX(pos.x), this.cY(pos.y));
          this.c.lineTo(this.cX(tipX), this.cY(tipY));
          this.c.stroke();
        }
        this.c.restore();
      }
    }


    // Base line width and debug radius (adjust as needed)
    const baseLineWidth = 2.0;
    const baseDebugRadius = 3;
    const fallbackLineWidthPx = Math.max(1.0, baseLineWidth * this.effectiveCScale / 250);
    const physicalCableLineWidthPx = (halfWidthSim) => {
      const halfWidth = Number.isFinite(halfWidthSim) ? Math.max(0.0, halfWidthSim) : 0.0;
      if (!(halfWidth > 1e-9)) {
        return fallbackLineWidthPx;
      }
      return Math.max(0.75, (2.0 * halfWidth) * this.effectiveCScale);
    };

    // Render Border
    const borderEntities = world.query([BorderComponent, RenderableComponent]);
    if (borderEntities.length > 0) {
      const borderComp = world.getComponent(borderEntities[0], BorderComponent);
      const renderComp = world.getComponent(borderEntities[0], RenderableComponent);
      const points = borderComp.points;
      if (points.length >= 2) {
        this.c.strokeStyle = renderComp.color;
        this.c.lineWidth = 5; // Make this a component property?
        this.c.beginPath();
        let v = points[0];
        this.c.moveTo(this.cX(v.x), this.cY(v.y));
        for (let i = 1; i < points.length + 1; i++) {
          v = points[i % points.length];
          this.c.lineTo(this.cX(v.x), this.cY(v.y));
        }
        this.c.stroke();
        this.c.lineWidth = 1;
      }
    }

    // Render Flippers
    const flipperEntities = world.query([FlipperTagComponent, PositionComponent, RadiusComponent, FlipperStateComponent, RenderableComponent]);
    for (const entityId of flipperEntities) {
      const posComp = world.getComponent(entityId, PositionComponent);
      const radiusComp = world.getComponent(entityId, RadiusComponent);
      const stateComp = world.getComponent(entityId, FlipperStateComponent);
      const renderComp = world.getComponent(entityId, RenderableComponent);

      this.c.fillStyle = renderComp.color;

      // Simulation-space coordinates and scale
      const simX = posComp.pos.x;
      const simY = posComp.pos.y;
      const angle = stateComp.restAngle + stateComp.sign * stateComp.rotation;

      // Convert to canvas pixels
      const pivotX = this.cX(simX);
      const pivotY = this.cY(simY);
      const Lp = stateComp.length * this.effectiveCScale;
      const Rp = radiusComp.radius * this.effectiveCScale;

      this.c.save();
      this.c.translate(pivotX, pivotY);
      this.c.rotate(-angle);
      this.c.fillRect(0, -Rp, Lp, 2 * Rp);
      this.c.restore();

      // Draw end caps in simulation space
      this.c.fillStyle = renderComp.color;
      // Pivot disc
      this.drawDisc(simX, simY, radiusComp.radius);
      // Tip disc
      const tipSimX = simX + stateComp.length * Math.cos(angle);
      const tipSimY = simY + stateComp.length * Math.sin(angle);
      this.drawDisc(tipSimX, tipSimY, radiusComp.radius);
    }



    // Render Cable Joints (Lines)
    const pathEntities = world.query([CablePathComponent]);
    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (path.jointEntities.length < 1) continue;
      const pathMachineId = world.getComponent(pathId, MachineTagComponent)?.id || '';
      const jointEntities = path.jointEntities;
      const pathLineWidthPx = physicalCableLineWidthPx(path.cableHalfWidth);
      this.c.lineWidth = pathLineWidthPx;
      for (const entityId of jointEntities) {
        const jointComp = world.getComponent(entityId, CableJointComponent);
        const renderComp = world.getComponent(entityId, RenderableComponent);
        const pA = jointComp.attachmentPointA_world;
        const pB = jointComp.attachmentPointB_world;
        if (renderComp) {
            this.c.strokeStyle = renderComp.color;
        } else {
            this.c.strokeStyle = 'yellow';
        }
        this.c.beginPath();
        // Draw catenary if slack, otherwise straight
        const straightDist = pA.distanceTo(pB);
        if (jointComp.restLength > straightDist + 1e-6) {
          this.c.strokeStyle = 'orange';
          this._drawCatenary(entityId, pA, pB, jointComp.restLength, this.cableLinkObstacles, pathMachineId);
        } else {
          this.c.moveTo(this.cX(pA.x), this.cY(pA.y));
          this.c.lineTo(this.cX(pB.x), this.cY(pB.y));
          this.c.stroke();
        }
      }
    }

    const layeringEnabled =
      world.getResource('enableLayering') !== false &&
      world.getResource('layeringRenderWraps') !== false;

    const drawEndpointStoredWrap = (path, linkIndex, center, bodyRadius, attachmentPoint, renderComp) => {
      if (!center || !Number.isFinite(bodyRadius) || bodyRadius <= 1e-9 || !attachmentPoint) {
        return;
      }

      if (!layeringEnabled) {
        this.c.lineWidth = physicalCableLineWidthPx(path.cableHalfWidth);
        const radius = bodyRadius;
        if (!(radius > 1e-9)) {
          return;
        }
        const stored = Math.max(0.0, path.stored[linkIndex] ?? 0.0);
        if (!(stored > 1e-9)) {
          return;
        }
        const startAngle = Math.atan2(attachmentPoint.y - center.y, attachmentPoint.x - center.x);
        const cw = Boolean(path.cw[linkIndex]);
        const anticlockwise = !cw;
        const maxRenderableAngle = 2.0 * Math.PI - 0.0001;
        let deltaTheta = stored / radius;
        if (deltaTheta >= maxRenderableAngle) {
          deltaTheta = maxRenderableAngle;
        }
        const endAngle = cw ? startAngle - deltaTheta : startAngle + deltaTheta;

        this.c.beginPath();
        if (renderComp) {
          this.c.strokeStyle = renderComp.color;
        } else {
          this.c.strokeStyle = 'yellow';
        }
        this.c.arc(
          this.cX(center.x),
          this.cY(center.y),
          radius * this.effectiveCScale,
          -startAngle,
          -endAngle,
          anticlockwise
        );
        this.c.stroke();
        return;
      }

      const halfWidth = path.cableHalfWidth ?? 0.0;
      this.c.lineWidth = physicalCableLineWidthPx(halfWidth);
      const baseRadius = bodyRadius + halfWidth;
      if (!(baseRadius > 1e-9)) {
        return;
      }

      let remainingLength = Math.max(0.0, path.stored[linkIndex] ?? 0.0);
      if (!(remainingLength > 1e-9)) {
        return;
      }

      const startAngle = Math.atan2(attachmentPoint.y - center.y, attachmentPoint.x - center.x);
      const cw = Boolean(path.cw[linkIndex]);
      const anticlockwise = !cw;
      const fullWidth = 2.0 * halfWidth;
      const maxRenderableAngle = 2.0 * Math.PI - 0.0001;
      const MAX_LAYERS = 128;

      if (renderComp) {
        this.c.strokeStyle = renderComp.color;
      } else {
        this.c.strokeStyle = 'yellow';
      }

      let layerIndex = 0;
      while (remainingLength > 1e-9 && layerIndex < MAX_LAYERS) {
        const layerRadius = baseRadius + fullWidth * layerIndex;
        if (!(layerRadius > 1e-9)) {
          break;
        }
        const layerCircumference = 2.0 * Math.PI * layerRadius;
        if (!(layerCircumference > 1e-9)) {
          break;
        }
        const layerArcLength = Math.min(remainingLength, layerCircumference);
        if (!(layerArcLength > 1e-9)) {
          break;
        }

        this.c.beginPath();
        if (layerArcLength >= layerCircumference - 1e-6) {
          this.c.arc(
            this.cX(center.x),
            this.cY(center.y),
            layerRadius * this.effectiveCScale,
            0.0,
            2.0 * Math.PI
          );
        } else {
          let deltaTheta = layerArcLength / layerRadius;
          if (deltaTheta >= maxRenderableAngle) {
            deltaTheta = maxRenderableAngle;
          }
          const endAngle = cw ? startAngle - deltaTheta : startAngle + deltaTheta;
          this.c.arc(
            this.cX(center.x),
            this.cY(center.y),
            layerRadius * this.effectiveCScale,
            -startAngle,
            -endAngle,
            anticlockwise
          );
        }
        this.c.stroke();

        remainingLength -= layerArcLength;
        if (!(fullWidth > 1e-12)) {
          break;
        }
        layerIndex++;
      }
    };

    for (const pathId of pathEntities) {
      const path   = world.getComponent(pathId, CablePathComponent);
      if (path.jointEntities.length < 1) continue;
      const joints = path.jointEntities;
      const halfWidth = layeringEnabled ? (path.cableHalfWidth ?? 0.0) : 0.0;
      const pathLineWidthPx = physicalCableLineWidthPx(path.cableHalfWidth);
      // rolling or hybrid link types mean the cable can wrap on that entity
      for (let i = 1; i < path.linkTypes.length - 1; i++) {
        // Only draw wrap-around arc for rolling and hybrid links (when in rolling mode)
        if (path.linkTypes[i] !== 'rolling' && path.linkTypes[i] !== 'hybrid') continue;

        // corner joint before & after this roller
        const jPrev = world.getComponent(joints[i - 1], CableJointComponent);
        const jNext = world.getComponent(joints[i    ], CableJointComponent);
        // roller is the shared entityB of the previous joint
        const rollerId   = jPrev.entityB;
        const centerComp = world.getComponent(rollerId, PositionComponent);
        const radiusComp = world.getComponent(rollerId, RadiusComponent);
        const renderComp = world.getComponent(joints[i], RenderableComponent);
        if (!centerComp || !radiusComp) continue;
        const C    = centerComp.pos;
        const R    = radiusComp.radius + halfWidth;
        const P1   = jPrev.attachmentPointB_world;
        const P2   = jNext.attachmentPointA_world;
        // angles in sim coords
        const a1 = Math.atan2(P1.y - C.y, P1.x - C.x);
        const a2 = Math.atan2(P2.y - C.y, P2.x - C.x);
        // cw‐flag stored in path.cw[i]
        const anticlockwise = !path.cw[i];
        this.c.lineWidth = pathLineWidthPx;
        this.c.beginPath();

        // Determine tension for color
        const epsilon = 1e-6;
        const distPrev = jPrev.attachmentPointA_world.distanceTo(jPrev.attachmentPointB_world);
        const tensionPrev = distPrev > (jPrev.restLength + epsilon);

        const distNext = jNext.attachmentPointA_world.distanceTo(jNext.attachmentPointB_world);
        const tensionNext = distNext > (jNext.restLength + epsilon);

        if (tensionPrev && tensionNext) {
          if (renderComp) {
              this.c.strokeStyle = renderComp.color;
          } else {
              this.c.strokeStyle = 'yellow';
          }
        } else {
          this.c.strokeStyle = 'orange';
        }
        this.c.arc(
          this.cX(C.x),
          this.cY(C.y),
          R * this.effectiveCScale,
          -a1,                                  // negate for canvas’ flipped y‐axis
          -a2,
          anticlockwise
        );
        this.c.stroke();
      }

      // now draw wrap‐arcs at the two ends (i=0 and i=last)
      const nLinks = path.linkTypes.length;

      // Front end (link index 0)
      if (path.linkTypes[0] === 'hybrid') {
        // joint 0 ties into link 0 on its A side
        const joint0 = world.getComponent(joints[0], CableJointComponent);
        const renderComp = world.getComponent(joints[0], RenderableComponent);
        const rollerA = joint0.entityA;
        const cA    = world.getComponent(rollerA, PositionComponent)?.pos;
        const rA    = world.getComponent(rollerA, RadiusComponent)?.radius;
        const P0    = joint0.attachmentPointA_world;
        if (cA && rA !== null) {
          drawEndpointStoredWrap(path, 0, cA, rA, P0, renderComp);
        }
      }

      // Back end (link index = nLinks-1)
      if (path.linkTypes[nLinks - 1] === 'hybrid') {
        // joint nLinks-2 ties into link nLinks-1 on its B side
        const jointN = world.getComponent(joints[nLinks - 2], CableJointComponent);
        const renderComp = world.getComponent(joints[nLinks - 2], RenderableComponent);
        const rollerB = jointN.entityB;
        const cB    = world.getComponent(rollerB, PositionComponent)?.pos;
        const rB    = world.getComponent(rollerB, RadiusComponent)?.radius;
        const P1    = jointN.attachmentPointB_world;

        if (cB && rB && rB > 1e-6) { // Added check for rB being reasonably positive
          drawEndpointStoredWrap(path, nLinks - 1, cB, rB, P1, renderComp);
        }
      }
    }
    this.c.lineWidth = 1;

    // Render Rigid Group visual edges
    const rigidGroups = world.query([RigidGroupComponent]);
    if (rigidGroups.length > 0) {
      this.c.save();
      this.c.lineWidth = 1 * this.effectiveCScale/250;
      this.c.strokeStyle = 'green';
      for (const gid of rigidGroups) {
        const group = world.getComponent(gid, RigidGroupComponent);
        const members = group?.members || [];
        const n = members.length;
        if (n < 2) {
          continue;
        }
        const memberPositions = members.map((entityId) => world.getComponent(entityId, PositionComponent)?.pos || null);
        const segments = Array.isArray(group?.renderSegments) ? group.renderSegments : null;
        const edges = [];
        const seen = new Set();
        const addEdge = (aIdx, bIdx) => {
          if (!Number.isInteger(aIdx) || !Number.isInteger(bIdx)) {
            return;
          }
          if (aIdx < 0 || bIdx < 0 || aIdx >= n || bIdx >= n || aIdx === bIdx) {
            return;
          }
          const key = aIdx < bIdx ? `${aIdx}:${bIdx}` : `${bIdx}:${aIdx}`;
          if (seen.has(key)) {
            return;
          }
          seen.add(key);
          edges.push([aIdx, bIdx]);
        };

        if (segments && segments.length > 0) {
          for (const segment of segments) {
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
        } else {
          for (let i = 0; i < n; i += 1) {
            for (let j = i + 1; j < n; j += 1) {
              addEdge(i, j);
            }
          }
        }

        for (const [aIdx, bIdx] of edges) {
          const pA = memberPositions[aIdx];
          const pB = memberPositions[bIdx];
          if (!pA || !pB) {
            continue;
          }
          this.c.beginPath();
          this.c.moveTo(this.cX(pA.x), this.cY(pA.y));
          this.c.lineTo(this.cX(pB.x), this.cY(pB.y));
          this.c.stroke();
        }
      }
      this.c.restore();
    }

    // Render Distance Constraints
    const distanceConstraintEntities = world.query([DistanceConstraintComponent]);
    if (distanceConstraintEntities.length > 0) {
        this.c.save();
        this.c.lineWidth = 3 * this.effectiveCScale/250;

        for (const entityId of distanceConstraintEntities) {
            const renderComp = world.getComponent(entityId, RenderableComponent);
            this.c.strokeStyle = renderComp.color;
            if (renderComp) {
              this.c.strokeStyle = renderComp.color;
            } else {
              this.c.strokeStyle = 'yellow';
            }
            const constraint = world.getComponent(entityId, DistanceConstraintComponent);
            const posAComp = world.getComponent(constraint.entityA, PositionComponent);
            const posBComp = world.getComponent(constraint.entityB, PositionComponent);

            if (posAComp && posBComp) {
                const pA = posAComp.pos;
                const pB = posBComp.pos;

                this.c.beginPath();
                this.c.moveTo(this.cX(pA.x), this.cY(pA.y));
                this.c.lineTo(this.cX(pB.x), this.cY(pB.y));
                this.c.stroke();
            }
        }
        this.c.restore();
    }


    // Render Extruder circle if present
    const extruderEntities = world.query([ExtruderComponent]);
    let maxExtrusions = this.drawnExtrusionCount;
    if (extruderEntities.length > 0) {
        for (const extruderEntity of extruderEntities) {
            const extruderComp = world.getComponent(extruderEntity, ExtruderComponent);
            if (!extruderComp || !Array.isArray(extruderComp.extrusions)) {
                continue;
            }
            for (let i = this.drawnExtrusionCount; i < extruderComp.extrusions.length; i++) {
                const extrusion = extruderComp.extrusions[i];
                if (!extrusion) {
                    continue;
                }
                const pos = Array.isArray(extrusion) ? extrusion[0] : extrusion.pos;
                const length = Array.isArray(extrusion) ? extrusion[1] : extrusion.length;
                const color = this._getExtrusionColor(extrusion);
                const radius = Math.sqrt(length / Math.PI) * 0.01 * 0.5;

                this.extrusionCtx.fillStyle = this._colorWithAlpha(color, 0.5);
                this.extrusionCtx.beginPath();
                this.extrusionCtx.arc(
                    this.cX(pos[0]),
                    this.cY(pos[1]),
                    radius * this.effectiveCScale,
                    0, 2 * Math.PI
                );
                this.extrusionCtx.fill();
            }
            if (extruderComp.extrusions.length > maxExtrusions) {
                maxExtrusions = extruderComp.extrusions.length;
            }
        }
        this.drawnExtrusionCount = maxExtrusions;
    }


    // Render All Renderable Entities (Circles/Obstacles/Etc.) considering rotation
    const renderableEntities = world.query([PositionComponent, RenderableComponent]);
    const wobbleNowSec = this._nowSeconds();
    for (const entityId of renderableEntities) {
        const posComp = world.getComponent(entityId, PositionComponent);
        const renderComp = world.getComponent(entityId, RenderableComponent);
        const orientationComp = world.getComponent(entityId, OrientationComponent); // Get orientation

        if (renderComp.shape === 'circle') {
            const radiusComp = world.getComponent(entityId, RadiusComponent);
            if (posComp && radiusComp) {
                let simX = posComp.pos.x;
                let simY = posComp.pos.y;
                if (world.hasComponent(entityId, ObstacleTagComponent)) {
                  const wobbleOffset = this._getBumperWobbleOffset(entityId, wobbleNowSec);
                  if (wobbleOffset) {
                    simX += wobbleOffset.x;
                    simY += wobbleOffset.y;
                  }
                }
                const simRadius = radiusComp.radius;
                const angle = orientationComp ? orientationComp.angle : 0.0; // Use 0 if no orientation

                const cx = this.cX(simX);
                const cy = this.cY(simY);
                const cr = simRadius * this.effectiveCScale;

                this.c.save(); // Save context state
                this.c.translate(cx, cy); // Move origin to circle center
                this.c.rotate(-angle); // Rotate (negative for canvas Y-down)

                // Draw the circle centered at the new (0,0)
                this.c.fillStyle = renderComp.color;
                this.c.beginPath();
                this.c.arc(0, 0, cr, 0, 2 * Math.PI);
                this.c.closePath();
                this.c.fill();

                // Draw a line indicating orientation (e.g., from center to top edge)
                if (orientationComp) { // Only draw if it can rotate
                    this.c.strokeStyle = '#000000'; // Black line
                    this.c.lineWidth = 1 * this.effectiveCScale/250;
                    this.c.beginPath();
                    this.c.moveTo(0, 0); // Center
                    this.c.lineTo(0, -cr); // To top edge in local rotated coords
                    this.c.stroke();
                }

                this.c.restore(); // Restore context state
            }
        }
        // Add rendering for other shapes if needed
    }

    //// Render Angular Velocity for Obstacles
    //const obstacleEntitiesWithAngularVelocity = world.query([ObstacleTagComponent, AngularVelocityComponent, PositionComponent]);
    //if (obstacleEntitiesWithAngularVelocity.length > 0) {
    //    this.c.save();
    //    this.c.fillStyle = '#FFFFFF'; // White text
    //    this.c.strokeStyle = '#000000'; // Black outline for better visibility
    //    this.c.lineWidth = 2.5;
    //    this.c.font = `${12 * this.viewScaleMultiplier}px Arial`; // Scale font with zoom
    //    this.c.textAlign = 'center';
    //    this.c.textBaseline = 'bottom';
    //    for (const entityId of obstacleEntitiesWithAngularVelocity) {
    //        const posComp = world.getComponent(entityId, PositionComponent);
    //        const angularVelComp = world.getComponent(entityId, AngularVelocityComponent);
    //        if (posComp && angularVelComp) {
    //            const text = angularVelComp.angularVelocity.toFixed(1);
    //            const x = this.cX(posComp.pos.x);
    //            const y = this.cY(posComp.pos.y-world.getComponent(entityId, RadiusComponent).radius); // Offset above the center, scaled
    //            // Draw outline then text
    //            this.c.strokeText(text, x, y);
    //            this.c.fillText(text, x, y);
    //        }
    //    }
    //    this.c.restore();
    //}

    // Draw special markers for hybrid links before overlays so reference paths render above them.
    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);

      // Draw markers for hybrid links
      for (let i = 0; i < path.linkTypes.length; i++) {
        if (path.linkTypes[i] === 'hybrid' || path.linkTypes[i] === 'hybrid-attachment') {
          // Find the entity for this link
          let entityId = null;
          if (i === 0) {
            // First link is entityA of first joint
            const jointId = path.jointEntities[0];
            const joint = world.getComponent(jointId, CableJointComponent);
            if (joint) entityId = joint.entityA;
          } else if (i === path.linkTypes.length - 1) {
            // Last link is entityB of last joint
            const jointId = path.jointEntities[path.jointEntities.length - 1];
            const joint = world.getComponent(jointId, CableJointComponent);
            if (joint) entityId = joint.entityB;
          } else {
            // Middle links are entityB of previous joint (or entityA of next joint)
            const jointId = path.jointEntities[i - 1];
            const joint = world.getComponent(jointId, CableJointComponent);
            if (joint) entityId = joint.entityB;
          }

          if (entityId !== null) {
            const posComp = world.getComponent(entityId, PositionComponent);
            const radiusComp = world.getComponent(entityId, RadiusComponent);

            if (posComp && radiusComp) {
              // Draw a small marker on the edge of the entity to indicate hybrid link
              // Determine the correct attachment point for the marker
              let attachmentPoint = null;
              if (i === 0) {
                const jointId = path.jointEntities[0];
                const joint = world.getComponent(jointId, CableJointComponent);
                if (joint) attachmentPoint = joint.attachmentPointA_world;
              } else if (i === path.linkTypes.length - 1) {
                const jointId = path.jointEntities[path.jointEntities.length - 1];
                const joint = world.getComponent(jointId, CableJointComponent);
                if (joint) attachmentPoint = joint.attachmentPointB_world;
              }
              // Note: Middle hybrid links are not typically expected.

              const markerRadius = 1.5 * this.effectiveCScale/250;

              if (path.linkTypes[i] === 'hybrid-attachment') {
                // Draw RED dot at the fixed attachment point
                if (attachmentPoint) {
                  this.c.beginPath();
                  this.c.fillStyle = '#FF0000'; // Red for attachment
                  const markerX = this.cX(attachmentPoint.x);
                  const markerY = this.cY(attachmentPoint.y);
                  this.c.arc(markerX, markerY, markerRadius, 0, 1.5 * Math.PI);
                  this.c.fill();

                  // Show stored length (should be 0)
                  //if (debugPoints) {
                  //  this.c.fillStyle = '#FFFFFF';
                  //  this.c.font = '10px Arial';
                  //  this.c.textAlign = 'left';
                  //  this.c.textBaseline = 'middle';
                  //  this.c.fillText(path.stored[i].toFixed(2), markerX + 7, markerY);
                  //}
                }
              } else if (path.linkTypes[i] === 'hybrid') {
                // Draw yellow dot at the current TANGENT point
                let tangentPoint = null;
                if (i === 0) {
                  const joint = world.getComponent(path.jointEntities[0], CableJointComponent);
                  if (joint) tangentPoint = joint.attachmentPointA_world;
                } else if (i === path.linkTypes.length - 1) {
                  const joint = world.getComponent(path.jointEntities[path.jointEntities.length - 1], CableJointComponent);
                  if (joint) tangentPoint = joint.attachmentPointB_world;
                }

                if (tangentPoint) {
                  // Calculate and draw GREEN dot at the end of the stored arc
                  const center = posComp.pos;
                  const halfWidth = layeringEnabled ? (path.cableHalfWidth ?? 0.0) : 0.0;
                  const baseRadius = radiusComp.radius + halfWidth;
                  const storedLength = Math.max(0.0, path.stored[i] ?? 0.0);
                  const cw = path.cw[i];

                  if (baseRadius > 1e-9) {
                    const toTangent = tangentPoint.clone().subtract(center);
                    const tangentAngle = Math.atan2(toTangent.y, toTangent.x);
                    let attachmentAngle = tangentAngle;
                    if (layeringEnabled) {
                      let layerRadius = baseRadius;
                      let partialLength = storedLength;
                      const fullWidth = 2.0 * halfWidth;
                      const MAX_LAYERS = 128;
                      let layerCount = 0;
                      while (
                        fullWidth > 1e-12 &&
                        layerCount < MAX_LAYERS
                      ) {
                        const layerCircumference = 2.0 * Math.PI * layerRadius;
                        if (!(partialLength > layerCircumference + 1e-9)) {
                          break;
                        }
                        partialLength -= layerCircumference;
                        layerCount += 1;
                        layerRadius = baseRadius + fullWidth * layerCount;
                      }
                      const deltaAngle = (partialLength > 1e-9 && layerRadius > 1e-9)
                        ? (partialLength / layerRadius)
                        : 0.0;
                      attachmentAngle = cw ? tangentAngle - deltaAngle : tangentAngle + deltaAngle;
                    } else {
                      const deltaAngle = storedLength / baseRadius;
                      attachmentAngle = cw ? tangentAngle - deltaAngle : tangentAngle + deltaAngle;
                    }

                    const endOfArcPoint = new Vector2(
                      center.x + baseRadius * Math.cos(attachmentAngle),
                      center.y + baseRadius * Math.sin(attachmentAngle)
                    );

                    this.c.beginPath();
                    this.c.fillStyle = '#FF0000';
                    const endMarkerX = this.cX(endOfArcPoint.x);
                    const endMarkerY = this.cY(endOfArcPoint.y);
                    this.c.arc(endMarkerX, endMarkerY, markerRadius, 0, 2 * Math.PI);
                    this.c.fill();
                  }
                }
              }
            }
          }
        }
      }
    }

    if (this.positionTraceEnabled) {
      this._drawPositionTraceMarkers();
    }

    if (this.referenceDirty) {
      this._redrawReferencePaths();
    }
    if (this.referenceVisible && this.referenceCanvas) {
      this.c.drawImage(this.referenceCanvas, 0, 0);
    }
    if (this.extrusionCanvas) {
      this.c.drawImage(this.extrusionCanvas, 0, 0);
    }

    // Render Debug Points
    const debugPoints = world.getResource('debugRenderPoints');

    if (debugPoints) {
        // Keep debug points a constant pixel size
        const scaledDebugRadius = baseDebugRadius;
        this.c.save();
        for (const key in debugPoints) {
            const pointData = debugPoints[key];
            if (pointData && pointData.pos) {
                this.c.fillStyle = pointData.color;
                this.c.beginPath();
                // Use transformed coordinates using instance methods
                this.c.arc(
                    this.cX(pointData.pos.x),
                    this.cY(pointData.pos.y),
                    scaledDebugRadius,
                    0, 2 * Math.PI
                );
                this.c.fill();
            }
        }
        this.c.restore();
    }

    // Re-draw collision envelopes last so they stay visible above all other layers.
    const collisionOverlayRenderingEnabled =
      world.getResource('layeringRenderCollisionOverlays') !== false;
    {
      const overlayEntities = world.query([PositionComponent, OverlayRadiusComponent]);
      const sectorEntityIds = new Set([
        ...world.query([PositionComponent, CircleSectorComponent]),
        ...world.query([PositionComponent, CircleSectorsComponent]),
      ]);
      const ballBallContacts = Array.isArray(world.getResource('ball_ball_contacts'))
        ? world.getResource('ball_ball_contacts')
        : [];
      const obstacleContacts = Array.isArray(world.getResource('ball_obstacle_contacts'))
        ? world.getResource('ball_obstacle_contacts')
        : [];

      if (
        collisionOverlayRenderingEnabled && (
          overlayEntities.length > 0 ||
          sectorEntityIds.size > 0 ||
          ballBallContacts.length > 0 ||
          obstacleContacts.length > 0
        )
      ) {
        this.c.save();
        const canSetLineDash = typeof this.c.setLineDash === 'function';

        if (canSetLineDash) {
          this.c.setLineDash([4, 3]);
        }
        const overlayStrokeWidthPx = Math.max(1.0, 1.25 * this.effectiveCScale / 250);
        const sectorStrokeWidthPx = Math.max(1.0, 1.25 * this.effectiveCScale / 250);
        this.c.strokeStyle = 'rgba(0, 220, 255, 0.98)';
        for (const entityId of overlayEntities) {
          const pos = world.getComponent(entityId, PositionComponent)?.pos;
          const overlayRadius = world.getComponent(entityId, OverlayRadiusComponent)?.radius;
          if (!pos || !Number.isFinite(overlayRadius) || !(overlayRadius > 1e-9)) {
            continue;
          }
          this.c.lineWidth = overlayStrokeWidthPx;
          this.c.beginPath();
          this.c.arc(
            this.cX(pos.x),
            this.cY(pos.y),
            overlayRadius * this.effectiveCScale,
            0.0,
            2.0 * Math.PI
          );
          this.c.stroke();
        }

        if (canSetLineDash) {
          this.c.setLineDash([]);
        }
        for (const entityId of sectorEntityIds) {
          const pos = world.getComponent(entityId, PositionComponent)?.pos;
          if (!pos) {
            continue;
          }
          const multi = world.getComponent(entityId, CircleSectorsComponent);
          const single = world.getComponent(entityId, CircleSectorComponent);
          const sectors = (
            multi && Array.isArray(multi.sectors) && multi.sectors.length > 0
              ? multi.sectors
              : (single ? [single] : [])
          );
          for (const sector of sectors) {
            if (!sector || !Number.isFinite(sector.radius) || !(sector.radius > 1e-9)) {
              continue;
            }
            const startAngle = Number.isFinite(sector.startAngle) ? sector.startAngle : 0.0;
            const endAngle = Number.isFinite(sector.endAngle) ? sector.endAngle : 0.0;
            const anticlockwise = sector.cw !== true;

            this.c.fillStyle = 'rgba(255, 136, 0, 0.18)';
            this.c.beginPath();
            this.c.moveTo(this.cX(pos.x), this.cY(pos.y));
            this.c.arc(
              this.cX(pos.x),
              this.cY(pos.y),
              sector.radius * this.effectiveCScale,
              -startAngle,
              -endAngle,
              anticlockwise
            );
            this.c.closePath();
            this.c.fill();

            this.c.strokeStyle = 'rgba(255, 136, 0, 0.95)';
            this.c.lineWidth = sectorStrokeWidthPx;
            this.c.beginPath();
            this.c.arc(
              this.cX(pos.x),
              this.cY(pos.y),
              sector.radius * this.effectiveCScale,
              -startAngle,
              -endAngle,
              anticlockwise
            );
            this.c.stroke();
          }
        }

        // Show pairwise manifold envelopes used by pinch-share contacts.
        if (canSetLineDash) {
          this.c.setLineDash([3, 2]);
        }
        this.c.lineWidth = Math.max(1.0, 1.75 * this.effectiveCScale / 250);
        this.c.strokeStyle = 'rgba(255, 40, 180, 0.95)';

        for (const contact of ballBallContacts) {
          if (!contact || contact.pinch_shared !== true) {
            continue;
          }
          const aPos = world.getComponent(contact.ball_a, PositionComponent)?.pos;
          const bPos = world.getComponent(contact.ball_b, PositionComponent)?.pos;
          if (aPos && Number.isFinite(contact.radius_a) && contact.radius_a > 1e-9) {
            this.c.beginPath();
            this.c.arc(
              this.cX(aPos.x),
              this.cY(aPos.y),
              contact.radius_a * this.effectiveCScale,
              0.0,
              2.0 * Math.PI
            );
            this.c.stroke();
          }
          if (bPos && Number.isFinite(contact.radius_b) && contact.radius_b > 1e-9) {
            this.c.beginPath();
            this.c.arc(
              this.cX(bPos.x),
              this.cY(bPos.y),
              contact.radius_b * this.effectiveCScale,
              0.0,
              2.0 * Math.PI
            );
            this.c.stroke();
          }
        }

        for (const contact of obstacleContacts) {
          if (!contact || contact.pinch_shared !== true) {
            continue;
          }
          const ballPos = world.getComponent(contact.ball_id, PositionComponent)?.pos;
          const obsPos = world.getComponent(contact.obs_id, PositionComponent)?.pos;
          if (
            ballPos &&
            Number.isFinite(contact.ball_contact_radius) &&
            contact.ball_contact_radius > 1e-9
          ) {
            this.c.beginPath();
            this.c.arc(
              this.cX(ballPos.x),
              this.cY(ballPos.y),
              contact.ball_contact_radius * this.effectiveCScale,
              0.0,
              2.0 * Math.PI
            );
            this.c.stroke();
          }
          if (
            obsPos &&
            Number.isFinite(contact.obstacle_contact_radius) &&
            contact.obstacle_contact_radius > 1e-9
          ) {
            this.c.beginPath();
            this.c.arc(
              this.cX(obsPos.x),
              this.cY(obsPos.y),
              contact.obstacle_contact_radius * this.effectiveCScale,
              0.0,
              2.0 * Math.PI
            );
            this.c.stroke();
          }
        }

        if (canSetLineDash) {
          this.c.setLineDash([]);
        }
        this.c.restore();
      }
    }

    this._updateAndRenderBumperHitFx(world);
  }

  _getExtrusionColor(extrusion) {
    if (extrusion && typeof extrusion === 'object' && !Array.isArray(extrusion)) {
      if (typeof extrusion.qualityColor === 'string' && extrusion.qualityColor.length > 0) {
        return extrusion.qualityColor;
      }
      if (typeof extrusion.color === 'string' && extrusion.color.length > 0) {
        return extrusion.color;
      }
    }
    return null;
  }

  _colorWithAlpha(color, alpha = 1) {
    if (typeof color !== 'string' || color.length === 0) {
      return `rgba(100, 255, 100, ${alpha})`;
    }
    if (color.startsWith('rgba')) {
      const parts = color.match(/\d+(?:\.\d+)?/g);
      if (parts && parts.length >= 3) {
        const [r, g, b] = parts;
        return `rgba(${r}, ${g}, ${b}, ${alpha})`;
      }
    }
    if (color.startsWith('rgb(')) {
      const parts = color.match(/\d+(?:\.\d+)?/g);
      if (parts && parts.length >= 3) {
        const [r, g, b] = parts;
        return `rgba(${r}, ${g}, ${b}, ${alpha})`;
      }
    }
    if (color.startsWith('#')) {
      let hex = color.slice(1);
      if (hex.length === 3) {
        hex = hex.split('').map((c) => c + c).join('');
      }
      if (hex.length === 6) {
        const r = parseInt(hex.slice(0, 2), 16);
        const g = parseInt(hex.slice(2, 4), 16);
        const b = parseInt(hex.slice(4, 6), 16);
        return `rgba(${r}, ${g}, ${b}, ${alpha})`;
      }
    }
    return color;
  }
}
