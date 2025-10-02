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
    const baseWidthPx = Math.max(0.75/2, (0.5 * this.effectiveCScale) / 250);
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
      const color = Array.isArray(extrusion) ? null : extrusion.color;

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
  }


  update(world, dt) {
    if (this.drawingSuspended) {
      return;
    }
    // Viewport settings are now instance properties (this.viewScaleMultiplier, etc.)
    // effectiveCScale is also an instance property (this.effectiveCScale)

    this.c.clearRect(0, 0, this.canvas.width, this.canvas.height);

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
      // Scale line width by zoom using instance property
      this.c.lineWidth = baseLineWidth * this.effectiveCScale/250;
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

    for (const pathId of pathEntities) {
      const path   = world.getComponent(pathId, CablePathComponent);
      if (path.jointEntities.length < 1) continue;
      const joints = path.jointEntities;
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
        const R    = radiusComp.radius;
        const P1   = jPrev.attachmentPointB_world;
        const P2   = jNext.attachmentPointA_world;
        // angles in sim coords
        const a1 = Math.atan2(P1.y - C.y, P1.x - C.x);
        const a2 = Math.atan2(P2.y - C.y, P2.x - C.x);
        // cw‐flag stored in path.cw[i]
        const anticlockwise = !path.cw[i];
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
          const a1     = Math.atan2(P0.y - cA.y, P0.x - cA.x);
          const s      = path.stored[0];
          const max_renderable_ang = 2.0*Math.PI - 0.0001
          let delta_theta     = s / rA;
          if (delta_theta >= max_renderable_ang) {
            delta_theta = max_renderable_ang;
          }
          const cw0    = path.cw[0];
          const anticw = !cw0;
          const a2     = cw0 ? a1 - delta_theta : a1 + delta_theta;
          this.c.beginPath();
          if (renderComp) {
            this.c.strokeStyle = renderComp.color;
          } else {
            this.c.strokeStyle = 'yellow';
          }
          this.c.arc(
            this.cX(cA.x), this.cY(cA.y),
            rA * this.effectiveCScale,
            -a1, -a2, anticw
          );
          this.c.stroke();
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
          const a1 = Math.atan2(P1.y - cB.y, P1.x - cB.x);
          const s = path.stored[nLinks - 1];
          const max_renderable_ang = 2.0*Math.PI - 0.0001
          let delta_theta = s / rB;
          if (delta_theta >= max_renderable_ang) {
            delta_theta = max_renderable_ang;
          }
          const cw1 = path.cw[nLinks - 1];
          const anticw = !cw1;

          // Calculate a2 based on the render_sweep_angle
          const a2 = cw1 ? a1 - delta_theta : a1 + delta_theta;

          this.c.beginPath();
          if (renderComp) {
            this.c.strokeStyle = renderComp.color;
          } else {
            this.c.strokeStyle = 'yellow';
          }
          this.c.arc(
            this.cX(cB.x), this.cY(cB.y),
            rB * this.effectiveCScale,
            -a1, -a2, anticw
          );
          this.c.stroke();
        }
      }
    }
    this.c.lineWidth = 1;

    // Render Rigid Group visual edges (green lines between sequential members)
    const rigidGroups = world.query([RigidGroupComponent]);
    if (rigidGroups.length > 0) {
      this.c.save();
      this.c.lineWidth = 1 * this.effectiveCScale/250;
      this.c.strokeStyle = 'green';
      for (const gid of rigidGroups) {
        const group = world.getComponent(gid, RigidGroupComponent);
        const members = group?.members || [];
        const n = members.length | 0;
        if (n === 9) {
          for (const i of [[3, 4], [4, 5], [5, 6], [6, 7], [7, 8], [8, 3]]) {
            const a = members[i[0]];
            const b = members[i[1] % n];
            const pA = world.getComponent(a, PositionComponent)?.pos;
            const pB = world.getComponent(b, PositionComponent)?.pos;
            if (!pA || !pB) continue;
            this.c.beginPath();
            this.c.moveTo(this.cX(pA.x), this.cY(pA.y));
            this.c.lineTo(this.cX(pB.x), this.cY(pB.y));
            this.c.stroke();
          }
        }
        if (n === 6) {
          for (const i of [[3, 4], [4, 5], [5, 3]]) {
            const a = members[i[0]];
            const b = members[i[1] % n];
            const pA = world.getComponent(a, PositionComponent)?.pos;
            const pB = world.getComponent(b, PositionComponent)?.pos;
            if (!pA || !pB) continue;
            this.c.beginPath();
            this.c.moveTo(this.cX(pA.x), this.cY(pA.y));
            this.c.lineTo(this.cX(pB.x), this.cY(pB.y));
            this.c.stroke();
          }
        }
        if (n === 3) {
          for (const i of [[0, 1], [1, 2], [2, 0]]) {
            const a = members[i[0]];
            const b = members[i[1] % n];
            const pA = world.getComponent(a, PositionComponent)?.pos;
            const pB = world.getComponent(b, PositionComponent)?.pos;
            if (!pA || !pB) continue;
            this.c.beginPath();
            this.c.moveTo(this.cX(pA.x), this.cY(pA.y));
            this.c.lineTo(this.cX(pB.x), this.cY(pB.y));
            this.c.stroke();
          }
        }
        //for (const i of [[0, 1], [1, 2], [2, 0]]) {
        //  const a = members[i[0]];
        //  const b = members[i[1] % n];
        //  const pA = world.getComponent(a, PositionComponent)?.pos;
        //  const pB = world.getComponent(b, PositionComponent)?.pos;
        //  if (!pA || !pB) continue;
        //  this.c.beginPath();
        //  this.c.moveTo(this.cX(pA.x), this.cY(pA.y));
        //  this.c.lineTo(this.cX(pB.x), this.cY(pB.y));
        //  this.c.stroke();
        //}
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
                const color = Array.isArray(extrusion) ? null : extrusion.color;
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
    for (const entityId of renderableEntities) {
        const posComp = world.getComponent(entityId, PositionComponent);
        const renderComp = world.getComponent(entityId, RenderableComponent);
        const orientationComp = world.getComponent(entityId, OrientationComponent); // Get orientation

        if (renderComp.shape === 'circle') {
            const radiusComp = world.getComponent(entityId, RadiusComponent);
            if (posComp && radiusComp) {
                const simX = posComp.pos.x;
                const simY = posComp.pos.y;
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
                  const radius = radiusComp.radius;
                  const storedLength = path.stored[i];
                  const cw = path.cw[i];

                  if (radius > 1e-9) {
                    const toTangent = tangentPoint.clone().subtract(center);
                    const tangentAngle = Math.atan2(toTangent.y, toTangent.x);
                    const deltaAngle = storedLength / radius;
                    const endAngle = cw ? tangentAngle - deltaAngle : tangentAngle + deltaAngle;

                    const endOfArcPoint = new Vector2(
                      center.x + radius * Math.cos(endAngle),
                      center.y + radius * Math.sin(endAngle)
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
