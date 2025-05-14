import Vector2 from './vector2.js';

import {
  PositionComponent,
  RadiusComponent,
  RenderableComponent,
  OrientationComponent,
  BorderComponent,
  FlipperTagComponent,
  FlipperStateComponent
} from './ecs.js';

import {
  CableLinkComponent,
  CableJointComponent,
  CablePathComponent
} from './cable_joints_core.js';

export class RenderSystem {
  runInPause = true; // Always render
  constructor(canvas, cScale, simHeight, viewScaleMultiplier = 1.0, viewOffsetX_sim = 0.0, viewOffsetY_sim = 0.0) {
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
  _drawCatenary(jointId, pA, pB, length, obstacles, sagDir = new Vector2(0, -1), segments = 20) {
      const ctx = this.c;
      const dx = pB.x - pA.x, dy = pB.y - pA.y;
      const D  = Math.hypot(dx, dy);
      if (D <= 1e-6) return;

      // parabolic sag magnitude
      const sag = Math.max(length - D, 0) * 0.1;
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


  update(world, dt) {
    // Viewport settings are now instance properties (this.viewScaleMultiplier, etc.)
    // effectiveCScale is also an instance property (this.effectiveCScale)

    this.c.clearRect(0, 0, this.canvas.width, this.canvas.height);

    // --- Query for Cable Link Obstacles ---
    this.cableLinkObstacles = [];
    const linkEntities = world.query([CableLinkComponent, PositionComponent, RadiusComponent]);
    for (const entityId of linkEntities) {
        const posComp = world.getComponent(entityId, PositionComponent);
        const radiusComp = world.getComponent(entityId, RadiusComponent);
        if (posComp && radiusComp) {
            this.cableLinkObstacles.push({
                pos: posComp.pos.clone(), // Use current position
                radius: radiusComp.radius
            });
        }
    }
    // --- End Obstacle Query ---


    // Base line width and debug radius (adjust as needed)
    const baseLineWidth = 2;
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
      const jointEntities = path.jointEntities;
      // Scale line width by zoom using instance property
      this.c.lineWidth = baseLineWidth * this.viewScaleMultiplier;
      for (const entityId of jointEntities) {
        const jointComp = world.getComponent(entityId, CableJointComponent);
        const renderComp = world.getComponent(entityId, RenderableComponent);

        if (renderComp.shape !== 'line') continue;

        const pA = jointComp.attachmentPointA_world;
        const pB = jointComp.attachmentPointB_world;
        this.c.strokeStyle = renderComp.color;
        this.c.beginPath();
        // Draw catenary if slack, otherwise straight
        const straightDist = pA.distanceTo(pB);
        if (jointComp.restLength > straightDist + 1e-6) {
          this._drawCatenary(entityId, pA, pB, jointComp.restLength, this.cableLinkObstacles);
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
        this.c.strokeStyle = renderComp.color;               // or pick your cable colour
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
          this.c.strokeStyle = renderComp.color;
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
          this.c.strokeStyle = renderComp.color;
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
                    this.c.lineWidth = 1 * this.viewScaleMultiplier;
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


    // Render Debug Points
    const debugPoints = world.getResource('debugRenderPoints');

    // Draw special markers for hybrid links AFTER debugPoints is defined
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

              const markerRadius = 4;

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
}
