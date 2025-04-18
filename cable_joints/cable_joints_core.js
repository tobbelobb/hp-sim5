const linecolor1 = '#FFFF00'
// --- Utility: Vector2 ---
class Vector2 {
  constructor(x = 0.0, y = 0.0) { this.x = x; this.y = y; }
  set(v) { this.x = v.x; this.y = v.y; }
  clone() { return new Vector2(this.x, this.y); }
  add(v, s = 1.0) { this.x += v.x * s; this.y += v.y * s; return this; }
  addVectors(a, b) { this.x = a.x + b.x; this.y = a.y + b.y; return this; }
  subtract(v, s = 1.0) { this.x -= v.x * s; this.y -= v.y * s; return this; }
  subtractVectors(a, b) { this.x = a.x - b.x; this.y = a.y - b.y; return this; }
  distanceTo(b) { return Math.sqrt((this.x - b.x)*(this.x - b.x) + (this.y - b.y)*(this.y - b.y)); }
  length() { return Math.sqrt(this.x * this.x + this.y * this.y); }
  lengthSq() { return this.x * this.x + this.y * this.y; }
  scale(s) { this.x *= s; this.y *= s; return this; }
  dot(v) { return this.x * v.x + this.y * v.y; }
  perp() { return new Vector2(-this.y, this.x); }
  normalize() { const l = this.length(); if (l > 0) this.scale(1.0 / l); return this; }
}

// --- Utility: Geometry ---
function closestPointOnSegment(p, a, b) {
  const ab = new Vector2().subtractVectors(b, a);
  const ap = new Vector2().subtractVectors(p, a);
  let t = ap.dot(ab);
  if (t <= 0.0) return a.clone();
  const denom = ab.dot(ab);
  if (t >= denom) return b.clone();
  t = t / denom;
  return a.clone().add(ab, t);
}

// Helper: Calculate tangent points between a point and a circle (Algorithm 3 from Cable Joints paper)
function _tangentPointCircle(p_attach, p_circle, r_circle, cw, pointIsFirst) {
  const dVec = new Vector2().subtractVectors(p_circle, p_attach);
  const dSq = dVec.lengthSq();

  if (dSq <= r_circle * r_circle + 1e-9) {
    console.warn("Cable tangent calculation: Attachment point inside or on rolling circle.");
    const dir = dVec.lengthSq() > 1e-9 ? dVec.clone().normalize() : new Vector2(1, 0);
    return {
      a_attach: p_attach.clone(),
      a_circle: p_circle.clone().add(dir, r_circle)
    };
  }

  const d = Math.sqrt(dSq);
  const alpha = Math.atan2(dVec.y, dVec.x);
  const phi = Math.asin(r_circle / d);

  let tangent_point_angle_on_circle;
  if ((cw && pointIsFirst) || (!cw && !pointIsFirst)) {
    tangent_point_angle_on_circle = alpha + phi + Math.PI/2;
  } else {
    tangent_point_angle_on_circle = alpha - phi - Math.PI/2;
  }

  const a_circle = new Vector2(
    p_circle.x + r_circle * Math.cos(tangent_point_angle_on_circle),
    p_circle.y + r_circle * Math.sin(tangent_point_angle_on_circle)
  );

  return {
    a_attach: p_attach.clone(), // Attachment point doesn't change
    a_circle: a_circle          // Calculated tangent point on the circle
  };
}

function tangentFromPointToCircle(p_attach, p_circle, r_circle, cw) {
  return _tangentPointCircle(p_attach, p_circle, r_circle, cw, true);
}

function tangentFromCircleToPoint(p_attach, p_circle, r_circle, cw) {
  return _tangentPointCircle(p_attach, p_circle, r_circle, cw, false);
}

function tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB) {
  let dVec = new Vector2().subtractVectors(posB, posA);
  let d = dVec.length();

  let r = (cwA === cwB) ? (radiusB - radiusA) : (radiusA + radiusB);
  if (d <= Math.abs(r)) {
    console.warn("Circles too close. Returning closest contact point as both \"tangent points\".");

    // Direction to push along
    const pushDir = dVec.length() === 0
      ? new Vector2(1, 0) // fallback direction if centers coincide
      : dVec.clone().normalize();

    // Desired distance: just touching
    const requiredDist = Math.abs(r);
    const midPoint = posA.clone().add(posB).scale(0.5);

    const offset = pushDir.scale(requiredDist / 2.0);
    const newA = midPoint.clone().add(offset.clone().scale(-1.0));
    const newB = midPoint.clone().add(offset);

    // Contact point is on the line between the centers, offset from either by their radius
    const contactPoint = newA.clone().add(pushDir.clone().scale(radiusA));

    return {
      a_circle: contactPoint.clone(),
      b_circle: contactPoint.clone()
    };
  }

  // Normal tangent calculation
  r = (cwA === cwB) ? (radiusB - radiusA) : (radiusA + radiusB);
  const alpha = Math.atan2(dVec.y, dVec.x);
  const phi = Math.asin(Math.max(-1, Math.min(1, r / d)));

  let angleA, angleB;
  if (!cwA === cwB) {
    if (!cwA) {
      // console.log("FT");
      angleA = alpha - Math.PI / 2 + phi;
      angleB = alpha + Math.PI / 2 + phi;
    } else {
      // console.log("TF");
      angleA = alpha + Math.PI / 2 - phi;
      angleB = alpha - Math.PI / 2 - phi;
    }
  } else {
    if (!cwA) {
      // console.log("FF");
      angleA = alpha - Math.PI / 2 - phi;
      angleB = alpha - Math.PI / 2 - phi;
    } else {
      // console.log("TT");
      angleA = alpha + Math.PI / 2 + phi;
      angleB = alpha + Math.PI / 2 + phi;
    }
  }

  const tangentA = new Vector2(
    posA.x + radiusA * Math.cos(angleA),
    posA.y + radiusA * Math.sin(angleA)
  );
  const tangentB = new Vector2(
    posB.x + radiusB * Math.cos(angleB),
    posB.y + radiusB * Math.sin(angleB)
  );

  return {
    a_circle: tangentA,
    b_circle: tangentB
  };
}


/**
 * Calculates signed arc length between two world-space points on the circumference of a wheel.
 *
 * @param {Vector2} prevPoint - Previous attachment point on the wheel (world-space)
 * @param {Vector2} currPoint - Current attachment point on the wheel (world-space)
 * @param {Vector2} center - Center of the wheel (world-space)
 * @param {number} radius - Radius of the wheel
 * @param {boolean} clockwisePreference - If true, positive arc length means CW, else CCW
 * @returns {number} Signed arc length (positive = preferred direction)
 */
function signedArcLengthOnWheel(prevPoint, currPoint, center, radius, clockwisePreference, force_positive = false) {
  const toPrev = new Vector2().subtractVectors(prevPoint, center);
  const toCurr = new Vector2().subtractVectors(currPoint, center);

  // Get angle between the two vectors (signed)
  let angle = Math.atan2(toCurr.y, toCurr.x) - Math.atan2(toPrev.y, toPrev.x);

  // Normalize angle to range [-π, π]
  if (angle > Math.PI) angle -= 2 * Math.PI;
  if (angle < -Math.PI) angle += 2 * Math.PI;

  if (clockwisePreference) {
    angle *= -1;
  }

  if (force_positive) {
    while (angle < 0.0) {
      angle += 2 * Math.PI;
    }
  }
  // Arc length = radius * angle (signed)
  return radius * angle;
}

/**
 * Checks if a line segment intersects a circle.
 * @param {Vector2} p1 - Start point of the segment
 * @param {Vector2} p2 - End point of the segment
 * @param {Vector2} center - Center of the circle
 * @param {number} radius - Radius of the circle
 * @returns {boolean} True if the segment intersects the circle, false otherwise.
 */
function lineSegmentCircleIntersection(p1, p2, center, radius) {
  // 1. Check if either endpoint is inside the circle
  if (p1.distanceTo(center) <= radius || p2.distanceTo(center) <= radius) {
    return true;
  }

  // 2. Check if the projection of the center onto the line lies within the segment
  const d = new Vector2().subtractVectors(p2, p1);
  const lc = new Vector2().subtractVectors(center, p1);
  const dLengthSq = d.lengthSq();

  // Project center onto the line containing the segment
  let t = lc.dot(d);
  if (dLengthSq > 1e-9) { // Avoid division by zero for zero-length segment
      t /= dLengthSq;
  }

  if (t < 0.0) {
    return false; // Closest point is p1
  } else if (t > 1.0) {
    return false; // Closest point is p2
  }

  const closestPointOnLine = p1.clone().add(d, t);

  // 3. Check if the closest point on the segment is within the circle's radius
  return closestPointOnLine.distanceTo(center) <= radius;
}

/**
 * Determines if point x is to the left of the directed line segment from p0 to p1.
 * Uses the 2D cross product.
 * @param {Vector2} x - The point to test.
 * @param {Vector2} p0 - The start point of the line segment.
 * @param {Vector2} p1 - The end point of the line segment.
 * @returns {boolean} True if x is strictly to the left, false otherwise (right or collinear).
 */
function rightOfLine(x, p0, p1) {
  const v_x = p1.x - p0.x;
  const v_y = p1.y - p0.y;
  const w_x = x.x - p0.x;
  const w_y = x.y - p0.y;
  // Calculate the 2D cross product (determinant)
  const crossProduct = v_x * w_y - v_y * w_x;
  // Return true if the cross product is positive (indicating left)
  return crossProduct < 0.0;
}


// --- ECS Core ---
class World {
  constructor() {
    this.entities = new Map(); // entityId -> Set<ComponentClass>
    this.components = new Map(); // ComponentClass -> Map<entityId, ComponentInstance>
    this.nextEntityId = 0;
    this.systems = [];
    this.resources = {}; // Global data like gravity, dt
  }

  createEntity() {
    const id = this.nextEntityId++;
    this.entities.set(id, new Set());
    return id;
  }

  addComponent(entityId, component) {
    const componentClass = component.constructor;
    if (!this.components.has(componentClass)) {
      this.components.set(componentClass, new Map());
    }
    this.components.get(componentClass).set(entityId, component);
    if (this.entities.has(entityId)) {
      this.entities.get(entityId).add(componentClass);
    } else {
      console.warn(`Entity ${entityId} does not exist when adding ${componentClass.name}`);
    }
  }

  getComponent(entityId, componentClass) {
    const componentMap = this.components.get(componentClass);
    return componentMap ? componentMap.get(entityId) : undefined;
  }

  hasComponent(entityId, componentClass) {
    const componentMap = this.components.get(componentClass);
    return componentMap ? componentMap.has(entityId) : false;
  }

  removeComponent(entityId, componentClass) {
    const componentMap = this.components.get(componentClass);
    if (componentMap) {
      componentMap.delete(entityId);
    }
    const entityComponents = this.entities.get(entityId);
    if (entityComponents) {
      entityComponents.delete(componentClass);
    }
  }

  destroyEntity(entityId) {
    if (!this.entities.has(entityId)) return;
    for (const componentClass of this.entities.get(entityId)) {
      this.removeComponent(entityId, componentClass);
    }
    this.entities.delete(entityId);
  }

  query(componentClasses) {
    const entities = [];
    if (componentClasses.length === 0) return [];

    // Start with entities having the first component
    const firstComponentMap = this.components.get(componentClasses[0]);
    if (!firstComponentMap) return [];

    for (const entityId of firstComponentMap.keys()) {
      let hasAll = true;
      for (let i = 1; i < componentClasses.length; i++) {
        if (!this.hasComponent(entityId, componentClasses[i])) {
          hasAll = false;
          break;
        }
      }
      if (hasAll) {
        entities.push(entityId);
      }
    }
    return entities;
  }

  registerSystem(system) {
    this.systems.push(system);
  }

  setResource(name, value) {
    this.resources[name] = value;
  }

  getResource(name) {
    return this.resources[name];
  }

  clear() {
    this.entities.clear();
    this.components.clear();
    this.nextEntityId = 0;
    // Keep systems and resources
  }

  update(dt) {
    const pauseState = this.getResource('pauseState');
    const errorState = this.getResource('errorState');
    const isPaused = pauseState ? pauseState.paused : false;
    const hasError = errorState ? errorState.hasError : false;

    for (const system of this.systems) {
      // Skip systems if paused (and system doesn't run in pause) OR if there's an error
      if (system.update && ((!system.runInPause && isPaused) || hasError)) {
        continue; // Skip systems if paused or error occurred
      }
      if (system.update) {
        system.update(this, dt);
      }
    }
    canvas.focus();
  }
}

// --- Components (Plain Data) ---

class PositionComponent {
  constructor(x = 0, y = 0) {
    this.pos = new Vector2(x, y);
    this.prevPos = new Vector2(x, y); // Store previous position here
  }
}
class VelocityComponent { constructor(x = 0, y = 0) { this.vel = new Vector2(x, y); } }
class RadiusComponent { constructor(radius = 0.1) { this.radius = radius; } }
class MassComponent { constructor(mass = 1.0) { this.mass = mass; } }
class RestitutionComponent { constructor(restitution = 0.5) { this.restitution = restitution; } }
class GravityAffectedComponent { /* Tag component */ }
class BallTagComponent { /* Tag component */ }
class BorderComponent { constructor(points = []) { this.points = points.map(p => p.clone()); } }
class FlipperTagComponent { }
class FlipperStateComponent {
  constructor(length, restAngle, maxRotation, angularVelocity) {
    this.length = length;
    this.restAngle = restAngle;
    this.maxRotation = Math.abs(maxRotation);
    this.sign = Math.sign(maxRotation); // Direction it rotates
    this.angularVelocity = angularVelocity;
    // Dynamic state
    this.rotation = 0.0; // Current rotation from restAngle
    this.currentAngularVelocity = 0.0; // Velocity in the last frame
    this.pressed = false; // Was it activated?
  }
}
class ObstacleTagComponent { /* Tag component */ }
class PauseStateComponent { constructor(paused = true) { this.paused = paused; } }
class SimulationErrorStateComponent { constructor(hasError = false) { this.hasError = hasError; } } // New component for error state
class CableLinkComponent { /* Tag component to identify entities that are part of a cable path and can interact */ }

// --- new component: tag an entity as being grabbed by the mouse ---
class GrabComponent {
  constructor(offsetX = 0.0, offsetY = 0.0) {
    this.offset = new Vector2(offsetX, offsetY);
  }
}

// Represents a single segment constraint between two entities
class CableJointComponent {
  constructor(entityA, entityB, restLength, attachmentPointA_world, attachmentPointB_world) {
    this.entityA = entityA;
    this.entityB = entityB;
    this.restLength = restLength; // dn - the dynamic maximum length
    this.isActive = true; // For merge/split logic
    this.attachmentPointA_world = attachmentPointA_world.clone();
    this.attachmentPointB_world = attachmentPointB_world.clone();
  }
}

// Represents the entire cable path
class CablePathComponent {
  constructor(world, jointEntities = [], linkTypes = [], cw = []) {
    this.totalRestLength = 0.0;
    this.jointEntities = jointEntities; // Ordered list of CableJoint entity IDs
    this.linkTypes = linkTypes; // Ordered. linkTypes.length === jointEntities.length + 1
    this.cw = cw // Ordered. cw.length === linkTypes.length
    this.stored = new Array(cw.length).fill(0.0); // Ordered. stored.length === cw.length

    for (const jointId of jointEntities) {
      const joint = world.getComponent(jointId, CableJointComponent);
      this.totalRestLength += joint.restLength;
    }
    for (let i = 0; i < jointEntities.length - 1; i++) { // Iterate over adjacent pairs
      const jointId_i = jointEntities[i];
      const jointId_i_plus_1 = jointEntities[i + 1];
      const joint_i = world.getComponent(jointId_i, CableJointComponent);
      const joint_i_plus_1 = world.getComponent(jointId_i_plus_1, CableJointComponent);
      const linkId = joint_i.entityB;
      const linkId2 = joint_i_plus_1.entityA;
      if (linkId !== linkId2) {
        console.warn("Links don't match up. There's something wrong with this cable path.");
        return;
      }
      const isRolling = linkTypes[i + 1] === 'rolling';
      // Assuming standard path structure A->B, A->B, check B_i == A_i+1
      if (isRolling) {
        const rolling_link = world.getComponent(linkId, CableLinkComponent);
        const center = world.getComponent(linkId, PositionComponent).pos;
        const radius = world.getComponent(linkId, RadiusComponent).radius;
        const isCw = cw[i + 1];

        const initialStoredLength = signedArcLengthOnWheel(
            joint_i.attachmentPointB_world,
            joint_i_plus_1.attachmentPointA_world,
            center,
            radius,
            isCw,
            true
        );
        this.stored[i + 1] = initialStoredLength;
        this.totalRestLength += initialStoredLength;
      }
    }
  }
}


// Render-specific component (could be more complex)
class RenderableComponent {
  constructor(shape = 'circle', color = '#888888') {
    this.shape = shape; // 'circle', 'flipper', 'border'
    this.color = color;
  }
}

// --- Systems (Logic) ---

// --- System: Gravity ---
class GravitySystem {
  runInPause = false;
  update(world, dt) {
    const grabbed = world.getResource('grabbedBall');
    const gravity = world.getResource('gravity');
    if (!gravity) return;

    const entities = world.query([VelocityComponent, GravityAffectedComponent]);
    for (const entityId of entities) {
      if (entityId === grabbed) continue;
      const velComp = world.getComponent(entityId, VelocityComponent);
      velComp.vel.add(gravity, dt);
    }
  }
}

// --- System: Movement ---
class MovementSystem {
  runInPause = false;
  update(world, dt) {
    const grabbed = world.getResource('grabbedBall');
    // Update linear position
    const linearEntities = world.query([PositionComponent, VelocityComponent]);
    for (const entityId of linearEntities) {
      if (entityId === grabbed) continue;
      const posComp = world.getComponent(entityId, PositionComponent);
      const velComp = world.getComponent(entityId, VelocityComponent);
      posComp.pos.add(velComp.vel, dt);
    }
  }
}


// --- System: Cable Attachment Update ---
// Calculates tangent points and updates rest lengths (dn) BEFORE the main solver
class CableAttachmentUpdateSystem {
  runInPause = false; // Physics system

  update(world, dt) {
    const debugPoints = world.getResource('debugRenderPoints');
    // Clear points from the previous frame
    for (const key in debugPoints) {
        delete debugPoints[key];
    }

    const pathEntities = world.query([CablePathComponent]);
    //// Merge joints
    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (!path || path.jointEntities.length < 2) continue;
      const jointsInPath = path.jointEntities;
      for (let i = 0; i < path.jointEntities.length - 1; i++) { // Iterate over adjacent pairs
        const jointId_i = path.jointEntities[i];
        const jointId_i_plus_1 = path.jointEntities[i + 1];
        const joint_i = world.getComponent(jointId_i, CableJointComponent);
        const joint_i_plus_1 = world.getComponent(jointId_i_plus_1, CableJointComponent);
        const linkId = joint_i.entityB;
        const linkId2 = joint_i_plus_1.entityA;
        if (linkId !== linkId2) {
          console.warn("Merge loop saw disconnected cable path");
        }
        if (joint_i.entityA === joint_i_plus_1.entityB) {
          continue;
        }
        const isRolling = path.linkTypes[i + 1] === 'rolling';
        if (isRolling) {
          // --- New Merge Condition ---
          const linkId = joint_i.entityB; // Shared rolling link
          const radiusComp = world.getComponent(linkId, RadiusComponent);
          const linkRadius = radiusComp ? radiusComp.radius : 0;

          // Calculate angle between the two segments
          const pA1 = joint_i.attachmentPointA_world;
          const pB1 = joint_i.attachmentPointB_world;
          const pA2 = joint_i_plus_1.attachmentPointA_world;
          const pB2 = joint_i_plus_1.attachmentPointB_world;

          const vec1 = pB1.clone().subtract(pA1);
          const vec2 = pB2.clone().subtract(pA2);
          const len1Sq = vec1.lengthSq();
          const len2Sq = vec2.lengthSq();
          let angle = 0; // Default to 0 (straight) if segments are too short

          if (len1Sq > 1e-9 && len2Sq > 1e-9) {
              vec1.normalize();
              vec2.normalize();
              const dot = vec1.dot(vec2);
              angle = Math.acos(Math.max(-1.0, Math.min(1.0, dot))); // Angle between 0 and PI
          }

          const storedLength = path.stored[i + 1];
          const angleThreshold = Math.PI / 180.0; // 1 degree
          const nothing_stored = storedLength < 0.0;
          const little_stored_and_angle_minimal = radiusComp && storedLength < linkRadius && angle < angleThreshold;

          // Complex merge condition
          if (nothing_stored) {
            console.log(`Merging joints ${jointId_i} and ${jointId_i_plus_1} (stored: ${storedLength.toFixed(4)}, radius: ${linkRadius.toFixed(4)}, angle: ${(angle * 180/Math.PI).toFixed(2)} degrees)`);
            if (angle > 10.0 * Math.PI/180.0) {
              console.warn("angle > 10.0 degrees");
            }
            if (angle < 0.0) {
              console.warn("angle < 0.0");
            }
            const posA = world.getComponent(joint_i.entityA, PositionComponent).prevPos;
            const radiusA = world.getComponent(joint_i.entityA, RadiusComponent)?.radius;
            const cwA = path.cw[i];
            const posB = world.getComponent(joint_i_plus_1.entityB, PositionComponent).prevPos;
            const radiusB = world.getComponent(joint_i_plus_1.entityB, RadiusComponent)?.radius;
            const cwB = path.cw[i+2];

            const lengthToAdd = joint_i_plus_1.restLength + path.stored[i + 1];
            joint_i.restLength += lengthToAdd;
            joint_i.entityB = joint_i_plus_1.entityB;
            const isRollingA = path.linkTypes[i] === 'rolling';
            const isAttachmentA = path.linkTypes[i] === 'attachment';
            const isRollingB = path.linkTypes[i+2] === 'rolling';
            const isAttachmentB = path.linkTypes[i+2] === 'attachment';
            if (isRollingA && isRollingB) {
              const tangents = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
              const sA = signedArcLengthOnWheel(joint_i.attachmentPointA_world, tangents.a_circle, posA, radiusA, cwA);
              path.stored[i] += sA;
              joint_i.restLength -= sA;
              const sB = signedArcLengthOnWheel(joint_i_plus_1.attachmentPointB_world, tangents.b_circle, posB, radiusB, cwB);
              path.stored[i+2] -= sB;
              joint_i.restLength += sB;
              joint_i.attachmentPointA_world.set(tangents.a_circle);
              joint_i.attachmentPointB_world.set(tangents.b_circle);
            } else if (isRollingA && isAttachmentB) {
              const tangents = tangentFromCircleToPoint(posB, posA, radiusA, cwA);
              const sA = signedArcLengthOnWheel(joint_i.attachmentPointA_world, tangents.a_circle, posA, radiusA, cwA);
              path.stored[i] += sA;
              joint_i.restLength -= sA;
              joint_i.attachmentPointA_world.set(tangents.a_circle);
              joint_i.attachmentPointB_world.set(tangents.a_attach);
            } else if (isAttachmentA && isRollingB) {
              const tangents = tangentFromPointToCircle(posA, posB, radiusB, cwB);
              const sB = signedArcLengthOnWheel(joint_i_plus_1.attachmentPointB_world, tangents.a_circle, posB, radiusB, cwB);
              path.stored[i+2] -= sB;
              joint_i.restLength += sB;
              joint_i.attachmentPointA_world.set(tangents.a_attach);
              joint_i.attachmentPointB_world.set(tangents.a_circle);
            } else {
              joint_i.attachmentPointA_world.set(posA.clone());
              joint_i.attachmentPointB_world.set(posB.clone());
            }

            path.jointEntities.splice(i+1, 1);
            path.stored.splice(i+1, 1);
            path.cw.splice(i+1, 1);
            path.linkTypes.splice(i+1, 1);
            joint_i_plus_1.isActive = false;
            world.destroyEntity(jointId_i_plus_1);
          }
        }
      }
    }


    // -- New attachment points --
    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (!path) continue; // Ensure path exists

      // Iterate through joints to calculate NEW tangent points and update lengths
      for (var jointIdx = 0; jointIdx < path.jointEntities.length; jointIdx++) {
        const jointId = path.jointEntities[jointIdx];
        const joint = world.getComponent(jointId, CableJointComponent);
        if (!joint || !joint.isActive) continue; // Check joint validity

        const A = jointIdx; // Index for link/cw/stored related to entity A side
        const B = jointIdx + 1; // Index for link/cw/stored related to entity B side

        const entityA = joint.entityA;
        const entityB = joint.entityB;

        // Get components for Entity A using CURRENT positions
        const posAComp = world.getComponent(entityA, PositionComponent);
        const radiusAComp = world.getComponent(entityA, RadiusComponent);
        const linkAComp = world.getComponent(entityA, CableLinkComponent);
        const posA = posAComp?.pos;
        const prevPosA = posAComp?.prevPos;
        const radiusA = radiusAComp?.radius;
        const cwA = path.cw[A];
        const attachmentLinkA = path.linkTypes[A] === 'attachment';
        const rollingLinkA = path.linkTypes[A] === 'rolling';

        // Get components for Entity B using CURRENT positions
        const posBComp = world.getComponent(entityB, PositionComponent);
        const radiusBComp = world.getComponent(entityB, RadiusComponent);
        const linkBComp = world.getComponent(entityB, CableLinkComponent);
        const posB = posBComp?.pos;
        const prevPosB = posBComp?.prevPos;
        const radiusB = radiusBComp?.radius;
        const cwB = path.cw[B];
        const attachmentLinkB = path.linkTypes[B] === 'attachment';
        const rollingLinkB = path.linkTypes[B] === 'rolling';

        if (!posA || !posB) {
          console.warn(`CableJoint ${jointId} missing PositionComponent on entities ${entityA} or ${entityB}. Skipping update.`);
          continue;
        }
        if (rollingLinkA && (!radiusAComp || !linkAComp)) {
           console.warn(`CableJoint ${jointId} entity ${entityA} is 'rolling' but missing RadiusComponent or CableLinkComponent. Skipping update.`);
           continue;
        }
         if (rollingLinkB && (!radiusBComp || !linkBComp)) {
           console.warn(`CableJoint ${jointId} entity ${entityB} is 'rolling' but missing RadiusComponent or CableLinkComponent. Skipping update.`);
           continue;
        }


        let attachmentA_current, attachmentB_current;
        let sA = 0; // Change in stored length on side A due to wrapping/unwrapping this frame
        let sB = 0; // Change in stored length on side B due to wrapping/unwrapping this frame

        // --- Calculate Ideal Current Attachment Points based on CURRENT entity positions ---
        let tangents; // Declare tangents variable outside specific cases
        if (attachmentLinkA && rollingLinkB) {
          tangents = tangentFromPointToCircle(posA, posB, radiusB, cwB);
          attachmentA_current = tangents.a_attach; // This is just posA
          attachmentB_current = tangents.a_circle; // Tangent on circle B
        } else if (rollingLinkA && attachmentLinkB) {
          tangents = tangentFromCircleToPoint(posB, posA, radiusA, cwA);
          attachmentA_current = tangents.a_circle; // Tangent on circle A
          attachmentB_current = tangents.a_attach; // This is just posB
        } else if (rollingLinkA && rollingLinkB) {
          tangents = tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB);
          if (tangents) {
              attachmentA_current = tangents.a_circle;
              attachmentB_current = tangents.b_circle;
          } else {
              // Fallback if tangent calculation fails (e.g., overlap)
              console.warn(`Could not calculate tangents between rolling ${entityA} and ${entityB}. Using previous points.`);
              // Keep previous points to avoid NaN issues, but lengths won't update correctly this frame
              attachmentA_current = joint.attachmentPointA_world.clone();
              attachmentB_current = joint.attachmentPointB_world.clone();
              // Set sA and sB to 0 because we couldn't calculate the change
              sA = 0;
              sB = 0;
              // Skip the arc length calculation below for this case
              // We will jump directly to updating the joint's points at the end
          }
        } else { // attachmentLinkA && attachmentLinkB
          attachmentA_current = posA.clone();
          attachmentB_current = posB.clone();
          sA = 0; // No arc length change for point-to-point
          sB = 0;
        }

        if (tangents !== null) { // Check if tangents were successfully calculated or if it's point-to-point
            if (rollingLinkA) {
                const vec_prevCenter_to_prevAttachA = joint.attachmentPointA_world.clone().subtract(prevPosA);
                const projected_prevAttachA = posA.clone().add(vec_prevCenter_to_prevAttachA);
                sA = signedArcLengthOnWheel(projected_prevAttachA, attachmentA_current, posA, radiusA, cwA);
            } else {
                sA = 0;
            }

            if (rollingLinkB) {
                const vec_prevCenter_to_prevAttachB = joint.attachmentPointB_world.clone().subtract(prevPosB);
                const projected_prevAttachB = posB.clone().add(vec_prevCenter_to_prevAttachB);
                sB = signedArcLengthOnWheel(projected_prevAttachB, attachmentB_current, posB, radiusB, cwB);
            } else {
                sB = 0;
            }
        }
        // --- End Wrapping/Unwrapping Calculation ---

        // --- Update stored lengths and rest length based on calculated sA and sB ---
        // sA > 0 means cable wrapped onto A (in preferred direction) -> stored[A] increases, restLength decreases
        path.stored[A] += sA;
        joint.restLength -= sA;
        path.stored[B] -= sB;
        joint.restLength += sB;

        if (joint.restLength < 0) {
          console.warn(`Joint ${jointId} restLength became negative (${joint.restLength.toFixed(4)})`);
          if (rollingLinkA && rollingLinkB) {
            console.warn("Clamping both");
            path.stored[A] += joint.restLength/2.0;
            path.stored[B] += joint.restLength/2.0;
            joint.restLength = 0.0;
          } else if (rollingLinkA) {
            console.warn("Clamping left");
            path.stored[A] += joint.restLength;
            joint.restLength = 0.0;
          } else if (rollingLinkB) {
            console.warn("Clamping right");
            path.stored[B] += joint.restLength;
            joint.restLength = 0.0;
          } else {
            console.warn("No Clamp");
          }
        }

        // --- Update the joint's stored attachment points for the NEXT frame's comparison ---
        joint.attachmentPointA_world.set(attachmentA_current);
        joint.attachmentPointB_world.set(attachmentB_current);

      } // End loop through joints
    } // End loop through paths

    // Split joints
    // Entities that can cause a split
    const potentialSplitters = world.query([PositionComponent, RadiusComponent, CableLinkComponent]);
    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (!path || path.jointEntities.length < 1) continue;
      for (var jointIndex = 0; jointIndex < path.jointEntities.length; jointIndex++) {
        const jointId = path.jointEntities[jointIndex];
        const joint = world.getComponent(jointId, CableJointComponent);
        if (!joint.isActive) {
          console.warn("An inactive joint seems to be part of a path right now.");
          continue;
        }
        const pA = joint.attachmentPointA_world;
        const pB = joint.attachmentPointB_world;
        for (const splitterId of potentialSplitters) {
          if (splitterId === joint.entityA || splitterId === joint.entityB) {
            continue;
          }
          const posSplitter = world.getComponent(splitterId, PositionComponent).pos;
          const prevPosSplitter = world.getComponent(splitterId, PositionComponent).prevPos;
          const radiusSplitter = world.getComponent(splitterId, RadiusComponent).radius;
          if (lineSegmentCircleIntersection(pA, pB, posSplitter, radiusSplitter)) {
            console.log(`Splitting joint ${jointId} due to intersection with ${splitterId}`);
            const newJointId = world.createEntity();

            const cw = rightOfLine(prevPosSplitter, pA, pB);
            const linkTypeB = path.linkTypes[jointIndex + 1];
            const entityB = joint.entityB;
            const entityA = joint.entityA;
            var initialPoints;
            var initialDist1;
            var attachmentPointAForNewJoint;
            var attachmentPointBForNewJoint;
            const posB = world.getComponent(entityB, PositionComponent).pos;
            if (linkTypeB === 'rolling') {
              const radiusB = world.getComponent(entityB, RadiusComponent).radius;
              const cwB = path.cw[jointIndex + 1];
              initialPoints = tangentFromCircleToCircle(posSplitter, radiusSplitter, cw, posB, radiusB, cwB);
              initialDist1 = initialPoints.a_circle.clone().subtract(initialPoints.b_circle).length();
              attachmentPointAForNewJoint = initialPoints.a_circle;
              attachmentPointBForNewJoint = initialPoints.b_circle;
              const sB = signedArcLengthOnWheel(pB, attachmentPointBForNewJoint, posB, radiusB, cwB);
              path.stored[jointIndex + 1] -= sB;
              joint.restLength += sB;
            } else if (linkTypeB === 'attachment') {
              initialPoints = tangentFromCircleToPoint(posB, posSplitter, radiusSplitter, cw);
              initialDist1 = initialPoints.a_attach.clone().subtract(initialPoints.a_circle).length();
              attachmentPointAForNewJoint = initialPoints.a_circle;
              attachmentPointBForNewJoint = initialPoints.a_attach;
            } else {
              console.warn(`Splitting cable joint attached to ${linkTypeB} is not supported.`);
            }

            path.jointEntities.splice(jointIndex + 1, 0, newJointId);
            path.cw.splice(jointIndex + 1, 0, cw);
            path.linkTypes.splice(jointIndex + 1, 0, 'rolling');

            const linkTypeA = path.linkTypes[jointIndex];
            const posA = world.getComponent(entityA, PositionComponent).pos;
            var newAttachmentPointBForJoint;
            var newAttachmentPointAForJoint;
            var initialPoints2;
            var initialDist2;
            if (linkTypeA === 'rolling') {
                // --- Case: Rolling -> Splitter ---
                const radiusA = world.getComponent(entityA, RadiusComponent).radius;
                const cwA = path.cw[jointIndex];
                initialPoints2 = tangentFromCircleToCircle(posA, radiusA, cwA, posSplitter, radiusSplitter, cw);
                initialDist2 = initialPoints2.a_circle.clone().subtract(initialPoints2.b_circle).length();
                newAttachmentPointAForJoint = initialPoints2.a_circle;
                newAttachmentPointBForJoint = initialPoints2.b_circle;
                const sA = signedArcLengthOnWheel(pA, newAttachmentPointAForJoint, posA, radiusA, cwA);
                joint.attachmentPointA_world.set(newAttachmentPointAForJoint);
                path.stored[jointIndex] += sA;
                joint.restLength -= sA;
            } else if (linkTypeA === 'attachment') {
                // --- Case: Attachment -> Splitter ---
                initialPoints2 = tangentFromPointToCircle(posA, posSplitter, radiusSplitter, cw);
                initialDist2 = initialPoints2.a_circle.clone().subtract(initialPoints2.a_attach).length();
                newAttachmentPointAForJoint = initialPoints2.a_attach; // Point A is the attachment
                newAttachmentPointBForJoint = initialPoints2.a_circle; // Point B is on the splitter
                // No sA calculation needed as link A is attachment
                joint.attachmentPointA_world.set(newAttachmentPointAForJoint);
            } else {
                console.warn(`Splitting cable joint coming from link type ${linkTypeA} is not supported.`);
                // Skip this split if unsupported combination
                continue; // Go to the next potential splitter
            }

            // --- Debug Visualization for initialPoints2 ---
            //if (initialPoints2 && debugPoints) {
            //    if (initialPoints2.a_attach) {
            //        debugPoints[`split_${jointId}_init2_attach`] = { pos: initialPoints2.a_attach, color: '#FFA500' }; // Orange
            //    }
            //    if (initialPoints2.a_circle) {
            //        debugPoints[`split_${jointId}_init2_circleA`] = { pos: initialPoints2.a_circle, color: '#FF00FF' }; // Magenta
            //    }
            //     if (initialPoints2.b_circle) { // For circle-circle case
            //        debugPoints[`split_${jointId}_init2_circleB`] = { pos: initialPoints2.b_circle, color: '#00FFFF' }; // Cyan
            //    }
            //}
            // --- End Debug Visualization ---


            joint.entityB = splitterId; // Original joint now connects A -> Splitter

            // Calculate stored length 's' on the new splitter link
            const s = signedArcLengthOnWheel(
                newAttachmentPointBForJoint, // Point on splitter from A side
                attachmentPointAForNewJoint, // Point on splitter from B side
                posSplitter,
                radiusSplitter,
                cw
            );

            // The line is slightly stretched due to the intersection with the new link (the splitterId Entity).
            // Distributes the discrepancy (length_error) to both sides of the new link such that tension remains constant on both sides of the new link.
            // The paper states:
            // "The rest length of the original joint is distributed among the new joints such that the tension on both sides are equal."
            // "The rest length is split such that the tension remains constant."
            // "tension (the current length divided by the rest length)"
            // Distribute original rest length (minus new stored length s)
            // between the two new segments to maintain equal tension.
            const originalRestLength = joint.restLength; // Store before modifying
            const totalDist = initialDist1 + initialDist2;
            let newRestLength1 = 0; // For new joint (splitter -> entityB)
            let newRestLength2 = 0; // For original joint (entityA -> splitter)


            if (totalDist > 1e-9) { // Avoid division by zero if distances are tiny
                const availableRestLength = originalRestLength - s;
                if (availableRestLength < 0) {
                    console.warn(`Split resulted in negative available rest length (${availableRestLength.toFixed(4)}).`);
                }
                newRestLength1 = availableRestLength * initialDist1 / totalDist;
                newRestLength2 = availableRestLength * initialDist2 / totalDist;
            } else {
                console.warn("Split occurred with near-zero distance between new segments:", totalDist);
            }

            // Update original joint (now entityA -> splitter)
            joint.restLength = newRestLength2;
            joint.attachmentPointB_world.set(newAttachmentPointBForJoint); // Update endpoint

            // Create the new joint (splitter -> entityB)
            world.addComponent(newJointId, new CableJointComponent(
                splitterId, entityB, newRestLength1, attachmentPointAForNewJoint, attachmentPointBForNewJoint));
            world.addComponent(newJointId, new RenderableComponent('line', linecolor1));
            const discrepancy = originalRestLength - s - newRestLength1 - newRestLength2;
            const tension1 = initialDist1/newRestLength1;
            const tension2 = initialDist2/newRestLength2;


            console.log(`Split: L_orig=${originalRestLength.toFixed(4)}, s=${s.toFixed(4)} -> L1=${newRestLength1.toFixed(4)} (d1=${initialDist1.toFixed(4)}), L2=${newRestLength2.toFixed(4)} (d2=${initialDist2.toFixed(4)})`);
            console.log(`Split stats: discrepancy=${discrepancy.toFixed(4)}, tension1=${tension1.toFixed(4)}, tension2=${tension2.toFixed(4)}`);
            if (discrepancy > 1.0) {
              console.warn("discrepancy > 1.0");
            }
            if (tension1 > 1.5) {
              console.warn("tension1 > 1.5");
            }
            if (tension2 > 1.5) {
              console.warn("tension2 > 1.5");
            }
            path.stored.splice(jointIndex + 1, 0, s);
          }
        }
      }
    }

    // Even out tension
    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (!path || path.jointEntities.length < 2) continue;
      for (var jointIndex = 0; jointIndex < path.jointEntities.length - 1; jointIndex++) {
        const jointId_i = path.jointEntities[jointIndex];
        const jointId_i_plus_1 = path.jointEntities[jointIndex + 1];
        const joint_i = world.getComponent(jointId_i, CableJointComponent);
        const joint_i_plus_1 = world.getComponent(jointId_i_plus_1, CableJointComponent);
        if (!joint_i.isActive || !joint_i_plus_1.isActive) {
          console.warn("An inactive joint seems to be part of a path right now.");
          continue;
        }
        const pA = joint_i.attachmentPointA_world;
        const pB = joint_i.attachmentPointB_world;
        const pC = joint_i_plus_1.attachmentPointA_world;
        const pD = joint_i_plus_1.attachmentPointB_world;
        const l_i = joint_i.restLength;
        const l_i_plus_1 = joint_i_plus_1.restLength;
        const availableRestLength = l_i + l_i_plus_1;
        const d_i = new Vector2().subtractVectors(pA, pB).length();
        const d_i_plus_1 = new Vector2().subtractVectors(pC, pD).length();
        const totalDist = d_i + d_i_plus_1;
        const tension_i = d_i/joint_i.restLength;
        const tension_i_plus_1 = d_i_plus_1/joint_i_plus_1.restLength;
        //if (tension_i > 1.0 || tension_i_plus_1 > 1.0) {
        joint_i.restLength = availableRestLength * d_i/totalDist;
        joint_i_plus_1.restLength = availableRestLength * d_i_plus_1/totalDist;
          //joint_i.restLength = 0.5*availableRestLength * (d_i/totalDist + l_i/availableRestLength);
          //joint_i_plus_1.restLength = 0.5*availableRestLength * (d_i_plus_1/totalDist + l_i_plus_1/availableRestLength);
        //}
        //console.log(`tension_i=${tension_i}, tension_i_plus_1=${tension_i_plus_1}`);
      }
    }


    const linkEntities = world.query([CableLinkComponent, PositionComponent]);
    for (const linkId of linkEntities) {
      const posComp = world.getComponent(linkId, PositionComponent)
      posComp.prevPos.set(posComp.pos);
    }

    // Debugging/test loop 1
    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (!path || path.jointEntities.length < 1) continue;
      var totalCurrentDist = 0.0;
      var totalCurrentRestLength = 0.0;
      for (var jointIdx = 0; jointIdx < path.jointEntities.length; jointIdx++) {
        const jointId = path.jointEntities[jointIdx];
        const joint = world.getComponent(jointId, CableJointComponent);

        // Ensure joint exist and is active
        if (!joint || !joint.isActive) continue;

        // Calculate current lengths using the attachment points updated in Pass A
        const currentDist = joint.attachmentPointA_world.distanceTo(joint.attachmentPointB_world);
        totalCurrentDist += currentDist + path.stored[jointIdx + 1];
        totalCurrentRestLength += joint.restLength + path.stored[jointIdx + 1];
      }

      const error = path.totalRestLength - totalCurrentRestLength;
      //console.log(`error path ${pathId}: ${error}`); // rest length error is and should be very close to zero
      //console.log(`stored: ${path.stored}`);
    }
  }
}

class PBDBallBallCollisions {
  runInPause = false;
  update(world, dt) {
    const grabbed = world.getResource('grabbedBall');
    const ballEntities = world.query([BallTagComponent, PositionComponent, VelocityComponent, RadiusComponent, MassComponent, RestitutionComponent]);
    for (let i = 0; i < ballEntities.length; i++) {
      for (let j = i + 1; j < ballEntities.length; j++) {
        const e1 = ballEntities[i];
        const e2 = ballEntities[j];

        const p1 = world.getComponent(e1, PositionComponent).pos;
        const v1 = world.getComponent(e1, VelocityComponent).vel;
        const r1 = world.getComponent(e1, RadiusComponent).radius;
        const m1 = world.getComponent(e1, MassComponent).mass;
        const res1 = world.getComponent(e1, RestitutionComponent).restitution;

        const p2 = world.getComponent(e2, PositionComponent).pos;
        const v2 = world.getComponent(e2, VelocityComponent).vel;
        const r2 = world.getComponent(e2, RadiusComponent).radius;
        const m2 = world.getComponent(e2, MassComponent).mass;
        const res2 = world.getComponent(e2, RestitutionComponent).restitution;

        const restitution = Math.min(res1, res2);
        const dir = new Vector2().subtractVectors(p2, p1);
        const dSq = dir.lengthSq();
        const rSum = r1 + r2;

        if (dSq == 0.0 || dSq > rSum * rSum) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d); // Normalize

        // Resolve penetration
        const corr = (rSum - d) / 2.0;
        p1.add(dir, -corr);
        p2.add(dir, corr);

        // Resolve velocity
        const vel1_dot = v1.dot(dir);
        const vel2_dot = v2.dot(dir);

        const newV1_dot = (m1 * vel1_dot + m2 * vel2_dot - m2 * (vel1_dot - vel2_dot) * restitution) / (m1 + m2);
        const newV2_dot = (m1 * vel1_dot + m2 * vel2_dot - m1 * (vel2_dot - vel1_dot) * restitution) / (m1 + m2);

        v1.add(dir, newV1_dot - vel1_dot);
        v2.add(dir, newV2_dot - vel2_dot);
      }
    }
  }
}

class PBDBallObstacleCollisions {
  runInPause = false;

  update(world, dt) {
    const ballEntities = world.query([BallTagComponent, PositionComponent, VelocityComponent, RadiusComponent, MassComponent, RestitutionComponent]);
    const obstacleEntities = world.query([ObstacleTagComponent, PositionComponent, RadiusComponent]);
    for (const ballId of ballEntities) {
      const p1 = world.getComponent(ballId, PositionComponent).pos;
      const v1 = world.getComponent(ballId, VelocityComponent).vel;
      const r1 = world.getComponent(ballId, RadiusComponent).radius;

      for (const obsId of obstacleEntities) {
        const p2 = world.getComponent(obsId, PositionComponent).pos;
        const r2 = world.getComponent(obsId, RadiusComponent).radius;
        const pushVel = 2.0;

        const dir = new Vector2().subtractVectors(p1, p2);
        const dSq = dir.lengthSq();
        const rSum = r1 + r2;

        if (dSq == 0.0 || dSq > rSum * rSum) continue;

        const d = Math.sqrt(dSq);
        dir.scale(1.0 / d); // Normalize

        // Resolve penetration
        const corr = rSum - d;
        p1.add(dir, corr);

        // Resolve velocity (simple push)
        const v_dot = v1.dot(dir);
        v1.add(dir, pushVel - v_dot); // Impart obstacle's push velocity along normal
      }
    }
  }
}

class PBDCableConstraintSolver {
  runInPause = false;

  update(world, dt) {
    const pathEntities = world.query([CablePathComponent]);
    const epsilon = 1e-9; // Small value to avoid division by zero

    for (const pathId of pathEntities) {
      const path = world.getComponent(pathId, CablePathComponent);
      if (!path || path.jointEntities.length === 0) continue;

      // --- 1. Calculate Total Current Length and Overall Tension ---
      let totalCurrentLength = 0.0;
      for (let i = 0; i < path.jointEntities.length; i++) {
        const jointId = path.jointEntities[i];
        const joint = world.getComponent(jointId, CableJointComponent);
        if (!joint || !joint.isActive) continue;

        const currentSegmentLength = joint.attachmentPointA_world.distanceTo(joint.attachmentPointB_world);
        totalCurrentLength += currentSegmentLength;

        // Add stored length for the link *after* this joint (if it exists)
        if (path.linkTypes[i+1] === 'rolling') {
           totalCurrentLength += path.stored[i+1];
        }
      }

      let overallTension = 0.0;
      if (path.totalRestLength > epsilon) {
        overallTension = totalCurrentLength / path.totalRestLength;
      }

      // --- Calculate Total Inverse Mass for the Entire Path ---
      let pathTotalInvMass = 0.0;
      const uniqueEntities = new Set();
      for (const jointId of path.jointEntities) {
          const joint = world.getComponent(jointId, CableJointComponent);
          if (!joint || !joint.isActive) continue;

          const entityA = joint.entityA;
          const entityB = joint.entityB;

          if (!uniqueEntities.has(entityA)) {
              uniqueEntities.add(entityA);
              const massAComp = world.getComponent(entityA, MassComponent);
              if (massAComp) {
                  const massA = massAComp.mass;
                  pathTotalInvMass += (massA > 0 ? 1.0 / massA : 0.0);
              }
          }
          if (!uniqueEntities.has(entityB)) {
              uniqueEntities.add(entityB);
              const massBComp = world.getComponent(entityB, MassComponent);
              if (massBComp) {
                  const massB = massBComp.mass;
                  pathTotalInvMass += (massB > 0 ? 1.0 / massB : 0.0);
              }
          }
      }

      // If the entire path is immovable, skip corrections for this path
      if (pathTotalInvMass <= epsilon) {
          continue;
      }

      // --- Calculate Total Path Error ---
      const totalPathError = totalCurrentLength - path.totalRestLength;


      // --- 2. Apply PBD distance constraint using proper gradient sums ---
      // Apply correction only if the entire path is longer than its rest length
      if (totalPathError > epsilon) {
        // Build per-entity gradients: ∂C/∂x
        const grads = new Map(); // entityId -> Vector2
        // Initialize gradients for each entity in the path
        for (const jointId of path.jointEntities) {
          const joint = world.getComponent(jointId, CableJointComponent);
          if (!joint || !joint.isActive) continue;
          for (const e of [joint.entityA, joint.entityB]) {
            if (!grads.has(e)) {
              grads.set(e, new Vector2(0, 0));
            }
          }
        }
        // Accumulate gradients for each segment
        for (const jointId of path.jointEntities) {
          const joint = world.getComponent(jointId, CableJointComponent);
          if (!joint || !joint.isActive) continue;
          const pA = joint.attachmentPointA_world;
          const pB = joint.attachmentPointB_world;
          const diff = new Vector2().subtractVectors(pB, pA);
          const len = diff.length();
          if (len <= epsilon) continue;
          const dir = diff.clone().scale(1.0 / len);
          grads.get(joint.entityA).add(dir);
          grads.get(joint.entityB).add(dir.clone().scale(-1.0));
        }
        // Compute denominator D = Σ invMass_i * |grad_i|²
        let denom = 0.0;
        const invMassMap = new Map();
        for (const [entityId, grad] of grads.entries()) {
          const massComp = world.getComponent(entityId, MassComponent);
          const invM = massComp && massComp.mass > 0 ? 1.0 / massComp.mass : 0.0;
          invMassMap.set(entityId, invM);
          denom += invM * grad.lengthSq();
        }
        if (denom <= epsilon) {
          console.warn("PBDCableConstraintSolver: zero denominator in constraint correction");
        } else {
          // Correction magnitude λ = -C/D
          const lambda = -totalPathError / denom;
          // Apply corrections per entity
          for (const [entityId, grad] of grads.entries()) {
            const invM = invMassMap.get(entityId);
            if (invM <= 0) continue;
            const delta = grad.clone().scale(-invM * lambda);
            const posComp = world.getComponent(entityId, PositionComponent);
            if (posComp) {
              posComp.pos.add(delta);
            }
            const velComp = world.getComponent(entityId, VelocityComponent);
            if (velComp && dt > epsilon) {
              velComp.vel.add(delta, 1.0 / dt);
            }
          }
        }
      }
    } // End loop through paths
  } // end update
} // end PBDCableConstraintSolver

// --- System: Rendering ---
class RenderSystem {
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
              // exactly at center → force straight‐down
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
      if (!path || path.jointEntities.length < 1) continue;
      const jointEntities = path.jointEntities;
      // Scale line width by zoom using instance property
      this.c.lineWidth = baseLineWidth * this.viewScaleMultiplier;
      for (const entityId of jointEntities) {
        const jointComp = world.getComponent(entityId, CableJointComponent);
        const renderComp = world.getComponent(entityId, RenderableComponent);

        if (!jointComp.isActive || renderComp.shape !== 'line') continue;

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
    this.c.lineWidth = 1;

    // Render All Renderable Entities (Circles/Obstacles/Etc.)
    const renderableEntities = world.query([PositionComponent, RenderableComponent]);
    for (const entityId of renderableEntities) {
        const posComp = world.getComponent(entityId, PositionComponent);
        const renderComp = world.getComponent(entityId, RenderableComponent);

        if (renderComp.shape === 'circle') {
            const radiusComp = world.getComponent(entityId, RadiusComponent); // Need radius for circles
            if (posComp && radiusComp) {
                this.c.fillStyle = renderComp.color;
                this.drawDisc(posComp.pos.x, posComp.pos.y, radiusComp.radius);
            }
        }
        // Add rendering for other shapes if needed (e.g., 'line' is handled below)
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
}


// --- State Dumping Function ---
function dumpWorldState(world) {
  const renderSystem = world.getResource('renderSystem');
  const viewportScale = renderSystem ? renderSystem.viewScaleMultiplier : 1.0;
  const viewportOffsetX = renderSystem ? renderSystem.viewOffsetX_sim : 0.0;
  const viewportOffsetY = renderSystem ? renderSystem.viewOffsetY_sim : 0.0;

  let dump = `
// --- Generated Error Test Case ---
testGeneratedError: {
  // Viewport settings from the time of the error
  viewport: { scale: ${viewportScale}, offsetX: ${viewportOffsetX}, offsetY: ${viewportOffsetY} },
  run: function() {
    const testName = "Generated Error Case";
    const world = new World();
    window.testWorld = world; // Set global world
    world.setResource('dt', ${world.getResource('dt')});
    world.setResource('debugRenderPoints', {});
    world.setResource('simWidth', ${world.getResource('simWidth')});
    world.setResource('simHeight', ${world.getResource('simHeight')});

    // --- Resources ---
`;
  // Dump simple resources (add more as needed)
  const gravity = world.getResource('gravity');
  if (gravity) {
    dump += `    world.setResource('gravity', new Vector2(${gravity.x}, ${gravity.y}));\n`;
  }
  const errorState = world.getResource('errorState');
  if (errorState) {
    dump += `    world.setResource('errorState', new SimulationErrorStateComponent(${errorState.hasError}));\n`;
  }
  // Note: pauseState is usually set by the test runner or UI
  // Note: renderSystem is created by the test runner

  dump += `
    // --- Entities and Components ---
`;
  const entityVarMap = new Map(); // Map original entityId to generated variable name

  // First pass: Create entities and map IDs
  for (const entityId of world.entities.keys()) {
    const varName = `entity${entityId}`;
    entityVarMap.set(entityId, varName);
    dump += `    const ${varName} = world.createEntity(); // Original ID: ${entityId}\n`;
  }
  dump += "\n"; // Add a newline for separation

  // Second pass: Add components
  for (const entityId of world.entities.keys()) {
    const varName = entityVarMap.get(entityId);
    const componentClasses = world.entities.get(entityId);
    if (!componentClasses) continue;

    for (const componentClass of componentClasses) {
      const component = world.getComponent(entityId, componentClass);
      if (!component) continue;

      let addCompStr = `    world.addComponent(${varName}, new ${componentClass.name}(`;

      // Serialize component data based on type
      switch (componentClass.name) {
        case 'PositionComponent':
          addCompStr += `${component.pos.x}, ${component.pos.y}`;
          break;
        case 'VelocityComponent':
          addCompStr += `${component.vel.x}, ${component.vel.y}`;
          break;
        case 'RadiusComponent':
          addCompStr += `${component.radius}`;
          break;
        case 'MassComponent':
          addCompStr += `${component.mass}`;
          break;
        case 'ObstaclePushComponent':
          addCompStr += `${component.pushVel}`;
          break;
        case 'ScoreComponent':
          addCompStr += `${component.value}`;
          break;
        case 'RestitutionComponent':
          addCompStr += `${component.restitution}`;
          break;
        case 'RenderableComponent':
          addCompStr += `'${component.shape}', '${component.color}'`;
          break;
        case 'FlipperStateComponent':
          addCompStr += `${component.length}, ${component.restAngle},  ${component.maxRotation}, ${component.angularVelocity}`;
          break;
        case 'CableJointComponent':
          const entityAVar = entityVarMap.get(component.entityA);
          const entityBVar = entityVarMap.get(component.entityB);
          if (!entityAVar || !entityBVar) {
            console.warn(`Could not find variable names for entities ${component.entityA} or ${component.entityB} in joint ${entityId}`);
            addCompStr = `    // Skipped CableJointComponent for ${varName} due to missing entity mapping\n`;
          } else {
            addCompStr += `${entityAVar}, ${entityBVar}, ${component.restLength}, `;
            addCompStr += `new Vector2(${component.attachmentPointA_world.x}, ${component.attachmentPointA_world.y}), `;
            addCompStr += `new Vector2(${component.attachmentPointB_world.x}, ${component.attachmentPointB_world.y})`;
          }
          break;
        case 'PauseStateComponent': // Usually a resource, but handle if added to entity
          addCompStr += component.paused;
          break;
        case 'SimulationErrorStateComponent': // Usually a resource, but handle if added to entity
          addCompStr += component.hasError;
          break;
        case 'BorderComponent':
          const pointsStr = component.points.map(p => `new Vector2(${p.x}, ${p.y})`).join(', ');
          addCompStr += `[${pointsStr}]`;
          break;
        case 'CablePathComponent': // Handled separately below
          break;
        case 'GravityAffectedComponent':
        case 'CableLinkComponent':
        case 'BallTagComponent':
        case 'FlipperTagComponent':
        case 'ObstacleTagComponent':
          break;
        default:
          console.warn(`Unhandled component type for serialization: ${componentClass.name}`);
          addCompStr = `    // Skipped unknown component ${componentClass.name} for ${varName}\n`;

      }
      if (!addCompStr.endsWith('\n')) { // Avoid adding extra parenthesis if skipped or handled above
        addCompStr += '));\n';
      }
      dump += addCompStr;
    }
    dump += "\n"; // Add a newline between entities
  }
  for (const entityId of world.entities.keys()) {
    const varName = entityVarMap.get(entityId);
    const componentClasses = world.entities.get(entityId);
    if (!componentClasses) continue;

    for (const componentClass of componentClasses) {
      const component = world.getComponent(entityId, componentClass);
      if (!component) continue;
      let addCompStr = "";
      // Register paths last. They depend on everything else being defined first
      if (componentClass.name === 'CablePathComponent') {
        addCompStr += `    world.addComponent(${varName}, new ${componentClass.name}(`;
        const jointVars = component.jointEntities.map(id => entityVarMap.get(id)).filter(Boolean);
        if (jointVars.length !== component.jointEntities.length) {
          console.warn(`Could not find variable names for all joints in path ${entityId}`);
          addCompStr = `    // Skipped CablePathComponent for ${varName} due to missing joint mapping\n`;
        } else {
          addCompStr += `world, [${jointVars.join(', ')}], `;
          addCompStr += `[${component.linkTypes.map(t => `'${t}'`).join(', ')}], `;
          addCompStr += `[${component.cw.join(', ')}]`;
          // Need to manually set stored and totalRestLength after creation in the test
          addCompStr += `));\n`;
          addCompStr += `    const pathComp_${varName} = world.getComponent(${varName}, CablePathComponent);\n`;
          addCompStr += `    pathComp_${varName}.stored = [${component.stored.join(', ')}];\n`;
          addCompStr += `    pathComp_${varName}.totalRestLength = ${component.totalRestLength};\n`;
          dump += addCompStr;
          continue; // Skip the default closing parenthesis below
        }
      }
      if (!addCompStr.endsWith('\n') && addCompStr !== "") { // Avoid adding extra parenthesis if skipped or handled above
        addCompStr += '));\n';
      }
      dump += addCompStr;
    }
  }
  dump += "\n"; // Add a newline between entities

  dump += `
    // --- Systems Registration (Copy from a working test or main file if needed) ---
    // world.registerSystem(new CableAttachmentUpdateSystem());
    // world.registerSystem(new PBDCableConstraintSolver());
    // ... etc ...

    // --- Run the system that caused the error (or the full update) ---
    // const system = new SpecificSystem();
    // system.update(world, world.getResource('dt'));
    // OR
    // world.update(world.getResource('dt')); // If the error happens during the full update cycle

    // --- Assertions (Add assertions here to verify the error or expected state) ---
    // assertTrue(..., testName, "Some condition");

    logTestResult(testName, false, "Generated test case - needs verification and assertions");
    return world; // Return world for rendering
  }
},
`;
  return dump;
}


// --- System: Input Replay ---
// Replays recorded inputLog into the InputSystem each frame
class InputReplaySystem {
  runInPause = false;
  constructor(inputLog, inputSystem) {
    this.inputLog = inputLog;
    this.currentIndex = 0;
    this.frame = 0;
    this.inputSystem = inputSystem;
  }
  update(world, dt) {
    if (this.inputLog.length > 0) {
      if (this.inputSystem.frame === this.inputLog[0].frame) {
        const frame = this.inputLog.shift();
        this.inputSystem.clicks = frame.clicks.slice();
        this.inputSystem.releases = frame.releases.slice();
      }
    }
  }
}

// Export for testing
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    Vector2,
    closestPointOnSegment,
    _tangentPointCircle,
    tangentFromPointToCircle,
    tangentFromCircleToPoint,
    tangentFromCircleToCircle,
    signedArcLengthOnWheel,
    lineSegmentCircleIntersection,
    rightOfLine,
    World,
    PositionComponent,
    RadiusComponent,
    CableJointComponent,
    CableLinkComponent,
    CablePathComponent,
    VelocityComponent,
    BallTagComponent,
    MassComponent,
    RestitutionComponent,
    GravityAffectedComponent,
    GravitySystem,
    MovementSystem,
    PBDBallBallCollisions,
    CableAttachmentUpdateSystem,
    InputReplaySystem,
    GrabComponent
  };
}

