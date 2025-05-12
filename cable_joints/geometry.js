import Vector2 from './vector2.js';

export function closestPointOnSegment(p, a, b) {
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
export function _tangentPointCircle(p_attach, p_circle, r_circle, cw, pointIsFirst) {
  const dVec = new Vector2().subtractVectors(p_circle, p_attach);
  const dSq = dVec.lengthSq();

  if (dSq <= r_circle * r_circle + 1e-9) {
    console.warn("Cable tangent calculation: Attachment point inside or on rolling circle.");
    const dir = dVec.lengthSq() > 1e-9 ? dVec.clone().normalize() : new Vector2(1, 0);
    return {
      a_attach: p_attach.clone(),
      a_circle: p_circle.clone().subtract(dir, r_circle)
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

export function tangentFromPointToCircle(p_attach, p_circle, r_circle, cw){
  return _tangentPointCircle(p_attach,p_circle,r_circle,cw,true);
}

export function tangentFromCircleToPoint(p_attach, p_circle, r_circle, cw){
  return _tangentPointCircle(p_attach,p_circle,r_circle,cw,false);
}

export function tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB) {
  let dVec = new Vector2().subtractVectors(posB, posA);
  let d = dVec.length();

  const r = (cwA === cwB) ? (radiusB - radiusA) : (radiusA + radiusB);
  const alpha = Math.atan2(dVec.y, dVec.x);
  const phi = Math.asin(Math.max(-1, Math.min(1, r / d)));

  let angleA, angleB;
  if (!cwA === cwB) {
    if (!cwA) {
      angleA = alpha - Math.PI / 2 + phi;
      angleB = alpha + Math.PI / 2 + phi;
    } else {
      angleA = alpha + Math.PI / 2 - phi;
      angleB = alpha - Math.PI / 2 - phi;
    }
  } else {
    if (!cwA) {
      angleA = alpha - Math.PI / 2 - phi;
      angleB = alpha - Math.PI / 2 - phi;
    } else {
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
export function signedArcLengthOnWheel(prevPoint, currPoint, center, radius, clockwisePreference, force_positive = false) {
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
export function lineSegmentCircleIntersection(p1, p2, center, radius) {
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
export function rightOfLine(x, p0, p1) {
  const v_x = p1.x - p0.x;
  const v_y = p1.y - p0.y;
  const w_x = x.x - p0.x;
  const w_y = x.y - p0.y;
  // Calculate the 2D cross product (determinant)
  const crossProduct = v_x * w_y - v_y * w_x;
  // Return true if the cross product is positive (indicating left)
  return crossProduct < 0.0;
}
