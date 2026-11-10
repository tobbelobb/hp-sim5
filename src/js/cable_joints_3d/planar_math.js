// Minimal planar helpers used by the 3D geometry projections.
export default class Vector2 {
  constructor(x = 0.0, y = 0.0) { this.x = x; this.y = y; }
  set(v) { this.x = v.x; this.y = v.y; return this; }
  clone() { return new Vector2(this.x, this.y); }
  add(v, s = 1.0) { this.x += v.x * s; this.y += v.y * s; return this; }
  addVectors(a, b) { this.x = a.x + b.x; this.y = a.y + b.y; return this; }
  subtract(v, s = 1.0) { this.x -= v.x * s; this.y -= v.y * s; return this; }
  subtractVectors(a, b) { this.x = a.x - b.x; this.y = a.y - b.y; return this; }
  distanceTo(b) { return Math.hypot(this.x - b.x, this.y - b.y); }
  length() { return Math.hypot(this.x, this.y); }
  lengthSq() { return this.x ** 2 + this.y ** 2; }
  scale(s) { this.x *= s; this.y *= s; return this; }
  dot(v) { return this.x * v.x + this.y * v.y; }
  normalize() { const l = this.length(); if (l > 0) this.scale(1 / l); return this; }
}

export function _tangentPointCircle(p_attach, p_circle, r_circle, cw, pointIsFirst) {
  const dVec = new Vector2().subtractVectors(p_circle, p_attach);
  const dSq = dVec.lengthSq();

  if (dSq <= r_circle * r_circle + 1e-9) {
    const dir = dVec.lengthSq() > 1e-9 ? dVec.clone().normalize() : new Vector2(1, 0);
    return {
      a_attach: p_attach.clone(),
      a_circle: p_circle.clone().subtract(dir, r_circle)
    };
  }

  const d = Math.sqrt(dSq);
  const alpha = Math.atan2(dVec.y, dVec.x);
  const phi = Math.asin(r_circle / d);

  const tangentPointAngleOnCircle = ((cw && pointIsFirst) || (!cw && !pointIsFirst))
    ? alpha + phi + Math.PI / 2
    : alpha - phi - Math.PI / 2;

  return {
    a_attach: p_attach.clone(),
    a_circle: new Vector2(
      p_circle.x + r_circle * Math.cos(tangentPointAngleOnCircle),
      p_circle.y + r_circle * Math.sin(tangentPointAngleOnCircle)
    )
  };
}

export function tangentFromCircleToCircle(posA, radiusA, cwA, posB, radiusB, cwB) {
  const dVec = new Vector2().subtractVectors(posB, posA);
  const d = dVec.length();

  if (d < 1e-9) {
    return {
      a_circle: posA.clone().add(new Vector2(radiusA, 0)),
      b_circle: posB.clone().add(new Vector2(radiusB, 0)),
    };
  }

  cwA = Boolean(cwA);
  cwB = Boolean(cwB);

  const r = (cwA === cwB) ? (radiusB - radiusA) : (radiusA + radiusB);
  const alpha = Math.atan2(dVec.y, dVec.x);
  const phi = Math.asin(Math.max(-1, Math.min(1, r / d)));

  let angleA, angleB;
  if (cwA !== cwB) {
    if (!cwA) {
      angleA = alpha - Math.PI / 2 + phi;
      angleB = alpha + Math.PI / 2 + phi;
    } else {
      angleA = alpha + Math.PI / 2 - phi;
      angleB = alpha - Math.PI / 2 - phi;
    }
  } else if (!cwA) {
    angleA = alpha - Math.PI / 2 - phi;
    angleB = alpha - Math.PI / 2 - phi;
  } else {
    angleA = alpha + Math.PI / 2 + phi;
    angleB = alpha + Math.PI / 2 + phi;
  }

  return {
    a_circle: new Vector2(posA.x + radiusA * Math.cos(angleA), posA.y + radiusA * Math.sin(angleA)),
    b_circle: new Vector2(posB.x + radiusB * Math.cos(angleB), posB.y + radiusB * Math.sin(angleB))
  };
}

export function signedArcLengthOnWheel(prevPoint, currPoint, center, radius, clockwisePreference, force_positive = false) {
  const toPrev = new Vector2().subtractVectors(prevPoint, center);
  const toCurr = new Vector2().subtractVectors(currPoint, center);
  let angle = Math.atan2(toCurr.y, toCurr.x) - Math.atan2(toPrev.y, toPrev.x);
  if (angle > Math.PI) angle -= 2 * Math.PI;
  if (angle < -Math.PI) angle += 2 * Math.PI;
  if (clockwisePreference) angle *= -1;
  if (force_positive) while (angle < 0.0) angle += 2 * Math.PI;
  return radius * angle;
}

export function rightOfLine(x, p0, p1) {
  const vx = p1.x - p0.x, vy = p1.y - p0.y;
  const wx = x.x - p0.x, wy = x.y - p0.y;
  return vx * wy - vy * wx < 0.0;
}
