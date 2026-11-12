// Rendered shape of a cable span: straight when taut, a catenary (or a
// parabolic approximation when nearly vertical) when its rest length exceeds
// the span. Pure math, shared by the renderer and the flight recorder.

const EPSILON = 1e-9;
const CATENARY_HORIZONTAL_EPSILON = 1e-5;
const CATENARY_SOLVER_ITERATIONS = 48;
const CATENARY_MAX_SINH_ARGUMENT = 20;
const PARABOLIC_SAG_SCALE = 0.5;

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

export function writeSlackCablePositions(positions, start, end, cableLength, upDirection, segments) {
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
