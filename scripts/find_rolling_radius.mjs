const EPSILON = 1e-9;
const KNOT_SPAN = Math.PI / 30.0;

function _layerWrapParams(r0, dr, rampLength, layerIndex) {
  const twoPi = 2.0 * Math.PI;
  const rn = r0 + dr * layerIndex;

  let dPhiRamp = 0.0;
  if (rampLength > EPSILON) {
    dPhiRamp = rampLength / (rn + 0.5 * dr);
    if (dPhiRamp > twoPi) dPhiRamp = twoPi;
    if (dPhiRamp < 0.0) dPhiRamp = 0.0;
  }

  const phiConst = twoPi - dPhiRamp;
  const Lconst = rn * phiConst;
  const Lwrap = Lconst + dPhiRamp * (rn + 0.5 * dr);
  return { rn, dPhiRamp, phiConst, Lconst, Lwrap };
}

function _storedToRadiusAndTheta(storedLength, baseRadius, halfWidth, rampLength) {
  const EPS = 1e-9;
  const stored = Math.max(0.0, storedLength ?? 0.0);
  const twoPi = 2.0 * Math.PI;

  const r0 = baseRadius + halfWidth;
  const dr = 2.0 * halfWidth;
  const LrampTarget = Math.max(0.0, rampLength ?? 0.0);

  if (!(r0 > EPS) || !(dr > EPS)) {
    const linearRadius = Number.isFinite(baseRadius) ? Math.max(baseRadius, 0.0) : 0.0;
    const theta = linearRadius > EPS ? (stored / linearRadius) : 0.0;
    return { radius: linearRadius, theta, layer: 0, phi: theta, inRamp: false };
  }

  let s = stored;
  let thetaBase = 0.0;
  let n = 0;

  const MAX_LAYERS = 2048;
  while (n < MAX_LAYERS) {
    const wrap = _layerWrapParams(r0, dr, LrampTarget, n);

    if (s > wrap.Lwrap + EPS) {
      s -= wrap.Lwrap;
      thetaBase += twoPi; // global theta advances exactly one wrap
      n++;
      continue;
    }

    // We are inside wrap/layer n.
    if (s <= wrap.Lconst + EPS || !(wrap.dPhiRamp > EPS)) {
      // Constant-radius region
      const phi = (wrap.rn > EPS) ? (Math.min(wrap.phiConst, s / wrap.rn)) : 0.0;
      return { radius: wrap.rn, theta: thetaBase + phi, layer: n, phi, inRamp: false };
    }

    // Ramp region at end of wrap: sRamp in [0, LrampActual]
    const sRamp = Math.max(0.0, s - wrap.Lconst);

    // Solve sRamp = rn*x + (dr/(2*dPhiRamp))*x^2 for x in [0, dPhiRamp]
    const a = dr / (2.0 * wrap.dPhiRamp); // > 0
    const b = wrap.rn;
    const disc = b * b + 4.0 * a * sRamp; // since c=-sRamp
    const x = (-b + Math.sqrt(Math.max(0.0, disc))) / (2.0 * a);

    const xClamped = Math.max(0.0, Math.min(wrap.dPhiRamp, x));
    const alpha = xClamped / wrap.dPhiRamp;     // 0..1 across ramp
    const radius = wrap.rn + dr * alpha;        // linearly ramps to next layer
    const phi = wrap.phiConst + xClamped;       // near end of wrap

    return { radius, theta: thetaBase + phi, layer: n, phi, inRamp: true };
  }

  // Fallback if MAX_LAYERS exceeded
  const rn = r0 + dr * MAX_LAYERS;
  return { radius: rn, theta: thetaBase, layer: MAX_LAYERS, phi: 0.0, inRamp: false };
}

const stored = 0.942477796076938;
const baseRadius = 0.03;
const halfWidth = 0.001;
const rampLength = baseRadius*KNOT_SPAN;
const {radius, theta, layer, phi, inRamp} = _storedToRadiusAndTheta(stored, baseRadius, halfWidth, rampLength);

console.log(`radius: ${radius}, theta: ${theta}, layer: ${layer}, phi: ${phi}, inRamp: ${inRamp}`);
