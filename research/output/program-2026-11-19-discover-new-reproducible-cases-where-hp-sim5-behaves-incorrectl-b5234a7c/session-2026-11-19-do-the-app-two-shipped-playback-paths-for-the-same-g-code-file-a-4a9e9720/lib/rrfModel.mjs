// Exact re-implementation of the RepRapFirmware Hangprinter line<->motor map
// that both shipped JS paths use, plus a least-squares effector reconstruction.
//
// RRF source of truth:
//   RRF/ReprapFirmware/src/Movement/Kinematics/HangprinterKinematics.cpp
//     Recalc():  spr[i] = U*fullSteps*microsteps*H / (2*pi*L)
//                k2[i]  = -(U*O)*Q
//     CartesianToMotorSteps(): motorPos = line*(k0*k2)/(sqrt(R^2+line*k2)+R)
//                with k0*k2 = 2*spr
//     MotorPosToLinePos():     inverse of the above
// hp-sim forward model: hp-sim-3d/app/workers/kinematics.js
//   pos_to_motor_pos_samples_deg() is the same equation in degrees.

export const FULL_STEPS = 200;
export const MICROSTEPS = 16;
export const STEPS_PER_REV = FULL_STEPS * MICROSTEPS;

// Same parse as MoveCommander.configure() (moveCommander.js:59-83), plus the
// extra RRF parameters used by the firmware but ignored by the upload path.
export function parseMachine(configText) {
  const anchors = new Map();
  const params = {};
  let flexEnabled = null;
  for (const raw of configText.split(/\r?\n/)) {
    const line = raw.split(';')[0];
    if (/^M669\b/.test(line)) {
      for (const m of line.matchAll(/\b([ABCDIJKLO])([-+\d.]+:[-+\d.]+:[-+\d.]+)/g)) {
        anchors.set(m[1], m[2].split(':').map(Number));
      }
    }
    if (/^M666\b/.test(line)) {
      for (const m of line.matchAll(/\b([QRULHO])([-+\d.:]+)/g)) {
        params[m[1]] = m[2].split(':').map(Number);
      }
      const f = /\bF(-?\d+)\b/.exec(line);
      if (f) flexEnabled = Number(f[1]) !== 0;
    }
  }
  const axes = [...'ABCDIJKLO'].filter((a) => anchors.has(a));
  const n = axes.length;
  const pick = (key, fallback) => axes.map((_, i) => params[key]?.[i] ?? fallback);
  const machine = {
    axes,
    anchors: axes.map((a) => anchors.get(a)),
    Q: params.Q?.[0] ?? 0,
    R: pick('R', 30),
    U: pick('U', 1),
    O: pick('O', 1),
    L: pick('L', 1),
    H: pick('H', 1),
    flexEnabled,
    spr: axes.map((_, i) => {
      const H = params.H?.[i] ?? 1;
      const L = params.L?.[i] ?? 1;
      const U = params.U?.[i] ?? 1;
      return (U * STEPS_PER_REV * H) / (2 * Math.PI * L); // steps per radian
    }),
  };
  machine.k2 = axes.map((_, i) => -(machine.U[i] * machine.O[i]) * machine.Q);
  machine.useConstantSpool = machine.k2.map((k) => Math.abs(k) <= 1e-9);
  machine.distancesOrigin = machine.anchors.map((a) => Math.hypot(a[0], a[1], a[2]));
  return machine;
}

export function lineFromSteps(machine, i, steps) {
  if (machine.useConstantSpool[i]) {
    return steps / (machine.spr[i] / machine.R[i]);
  }
  const k2 = machine.k2[i];
  const R = machine.R[i];
  const deltaRadius = steps * k2 / (2 * machine.spr[i]);
  return (deltaRadius * (2 * R + deltaRadius)) / k2;
}

export function stepsFromLine(machine, i, line) {
  if (machine.useConstantSpool[i]) {
    return line * (machine.spr[i] / machine.R[i]);
  }
  const k2 = machine.k2[i];
  const R = machine.R[i];
  return (line * (2 * machine.spr[i])) / (Math.sqrt(R * R + line * k2) + R);
}

export const stepsFromAngleRad = (angle) => (angle * STEPS_PER_REV) / (2 * Math.PI);
export const angleRadFromSteps = (steps) => (steps * 2 * Math.PI) / STEPS_PER_REV;

export const linesFromAngles = (machine, angles) =>
  machine.axes.map((a, i) => lineFromSteps(machine, i, stepsFromAngleRad(angles[a] ?? 0)));

// Ideal (uncompensated) line positions of a cartesian point, relative to the
// homed position at the origin: line_i = |anchor_i - p| - |anchor_i|.
export function idealLines(machine, p) {
  return machine.anchors.map((a, i) =>
    Math.hypot(a[0] - p[0], a[1] - p[1], a[2] - p[2]) - machine.distancesOrigin[i]);
}

/** 4x3 over-determined Gauss-Newton inverse: line positions -> effector position. */
export function positionFromLines(machine, line, guess = [0, 0, 0], iterations = 40) {
  const A = machine.anchors;
  const n = A.length;
  let p = guess.slice();
  const resid = new Array(n);
  const J = [];
  for (let i = 0; i < n; i += 1) J.push([0, 0, 0]);
  for (let it = 0; it < iterations; it += 1) {
    let cost = 0;
    for (let i = 0; i < n; i += 1) {
      const dx = p[0] - A[i][0];
      const dy = p[1] - A[i][1];
      const dz = p[2] - A[i][2];
      const d = Math.hypot(dx, dy, dz);
      resid[i] = d - machine.distancesOrigin[i] - line[i];
      cost += resid[i] * resid[i];
      const inv = 1 / d;
      J[i][0] = dx * inv;
      J[i][1] = dy * inv;
      J[i][2] = dz * inv;
    }
    if (cost < 1e-20) break;
    // normal equations (J^T J + lambda I) dp = -J^T r
    const lambda = 1e-9;
    const N = [[lambda, 0, 0], [0, lambda, 0], [0, 0, lambda]];
    const g = [0, 0, 0];
    for (let i = 0; i < n; i += 1) {
      for (let r = 0; r < 3; r += 1) {
        g[r] += J[i][r] * resid[i];
        for (let c = 0; c < 3; c += 1) N[r][c] += J[i][r] * J[i][c];
      }
    }
    const dp = solve3(N, [-g[0], -g[1], -g[2]]);
    p = [p[0] + dp[0], p[1] + dp[1], p[2] + dp[2]];
  }
  let rms = 0;
  for (let i = 0; i < n; i += 1) {
    const d = Math.hypot(p[0] - A[i][0], p[1] - A[i][1], p[2] - A[i][2])
      - machine.distancesOrigin[i] - line[i];
    rms += d * d;
  }
  return { p, residualRms: Math.sqrt(rms / n) };
}

function solve3(M, b) {
  const m = [M[0].slice(), M[1].slice(), M[2].slice()];
  const v = b.slice();
  for (let c = 0; c < 3; c += 1) {
    let piv = c;
    for (let r = c + 1; r < 3; r += 1) if (Math.abs(m[r][c]) > Math.abs(m[piv][c])) piv = r;
    [m[c], m[piv]] = [m[piv], m[c]];
    [v[c], v[piv]] = [v[piv], v[c]];
    for (let r = c + 1; r < 3; r += 1) {
      const f = m[r][c] / m[c][c];
      for (let k = c; k < 3; k += 1) m[r][k] -= f * m[c][k];
      v[r] -= f * v[c];
    }
  }
  const x = [0, 0, 0];
  for (let r = 2; r >= 0; r -= 1) {
    let s = v[r];
    for (let k = r + 1; k < 3; k += 1) s -= m[r][k] * x[k];
    x[r] = s / m[r][r];
  }
  return x;
}

/** G-code position polyline of a file, in the order the upload path walks it. */
export function gcodePolyline(gcodeText) {
  const pts = [];
  let pos = [0, 0, 0];
  pts.push(pos.slice());
  for (const raw of gcodeText.split(/\r?\n/)) {
    const line = raw.split(';', 1)[0].trim();
    const code = line.split(/\s+/, 1)[0].toUpperCase();
    if (code !== 'G1') {
      if (code === 'G92') {
        const parts = line.split(/\s+/);
        const next = pos.slice();
        for (const part of parts) {
          const c = part.charAt(0).toUpperCase();
          const v = parseFloat(part.slice(1));
          if (Number.isFinite(v) && (c === 'X' || c === 'Y' || c === 'Z')) next['XYZ'.indexOf(c)] = v;
        }
        pos = next;
        pts.push(pos.slice());
      }
      continue;
    }
    const parts = line.split(/\s+/);
    const next = pos.slice();
    let moved = false;
    for (const part of parts) {
      const c = part.charAt(0).toUpperCase();
      const v = parseFloat(part.slice(1));
      if (Number.isFinite(v) && (c === 'X' || c === 'Y' || c === 'Z')) {
        next['XYZ'.indexOf(c)] = v;
        moved = true;
      }
    }
    if (moved) {
      pos = next;
      pts.push(pos.slice());
    }
  }
  return pts;
}

/** Distance from p to segment [a,b]; also returns the fraction along the segment. */
export function pointSegmentDistance(p, a, b) {
  const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
  const ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
  const len2 = ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2;
  let t = len2 > 0 ? (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / len2 : 0;
  t = Math.max(0, Math.min(1, t));
  const q = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
  return { d: Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]), t, q };
}

export function nearestOnPolyline(p, pts) {
  let best = { d: Infinity, seg: -1, t: 0, q: null };
  for (let i = 0; i + 1 < pts.length; i += 1) {
    const r = pointSegmentDistance(p, pts[i], pts[i + 1]);
    if (r.d < best.d) best = { ...r, seg: i };
  }
  return best;
}
