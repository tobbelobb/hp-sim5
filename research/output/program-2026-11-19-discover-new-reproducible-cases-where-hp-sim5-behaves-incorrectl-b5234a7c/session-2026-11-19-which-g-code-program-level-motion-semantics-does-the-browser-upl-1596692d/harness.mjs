// Shared measurement harness for the "G-code program-level motion semantics"
// investigation.
//
// It drives the SHIPPED simulation worker (hp-sim-3d/app/workers/moveCommander.js)
// with a stubbed postMessage, then reconstructs the *physical* trajectory the app
// would execute (each emitted command = one physics step in
// hp-sim-3d/app/remoteSpoolSystem.js:425-449). A small independent G-code
// interpreter (RRF/Klipper semantics, see oracle() below) provides the
// "commanded" side.
//
// Read-only w.r.t. the product tree.

import { pathToFileURL } from 'node:url';

export const REPO = '/home/torbjorn/repos/hp-sim5';
const WORKER = `${REPO}/hp-sim-3d/app/workers/moveCommander.js`;
const KINEMATICS = `${REPO}/hp-sim-3d/app/workers/kinematics.js`;
const REFERENCE = `${REPO}/hp-sim-3d/app/inspectionToolsController.js`;

// Three non-degenerate anchors around the origin (mm). z-offset keeps the
// Jacobian well conditioned at the (0,0,0) start position.
const ANCHORS = [
  [0.0, -2000.0, 1000.0],
  [1689.39746, 975.0, 1000.0],
  [-1689.39746, 975.0, 1000.0],
];
export const SPOOL_R = [30.0, 30.0, 30.0];

export const CONFIG = [
  'M669 A0:-2000:1000 B1689.39746:975:1000 C-1689.39746:975:1000',
  'M666 R30:30:30',
  '',
].join('\n');

// ---------------------------------------------------------------- sim worker

let cached = null;

async function loadWorkerModule() {
  if (cached) return cached;
  globalThis.self = { addEventListener() {} };
  globalThis.postMessage = () => {};
  cached = await import(pathToFileURL(WORKER).href);
  return cached;
}

export async function loadReferenceParser() {
  globalThis.self ??= { addEventListener() {} };
  const mod = await import(pathToFileURL(REFERENCE).href);
  return mod.parseGcodeText;
}

function streamOf(text) {
  const enc = new TextEncoder();
  return new ReadableStream({
    start(c) {
      // Feed in two chunks so the line iterator's buffering path is exercised.
      const bytes = enc.encode(text);
      const mid = Math.floor(bytes.length / 2);
      c.enqueue(bytes.slice(0, mid));
      c.enqueue(bytes.slice(mid));
      c.close();
    },
  });
}

/**
 * Run the shipped MoveCommander over `text`.
 * @returns {commands, moves, belief, finalAnglesRad, extrusionTotal}
 */
export async function runSim(text, { dt = 0.005 } = {}) {
  const { MoveCommander } = await loadWorkerModule();
  const sent = [];
  const savedPost = globalThis.postMessage;
  globalThis.postMessage = (m) => sent.push(m);
  try {
    const mc = new MoveCommander({ uri: null });
    mc.configure(CONFIG);
    mc.dt = dt;
    mc.fastMode = true;
    mc.setSpeedScale(1e9);
    mc.setAsapMode(true);
    await mc.run(streamOf(text));
    const cmds = sent.filter((m) => m && m.action === 'gcode').map((m) => m.command);
    const moves = cmds.filter((c) => c.type === 'Move');
    const lastMove = moves.length ? moves[moves.length - 1] : null;
    const finalAnglesRad = lastMove ? ['A', 'B', 'C'].map((a) => lastMove[a]) : mc.current_angles_rad;
    const extrusionTotal = moves.reduce((s, c) => s + (Number.isFinite(c.E) ? c.E : 0), 0);
    return {
      commands: cmds,
      commandCount: cmds.length,
      moves,
      moveCount: moves.length,
      stepTimeSec: cmds.length * dt,
      dt,
      belief: { ...mc.current_pos_mm },
      finalAnglesRad,
      extrusionTotal,
      errors: sent.filter((m) => m && m.type === 'error'),
    };
  } finally {
    globalThis.postMessage = savedPost;
  }
}

// ------------------------------------------------------- kinematics inversion

let kin = null;
async function loadKin() {
  if (!kin) kin = await import(pathToFileURL(KINEMATICS).href);
  return kin;
}

function motorDeg(pos, k) {
  return k.pos_to_motor_pos_samples_deg(ANCHORS, [pos], 20.0, false, 0.0, SPOOL_R, {})[0];
}

/** Invert absolute motor angles (rad) to effector position (mm). */
export async function anglesToPos(anglesRad, guess = [0, 0, 0]) {
  const k = await loadKin();
  const target = anglesRad.map((a) => (a * 180) / Math.PI);
  let p = guess.slice();
  for (let iter = 0; iter < 60; iter++) {
    const f = motorDeg(p, k);
    const r = f.map((v, i) => v - target[i]);
    const err = Math.max(...r.map(Math.abs));
    if (err < 1e-9) break;
    // numerical Jacobian
    const J = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let j = 0; j < 3; j++) {
      const h = 1e-4;
      const q = p.slice();
      q[j] += h;
      const g = motorDeg(q, k);
      for (let i = 0; i < 3; i++) J[i][j] = (g[i] - f[i]) / h;
    }
    const dp = solve3(J, r.map((v) => -v));
    p = p.map((v, j) => v + dp[j]);
  }
  const res = Math.max(...motorDeg(p, k).map((v, i) => Math.abs(v - target[i])));
  return { pos: p, residualDeg: res };
}

function solve3(A, b) {
  const M = A.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < 3; col++) {
    let piv = col;
    for (let r = col + 1; r < 3; r++) if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    [M[col], M[piv]] = [M[piv], M[col]];
    const d = M[col][col];
    for (let c = col; c < 4; c++) M[col][c] /= d;
    for (let r = 0; r < 3; r++) {
      if (r === col) continue;
      const f = M[r][col];
      for (let c = col; c < 4; c++) M[r][c] -= f * M[col][c];
    }
  }
  return [M[0][3], M[1][3], M[2][3]];
}

/**
 * Physical trajectory reconstructed from the emitted commands.
 * One emitted command = one physics step (remoteSpoolSystem consumes exactly
 * one queued command per step), so path length is the polyline through the
 * inverted per-step effector positions.
 */
export async function simTrajectory(sim) {
  const pts = [];
  let total = 0;
  let prev = [0, 0, 0];
  for (const move of sim.moves) {
    const angles = ['A', 'B', 'C'].map((a) => move[a]);
    const { pos, residualDeg } = await anglesToPos(angles, prev);
    if (residualDeg > 1e-6) throw new Error(`kinematics inversion failed: ${residualDeg} deg`);
    total += Math.hypot(pos[0] - prev[0], pos[1] - prev[1], pos[2] - prev[2]);
    pts.push(pos);
    prev = pos;
  }
  return { pathLength: total, end: prev, points: pts };
}

// ------------------------------------------------------------- G-code oracle
//
// Deliberately small, explicit interpreter implementing the semantics that
// RepRapFirmware and Klipper agree on for the commands under test:
//   G0/G1, G2/G3 (XY arcs), G4, G20/G21, G90/G91, G92, M82/M83,
//   N<line number> and *<checksum> prefixes/suffixes.
// Distance-time model: constant feed (no acceleration), matching what
// moveCommander models, so timing comparisons isolate *semantics*, not
// acceleration planning.
//
// feedDefaults: RRF DefaultFeedRate = 3000 mm/min (Config/Configuration.h:220);
// Klipper GCodeMove.speed = 25 mm/s = 1500 mm/min (gcode_move.py:55).

export function oracle(text, opts = {}) {
  const {
    defaultFeed = 3000.0,
    arcSegmentMm = 0.5,
    defaultAbs = true,
    defaultAbsE = true,
  } = opts;

  const st = {
    pos: [0, 0, 0],
    e: 0,
    absPos: defaultAbs,
    absE: defaultAbsE,
    feed: null, // modal, mm/min
    unit: 1.0, // mm per G-code unit
  };
  const out = {
    travel: 0,
    extrusion: 0,
    time: 0,
    end: [0, 0, 0],
    moves: 0,
    dwellSec: 0,
    unsupported: [],
    log: [],
  };

  const lines = String(text).split(/\r?\n/);
  for (const raw of lines) {
    let line = raw.replace(/\([^)]*\)/g, '');
    const sc = line.indexOf(';');
    if (sc >= 0) line = line.slice(0, sc);
    line = line.trim();
    if (!line) continue;
    // RRF StringParser.cpp:123-141 accepts an N<digits> line number prefix and
    // '*' + checksum suffix (StringParser.cpp:186-192, 272-285).
    line = line.replace(/^[Nn]\s*\d+\s*/, '');
    line = line.replace(/\*\s*\d+\s*$/, '');
    line = line.trim();
    if (!line) continue;
    // RRF/Klipper do not accept several commands on one line; a lone
    // "M82 G1 X10" is a G1 with an unknown parameter -> error, not two commands.
    const tokens = line.split(/\s+/).filter(Boolean);
    const m = /^([GMTgm])(\d+(?:\.\d+)?)$/.exec(tokens[0]);
    if (!m) {
      out.unsupported.push(`${tokens[0]} (unparsed head)`);
      continue;
    }
    const letter = m[1].toUpperCase();
    const num = Math.round(Number(m[2]));

    const args = (names) => {
      const o = {};
      for (const t of tokens.slice(1)) {
        const a = /^([A-Za-z])([-+]?(?:\d+\.?\d*|\.\d+))$/.exec(t);
        if (!a) continue;
        const L = a[1].toUpperCase();
        if (names.includes(L)) o[L] = Number(a[2]);
      }
      return o;
    };

    if (letter === 'M' && (num === 82 || num === 83)) {
      st.absE = num === 82;
      continue;
    }
    if (letter === 'G' && (num === 90 || num === 91)) {
      st.absPos = num === 90;
      continue;
    }
    if (letter === 'G' && num === 20) {
      st.unit = 25.4;
      continue;
    }
    if (letter === 'G' && num === 21) {
      st.unit = 1.0;
      continue;
    }
    if (letter === 'G' && num === 92) {
      const a = args(['X', 'Y', 'Z', 'E']);
      if ('X' in a) st.pos[0] = a.X * st.unit;
      if ('Y' in a) st.pos[1] = a.Y * st.unit;
      if ('Z' in a) st.pos[2] = a.Z * st.unit;
      if ('E' in a) st.e = a.E * st.unit; // RRF scales E by the unit too (GCodes.cpp:1985)
      continue;
    }
    if (letter === 'G' && num === 4) {
      const a = args(['P', 'S']);
      const s = ('P' in a ? a.P / 1000 : 0) + ('S' in a ? a.S : 0);
      out.dwellSec += s;
      out.time += s;
      out.log.push({ cmd: 'G4', dwell: s });
      continue;
    }

    const isMove = letter === 'G' && (num === 0 || num === 1 || num === 2 || num === 3);
    if (!isMove) {
      out.unsupported.push(tokens[0]);
      continue;
    }

    const a = args(['X', 'Y', 'Z', 'E', 'F', 'I', 'J', 'R', 'P']);
    const start = st.pos.slice();
    const next = st.pos.slice();
    for (const [i, L] of ['X', 'Y', 'Z'].entries()) {
      if (L in a) next[i] = st.absPos ? a[L] * st.unit : st.pos[i] + a[L] * st.unit;
    }
    let feed = st.feed;
    if ('F' in a) {
      feed = a.F * st.unit;
      st.feed = feed;
    }
    // RRF (GCodes.cpp:1900-1935) and Klipper (gcode_move.py:147-153) both use the
    // modal feed rate for G0 as well once one is known; only the *initial* default
    // differs (RRF 3000, Klipper 1500 mm/min).
    if (feed == null) feed = defaultFeed;

    let eDelta = 0;
    if ('E' in a) {
      // Extrusion is a distance, so G20 scales it like X/Y/Z:
      // RRF GCodes.cpp:1985 `gb.ConvertDistance(eMovement[0])`.
      const eVal = a.E * st.unit;
      if (st.absE) {
        eDelta = eVal - st.e;
        st.e = eVal;
      } else {
        eDelta = eVal;
        st.e += eVal;
      }
    }

    let pts;
    if (num === 2 || num === 3) {
      pts = arcPoints(start, next, a, num === 2, arcSegmentMm);
    } else {
      pts = [start, next];
    }
    let len = 0;
    for (let i = 1; i < pts.length; i++) len += dist(pts[i - 1], pts[i]);
    const t = feed > 0 ? (len / feed) * 60 : 0;
    out.travel += len;
    out.extrusion += eDelta;
    out.time += t;
    out.moves += 1;
    out.log.push({ cmd: `G${num}`, start, end: next, len, eDelta, feed, t });
    st.pos = next;
  }
  out.end = st.pos;
  return out;
}

function dist(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function arcPoints(start, end, a, clockwise, segMm) {
  // XY-plane arcs (G17), I/J center offsets or R radius; RRF GCodes.cpp:251-277.
  const sx = start[0], sy = start[1];
  const ex = end[0], ey = end[1];
  let cx, cy;
  if ('I' in a || 'J' in a) {
    cx = sx + (a.I ?? 0);
    cy = sy + (a.J ?? 0);
  } else if ('R' in a) {
    const dx = ex - sx, dy = ey - sy;
    const d = Math.hypot(dx, dy);
    const r = Math.abs(a.R);
    if (d === 0 || d > 2 * r) return [start, end];
    const h = Math.sqrt(Math.max(0, r * r - (d / 2) ** 2));
    const mx = (sx + ex) / 2, my = (sy + ey) / 2;
    const ux = -dy / d, uy = dx / d;
    const sgn = (a.R > 0) === clockwise ? -1 : 1;
    cx = mx + sgn * h * ux;
    cy = my + sgn * h * uy;
  } else {
    return [start, end];
  }
  const r0 = Math.hypot(sx - cx, sy - cy);
  const r1 = Math.hypot(ex - cx, ey - cy);
  let a0 = Math.atan2(sy - cy, sx - cx);
  let a1 = Math.atan2(ey - cy, ex - cx);
  const tau = Math.PI * 2;
  if (clockwise) {
    while (a1 >= a0) a1 -= tau;
  } else {
    while (a1 <= a0) a1 += tau;
  }
  const sweep = Math.abs(a1 - a0);
  const n = Math.max(2, Math.ceil((sweep * Math.max(r0, 1e-9)) / segMm));
  const pts = [];
  for (let i = 0; i <= n; i++) {
    const u = i / n;
    const ang = a0 + (a1 - a0) * u;
    const r = r0 + (r1 - r0) * u;
    pts.push([cx + r * Math.cos(ang), cy + r * Math.sin(ang), start[2] + (end[2] - start[2]) * u]);
  }
  pts[0] = start;
  pts[pts.length - 1] = end;
  return pts;
}

// ------------------------------------------------------------------ helpers

export function pct(expected, actual) {
  if (expected === 0) return actual === 0 ? 0 : Infinity;
  return ((actual - expected) / expected) * 100;
}

/** Max distance from any simulated trajectory point to the commanded polyline. */
export function maxPathDeviation(points, polyline) {
  let worst = 0;
  for (const p of points) {
    let best = Infinity;
    for (let i = 1; i < polyline.length; i++) best = Math.min(best, pointSegDist(p, polyline[i - 1], polyline[i]));
    if (polyline.length === 1) best = dist3(p, polyline[0]);
    worst = Math.max(worst, best);
  }
  return worst;
}

function dist3(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function pointSegDist(p, a, b) {
  const ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
  const ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
  const denom = ab[0] ** 2 + ab[1] ** 2 + ab[2] ** 2;
  let u = denom > 0 ? (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / denom : 0;
  u = Math.max(0, Math.min(1, u));
  return dist3(p, [a[0] + u * ab[0], a[1] + u * ab[1], a[2] + u * ab[2]]);
}

/** Commanded polyline (mm) of the oracle's moves. */
export function oraclePolyline(ref) {
  const pts = [];
  for (const m of ref.log) {
    if (!m.start) continue;
    if (!pts.length) pts.push(m.start);
    pts.push(m.end);
  }
  return pts;
}

export function fmt(n, d = 4) {
  return Number(n).toFixed(d);
}

export const fmtPos = (p) => `(${p.map((v) => v.toFixed(3)).join(', ')})`;

/** Aggregate both sides for a given G-code text. */
export async function measure(text, { dt = 0.002, oracleOpts = {} } = {}) {
  const sim = await runSim(text, { dt });
  const traj = await simTrajectory(sim);
  const ref = oracle(text, oracleOpts);
  const poly = oraclePolyline(ref);
  const parseGcodeText = await loadReferenceParser();
  const overlay = overlaySummary(parseGcodeText(text));
  return {
    text,
    dt,
    sim: {
      pathLengthMm: traj.pathLength,
      endMm: traj.end,
      extrusionMm: sim.extrusionTotal,
      steps: sim.commandCount,
      timeSec: sim.stepTimeSec,
      moveCommands: sim.moveCount,
      beliefMm: sim.belief,
      errors: sim.errors.length,
      maxPathDeviationMm: maxPathDeviation(traj.points, poly),
      endErrorMm: poly.length ? dist3(traj.end, ref.end) : 0,
    },
    commanded: {
      travelMm: ref.travel,
      endMm: ref.end,
      extrusionMm: ref.extrusion,
      timeSec: ref.time,
      dwellSec: ref.dwellSec,
      unsupported: ref.unsupported,
      moves: ref.moves,
    },
    overlay,
  };
}

/** What the app's own reference parser/overlay (inspectionToolsController.js) shows. */
export function overlaySummary(parsed) {
  let len = 0;
  let ext = 0;
  for (const s of parsed.segments) {
    len += s.length;
    ext += s.extrusion;
  }
  const last = parsed.segments.length ? parsed.segments[parsed.segments.length - 1] : null;
  return {
    segments: parsed.segments.length,
    travelMm: len,
    extrusionMm: ext,
    endMm: last ? last.end : [0, 0, 0],
  };
}

/** Print a comparison block; returns nothing. */
export function printMeasurement(label, m) {  const { sim, commanded: cmd, overlay } = m;
  const f = (v, d = 3) => (Number.isFinite(v) ? v.toFixed(d) : String(v));
  console.log(`--- ${label} (dt=${m.dt}s)`);
  console.log('    G-code:');
  for (const line of m.text.trimEnd().split('\n')) console.log(`      ${line}`);
  console.log('    commanded (RRF/Klipper semantics)  travel %s mm  extrusion %s mm  time %s s  end %s',
    f(cmd.travelMm), f(cmd.extrusionMm), f(cmd.timeSec), fmtPos(cmd.endMm));
  console.log('    simulated (moveCommander)          travel %s mm  extrusion %s mm  time %s s  end %s',
    f(sim.pathLengthMm), f(sim.extrusionMm), f(sim.timeSec), fmtPos(sim.endMm));
  console.log('    app reference overlay (parseGcodeText)  travel %s mm  extrusion %s mm  end %s  (%d segments)',
    f(overlay.travelMm), f(overlay.extrusionMm), fmtPos(overlay.endMm), overlay.segments);
  console.log('    error: travel %s%%  extrusion %s%%  time %s%%  end error %s mm  max path deviation %s mm  (%d emitted steps)',
    f(pct(cmd.travelMm, sim.pathLengthMm), 1), f(pct(cmd.extrusionMm, sim.extrusionMm), 1),
    f(pct(cmd.timeSec, sim.timeSec), 1), f(sim.endErrorMm), f(sim.maxPathDeviationMm), sim.steps);
}

// ----------------------------------------------------------------- contracts
//
// `expect*` helpers compare the simulated numbers against hand-derived
// values (arithmetic written next to each call), never against the oracle's
// own output, so the assertions stay independent of the oracle implementation.

export function expectClose(failures, label, expected, actual, { rel = 0.01, abs = 0.05 } = {}) {
  const tol = Math.max(Math.abs(expected) * rel, abs);
  const ok = Number.isFinite(actual) && Math.abs(actual - expected) <= tol;
  const line = `${ok ? 'pass' : 'FAIL'}  ${label}: expected ${expected} (±${tol.toPrecision(3)}), got ${Number.isFinite(actual) ? actual.toFixed(4) : actual}`;
  console.log(`  ${line}`);
  if (!ok) failures.push(line);
}

export function expectBelow(failures, label, limit, actual) {
  const ok = Number.isFinite(actual) && actual <= limit;
  const line = `${ok ? 'pass' : 'FAIL'}  ${label}: expected <= ${limit}, got ${Number.isFinite(actual) ? actual.toFixed(4) : actual}`;
  console.log(`  ${line}`);
  if (!ok) failures.push(line);
}

export function finish(title, failures) {
  if (failures.length) {
    console.log(`\nRESULT: BUG PRESENT [${title}] - ${failures.length} contract violation(s)`);
    process.exit(1);
  }
  console.log(`\nRESULT: OK [${title}] - simulated motion matches the commanded motion`);
  process.exit(0);
}
