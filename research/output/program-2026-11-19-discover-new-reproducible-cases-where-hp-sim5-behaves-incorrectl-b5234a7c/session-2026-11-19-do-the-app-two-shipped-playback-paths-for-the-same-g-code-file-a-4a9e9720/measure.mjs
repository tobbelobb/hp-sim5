// Full measurement set: preset (.can via rrfCanPlayer) vs upload (moveCommander)
// for the shipped "Print Squares" G-code on HP4, plus the G6 semantics.
// Writes measurements.json.
import fs from 'node:fs';
import { parseMachine, gcodePolyline, idealLines, nearestOnPolyline, stepsFromLine, STEPS_PER_REV } from './lib/rrfModel.mjs';
import { playCan, playUpload } from './lib/players.mjs';
import { createAccumulator, pathStats, lineError, polylineLength } from './lib/paths.mjs';

const ROOT = new URL('../../../../', import.meta.url);
const read = (p) => fs.readFileSync(new URL(p, ROOT), 'utf8');
const out = { generatedAt: new Date().toISOString(), cases: {}, g6: {} };

const cases = [
  {
    id: 'hp4_no_buildup',
    note: 'control: HP4 without line layering (Q=0, R=30, U=2:2:2:4)',
    can: 'public/RRF_CAN_commands/draw_squares_bigger_hp4_no_buildup.can',
    config: 'RRF/run/vsd/sys/config_hp4.g',
    gcode: 'public/gcode/draw_squares_bigger.gcode',
  },
  {
    id: 'hp4_w_line_layers',
    note: 'HP4 with line layering (Q=0.6366197723675814, R=46.6045)',
    can: 'public/RRF_CAN_commands/draw_squares_bigger_hp4_w_line_layers.can',
    config: 'RRF/run/vsd/sys/config_hp4_w_line_layers.g',
    gcode: 'public/gcode/draw_squares_bigger.gcode',
  },
];

for (const c of cases) {
  const configText = read(c.config);
  const machine = parseMachine(configText);
  const gcodeText = read(c.gcode);
  const polyline = gcodePolyline(gcodeText);
  const na = machine.axes.length;

  const canAcc0 = createAccumulator(machine);
  await playCan(new URL(c.can, ROOT), configText, (cmd) => canAcc0.push(cmd));
  const can = canAcc0.finish();
  const upAcc0 = createAccumulator(machine);
  await playUpload(gcodeText, configText, (cmd) => upAcc0.push(cmd));
  const up = upAcc0.finish();

  const canPath = pathStats(can, polyline);
  const upPath = pathStats(up, polyline);
  const canErr = lineError(machine, can, polyline);
  const upErr = lineError(machine, up, polyline);

  // per G-code segment table
  const segs = polyline.slice(1).map((p, i) => ({
    index: i, from: polyline[i], to: p,
    length: Math.hypot(p[0] - polyline[i][0], p[1] - polyline[i][1], p[2] - polyline[i][2]),
    canMaxDev: 0, canRms: 0, n: 0,
    canLineErrAtEnd: null, canPosAtEnd: null,
  }));
  for (let i = 0; i < can.n; i += 1) {
    const p = Array.from(can.pos.row(i));
    const near = nearestOnPolyline(p, polyline);
    const s = segs[near.seg];
    if (!s) continue;
    s.canMaxDev = Math.max(s.canMaxDev, near.d);
    s.canRms += near.d * near.d;
    s.n += 1;
    if (near.t >= 0.999 || (s.canLineErrAtEnd === null && near.t > 0.98)) {
      const ideal = idealLines(machine, near.q);
      s.canLineErrAtEnd = machine.axes.map((a, k) => can.lines.row(i)[k] - ideal[k]);
      s.canPosAtEnd = near.q;
    }
  }
  for (const s of segs) {
    s.canRms = s.n ? Math.sqrt(s.canRms / s.n) : 0;
    s.canMaxDev = +s.canMaxDev.toFixed(4);
    s.canRms = +s.canRms.toFixed(4);
    if (s.canLineErrAtEnd) s.canLineErrAtEnd = s.canLineErrAtEnd.map((v) => +v.toFixed(4));
  }

  // flex signature: per-axis offset ratio vs D, and the RRF prediction
  const ratios = [];
  for (let i = 0; i < can.n; i += 1) {
    const p = Array.from(can.pos.row(i));
    const near = nearestOnPolyline(p, polyline);
    const ideal = idealLines(machine, near.q);
    const row = can.lines.row(i);
    const offs = machine.axes.map((a, k) => row[k] - ideal[k]);
    if (Math.abs(offs[3]) > 0.02) ratios.push(offs.map((v) => v / offs[3]));
  }
  const meanRatio = [0, 1, 2].map((k) => ratios.reduce((s, r) => s + r[k], 0) / ratios.length);
  const predicted = [0, 1, 2].map((k) => (machine.U[3] ** 2 * (machine.N?.[3] ?? 3)) / (machine.U[k] ** 2 * (machine.N?.[k] ?? 2)));

  const deg = (v) => v * 180 / Math.PI;
  out.cases[c.id] = {
    ...c,
    Q: machine.Q, R: machine.R, U: machine.U, O: machine.O, H: machine.H, L: machine.L,
    flexEnabledByGcode: /^\s*M666\s+F\d/m.test(gcodeText),
    moves: { can: can.n, upload: up.n },
    sumAbsAngleDeg: {
      can: machine.axes.map((a, k) => deg(can.absAngle[k])),
      upload: machine.axes.map((a, k) => deg(up.absAngle[k])),
      relDiffPct: machine.axes.map((a, k) => 100 * (can.absAngle[k] - up.absAngle[k]) / up.absAngle[k]),
    },
    sumAbsLineMm: {
      can: Array.from(can.absLine),
      upload: Array.from(up.absLine),
      relDiffPct: machine.axes.map((a, k) => 100 * (can.absLine[k] - up.absLine[k]) / up.absLine[k]),
    },
    netLineMm: {
      can: machine.axes.map((a, k) => can.lines.row(can.n - 1)[k]),
      upload: machine.axes.map((a, k) => up.lines.row(up.n - 1)[k]),
    },
    pathTravelMm: { can: can.pathTravel, upload: up.pathTravel, gcode: polylineLength(polyline) },
    deviationFromGcodePolyline: {
      can: { max: canPath.maxD, rms: canPath.rms, worstPoint: canPath.worst, worstSegment: canPath.worstSeg },
      upload: { max: upPath.maxD, rms: upPath.rms },
    },
    lineErrorVsIdealMm: {
      can: { max: canErr.max, min: canErr.min, rms: canErr.rms, atWorstPoint: canErr.atWorst },
      upload: { max: upErr.max, min: upErr.min },
    },
    ikResidualMm: {
      can: { max: Math.max(...can.residual), rms: Math.sqrt(can.residual.reduce((s, v) => s + v * v, 0) / can.n) },
      upload: { max: Math.max(...up.residual) },
    },
    flexSignature: {
      meanOffsetRatioVsAxisD: meanRatio.map((v) => +v.toFixed(3)),
      rrfPredictionUD2N: predicted.map((v) => +v.toFixed(3)),
      note: 'RRF flex[i] = -F[i]/(SpringK*U^2*N), so offsets should scale as 1/(U^2*N)',
    },
    perGcodeSegment: segs,
  };
  console.log(c.id, 'ok');
}

// ---- G6 semantics ----------------------------------------------------------
for (const [label, cfgPath] of Object.entries({
  hp4: 'RRF/run/vsd/sys/config_hp4.g',
  hp4_w_line_layers: 'RRF/run/vsd/sys/config_hp4_w_line_layers.g',
})) {
  const configText = read(cfgPath);
  const machine = parseMachine(configText);
  const gcode = read('public/gcode/tighten.gcode');
  const { commands } = await playUpload(gcode, configText);
  const na = machine.axes.length;
  const commanded = new Float64Array(na);
  const delta = new Float64Array(na);
  const t0 = machine.axes.map((_, k) => commanded[k] - delta[k]);
  const peak = t0.slice();
  const noRebaseEnd = new Float64Array(na);
  for (const cmd of commands) {
    if (cmd.type === 'Move') {
      for (let k = 0; k < na; k += 1) if (Number.isFinite(cmd[machine.axes[k]])) commanded[k] = cmd[machine.axes[k]];
    } else if (cmd.type === 'Add to reference') {
      for (let k = 0; k < na; k += 1) if (Number.isFinite(cmd[machine.axes[k]])) delta[k] += cmd[machine.axes[k]];
    }
    for (let k = 0; k < na; k += 1) {
      const v = commanded[k] - delta[k];
      if (Math.abs(v - t0[k]) > Math.abs(peak[k] - t0[k])) peak[k] = v;
      noRebaseEnd[k] = v;
    }
  }
  // variant: a 'Rebase reference' before the stream (external bridge only);
  // the first Move per axis then only re-origins the command, remoteSpoolSystem.js:271-276
  const rc = new Float64Array(na);
  const rd = new Float64Array(na);
  const rebased = new Set();
  for (const cmd of commands) {
    if (cmd.type === 'Move') {
      for (let k = 0; k < na; k += 1) {
        const v = cmd[machine.axes[k]];
        if (!Number.isFinite(v)) continue;
        if (!rebased.has(k)) { rd[k] += v - rc[k]; rc[k] = v; rebased.add(k); continue; }
        rc[k] = v;
      }
    } else if (cmd.type === 'Add to reference') {
      for (let k = 0; k < na; k += 1) if (Number.isFinite(cmd[machine.axes[k]])) rd[k] += cmd[machine.axes[k]];
    }
  }
  const withRebase = machine.axes.map((_, k) => rc[k] - rd[k] - t0[k]);
  const intendedLine = [-42, -42, -42, 0];
  out.g6[label] = {
    gcode: gcode.trim(),
    commands: { Move: commands.filter((c) => c.type === 'Move').length, AddToReference: commands.filter((c) => c.type === 'Add to reference').length },
    dtVariants: {},
    perAxis: machine.axes.map((a, k) => {
      const exactDeg = stepsFromLine(machine, k, intendedLine[k]) * 2 * Math.PI / STEPS_PER_REV * 180 / Math.PI;
      const linearDeg = intendedLine[k] / machine.R[k] * machine.U[k] * machine.H[k] / machine.L[k] * 180 / Math.PI;
      return {
        axis: a,
        intendedLineMm: intendedLine[k],
        exactAngleDeg: exactDeg,
        workerLinearisedAngleDeg: linearDeg,
        linearisationErrorDeg: linearDeg - exactDeg,
        linearisationErrorLineMm: (linearDeg - exactDeg) * Math.PI / 180 * machine.R[k] / machine.U[k],
        netTargetChangeDeg: (noRebaseEnd[k] - t0[k]) * 180 / Math.PI,
        netTargetChangeDegWithStreamRebase: withRebase[k] * 180 / Math.PI,
        peakExcursionDeg: (peak[k] - t0[k]) * 180 / Math.PI,
      };
    }),
  };
  // dt variants: MoveCommander.dt defaults to the 5 ms Python fallback, but the
  // app sets it from state.simDtSec (hp4.usda timeCodesPerSecond=500 -> 2 ms).
  for (const dt of [0.005, 0.002]) {
    const { MoveCommander } = await import(new URL('hp-sim-3d/app/workers/moveCommander.js', ROOT));
    const c2 = new MoveCommander();
    c2.configure(configText);
    c2.setAsapMode(true);
    c2.dt = dt;
    const out2 = [];
    c2.sendCommand = async (x) => { out2.push(x); };
    await c2.run(new ReadableStream({ start(ctl) { ctl.enqueue(new TextEncoder().encode(gcode)); ctl.close(); } }));
    const cm = new Float64Array(na), de = new Float64Array(na);
    const t00 = machine.axes.map(() => 0);
    const pk = t00.slice();
    for (const cmd of out2) {
      if (cmd.type === 'Move') for (let k = 0; k < na; k += 1) if (Number.isFinite(cmd[machine.axes[k]])) cm[k] = cmd[machine.axes[k]];
      if (cmd.type === 'Add to reference') for (let k = 0; k < na; k += 1) if (Number.isFinite(cmd[machine.axes[k]])) de[k] += cmd[machine.axes[k]];
      for (let k = 0; k < na; k += 1) {
        const v = cm[k] - de[k];
        if (Math.abs(v) > Math.abs(pk[k])) pk[k] = v;
      }
    }
    out.g6[label].dtVariants[String(dt)] = {
      moves: out2.filter((x) => x.type === 'Move').length,
      addToReference: out2.filter((x) => x.type === 'Add to reference').length,
      netTargetChangeDeg: machine.axes.map((a, k) => (cm[k] - de[k]) * 180 / Math.PI),
      peakExcursionDeg: machine.axes.map((a, k) => pk[k] * 180 / Math.PI),
    };
  }
  console.log('g6', label, 'ok', JSON.stringify(Object.fromEntries(
    Object.entries(out.g6[label].dtVariants).map(([dt, v]) => [dt, v.moves]))));
}

// ---- G92 X<nonzero> --------------------------------------------------------
{
  const configText = read('RRF/run/vsd/sys/config_hp4.g');
  const machine = parseMachine(configText);
  const { commands } = await playUpload('G92 X5\n', configText);
  const na = machine.axes.length;
  const add = Object.fromEntries(machine.axes.map((a) => [a, 0]));
  for (const cmd of commands) if (cmd.type === 'Add to reference') {
    for (let k = 0; k < na; k += 1) if (Number.isFinite(cmd[machine.axes[k]])) add[machine.axes[k]] = cmd[machine.axes[k]];
  }
  const jump = Object.fromEntries(machine.axes.map((a, k) => [a, -add[a]]));
  out.g92 = {
    gcode: 'G92 X5',
    commands: commands.map((c) => c.type),
    addToReferenceRad: add,
    targetJumpRad: jump,
    targetJumpLineMm: Object.fromEntries(machine.axes.map((a, k) =>
      [a, jump[a] * machine.R[k] / machine.U[k]])),
    note: 'target = commandedAngle - deltaAngle (engine.rs:1568), so the jump is -Add; it must be 0.',
    worldMeasurement: {
      source: 'probe_world.mjs / repro_g92_moves_tool.mjs',
      targetJumpDeg: { A: -0.0249, B: 16.3565, C: -16.3697, D: -0.0503 },
      motorRotationDegOpenLoop: { A: -0.0249, B: 9.1565, C: -1.9697, D: -0.0503 },
    },
    reviewEndToEndToolMm: {
      source: 'independent review, same headless HP4 world',
      terminalG92X5: { closedLoop: -5.000, openLoop: -0.606, expected: 0 },
      g92X5ThenG1X10: { closedLoop: 4.983, openLoop: 0.538, expectedPlain: 4.998 },
      note: 'size is motor-mode dependent; HP4 ships open loop (7/5 missed steps in review)',
    },
  };
  console.log('g92 ok');
}

// pin every input and source file by content hash
import crypto from 'node:crypto';
const hashOf = (rel) => crypto.createHash('sha256').update(fs.readFileSync(new URL(rel, ROOT))).digest('hex');
out.inputs = {};
for (const rel of [
  'public/RRF_CAN_commands/draw_squares_bigger_hp4_no_buildup.can',
  'public/RRF_CAN_commands/draw_squares_bigger_hp4_w_line_layers.can',
  'public/gcode/draw_squares_bigger.gcode',
  'public/gcode/tighten.gcode',
  'RRF/run/vsd/sys/config_hp4.g',
  'RRF/run/vsd/sys/config_hp4_w_line_layers.g',
  'integrations/rrf/rrfCanPlayer.js',
  'integrations/rrf/rrfFirmwareModel.js',
  'integrations/shared/motionUtils.js',
  'hp-sim-3d/app/workers/moveCommander.js',
  'hp-sim-3d/app/workers/kinematics.js',
  'hp-sim-3d/app/remoteSpoolSystem.js',
  'src/wasm/cable_solver/src/engine.rs',
  'RRF/ReprapFirmware/src/Movement/Kinematics/HangprinterKinematics.cpp',
]) out.inputs[rel] = hashOf(rel);
out.revision = fs.existsSync(new URL('.git/HEAD', ROOT)) ? 'see report.md (56a66f77)' : null;
fs.writeFileSync(new URL('./measurements.json', import.meta.url), JSON.stringify(out, null, 2));
console.log('wrote measurements.json');
