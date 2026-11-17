#!/usr/bin/env node
// End-to-end kinematic check for every machine in machines/catalog.json.
//
// Path exercised (the production one):
//   G-code -> RRF firmware (RRF/build/rrf_simulator) -> CAN step capture
//   -> RrfCanPlayer (the same worker the app uses for .can presets)
//   -> cumulative spool angles (steps * 2*pi/3200)
//   -> RemoteSpoolSystem -> production headless world (native solver)
//   -> measured effector displacement.
//
// For every machine and every pure +X/+Y/+Z move of MOVE_MM we report
//   * the firmware's own geometric final position (M114/M400 reply), and
//   * the measured effector displacement in the headless world.
//
// Also checked per machine:
//   * CAN step totals against the RRF config's own closed-form
//     steps/mm = MA * 3200 / (2*pi*R) and its M669 anchors;
//   * authored spool radius vs the RRF M666 R entry;
//   * the production winding model's line-length-vs-angle monotonicity and
//     linearity over a small range.
//
// No source file is modified. Results go to results.json and stdout.
//
// Usage: node kinematics_e2e.mjs [--machines=a,b] [--axes=X,Y,Z]
//                               [--move=5] [--settle=9000] [--avg=3000]

import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..', '..', '..', '..');
const OUT = __dirname;

function arg(name, fallback) {
  const hit = process.argv.find((a) => a.startsWith(`--${name}=`));
  return hit ? hit.slice(name.length + 3) : fallback;
}
const MOVE_MM = Number(arg('move', 5));
const AXES = arg('axes', 'X,Y,Z').split(',').map((s) => s.trim()).filter(Boolean);
const SETTLE = Number(arg('settle', 9000));
const AVG = Number(arg('avg', 3000));
const GRAVITY = arg('gravity', '0') === '1';
const OUTFILE = arg('out', 'results.json');
const ONLY = arg('machines', '') ? new Set(arg('machines', '').split(',')) : null;
const F600 = 'F600';

const STEP_ANGLE_RAD = (2 * Math.PI) / (200 * 16);
const TICKS = 750000;
const SCRATCH = path.join('/tmp/opencode', 'kin-e2e');
fs.mkdirSync(SCRATCH, { recursive: true });

// ---------------------------------------------------------------- RRF runner
function runRrf(configPath, axis, moveMm) {
  const key = `${path.basename(configPath)}|${axis}|${moveMm}`;
  const cacheFile = path.join(SCRATCH, key.replace(/[^A-Za-z0-9_.-]/g, '_') + '.json');
  if (fs.existsSync(cacheFile)) return JSON.parse(fs.readFileSync(cacheFile, 'utf8'));

  const dir = fs.mkdtempSync(path.join(SCRATCH, 'rrf-'));
  const vsd = path.join(dir, 'vsd');
  fs.mkdirSync(path.join(vsd, 'sys'), { recursive: true });
  fs.mkdirSync(path.join(vsd, 'gcodes'), { recursive: true });
  fs.copyFileSync(path.join(ROOT, configPath), path.join(vsd, 'sys', path.basename(configPath)));
  const gcode = [
    'G21', 'G90', 'M83',
    'G92 X0 Y0 Z0',
    `G1 ${axis}${moveMm} ${F600}`,
    'M400',
    '',
  ].join('\n');
  fs.writeFileSync(path.join(vsd, 'gcodes', 'probe.gcode'), gcode);
  const csv = path.join(dir, 'capture.csv');
  const proc = execFileSync(path.join(ROOT, 'RRF', 'build', 'rrf_simulator'), [
    '--vsd', vsd, '--gcode', 'gcodes/probe.gcode', '--can-log', csv,
    '-c', `sys/${path.basename(configPath)}`,
  ], { encoding: 'utf8', timeout: 55000 });

  const rows = [];
  let finalPos = null;
  for (const line of proc.split(/\r?\n/)) {
    const m = line.match(/Final machine position:(.*)/);
    if (m) {
      finalPos = {};
      for (const tok of m[1].trim().split(/\s+/)) {
        const [k, v] = tok.split('=');
        if (k && v !== undefined) finalPos[k] = Number(v);
      }
    }
  }
  if (fs.existsSync(csv)) {
    for (const line of fs.readFileSync(csv, 'utf8').split(/\r?\n/)) {
      const parts = line.split(',');
      if (parts.length < 9) continue;
      const vals = parts.map(Number);
      const numDrives = vals.length - 8;
      rows.push({
        drive: vals[1],
        when: vals[2],
        accel: vals[3],
        steady: vals[4],
        decel: vals[5],
        steps: vals.slice(6, 6 + numDrives),
      });
    }
  }
  const totals = {};
  for (const r of rows) totals[r.drive] = (totals[r.drive] || 0) + r.steps.reduce((a, b) => a + b, 0);
  const result = { configPath, axis, moveMm, gcode, rows, totals, finalPos, csvText: fs.readFileSync(csv, 'utf8') };
  fs.writeFileSync(cacheFile, JSON.stringify(result));
  return result;
}

// ------------------------------------------------------------ player (prod path)
const playerCommands = new Map();
async function playerFor(capture) {
  const key = `${capture.configPath}|${capture.axis}|${capture.moveMm}`;
  if (playerCommands.has(key)) return playerCommands.get(key);
  const { RrfCanPlayer } = await import(path.join(ROOT, 'integrations/rrf/rrfCanPlayer.js'));
  const { FileFormat } = await import(path.join(ROOT, 'integrations/shared/fileFormatUtils.js'));
  const collected = [];
  globalThis.postMessage = (msg) => {
    const c = msg?.command ?? msg;
    if (c && (c.type === 'Move' || c.type === 'Add to reference' || c.type === 'Rebase reference')) collected.push(c);
  };
  const player = new RrfCanPlayer();
  player.asapMode = true;
  player.asapYieldBudgetMs = 1e9;
  const bytes = new TextEncoder().encode(capture.csvText);
  await player.run(new ReadableStream({ start(c) { c.enqueue(bytes); c.close(); } }), FileFormat.RRF_CAN);
  const out = { commands: collected, angles: [...player.axisAngles.entries()] };
  playerCommands.set(key, out);
  return out;
}

// --------------------------------------------------------------- scene/world
let worldMods = null;
async function mods() {
  if (worldMods) return worldMods;
  const { createHeadlessWorld } = await import(path.join(ROOT, 'hp-sim-3d/app/headlessWorld.js'));
  const ecs = await import(path.join(ROOT, 'src/js/cable_joints_3d/ecs.js'));
  const core = await import(path.join(ROOT, 'src/js/cable_joints_3d/cable_joints_core.js'));
  const spoolmod = await import(path.join(ROOT, 'hp-sim-3d/app/hangprinter_spools.js'));
  const { StepperMotorComponent } = await import(path.join(ROOT, 'hp-sim-3d/app/hangprinter_stepper_motor.js'));
  const { ExtruderComponent } = await import(path.join(ROOT, 'hp-sim-3d/app/hangprinter_extruder.js'));
  const Vector3 = (await import(path.join(ROOT, 'src/js/cable_joints_3d/vector3.js'))).default;
  worldMods = { createHeadlessWorld, ecs, core, spoolmod, StepperMotorComponent, ExtruderComponent, Vector3 };
  return worldMods;
}

function settleAndReplay(sceneText, sceneSettings, layering, commands, { settle = SETTLE, avg = AVG, gravity = false } = {}) {
  const { createHeadlessWorld, ecs, core, spoolmod, StepperMotorComponent, ExtruderComponent, Vector3 } = worldMods;
  const { world, remote, dt } = createHeadlessWorld(sceneText, {
    lineLayeringEnabled: layering,
    physicsSubsteps: sceneSettings?.physics_substeps,
    solverIterations: sceneSettings?.solver_iterations,
  });
  if (!gravity) world.setResource('gravity', new Vector3());
  const body = world.query([ecs.RigidBodyComponent])[0];
  for (let i = 0; i < 1500; i++) world.update(dt);
  const p0 = world.getComponent(body, ecs.PositionComponent).pos.clone();
  const q0 = world.getComponent(body, ecs.OrientationComponent).quaternion.clone();
  const extruder = world.query([ExtruderComponent])[0];
  const tipOf = () => {
    const c = extruder !== undefined ? world.getComponent(extruder, ExtruderComponent) : null;
    const t = c?.tipPos;
    return t ? { x: t.x, y: t.y, z: t.z } : null;
  };
  const tip0 = tipOf();

  for (const cmd of commands) {
    remote.commands = [cmd];
    world.update(dt);
  }
  for (let i = 0; i < settle; i++) world.update(dt);

  let sx = 0, sy = 0, sz = 0;
  let tx = 0, ty = 0, tz = 0;
  let minx = Infinity, maxx = -Infinity, miny = Infinity, maxy = -Infinity, minz = Infinity, maxz = -Infinity;
  for (let i = 0; i < avg; i++) {
    world.update(dt);
    const p = world.getComponent(body, ecs.PositionComponent).pos;
    sx += p.x; sy += p.y; sz += p.z;
    minx = Math.min(minx, p.x); maxx = Math.max(maxx, p.x);
    miny = Math.min(miny, p.y); maxy = Math.max(maxy, p.y);
    minz = Math.min(minz, p.z); maxz = Math.max(maxz, p.z);
    const t = tipOf();
    if (t) { tx += t.x; ty += t.y; tz += t.z; }
  }
  const p1 = { x: sx / avg, y: sy / avg, z: sz / avg };
  const tip1 = tip0 ? { x: tx / avg, y: ty / avg, z: tz / avg } : null;
  const spoolAngles = {};
  for (const e of world.query([spoolmod.SpoolTagComponent, spoolmod.SpoolStateComponent])) {
    const st = world.getComponent(e, spoolmod.SpoolStateComponent);
    const m = world.getComponent(e, StepperMotorComponent);
    if (!st.axis) continue;
    const q = world.getComponent(e, ecs.OrientationComponent).quaternion;
    spoolAngles[st.axis] = {
      target: m.commandedAngle - m.deltaAngle,
      actual: spoolmod.getSpoolRotationAngle(st, q),
    };
  }
  return {
    p0: { x: p0.x, y: p0.y, z: p0.z },
    q0: { x: q0.x, y: q0.y, z: q0.z, w: q0.w },
    p1,
    delta_mm: [(p1.x - p0.x) * 1000, (p1.y - p0.y) * 1000, (p1.z - p0.z) * 1000],
    tip0, tip1,
    tip_delta_mm: tip0 ? [(tip1.x - tip0.x) * 1000, (tip1.y - tip0.y) * 1000, (tip1.z - tip0.z) * 1000] : null,
    oscillation_mm: [(maxx - minx) * 1000, (maxy - miny) * 1000, (maxz - minz) * 1000],
    spoolAngles,
  };
}

// ----------------------------------------------------- pure winding geometry
function windingCheck(sceneText, layering) {
  const { createHeadlessWorld, ecs, core, spoolmod } = worldMods;
  const { world } = createHeadlessWorld(sceneText, { lineLayeringEnabled: layering });
  const spools = world.query([spoolmod.SpoolTagComponent, spoolmod.SpoolStateComponent]);
  const out = {};
  for (const e of spools) {
    const st = world.getComponent(e, spoolmod.SpoolStateComponent);
    if (!st.axis) continue;
    const radius = world.getComponent(e, ecs.RadiusComponent)?.radius ?? null;
    const paths = [];
    for (const pe of world.query([core.CablePathComponent])) {
      const c = world.getComponent(pe, core.CablePathComponent);
      let link = -1;
      for (let li = 0; li < c.linkTypes.length; li++) {
        if (c.linkTypes[li] !== 'hybrid') continue;
        const joint = c.jointEntities[li === c.linkTypes.length - 1 ? c.jointEntities.length - 1 : li];
        if (!joint) continue;
        const j = world.getComponent(joint, core.CableJointComponent);
        const ent = li === 0 ? j.entityA : j.entityB;
        if (ent === e) link = li;
      }
      if (link < 0) continue;
      const samples = [];
      for (let a = -0.05; a <= 0.05001; a += 0.01) {
        const before = c.stored[link];
        const after = core.cableStoredLengthAfterRotation(world, c, link, e, a);
        samples.push({ angle: a, delta: after - before });
      }
      const first = samples[0];
      const last = samples[samples.length - 1];
      const monotone = last.delta > first.delta;
      const deltas = samples.map((s) => s.delta);
      const n = samples.length;
      const meanA = samples.reduce((s, x) => s + x.angle, 0) / n;
      const meanD = deltas.reduce((a, b) => a + b, 0) / n;
      let cov = 0, varA = 0;
      for (let i = 0; i < n; i++) {
        cov += (samples[i].angle - meanA) * (deltas[i] - meanD);
        varA += (samples[i].angle - meanA) ** 2;
      }
      const slope = cov / varA;
      let maxDev = 0;
      for (let i = 0; i < n; i++) {
        maxDev = Math.max(maxDev, Math.abs(deltas[i] - (meanD + slope * (samples[i].angle - meanA))));
      }
      paths.push({ path: pe, link, samples, monotone, slope, maxDevFromLinear: maxDev });
    }
    out[st.axis] = { radius, paths };
  }
  return out;
}

// --------------------------------------------- RRF config / step-count audit
function parseConfig(text) {
  const get = (re) => {
    for (const line of text.split(/\r?\n/)) {
      const l = line.split(';')[0].trim();
      const m = l.match(re);
      if (m) return m;
    }
    return null;
  };
  const anchors = {};
  const anchorOrder = [];
  for (const line of text.split(/\r?\n/)) {
    const l = line.split(';')[0].trim();
    if (!/^M669\b/.test(l)) continue;
    const re = /\b([ABCDIJLOK])([-+0-9.eE]+):([-+0-9.eE]+):([-+0-9.eE]+)/g;
    let m;
    while ((m = re.exec(l))) {
      anchors[m[1]] = [Number(m[2]), Number(m[3]), Number(m[4])];
      if (!anchorOrder.includes(m[1])) anchorOrder.push(m[1]);
    }
  }
  const getM666 = (letter) => {
    for (const line of text.split(/\r?\n/)) {
      const l = line.split(';')[0].trim();
      if (!l.startsWith('M666')) continue;
      const m = l.slice(4).match(new RegExp(`(?:^|\\s)${letter}([-+0-9.eE:]+)`));
      if (m) return m[1].split(':').map(Number);
    }
    return null;
  };
  const m584 = get(/\bM584\s+(.*)/);
  return {
    anchors,
    radii: getM666('R') || [],
    ma: getM666('U') || [],
    linesPerSpool: getM666('O') || [],
    parallelLines: getM666('N') || [],
    m584: m584 ? m584[1].trim() : null,
    anchorOrder,
  };
}

function stepAudit(capture, cfg) {
  // Pure geometric line-position change from the firmware's own final pose.
  const letters = cfg.anchorOrder;
  const rows = [];
  for (let i = 0; i < letters.length; i++) {
    const L = letters[i];
    const a = cfg.anchors[L];
    const drive = 40 + i;
    const actual = capture.totals[drive] ?? 0;
    if (!capture.finalPos) { rows.push({ letter: L, drive, actualSteps: actual }); continue; }
    // Machine position vector: the firmware's visible-axis letters X,Y,Z,U,V,W,A,B
    // map to A.. in M669 order only for the anchor letters. Use the reported
    // components by visible axis names.
    const p = [
      capture.finalPos?.X ?? 0,
      capture.finalPos?.Y ?? 0,
      capture.finalPos?.Z ?? 0,
    ];
    // For cubes/skycam extra anchors are in the same XYZ space. The firmware
    // reports only X/Y/Z; U/V/W/A/B are unused for Hangprinter kinematics.
    const d = Math.hypot(p[0] - a[0], p[1] - a[1], p[2] - a[2]);
    const d0 = Math.hypot(a[0], a[1], a[2]);
    const linePos = d - d0;
    const radius = cfg.radii[i];
    const ma = cfg.ma[i] ?? 1;
    const expectedSteps = Math.round(linePos * ma * 3200 / (2 * Math.PI * radius));
    rows.push({
      letter: L, drive, actualSteps: actual, linePosMm: linePos,
      radiusMm: radius, mechanicalAdvantage: ma,
      expectedSteps, errorSteps: actual - expectedSteps,
    });
  }
  return rows;
}

// --------------------------------------------------------------------- main
const catalog = JSON.parse(fs.readFileSync(path.join(ROOT, 'machines/catalog.json'), 'utf8'));
const sceneSettings = catalog.scenes;
const machines = catalog.machines.filter((m) => !ONLY || ONLY.has(m.nicknames[0]) || ONLY.has(m.name));
const results = {
  generated: new Date().toISOString(),
  revisions: {},
  settings: { moveMm: MOVE_MM, axes: AXES, settle: SETTLE, avg: AVG, gravity: GRAVITY },
  machines: [],
  stepAudit: {},
  winding: {},
};
try {
  results.revisions.hp_sim5 = execFileSync('git', ['rev-parse', '--short', 'HEAD'], { cwd: ROOT, encoding: 'utf8' }).trim();
  results.revisions.node = process.version;
} catch {}

globalThis.self = { addEventListener() {}, onmessage: null, postMessage() {}, console };
await mods();

const configTexts = new Map();
function configText(p) {
  if (!configTexts.has(p)) configTexts.set(p, fs.readFileSync(path.join(ROOT, p), 'utf8'));
  return configTexts.get(p);
}

const out = [];
for (const machine of machines) {
  const scenePath = machine.scene;
  const sceneText = fs.readFileSync(path.join(ROOT, scenePath), 'utf8');
  const settings = sceneSettings[scenePath] || {};
  const layering = Boolean(machine.line_layering);
  const cfg = parseConfig(configText(machine.rrf_config));
  const entry = {
    name: machine.name,
    nicknames: machine.nicknames,
    scene: scenePath,
    lineLayering: layering,
    rrfConfig: machine.rrf_config,
    config: cfg,
    axes: [],
  };
  console.log(`\n=== ${machine.name}  [${path.basename(scenePath)}${layering ? ' +layering' : ''}]`);

  // Winding check is scene+layering dependent; cache it.
  const wkey = `${scenePath}|${layering}`;
  if (!results.winding[wkey]) results.winding[wkey] = windingCheck(sceneText, layering);
  entry.winding = results.winding[wkey];

  // Spool radius audit
  const radii = [];
  for (const [axis, info] of Object.entries(entry.winding)) {
    radii.push({ axis, sceneRadiusMm: info.radius * 1000 });
  }
  entry.spoolRadii = radii;

  for (const axis of AXES) {
    const capture = runRrf(machine.rrf_config, axis, MOVE_MM);
    const skeys = `${machine.rrf_config}|${axis}`;
    if (!results.stepAudit[skeys]) results.stepAudit[skeys] = stepAudit(capture, cfg);
    const played = await playerFor(capture);
    const allAnchorsAbove = Object.values(cfg.anchors).length > 0 && Object.values(cfg.anchors).every((a) => a[2] > 0);
    const useGravity = GRAVITY || allAnchorsAbove;
    const world = settleAndReplay(sceneText, settings, layering, played.commands, { gravity: useGravity });
    const [dx, dy, dz] = world.tip_delta_mm || world.delta_mm;
    const measured = axis === 'X' ? dx : axis === 'Y' ? dy : dz;
    const cross = axis === 'X' ? Math.hypot(dy, dz) : axis === 'Y' ? Math.hypot(dx, dz) : Math.hypot(dx, dy);
    // Firmware geometric final position (quantised inverse kinematics).
    const fw = capture.finalPos
      ? [capture.finalPos.X ?? 0, capture.finalPos.Y ?? 0, capture.finalPos.Z ?? 0]
      : null;
    const fwMeasured = fw ? (axis === 'X' ? fw[0] : axis === 'Y' ? fw[1] : fw[2]) : null;
    const fwCross = fw ? (axis === 'X' ? Math.hypot(fw[1], fw[2])
      : axis === 'Y' ? Math.hypot(fw[0], fw[2]) : Math.hypot(fw[0], fw[1])) : null;
    const noStepMotion = Object.values(capture.totals).every((v) => v === 0);
    const fwNoMotion = fw !== null && Math.abs(fwMeasured) < 1e-9;
    const row = {
      axis,
      commandedMm: MOVE_MM,
      noStepMotion,
      fwNoMotion,
      stepTotals: capture.totals,
      playerAngles: Object.fromEntries(played.angles.map(([k, v]) => [k, +v.toFixed(9)])),
      firmwareFinalPosMm: fw,
      firmwareMajorMm: fwMeasured,
      firmwareCrossMm: fwCross,
      measuredDeltaMm: (world.tip_delta_mm || world.delta_mm).map((v) => +v.toFixed(6)),
      bodyDeltaMm: world.delta_mm.map((v) => +v.toFixed(6)),
      gravity: useGravity,
      measuredMajorMm: +measured.toFixed(6),
      measuredCrossMm: +cross.toFixed(6),
      measuredOscillationMm: world.oscillation_mm.map((v) => +v.toFixed(6)),
      spoolAngles: world.spoolAngles,
      errorMm: +(measured - MOVE_MM).toFixed(6),
      errorPct: +((measured - MOVE_MM) / MOVE_MM * 100).toFixed(4),
      spoolAngleErrorRad: Object.fromEntries(Object.entries(world.spoolAngles).map(([k, v]) => [k, +(v.actual - v.target).toExponential(2)])),
    };
    entry.axes.push(row);
    out.push({ machine: machine.name, ...row });
    console.log(`  ${axis}: measured ${measured.toFixed(4)} mm (firmware geometry ${fwMeasured?.toFixed(4)}), `
      + `cross ${cross.toFixed(4)} mm, osc ${Math.max(...world.oscillation_mm).toFixed(4)} mm, `
      + `body ${Math.max(...world.delta_mm.map(Math.abs)).toFixed(3)} mm, `
      + `spool-angle err ${Math.max(...Object.values(row.spoolAngleErrorRad).map(Math.abs)).toExponential(1)} rad, `
      + `steps ${JSON.stringify(capture.totals)}`);
  }
  results.machines.push(entry);
}

// ------------------------------------------------------------------- summary
const tolMajor = Math.max(0.15, 0.015 * MOVE_MM);
const tolCross = Math.max(0.15, 0.02 * MOVE_MM);
const bad = [];
for (const r of out) {
  if (r.noStepMotion || r.fwNoMotion) continue; // planar/no-motion axis, reported separately
  if (Math.abs(r.measuredMajorMm - MOVE_MM) > tolMajor) {
    bad.push({ machine: r.machine, axis: r.axis, kind: 'magnitude', got: r.measuredMajorMm, want: MOVE_MM });
  }
  if (r.measuredCrossMm > tolCross) {
    bad.push({ machine: r.machine, axis: r.axis, kind: 'cross-axis', got: r.measuredCrossMm, tol: tolCross });
  }
  const maxSpoolErr = Math.max(0, ...Object.values(r.spoolAngleErrorRad).map(Math.abs));
  if (maxSpoolErr > 2 * STEP_ANGLE_RAD) {
    bad.push({ machine: r.machine, axis: r.axis, kind: 'spool-angle', got: maxSpoolErr });
  }
}
results.settings.toleranceMajorMm = tolMajor;
results.settings.toleranceCrossMm = tolCross;
results.summary = {
  machines: machines.length,
  axesPerMachine: AXES.length,
  measurements: out.length,
  failures: bad,
  verdict: bad.length === 0 ? 'PASS' : 'FAIL',
};
fs.writeFileSync(path.resolve(OUT, OUTFILE), JSON.stringify(results, null, 2));

console.log('\n================ SUMMARY ================');
for (const r of out) {
  console.log(`${r.machine.padEnd(56)} ${r.axis}  measured ${String(r.measuredMajorMm).padStart(9)} mm  `
    + `cross ${String(r.measuredCrossMm).padStart(8)} mm  fw ${String(r.firmwareMajorMm).padStart(9)} mm`);
}
console.log(`\n${results.summary.verdict}: ${bad.length} failure(s)`);
for (const b of bad) console.log('  FAIL', JSON.stringify(b));
process.exit(bad.length === 0 ? 0 : 1);
