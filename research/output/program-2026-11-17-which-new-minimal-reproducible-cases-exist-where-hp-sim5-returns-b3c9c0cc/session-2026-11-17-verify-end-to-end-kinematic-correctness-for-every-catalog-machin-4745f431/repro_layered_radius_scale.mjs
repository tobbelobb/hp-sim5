#!/usr/bin/env node
// F1: every `_w_line_layers` catalog preset moves by the wrong scale because
// the simulator's discrete layer-model spool radius does not equal the RRF
// config's M666 R.
//
// The simulator's winding slope d(stored length)/d(spool angle) is measured
// with the production function `cableStoredLengthAfterRotation`.  The RRF
// config converts line position to motor steps with 3200/(2*pi*R).  If the two
// radii differ, a commanded move is scaled by slope/R.
//
// Also drives a 20 mm +X move through the production RRF -> RrfCanPlayer ->
// headless-world path on the HP4 layered preset and measures the tool tip.
//
// Exits nonzero while the mismatch is present.

import fs from 'node:fs';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..', '..', '..', '..');
const HARNESS = path.join(__dirname, 'kinematics_e2e.mjs');
const TMP_OUT = 'repro-layered-radius.json';
const PY = path.join(ROOT, '.venv', 'bin', 'python');

globalThis.self = { addEventListener() {}, onmessage: null, postMessage() {}, console };

const catalog = JSON.parse(fs.readFileSync(path.join(ROOT, 'machines/catalog.json'), 'utf8'));
const layered = catalog.machines.filter((m) => m.line_layering);

// Build the worlds and measure the production winding slope.
const { createHeadlessWorld } = await import(path.join(ROOT, 'hp-sim-3d/app/headlessWorld.js'));
const ecs = await import(path.join(ROOT, 'src/js/cable_joints_3d/ecs.js'));
const core = await import(path.join(ROOT, 'src/js/cable_joints_3d/cable_joints_core.js'));
const { SpoolTagComponent, SpoolStateComponent } = await import(path.join(ROOT, 'hp-sim-3d/app/hangprinter_spools.js'));

function parseM666R(text) {
  for (const line of text.split(/\r?\n/)) {
    const l = line.split(';')[0].trim();
    if (!l.startsWith('M666')) continue;
    const m = l.slice(4).match(/(?:^|\s)R([-+0-9.eE:]+)/);
    if (m) return m[1].split(':').map(Number);
  }
  return [];
}

const rows = [];
for (const machine of layered) {
  const sceneText = fs.readFileSync(path.join(ROOT, machine.scene), 'utf8');
  const cfgText = fs.readFileSync(path.join(ROOT, machine.rrf_config), 'utf8');
  const radii = parseM666R(cfgText);
  const { world } = createHeadlessWorld(sceneText, { lineLayeringEnabled: true });
  const spools = world.query([SpoolTagComponent, SpoolStateComponent]);
  let maxErr = 0;
  const perAxis = [];
  for (const e of spools) {
    const st = world.getComponent(e, SpoolStateComponent);
    if (!st.axis) continue;
    let link = -1;
    let targetPath = null;
    for (const pe of world.query([core.CablePathComponent])) {
      const c = world.getComponent(pe, core.CablePathComponent);
      for (let li = 0; li < c.linkTypes.length; li++) {
        if (c.linkTypes[li] !== 'hybrid') continue;
        const jIdx = li === c.linkTypes.length - 1 ? c.jointEntities.length - 1 : li;
        const je = c.jointEntities[jIdx];
        if (!je) continue;
        const j = world.getComponent(je, core.CableJointComponent);
        const ent = li === 0 ? j.entityA : j.entityB;
        if (ent === e) { link = li; targetPath = c; break; }
      }
      if (link >= 0) break;
    }
    if (link < 0) continue;
    const ds = core.cableStoredLengthAfterRotation(world, targetPath, link, e, 0.01)
      - targetPath.stored[link];
    const slopeMm = ds / 0.01 * 1000;
    const simRadiusMm = Math.abs(slopeMm);
    const idx = [...'ABCDIJLO'].indexOf(st.axis);
    const cfgR = radii[idx] ?? radii[0];
    const rel = simRadiusMm / cfgR - 1;
    maxErr = Math.max(maxErr, Math.abs(rel));
    perAxis.push({
      axis: st.axis, simRadiusMm: +simRadiusMm.toFixed(4),
      windingSign: Math.sign(slopeMm), rrfRadiusMm: cfgR, relativeError: +rel.toFixed(6),
    });
  }
  rows.push({ machine: machine.name, scene: machine.scene, config: machine.rrf_config, maxRelError: maxErr, perAxis });
  console.log(`${machine.name}`);
  for (const a of perAxis) {
    console.log(`  ${a.axis}: sim radius ${a.simRadiusMm} mm (winding sign ${a.windingSign}), `
      + `RRF M666 R ${a.rrfRadiusMm} mm, scale ${(100 * a.relativeError).toFixed(3)} %`);
  }
}

console.log('\nProduction 20 mm +X move through RRF -> RrfCanPlayer -> headless world:');
const run = execFileSync(process.execPath, [
  HARNESS, '--machines=hp4_w_line_layers', '--move=20', '--axes=X',
  '--settle=10000', '--avg=4000', `--out=${TMP_OUT}`,
], { cwd: __dirname, encoding: 'utf8' });
const measured = JSON.parse(fs.readFileSync(path.join(__dirname, TMP_OUT), 'utf8'));
const axisRow = measured.machines[0].axes[0];
const moveErr = axisRow.measuredMajorMm / 20 - 1;
console.log(`  tip ${axisRow.measuredMajorMm.toFixed(4)} mm for a 20 mm command `
  + `(${(100 * moveErr).toFixed(3)} %)`);

const WINDING_TOL = 0.0025; // 0.25 %
const MOVE_TOL = 0.005;     // 0.5 %
const failures = [];
for (const r of rows) {
  if (r.maxRelError > WINDING_TOL) {
    failures.push(`${r.machine}: sim spool radius differs from RRF M666 R by `
      + `${(100 * r.maxRelError).toFixed(3)} % (tolerance ${100 * WINDING_TOL} %)`);
  }
}
if (Math.abs(moveErr) > MOVE_TOL) {
  failures.push(`HP4 layered 20 mm +X tip error ${(100 * moveErr).toFixed(3)} % `
    + `(tolerance ${100 * MOVE_TOL} %)`);
}

if (failures.length > 0) {
  console.error('\nFAIL: layered spool-radius conversion is not the authored geometry:');
  for (const f of failures) console.error('  ' + f);
  process.exit(1);
}
console.log('\nPASS: layered radius model matches the RRF config');
