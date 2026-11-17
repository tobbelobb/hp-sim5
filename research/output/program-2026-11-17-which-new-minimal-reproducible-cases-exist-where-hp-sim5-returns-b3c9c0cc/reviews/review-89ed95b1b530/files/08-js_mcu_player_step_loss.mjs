globalThis.postMessage = () => {};
globalThis.self = { addEventListener() {}, onmessage: null, postMessage: globalThis.postMessage, console };
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..', '..', '..', '..', '..');
const { FileFormat } = await import(
  path.join(ROOT, 'integrations', 'shared', 'fileFormatUtils.js'));
const { KlipperMcuCommandPlayer } = await import(
  path.join(ROOT, 'integrations', 'klipper', 'klipperMcuCommandPlayer.js'));
const { STEP_ANGLE_RAD, EXTRUDER_MM_PER_STEP_KLIPPER } = await import(
  path.join(ROOT, 'integrations', 'klipper', 'klipperFirmwareModel.js'));

const file = process.argv[2];
const text = fs.readFileSync(file, 'utf8');

function truth(text) {
  const dir = {}, steps = {};
  for (const raw of text.split('\n')) {
    const line = raw.trim();
    let m = line.match(/set_next_step_dir oid=(\d+) dir=(\d+)/);
    if (m) { dir[+m[1]] = m[2] === '1' ? 1 : -1; continue; }
    m = line.match(/queue_step oid=(\d+) interval=(-?\d+) count=(-?\d+) add=(-?\d+)/);
    if (m) steps[+m[1]] = (steps[+m[1]] || 0) + (dir[+m[1]] ?? 1) * (+m[3]);
  }
  return steps;
}

const p = new KlipperMcuCommandPlayer();
p.setAsapMode(true);
const cmds = [];
p.sendCommand = async (c) => { cmds.push(c); };
await p.run(new ReadableStream({
  start(c) { c.enqueue(new TextEncoder().encode(text)); c.close(); },
}), FileFormat.MCU_TEXT);

const moves = cmds.filter((c) => c.type === 'Move');
const last = moves.at(-1) || {};
const playerSteps = {};
for (const axis of ['A', 'B', 'C', 'D', 'I', 'J', 'L', 'O']) {
  playerSteps[axis] = (last[axis] || 0) / STEP_ANGLE_RAD;
}
let extrusion = 0;
for (const m of moves) extrusion += (m.E || 0);

const t = truth(text);
// Use the player's own oid -> axis map (parsed from config_stepper step pins).
const truthByAxis = {};
for (const [oid, value] of Object.entries(t)) {
  const axis = p.axisByOid.get(Number(oid));
  if (!axis) continue;
  if (axis === 'E') {
    truthByAxis.E_extrusionSteps = (truthByAxis.E_extrusionSteps || 0) + value;
  } else {
    truthByAxis[axis] = (truthByAxis[axis] || 0) + value;
  }
}

console.log(JSON.stringify({
  file: path.relative(ROOT, file),
  stream_steps_by_oid: t,
  oid_to_axis_used_by_player: Object.fromEntries(p.axisByOid),
  player_final_steps: Object.fromEntries(Object.entries(playerSteps).map(([k, v]) => [k, +v.toFixed(4)])),
  player_extrusion_steps: +(extrusion / EXTRUDER_MM_PER_STEP_KLIPPER).toFixed(4),
  move_commands: moves.length,
}, null, 2));

const problems = [];
for (const axis of Object.keys(truthByAxis)) {
  if (axis === 'E_extrusionSteps') continue;
  const want = truthByAxis[axis];
  const got = playerSteps[axis] || 0;
  if (Math.abs(got - want) > 0.05) {
    problems.push(`${axis}: player ends at ${got.toFixed(3)} steps, stream contains ${want} steps (lost ${(want - got).toFixed(3)})`);
  }
}
if (truthByAxis.E_extrusionSteps !== undefined) {
  const got = extrusion / EXTRUDER_MM_PER_STEP_KLIPPER;
  const want = truthByAxis.E_extrusionSteps;
  if (Math.abs(got - want) > 0.5) {
    problems.push(`E: player extrudes ${got.toFixed(2)} steps, stream contains ${want} steps (lost ${(want - got).toFixed(2)})`);
  }
}

if (problems.length) {
  console.error('\nSTEP LOSS REPRODUCED on shipped MCU-command preset:');
  for (const p of problems) console.error('  - ' + p);
  process.exit(1);
}
console.log('OK: player replays every step in the stream');
