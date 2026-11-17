#!/usr/bin/env node
// Helper for repro/f3_impact_slideprinter.py: run the real
// KlipperMcuCommandPlayer on a Klipper MCU stream and print the exact vs
// player-delivered cumulative motor steps as JSON.
//
//   node repro/f3_player_totals.mjs <stream.txt> [prefixFraction]
//
// `prefixFraction` (0..1) truncates the stream after that fraction of its
// queue_step lines (still a valid stream) to check whether the loss is a
// one-time offset or grows during the print.

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

globalThis.postMessage = () => {};
globalThis.self = {
  addEventListener() {}, onmessage: null,
  postMessage: globalThis.postMessage, console,
};

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..', '..', '..', '..', '..');
const { FileFormat } = await import(
  path.join(ROOT, 'integrations', 'shared', 'fileFormatUtils.js'));
const { KlipperMcuCommandPlayer } = await import(
  path.join(ROOT, 'integrations', 'klipper', 'klipperMcuCommandPlayer.js'));
const { STEP_ANGLE_RAD, EXTRUDER_MM_PER_STEP_KLIPPER } = await import(
  path.join(ROOT, 'integrations', 'klipper', 'klipperFirmwareModel.js'));

const streamPath = path.resolve(process.argv[2]);
const fraction = Number(process.argv[3] ?? '1');
let text = fs.readFileSync(streamPath, 'utf8');

if (fraction < 1) {
  const lines = text.split('\n');
  const idx = lines.map((l, i) => (l.trim().startsWith('queue_step') ? i : -1)).filter((i) => i >= 0);
  const cut = idx[Math.max(0, Math.ceil(idx.length * fraction) - 1)];
  text = lines.slice(0, cut + 1).join('\n');
}

function exactTotals(textIn) {
  const dir = {}, steps = {};
  for (const raw of textIn.split('\n')) {
    const line = raw.trim();
    let m = line.match(/set_next_step_dir oid=(\d+) dir=(\d+)/);
    if (m) { dir[+m[1]] = m[2] === '1' ? 1 : -1; continue; }
    m = line.match(/queue_step oid=(\d+) interval=(-?\d+) count=(-?\d+) add=(-?\d+)/);
    if (m) steps[+m[1]] = (steps[+m[1]] || 0) + (dir[+m[1]] ?? 1) * (+m[3]);
  }
  return steps;
}

const player = new KlipperMcuCommandPlayer();
player.setAsapMode(true);
const commands = [];
player.sendCommand = async (c) => { commands.push(c); };
await player.run(new ReadableStream({
  start(c) { c.enqueue(new TextEncoder().encode(text)); c.close(); },
}), FileFormat.MCU_TEXT);

const moves = commands.filter((c) => c.type === 'Move');
const last = moves.at(-1) || {};
let extrusionMm = 0;
for (const m of moves) extrusionMm += (m.E || 0);

const exact = exactTotals(text);
const result = { exact_steps: {}, player_steps: {}, deficit_steps: {}, extrusion_mm: extrusionMm };
for (const [oid, axis] of player.axisByOid) {
  const value = exact[+oid] || 0;
  result.exact_steps[axis] = value;
  if (axis === 'E') {
    result.deficit_steps.E = value - extrusionMm / EXTRUDER_MM_PER_STEP_KLIPPER;
  } else {
    const playerSteps = (last[axis] || 0) / STEP_ANGLE_RAD;
    result.player_steps[axis] = playerSteps;
    result.deficit_steps[axis] = value - playerSteps;
  }
}
// Extruder as steps too, for completeness.
result.exact_extrusion_steps = Object.fromEntries(
  Object.entries(exact).filter(([oid]) => player.axisByOid.get(Number(oid)) === 'E'));
console.log(JSON.stringify(result));
