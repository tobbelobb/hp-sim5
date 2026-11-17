#!/usr/bin/env node
// Follow-up F2: does the binary `.can` path handle a two-driver CAN board?
//
// Two questions:
//  1. Does the shipped CSV -> .can converter (RRF/csv2can.py) preserve a
//     multi-driver capture row?  (It reads fixed columns like the CSV player.)
//  2. If a *correct* .can is produced by hand -- two body records in the same
//     context block, one per drive, as CanCapture's `numDrivers` implies --
//     can the binary decoder distinguish the two drives?
//
// The `.can` schema (documented at the top of RRF/csv2can.py) stores one
// record per body item as `CAN ID (6 bits) + value`; the CAN ID is the CAN
// *destination address* (40), not the local driver index, so a correct .can
// still cannot tell driver 0 from driver 1.
//
// Repro: node repro/f2_binary_path_probe.mjs   (exits 1 while this is broken)

import { execFileSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

globalThis.postMessage = () => {};
globalThis.self = { addEventListener() {}, onmessage: null, postMessage: globalThis.postMessage, console };

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(__dirname, '..', '..', '..', '..', '..');
const { RrfCanPlayer } = await import(path.join(ROOT, 'integrations/rrf/rrfCanPlayer.js'));
const { parseRrfMotorAxisMapFromConfigText, STEP_ANGLE_RAD } = await import(
  path.join(ROOT, 'integrations/rrf/rrfFirmwareModel.js'));
const { FileFormat } = await import(path.join(ROOT, 'integrations/shared/fileFormatUtils.js'));

const fixtureDir = path.join(__dirname, 'fixtures');
const csvPath = path.join(fixtureDir, 'rrf_two_drivers_one_board.csv');
const cfgPath = path.join(fixtureDir, 'rrf_two_drivers_one_board_config.g');
const workDir = '/tmp/opencode/f2_binary_probe';
fs.mkdirSync(workDir, { recursive: true });
const workCsv = path.join(workDir, 'two_drivers.csv');
fs.copyFileSync(csvPath, workCsv);

// 1. shipped converter
execFileSync(path.join(ROOT, '.venv', 'bin', 'python'),
  [path.join(ROOT, 'RRF/csv2can.py'), workCsv], { encoding: 'utf8' });
const canPath = workCsv.replace(/\.csv$/, '.can');
const converted = fs.readFileSync(canPath);

// 2. a hand-built "ideal" .can: same context, two records in one block, both
//    carrying the CAN destination address 40, values 800 (X) and 400 (Y).
//    Header mask=0 (nothing changed), count=2, then int16 items.
function idealCan() {
  const header = Buffer.from([0x00, 0x02]);
  const item1 = Buffer.alloc(3); item1[0] = (1 << 6) | 40; item1.writeInt16LE(800, 1);
  const item2 = Buffer.alloc(3); item2[0] = (1 << 6) | 40; item2.writeInt16LE(400, 1);
  return Buffer.concat([header, item1, item2]);
}

function rowSizes(buf) {
  // Walk the block structure and report the body item count of each block.
  const blocks = [];
  let i = 0;
  while (i < buf.length) {
    const mask = buf[i], count = buf[i + 1];
    i += 2;
    for (let bit = 0; bit < 6; bit++) {
      if (!(mask & (1 << bit))) continue;
      i += bit === 0 ? 8 : 4;
    }
    const items = [];
    for (let k = 0; k < count; k++) {
      const head = buf[i++];
      const type = head >> 6;
      const id = head & 63;
      const size = [1, 2, 4, 4][type];
      items.push({ id, type, value: type === 3 ? buf.readFloatLE(i) : buf.readIntLE(i, size) });
      i += size;
    }
    blocks.push({ mask, count, items });
  }
  return blocks;
}

async function play(buf, driverToAxis) {
  const player = new RrfCanPlayer();
  player.setDriverToAxis(driverToAxis);
  player.setAsapMode(true);
  const commands = [];
  player.sendCommand = async (c) => { commands.push(c); };
  await player.run(new ReadableStream({
    start(c) { c.enqueue(new Uint8Array(buf)); c.close(); },
  }), FileFormat.RRF_CAN_BINARY);
  const moves = commands.filter((c) => c.type === 'Move');
  const last = moves.at(-1) || {};
  let extrusion = 0;
  for (const m of moves) extrusion += (m.E || 0);
  const steps = {};
  for (const axis of ['A', 'B', 'C', 'D', 'I', 'J', 'L', 'O']) {
    steps[axis] = +(((last[axis] || 0) / STEP_ANGLE_RAD).toFixed(3));
  }
  return { steps, extrusionMm: +extrusion.toFixed(4) };
}

const driverToAxis = parseRrfMotorAxisMapFromConfigText(fs.readFileSync(cfgPath, 'utf8'));
const convertedBlocks = rowSizes(converted);
const idealBlocks = rowSizes(idealCan());
const convertedResult = await play(converted, driverToAxis);
const idealResult = await play(idealCan(), driverToAxis);

console.log(JSON.stringify({
  fixture_row: 'seq=0, addr=40, when=0, accel=2500, steady=836026, decel=2500, X=800, Y=400, a, d',
  converted_blocks: convertedBlocks,
  ideal_blocks: idealBlocks,
  ideal_result: idealResult,
  converted_result: convertedResult,
  expected_for_two_drivers: { X_steps: 800, Y_steps: 400, extrusion_mm: 0 },
}, null, 2));

// Checks: the converted file keeps only 800 steps (the 400 Y steps were read as
// the acceleration field), and even the ideal two-record file puts 1200 steps on
// one axis because the record carries the board address, not the driver index.
const problems = [];
const lossyConverted = convertedBlocks.some((b) => b.items.some((it) => Math.abs(it.value) === 400));
if (lossyConverted) {
  problems.push('csv2can.py kept the Y step count (400) as a numeric field (it is read as the acceleration)');
}
const idealTotal = Object.values(idealResult.steps).reduce((s, v) => s + v, 0);
if (idealTotal !== 800 && idealTotal !== 400) {
  problems.push(`a correct two-record .can puts ${idealTotal} steps on one axis; the two drives cannot be separated`);
}
const idealY = idealResult.steps.B || idealResult.steps.C || 0;
if (Math.abs(idealY) > 1e-9 || Math.abs(idealResult.extrusionMm) > 1e-9) {
  problems.push('the two drives were merged onto one axis instead of X and Y');
}

if (problems.length) {
  console.error('\nF2 BINARY-PATH RESULT: multi-driver boards are not representable in the .can schema either:');
  for (const p of problems) console.error('  - ' + p);
  process.exit(1);
}
console.log('OK: .can path separates the two drives');
