// Reproducer: the Upload path's G6 support moves the machine out and back.
//
// Expects (Hangprinter G6 semantics, and moveCommander's own intent):
//   G6 A-42 B-42 C-42 F5000  ->  A, B and C each pay out -42 mm of line
//   (hp4: R=30, U=2:2:2:4, H=L=20  ->  -2.8 rad = -160.428 deg of spool rotation)
//
// Actual: the 175 interpolated 'Move' commands ramp the motor target by
// -2.8 rad, then the trailing 'Add to reference' (moveCommander.js:409-411)
// adds the same -2.8 rad to deltaAngle, and the world's target is
// commandedAngle - deltaAngle (src/wasm/cable_solver/src/engine.rs:1568), so
// the net commanded displacement is 0.000 deg -- the tightening is undone.
//
// Confirmed end-to-end in the shipped world by probe_world_g6.mjs:
//   net 0.00000 deg, peak -160.428 deg on A/B/C.
//
// Exit code 1 while the bug is present, 0 when the payout survives.
import fs from 'node:fs';

globalThis.self = { addEventListener() {} };
globalThis.postMessage = () => {};
const ROOT = new URL('../../../../', import.meta.url);
const { MoveCommander } = await import(new URL('hp-sim-3d/app/workers/moveCommander.js', ROOT));

const configText = fs.readFileSync(new URL('RRF/run/vsd/sys/config_hp4.g', ROOT), 'utf8');
const gcode = fs.readFileSync(new URL('public/gcode/tighten.gcode', ROOT), 'utf8');

const commander = new MoveCommander();
commander.configure(configText);
commander.setAsapMode(true);
const commands = [];
commander.sendCommand = async (c) => { commands.push(c); };
await commander.run(new ReadableStream({
  start(controller) { controller.enqueue(new TextEncoder().encode(gcode)); controller.close(); },
}));

// --- model of the world's command application -------------------------------
// hp-sim-3d/app/remoteSpoolSystem.js:267-289  Move -> commandedAngle = v
//                                             Add  -> deltaAngle += v
// src/wasm/cable_solver/src/engine.rs:1568    target = commandedAngle - deltaAngle
const axes = commander.motorAxes;                 // ['A','B','C','D'] for hp4
const commanded = Object.fromEntries(axes.map((a) => [a, 0]));
const delta = Object.fromEntries(axes.map((a) => [a, 0]));
const target = () => Object.fromEntries(axes.map((a) => [a, commanded[a] - delta[a]]));
const t0 = target();
const peak = { ...t0 };
for (const c of commands) {
  if (c.type === 'Move') {
    for (const a of axes) if (Number.isFinite(c[a])) commanded[a] = c[a];
  } else if (c.type === 'Add to reference') {
    for (const a of axes) if (Number.isFinite(c[a])) delta[a] += c[a];
  }
  const t = target();
  for (const a of axes) if (Math.abs(t[a] - t0[a]) > Math.abs(peak[a] - t0[a])) peak[a] = t[a];
}
const tEnd = target();
const deg = (v) => v * 180 / Math.PI;
const expectedA = -42 / commander.spool_radius_mm[0] * commander.spoolParams.U[0]; // -2.8 rad

console.log(`G-code: ${JSON.stringify(gcode.trim())}`);
console.log(`commands: ${commands.length} (${commands.filter((c) => c.type === 'Move').length} Move, ` +
  `${commands.filter((c) => c.type === 'Add to reference').length} Add to reference)`);
console.log(`expected net A/B/C spool rotation: ${deg(expectedA).toFixed(3)} deg (= -42 mm of line)`);
console.log('axis  net target change (deg)   peak during move (deg)');
for (const a of axes) {
  console.log(`  ${a}   ${deg(tEnd[a] - t0[a]).toFixed(6).padStart(18)}   ${deg(peak[a] - t0[a]).toFixed(3).padStart(18)}`);
}
const net = Math.max(...axes.map((a) => Math.abs(tEnd[a] - t0[a])));
const travel = Math.max(...axes.map((a) => Math.abs(peak[a] - t0[a])));
console.log(`max |net| = ${deg(net).toFixed(6)} deg; max excursion = ${deg(travel).toFixed(3)} deg`);
console.log(`line payout actually delivered: ${(net * (commander.spool_radius_mm[0] / commander.spoolParams.U[0])).toFixed(4)} mm ` +
  `of the commanded -42 mm (${(100 * net / Math.abs(expectedA)).toFixed(1)} %)`);

if (Math.abs(net - Math.abs(expectedA)) > 0.05) {
  console.error(`\nFAIL: net spool rotation is ${deg(net).toFixed(6)} deg but should be ${deg(-Math.abs(expectedA)).toFixed(3)} deg.`);
  console.error('The G6 move is undone by the trailing Add to reference (moveCommander.js:409-411).');
  process.exit(1);
}
console.log('\nPASS: the G6 line payout survives.');
