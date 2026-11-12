// Full HP4 layered logo, native engine, open loop, 1 substep. Logs missed
// steps per motor over time and the effector pose around each new slip.
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { parseRrfMotorAxisMapFromConfigText } from '../../../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../../../integrations/shared/fileFormatUtils.js';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { getMachineMotorDiagnostics } from '../../../../hp-sim-3d/app/motor-diagnostics.js';
import { PositionComponent, RigidBodyComponent } from '../../../../src/js/cable_joints_3d/ecs.js';
globalThis.self = { addEventListener() {} };
globalThis.postMessage = m => { if (m.type === 'error') throw new Error(m.message); };
const root = new URL('../../../../', import.meta.url);
const { RrfCanPlayer } = await import('../../../../integrations/rrf/rrfCanPlayer.js');
const player = new RrfCanPlayer();
player.setDt(.002); player.setAsapMode(true);
player.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(fs.readFileSync(new URL('RRF/run/vsd/sys/config_hp4_w_line_layers.g', root), 'utf8')));
const commands = [];
player.sendCommand = async c => { commands.push(c); };
await player.run(Readable.toWeb(fs.createReadStream(new URL('public/RRF_CAN_commands/Hangprinter_logo6_hp4_w_line_layers.can', root))), FileFormat.RRF_CAN_BINARY);
const out = process.argv[2];
if (out === 'commands') { fs.writeFileSync('/dev/stdout', JSON.stringify(commands.slice(-5))); console.log('\n', commands.length); process.exit(0); }
const scene = fs.readFileSync(new URL('public/usd_scenes/hp4_rigid_body.usda', root), 'utf8');
const { world, remote, dt } = createHeadlessWorld(scene);
remote.commands = commands;
const body = world.query([RigidBodyComponent])[0];
const extrusions = [];
remote.setExtrusionListener(e => extrusions.push([e.pos[0], e.pos[1], e.pos[2], e.length, step]));
let prev = 0, step = 0;
const log = [];
const total = commands.length + 2000;
const t0 = performance.now();
for (step = 0; step < total; step++) {
  world.update(dt);
  const d = getMachineMotorDiagnostics(world);
  if (d.totalMissedSteps !== prev) {
    const p = world.getComponent(body, PositionComponent).pos;
    log.push({ step, t: step * dt, total: d.totalMissedSteps, queue: remote.getQueueLength(), pos: [p.x, p.y, p.z], motors: d.motors.map(m => `${m.axis}:${m.slips}/${m.netSlips}`).join(' ') });
    prev = d.totalMissedSteps;
  }
}
const sec = (performance.now() - t0) / 1000;
fs.writeFileSync(out, JSON.stringify({ commands: commands.length, steps: total, wall_s: sec, log, lastCommands: commands.slice(-3) }, null, 1));
fs.writeFileSync(out.replace('.json', '_extrusions.json'), JSON.stringify(extrusions));
console.log('commands', commands.length, 'wall', sec.toFixed(1), 's, slip events logged', log.length, 'final', prev);
