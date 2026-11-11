// Print per-joint forces along each cable path at a chosen time of the Squares replay.
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { createHeadlessWorld } from '../../../../hp-sim-3d/app/headlessWorld.js';
import { SceneEntityInfoComponent } from '../../../../src/js/cable_joints_3d/ecs.js';
import { CablePathComponent, CableJointComponent } from '../../../../src/js/cable_joints_3d/cable_joints_core.js';
import { StepperMotorComponent } from '../../../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { parseRrfMotorAxisMapFromConfigText } from '../../../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../../../integrations/shared/fileFormatUtils.js';
const root = new URL('../../../../', import.meta.url);
const [tStop = '20'] = process.argv.slice(2);
globalThis.self = { addEventListener() {} }; globalThis.postMessage = () => {};
const { RrfCanPlayer } = await import('../../../../integrations/rrf/rrfCanPlayer.js');
const player = new RrfCanPlayer(); player.setDt(.002); player.setAsapMode(true);
player.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(fs.readFileSync(new URL('RRF/run/vsd/sys/config_hp4.g', root), 'utf8')));
const commands = []; const n = Math.round(Number(tStop) / .002);
player.sendCommand = async c => { if (commands.length < n) commands.push(c); };
await player.run(Readable.toWeb(fs.createReadStream(new URL('public/RRF_CAN_commands/draw_squares_bigger_hp4_no_buildup.can', root))), FileFormat.RRF_CAN_BINARY);
const { world, remote, dt } = createHeadlessWorld(fs.readFileSync(new URL(process.env.SCENE ?? 'public/usd_scenes/hp4_rigid_body.usda', root), 'utf8'), { lineLayeringEnabled: false, closedLoopMotorsEnabled: true });
remote.commands = commands.slice();
for (let i = 0; i < n; i++) world.update(dt);
const name = id => world.getComponent(id, SceneEntityInfoComponent)?.name ?? world.getComponent(id, SceneEntityInfoComponent)?.path ?? id;
for (const pid of world.query([CablePathComponent])) {
  const p = world.getComponent(pid, CablePathComponent);
  console.log(name(pid), p.linkTypes.join(','));
  for (const j of p.jointEntities) { const c = world.getComponent(j, CableJointComponent); console.log('   ', String(name(c.entityA)).padEnd(28), '->', String(name(c.entityB)).padEnd(28), c.constraintForceMagnitude.toFixed(2), 'N'); }
}
for (const id of world.query([StepperMotorComponent])) console.log(name(id), 'servoTau', world.getComponent(id, StepperMotorComponent).servoIntegralTorque.toFixed(3));
