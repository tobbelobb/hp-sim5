import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { getMachineMotorDiagnostics } from '../../hp-sim-3d/app/motor-diagnostics.js';
import { StepperMotorComponent } from '../../hp-sim-3d/app/hangprinter_stepper_motor.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { parseRrfMotorAxisMapFromConfigText } from '../../integrations/rrf/rrfFirmwareModel.js';
import { FileFormat } from '../../integrations/shared/fileFormatUtils.js';

// Schedule the real bundled logo at the firmware's 2 ms cadence. Keep only
// the prefix needed to expose both the immediate and the later missed steps.
globalThis.self = { addEventListener() {} };
globalThis.postMessage = message => {
  if (message.type === 'error') throw new Error(message.message);
};
const { RrfCanPlayer } = await import('../../integrations/rrf/rrfCanPlayer.js');
const player = new RrfCanPlayer();
player.setDt(.002);
player.setAsapMode(true);
player.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(fs.readFileSync(
  new URL('../../RRF/run/vsd/sys/config_hp4_w_line_layers.g', import.meta.url), 'utf8')));
const commands = [];
player.sendCommand = async command => { if (commands.length < 10000) commands.push(command); };
await player.run(Readable.toWeb(fs.createReadStream(
  new URL('../../public/RRF_CAN_commands/Hangprinter_logo6_hp4_w_line_layers.can', import.meta.url))), FileFormat.RRF_CAN_BINARY);
assert.equal(commands.length, 10000);
const scene = fs.readFileSync(new URL('../../public/usd_scenes/hp4_rigid_body.usda', import.meta.url), 'utf8');
const module = new WebAssembly.Module(fs.readFileSync(new URL('../../public/wasm/cable_solver.wasm', import.meta.url)));

for (const [name, createBackend, steps, substeps] of [
  ['JavaScript', () => null, 150, 4],
  ['Wasm', () => new WasmPhysicsEngine(module), 10000, 2],
  ['Wasm', () => new WasmPhysicsEngine(module), 10000, 4],
  ['Wasm', () => new WasmPhysicsEngine(module), 10000, 16],
]) test(`HP4 open-loop layered logo stays in step: ${name}, ${substeps} substeps`, () => {
  const backend = createBackend();
  try {
    const { world, remote, dt } = createHeadlessWorld(scene, {
      cableSolverBackend: backend, physicsSubsteps: substeps,
      lineLayeringEnabled: true, closedLoopMotorsEnabled: false,
    });
    remote.commands = commands.slice(0, steps);
    for (let step = 0; step < steps; step++) world.update(dt);
    assert.equal(remote.getQueueLength(), 0);
    assert(world.query([StepperMotorComponent]).every(id => !world.getComponent(id, StepperMotorComponent).closedLoop));
    assert.equal(world.getResource('dt'), .002);
    assert.equal(getMachineMotorDiagnostics(world).totalMissedSteps, 0);
  } finally {
    backend?.dispose();
  }
});
