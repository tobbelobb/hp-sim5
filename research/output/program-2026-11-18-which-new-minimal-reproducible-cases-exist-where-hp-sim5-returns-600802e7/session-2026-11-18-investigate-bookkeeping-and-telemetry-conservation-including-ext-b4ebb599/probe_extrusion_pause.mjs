// Secondary probe: extrusion ledger across E signs and pause/resume.
// Positive E is deposited once, negative/zero E is not, a paused world does not
// consume the queued command, and resuming deposits it once.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = process.env.HP_SIM5_ROOT || path.resolve(here, '../../../..');
const { createHeadlessWorld } = await import(path.join(root, 'hp-sim-3d/app/headlessWorld.js'));
const { WasmPhysicsEngine } = await import(path.join(root, 'src/js/cable_joints_3d/wasm_physics_engine.js'));
const { ExtruderComponent } = await import(path.join(root, 'hp-sim-3d/app/hangprinter_extruder.js'));

const backend = new WasmPhysicsEngine(
  new WebAssembly.Module(fs.readFileSync(path.join(root, 'public/wasm/cable_solver.wasm'))));
const { world, remote, dt } = createHeadlessWorld(
  fs.readFileSync(path.join(root, 'public/usd_scenes/slideprinter.usda'), 'utf8'),
  { lineLayeringEnabled: false, cableSolverBackend: backend });
world.systems.find(s => s.constructor.name === 'ExtruderSystem')?.update(world, 0);

const ledger = () => {
  const id = world.query([ExtruderComponent])[0];
  const component = world.getComponent(id, ExtruderComponent);
  return { n: component.extrusions.length, sum: component.extrusions.reduce((sum, e) => sum + e.length, 0) };
};
const axes = Object.keys(remote.axisToEntity);
const move = (E) => {
  const command = { type: 'Move' };
  axes.forEach(axis => { command[axis] = 0; });
  if (E !== undefined) command.E = E;
  return command;
};

const out = [];
remote.addCommand(move(5)); world.update(dt); out.push({ label: 'E=5', ...ledger() });
remote.addCommand(move(-2)); world.update(dt); out.push({ label: 'E=-2 (retract)', ...ledger() });
remote.addCommand(move(0)); world.update(dt); out.push({ label: 'E=0', ...ledger() });
world.setResource('pauseState', { paused: true });
remote.addCommand(move(3)); world.update(dt);
out.push({ label: 'paused, queued E=3', ...ledger(), queue: remote.getQueueLength() });
world.setResource('pauseState', { paused: false });
world.update(dt); out.push({ label: 'resumed', ...ledger(), queue: remote.getQueueLength() });
console.log(JSON.stringify(out));
