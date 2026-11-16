// Optional complete-logo replay: node tests/benchmark/logo_accuracy.mjs --machine hp4 [--substeps 1] [--iterations 10]
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { parseArgs } from 'node:util';
import { findMachine } from '../../machines/catalog.js';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { parseGcodeText } from '../../hp-sim-3d/app/inspectionToolsController.js';
import { QualityMonitor } from '../../hp-sim-3d/app/quality-monitor.js';
import { getMachineMotorDiagnostics } from '../../hp-sim-3d/app/motor-diagnostics.js';
import { parseRrfMotorAxisMapFromConfigText } from '../../integrations/rrf/rrfFirmwareModel.js';
import { detectFileFormat } from '../../integrations/shared/fileFormatUtils.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { PositionComponent, VelocityComponent, AngularVelocityComponent, RigidBodyComponent } from '../../src/js/cable_joints_3d/ecs.js';

const { values: args } = parseArgs({ options: {
  solver: { type: 'string', default: 'native' },
  firmware: { type: 'string', default: 'rrf' }, gcode: { type: 'string', default: 'Hangprinter_logo6' },
  commands: { type: 'string' }, 'max-steps': { type: 'string' },
  machine: { type: 'string', default: 'hp4' }, substeps: { type: 'string' }, iterations: { type: 'string' },
  'open-loop': { type: 'boolean', default: false },
} });
const machine = findMachine(args.machine);
if (!machine) throw new Error(`Unknown machine: ${args.machine}`);
const root = new URL('../../', import.meta.url);
const read = path => fs.readFileSync(new URL(path, root), 'utf8');
const scene = read(machine.scene);
const gcode = `public/gcode/${args.gcode}.gcode`;
globalThis.self = { addEventListener() {} };
globalThis.postMessage = message => { if (message.type === 'error') throw new Error(message.message); };
const { RrfCanPlayer } = await import('../../integrations/rrf/rrfCanPlayer.js');
const { KlipperMcuCommandPlayer } = await import('../../integrations/klipper/klipperMcuCommandPlayer.js');
const player = args.firmware === 'klipper' ? new KlipperMcuCommandPlayer() : new RrfCanPlayer();
player.setDt(.002);
player.setAsapMode(true);
player.setDriverToAxis?.(parseRrfMotorAxisMapFromConfigText(read(machine.rrf_config)));
const commands = [];
player.sendCommand = async command => { commands.push(command); };
const commandPath = args.commands ?? machine.compiled_commands[gcode][args.firmware];
const commandUrl = commandPath.startsWith('/') ? new URL(`file://${commandPath}`) : new URL(commandPath, root);
await player.run(Readable.toWeb(fs.createReadStream(commandUrl)), detectFileFormat(commandPath));
if (args['max-steps']) commands.length = Math.min(commands.length, Number(args['max-steps']));
const { world, remote, dt } = createHeadlessWorld(scene, {
  cableSolverBackend: args.solver === 'wasm'
    ? new WasmPhysicsEngine(new WebAssembly.Module(fs.readFileSync(new URL('public/wasm/cable_solver.wasm', root)))) : undefined,
  lineLayeringEnabled: machine.line_layering, closedLoopMotorsEnabled: !args['open-loop'],
  solverIterations: args.iterations === undefined ? undefined : Number(args.iterations),
  physicsSubsteps: args.substeps === undefined ? undefined : Number(args.substeps),
});
try {
  const monitor = new QualityMonitor();
  monitor.setReferenceSegments(parseGcodeText(read(gcode)).segments.map(segment => ({
    start: segment.start.map(v => v * .001), end: segment.end.map(v => v * .001), extrusion: segment.extrusion,
  })));
  monitor.setEnabled(false);
  monitor.setMotorDiagnosticsProvider(() => getMachineMotorDiagnostics(world));
  remote.setExtrusionListener(event => monitor.recordExtrusion({ pos: event.pos, length: event.length }));
  remote.commands = commands;
  const steps = commands.length;
  const body = world.query([RigidBodyComponent])[0];
  const peaks = { speed: 0, omega: 0, heightFromStart: 0 };
  const startZ = world.getComponent(body, PositionComponent).pos.z;
  const started = performance.now();
  for (let step = 0; step < steps; step++) {
    world.update(dt);
    const position = world.getComponent(body, PositionComponent).pos;
    peaks.speed = Math.max(peaks.speed, world.getComponent(body, VelocityComponent).vel.length());
    peaks.omega = Math.max(peaks.omega, world.getComponent(body, AngularVelocityComponent).omega.length());
    peaks.heightFromStart = Math.max(peaks.heightFromStart, Math.abs(position.z - startZ));
    if (![position.x, position.y, position.z, ...Object.values(peaks)].every(Number.isFinite)) {
      throw new Error(`Non-finite physics at step ${step}`);
    }
    if (world.getResource('errorState')?.hasError || world.getResource('pauseState')?.paused) {
      throw new Error(`Physics stopped at step ${step}`);
    }
  }
  const wallSeconds = (performance.now() - started) / 1000;
  monitor.setEnabled(true);
  monitor.runFinalCheck();
  console.log(JSON.stringify({ machine: args.machine, firmware: args.firmware, solver: args.solver, gcode, substeps: world.getResource('physicsSubsteps'),
    iterations: world.getResource('solverIterations'), closedLoop: !args['open-loop'], steps, wallSeconds, peaks, metrics: monitor.getMetrics(),
    motors: getMachineMotorDiagnostics(world) }));
} finally {
  world.getResource('cableSolverBackend')?.dispose();
}
