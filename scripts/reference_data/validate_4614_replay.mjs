// Replay the same 2 ms commands and WASM scene as hp-sim-3d, including stationary intervals.
import fs from 'node:fs';
import { Readable } from 'node:stream';
import { findMachine } from '../../machines/catalog.js';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { WasmPhysicsEngine } from '../../src/js/cable_joints_3d/wasm_physics_engine.js';
import { SceneEntityInfoComponent, PositionComponent } from '../../src/js/cable_joints_3d/ecs.js';
import { CableJointComponent } from '../../src/js/cable_joints_3d/cable_joints_core.js';
import { FileFormat } from '../../integrations/shared/fileFormatUtils.js';
import { parseRrfMotorAxisMapFromConfigText } from '../../integrations/rrf/rrfFirmwareModel.js';

globalThis.self = { addEventListener() {} };
globalThis.postMessage = message => { if (message.type === 'error') throw Error(message.message); };
const { RrfCanPlayer } = await import('../../integrations/rrf/rrfCanPlayer.js');
const replay = process.argv[2] ?? 'planeATargets';
const machine = findMachine('copacabana');
const url = path => new URL('../../' + path, import.meta.url);
const read = path => fs.readFileSync(url(path), 'utf8');
const backend = new WasmPhysicsEngine(new WebAssembly.Module(fs.readFileSync(url('public/wasm/cable_solver.wasm'))));
const { world, remote, dt } = createHeadlessWorld(read(machine.scene), {
  cableSolverBackend: backend, lineLayeringEnabled: false, closedLoopMotorsEnabled: true,
});
const named = new Map(world.query([SceneEntityInfoComponent]).map(id => [world.getComponent(id, SceneEntityInfoComponent).name, id]));
const axes = Array.from('ABCDIJLO');
const joints = axes.map(axis => world.getComponent(named.get(`Joint${axis}_1`), CableJointComponent));
const rows = read('tests/fixtures/reference_data/darus4614_planeA_cycle.csv').trim().split('\n').slice(1)
  .map(line => {
    const values = line.split(',').map(Number);
    return { time: values[0], marker: values.slice(1, 4), force: values.slice(4, 12), target: values.slice(12) };
  });
let tick = 0, sample = 0, positionSquared = 0, forceSquared = 0, maxDistance = 0;
const spans = axes.map(axis => {
  const position = prim => world.getComponent(named.get(prim + axis), PositionComponent).pos;
  return position('Anchor').clone().subtract(position('Attach'));
});
const holds = [];
let holdStart = 0;
const homeMarker = world.getComponent(named.get('Reflector'), PositionComponent).pos.clone();
let nominalSquared = 0;
let constantForceSquared = 0;
function measure() {
  const row = rows[sample];
  if (!row || Math.round(row.time / dt) !== tick) return;
  const { x, y, z } = world.getComponent(named.get('Reflector'), PositionComponent).pos;
  const distance = Math.hypot(x - row.marker[0], y - row.marker[1], z - row.marker[2]);
  maxDistance = Math.max(distance, maxDistance);
  positionSquared += distance ** 2;
  nominalSquared += (homeMarker.x + row.target[0] - row.marker[0]) ** 2
    + (homeMarker.y + row.target[1] - row.marker[1]) ** 2 + (homeMarker.z + row.target[2] - row.marker[2]) ** 2;
  if (sample > 0) {
    joints.forEach((joint, i) => {
      forceSquared += (joint.constraintForceMagnitude - row.force[i]) ** 2;
      constantForceSquared += (rows[0].force[i] - row.force[i]) ** 2;
    });
  }
  // Once the nominal target has held still for 0.5 s, compare observations at rest.
  if (sample && row.target.some((v, i) => Math.abs(v - rows[sample - 1].target[i]) > 1e-7)) {
    holdStart = sample;
  }
  if (row.time - rows[holdStart].time > .5) {
    let hold = holds.at(-1);
    if (hold?.start !== holdStart) {
      hold = { start: holdStart, count: 0, trackerSquared: 0, forceSquared: 0, netForce: [0, 0, 0],
        predictedForce: axes.map(() => 0), loggedForce: axes.map(() => 0) };
      holds.push(hold);
    }
    // Static force closure using the nominal unrotated pose and home tangent anchors.
    const net = [0, 0, -13.9 * 9.81];
    spans.forEach((span, i) => {
      const direction = [span.x - row.target[0], span.y - row.target[1], span.z - row.target[2]];
      const scale = row.force[i] / Math.hypot(...direction);
      direction.forEach((v, k) => { net[k] += scale * v; });
      hold.predictedForce[i] += joints[i].constraintForceMagnitude;
      hold.loggedForce[i] += row.force[i];
      hold.forceSquared += (joints[i].constraintForceMagnitude - row.force[i]) ** 2;
    });
    net.forEach((v, k) => { hold.netForce[k] += v; });
    hold.count++;
    hold.trackerSquared += distance ** 2;
  }
  sample++;
}
const player = new RrfCanPlayer();
player.setDt(dt);
player.setAsapMode(true);
player.setDriverToAxis(parseRrfMotorAxisMapFromConfigText(read(machine.rrf_config)));
measure();
player.sendCommand = async command => {
  remote.addCommand(command);
  world.update(dt);
  tick++;
  measure();
};
try {
  await player.run(Readable.toWeb(fs.createReadStream(url(machine.reference_replay.streams[replay]))),
    FileFormat.RRF_CAN_BINARY, { preserveTiming: true });
  if (holds.length !== 5 || sample !== rows.length || !Number.isFinite(positionSquared + forceSquared)
    || world.getResource('errorState')?.hasError) {
    throw Error(`Incomplete comparison at tick ${tick}, sample ${sample}`);
  }
  const rms = (sum, count) => Math.sqrt(sum / count);
  const forceCount = (sample - 1) * joints.length;
  console.log(JSON.stringify({
    replay,
    duration_s: tick * dt,
    samples: sample,
    reflector_rms_mm: 1000 * rms(positionSquared, sample),
    nominal_rms_mm: 1000 * rms(nominalSquared, sample),
    reflector_max_mm: 1000 * maxDistance,
    cable_force_rms_N: rms(forceSquared, forceCount),
    initial_force_baseline_rms_N: rms(constantForceSquared, forceCount),
    settled: holds.map((h, i) => ({
      pose: `P${i + 1}`,
      reflector_rms_mm: 1000 * rms(h.trackerSquared, h.count),
      force_rms_N: rms(h.forceSquared, h.count * joints.length),
      mean_simulated_cable_forces_N: h.predictedForce.map(v => v / h.count),
      mean_logged_cable_forces_N: h.loggedForce.map(v => v / h.count),
      measured_force_closure_N: Math.hypot(...h.netForce) / h.count,
    })),
  }, null, 2));
} finally {
  backend.dispose();
}
