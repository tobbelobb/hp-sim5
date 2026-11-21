// Scratch: replay autocal_headless events.jsonl; dump true effector pose at each full encoder read.
import fs from 'node:fs';
import readline from 'node:readline';
import { setSolverIterations } from '../hp-sim-3d/app/solver-iterations.js';
import { setPhysicsSubsteps } from '../hp-sim-3d/app/physics-substeps.js';
import { createHeadlessWorld } from '../hp-sim-3d/app/headlessWorld.js';
import { setLineLayeringFeatureFlags } from '../hp-sim-3d/app/line-layering-flags.js';
import { setClosedLoopMotorFeatureFlags } from '../hp-sim-3d/app/closed-loop-flags.js';
import { createExternalCommandController } from '../hp-sim-3d/app/externalCommandSocket.js';
import { PositionComponent, OrientationComponent, SceneEntityInfoComponent } from '../src/js/cable_joints_3d/ecs.js';
import { CableJointComponent } from '../src/js/cable_joints_3d/cable_joints_core.js';

const [scene, eventsPath, outPath, initialLayering] = process.argv.slice(2);
const sceneText = fs.readFileSync(scene, 'utf8');
let lineLayeringEnabled = initialLayering === 'true', closedLoopMotorsEnabled = false, physicsSubsteps, solverIterations;
let world, remote, dt, step = 0, epoch = 0, response, controller;
const out = fs.openSync(outPath, 'w');
function buildWorld() { ({ world, remote, dt } = createHeadlessWorld(sceneText, { lineLayeringEnabled, closedLoopMotorsEnabled, physicsSubsteps, solverIterations })); }
buildWorld();
class ObservationSocket { static OPEN = 1; readyState = 1; addEventListener() {} send(v) { response = JSON.parse(v); } }
function attachController() {
  controller = createExternalCommandController({ world, url: 'ws://observation', WebSocketCtor: ObservationSocket,
    featureFlags: {
      setSolverIterationsState(c) { const a = setSolverIterations(world, c); solverIterations = c; return a; },
      setPhysicsSubstepsState(c) { physicsSubsteps = setPhysicsSubsteps(world, c); },
      get lineLayeringEnabled() { return lineLayeringEnabled; },
      setLineLayeringEnabledState(e) { lineLayeringEnabled = e; setLineLayeringFeatureFlags(world, e); },
      setClosedLoopMotorsEnabledState(e) { closedLoopMotorsEnabled = e; setClosedLoopMotorFeatureFlags(world, e); },
    },
    commands: {
      getRemoteSystem: () => remote,
      pushExternalCommands(batch) { batch.forEach(c => remote.addCommand(c)); return true; },
      handleUserReset() { epoch++; step = 0; buildWorld(); attachController(); },
      applyTimeScaleChange() {},
    }, runtime: { resume() {} }, logger: { info() {}, log() {}, warn() {} },
  });
  controller.connect();
}
attachController();
function advanceTo(target) { while (step < target) { world.update(dt); step++; } }
function drain() { const n = remote.getQueueLength(); for (let i = 0; i < n; i++) { world.update(dt); step++; } }
const named = () => {
  const r = {};
  for (const id of world.query([SceneEntityInfoComponent])) r[world.getComponent(id, SceneEntityInfoComponent).name] = id;
  return r;
};
let names = null, reads = 0, maxErr = 0;
const xyz = v => [v.x, v.y, v.z];
for await (const line of readline.createInterface({ input: fs.createReadStream(eventsPath) })) {
  const e = JSON.parse(line);
  if (e.type !== 'bridge_payload') continue;
  if (e.epoch !== epoch) throw new Error(`epoch mismatch ${e.epoch} ${epoch}`);
  advanceTo(e.step);
  const p = e.payload;
  if (p.type === 'encoder_request' && p.axes.length) drain();
  controller.handlePayload(p);
  if (p.type === 'encoder_request' && p.axes.length >= 4) {
    names ??= named();
    const eff = names.Effector;
    const rec = { epoch, step, axes: p.axes, angles_deg: response.anglesDeg,
      pos: xyz(world.getComponent(eff, PositionComponent).pos), q: (q => [q.x, q.y, q.z, q.w])(world.getComponent(eff, OrientationComponent).quaternion),
      forces: Object.fromEntries(Object.entries(names).filter(([n, id]) => world.getComponent(id, CableJointComponent)).map(([n, id]) => [n, world.getComponent(id, CableJointComponent).constraintForceMagnitude])) };
    fs.writeSync(out, JSON.stringify(rec) + '\n');
    if (++reads % 1000 === 0) console.error(reads, 'reads step', step);
  }
}
fs.closeSync(out);
console.error('done', reads);
