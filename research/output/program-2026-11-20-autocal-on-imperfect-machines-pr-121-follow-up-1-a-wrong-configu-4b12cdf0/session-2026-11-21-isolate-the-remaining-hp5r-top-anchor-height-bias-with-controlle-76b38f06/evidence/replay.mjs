import fs from 'node:fs';
import readline from 'node:readline';
import { setSolverIterations } from '../../../../../hp-sim-3d/app/solver-iterations.js';
import { setPhysicsSubsteps } from '../../../../../hp-sim-3d/app/physics-substeps.js';
import { createHeadlessWorld } from '../../../../../hp-sim-3d/app/headlessWorld.js';
import { setLineLayeringFeatureFlags } from '../../../../../hp-sim-3d/app/line-layering-flags.js';
import { setClosedLoopMotorFeatureFlags } from '../../../../../hp-sim-3d/app/closed-loop-flags.js';
import { createExternalCommandController } from '../../../../../hp-sim-3d/app/externalCommandSocket.js';
import { AngularVelocityComponent, MomentOfInertiaComponent, PositionComponent, OrientationComponent, RigidBodyMemberComponent, SceneEntityInfoComponent } from '../../../../../src/js/cable_joints_3d/ecs.js';
import { CableJointComponent, CablePathComponent } from '../../../../../src/js/cable_joints_3d/cable_joints_core.js';

// Adapted from the compliance-probe replay. Only collect the data-point time windows.
const [scene, eventsPath, outPath, windowsPath, option] = process.argv.slice(2);
if (option && option !== '--lock-effector') throw new Error('Unknown replay option');
const lockEffector = option === '--lock-effector';
const windows = JSON.parse(fs.readFileSync(windowsPath, 'utf8')).sort((a, b) => a[0] - b[0]);
let windowIndex = 0;
const sceneText = fs.readFileSync(scene, 'utf8');
let lineLayeringEnabled = false, closedLoopMotorsEnabled = false, physicsSubsteps, solverIterations;
let world, remote, dt, step = 0, epoch = 0, response, controller;
const out = fs.openSync(outPath, 'w');
function buildWorld() {
  ({ world, remote, dt } = createHeadlessWorld(sceneText, {
    lineLayeringEnabled, closedLoopMotorsEnabled, physicsSubsteps, solverIterations,
  }));
  if (!lockEffector) return;
  const body = world.query([SceneEntityInfoComponent, MomentOfInertiaComponent])
    .find(id => world.getComponent(id, SceneEntityInfoComponent).name === 'Effector');
  const inertia = world.getComponent(body, MomentOfInertiaComponent);
  inertia.invInertiaTensor.forEach(row => row.fill(0));
  world.getComponent(body, AngularVelocityComponent).omega.scale(0);
}
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
let names = null, reads = 0, compared = 0, maxErr = 0, requestId, homeQ;
const lengths = (j, eff, pos, undoRoll) => {
  const onEffector = id => id === eff ||
    world.getComponent(id, RigidBodyMemberComponent)?.bodyEntity === eff;
  const frozen = (endpoint, id) => onEffector(id) ?
    undoRoll.transformVector(endpoint.clone().subtract(pos)).add(pos) : endpoint;
  const a = j.attachmentPointA_world, b = j.attachmentPointB_world;
  const force = j.constraintForceMagnitude;
  const frozenDistance = frozen(a, j.entityA).distanceTo(frozen(b, j.entityB));
  const verticalForce = onEffector(j.entityA) === onEffector(j.entityB) ? 0 :
    (onEffector(j.entityA) ? b.z - a.z : a.z - b.z) * force / a.distanceTo(b);
  return [force, j.restLength, a.distanceTo(b), frozenDistance, verticalForce];
};
const xyz = v => [v.x, v.y, v.z];
for await (const line of readline.createInterface({ input: fs.createReadStream(eventsPath) })) {
  const e = JSON.parse(line);
  if (e.type === 'encoder_response' && e.axes.length === 5) {
    if (requestId !== e.request_id) throw new Error('Encoder response has no replay request');
    compared++;
    maxErr = Math.max(maxErr, ...e.angles_deg.map((v, i) => Math.abs(v - response.anglesDeg[i])));
  }
  if (e.type !== 'bridge_payload') continue;
  if (e.epoch !== epoch) throw new Error(`epoch mismatch ${e.epoch} ${epoch}`);
  advanceTo(e.step);
  const p = e.payload;
  if (p.type === 'encoder_request' && p.axes.length) drain();
  controller.handlePayload(p);
  if (p.type !== 'encoder_request' || p.axes.length !== 5) continue;
  requestId = p.requestId;
  while (windowIndex < windows.length && windows[windowIndex][1] < step) windowIndex++;
  if (reads > 0 && (windowIndex === windows.length || step < windows[windowIndex][0])) continue;
  names ??= named();
  const eff = names.Effector;
  const pos = world.getComponent(eff, PositionComponent).pos;
  const q = world.getComponent(eff, OrientationComponent).quaternion;
  homeQ ??= q.clone();
  const undoRoll = homeQ.clone().multiply(q.clone().conjugate());
  const joints = {}, wraps = {};
  for (const [name, id] of Object.entries(names)) {
    const joint = world.getComponent(id, CableJointComponent);
    if (joint) joints[name] = lengths(joint, eff, pos, undoRoll);
    const path = world.getComponent(id, CablePathComponent);
    if (path) wraps[name] = path.stored.slice(1, -1).reduce((a, b) => a + b, 0);
  }
  fs.writeSync(out, JSON.stringify({ epoch, step, angles_deg: response.anglesDeg,
    pos: xyz(pos), q: [q.x, q.y, q.z, q.w], joints, wraps }) + '\n');
  reads++;
}
fs.closeSync(out);
console.log(JSON.stringify({reads, compared, lockEffector, max_encoder_error_deg: maxErr}));
