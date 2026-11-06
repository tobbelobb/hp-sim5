import {applyMotorFrictionPlant,readNativeMotorFriction} from 'file:///workspace/hp-sim5/research/sessions/attempt-4f9da7c284d04b85810ceb12d0acbdfb/harness/proposed_motor_perturbation.mjs';
import { SceneEntityInfoComponent, CoefficientOfFrictionComponent } from 'file:///workspace/hp-sim5-worktrees/research-question-abfd2d9a/src/js/cable_joints_3d/ecs.js';
// Persistent production Rust/WASM world and JS RRF bridge owned by autocal.py.
import { setPhysicsSubsteps } from 'file:///workspace/hp-sim5-worktrees/research-question-abfd2d9a/hp-sim-3d/app/physics-substeps.js';
import fs from 'node:fs';
import http from 'node:http';
import { setImmediate as yieldStep } from 'node:timers/promises';
import WebSocket from 'file:///workspace/hp-sim5-worktrees/research-question-abfd2d9a/node_modules/ws/wrapper.mjs';
import { createHash } from 'node:crypto';
import { createHeadlessWorld } from 'file:///workspace/hp-sim5-worktrees/research-question-abfd2d9a/hp-sim-3d/app/headlessWorld.js';
import { setLineLayeringFeatureFlags } from 'file:///workspace/hp-sim5-worktrees/research-question-abfd2d9a/hp-sim-3d/app/line-layering-flags.js';
import { setClosedLoopMotorFeatureFlags } from 'file:///workspace/hp-sim5-worktrees/research-question-abfd2d9a/hp-sim-3d/app/closed-loop-flags.js';
import { createExternalCommandController } from 'file:///workspace/hp-sim5-worktrees/research-question-abfd2d9a/hp-sim-3d/app/externalCommandSocket.js';
import { createGcodeBridge } from 'file:///workspace/hp-sim5-worktrees/research-question-abfd2d9a/integrations/rrf/rrfSimulatorBridge.mjs';
import { FlightRecorder } from 'file:///workspace/hp-sim5-worktrees/research-question-abfd2d9a/hp-sim-3d/app/flightRecorder.js';

const [scene, rrfUrl, wsPort, apiPort, directory, initialLayering, recordingMode] = process.argv.slice(2);
const sceneText = fs.readFileSync(scene, 'utf8');
let step = 0, epoch = 0, totalSteps = 0, speed = 1, collectorTime = 0, stopped = false, failure = null;
let lineLayeringEnabled = initialLayering === 'true', closedLoopMotorsEnabled = false, physicsSubsteps = 1;
let world, remote, dt, bakedScene, bakedSceneFile, bakedSceneHash;
function buildWorld() {
  ({ world, remote, dt, bakedScene } = createHeadlessWorld(sceneText, { lineLayeringEnabled, closedLoopMotorsEnabled, physicsSubsteps }));
  const plantSpec = JSON.parse(fs.readFileSync(process.env.FRICTION_WORLD_SPEC, 'utf8'));
  const motorSelection=applyMotorFrictionPlant(world,plantSpec.motor_factors);
  fs.writeFileSync(`${directory}/motor-selection.json`,JSON.stringify(motorSelection,null,2));
  bakedSceneFile = `baked-scene${epoch ? `-${epoch}` : ''}.usda`;
  fs.writeFileSync(`${directory}/${bakedSceneFile}`, bakedScene);
  bakedSceneHash = createHash('sha256').update(bakedScene).digest('hex');
}
buildWorld();
const noiseSpec = JSON.parse(fs.readFileSync(process.env.FRICTION_WORLD_SPEC, 'utf8'));
let noiseState = noiseSpec.noise_seed >>> 0;
function uniform() { noiseState += 0x6D2B79F5; let t = noiseState; t = Math.imul(t ^ t >>> 15, t | 1); t ^= t + Math.imul(t ^ t >>> 7, t | 61); return ((t ^ t >>> 14) >>> 0) / 4294967296; }
function normal() { return Math.sqrt(-2*Math.log(Math.max(uniform(),1e-12)))*Math.cos(2*Math.PI*uniform()); }
function auditNative() {
  const engine = world.getResource('cableSolverBackend');
  if (!engine.fullEngine || engine.physicsEngine !== 'rust-native') throw new Error('Expected native Rust');
  if (engine.world !== world) return;
  const rows = engine.ids.map((id,i) => { const c=world.getComponent(id,SceneEntityInfoComponent), a=engine.entityRegion.data, o=i*160; return {id,name:c?.name,mu_ecs:world.getComponent(id,CoefficientOfFrictionComponent)?.mu,mu_native:a[o+105],radius_native:a[o+104],motor_native:Array.from(a.slice(o+107,o+121))}; });
  const motors=readNativeMotorFriction(world),selected=JSON.parse(fs.readFileSync(`${directory}/motor-selection.json`));
  for(const row of selected)for(const [key,value] of Object.entries(row.selected))if(Math.abs(motors.find(m=>m.entity===row.entity)[key]-value)>1e-14)throw Error(`Motor readback mismatch ${row.name}.${key}`);
  for(const row of rows)if(/^Anchor[ABCD]/.test(row.name)&&row.mu_native!==0)throw Error(`Unexpected guide friction ${row.name}`);
  fs.writeFileSync(`${directory}/intervention-native.json`,JSON.stringify({noise:noiseSpec,node_version:process.version,physics_engine:engine.physicsEngine,motors,selected,scene_sha256:createHash('sha256').update(sceneText).digest('hex'),baked_scene_sha256:bakedSceneHash,rows},null,2));
  fs.writeFileSync(`${directory}/initial-native.json`,JSON.stringify({step:1,dt,native_entities:Array.from(engine.entityRegion.data),motors}));
}
let recordingDrained = false, recordingError = null;
const started = performance.now();
const maxPhysicsSteps = Number(process.env.HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS || Infinity);
let recorder = process.env.AUTOCAL_REFERENCE_WS ? new FlightRecorder({ world,
  url: process.env.AUTOCAL_REFERENCE_WS, WebSocketClass: WebSocket, source: 'headless',
  recordPhysics: recordingMode !== 'events', nonFatal: true }) : null;
function disableRecorder(error) {
  const message = String(error?.message ?? error ?? 'Recording unavailable');
  recordingError ||= message;
  try { recorder?.fail(error instanceof Error ? error : new Error(message)); } catch {}
  recorder = null;
}
function attachRecorder() {
  world.setResource('sceneGeneration', epoch);
  world.setResource('researchClock', { generation: epoch, step: 0, time: 0, source: 'headless.researchClock' });
  world.setResource('timeScale', speed);
  if (!recorder) return;
  recorder.world = world;
  recorder.extendedReference = true;
  recorder.contextProvider = () => ({ backend: 'headless-js', physics_engine: world.getResource('physicsEngine'), physics_artifact: world.getResource('physicsArtifact'), scene, sourceText: sceneText,
    bakedSourceText: bakedScene, baked_scene_file: bakedSceneFile, baked_scene_sha256: bakedSceneHash,
    node_version: process.version, settings: Object.fromEntries(['gravity', 'dt', 'timeScale', 'enableLayering',
      'layeringFrictionEffectiveRadius', 'closedLoopMotorsEnabled', 'physicsSubsteps'].map(key => [key, world.getResource(key)])) });
  world.setResource('flightRecorder', recorder);
  world.registerSystem(recorder);
  if (recorder.socket?.readyState === 1) recorder.update(world, 0);
}
attachRecorder();
if (recorder) {
  try { await recorder.ensureConnected(recorder.url); }
  catch (error) { disableRecorder(error); }
}
const events = fs.openSync(`${directory}/events.jsonl`, 'a');
function event(type, values = {}) {
  fs.writeSync(events, JSON.stringify({ type, epoch, step, sim_time_s: step * dt, ...values }) + '\n');
}
function status() {
  return { backend: 'headless-js', physics_engine: world.getResource('physicsEngine'), physics_artifact: world.getResource('physicsArtifact'), step, epoch, total_steps: totalSteps, dt_s: dt,
    simulated_s: totalSteps * dt, collector_time_s: collectorTime, wall_s: (performance.now() - started) / 1000,
    queue_length: remote.getQueueLength(), error: failure, recording_drained: recordingDrained,
    recording_error: recordingError,
    baked_scene_file: bakedSceneFile, baked_scene_sha256: bakedSceneHash,
    settings: { enableLayering: world.getResource('enableLayering') === true,
      closedLoopMotorsEnabled: world.getResource('closedLoopMotorsEnabled') === true, physicsSubsteps, timeScale: speed } };
}
async function advance(seconds = 0, drain = false) {
  if (!Number.isFinite(seconds) || seconds < 0) throw new Error('Advance seconds must be finite and nonnegative');
  // Collector waits are simulation seconds, as in createBrowserClock. This
  // headless world already steps as fast as possible; playback speed must not
  // lengthen force trials or change their polling and settling windows.
  const count = Math.max(Math.ceil(seconds / dt - 1e-9), drain ? remote.getQueueLength() : 0);
  for (let i = 0; i < count; i++) {
    if (stopped || failure) throw new Error(failure ?? 'Simulation stopped');
    if (totalSteps >= maxPhysicsSteps) {
      failure = 'Physics step budget exhausted';
      throw new Error(failure);
    }
    if (recorder) {
      try {
        if (!recorder.socket || recorder.socket.readyState !== 1) throw new Error(recorder.error || 'Flight recorder disconnected');
        if (!recorder.readyForStep()) await recorder.drain(10000, { all: false });
      } catch (error) { disableRecorder(error); }
    }
    world.update(dt); step++; totalSteps++; collectorTime += dt;
    if (step === 1) auditNative();
    world.setResource('researchClock', { generation: epoch, step, time: step * dt, source: 'headless.researchClock' });
    if (totalSteps % 1000 === 0) fs.writeFileSync(`${directory}/clock.json`, JSON.stringify(status()));
    if (i % 50 === 49) await yieldStep();
  }
}
const bridge = createGcodeBridge({ server: rrfUrl, wsPort: Number(wsPort), quiet: true, encoderTimeoutMs: 120000 });
// Use the browser's actual command dispatch and encoder conversion.
let response;
class ObservationSocket {
  static OPEN = 1;
  readyState = 1;
  addEventListener() {}
  send(value) { response = JSON.parse(value); }
}
let controller;
function attachController() {
  controller = createExternalCommandController({ world, url: 'ws://observation', WebSocketCtor: ObservationSocket,
    featureFlags: {
      setPhysicsSubstepsState(count) { physicsSubsteps = setPhysicsSubsteps(world, count); },
      get lineLayeringEnabled() { return lineLayeringEnabled; },
      setLineLayeringEnabledState(enabled) {
        lineLayeringEnabled = enabled;
        setLineLayeringFeatureFlags(world, enabled);
      },
      setClosedLoopMotorsEnabledState(enabled) {
        closedLoopMotorsEnabled = enabled;
        setClosedLoopMotorFeatureFlags(world, enabled);
      },
    },
    commands: {
      getRemoteSystem: () => remote,
      pushExternalCommands(batch) { batch.forEach(command => remote.addCommand(command)); return true; },
      handleUserReset() {
        epoch++; step = 0; buildWorld();
        attachRecorder();
        attachController();
      },
      applyTimeScaleChange(value) { speed = value; world.setResource('timeScale', speed); },
    }, runtime: { resume() {} }, logger: { info() {}, log() {}, warn() {} },
  });
  controller.connect();
}
attachController();
let delivered = Promise.resolve();
async function handle(payload) {
  event('bridge_payload', { payload });
  if (payload.type === 'encoder_request' && payload.axes.length) await advance(0, true);
  controller.handlePayload(payload);
  if (payload.type === 'encoder_request') {
    if (response.anglesDeg.some(value => !Number.isFinite(value))) throw new Error('Non-finite encoder observation');
    const cleanAngles = response.anglesDeg.slice();
    if (noiseSpec.sigma_deg > 0) response.anglesDeg = response.anglesDeg.map(value => value + noiseSpec.sigma_deg * normal());
    event('sensor_noise', { request_id: payload.requestId, axes: payload.axes, clean_angles_deg: cleanAngles, noisy_angles_deg: response.anglesDeg, sigma_deg: noiseSpec.sigma_deg, seed: noiseSpec.noise_seed });
    event('encoder_response', { request_id: payload.requestId, axes: payload.axes, angles_deg: response.anglesDeg });
    socket.send(JSON.stringify(response));
  }
}
const socket = new WebSocket(`ws://127.0.0.1:${wsPort}`);
await new Promise((resolve, reject) => { socket.once('open', resolve); socket.once('error', reject); });
socket.on('message', data => {
  delivered = delivered.then(() => handle(JSON.parse(data))).catch(error => { failure = error.message; });
});
let busy = false;
const server = http.createServer(async (request, reply) => {
  reply.setHeader('Content-Type', 'application/json');
  if (busy) { reply.writeHead(409).end(JSON.stringify({ error: 'Simulation is busy' })); return; }
  busy = true;
  try {
    let text = '';
    for await (const chunk of request) text += chunk;
    const args = text ? JSON.parse(text) : {};
    let result;
    if (failure) throw new Error(failure);
    if (recorder && (!recorder.socket || recorder.socket.readyState !== 1)) disableRecorder(recorder.error || 'Flight recorder disconnected');
    if (request.url === '/gcode') {
      const commandId = `${process.pid}:${totalSteps}:${Date.now()}`;
      event('gcode_send', { command_id: commandId, line: args.line });
      recorder?.recordEvent('service_gcode_send', { commandId, line: args.line });
      // Drain outside RRF's short encoder resolver timeout before its M569.3 query.
      if (/^M569\.3\b/i.test(args.line)) await advance(0, true);
      result = await bridge.sendGcodeLine(args.line, { timeout: 120000 });
      event('gcode_reply', { command_id: commandId, result });
      if (/^Error:/im.test(result?.reply ?? '')) throw new Error(result.reply);
      await bridge.sendEncoderRequest([], 120000); // barrier for all preceding CAN payloads
      recorder?.recordEvent('service_gcode_reply', { commandId, result });
    } else if (request.url === '/advance') {
      await delivered;
      await advance(args.seconds);
    } else if (request.url === '/payload') {
      bridge.broadcast(args.payload);
      result = await bridge.sendEncoderRequest([], 120000);
    } else if (request.url !== '/status') throw new Error('Unknown operation');
    const state = status();
    fs.writeFileSync(`${directory}/clock.json`, JSON.stringify(state, null, 2));
    reply.end(JSON.stringify({ ...state, result }));
  } catch (error) {
    reply.writeHead(500).end(JSON.stringify({ error: error.message }));
  } finally { busy = false; }
});
server.listen(Number(apiPort), '127.0.0.1');
function writeTerminalNative() {
  const engine = world.getResource('cableSolverBackend');
  if (engine?.world !== world || !engine.entityRegion) return;
  engine.refreshMemory();
  const a=engine.entityRegion.data;
  const entities=engine.ids.map((id,i)=>{const o=i*160;return {id,name:world.getComponent(id,SceneEntityInfoComponent)?.name,position_m:Array.from(a.slice(o,o+3)),quaternion_xyzw:Array.from(a.slice(o+3,o+7)),velocity_m_s:Array.from(a.slice(o+69,o+72)),angular_velocity_rad_s:Array.from(a.slice(o+72,o+75)),encoder_rad:a[o+66],torque_mode:!!a[o+57],commanded_angle_rad:a[o+107],target_torque_nm:a[o+114],closed_loop:!!a[o+113],mu:a[o+105],radius_m:a[o+104]};});
  const paths=engine.pathIds.map((id,i)=>{const p=engine.transport.path(i),n=p[0];return {id,stiffness_n_m:p[2],total_rest_length_m:p[7],link_types:Array.from({length:n+1},(_,j)=>p[16+j*5]),stored_m:Array.from({length:n+1},(_,j)=>p[16+j*5+2]),joints:Array.from({length:n},(_,j)=>{const o=16+(n+1)*5+j*16,pa=Array.from(p.slice(o+4,o+7)),pb=Array.from(p.slice(o+7,o+10));return {id:p[o],entity_a:engine.ids[p[o+1]],entity_b:engine.ids[p[o+2]],length_m:Math.hypot(...pa.map((x,k)=>x-pb[k])),rest_length_m:p[o+3],point_a_m:pa,point_b_m:pb,constraint_lambda:p[o+10],constraint_force_n:p[o+11],transferred_constraint_force_n:p[o+12],constraint_force_vector_n:Array.from(p.slice(o+13,o+16))};})};});
  fs.writeFileSync(`${directory}/terminal-native.json`,JSON.stringify({status:status(),node_version:process.version,units:'m, s, rad, N, Nm; simulated constraint forces, not measured tension',stage:'terminal after collector cleanup, final stop commands may already zero motor targets; inspect command journal for pre-stop command',entities,paths},null,2));
}
async function close() {
  if (stopped) return;
  stopped = true;
  try {
    await delivered;
    if (recorder) {
      try {
        recorder.recordEvent('service_stopped', status());
        await recorder.drain(10000, { wallTimeoutMs: 10000 });
        recordingDrained = true;
        recorder.disconnect();
      } catch (error) { disableRecorder(error); }
    }
  } catch (error) { failure = error.message; }
  fs.writeFileSync(`${directory}/clock.json`, JSON.stringify(status(), null, 2));
  writeTerminalNative();
  event('service_stopped'); fs.closeSync(events);
  socket.close(); bridge.close(); server.close();
  process.exit(failure ? 1 : 0);
}
process.once('SIGTERM', close);
process.once('SIGINT', close);
