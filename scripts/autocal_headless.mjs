// Persistent production Rust/WASM world and direct in-process RRF bridge owned by autocal.py.
import { setSolverIterations } from '../hp-sim-3d/app/solver-iterations.js';
import { setPhysicsSubsteps } from '../hp-sim-3d/app/physics-substeps.js';
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { setImmediate as yieldStep } from 'node:timers/promises';
import WebSocket from 'ws';
import { createHash } from 'node:crypto';
import { createHeadlessWorld } from '../hp-sim-3d/app/headlessWorld.js';
import { setLineLayeringFeatureFlags } from '../hp-sim-3d/app/line-layering-flags.js';
import { setClosedLoopMotorFeatureFlags } from '../hp-sim-3d/app/closed-loop-flags.js';
import { createExternalCommandController } from '../hp-sim-3d/app/externalCommandSocket.js';
import { RrfHttpBridge } from '../integrations/rrf/rrfHttpBridge.js';
import { FlightRecorder } from '../hp-sim-3d/app/flightRecorder.js';
import { compileCableSolverWasm } from '../src/js/cable_joints_3d/wasm_cable_solver.js';
import { WasmPhysicsEngine } from '../src/js/cable_joints_3d/wasm_physics_engine.js';

const [scene, rrfUrl, _wsPort, apiPort, directory, initialLayering, recordingMode] = process.argv.slice(2);
const sceneText = fs.readFileSync(scene, 'utf8');
const clockPath = `${directory}/clock.json`;
const solverChoice = (process.env.HP_SIM5_PHYSICS_SOLVER || 'native').toLowerCase();
const repoRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
let wasmModule = null;
if (solverChoice === 'wasm') {
  const wasmPath = process.env.HP_SIM5_WASM_PATH || path.join(repoRoot, 'public/wasm/cable_solver.wasm');
  wasmModule = await compileCableSolverWasm(fs.readFileSync(wasmPath));
} else if (solverChoice !== 'native') {
  throw new Error(`Unsupported HP_SIM5_PHYSICS_SOLVER: ${solverChoice} (expected native or wasm)`);
}

let step = 0, epoch = 0, totalSteps = 0, speed = 1, collectorTime = 0, stopped = false, failure = null;
let lineLayeringEnabled = initialLayering === 'true', closedLoopMotorsEnabled = false, physicsSubsteps, solverIterations;
let world, remote, dt, bakedScene, bakedSceneFile, bakedSceneHash;
function buildWorld() {
  const cableSolverBackend = wasmModule ? new WasmPhysicsEngine(wasmModule) : undefined;
  ({ world, remote, dt, bakedScene } = createHeadlessWorld(sceneText, {
    lineLayeringEnabled, closedLoopMotorsEnabled, physicsSubsteps, solverIterations, cableSolverBackend,
  }));
  bakedSceneFile = `baked-scene${epoch ? `-${epoch}` : ''}.usda`;
  fs.writeFileSync(`${directory}/${bakedSceneFile}`, bakedScene);
  bakedSceneHash = createHash('sha256').update(bakedScene).digest('hex');
}
buildWorld();
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
      'layeringFrictionEffectiveRadius', 'closedLoopMotorsEnabled', 'physicsSubsteps', 'solverIterations'].map(key => [key, world.getResource(key)])) });
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
const EVENT_FLUSH_BYTES = 65536;
let eventBuffer = [];
let eventBufferBytes = 0;
function flushEvents() {
  if (!eventBuffer.length) return;
  fs.writeSync(events, eventBuffer.join(''));
  eventBuffer = [];
  eventBufferBytes = 0;
}
function event(type, values = {}) {
  const line = JSON.stringify({ type, epoch, step, sim_time_s: step * dt, ...values }) + '\n';
  eventBuffer.push(line);
  eventBufferBytes += line.length;
  if (eventBufferBytes >= EVENT_FLUSH_BYTES) flushEvents();
}

function status() {
  return { backend: 'headless-js', physics_engine: world.getResource('physicsEngine'), physics_artifact: world.getResource('physicsArtifact'), step, epoch, total_steps: totalSteps, dt_s: dt,
    simulated_s: totalSteps * dt, collector_time_s: collectorTime, wall_s: (performance.now() - started) / 1000,
    queue_length: remote.getQueueLength(), error: failure, recording_drained: recordingDrained,
    recording_error: recordingError,
    baked_scene_file: bakedSceneFile, baked_scene_sha256: bakedSceneHash,
    settings: { enableLayering: world.getResource('enableLayering') === true,
      closedLoopMotorsEnabled: world.getResource('closedLoopMotorsEnabled') === true, physicsSubsteps: world.getResource('physicsSubsteps'), solverIterations: world.getResource('solverIterations'), timeScale: speed } };
}
let lastClockWriteMs = 0;
function writeClock(state = status(), { force = false } = {}) {
  const now = performance.now();
  if (!force && now - lastClockWriteMs < 150) return;
  lastClockWriteMs = now;
  // Readers poll this file while physics is advancing. Publish a complete snapshot.
  const temporary = `${clockPath}.tmp`;
  fs.writeFileSync(temporary, JSON.stringify(state));
  fs.renameSync(temporary, clockPath);
}
writeClock(status(), { force: true });

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
    world.setResource('researchClock', { generation: epoch, step, time: step * dt, source: 'headless.researchClock' });
    if (totalSteps % 1000 === 0) writeClock(status(), { force: true });
    if (i % 50 === 49) await yieldStep();
  }
}

// Use the browser's actual command dispatch and encoder conversion directly in-process
// without serializing every 2 ms command through a loopback WebSocket server.
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
      setSolverIterationsState(count) {
        const applied = setSolverIterations(world, count);
        solverIterations = count;
        return applied;
      },
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

let encoderRequestSeq = 1;
let currentGcode = null;

function dispatchPayloadSync(payload) {
  event('bridge_payload', { payload });
  controller.handlePayload(payload);
  if (payload.type === 'encoder_request') {
    if (response.anglesDeg.some(value => !Number.isFinite(value))) throw new Error('Non-finite encoder observation');
    event('encoder_response', { request_id: payload.requestId, axes: payload.axes, angles_deg: response.anglesDeg });
    return response;
  }
  return null;
}

async function dispatchPayload(payload) {
  if (payload.type === 'encoder_request' && payload.axes.length) await advance(0, true);
  return dispatchPayloadSync(payload);
}

const rrfBridge = new RrfHttpBridge({
  baseUrl: rrfUrl.replace(/\/$/, ''),
  remoteSpoolSystem: {
    addCommand(command) {
      if (!command) return;
      dispatchPayloadSync({ type: 'command', command, gcode: currentGcode });
    },
  },
  encoderResolver: async ({ axes }) => {
    const res = await dispatchPayload({ type: 'encoder_request', requestId: encoderRequestSeq++, axes });
    return Array.isArray(res?.anglesDeg) ? res.anglesDeg : [];
  },
});

async function sendGcodeLine(line, options = {}) {
  const trimmed = line?.trim?.();
  if (!trimmed) return null;
  currentGcode = trimmed;
  try {
    const result = await rrfBridge.sendGCode(trimmed, options);
    dispatchPayloadSync({ type: 'reply', gcode: trimmed, reply: result.reply });
    return result;
  } finally {
    currentGcode = null;
  }
}

let pending = Promise.resolve();
const server = http.createServer(async (request, reply) => {
  reply.setHeader('Content-Type', 'application/json');
  // A collector can disconnect on Ctrl-C while its last advance is still
  // running. Serialize the following application behind that operation;
  // rejecting it as busy loses the selected calibration, and retrying a
  // mutating G-code request would leave its outcome ambiguous.
  const previous = pending;
  let release;
  pending = new Promise(resolve => { release = resolve; });
  await previous;
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
      result = await sendGcodeLine(args.line, { timeout: 120000 });
      event('gcode_reply', { command_id: commandId, result });
      if (/^Error:/im.test(result?.reply ?? '')) throw new Error(result.reply);
      recorder?.recordEvent('service_gcode_reply', { commandId, result });
    } else if (request.url === '/advance') {
      await advance(args.seconds);
    } else if (request.url === '/payload') {
      await dispatchPayload(args.payload);
      result = await dispatchPayload({ type: 'encoder_request', requestId: encoderRequestSeq++, axes: [] });
    } else if (request.url !== '/status') throw new Error('Unknown operation');
    const state = status();
    writeClock(state);
    reply.end(JSON.stringify({ ...state, result }));
  } catch (error) {
    reply.writeHead(500).end(JSON.stringify({ error: error.message }));
  } finally { release(); }
});
server.listen(Number(apiPort), '127.0.0.1');
async function close() {
  if (stopped) return;
  stopped = true;
  try {
    if (recorder) {
      try {
        recorder.recordEvent('service_stopped', status());
        await recorder.drain(10000, { wallTimeoutMs: 10000 });
        recordingDrained = true;
        recorder.disconnect();
      } catch (error) { disableRecorder(error); }
    }
  } catch (error) { failure = error.message; }
  writeClock(status(), { force: true });
  event('service_stopped');
  flushEvents();
  fs.closeSync(events);
  server.close();
  process.exit(failure ? 1 : 0);
}
process.once('SIGTERM', close);
process.once('SIGINT', close);
