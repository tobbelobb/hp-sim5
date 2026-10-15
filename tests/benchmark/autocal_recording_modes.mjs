// Frozen production collection replay; orchestrated by autocal_recording_modes.py.
import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { setImmediate } from 'node:timers/promises';
import WebSocket from 'ws';
import { createHeadlessWorld } from '../../hp-sim-3d/app/headlessWorld.js';
import { createExternalCommandController } from '../../hp-sim-3d/app/externalCommandSocket.js';
import { FlightRecorder } from '../../hp-sim-3d/app/flightRecorder.js';
import { observe } from '../../scripts/profile_headless.mjs';

const [input, directory, mode, url] = process.argv.slice(2);
const contract = JSON.parse(fs.readFileSync(`${input}/contract.json`));
const scene = fs.readFileSync(`${input}/scene.usda`, 'utf8');
const trace = fs.readFileSync(`${input}/events.jsonl`, 'utf8').trim().split('\n').map(JSON.parse);
const finiteJson = value => JSON.stringify(value, (key, item) => {
  if (typeof item === 'number' && !Number.isFinite(item)) throw new Error(`Non-finite ${key}`);
  return item;
});
const createWorld = () => createHeadlessWorld(scene, { lineLayeringEnabled: false });
// Equal cold-process JIT warmup in a separate world, excluded from replay time.
const warm = createWorld();
for (let i = 0; i < 5000; i++) warm.world.update(warm.dt);
warm.world.getResource('cableSolverBackend')?.dispose?.();
const { world, remote, dt, bakedScene } = createWorld();
let step = 0, response;
const encoderRequests = new Map();
const clock = () => {
  world.setResource('sceneGeneration', 0);
  world.setResource('researchClock', { generation: 0, step, time: step * dt, source: 'headless.researchClock' });
};
clock();
world.setResource('timeScale', 1);
const recorder = mode === 'none' ? null : new FlightRecorder({ world, source: 'headless',
  url, WebSocketClass: WebSocket, recordPhysics: mode !== 'events' });
if (recorder) {
  recorder.extendedReference = true;
  recorder.contextProvider = () => ({ benchmark: contract, sourceText: scene, bakedSourceText: bakedScene });
  world.setResource('flightRecorder', recorder);
  world.registerSystem(recorder);
  await recorder.ensureConnected(url);
}
class ObservationSocket {
  static OPEN = 1;
  readyState = 1;
  addEventListener() {}
  send(value) { response = JSON.parse(value); }
}
const controller = createExternalCommandController({ world, url: 'ws://benchmark', WebSocketCtor: ObservationSocket,
  commands: {
    getRemoteSystem: () => remote,
    pushExternalCommands(batch) { batch.forEach(command => remote.addCommand(command)); return true; },
    handleUserReset() { throw new Error('Freeze a single generation without resets/layering changes'); },
    applyTimeScaleChange(value) { world.setResource('timeScale', value); },
  }, runtime: { resume() {} }, logger: { info() {}, log() {}, warn() {} },
});
controller.connect();
const checkpoints = [], encoders = [], schedule = createHash('sha256');
const journal = fs.openSync(`${directory}/events.jsonl`, 'wx');
let maxOriginalEncoderErrorDeg = 0, commands = 0;
function checkpoint() { checkpoints.push({ step, snapshot: observe(world, remote) }); }
const started = performance.now();
checkpoint();
for (const event of [...trace, { type: 'benchmark_end', step: contract.steps }]) {
  if (event.step < step || event.epoch && event.epoch !== 0) throw new Error('Nonmonotonic or multi-epoch trace');
  while (step < event.step) {
    if (recorder) {
      if (!recorder.socket || recorder.socket.readyState !== 1) throw new Error(recorder.error || 'Recorder disconnected');
      if (!recorder.readyForStep()) await recorder.drain(10000, { all: false });
    }
    world.update(dt); step++; clock();
    if (world.getResource('errorState')?.hasError || world.getResource('pauseState')?.paused) throw new Error(`Physics stopped at ${step}`);
    if (step % 1000 === 0) checkpoint();
    if (step % 50 === 0) await setImmediate();
  }
  fs.writeSync(journal, JSON.stringify(event) + '\n');
  if (event.type === 'bridge_payload') {
    if (event.payload.type === 'reset' || typeof event.payload.lineLayeringEnabled === 'boolean') throw new Error('Unsupported reset/layering change');
    schedule.update(JSON.stringify({ step, payload: event.payload }));
    commands += event.payload.type === 'command' ? 1 : event.payload.commands?.length ?? 0;
    if (event.payload.type === 'encoder_request') encoderRequests.set(event.payload.requestId, event.payload);
    else controller.handlePayload(event.payload);
  } else if (event.type === 'encoder_response') {
    // The live service can drain queued motion between request arrival and read.
    const request = encoderRequests.get(event.request_id);
    if (!request) throw new Error('Encoder response has no matching request');
    encoderRequests.delete(event.request_id);
    controller.handlePayload(request);
    if (JSON.stringify(response.axes) !== JSON.stringify(event.axes)) throw new Error('Encoder axes changed');
    if (event.axes.length) {
      encoders.push({ step, axes: event.axes, angles: response.anglesDeg });
      maxOriginalEncoderErrorDeg = Math.max(maxOriginalEncoderErrorDeg,
        ...event.angles_deg.map((value, i) => Math.abs(value - response.anglesDeg[i])));
    }
  }
}
checkpoint();
const executionMs = performance.now() - started;
if (recorder) {
  recorder.recordEvent('service_stopped', { step, queue_length: remote.getQueueLength() });
  await recorder.drain(10000, { wallTimeoutMs: 120000 });
  recorder.disconnect();
}
fs.closeSync(journal);
const replayAndDrainMs = performance.now() - started;
const observations = finiteJson({ checkpoints, encoders });
fs.writeFileSync(`${directory}/checkpoints.json`, observations + '\n');
fs.writeFileSync(`${directory}/result.json`, finiteJson({ mode, steps: step, dt, commands,
  encoderReads: encoders.length, maxOriginalEncoderErrorDeg, executionMs, replayAndDrainMs,
  checkpointSha256: createHash('sha256').update(observations).digest('hex'),
  scheduleSha256: schedule.digest('hex'), bakedSceneSha256: createHash('sha256').update(bakedScene).digest('hex'),
  physicsEngine: world.getResource('physicsEngine'), physicsArtifact: world.getResource('physicsArtifact'),
  pendingPhysics: recorder?.pending ?? 0, pendingEvents: recorder?.pendingEvents ?? 0 }) + '\n');
world.getResource('cableSolverBackend')?.dispose?.();
