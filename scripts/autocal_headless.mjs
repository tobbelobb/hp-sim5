// Persistent production JS world and RRF bridge owned by autocal.py.
import fs from 'node:fs';
import http from 'node:http';
import { setImmediate as yieldStep } from 'node:timers/promises';
import WebSocket from 'ws';
import { createHeadlessWorld } from '../hp-sim-3d/app/headlessWorld.js';
import { bakeCableSceneUsdaSource } from '../src/js/usd/cable_scene_baker.js';
import { createExternalCommandController } from '../hp-sim-3d/app/externalCommandSocket.js';
import { createGcodeBridge } from '../integrations/rrf/rrfSimulatorBridge.mjs';
import { FlightRecorder } from '../hp-sim-3d/app/flightRecorder.js';

const [scene, rrfUrl, wsPort, apiPort, directory] = process.argv.slice(2);
const sceneText = fs.readFileSync(scene, 'utf8');
fs.writeFileSync(`${directory}/baked-scene.usda`, bakeCableSceneUsdaSource(sceneText).source);
let { world, remote, dt } = createHeadlessWorld(sceneText);
let step = 0, epoch = 0, totalSteps = 0, speed = 1, collectorTime = 0, stopped = false, failure = null;
const started = performance.now();
const recorder = process.env.AUTOCAL_REFERENCE_WS ? new FlightRecorder({ world,
  url: process.env.AUTOCAL_REFERENCE_WS, WebSocketClass: WebSocket, source: 'headless' }) : null;
function attachRecorder() {
  world.setResource('sceneGeneration', epoch);
  world.setResource('researchClock', { generation: epoch, step: 0, time: 0, source: 'headless.researchClock' });
  world.setResource('timeScale', speed);
  if (!recorder) return;
  recorder.world = world;
  recorder.extendedReference = true;
  recorder.contextProvider = () => ({ backend: 'headless-js', scene, sourceText: sceneText,
    node_version: process.version, settings: Object.fromEntries(['gravity', 'dt', 'timeScale', 'enableLayering',
      'layeringFrictionEffectiveRadius', 'closedLoopMotorsEnabled'].map(key => [key, world.getResource(key)])) });
  world.setResource('flightRecorder', recorder);
  world.registerSystem(recorder);
  if (recorder.socket?.readyState === 1) recorder.update(world, 0);
}
attachRecorder();
if (recorder) await recorder.ensureConnected(recorder.url);
const events = fs.openSync(`${directory}/events.jsonl`, 'a');
function event(type, values = {}) {
  fs.writeSync(events, JSON.stringify({ type, epoch, step, sim_time_s: step * dt, ...values }) + '\n');
}
function status() {
  return { backend: 'headless-js', step, epoch, total_steps: totalSteps, dt_s: dt,
    simulated_s: totalSteps * dt, collector_time_s: collectorTime, wall_s: (performance.now() - started) / 1000,
    queue_length: remote.getQueueLength(), error: failure };
}
async function advance(seconds = 0, drain = false) {
  if (!Number.isFinite(seconds) || seconds < 0) throw new Error('Advance seconds must be finite and nonnegative');
  const count = Math.max(Math.ceil(seconds * speed / dt - 1e-9), drain ? remote.getQueueLength() : 0);
  for (let i = 0; i < count; i++) {
    if (stopped || failure) throw new Error(failure ?? 'Simulation stopped');
    if (recorder) {
      if (!recorder.socket || recorder.socket.readyState !== 1) throw new Error(recorder.error || 'Flight recorder disconnected');
      if (!recorder.readyForStep()) await recorder.drain(10000, { all: false });
    }
    world.update(dt); step++; totalSteps++; collectorTime += dt / speed;
    world.setResource('researchClock', { generation: epoch, step, time: step * dt, source: 'headless.researchClock' });
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
    commands: {
      getRemoteSystem: () => remote,
      pushExternalCommands(batch) { batch.forEach(command => remote.addCommand(command)); return true; },
      handleUserReset() {
        ({ world, remote, dt } = createHeadlessWorld(sceneText)); epoch++; step = 0;
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
    if (recorder && (!recorder.socket || recorder.socket.readyState !== 1)) throw new Error(recorder.error || 'Flight recorder disconnected');
    if (request.url === '/gcode') {
      const commandId = `${process.pid}:${totalSteps}:${Date.now()}`;
      recorder?.recordEvent('service_gcode_send', { commandId, line: args.line });
      // Drain outside RRF's short encoder resolver timeout before its M569.3 query.
      if (/^M569\.3\b/i.test(args.line)) await advance(0, true);
      result = await bridge.sendGcodeLine(args.line, { timeout: 120000 });
      if (/^Error:/im.test(result?.reply ?? '')) throw new Error(result.reply);
      await bridge.sendEncoderRequest([], 120000); // barrier for all preceding CAN payloads
      recorder?.recordEvent('service_gcode_reply', { commandId, result });
    } else if (request.url === '/advance') {
      await delivered;
      await advance(args.seconds);
    } else if (request.url === '/payload') {
      bridge.broadcast(args.payload);
      await bridge.sendEncoderRequest([], 120000);
    } else if (request.url !== '/status') throw new Error('Unknown operation');
    const state = status();
    fs.writeFileSync(`${directory}/clock.json`, JSON.stringify(state, null, 2));
    reply.end(JSON.stringify({ ...state, result }));
  } catch (error) {
    reply.writeHead(500).end(JSON.stringify({ error: error.message }));
  } finally { busy = false; }
});
server.listen(Number(apiPort), '127.0.0.1');
async function close() {
  if (stopped) return;
  stopped = true;
  try {
    await delivered;
    if (recorder) {
      recorder.recordEvent('service_stopped', status());
      await recorder.drain(10000, { wallTimeoutMs: 10000 });
      recorder.disconnect();
    }
  } catch (error) { failure = error.message; }
  fs.writeFileSync(`${directory}/clock.json`, JSON.stringify(status(), null, 2));
  event('service_stopped'); fs.closeSync(events);
  socket.close(); bridge.close(); server.close();
  process.exit(failure ? 1 : 0);
}
process.once('SIGTERM', close);
process.once('SIGINT', close);
