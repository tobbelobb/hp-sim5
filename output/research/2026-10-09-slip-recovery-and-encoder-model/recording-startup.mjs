import fs from 'node:fs';
import net from 'node:net';
import { spawn } from 'node:child_process';
import { once } from 'node:events';
import WebSocket from 'ws';
import { World } from '../../../src/js/cable_joints_3d/ecs.js';
import { FlightRecorder } from '../../../hp-sim-3d/app/flightRecorder.js';
import { createExternalCommandController } from '../../../hp-sim-3d/app/externalCommandSocket.js';
import { sendHpSimSettings } from '../../../autocal/control/primitives/encoder_utils.mjs';
import { createReferenceLogger } from '../../../autocal/control/primitives/extended_reference.mjs';

const directory = new URL('./recording-startup/', import.meta.url).pathname;
fs.mkdirSync(directory, { recursive: true });
const portServer = net.createServer();
await new Promise(resolve => portServer.listen(0, '127.0.0.1', resolve));
const port = portServer.address().port;
await new Promise(resolve => portServer.close(resolve));
const url = `ws://127.0.0.1:${port}`;
const child = spawn('.venv/bin/python', ['scripts/hangprinter_flight_recorder.py', '--extended-reference', '--no-viewer', '--port', String(port), '--output', directory]);
let output = '';
child.stdout.on('data', value => { output += value; });
child.stderr.on('data', value => { output += value; });
let recorder;
try {
  const deadline = Date.now() + 10000;
  while (!output.includes('Flight recorder:')) {
    if (Date.now() > deadline || child.exitCode !== null) throw new Error(`Receiver startup failed: ${output}`);
    await new Promise(resolve => setTimeout(resolve, 25));
  }
  const log = createReferenceLogger(url);
  log.emit('run_start', { test: 'automatic browser recorder' });
  await log.flush();
  const world = new World();
  world.setResource('sceneGeneration', 1);
  recorder = new FlightRecorder({ world, WebSocketClass: WebSocket });
  world.setResource('flightRecorder', recorder);
  let reply;
  class ObservationSocket {
    static OPEN = 1;
    readyState = 1;
    addEventListener() {}
    send(value) { reply(JSON.parse(value)); }
  }
  const controller = createExternalCommandController({ world, url: 'ws://observation', WebSocketCtor: ObservationSocket });
  controller.connect();
  const bridge = { broadcast: payload => controller.handlePayload(payload),
    sendEncoderRequest: () => new Promise(resolve => { reply = resolve; controller.handlePayload({ type: 'encoder_request', requestId: 1, axes: [] }); }) };
  const settings = await sendHpSimSettings(bridge, { closedLoopMotorsEnabled: true, lineLayeringEnabled: true, recordingUrl: url });
  const session = recorder.session;
  await recorder.ensureConnected(url);
  if (recorder.session !== session) throw new Error('Existing recording was restarted');
  log.emit('gcode_send', { line: 'M666', source: { file: 'recording-startup.mjs', line: 1 } });
  for (let step = 1; step <= 20; step++) {
    world.setResource('researchClock', { step, time: step * .002 });
    recorder.update(world, .002);
  }
  log.emit('collection_complete', {});
  await log.close();
  const drainDeadline = Date.now() + 5000;
  while (recorder.pending && Date.now() < drainDeadline) await new Promise(resolve => setTimeout(resolve, 10));
  if (recorder.pending) throw new Error('Physics samples did not drain');
  fs.writeFileSync(`${directory}/result.json`, JSON.stringify({ settings, session, sample_stride: recorder.sampleStride, pending: recorder.pending }, null, 2));
} finally {
  recorder?.disconnect();
  const closed = once(child, 'exit');
  child.kill('SIGINT');
  await closed;
  fs.writeFileSync(`${directory}/receiver.log`, output);
}
console.log('Automatic recording startup and existing-session reuse passed');
