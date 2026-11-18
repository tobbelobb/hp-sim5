// Reproducer: a closed flight-recorder socket silently disables the whole
// external command/encoder protocol.
//
// Expected: the external bridge keeps working when the optional Rerun recording
//           stops; the protocol handler must not depend on recorder health.
// Actual:   FlightRecorder.recordEvent() throws once `error` is set. The
//           controller calls it at the top of handlePayload() (before dispatch),
//           so every command, encoder request and position-trace message is
//           dropped; the socket layer logs one console warning and continues.
// Exit 0 = expected behaviour, exit 1 = defect reproduced.
//
// Run: node repro_recorder_blocks_protocol.mjs
import { createProbe, StubSocket, StubWorld } from './harness.mjs';
import { FlightRecorder } from '../../../../hp-sim-3d/app/flightRecorder.js';

const world = new StubWorld();
const recorder = new FlightRecorder({ world, WebSocketClass: StubSocket }); // appBootstrap defaults
recorder.extendedReference = true;
world.setResource('flightRecorder', recorder);

const queue = [];
const remote = { addCommand: (c) => queue.push(c), getQueueLength: () => queue.length,
  clearCommandQueue: () => { queue.length = 0; } };
const probe = createProbe({
  world,
  commands: { getRemoteSystem: () => remote, pushExternalCommands(b) { b.forEach(c => remote.addCommand(c)); return true; } },
});
probe.controller.connect();
const bridge = StubSocket.instances.at(-1);
bridge.open();
queue.length = 0; // drop the connect-time Rebase bookkeeping

const failures = [];
const measure = (label, fn) => {
  const before = queue.length;
  fn();
  return { commandsAccepted: queue.length - before };
};

// Control: recording is healthy and acknowledged.
recorder.connect();
const receiver = StubSocket.instances.at(-1);
receiver.open();
receiver.deliver({ type: 'ack' });
const control = measure('control', () => {
  bridge.deliver({ type: 'command', command: { type: 'Move', B: 0.02 } });
});
const controlResponses = () => bridge.sent.filter(p => p.type === 'encoder_response');
bridge.deliver({ type: 'encoder_request', requestId: 100, axes: ['B'] });
const controlAnswered = controlResponses().some(r => r.requestId === 100);

// The Rerun receiver goes away (script stopped, port taken, laptop slept, ...).
receiver.close();
const recorderError = recorder.error;

const afterCommand = measure('after', () => {
  bridge.deliver({ type: 'command', command: { type: 'Move', B: 0.05 } });
});
bridge.deliver({ type: 'encoder_request', requestId: 200, axes: ['B'] });
bridge.deliver({ type: 'encoder_request', requestId: 201, axes: [] }); // clock poll
const answered = new Set(controlResponses().map(r => r.requestId));
const warnings = probe.log.order.filter(o => String(o).startsWith('warn:')).length;

const result = {
  recorderError,
  control: { commandsAccepted: control.commandsAccepted, encoderRequestAnswered: controlAnswered },
  afterRecorderLoss: {
    commandsAccepted: afterCommand.commandsAccepted,
    encoderRequestAnswered: answered.has(200),
    clockRequestAnswered: answered.has(201),
    consoleWarnings: warnings,
    bridgeWasTold: false,
  },
};

console.log(JSON.stringify(result, null, 2));

if (control.commandsAccepted !== 1 || !controlAnswered) {
  console.error('SETUP FAILED: control traffic was not accepted before the recorder loss');
  process.exit(2);
}
if (result.afterRecorderLoss.commandsAccepted !== 1 || !result.afterRecorderLoss.encoderRequestAnswered) {
  console.error('DEFECT: after the recording socket closed, the external protocol dropped every command '
    + 'and never answered the encoder request (only a console warning).');
  process.exit(1);
}
console.log('OK: external protocol unaffected by recorder loss');
