// E1: what does the external command controller do with each inbound payload?
// Drives the real handlePayload through the real socket message path (JSON text).
// Prints one JSON row per case. exit 0 always (diagnostic table).
import { createProbe, StubSocket, StubWorld } from './harness.mjs';

StubSocket.reset();

const rows = [];
for (const [label, payload, opts = {}] of [
  ['command:single', { type: 'command', command: { type: 'Move', B: 1 } }],
  ['command:duplicate-twice', { type: 'command', command: { type: 'Move', B: 1 } }, { duplicate: true }],
  ['command:no-command-field', { type: 'command' }],
  ['command:null-command', { type: 'command', command: null }],
  ['command:string-command', { type: 'command', command: 'G1 X1' }],
  ['commands:array', { type: 'commands', commands: [{ type: 'Move', A: 1 }, { type: 'Move', B: 2 }] }],
  ['commands:with-nulls', { type: 'commands', commands: [null, { type: 'Move', A: 1 }, undefined] }],
  ['reply:status-query', { type: 'reply', gcode: 'M105', reply: 'ok T:20.0' }],
  ['reply:empty', { type: 'reply', reply: '' }],
  ['unknown-type', { type: 'no_such_message', value: 1 }],
  ['empty-object', {}],
  ['json-array', [1, 2, 3]],
  ['json-null', null],
  ['json-number', 42],
  ['reset', { type: 'reset' }],
  ['reset:with-command', { type: 'reset', command: { type: 'Move', B: 1 } }],
  ['set_speed_scale:25', { type: 'set_speed_scale', value: 25 }],
  ['set_speed_scale:0', { type: 'set_speed_scale', value: 0 }],
  ['set_speed_scale:string', { type: 'set_speed_scale', value: '25' }],
  ['encoder_request:no-requestId', { type: 'encoder_request', axes: [] }],
  ['encoder_request:no-axes', { type: 'encoder_request', requestId: 7 }],
  ['simulation_settings:empty', { type: 'simulation_settings' }],
  ['klipper_api_session_start', { type: 'klipper_api_session_start', clock_hz: 1000 }],
  ['malformed-json', '{"type":"command",'],
  ['json-string', '"command"'],
]) {
  const remote = { queue: [], addCommand(c) { this.queue.push(c); }, getQueueLength() { return this.queue.length; } };
  const probe = createProbe({
    world: new StubWorld({ recorder: null }),
    commands: { getRemoteSystem: () => remote, pushExternalCommands(batch) {
      batch.forEach(c => remote.addCommand(c)); return true; } },
  });
  probe.controller.connect();
  const socket = StubSocket.instances.at(-1);
  socket.open();
  probe.log.order.length = 0; // ignore the connect-time Rebase bookkeeping
  probe.log.pushed.length = 0;

  const before = { resume: probe.log.resume, reset: probe.log.reset, queue: remote.queue.length };
  socket.deliver(payload);
  if (opts.duplicate) socket.deliver(payload);
  const after = { resume: probe.log.resume, reset: probe.log.reset, queue: remote.queue.length };

  rows.push({
    label,
    queueDelta: after.queue - before.queue,
    applied: remote.queue.map(c => (typeof c === 'string' ? c : c?.type)),
    resumed: after.resume - before.resume,
    reset: after.reset - before.reset,
    speed: probe.log.timeScale.slice(),
    responses: socket.sent.filter(p => p.type === 'encoder_response').length,
    warnings: probe.log.order.filter(o => String(o).startsWith('warn:')),
    rebaseInjectedAtConnect: probe.log.order.includes('resume') ? undefined : undefined,
  });
}

// Connect-time behaviour, separate probe so the Rebase is visible.
const connectRemote = { queue: [], addCommand(c) { this.queue.push(c); }, getQueueLength() { return this.queue.length; } };
const cp = createProbe({
  world: new StubWorld(),
  commands: { getRemoteSystem: () => connectRemote, pushExternalCommands(b) { b.forEach(c => connectRemote.addCommand(c)); return true; } },
});
cp.controller.connect();
StubSocket.instances.at(-1).open();
const connectBehaviour = { afterOpenQueue: connectRemote.queue.map(c => c?.type) };

// Reconnect: two closes and two opens.
const s1 = StubSocket.instances.at(-1);
cp.controller.connect();
const socketCountBefore = StubSocket.instances.length;

console.log(JSON.stringify({
  note: 'rows come from real createExternalCommandController.handlePayload via a stub WebSocket',
  rows, connectBehaviour,
  socketCountBeforeReconnectAttempt: socketCountBefore,
  socketsCreated: StubSocket.instances.length,
}, null, 2));
