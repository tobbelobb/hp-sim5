// E2: does a failed flight recorder block the external command path?
// Uses the real FlightRecorder + real createExternalCommandController + stub socket.
// exit 0 = diagnostic; prints a JSON record.
import { createProbe, StubSocket, StubWorld } from './harness.mjs';
import { FlightRecorder } from '../../../../hp-sim-3d/app/flightRecorder.js';

const out = { cases: [] };

function makeRemote() {
  const q = [];
  return { q, addCommand(c) { q.push(c); }, getQueueLength() { return q.length; }, clearCommandQueue() { q.length = 0; } };
}

function setup() {
  const world = new StubWorld();
  const recorder = new FlightRecorder({ world, WebSocketClass: StubSocket });
  recorder.extendedReference = true;
  world.setResource('flightRecorder', recorder);
  const remote = makeRemote();
  const probe = createProbe({
    world,
    commands: { getRemoteSystem: () => remote, pushExternalCommands(b) { b.forEach(c => remote.addCommand(c)); return true; } },
  });
  probe.controller.connect();
  probe.socket = StubSocket.instances.at(-1);
  probe.socket.open();
  return { world, recorder, remote, probe };
}

// --- case A: healthy recorder, no connection ever attempted -----------------
{
  const { recorder, remote, probe } = setup();
  probe.socket.deliver({ type: 'command', command: { type: 'Move', B: 1 } });
  out.cases.push({ case: 'A_healthy_never_connected', recorderError: recorder.error,
    commandsAccepted: remote.q.length, warnings: probe.log.order.filter(o => String(o).startsWith('warn:')) });
}

// --- case B: recorder connected then receiver dies mid-run ------------------
{
  const { recorder, remote, probe } = setup();
  recorder.connect();
  const recSocket = StubSocket.instances.at(-1);
  recSocket.open();
  recSocket.deliver({ type: 'ack' });
  probe.socket.deliver({ type: 'command', command: { type: 'Move', B: 2 } });
  const acceptedWhileRecording = remote.q.length;
  recSocket.close(); // Rerun receiver goes away
  for (let i = 0; i < 5; i += 1) probe.socket.deliver({ type: 'command', command: { type: 'Move', B: 3 + i } });
  out.cases.push({ case: 'B_recorder_died_mid_run', recorderError: recorder.error,
    acceptedWhileRecording, acceptedAfterFailure: remote.q.length,
    warnings: probe.log.order.filter(o => String(o).startsWith('warn:')).length,
    lastWarning: probe.log.order.filter(o => String(o).startsWith('warn:')).at(-1) });
}

// --- case C: user clicked Rerun with no receiver (very common) --------------
{
  const { recorder, remote, probe } = setup();
  recorder.connect();
  const recSocket = StubSocket.instances.at(-1);
  recSocket.dispatchEvent(new Event('error'));
  recSocket.close();
  probe.socket.deliver({ type: 'command', command: { type: 'Move', B: 2 } });
  out.cases.push({ case: 'C_rerun_button_without_receiver', recorderError: recorder.error,
    commandsAccepted: remote.q.length,
    warnings: probe.log.order.filter(o => String(o).startsWith('warn:')).length });
}

// --- case D: same failure, but a simulation_settings message arrives --------
{
  const { recorder, remote, probe } = setup();
  recorder.connect();
  const recSocket = StubSocket.instances.at(-1);
  recSocket.open();
  recSocket.deliver({ type: 'ack' });
  recSocket.close();
  probe.socket.deliver({ type: 'command', command: { type: 'Move', B: 2 } }); // dropped
  const afterFailure = remote.q.length;
  probe.socket.deliver({ type: 'simulation_settings', solverIterations: 3 }); // clears recorder.error
  probe.socket.deliver({ type: 'command', command: { type: 'Move', B: 4 } });
  out.cases.push({ case: 'D_settings_message_recovers_path', recorderErrorAfterSettings: recorder.error,
    acceptedBeforeRecovery: afterFailure, acceptedAfterRecovery: remote.q.length,
    warnings: probe.log.order.filter(o => String(o).startsWith('warn:')).length });
}

// --- case E: control, recorder disconnected cleanly (no error) --------------
{
  const { recorder, remote, probe } = setup();
  recorder.connect();
  const recSocket = StubSocket.instances.at(-1);
  recSocket.open();
  recSocket.deliver({ type: 'ack' });
  recorder.disconnect();
  probe.socket.deliver({ type: 'command', command: { type: 'Move', B: 5 } });
  out.cases.push({ case: 'E_clean_disconnect', recorderError: recorder.error,
    commandsAccepted: remote.q.length,
    warnings: probe.log.order.filter(o => String(o).startsWith('warn:')).length });
}

// --- case F: nonFatal recorder (tests use it; production does not) ----------
{
  const world = new StubWorld();
  const recorder = new FlightRecorder({ world, WebSocketClass: StubSocket, nonFatal: true });
  recorder.extendedReference = true;
  world.setResource('flightRecorder', recorder);
  const remote = makeRemote();
  const probe = createProbe({ world, commands: { getRemoteSystem: () => remote,
    pushExternalCommands(b) { b.forEach(c => remote.addCommand(c)); return true; } } });
  probe.controller.connect();
  const s = StubSocket.instances.at(-1);
  s.open();
  recorder.connect();
  StubSocket.instances.at(-1).close();
  s.deliver({ type: 'command', command: { type: 'Move', B: 6 } });
  out.cases.push({ case: 'F_nonFatal_recorder', recorderError: recorder.error,
    commandsAccepted: remote.q.length, warnings: probe.log.order.filter(o => String(o).startsWith('warn:')).length });
}

// --- case G: the collector's actual traffic after a recorder failure ------
{
  const { world, recorder, remote, probe } = setup();
  recorder.connect();
  const recSocket = StubSocket.instances.at(-1);
  recSocket.open();
  recSocket.deliver({ type: 'ack' });
  recSocket.close();
  const traceCalls = [];
  world.setResource('renderSystem', { setPositionTraceEnabled: v => traceCalls.push(v), update() {} });

  probe.socket.deliver({ type: 'encoder_request', requestId: 1, axes: [] });     // clock poll
  probe.socket.deliver({ type: 'encoder_request', requestId: 2, axes: ['A'] });  // encoder read
  probe.socket.deliver({ type: 'position_trace_mode', enabled: true });
  probe.socket.deliver({ type: 'command', command: { type: 'Move', A: 0.1 } });
  const responses = probe.socket.sent.filter(p => p.type === 'encoder_response');
  out.cases.push({
    case: 'G_traffic_after_recorder_failure',
    recorderError: recorder.error,
    responsesSent: responses.length,
    answeredRequestIds: responses.map(r => r.requestId),
    unansweredRequestIds: [1, 2].filter(id => !responses.some(r => r.requestId === id)),
    traceModeApplied: traceCalls.length > 0,
    commandsAccepted: remote.q.length,
    warnings: probe.log.order.filter(o => String(o).startsWith('warn:')).length,
  });
}

// --- case H: default page, extendedReference never enabled ----------------
{
  const world = new StubWorld();
  const recorder = new FlightRecorder({ world, WebSocketClass: StubSocket }); // no ?extended_autocal=1
  world.setResource('flightRecorder', recorder);
  const remote = makeRemote();
  const probe = createProbe({ world, commands: { getRemoteSystem: () => remote,
    pushExternalCommands(b) { b.forEach(c => remote.addCommand(c)); return true; } } });
  probe.controller.connect();
  const s = StubSocket.instances.at(-1);
  s.open();
  recorder.connect();
  StubSocket.instances.at(-1).close();
  const before = remote.q.length;
  s.deliver({ type: 'command', command: { type: 'Move', B: 7 } });
  out.cases.push({ case: 'H_default_page_error_before_extended_reference_check',
    extendedReference: recorder.extendedReference ?? false, recorderError: recorder.error,
    commandsAccepted: remote.q.length - before,
    warnings: probe.log.order.filter(o => String(o).startsWith('warn:')).length });
}

console.log(JSON.stringify(out, null, 2));
