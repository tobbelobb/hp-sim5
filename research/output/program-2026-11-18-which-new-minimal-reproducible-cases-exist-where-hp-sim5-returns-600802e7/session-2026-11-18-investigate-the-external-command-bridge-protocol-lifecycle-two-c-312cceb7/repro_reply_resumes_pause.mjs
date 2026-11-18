// Reproducer: payloads that carry no command un-pause the simulation.
//
// The RRF/Klipper bridge broadcasts {type:'reply', gcode, reply} for *every*
// G-code line it forwards, including pure status queries (M105, M114, M115).
// handlePayload()'s final fallthrough calls runtime.resume() (runner.js:
// resumeSimulation -> pauseState.paused = false) for any payload it does not
// explicitly recognise. A paused simulator therefore starts stepping because a
// client asked for a temperature or position readout.
//
// Expected: only motion-bearing payloads (and explicit control messages that
//           say so) change the run state. A reply to a query, an unknown type,
//           or an empty object must leave pause alone.
// Actual:   every non-control payload resumes.
// Exit 0 = expected behaviour, exit 1 = defect reproduced.
//
// Run: node repro_reply_resumes_pause.mjs
import { createProbe, StubSocket, StubWorld } from './harness.mjs';

// Same state transition as hp-sim-3d/app/runner.js resumeSimulation().
const pauseState = { paused: true };
let logicalResumes = 0;
const runtime = {
  resume() {
    if (!pauseState.paused) return;
    pauseState.paused = false;
    logicalResumes += 1;
  },
};

const queue = [];
const remote = { addCommand: (c) => queue.push(c), getQueueLength: () => queue.length };
const probe = createProbe({
  world: new StubWorld(),
  runtime,
  commands: { getRemoteSystem: () => remote, pushExternalCommands(b) { b.forEach(c => remote.addCommand(c)); return true; } },
});
probe.controller.connect();
const bridge = StubSocket.instances.at(-1);
bridge.open();
queue.length = 0;
pauseState.paused = true; // the user pressed Pause (or started the page paused)

const results = [];
const send = (label, payload, expectedPaused) => {
  pauseState.paused = true;
  logicalResumes = 0;
  bridge.deliver(payload);
  results.push({ label, expectedPaused, actualPaused: pauseState.paused, resumed: logicalResumes === 1 });
};

send('reply-to-M105', { type: 'reply', gcode: 'M105', reply: 'ok T:20.0 /0.0 B:20.0' }, true);
send('reply-empty', { type: 'reply', reply: '' }, true);
send('unknown-type', { type: 'heartbeat', seq: 4 }, true);
send('empty-object', {}, true);
send('json-array', [1, 2, 3], true);
send('position_trace_mode', { type: 'position_trace_mode', enabled: true }, true);
send('command-Move', { type: 'command', command: { type: 'Move', B: 0.02 } }, false);

console.log(JSON.stringify({ pauseStateModel: 'runner.js resumeSimulation', results }, null, 2));

const wrong = results.filter(r => r.actualPaused !== r.expectedPaused);
if (wrong.length) {
  console.error(`DEFECT: ${wrong.length} payload(s) without motion changed the run state: `
    + wrong.map(r => r.label).join(', '));
  process.exit(1);
}
console.log('OK: run state only changes on command payloads');
