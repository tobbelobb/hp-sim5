// Reproducer: silent command loss and an unenforced queue bound at the two
// protocol queues, plus Reset replaying commands parked before it.
//
// A. Bridge backlog (integrations/rrf/rrfSimulatorBridge.mjs, MAX_PENDING_WS_PAYLOADS
//    = 5000): payloads produced with no client attached are queued and the
//    oldest are evicted with no log line, no error and no counter.
// B. Browser backlog (externalCommandSocket.js, queueLimit = 5000): one message
//    may carry a `commands` array that is larger than the limit; the eviction
//    calculation (`splice(0, overflow)` on an empty queue) then transmits more
//    than `queueLimit`.
// C. Reset does not clear that browser backlog, and machineSceneController
//    .rebuildScene() ends with flushQueue(), so commands that arrived before a
//    Reset are applied to the fresh world after it.
//
// Exit 0 = expected behaviour, exit 1 = defect reproduced.
//
// Run (bridge part needs the in-process ws stand-in):
//   node --import ./ws-stub/ws-register.mjs repro_queue_loss_and_reset.mjs
import { createGcodeBridge } from '../../../../integrations/rrf/rrfSimulatorBridge.mjs';
import { createProbe, StubSocket, StubWorld, createStubRemote } from './harness.mjs';

const checks = [];

// --- A. bridge backlog overflow -------------------------------------------
{
  const logs = [];
  const realLog = console.log;
  console.log = (...args) => logs.push(args.join(' '));
  const ctx = createGcodeBridge({ wsPort: 8790, quiet: true });
  const SENT = 5003;
  for (let i = 0; i < SENT; i += 1) ctx.broadcast({ type: 'command', command: { type: 'Move', seq: i } });
  const client = ctx.wss.connectClient();
  console.log = realLog;
  const delivered = client.sent.filter(p => p.type === 'command').map(p => p.command.seq);
  const notices = logs.filter(line => /drop|overflow|lost/i.test(line));
  checks.push({ check: 'A_bridge_backlog_overflow', sent: SENT, delivered: delivered.length,
    dropped: SENT - delivered.length, firstDelivered: delivered[0], lastDelivered: delivered.at(-1),
    lossNotices: notices.length, ok: delivered.length === SENT || notices.length > 0 });
  ctx.close();
}

// --- B and C. browser backlog ---------------------------------------------
const LIMIT = 5000;
function browser({ remotePresent = false } = {}) {
  const remote = createStubRemote();
  const state = { present: remotePresent };
  const probe = createProbe({
    world: new StubWorld(),
    queueLimit: LIMIT,
    commands: {
      getRemoteSystem: () => (state.present ? remote : null),
      pushExternalCommands(batch) {
        if (!state.present) return false;              // the real controller's contract
        batch.forEach(c => remote.addCommand(c));
        return true;
      },
      handleUserReset() { remote.clearCommandQueue(); probe.log.reset += 1; },
    },
  });
  probe.controller.connect();
  const socket = StubSocket.instances.at(-1);
  socket.open();
  remote.clearCommandQueue(); // ignore the connect-time Rebase
  return { probe, remote, state, socket };
}

{
  const { probe, remote, state } = browser();
  const batch = Array.from({ length: LIMIT + 1000 }, (_, i) => ({ type: 'Move', seq: i }));
  probe.controller.handlePayload({ type: 'commands', commands: batch });
  state.present = true;
  probe.controller.flushQueue();
  checks.push({ check: 'B_oversize_message_exceeds_limit', configuredLimit: LIMIT,
    sentInOneMessage: batch.length, queued: remote.queue.length,
    beyondLimit: remote.queue.length - LIMIT, ok: remote.queue.length <= LIMIT });
}

{
  const { probe, remote, state } = browser();
  for (let i = 0; i < 3; i += 1) {
    probe.controller.handlePayload({ type: 'command', command: { type: 'Move', preReset: i } });
  }
  state.present = true;                        // scene available for the reset
  probe.controller.handlePayload({ type: 'reset' });   // worker queue cleared here
  const afterReset = remote.queue.length;
  probe.controller.flushQueue();               // rebuildScene() tail
  const replayed = remote.queue.filter(c => c.preReset !== undefined).map(c => c.preReset);
  checks.push({ check: 'C_reset_replays_parked_commands', parkedBeforeReset: 3,
    workerQueueAfterReset: afterReset, replayedAfterReset: replayed, ok: replayed.length === 0 });
}

console.log(JSON.stringify({ checks }, null, 2));
const failed = checks.filter(c => !c.ok);
if (failed.length) {
  console.error(`DEFECT: ${failed.map(c => c.check).join(', ')}`);
  process.exit(1);
}
console.log('OK: no silent loss and Reset discards parked commands');
