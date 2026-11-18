// E3: browser-side external command queue: overflow policy and Reset interaction.
// exit 0 = diagnostic; prints JSON.
import { createProbe, StubSocket, StubWorld, createStubRemote } from './harness.mjs';

const out = {};
const LIMIT = 5000;

function make({ remotePresent = false } = {}) {
  const remote = createStubRemote();
  const state = { present: remotePresent };
  const probe = createProbe({
    world: new StubWorld(),
    commands: {
      getRemoteSystem: () => (state.present ? remote : null),
      pushExternalCommands(batch) {
        if (!state.present) return false;           // exactly what the real controller does
        batch.forEach(c => remote.addCommand(c));
        return true;
      },
      handleUserReset() {
        // production: handleUserReset -> stopAndClearWorkers -> resetRemoteQueue
        // -> remote.clearPlaybackState() (clears _commands and rebase state)
        remote.clearCommandQueue();
        probe.log.reset += 1;
      },
    },
  });
  probe.controller.connect();
  const socket = StubSocket.instances.at(-1);
  socket.open();
  remote.clearCommandQueue(); // drop the connect-time Rebase for a clean baseline
  return { probe, remote, state, socket };
}

// (a) steady drip while no scene is loaded, then engine comes back.
{
  const { probe, remote, state } = make();
  const ids = [];
  for (let i = 0; i < LIMIT + 3; i += 1) {
    ids.push(i);
    probe.controller.handlePayload({ type: 'command', command: { type: 'Move', seq: i } });
  }
  state.present = true;
  probe.controller.flushQueue();
  const landed = remote.queue.map(c => c.seq);
  out.overflowDrip = {
    sent: LIMIT + 3,
    queueLimit: LIMIT,
    landedCount: landed.length,
    firstLandedSeq: landed[0],
    lastLandedSeq: landed.at(-1),
    droppedSeqs: ids.filter(i => !landed.includes(i)).slice(0, 5),
    droppedCount: ids.length - landed.length,
    warnings: probe.log.order.filter(o => String(o).startsWith('warn:')).length,
  };
}

// (b) one message carrying more than the limit (protocol allows commands[]).
{
  const { probe, remote, state } = make();
  const batch = Array.from({ length: LIMIT + 1000 }, (_, i) => ({ type: 'Move', seq: i }));
  probe.controller.handlePayload({ type: 'commands', commands: batch });
  state.present = true;
  probe.controller.flushQueue();
  out.oversizeBatch = {
    sent: batch.length, queueLimit: LIMIT, landed: remote.queue.length,
    queuedBeyondLimit: remote.queue.length - LIMIT,
  };
}

// (c) Reset while commands are parked, then the rebuild-time flush.
{
  const { probe, remote, state } = make();
  for (let i = 0; i < 3; i += 1) {
    probe.controller.handlePayload({ type: 'command', command: { type: 'Move', preReset: i } });
  }
  const beforeReset = { remoteQueue: remote.queue.length };
  // scene becomes available (or the reset's rebuild finishes) and a reset arrives
  state.present = true;
  probe.controller.handlePayload({ type: 'reset' });
  const justAfterReset = { remoteQueue: remote.queue.length, resets: probe.log.reset };
  // machineSceneController.rebuildScene() ends with getExternal().flushQueue()
  probe.controller.flushQueue();
  out.resetVersusParked = {
    beforeReset, justAfterReset,
    afterRebuildFlush: remote.queue.map(c => c.preReset),
    replayedPreResetCommands: remote.queue.length,
  };
}

console.log(JSON.stringify(out, null, 2));
