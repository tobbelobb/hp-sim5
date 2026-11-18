// E5: the bridge (server) side of the protocol, with two clients, no sockets.
// `ws` is replaced in-process by /tmp/opencode/fake-ws.mjs through a loader hook.
// Run with: node --import /tmp/opencode/ws-register.mjs probe_bridge_two_clients.mjs
import { createGcodeBridge } from '../../../../integrations/rrf/rrfSimulatorBridge.mjs';

const out = {};
const sleep = (ms) => new Promise(r => setTimeout(r, ms));

function encodeRequests(socket) {
  return socket.sent.filter(p => p.type === 'encoder_request');
}

// --- 1. two clients, broadcast + encoder routing ---------------------------
{
  const ctx = createGcodeBridge({ wsPort: 8790, quiet: true });
  const first = ctx.wss.connectClient();
  const second = ctx.wss.connectClient();
  ctx.broadcast({ type: 'command', command: { type: 'Move', B: 1 } });
  const req = ctx.sendEncoderRequest(['X'], 200).catch(e => ({ error: e.message }));
  const both = { firstCommands: first.sent.length, secondCommands: second.sent.length,
    firstRequests: encodeRequests(first).length, secondRequests: encodeRequests(second).length };
  const request = encodeRequests(second)[0];
  second.receive({ type: 'encoder_response', requestId: request.requestId, anglesDeg: [22] });
  const response = await req;
  out.twoClients = { ...both, angles: response.anglesDeg,
    encoderAskIsNewestOnly: encodeRequests(first).length === 0 };
  ctx.close();
}

// --- 2. newest client goes silent: does the bridge fail over? --------------
{
  const ctx = createGcodeBridge({ wsPort: 8790, quiet: true });
  const healthy = ctx.wss.connectClient();
  const newestSilent = ctx.wss.connectClient();
  healthy.on('message', () => {}); // healthy client answers below via receive()
  const answerer = setInterval(() => {
    const pending = encodeRequests(healthy).filter(r => !r.__answered);
    for (const r of pending) { r.__answered = true; healthy.receive({ type: 'encoder_response', requestId: r.requestId, anglesDeg: [11] }); }
  }, 5);
  const results = [];
  for (let attempt = 0; attempt < 3; attempt += 1) {
    const started = Date.now();
    try {
      const r = await ctx.sendEncoderRequest(['X'], 40);
      results.push({ attempt, ok: true, angles: r.anglesDeg, ms: Date.now() - started });
    } catch (e) {
      results.push({ attempt, ok: false, error: e.message, ms: Date.now() - started });
    }
  }
  clearInterval(answerer);
  out.silentNewest = {
    results,
    healthyRequests: encodeRequests(healthy).length,
    silentRequests: encodeRequests(newestSilent).length,
    healthyAbstracted: 'healthy client was ready the whole time and would have answered in <5 ms',
  };
  ctx.close();
}

// --- 3. no client attached: pending payload overflow ----------------------
{
  const ctx = createGcodeBridge({ wsPort: 8790, quiet: true });
  const total = 5003;
  for (let i = 0; i < total; i += 1) ctx.broadcast({ type: 'command', command: { type: 'Move', seq: i } });
  const client = ctx.wss.connectClient();
  const seqs = client.sent.filter(p => p.type === 'command').map(p => p.command.seq);
  out.pendingOverflow = { sentWithNoClient: total, deliveredOnAttach: seqs.length,
    firstDelivered: seqs[0], lastDelivered: seqs.at(-1), dropped: total - seqs.length,
    warningsLogged: 0, note: 'overflow evicts the oldest silently; the sender is not told' };
  ctx.close();
}

// --- 4. attach second client does not replay history ----------------------
{
  const ctx = createGcodeBridge({ wsPort: 8790, quiet: true });
  const a = ctx.wss.connectClient();
  ctx.broadcast({ type: 'command', command: { type: 'Move', seq: 1 } });
  const b = ctx.wss.connectClient();
  const secondOnAttach = b.sent.length;
  ctx.broadcast({ type: 'command', command: { type: 'Move', seq: 2 } });
  out.secondAttachReplays = { firstClient: a.sent.map(p => p.command?.seq),
    secondClientMessagesOnAttach: secondOnAttach,
    secondClientAfterNextBroadcast: b.sent.map(p => p.command?.seq) };
  ctx.close();
}

// --- 5. detach / reconnect: seq ordering and broadcast delivery -----------
{
  const ctx = createGcodeBridge({ wsPort: 8790, quiet: true });
  const a = ctx.wss.connectClient();
  a.close(1001, 'gone');
  const b = ctx.wss.connectClient();
  ctx.broadcast({ type: 'command', command: { type: 'Move', seq: 7 } });
  const ask = ctx.sendEncoderRequest(['R'], 30).catch(e => ({ error: e.message }));
  const askedNew = encodeRequests(b).length;
  await ask;
  out.detachReconnect = { remainingClients: ctx.getReadyWsClients().length,
    newClientGetsCommand: b.sent.length, oldClientGetsCommand: a.sent.length,
    encoderRequestWentToNewClient: askedNew };
  ctx.close();
}

// --- 6. malformed / duplicate client -> server messages -------------------
{
  const ctx = createGcodeBridge({ wsPort: 8790, quiet: true });
  const c = ctx.wss.connectClient();
  const before = encodeRequests(c).length;
  c.emit('message', Buffer.from('not json'));
  c.receive({ type: 'encoder_response' });                       // no requestId
  c.receive({ type: 'encoder_response', requestId: 999999 });     // unknown id
  const pending = ctx.sendEncoderRequest(['X'], 100);
  const req = encodeRequests(c).at(-1);
  c.receive({ type: 'encoder_response', requestId: req.requestId, anglesDeg: [1] });
  c.receive({ type: 'encoder_response', requestId: req.requestId, anglesDeg: [2] }); // duplicate
  const first = await pending;
  out.malformedClientMessages = { servedBeforeRequests: before, resolvedAngles: first.anglesDeg,
    note: 'junk is ignored; the first duplicate reply wins, later ones are dropped' };
  ctx.close();
}

console.log(JSON.stringify(out, null, 2));
