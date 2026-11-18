// Reproducer: with two simulator clients attached, an unresponsive newest client
// permanently owns encoder/clock requests; the bridge never fails over.
//
// integrations/rrf/rrfSimulatorBridge.mjs, getPreferredEncoderClient():
//   * every connection becomes the preferred client;
//   * a request timeout clears preferredEncoderClient and drops the request;
//   * the next call re-derives the preferred client as "highest connection
//     sequence number", i.e. the same unresponsive client.
// So the timeout branch has no effect while that client stays connected, and
// every later encoder request pays the full timeout and reports
// "the browser may be busy or suspended".
//
// Expected: after one timeout, the next request goes to another ready client
//           (the code already has a fallback path for a *disconnected* preferred
//           client, so failover is the intended shape).
// Actual:   3/3 requests go to the silent client; the healthy client is never asked.
// Exit 0 = expected behaviour, exit 1 = defect reproduced.
//
// Run (from this directory):
//   node --import ./ws-stub/ws-register.mjs repro_two_clients_sticky_encoder.mjs
import { createGcodeBridge } from '../../../../integrations/rrf/rrfSimulatorBridge.mjs';

const ctx = createGcodeBridge({ wsPort: 8790, quiet: true });
const healthy = ctx.wss.connectClient();       // attached first
const silent = ctx.wss.connectClient();        // attached second -> preferred

// Make `healthy` behave like a working simulator: answer every encoder request
// immediately. (The fake socket's send() only records; a real client answers.)
let healthyOffers = 0;
const rawSend = healthy.send.bind(healthy);
healthy.send = (data) => {
  rawSend(data);
  const request = JSON.parse(data);
  if (request.type !== 'encoder_request') return;
  healthyOffers += 1;
  healthy.receive({ type: 'encoder_response', requestId: request.requestId, anglesDeg: [11] });
};

const asks = (socket) => socket.sent.filter(p => p.type === 'encoder_request').length;
const attempts = [];
for (let i = 0; i < 3; i += 1) {
  const started = Date.now();
  try {
    const response = await ctx.sendEncoderRequest(['X'], 50);
    attempts.push({ i, ok: true, anglesDeg: response.anglesDeg, ms: Date.now() - started,
      sentToHealthy: asks(healthy), sentToSilent: asks(silent) });
  } catch (error) {
    attempts.push({ i, ok: false, error: error.message, ms: Date.now() - started,
      sentToHealthy: asks(healthy), sentToSilent: asks(silent) });
  }
}

// Control: once the silent client is gone, the same healthy client answers.
const healthyOffersDuringAttempts = healthyOffers;
silent.close(1001, 'closed by test');
let control;
try {
  control = { ok: true, anglesDeg: (await ctx.sendEncoderRequest(['X'], 200)).anglesDeg };
} catch (error) {
  control = { ok: false, error: error.message };
}

const result = { attempts, healthyOffersDuringAttempts, control, readyClients: ctx.getReadyWsClients().length };
console.log(JSON.stringify(result, null, 2));
ctx.close();

if (healthyOffersDuringAttempts === 0) {
  console.error('DEFECT: with two ready clients, the healthy client was never asked '
    + `(${attempts.length}/${attempts.length} requests went to the unresponsive one, each costing the full timeout).`);
  process.exit(1);
}
console.log('OK: the bridge failed over to a responsive client');
