import fs from 'node:fs';
import http from 'node:http';
import net from 'node:net';
import path from 'node:path';
import { pathToFileURL } from 'node:url';
import { createHash } from 'node:crypto';
import { WebSocket } from 'ws';
const [label, output] = process.argv.slice(2);
const source = path.resolve('integrations/rrf/rrfSimulatorBridge.mjs');
const { createGcodeBridge } = await import(pathToFileURL(source));
const command = 'M569.3 P40.0:41.0:42.0:43.0';
const anglesDeg = [12.34, -56.78, 90.12, -34.56];
const origin = performance.now(), events = [], timers = [];
const record = (type, data = {}) => events.push({ type, elapsed_ms: performance.now() - origin, ...data });
const watchdog = setTimeout(() => { console.error('bounded WebSocket fixture expired'); process.exit(2); }, 15000);
const server = http.createServer(async (req, res) => {
  let body = ''; for await (const chunk of req) body += chunk;
  record('firmware_http_stub', { method: req.method, url: req.url, body, response: 'ok' });
  res.end('ok');
});
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
const portHolder = net.createServer();
await new Promise(resolve => portHolder.listen(0, '127.0.0.1', resolve));
const wsPort = portHolder.address().port;
await new Promise(resolve => portHolder.close(resolve));
const bridge = createGcodeBridge({ server: 'http://127.0.0.1:' + server.address().port, wsPort, quiet: true, encoderTimeoutMs: 120000 });
const socket = new WebSocket('ws://127.0.0.1:' + wsPort);
let rawReply, resolveSent;
const sent = new Promise(resolve => { resolveSent = resolve; });
socket.on('message', data => {
  const payload = JSON.parse(data.toString());
  if (payload.type !== 'encoder_request') return;
  record('encoder_request', { payload });
  rawReply = { type: 'encoder_response', requestId: payload.requestId, anglesDeg };
  record('encoder_values_captured', { payload: rawReply, delay_ms: 6000, physics_advanced: false });
  timers.push(setTimeout(() => { record('encoder_reply_sent', { payload: rawReply }); socket.send(JSON.stringify(rawReply)); resolveSent(); }, 6000));
});
let result, elapsed_ms;
try {
  await new Promise((resolve, reject) => { socket.once('open', resolve); socket.once('error', reject); });
  const start = performance.now(); result = await bridge.sendGcodeLine(command, { timeout: 120000 }); elapsed_ms = performance.now() - start;
  record('command_result', { result, command_elapsed_ms: elapsed_ms });
  let lateTimer;
  try { await Promise.race([sent, new Promise((_, reject) => { lateTimer = setTimeout(() => reject(Error('no delayed reply')), 7500); })]); }
  finally { clearTimeout(lateTimer); }
} finally {
  timers.forEach(clearTimeout); clearTimeout(watchdog);
  if (socket.readyState !== WebSocket.CLOSED) await new Promise(resolve => { socket.once('close', resolve); socket.close(); });
  bridge.close(); await new Promise(resolve => server.close(resolve));
}
const receipt = { label, node_version: process.version, fresh_node_process: true, pid: process.pid, source, source_sha256: createHash('sha256').update(fs.readFileSync(source)).digest('hex'), firmware_http: 'local stub, returns ok; no real firmware or physics', real_websocket: true, command, command_timeout_ms: 120000, encoderTimeoutMs: 120000, delay_ms: 6000, raw_reply: rawReply, result, elapsed_ms, events };
fs.writeFileSync(output, JSON.stringify(receipt, null, 2) + '\n');
console.log(JSON.stringify({ label, source_sha256: receipt.source_sha256, elapsed_ms, reply: result.reply, raw_reply: rawReply }));
