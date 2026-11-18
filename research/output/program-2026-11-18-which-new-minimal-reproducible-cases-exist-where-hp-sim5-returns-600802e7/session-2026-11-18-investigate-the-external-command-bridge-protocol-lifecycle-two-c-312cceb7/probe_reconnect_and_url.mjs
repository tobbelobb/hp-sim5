// E4: attach / detach / reconnect lifecycle of createExternalCommandSocket,
// and normalizeWsUrl handling. Pure stubs; short reconnect delay; no network.
import { createExternalCommandSocket, normalizeWsUrl } from '../../../../hp-sim-3d/app/externalCommandSocket.js';
import { StubSocket } from './harness.mjs';

const sleep = (ms) => new Promise(r => setTimeout(r, ms));
const out = { normalize: [], lifecycle: {}, reconnect: {} };

for (const raw of ['ws://localhost:8790', 'wss://h/ws', ' localhost:8790', '/gcode', '//gcode',
  'http://localhost:8790', 'https://localhost:8790', 'localhost:8790/stream', '', '   ', null, 42]) {
  out.normalize.push({ raw, normalized: normalizeWsUrl(raw) });
}

// --- lifecycle ------------------------------------------------------------
{
  StubSocket.reset();
  const events = [];
  const logs = [];
  const ctl = createExternalCommandSocket({
    url: 'ws://stub', WebSocketCtor: StubSocket, reconnectDelayMs: 5,
    logger: { log: (...a) => logs.push(a.join(' ')), warn: (...a) => logs.push(`WARN ${a.join(' ')}`) },
    onPayload: p => events.push(p),
    onOpen: () => events.push('open'),
  });
  const res = {};
  ctl.connect();
  res.socketsAfterFirstConnect = StubSocket.instances.length;
  res.sendBeforeOpen = ctl.send({ type: 'x' });
  res.connecting = ctl.connecting;
  const s0 = StubSocket.instances.at(-1);
  s0.open();
  res.sendAfterOpen = ctl.send({ type: 'x' });
  res.sentPayloads = s0.sent.length;
  res.connectingAfterOpen = ctl.connecting;
  // message dispatch, including a binary/non-string payload
  s0.deliver({ hello: 1 });
  res.payloads = events.filter(e => e !== 'open').length;
  // detach
  s0.close();
  res.socketsImmediatelyAfterClose = StubSocket.instances.length;
  res.sendWhileDown = ctl.send({ type: 'x' });
  await sleep(20);
  res.socketsAfterReconnectDelay = StubSocket.instances.length;
  const s1 = StubSocket.instances.at(-1);
  res.newSocketIsFreshInstance = s1 !== s0;
  res.secondSocketOpenedAutomatically = s1 && !s1.closed;
  s1.open();
  // duplicate connect() while open must not open a second socket
  ctl.connect();
  res.socketsAfterRedundantConnect = StubSocket.instances.length;
  // error path
  s1.dispatchEvent(new Event('error'));
  res.erroredSocketClosed = s1.closed;
  await sleep(20);
  res.socketsAfterErrorReconnect = StubSocket.instances.length;
  // retry logging: fail nine times then check the tenth message
  const s2 = StubSocket.instances.at(-1);
  for (let i = 0; i < 9; i += 1) {
    s2.close();
    await sleep(12);
    StubSocket.instances.at(-1).close();
    await sleep(12);
  }
  res.logsAfterRetries = logs.slice(-3);
  res.totalSockets = StubSocket.instances.length;
  out.lifecycle = res;
}

// --- socket that never opens (hung handshake) ------------------------------
{
  StubSocket.reset();
  class Hung extends StubSocket {}
  const ctl = createExternalCommandSocket({ url: 'ws://stub', WebSocketCtor: Hung, reconnectDelayMs: 5,
    logger: { log: () => {}, warn: () => {} } });
  ctl.connect();
  await sleep(40);
  out.reconnect.hungHandshake = { socketsCreated: StubSocket.instances.length, connecting: ctl.connecting };
}

// --- close() that throws inside the error handler --------------------------
{
  StubSocket.reset();
  class Stubborn extends StubSocket { close() { throw new Error('cannot close'); } }
  const ctl = createExternalCommandSocket({ url: 'ws://stub', WebSocketCtor: Stubborn, reconnectDelayMs: 5,
    logger: { log: () => {}, warn: () => {} } });
  ctl.connect();
  StubSocket.instances.at(-1).dispatchEvent(new Event('error'));
  await sleep(30);
  out.reconnect.closeThrows = { socketsCreated: StubSocket.instances.length, hasSocket: Boolean(ctl.socket),
    connecting: ctl.connecting };
}

// --- constructor that throws ----------------------------------------------
{
  StubSocket.reset();
  class Boom { constructor() { throw new Error('bad url'); } static OPEN = 1; }
  const ctl = createExternalCommandSocket({ url: 'ws://stub', WebSocketCtor: Boom, reconnectDelayMs: 5,
    logger: { log: () => {}, warn: () => {} } });
  ctl.connect();
  await sleep(30);
  out.reconnect.constructorThrows = { connecting: ctl.connecting, hasSocket: Boolean(ctl.socket) };
}

console.log(JSON.stringify(out, null, 2));
// Several cases deliberately leave a retry timer armed; do not wait it out.
process.exit(0);
