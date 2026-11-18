// Shared stub harness for the external command socket lifecycle investigation.
// Pure JS. No network, no timers that outlive a test, no physics.
import { createExternalCommandController } from '../../../../hp-sim-3d/app/externalCommandSocket.js';

export class StubSocket extends EventTarget {
  static OPEN = 1;
  static CLOSED = 3;
  static instances = [];
  static reset() { StubSocket.instances = []; }

  constructor(url) {
    super();
    this.url = url;
    this.readyState = 0; // CONNECTING
    this.sent = [];
    this.closed = false;
    StubSocket.instances.push(this);
  }
  send(value) {
    if (this.readyState !== StubSocket.OPEN) throw new Error('send on non-open socket');
    this.sent.push(typeof value === 'string' ? JSON.parse(value) : value);
  }
  close() {
    if (this.closed) return;
    this.closed = true;
    this.readyState = StubSocket.CLOSED;
    this.dispatchEvent(new Event('close'));
  }
  open() { this.readyState = StubSocket.OPEN; this.dispatchEvent(new Event('open')); }
  deliver(payload) {
    const data = typeof payload === 'string' ? payload : JSON.stringify(payload);
    this.dispatchEvent(new MessageEvent('message', { data }));
  }
}

export class StubWorld {
  constructor({ recorder = null } = {}) {
    this.resources = new Map();
    this.components = new Map();
    if (recorder) this.resources.set('flightRecorder', recorder);
  }
  getResource(key) { return this.resources.get(key); }
  setResource(key, value) { this.resources.set(key, value); }
  getComponent(entity, type) { return this.components.get(`${entity}:${type}`); }
  setComponent(entity, type, value) { this.components.set(`${entity}:${type}`, value); }
  query() { return []; }
}

// Records every externally visible effect of the controller's protocol handling.
export function createProbe({ world = new StubWorld(), commands = {}, runtime = {}, featureFlags = undefined,
  machines = undefined, inspectionTools = undefined, queueLimit = 5000 } = {}) {
  const log = { resume: 0, reset: 0, timeScale: [], pushed: [], rebasePushed: 0,
    encoderRequests: [], settingsApplied: 0, order: [] };

  const commandsProxy = {
    getRemoteSystem: commands.getRemoteSystem ?? (() => null),
    pushExternalCommands: commands.pushExternalCommands ?? (() => true),
    handleUserReset: commands.handleUserReset ?? (() => { log.reset += 1; log.order.push('reset'); }),
    applyTimeScaleChange: commands.applyTimeScaleChange ?? ((value) => {
      log.timeScale.push(value); log.order.push(`speed:${value}`);
    }),
    handleUserResetRaw: commands.handleUserReset,
    pushExternalCommandsRaw: commands.pushExternalCommands,
  };
  const runtimeProxy = {
    resume: () => { log.resume += 1; log.order.push('resume'); },
    ...runtime,
  };
  const controller = createExternalCommandController({
    world, url: 'ws://stub', WebSocketCtor: StubSocket, logger: {
      log: () => {}, info: () => {}, warn: (...args) => log.order.push(`warn:${args[0]}`),
    }, commands: commandsProxy, runtime: runtimeProxy, featureFlags, machines,
    inspectionTools, queueLimit,
  });
  return { controller, log, world, commands: commandsProxy, runtime: runtimeProxy };
}

// A remote spool system stand-in with the production queue semantics that the
// external controller depends on: addCommand appends, getQueueLength counts,
// clearCommandQueue empties (used by both reset paths).
export function createStubRemote() {
  const queue = [];
  return {
    queue,
    addCommand(command) { queue.push(command); },
    getQueueLength() { return queue.length; },
    clearCommandQueue() { queue.length = 0; },
    drain() { const out = queue.splice(0, queue.length); return out; },
  };
}
