import { World } from '../../../src/js/cable_joints_3d/ecs.js';
import { createExternalCommandController } from '../../../hp-sim-3d/app/externalCommandSocket.js';
import { createFeatureFlagsController } from '../../../hp-sim-3d/app/featureFlagsController.js';
import { FlightRecorder } from '../../../hp-sim-3d/app/flightRecorder.js';

jest.mock('three/addons/controls/OrbitControls.js', () => ({ OrbitControls: class {} }));
const cryptoDescriptor = Object.getOwnPropertyDescriptor(globalThis, 'crypto');
beforeAll(() => Object.defineProperty(globalThis, 'crypto', {
  configurable: true, value: { randomUUID: () => 'test-session' },
}));
afterAll(() => {
  if (cryptoDescriptor) Object.defineProperty(globalThis, 'crypto', cryptoDescriptor);
  else delete globalThis.crypto;
});

class Socket extends EventTarget {
  static OPEN = 1;
  readyState = 0;
  samples = [];
  constructor(url) { super(); this.url = url; Socket.last = this; }
  send(value) { this.samples.push(JSON.parse(value)); }
  close() { this.readyState = 3; this.dispatchEvent(new Event('close')); }
  open() { this.readyState = 1; this.dispatchEvent(new Event('open')); }
}

test('autocal settings update physics resources and visible feature toggles', () => {
  const world = new World();
  const toggles = { closedLoopMotorsToggle: { checked: false }, lineLayeringToggle: { checked: false } };
  const featureFlags = createFeatureFlagsController({ world, state: {}, toggles });
  const commands = { handleUserReset: jest.fn() };
  const controller = createExternalCommandController({ world, featureFlags, commands });
  controller.handlePayload({ type: 'simulation_settings', closedLoopMotorsEnabled: true, lineLayeringEnabled: true });
  expect(toggles.closedLoopMotorsToggle.checked).toBe(true);
  expect(toggles.lineLayeringToggle.checked).toBe(true);
  for (const key of ['closedLoopMotorsEnabled', 'enableLayering', 'layeringFrictionEffectiveRadius', 'layeringRenderWraps']) {
    expect(world.getResource(key)).toBe(true);
  }
  expect(commands.handleUserReset).toHaveBeenCalledTimes(1);
  controller.handlePayload({ type: 'simulation_settings', lineLayeringEnabled: true });
  expect(commands.handleUserReset).toHaveBeenCalledTimes(1);
  controller.handlePayload({ type: 'simulation_settings', closedLoopMotorsEnabled: false, lineLayeringEnabled: false });
  expect(toggles.closedLoopMotorsToggle.checked).toBe(false);
  expect(toggles.lineLayeringToggle.checked).toBe(false);
  expect(world.getResource('enableLayering')).toBe(false);
  expect(world.getResource('closedLoopMotorsEnabled')).toBe(false);
});

test('settings barrier waits for Rerun connection and preserves an existing recording', async () => {
  const world = new World();
  const recorder = new FlightRecorder({ world, WebSocketClass: Socket });
  world.setResource('flightRecorder', recorder);
  const controller = createExternalCommandController({ world, url: 'ws://bridge', WebSocketCtor: Socket });
  controller.connect();
  const bridge = Socket.last;
  bridge.open();
  controller.handlePayload({ type: 'simulation_settings', recordingUrl: 'ws://recording' });
  controller.handlePayload({ type: 'encoder_request', requestId: 1, axes: [] });
  await Promise.resolve();
  const recording = recorder.socket;
  expect(bridge.samples).toHaveLength(0);
  recording.open();
  expect(bridge.samples).toHaveLength(0);
  recording.dispatchEvent(new MessageEvent('message', { data: JSON.stringify({ type: 'ack' }) }));
  await new Promise(resolve => setTimeout(resolve, 0));
  expect(bridge.samples.at(-1)).toMatchObject({ requestId: 1, simulationSettings: { recording: true } });
  expect(recorder.extendedReference).toBe(true);
  const session = recorder.session;
  await recorder.ensureConnected('ws://recording');
  expect(recorder.socket).toBe(recording);
  expect(recorder.session).toBe(session);
  recorder.disconnect();
});

test('failed recorder connection releases physics backpressure', async () => {
  const recorder = new FlightRecorder({ world: new World(), WebSocketClass: Socket });
  const pending = recorder.ensureConnected('ws://unavailable', 10);
  await expect(pending).rejects.toThrow('timed out');
  expect(recorder.socket).toBeNull();
  expect(recorder.readyForStep()).toBe(true);
});

test.each([
  ['hangprinter_3', 'hp3_rigid_body.usda'],
  ['hangprinter_4', 'hp4_rigid_body.usda'],
  ['skycam', 'four_high_anchors_rigid_body.usda'],
  ['cubecorners', 'cubecorners_rigid_body.usda'],
  ['slideprinter', 'slideprinter_rigid_body.usda'],
])('machine %s replaces other scenes before acknowledging settings', async (machineType, sourceKey) => {
  const world = new World();
  let finishLoading;
  let loadedKey = 'old-machine.usda';
  const machines = {
    getMachines: () => [{ sourceKey: loadedKey }],
    removeAllMachines: jest.fn(async () => {}),
    addCatalogMachine: jest.fn(() => new Promise(resolve => { finishLoading = resolve; })),
  };
  const controller = createExternalCommandController({ world, machines, url: 'ws://bridge', WebSocketCtor: Socket });
  controller.connect();
  const bridge = Socket.last;
  bridge.open();
  controller.handlePayload({ type: 'simulation_settings', machineType });
  controller.handlePayload({ type: 'encoder_request', requestId: 1, axes: [] });
  await new Promise(resolve => setTimeout(resolve, 0));
  expect(machines.removeAllMachines).toHaveBeenCalledTimes(1);
  expect(machines.addCatalogMachine).toHaveBeenCalledWith(sourceKey, { resetView: true });
  expect(bridge.samples).toHaveLength(0);
  loadedKey = sourceKey;
  finishLoading({});
  await new Promise(resolve => setTimeout(resolve, 0));
  expect(bridge.samples.at(-1).simulationSettings).toMatchObject({ machineType, settingsError: null });
  world.clear(); // Scene resets discard world resources, but preserve the selected machine list.
  controller.handlePayload({ type: 'encoder_request', requestId: 2, axes: [] });
  await new Promise(resolve => setTimeout(resolve, 0));
  expect(bridge.samples.at(-1).simulationSettings.machineType).toBe(machineType);
});

test('recorder shutdown drains pending events before disconnecting', async () => {
  const world = new World();
  const recorder = new FlightRecorder({ world, WebSocketClass: Socket });
  recorder.extendedReference = true;
  recorder.recordPhysics = false;
  recorder.connect();
  const socket = recorder.socket;
  socket.open();
  socket.dispatchEvent(new MessageEvent('message', { data: JSON.stringify({ type: 'ack' }) }));
  recorder.recordEvent('last_event', {});
  socket.dispatchEvent(new MessageEvent('message', { data: JSON.stringify({ type: 'recording_stop' }) }));
  expect(recorder.socket).toBe(socket);
  recorder.recordEvent('after_stop', {});
  const batch = socket.samples.find(sample => sample.type === 'autocal_event_batch');
  expect(batch.events.map(event => event.kind)).toEqual(['scene_context', 'last_event']);
  socket.dispatchEvent(new MessageEvent('message', { data: JSON.stringify({ type: 'event_ack', count: batch.events.length }) }));
  await new Promise(resolve => setTimeout(resolve, 15));
  expect(recorder.socket).toBeNull();
  expect(recorder.error).toBeNull();
});

test('browser substep menu and remote settings agree, reject invalid counts, and survive resets', () => {
  const world = new World();
  const listeners = {};
  const select = {
    options: [{ value: '1' }, { value: '4' }], value: '1',
    ownerDocument: { createElement: () => ({}) },
    add(option) { this.options.push(option); },
    addEventListener(type, handler) { listeners[type] = handler; },
  };
  const state = {};
  const flags = createFeatureFlagsController({ world, state, toggles: { physicsSubsteps: select } });
  flags.bindUi();
  expect(world.getResource('physicsSubsteps')).toBe(1);
  select.value = '4';
  listeners.change();
  expect(world.getResource('physicsSubsteps')).toBe(4);
  const controller = createExternalCommandController({ world, featureFlags: flags, url: 'ws://test', WebSocketCtor: Socket });
  controller.connect();
  Socket.last.open();
  controller.handlePayload({ type: 'simulation_settings', physicsSubsteps: 3 });
  expect(select.value).toBe('3');
  world.clear();
  controller.handlePayload({ type: 'encoder_request', requestId: 1, axes: [] });
  expect(Socket.last.samples.at(-1).simulationSettings.physicsSubsteps).toBe(3);
  for (const count of [0, -1, 1.5, 17, '4', null]) {
    controller.handlePayload({ type: 'simulation_settings', physicsSubsteps: count });
    controller.handlePayload({ type: 'encoder_request', requestId: 2, axes: [] });
    expect(Socket.last.samples.at(-1).simulationSettings.settingsError).toMatch('integer');
    expect(world.getResource('physicsSubsteps')).toBe(3);
  }
  controller.handlePayload({ type: 'simulation_settings', physicsSubsteps: 1 });
  expect(select.value).toBe('1');
});
