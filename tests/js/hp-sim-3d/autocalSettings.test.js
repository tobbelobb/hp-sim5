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
