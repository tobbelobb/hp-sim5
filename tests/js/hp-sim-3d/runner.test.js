import { runGame } from '../../../hp-sim-3d/app/runner.js';

jest.mock('../../../hp-sim-3d/app/workerController.js', () => ({ startSimulationClock: jest.fn() }));

let controls;
const originalGlobals = Object.fromEntries(
  ['document', 'requestAnimationFrame', 'cancelAnimationFrame', 'MessageChannel'].map((key) => [key, global[key]]),
);

beforeEach(() => {
  jest.useFakeTimers();
  jest.spyOn(performance, 'now').mockReturnValue(1000);
  global.requestAnimationFrame = jest.fn(() => 1);
  global.cancelAnimationFrame = jest.fn();
  global.MessageChannel = class {
    port1 = {};
    port2 = { postMessage: jest.fn() };
  };
});

afterEach(() => {
  controls?.pause();
  jest.restoreAllMocks();
  jest.useRealTimers();
  Object.assign(global, originalGlobals);
});

function setup() {
  const stepButton = { addEventListener: (_event, handler) => { stepButton.click = () => handler({ preventDefault() {} }); } };
  const listeners = {};
  global.document = {
    hidden: true,
    getElementById: (id) => id === 'stepBtn' ? stepButton : null,
    addEventListener: (event, handler) => { listeners[event] = handler; },
  };
  const render = { update: jest.fn(), requestRender: jest.fn() };
  const resources = new Map([['dt', 0.002], ['pauseState', { paused: false }], ['renderSystem', render]]);
  const world = {
    systems: [],
    getResource: (key) => resources.get(key),
    setResource: (key, value) => resources.set(key, value),
    update: jest.fn(),
  };
  const clocks = [];
  const startClock = (tick, fail) => {
    const clock = { tick, fail, stop: jest.fn() };
    clocks.push(clock);
    return clock.stop;
  };
  controls = runGame(world, () => {}, { startClock });
  return { world, resources, render, clocks, stepButton, listeners };
}

test('physics advances at the selected speed without animation frames; drawing never advances physics', () => {
  const { world, resources, render, clocks } = setup();
  clocks[0].tick(1100);
  expect(world.update.mock.calls.length).toBeGreaterThanOrEqual(49);
  expect(world.update.mock.calls.every(([dt]) => dt === 0.002)).toBe(true);
  expect(resources.get('researchClock').time).toBeCloseTo(0.1, 2);
  expect(render.update).not.toHaveBeenCalled();

  const draw = requestAnimationFrame.mock.calls[0][0];
  draw();
  expect(render.update).not.toHaveBeenCalled();
  document.hidden = false;
  const beforeDraw = world.update.mock.calls.length;
  draw();
  expect(render.update).toHaveBeenCalledTimes(1);
  expect(world.update).toHaveBeenCalledTimes(beforeDraw);

  controls.setTimeScale(2);
  clocks[0].tick(1200);
  const beforeTick = world.update.mock.calls.length;
  clocks[0].tick(1300);
  expect(world.update.mock.calls.length - beforeTick).toBeGreaterThanOrEqual(99);
});

test('pause and reset cancel the clock and ignore stale ticks; single stepping works while hidden', () => {
  const { world, resources, clocks, stepButton } = setup();
  controls.pause();
  clocks[0].tick(1100);
  expect(clocks[0].stop).toHaveBeenCalledTimes(1);
  expect(world.update).not.toHaveBeenCalled();

  stepButton.click();
  clocks[1].tick(1100);
  expect(world.update).toHaveBeenCalledTimes(1);
  expect(resources.get('pauseState').paused).toBe(true);
  expect(clocks[1].stop).toHaveBeenCalledTimes(1);

  controls.resume();
  clocks[2].tick(1200);
  expect(world.update.mock.calls.length).toBeGreaterThan(1);
  controls.reset();
  world.update.mockClear();
  clocks[2].tick(1300);
  expect(clocks[2].stop).toHaveBeenCalledTimes(1);
  expect(world.update).not.toHaveBeenCalled();
  expect(resources.get('researchClock').step).toBe(0);
});

test('background stepping respects recorder backpressure', () => {
  const { world, resources, clocks } = setup();
  const recorder = { readyForStep: jest.fn(() => false) };
  resources.set('flightRecorder', recorder);
  clocks[0].tick(1100);
  expect(world.update).not.toHaveBeenCalled();
  recorder.readyForStep.mockReturnValue(true);
  clocks[0].tick(1120);
  expect(resources.get('researchClock').time).toBeCloseTo(0.12, 2);
});

test('returning to a tab paused in the background requests its latest state', () => {
  const { render, clocks, listeners } = setup();
  clocks[0].tick(1100);
  controls.pause();
  expect(render.requestRender).not.toHaveBeenCalled();
  document.hidden = false;
  listeners.visibilitychange();
  expect(render.requestRender).toHaveBeenCalledTimes(1);
});

test('clock failures pause playback, while stale failures cannot stop a resumed run', () => {
  const { resources, clocks } = setup();
  jest.spyOn(console, 'error').mockImplementation(() => {});
  clocks[0].fail(new Error('clock failed'));
  expect(resources.get('pauseState').paused).toBe(true);
  expect(clocks[0].stop).toHaveBeenCalledTimes(1);
  controls.resume();
  clocks[0].fail(new Error('stale failure'));
  expect(resources.get('pauseState').paused).toBe(false);
  expect(clocks[1].stop).not.toHaveBeenCalled();
});

test('switching to ASAP cancels linear ticks and returning starts a fresh clock', () => {
  const { world, resources, clocks, render } = setup();
  world.update.mockImplementationOnce(() => { resources.get('pauseState').paused = true; });
  controls.setAsapPlayback(true);
  expect(clocks[0].stop).toHaveBeenCalledTimes(1);
  expect(world.update).toHaveBeenCalledTimes(1);
  clocks[0].tick(1100);
  expect(world.update).toHaveBeenCalledTimes(1);
  expect(render.update).not.toHaveBeenCalled();
  controls.setAsapPlayback(false);
  controls.resume();
  clocks[1].tick(1200);
  expect(world.update.mock.calls.length).toBeGreaterThan(1);
});
