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

function setup({ render = { update: jest.fn(), requestRender: jest.fn() } } = {}) {
  const stepButton = { addEventListener: (_event, handler) => { stepButton.click = () => handler({ preventDefault() {} }); } };
  const pauseButton = { addEventListener: (_event, handler) => { pauseButton.click = () => handler({ preventDefault() {} }); } };
  const speed = { textContent: '' };
  const elements = { stepBtn: stepButton, pauseBtn: pauseButton, speed };
  const listeners = {};
  global.document = {
    hidden: true,
    getElementById: (id) => elements[id] ?? null,
    addEventListener: (event, handler) => { listeners[event] = handler; },
  };
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
  return { world, resources, render, clocks, stepButton, pauseButton, speed, listeners };
}

test('hidden tabs step on the worker clock and keep tracing; visible tabs step and draw in frames', () => {
  const { world, resources, render, clocks } = setup();
  render.samplePositionTrace = jest.fn();
  clocks[0].tick(1100);
  expect(world.update.mock.calls.length).toBeGreaterThanOrEqual(49);
  expect(world.update.mock.calls.every(([dt]) => dt === 0.002)).toBe(true);
  expect(resources.get('researchClock').time).toBeCloseTo(0.1, 2);
  expect(render.samplePositionTrace).toHaveBeenCalledTimes(1);
  expect(render.update).not.toHaveBeenCalled();

  controls.setTimeScale(2);
  clocks[0].tick(1200);
  const beforeTick = world.update.mock.calls.length;
  clocks[0].tick(1300);
  expect(world.update.mock.calls.length - beforeTick).toBeGreaterThanOrEqual(99);

  document.hidden = false;
  const beforeVisible = world.update.mock.calls.length;
  clocks[0].tick(1400);
  expect(world.update).toHaveBeenCalledTimes(beforeVisible);
  const frame = requestAnimationFrame.mock.calls[0][0];
  frame(1400);
  expect(world.update.mock.calls.length - beforeVisible).toBeGreaterThanOrEqual(99);
  expect(render.update).toHaveBeenCalledTimes(1);
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

// Each test drives the wall clock explicitly.
function wallClock(start = 1000) {
  const clock = { now: start };
  performance.now.mockImplementation(() => clock.now);
  return clock;
}

test.each([
  ['requestAnimationFrame', () => ({ update: jest.fn(), requestRender: jest.fn() })],
  ['setAnimationLoop', () => ({ update: jest.fn(), requestRender: jest.fn(), setAnimationLoop: jest.fn(), clearAnimationLoop: jest.fn() })],
])('every visible %s frame shows its own elapsed time of motion; clock ticks add none', (_loop, makeRender) => {
  // 1172c stepped physics on a 16 ms worker clock while frames drew whatever had
  // accumulated, so frames showed 0, 1 or 2 ticks of motion: visible stutter.
  const clock = wallClock();
  const render = makeRender();
  const { world, resources, clocks } = setup({ render });
  document.hidden = false;
  controls.setTimeScale(32);
  const frame = render.setAnimationLoop?.mock.calls.at(-1)?.[0] ?? requestAnimationFrame.mock.calls.at(-1)[0];
  frame(clock.now);
  const simTime = () => resources.get('researchClock')?.time ?? 0;
  const perFrame = [];
  for (let i = 1; i <= 30; i += 1) {
    clock.now = 1000 + i * 1000 / 60;
    clocks.at(-1).tick(clock.now - 8);
    const before = simTime();
    frame(clock.now);
    perFrame.push(simTime() - before);
  }
  const expected = 32 / 60;
  expect(perFrame.every((dt) => Math.abs(dt - expected) <= 0.002 + 1e-9)).toBe(true);
  expect(render.update).toHaveBeenCalledTimes(31);
  expect(world.update.mock.calls.length).toBe(Math.round(simTime() / 0.002));
});

test('a physics error inside a visible frame pauses playback, and Resume runs again', () => {
  const clock = wallClock();
  const { world, resources, pauseButton } = setup();
  document.hidden = false;
  jest.spyOn(console, 'error').mockImplementation(() => {});
  world.update.mockImplementationOnce(() => { throw new Error('recorder send failed'); });
  clock.now = 1100;
  expect(() => requestAnimationFrame.mock.calls.at(-1)[0](clock.now)).not.toThrow();
  expect(resources.get('pauseState').paused).toBe(true);
  expect(pauseButton.textContent).toBe('Resume');

  pauseButton.click();
  const frame = requestAnimationFrame.mock.calls.at(-1)[0];
  world.update.mockClear();
  clock.now = 1200;
  frame(clock.now);
  expect(world.update.mock.calls.length).toBeGreaterThanOrEqual(49);
});

test('high speeds step past the old 500-step cap but within the tick budget and catch-up limit', () => {
  // A fixed 500 steps per tick capped 2 ms scenes at 60x whatever the CPU could do.
  const clock = wallClock();
  const { world, clocks } = setup();
  controls.setTimeScale(100);
  clocks.at(-1).tick(clock.now);
  clock.now += 16;
  clocks.at(-1).tick(clock.now);
  expect(Math.abs(world.update.mock.calls.length - 800)).toBeLessThanOrEqual(1);

  // A 10 s stall catches up at most 0.25 s of wall time: 25 simulated seconds.
  world.update.mockClear();
  clock.now += 10000;
  clocks.at(-1).tick(clock.now);
  expect(Math.abs(world.update.mock.calls.length - 12500)).toBeLessThanOrEqual(1);

  // Expensive steps stop at the 30 ms budget, so frames keep coming.
  world.update.mockClear().mockImplementation(() => { clock.now += 1; });
  clock.now += 16;
  clocks.at(-1).tick(clock.now);
  expect(world.update.mock.calls.length).toBeGreaterThanOrEqual(30);
  expect(world.update.mock.calls.length).toBeLessThanOrEqual(64);
});

test('the speed readout shows achieved over the last second next to the target, excluding paused time', () => {
  const clock = wallClock();
  const { clocks, speed, pauseButton } = setup();
  controls.setTimeScale(4);
  expect(speed.textContent).toBe('– / 4x');
  const tickFor = (ms) => {
    const end = clock.now + ms;
    while (clock.now < end) { clock.now += 16; clocks.at(-1).tick(clock.now); }
  };
  tickFor(1200);
  expect(speed.textContent).toMatch(/^(3\.\d|4\.0)x \/ 4x$/);

  pauseButton.click();
  clock.now += 60000;
  pauseButton.click();
  const before = speed.textContent;
  while (speed.textContent === before) { clock.now += 16; clocks.at(-1).tick(clock.now); }
  expect(speed.textContent).toMatch(/^(3\.\d|4\.0)x \/ 4x$/);
});
