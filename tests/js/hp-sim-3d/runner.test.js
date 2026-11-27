import { runGame } from '../../../hp-sim-3d/app/runner.js';
import { createWorkerController } from '../../../hp-sim-3d/app/workerController.js';
import { createCommandJobController } from '../../../hp-sim-3d/app/commandJobController.js';
import { createQualityController } from '../../../hp-sim-3d/app/qualityController.js';
import { QualityMonitor } from '../../../hp-sim-3d/app/quality-monitor.js';
import { RemoteSpoolSystem } from '../../../hp-sim-3d/app/remoteSpoolSystem.js';
import { ExtruderComponent } from '../../../hp-sim-3d/app/hangprinter_extruder.js';
import { World } from '../../../src/js/cable_joints_3d/ecs.js';

jest.mock('../../../integrations/klipper/klipperSimulatorBridge.js', () => ({ createKlipperRawBridge: jest.fn() }));
jest.mock('../../../integrations/klipper/klipperMcuCommandPlayer.js?worker&url', () => '', { virtual: true });
jest.mock('../../../integrations/rrf/rrfCanPlayer.js?worker&url', () => '', { virtual: true });
jest.mock('../../../hp-sim-3d/app/workers/moveCommander.js?worker&url', () => '', { virtual: true });
jest.mock('../../../hp-sim-3d/app/workers/simulationClock.js?worker&url', () => '', { virtual: true });
jest.mock('../../../hp-sim-3d/app/machineCatalog.js', () => ({ buildRepositoryFileUrl: (path) => path }));

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

function setup({ render = { update: jest.fn(), requestRender: jest.fn() }, simulationWorld } = {}) {
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
  const world = simulationWorld ?? {
    systems: [],
    getResource: (key) => resources.get(key),
    setResource: (key, value) => resources.set(key, value),
    update: jest.fn(),
  };
  if (simulationWorld) {
    for (const [key, value] of resources) world.setResource(key, value);
  }
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

test('an unreachable target shows Max and limits command queues to the achieved speed', () => {
  const clock = wallClock();
  const { world, clocks, speed, resources } = setup();
  const queue = new RemoteSpoolSystem();
  queue.worker = { postMessage: jest.fn() };
  // Each 2 ms step costs 1 ms of wall time: at most about 2x is reachable.
  world.update.mockImplementation(() => { clock.now += 1; });
  controls.setTimeScale(1000);
  while (!controls.isAtMaxSpeed()) { clock.now += 16; clocks.at(-1).tick(clock.now); }
  expect(speed.textContent).toMatch(/^1\.\dx \/ Max$/);
  queue.update({ getResource: (key) => resources.get(key), query: () => [] }, 0.002);
  expect(queue.highWaterMark).toBeLessThanOrEqual(320 * 4);

  controls.setTimeScale(1);
  expect(controls.isAtMaxSpeed()).toBe(false);
  expect(speed.textContent).toBe('– / 1x');
});

async function setupWorkerPrint(segments, fileName = 'print.gcode') {
  jest.spyOn(console, 'log').mockImplementation(() => {});
  const clock = wallClock();
  const world = new World();
  const remote = new RemoteSpoolSystem();
  const extruder = new ExtruderComponent();
  world.addComponent(world.createEntity(), extruder);
  world.registerSystem(remote);
  let physicsCommands = 0;
  world.registerSystem({ update: () => { physicsCommands = remote.history.length; } });
  const { clocks } = setup({ simulationWorld: world });
  const state = { currentTimeScale: 1, jobSequenceCounter: 0 };
  const machines = { getMachines: () => [{ id: 'm1' }], getActiveSourceKeys: () => [] };
  const quality = createQualityController({ world, state, ...machines, getMachineDisplayName: () => 'Test machine' });
  const monitor = new QualityMonitor();
  monitor.setMachineContext({ id: 'm1' });
  monitor.setReferenceSegments(segments);
  quality.monitors.set('m1', { monitor });
  quality.attachToRemoteSystem(remote);

  let commands;
  const snapshots = [];
  const runFinalQualityChecks = quality.runFinalQualityChecks;
  quality.runFinalQualityChecks = (job) => {
    runFinalQualityChecks(job);
    snapshots.push({ physicsCommands, ...monitor.getMetrics() });
  };
  const workers = createWorkerController({
    state,
    getRemoteSystem: () => commands.getRemoteSystem(),
    onWorkerDrained: () => commands.finishJob(),
    onWorkerError: () => commands.handleWorkerError(),
    WorkerCtor: class {
      postMessage() {}
      terminate() {}
      receive(data) { this.onmessage({ data }); }
    },
  });
  const finishAsapBtn = {
    addEventListener: (_event, handler) => { finishAsapBtn.click = () => handler({ preventDefault() {} }); },
  };
  commands = createCommandJobController({
    world, state, dom: { finishAsapBtn }, machines, workers, quality,
    runtime: {
      ...controls, isStageReady: () => true, getGameControls: () => controls,
      setTimeScale: (scale) => { controls.setTimeScale(scale); return scale; },
    },
  });
  commands.bindUi();
  await commands.queueCommandFile({ name: fileName });
  const worker = remote.worker;
  const send = (segment) => worker.receive({ action: 'gcode', command: { type: 'Move', E: segment.extrusion } });
  const tick = (ms = 16) => {
    clock.now += ms;
    clocks.at(-1).tick(clock.now);
  };
  const drain = () => {
    for (let i = 0; i < 100 && remote.getQueueLength() > 0; i += 1) tick();
    expect(remote.getQueueLength()).toBe(0);
  };
  return { clock, commands, drain, quality, remote, snapshots, state, worker, send, tick, world, extruder, finishAsapBtn };
}

const firstSegment = { start: [0, 0, 0], end: [0.001, 0, 0], extrusion: 0.01 };

test('linear Quality History includes the tail of a 1000-command worker print', async () => {
  const corners = [[0, 0], [0.25, 0], [0.25, 0.25], [0, 0.25], [0, 0]];
  const segments = Array.from({ length: 1000 }, (_, index) => {
    const side = Math.floor(index / 250);
    const point = (fraction) => [...corners[side].map((start, axis) =>
      start + (corners[side + 1][axis] - start) * fraction), 0];
    return { start: point((index % 250) / 250), end: point((index % 250 + 1) / 250), extrusion: 0.01 };
  });
  const { drain, quality, remote, snapshots, state, worker, send, tick, world, extruder } = await setupWorkerPrint(segments);
  // Supply deterministic tips before command processing, with a 3 mm error on the last side.
  world.systems.unshift({ update: () => {
    const index = remote.history.length;
    const [x, y, z] = segments[Math.min(index, segments.length - 1)].end;
    extruder.machineTips.m1 = { x, y: y + (index >= 750 ? 0.003 : 0), z };
  } });
  segments.slice(0, 680).forEach(send);
  drain();
  segments.slice(680).forEach(send);
  worker.receive({ type: 'done' });
  expect(quality.historyRecords).toHaveLength(0);
  expect(state.printActive).toBe(true);
  drain();
  worker.receive({ type: 'done' });
  tick();
  expect(snapshots).toMatchObject([{ samples: 1000, cornerCount: 3, physicsCommands: 1000 }]);
  expect(quality.historyRecords).toHaveLength(1);
  expect(quality.historyRecords[0].score).toBeCloseTo(94.313247, 6);
  expect(state.printActive).toBe(false);
});

test.each([false, true])('short worker job completes once (queue already drained: %s)', async (alreadyDrained) => {
  const { extruder, quality, snapshots, worker, send, tick } = await setupWorkerPrint([firstSegment], 'print.serial');
  extruder.machineTips.m1 = { x: 0.001, y: 0, z: 0 };
  send(firstSegment);
  if (alreadyDrained) {
    tick();
    controls.pause();
  }
  worker.receive({ type: 'done' });
  expect(quality.historyRecords).toHaveLength(alreadyDrained ? 1 : 0);
  tick();
  expect(snapshots).toMatchObject([{ samples: 1, physicsCommands: 1 }]);
});

test.each(['reset', 'clear queue', 'replacement', 'worker error'])('%s does not record an unfinished job', async (action) => {
  jest.spyOn(console, 'error').mockImplementation(() => {});
  const { commands, remote, snapshots, worker, send, tick } = await setupWorkerPrint([firstSegment]);
  send(firstSegment);
  worker.receive({ type: action === 'worker error' ? 'error' : 'done' });
  if (action === 'reset') {
    commands.handleUserReset();
    controls.resume();
  } else if (action === 'clear queue') {
    remote.clearCommandQueue();
  } else if (action === 'replacement') {
    await commands.queueCommandFile({ name: 'replacement.gcode' });
  }
  worker.receive({ type: 'done' });
  tick();
  expect(snapshots).toHaveLength(0);
});

test('Finish ASAP uses the same post-physics completion', async () => {
  const channels = [];
  global.MessageChannel = class {
    port1 = {};
    port2 = { postMessage() {} };
    constructor() { channels.push(this); }
  };
  const { clock, commands, finishAsapBtn, extruder, quality, snapshots, worker, send, world } =
    await setupWorkerPrint([firstSegment], 'print.can');
  extruder.machineTips.m1 = { x: 0.001, y: 0, z: 0 };
  world.registerSystem({ update: () => { clock.now += 1; } });
  send(firstSegment);
  worker.receive({ type: 'done' });
  expect(quality.historyRecords).toHaveLength(0);
  finishAsapBtn.click();
  expect(snapshots).toMatchObject([{ samples: 1, physicsCommands: 1 }]);
  channels[1].port1.onmessage();
  await Promise.resolve();
  await Promise.resolve();
  expect(commands.isFinishAsapActive()).toBe(false);
  expect(quality.historyRecords).toHaveLength(1);
});
