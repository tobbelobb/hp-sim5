import { readFileSync } from 'fs';
import path from 'path';
import { World, MachineTagComponent } from '../../../src/js/cable_joints/ecs.js';
import { setupScene } from '../../../examples/js/slideprinter/setupScene.js';
import { RemoteSpoolSystem, ExtruderComponent } from '../../../examples/js/slideprinter/slideprinter_common.js';
import { QualityMonitor } from '../../../hp-sim/app/quality-monitor.js';
import { Open as UsdOpen } from '../../../src/js/usd/stage.js';

const usdPath = path.resolve(process.cwd(), 'examples/usd_scenes/slideprinter.usda');

const createContextStub = () => {
  const noop = () => {};
  return {
    save: noop,
    restore: noop,
    clearRect: noop,
    fillRect: noop,
    beginPath: noop,
    moveTo: noop,
    lineTo: noop,
    stroke: noop,
    arc: noop,
    closePath: noop,
    fill: noop,
    drawImage: noop,
    setTransform: noop,
    scale: noop,
    translate: noop,
    createLinearGradient: () => ({ addColorStop: noop }),
    createRadialGradient: () => ({ addColorStop: noop }),
    measureText: () => ({ width: 0 }),
    putImageData: noop,
    getImageData: () => ({ data: [] }),
    rect: noop,
  };
};

const createCanvasStub = () => ({
  width: 800,
  height: 600,
  clientWidth: 800,
  clientHeight: 600,
  style: {},
  getContext: () => createContextStub(),
  addEventListener: jest.fn(),
  removeEventListener: jest.fn(),
  getBoundingClientRect: () => ({ left: 0, top: 0, width: 800, height: 600, right: 800, bottom: 600 }),
  setPointerCapture: jest.fn(),
  releasePointerCapture: jest.fn(),
  setAttribute: jest.fn(),
  removeAttribute: jest.fn(),
  focus: jest.fn(),
});

const createControlStub = () => ({
  textContent: '',
  disabled: false,
  setAttribute: jest.fn(),
  removeAttribute: jest.fn(),
  addEventListener: jest.fn(),
  removeEventListener: jest.fn(),
  focus: jest.fn(),
  classList: {
    add: jest.fn(),
    remove: jest.fn(),
    toggle: jest.fn(),
    contains: jest.fn(() => false),
  },
  dataset: {},
  style: {},
  closest: jest.fn(() => null),
});

let stage;
let elements;

beforeAll(async () => {
  const usdSource = readFileSync(usdPath, 'utf8');
  stage = await UsdOpen(usdSource);
});

const cloneCommandList = (list) => (Array.isArray(list) ? list.map((cmd) => ({ ...cmd })) : []);

function buildOffsetReferenceSegments(extrusions, offset = 0.02) {
  if (!Array.isArray(extrusions) || extrusions.length < 2) {
    return [
      { start: [0, offset, 0], end: [1, offset, 0] },
    ];
  }
  const segments = [];
  for (let i = 0; i < extrusions.length - 1; i += 1) {
    const start = extrusions[i]?.pos || [0, 0, 0];
    const end = extrusions[i + 1]?.pos || start;
    segments.push({
      start: [Number(start[0]) || 0, (Number(start[1]) || 0) + offset, 0],
      end: [Number(end[0]) || 0, (Number(end[1]) || 0) + offset, 0],
    });
  }
  return segments;
}

async function runReplayLoop(world, remoteSystem, targetCount, dt) {
  if (!Number.isFinite(targetCount) || targetCount <= 0) {
    return;
  }
  const maxIterations = Math.max(targetCount * 4, targetCount + 200);
  let iterations = 0;
  while (remoteSystem.history.length < targetCount && iterations < maxIterations) {
    world.update(dt);
    iterations += 1;
    if (iterations % 500 === 0) {
      await Promise.resolve();
    }
  }
}

async function restorePrintState(world, remoteSystem, historyClone, queueClone, dt, options = {}) {
  const { qualityMonitors = [] } = options;
  const historyCopy = cloneCommandList(historyClone);
  const queueCopy = cloneCommandList(queueClone);
  const extruderEntity = world.query([ExtruderComponent])[0];
  if (extruderEntity !== undefined) {
    const extruder = world.getComponent(extruderEntity, ExtruderComponent);
    if (extruder) {
      extruder.extrusions = [];
    }
  }
  remoteSystem.worker = null;
  remoteSystem.wasPaused = false;
  remoteSystem.history = [];
  remoteSystem.commands = cloneCommandList(historyCopy).concat(cloneCommandList(queueCopy));
  remoteSystem.resetAxisMapping();
  for (const monitor of qualityMonitors) {
    if (monitor && typeof monitor.reset === 'function') {
      monitor.reset({ keepReference: true });
    }
  }
  await Promise.resolve();
  if (historyCopy.length > 0) {
    await runReplayLoop(world, remoteSystem, historyCopy.length, dt);
  }
  remoteSystem.history = historyCopy;
  remoteSystem.commands = queueCopy;
  for (const monitor of qualityMonitors) {
    if (monitor && typeof monitor.runFinalCheck === 'function') {
      monitor.runFinalCheck();
    }
  }
}

describe('machine toggles with replay', () => {
  beforeEach(() => {
    elements = new Map();
    const ensureElement = (id) => {
      if (!elements.has(id)) {
        const el = createControlStub();
        el.id = id;
        elements.set(id, el);
      }
      return elements.get(id);
    };
    global.document = {
      getElementById: (id) => ensureElement(id),
      createElement: (tag) => {
        if (tag === 'canvas') {
          return createCanvasStub();
        }
        const el = createControlStub();
        el.tagName = tag.toUpperCase();
        return el;
      },
      addEventListener: jest.fn(),
      removeEventListener: jest.fn(),
      body: {
        appendChild: jest.fn(),
      },
    };
    ['pauseBtn', 'resetBtn', 'stepBtn', 'dumpBtn', 'dt', 'speed'].forEach((id) => ensureElement(id));
    global.window = {
      setTimeout,
      clearTimeout,
      addEventListener: jest.fn(),
      removeEventListener: jest.fn(),
    };
    global.navigator = {
      scheduling: null,
    };
  });

  test('replay remains stable when concurrent scene restores run', async () => {
    const world = new World();
    const canvas = createCanvasStub();

    setupScene(world, stage, canvas, { namespace: 'machine-0', append: false });
    setupScene(world, stage, canvas, { namespace: 'machine-1', append: true });
    setupScene(world, stage, canvas, { namespace: 'machine-2', append: true });

    const remoteSystem = world.systems.find((system) => system instanceof RemoteSpoolSystem);
    expect(remoteSystem).toBeDefined();

    const dt = world.getResource('dt') || 1 / 120;
    const initialCommands = [
      { type: 'Move', A: 0.12, B: -0.08, C: 0.05, D: -0.03, E: 0.004 },
      { type: 'Move', A: 0.16, B: -0.09, C: 0.06, D: -0.035, E: 0.005 },
      { type: 'Move', A: 0.2, B: -0.11, C: 0.08, D: -0.04, E: 0.006 },
    ];
    for (const command of initialCommands) {
      remoteSystem.addCommand(command);
      world.update(dt);
    }
    expect(remoteSystem.history.length).toBeGreaterThan(0);

    const extruderEntityInitial = world.query([ExtruderComponent])[0];
    const extruderInitial = world.getComponent(extruderEntityInitial, ExtruderComponent);
    const originalExtrusions = extruderInitial.extrusions.map((entry) => ({
      machineId: entry.machineId,
      pos: [...entry.pos],
      length: entry.length,
    }));

    const playbackState = remoteSystem.getPlaybackState();

    const machineExtrusions = originalExtrusions.filter((entry) => typeof entry.machineId === 'string');
    expect(machineExtrusions.length).toBeGreaterThan(0);
    const survivingMachineIds = new Set(['machine-1', 'machine-2']);
    const preferred = machineExtrusions.find((entry) => survivingMachineIds.has(entry.machineId));
    const targetMachineId = preferred ? preferred.machineId : machineExtrusions[0].machineId;
    const filteredExtrusions = machineExtrusions.filter((entry) => entry.machineId === targetMachineId);
    expect(filteredExtrusions.length).toBeGreaterThan(0);
    const referenceSegments = buildOffsetReferenceSegments(filteredExtrusions);
    const qualityMonitor = new QualityMonitor();
    qualityMonitor.setMachineContext({ id: targetMachineId, label: targetMachineId });
    qualityMonitor.setReferenceSegments(referenceSegments);
    qualityMonitor.setEnabled(true);
    filteredExtrusions.forEach((extrusion) => qualityMonitor.recordExtrusion(extrusion));
    qualityMonitor.runFinalCheck();
    const baselineMetrics = qualityMonitor.metrics;
    qualityMonitor.attachRemoteSystem(remoteSystem);

    world.clear();
    setupScene(world, stage, canvas, { namespace: 'machine-1', append: false });
    setupScene(world, stage, canvas, { namespace: 'machine-2', append: true });

    const historyClone = cloneCommandList(playbackState.history);
    const queueClone = cloneCommandList(playbackState.queue);

    let sceneQueue = Promise.resolve();
    const enqueue = (task) => {
      const run = sceneQueue.then(() => task());
      sceneQueue = run.catch(() => {});
      return run;
    };

    await Promise.all([
      enqueue(() => restorePrintState(world, remoteSystem, historyClone, queueClone, dt, { qualityMonitors: [qualityMonitor] })),
      enqueue(() => restorePrintState(world, remoteSystem, historyClone, queueClone, dt, { qualityMonitors: [qualityMonitor] })),
    ]);

    const expectedMachines = new Set(['machine-1', 'machine-2']);
    for (const entities of Object.values(remoteSystem.axisToEntity)) {
      expect(Array.isArray(entities)).toBe(true);
      const seen = new Set();
      for (const entityId of entities) {
        const machineTag = world.getComponent(entityId, MachineTagComponent);
        expect(machineTag).toBeDefined();
        seen.add(machineTag.id);
      }
      expect(seen).toEqual(expectedMachines);
    }

    const extruderEntityAfter = world.query([ExtruderComponent])[0];
    const extruderAfter = world.getComponent(extruderEntityAfter, ExtruderComponent);
    const expectedExtrusions = originalExtrusions.filter(
      (entry) => entry.machineId === 'machine-1' || entry.machineId === 'machine-2'
    );
    expect(expectedExtrusions.length).toBeGreaterThan(0);
    expect(extruderAfter.extrusions).toHaveLength(expectedExtrusions.length);
    extruderAfter.extrusions.forEach((entry, index) => {
      const expected = expectedExtrusions[index];
      expect(expected).toBeDefined();
      entry.pos.forEach((value, coord) => {
        expect(value).toBeCloseTo(expected.pos[coord], 3);
      });
      expect(entry.length).toBeCloseTo(expected.length, 6);
      expect(entry.machineId).toBe(expected.machineId);
    });

    qualityMonitor.runFinalCheck();
    const replayMetrics = qualityMonitor.metrics;
    if (baselineMetrics && replayMetrics) {
      const keysToCheck = ['rmseStraight', 'p95Straight', 'coverage', 'iou', 'score'];
      for (const key of keysToCheck) {
        const before = baselineMetrics[key];
        const after = replayMetrics[key];
        if (Number.isFinite(before) && Number.isFinite(after)) {
          expect(after).toBeCloseTo(before, 6);
        } else {
          expect(after).toBe(before);
        }
      }
    }
  });
});
