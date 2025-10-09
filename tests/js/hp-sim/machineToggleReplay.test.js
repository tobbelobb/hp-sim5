import { readFileSync } from 'fs';
import path from 'path';
import { World, MachineTagComponent } from '../../../src/js/cable_joints/ecs.js';
import { setupScene } from '../../../examples/js/slideprinter/setupScene.js';
import { RemoteSpoolSystem, ExtruderComponent } from '../../../examples/js/slideprinter/slideprinter_common.js';
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

async function restorePrintState(world, remoteSystem, historyClone, queueClone, dt) {
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
  await Promise.resolve();
  if (historyCopy.length > 0) {
    await runReplayLoop(world, remoteSystem, historyCopy.length, dt);
  }
  remoteSystem.history = historyCopy;
  remoteSystem.commands = queueCopy;
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

    remoteSystem.addCommand({ type: 'Move', A: 0.12, B: -0.08, C: 0.05, D: -0.03, E: 0.004 });
    const dt = world.getResource('dt') || 1 / 120;
    world.update(dt);
    expect(remoteSystem.history.length).toBeGreaterThan(0);

    const extruderEntityInitial = world.query([ExtruderComponent])[0];
    const extruderInitial = world.getComponent(extruderEntityInitial, ExtruderComponent);
    const originalExtrusions = extruderInitial.extrusions.map((entry) => ({
      machineId: entry.machineId,
      pos: [...entry.pos],
      length: entry.length,
    }));

    const playbackState = remoteSystem.getPlaybackState();

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
      enqueue(() => restorePrintState(world, remoteSystem, historyClone, queueClone, dt)),
      enqueue(() => restorePrintState(world, remoteSystem, historyClone, queueClone, dt)),
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
  });
});
