import { readFileSync } from 'fs';
import path from 'path';
import { World } from '../../../src/js/cable_joints/ecs.js';
import { CablePathComponent } from '../../../src/js/cable_joints/cable_joints_core.js';
import { setupScene } from '../../../example_apps/js/slideprinter/setupScene.js';
import {
  StepperMotorComponent,
  SpoolTagComponent,
  ExtruderComponent,
} from '../../../example_apps/js/slideprinter/slideprinter_common.js';
import { OpenText as UsdOpenText } from '../../../src/js/usd/stage.js';
import { bakeCableSceneUsdaSource } from '../../../src/js/usd/cable_scene_baker.js';

const usdPath = path.resolve(process.cwd(), 'public/usd_scenes/slideprinter.usda');

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

describe('Stepper attributes from USD', () => {
  let stage;
  let originalDocument;
  let originalWindow;
  let originalNavigator;

  beforeAll(async () => {
    const usdSource = readFileSync(usdPath, 'utf8');
    stage = UsdOpenText(bakeCableSceneUsdaSource(usdSource).source);
  });

  beforeEach(() => {
    originalDocument = global.document;
    originalWindow = global.window;
    originalNavigator = global.navigator;

    const elements = new Map();
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

  afterEach(() => {
    global.document = originalDocument;
    global.window = originalWindow;
    global.navigator = originalNavigator;
  });

  test('setupScene applies stepper attributes from USD', () => {
    const world = new World();
    const canvas = createCanvasStub();

    setupScene(world, stage, canvas, { append: false });

    const spoolEntities = world.query([SpoolTagComponent, StepperMotorComponent]);
    expect(spoolEntities.length).toBeGreaterThan(0);

    for (const entity of spoolEntities) {
      const stepper = world.getComponent(entity, StepperMotorComponent);
      expect(stepper.holdingTorque).toBeCloseTo(0.5);
      expect(stepper.numPolePairs).toBe(50);
      expect(stepper.dampingCoeff).toBeCloseTo(0.01);
    }
  });

  test('setupScene applies namespaced cable path stiffness from USD', () => {
    const world = new World();
    const canvas = createCanvasStub();

    setupScene(world, stage, canvas, { append: false });

    const cablePathEntities = world.query([CablePathComponent]);
    expect(cablePathEntities.length).toBeGreaterThan(0);

    for (const entity of cablePathEntities) {
      const pathComp = world.getComponent(entity, CablePathComponent);
      expect(pathComp.spring_constant).toBeCloseTo(20000.0);
    }
  });

  test('setupScene resolves extruder center sources from the authored Extruder prim', () => {
    const world = new World();
    const canvas = createCanvasStub();

    setupScene(world, stage, canvas, { append: false });

    const extruderEntity = world.query([ExtruderComponent])[0];
    expect(extruderEntity).toBeDefined();

    const extruder = world.getComponent(extruderEntity, ExtruderComponent);
    const extruderSystem = world.systems.find((system) => system.constructor.name === 'ExtruderSystem');
    expect(extruderSystem).toBeDefined();

    extruderSystem.update(world, 0);

    expect(extruder.centerSources.default).toHaveLength(3);
    expect(extruder.centerOffsets.default.x).toBeCloseTo(0.0, 9);
    expect(extruder.centerOffsets.default.y).toBeCloseTo(0.0, 9);
    expect(extruder.centerPos.x).toBeCloseTo(0.0, 9);
    expect(extruder.centerPos.y).toBeCloseTo(0.0, 9);
  });
});
