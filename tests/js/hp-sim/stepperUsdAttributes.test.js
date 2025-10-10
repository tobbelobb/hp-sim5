import { readFileSync } from 'fs';
import path from 'path';
import { World } from '../../../src/js/cable_joints/ecs.js';
import { setupScene } from '../../../examples/js/slideprinter/setupScene.js';
import {
  StepperMotorComponent,
  SpoolTagComponent,
} from '../../../examples/js/slideprinter/slideprinter_common.js';
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

describe('Stepper attributes from USD', () => {
  let stage;
  let originalDocument;
  let originalWindow;
  let originalNavigator;

  beforeAll(async () => {
    const usdSource = readFileSync(usdPath, 'utf8');
    stage = await UsdOpen(usdSource);
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
});
