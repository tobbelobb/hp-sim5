import { runGame } from '../../../examples/js/slideprinter/runner.js';

function createEventTarget(initialText = '') {
  const listeners = new Map();

  return {
    textContent: initialText,
    disabled: false,
    addEventListener(type, callback) {
      const callbacks = listeners.get(type) ?? [];
      callbacks.push(callback);
      listeners.set(type, callbacks);
    },
    emit(type, overrides = {}) {
      const event = {
        preventDefault() {},
        target: this,
        ...overrides,
      };
      for (const callback of listeners.get(type) ?? []) {
        callback(event);
      }
    },
  };
}

function createDocument(elements) {
  return {
    getElementById(id) {
      return elements[id] ?? null;
    },
  };
}

describe('slideprinter runner controls', () => {
  const originalDocument = global.document;
  const originalPerformance = global.performance;
  const originalRequestAnimationFrame = global.requestAnimationFrame;
  const originalCancelAnimationFrame = global.cancelAnimationFrame;

  beforeEach(() => {
    jest.useFakeTimers();
  });

  afterEach(() => {
    jest.useRealTimers();
    global.document = originalDocument;
    global.performance = originalPerformance;
    global.requestAnimationFrame = originalRequestAnimationFrame;
    global.cancelAnimationFrame = originalCancelAnimationFrame;
  });

  test('resume control unpauses immediately and updates the pause button label', () => {
    const elements = {
      pauseBtn: createEventTarget('Start'),
      resetBtn: createEventTarget('Reset'),
      stepBtn: createEventTarget('Step'),
      dumpBtn: createEventTarget('Dump State'),
      dt: createEventTarget('N/A'),
      speed: createEventTarget('N/A'),
    };
    global.document = createDocument(elements);
    global.performance = { now: () => 1000 };
    global.requestAnimationFrame = jest.fn((callback) => {
      global.__runnerCallback = callback;
      return 1;
    });
    global.cancelAnimationFrame = jest.fn();

    const resources = new Map([
      ['dt', 0.01],
      ['pauseState', { paused: false }],
    ]);
    const renderSystem = {
      drawingSuspended: false,
      update: jest.fn(),
    };
    resources.set('renderSystem', renderSystem);

    const world = {
      systems: [],
      getResource(key) {
        return resources.get(key);
      },
      setResource(key, value) {
        resources.set(key, value);
      },
      update: jest.fn(),
    };

    const controls = runGame(world, () => {});
    resources.get('pauseState').paused = true;

    controls.resume();

    expect(resources.get('pauseState').paused).toBe(false);
    expect(elements.pauseBtn.textContent).toBe('Pause');
    expect(global.cancelAnimationFrame).toHaveBeenCalledTimes(1);
    expect(global.requestAnimationFrame).toHaveBeenCalledTimes(2);
    expect(typeof global.__runnerCallback).toBe('function');
  });
});
