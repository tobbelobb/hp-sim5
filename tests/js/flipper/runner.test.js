const { runGame } = require('../../../examples/js/flipper/runner.js');
const { RenderSystem } = require('../../../examples/js/flipper/renderSystem.js');

function createEventTarget(initialText = '') {
  const listeners = new Map();

  return {
    textContent: initialText,
    addEventListener(type, callback) {
      const callbacks = listeners.get(type) ?? [];
      callbacks.push(callback);
      listeners.set(type, callbacks);
    },
    emit(type, overrides = {}) {
      const event = {
        preventDefault() {},
        target: this,
        ...overrides
      };
      for (const callback of listeners.get(type) ?? []) {
        callback(event);
      }
    }
  };
}

function createDocument(elements) {
  const listeners = new Map();

  return {
    getElementById(id) {
      return elements[id];
    },
    addEventListener(type, callback) {
      const callbacks = listeners.get(type) ?? [];
      callbacks.push(callback);
      listeners.set(type, callbacks);
    },
    emit(type, overrides = {}) {
      const event = {
        preventDefault() {},
        target: {},
        ...overrides
      };
      for (const callback of listeners.get(type) ?? []) {
        callback(event);
      }
    }
  };
}

describe('runGame manual loop mode', () => {
  const originalDocument = global.document;
  const originalWindow = global.window;
  const originalPerformance = global.performance;
  const originalRequestAnimationFrame = global.requestAnimationFrame;

  afterEach(() => {
    global.document = originalDocument;
    global.window = originalWindow;
    global.performance = originalPerformance;
    global.requestAnimationFrame = originalRequestAnimationFrame;
  });

  test('disables the auto RAF loop and still supports paused single-step', () => {
    const elements = {
      pauseBtn: createEventTarget('Pause'),
      resetBtn: createEventTarget('Reset'),
      stepBtn: createEventTarget('Step'),
      dumpBtn: createEventTarget('Dump State'),
      dt: createEventTarget('N/A'),
      speed: createEventTarget('N/A')
    };
    global.document = createDocument(elements);
    global.window = { _flipperDisableAutoLoop: true };
    global.performance = { now: () => 1000 };
    global.requestAnimationFrame = jest.fn();

    const renderSystem = Object.create(RenderSystem.prototype);
    renderSystem.update = jest.fn();

    const resources = new Map([
      ['dt', 0.01],
      ['pauseState', { paused: true }]
    ]);
    const world = {
      systems: [renderSystem],
      getResource(key) {
        return resources.get(key);
      },
      update: jest.fn()
    };

    runGame(world, () => {}, null);

    expect(global.requestAnimationFrame).not.toHaveBeenCalled();
    expect(renderSystem.update).toHaveBeenCalledTimes(1);

    elements.stepBtn.emit('click');

    expect(world.update).toHaveBeenCalledTimes(1);
    expect(world.getResource('pauseState').paused).toBe(true);
    expect(global.requestAnimationFrame).not.toHaveBeenCalled();
  });
});
