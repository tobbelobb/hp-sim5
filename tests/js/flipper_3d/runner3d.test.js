jest.mock('../../../src/js/cable_joints_3d/render_system_3d.js', () => {
  class MockRenderSystem3D {}
  return { RenderSystem3D: MockRenderSystem3D };
});

const { runGame } = require('../../../examples/js/flipper_3d/runner.js');
const { RenderSystem3D } = require('../../../src/js/cable_joints_3d/render_system_3d.js');

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

describe('runGame 3D paused stepping', () => {
  const originalDocument = global.document;
  const originalWindow = global.window;
  const originalPerformance = global.performance;

  afterEach(() => {
    global.document = originalDocument;
    global.window = originalWindow;
    global.performance = originalPerformance;
  });

  test('holding step hotkey keeps advancing while paused until release', () => {
    const elements = {
      pauseBtn: createEventTarget('Pause'),
      resetBtn: createEventTarget('Reset'),
      stepBtn: createEventTarget('Step'),
      dumpBtn: createEventTarget('Dump State'),
      dt: createEventTarget('N/A'),
      speed: createEventTarget('N/A')
    };
    const documentMock = createDocument(elements);
    global.document = documentMock;
    global.window = { _flipper3dDebug: false };
    global.performance = { now: () => 1000 };

    let animationLoop = null;
    const renderSystem = Object.create(RenderSystem3D.prototype);
    renderSystem.update = jest.fn();
    renderSystem.setAnimationLoop = (callback) => {
      animationLoop = callback;
    };

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

    expect(typeof animationLoop).toBe('function');

    animationLoop(1);
    animationLoop(2);
    expect(world.update).not.toHaveBeenCalled();

    documentMock.emit('keydown', { key: 's', target: {} });
    animationLoop(30);

    expect(world.update.mock.calls.length).toBeGreaterThan(1);
    expect(world.getResource('pauseState').paused).toBe(true);

    const callsWhileHeld = world.update.mock.calls.length;
    documentMock.emit('keyup', { key: 's', target: {} });
    animationLoop(60);

    expect(world.update.mock.calls.length).toBe(callsWhileHeld);
  });
});
