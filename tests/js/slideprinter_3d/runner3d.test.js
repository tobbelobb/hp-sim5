import { runGame } from '../../../examples/js/slideprinter_3d/runner.js';

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
    click(overrides = {}) {
      this.emit('click', overrides);
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

describe('slideprinter 3D runner idle loop handling', () => {
  const originalDocument = global.document;
  const originalPerformance = global.performance;

  beforeEach(() => {
    jest.useFakeTimers();
  });

  afterEach(() => {
    jest.useRealTimers();
    global.document = originalDocument;
    global.performance = originalPerformance;
  });

  function setupRunner({ paused = false } = {}) {
    const elements = {
      pauseBtn: createEventTarget('Pause'),
      resetBtn: createEventTarget('Reset'),
      stepBtn: createEventTarget('Step'),
      dumpBtn: createEventTarget('Dump State'),
      dt: createEventTarget('N/A'),
      speed: createEventTarget('N/A'),
    };
    global.document = createDocument(elements);
    global.performance = { now: () => 1000 };

    let animationLoop = null;
    const resources = new Map([
      ['dt', 0.01],
      ['pauseState', { paused }],
    ]);
    const renderSystem = {
      drawingSuspended: false,
      requestRender: jest.fn(),
      update: jest.fn(),
      setAnimationLoop: jest.fn((callback) => {
        animationLoop = callback;
      }),
      clearAnimationLoop: jest.fn(() => {
        animationLoop = null;
      }),
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
    renderSystem.clearAnimationLoop.mockClear();
    renderSystem.requestRender.mockClear();

    return {
      controls,
      elements,
      world,
      resources,
      renderSystem,
      getAnimationLoop: () => animationLoop,
    };
  }

  test('pausing tears down the animation loop until resumed', () => {
    const { elements, renderSystem, resources, getAnimationLoop } = setupRunner();

    expect(renderSystem.setAnimationLoop).toHaveBeenCalledTimes(1);
    expect(typeof getAnimationLoop()).toBe('function');

    elements.pauseBtn.click();

    expect(resources.get('pauseState').paused).toBe(true);
    expect(renderSystem.clearAnimationLoop).toHaveBeenCalledTimes(1);
    expect(getAnimationLoop()).toBeNull();
    expect(renderSystem.requestRender).toHaveBeenCalledTimes(1);

    elements.pauseBtn.click();

    expect(resources.get('pauseState').paused).toBe(false);
    expect(renderSystem.setAnimationLoop).toHaveBeenCalledTimes(2);
    expect(typeof getAnimationLoop()).toBe('function');
  });

  test('paused single-step renders one frame and returns to idle', () => {
    const { elements, world, resources, renderSystem, getAnimationLoop } = setupRunner();

    elements.pauseBtn.click();
    renderSystem.clearAnimationLoop.mockClear();
    renderSystem.requestRender.mockClear();

    elements.stepBtn.click();
    const stepLoop = getAnimationLoop();

    expect(typeof stepLoop).toBe('function');

    stepLoop(1030);

    expect(world.update).toHaveBeenCalledTimes(1);
    expect(resources.get('pauseState').paused).toBe(true);
    expect(renderSystem.clearAnimationLoop).toHaveBeenCalledTimes(2);
    expect(getAnimationLoop()).toBeNull();
    expect(renderSystem.requestRender).toHaveBeenCalledTimes(1);
  });

  test('external resume from a paused state wakes the loop on the idle poll', () => {
    const { resources, renderSystem, getAnimationLoop } = setupRunner();

    resources.get('pauseState').paused = true;
    renderSystem.clearAnimationLoop.mockClear();
    renderSystem.requestRender.mockClear();

    renderSystem.setAnimationLoop.mock.calls[0][0](1016);

    expect(renderSystem.clearAnimationLoop).toHaveBeenCalledTimes(1);
    expect(getAnimationLoop()).toBeNull();
    expect(renderSystem.requestRender).toHaveBeenCalledTimes(1);

    resources.get('pauseState').paused = false;
    jest.advanceTimersByTime(100);

    expect(renderSystem.setAnimationLoop).toHaveBeenCalledTimes(2);
    expect(typeof getAnimationLoop()).toBe('function');
  });

  test('resume control unpauses immediately and updates the pause button label', () => {
    const { controls, elements, resources, renderSystem, getAnimationLoop } = setupRunner();

    renderSystem.clearAnimationLoop.mockClear();
    renderSystem.requestRender.mockClear();
    resources.get('pauseState').paused = true;
    elements.pauseBtn.textContent = 'Start';

    controls.resume();

    expect(resources.get('pauseState').paused).toBe(false);
    expect(elements.pauseBtn.textContent).toBe('Pause');
    expect(renderSystem.clearAnimationLoop).toHaveBeenCalledTimes(1);
    expect(renderSystem.setAnimationLoop).toHaveBeenCalledTimes(2);
    expect(typeof getAnimationLoop()).toBe('function');
  });
});
