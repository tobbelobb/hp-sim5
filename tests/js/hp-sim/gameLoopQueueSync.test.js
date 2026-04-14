import { World, MachineTagComponent } from '../../../src/js/cable_joints/ecs.js';
import { PauseStateComponent } from '../../../example_apps/js/flipper/flipper_common.js';
import {
  RemoteSpoolSystem,
  SpoolTagComponent,
  SpoolStateComponent,
  StepperMotorComponent,
} from '../../../example_apps/js/slideprinter/slideprinter_common.js';
import { runGame } from '../../../example_apps/js/slideprinter/runner.js';

class TestWorld extends World {
  constructor() {
    super();
    this.updateCount = 0;
  }

  update(dt) {
    this.updateCount += 1;
    super.update(dt);
  }
}

describe('game loop remote queue synchronisation', () => {
  let world;
  let remoteSystem;
  let controls;
  let fakeNow;
  let performanceSpy;
  let rafCallbacks;
  let originalRaf;
  let originalCancelRaf;
  let originalDocument;

  const advanceFrame = () => {
    let next;
    do {
      next = rafCallbacks.shift();
    } while (next == null && rafCallbacks.length > 0);
    expect(typeof next).toBe('function');
    fakeNow += 2;
    next(fakeNow);
  };

  beforeEach(() => {
    originalDocument = global.document;
    const elements = new Map();
    const createElement = () => ({
      textContent: 'N/A',
      disabled: false,
      setAttribute: jest.fn(),
      removeAttribute: jest.fn(),
      classList: {
        add: jest.fn(),
        remove: jest.fn(),
        toggle: jest.fn(),
      },
      addEventListener: jest.fn(),
      focus: jest.fn(),
    });
    global.document = {
      getElementById: (id) => {
        if (!elements.has(id)) {
          elements.set(id, createElement());
        }
        return elements.get(id);
      },
    };

    fakeNow = 0;
    performanceSpy = jest.spyOn(performance, 'now').mockImplementation(() => fakeNow);

    rafCallbacks = [];
    originalRaf = global.requestAnimationFrame;
    originalCancelRaf = global.cancelAnimationFrame;
    global.requestAnimationFrame = (cb) => {
      rafCallbacks.push(cb);
      return rafCallbacks.length;
    };
    global.cancelAnimationFrame = jest.fn((id) => {
      const index = typeof id === 'number' ? id - 1 : -1;
      if (index >= 0 && index < rafCallbacks.length) {
        rafCallbacks[index] = null;
      }
    });

    world = new TestWorld();
    world.setResource('dt', 1 / 500);
    world.setResource('pauseState', new PauseStateComponent(false));

    remoteSystem = new RemoteSpoolSystem();
    world.registerSystem(remoteSystem);

    const spoolId = world.createEntity();
    world.addComponent(spoolId, new SpoolTagComponent());
    world.addComponent(spoolId, new SpoolStateComponent('A'));
    world.addComponent(spoolId, new MachineTagComponent('machine-1'));
    world.addComponent(spoolId, new StepperMotorComponent());

    controls = runGame(world, () => {}, { initialTimeScale: 1 });
    remoteSystem.worker = { postMessage: jest.fn() };
  });

  afterEach(() => {
    performanceSpy.mockRestore();
    rafCallbacks = [];
    global.requestAnimationFrame = originalRaf;
    global.cancelAnimationFrame = originalCancelRaf;
    global.document = originalDocument;
  });

  test('linear playback keeps updating even when a live worker queue is empty', () => {
    controls.reset({ autoPause: false });
    expect(rafCallbacks.length).toBeGreaterThan(0);

    advanceFrame();
    advanceFrame();
    advanceFrame();

    expect(world.updateCount).toBeGreaterThan(0);
    const updatesBeforeMove = world.updateCount;

    remoteSystem.addCommand({ type: 'Move', A: 0.1 });

    advanceFrame();

    expect(world.updateCount).toBeGreaterThan(updatesBeforeMove);
    const stepperComp = world.getComponentStore(StepperMotorComponent).values().next().value;
    expect(stepperComp.commandedAngle).toBeCloseTo(0.1, 12);
  });

  test('assigning a worker replays the current ASAP mode to the worker', () => {
    const worker = { postMessage: jest.fn() };

    remoteSystem.asapModeActive = true;
    remoteSystem.worker = worker;
    expect(worker.postMessage).toHaveBeenCalledWith({ type: 'set_asap_mode', enable: true });

    worker.postMessage.mockClear();

    remoteSystem.worker = null;
    remoteSystem.asapModeActive = false;
    remoteSystem.worker = worker;

    expect(worker.postMessage).toHaveBeenCalledWith({ type: 'set_asap_mode', enable: false });
  });
});
